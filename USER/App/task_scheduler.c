/**
 * @file    task_scheduler.c
 * @brief   Five-task non-blocking cooperative scheduler.
 *
 * All timing derives from the 1 ms SysTick counter; each task keeps a
 * last-run timestamp and Scheduler_Run() invokes due task bodies inline
 * (no context switch).
 */

#include "task_scheduler.h"
#include "system_config.h"

/* System */
#include "SysTick.h"
#include "usart.h"

/* Hardware drivers */
#include "i2c.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Servo.h"
#include "Encoder.h"
#include "adc.h"

/* Algorithm modules */
#include "Filter.h"
#include "PID.h"

/* ================================================================
 *  TASK STATE  (persistent across Scheduler_Run calls)
 * ================================================================ */

/* ------ sensor data ------ */
static MPU6050_RawData     mpu_raw;
static MPU6050_PhyData     mpu_phy;
static AttitudeAngle       attitude;

/* ------ filter instances (one per axis) ------ */
static ComplementaryFilter filter_pitch;
static ComplementaryFilter filter_roll;

/* ------ PID ------ */
static PID_State           pid_state;
static PID_Params          pid_params;
static float               pid_output;
static float               servo_angle_current = 90.0f;  /* last servo angle for display */

/* ------ digital filters (noise suppression for VOFA+ channels) ------ */
static MovAvg_t  ma_accel_pitch;      /* VOFA ch2: raw accel pitch smoothing */
static MovAvg_t  ma_roll_display;     /* VOFA ch1: roll display smoothing */
static MovAvg_t  ma_adc;              /* VOFA ch6: ADC voltage smoothing */
static float     prev_roll_vofa = 0.0f;  /* VOFA ch1: limit-filter state */

/* ------ encoder / setpoint ------ */
static float     setpoint_angle = 0.0f;
static int16_t   encoder_delta;
static int32_t   encoder_accum;
static uint16_t  adc_mv = 0;           /* ADC voltage (mV) */

/* ------ tune mode ------ */
#define MODE_RUN      0
#define MODE_TUNE_KP  1
#define MODE_TUNE_KI  2
#define MODE_TUNE_KD  3
#define MODE_MAX      4

static uint8_t   tune_mode = MODE_RUN;

/* ------ OLED framebuffer paging (non-blocking refresh) ------ */
static const uint8_t oled_flush_pages[4] = { 0, 2, 4, 6 };
static uint8_t       oled_flush_idx = 0;

/* ------ task bookkeeping ------ */
static uint32_t  last_10ms   = 0;
static uint32_t  last_20ms   = 0;
static uint32_t  last_50ms   = 0;
static uint32_t  last_200ms  = 0;
static uint32_t  last_1000ms = 0;
static uint8_t   heartbeat   = 0;

/* ================================================================
 *  INITIALIZATION HELPERS
 * ================================================================ */

/**
 * @brief  Configure IWDG registers (counter started later, see IWDG_Start).
 * @note   LSI ~40 kHz / 64 prescaler = 625 Hz; reload 1250 -> 2 s.
 *         After start, the counter is fed on every main-loop pass.
 */
static void IWDG_Init(void)
{
    /* After an IWDG reset the counter is still running in hardware
     * (warm boot). Reload immediately to prevent a second reset
     * while the registers are being reconfigured. */
    IWDG_ReloadCounter();

    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_PRESCALER);
    IWDG_SetReload(IWDG_RELOAD);
    IWDG_ReloadCounter();
    /* IWDG is not enabled here (see IWDG_Start()). On cold boot the
     * counter starts only when enabled; on warm boot (after an IWDG
     * reset) it is already running and the reloads above keep the
     * counter alive through init. */
}

/**
 * @brief  Start the IWDG counter (call once, right before the main loop).
 * @note   Once started, the IWDG cannot be stopped except by reset;
 *         it stays armed even across a software reset.
 */
static void IWDG_Start(void)
{
    /* Standard sequence: unlock write access, load prescaler and
     * reload, feed, then enable. */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_PRESCALER);
    IWDG_SetReload(IWDG_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();

    /* DBGMCU_IWDG_STOP freezes the watchdog while the core is halted,
     * so a debugger breakpoint cannot trigger an IWDG reset. */
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);

    UART_SendStr("[OK]   IWDG enabled, timeout ~2s\r\n");
}

/* ================================================================
 *  PUBLIC INTERFACE
 * ================================================================ */

/**
 * @brief  One-time init of every subsystem, in dependency order.
 */
void Scheduler_Init(void)
{
    /* --- tick source (must come first) --- */
    SysTick_Init();

    /* --- debug serial --- */
    UART1_Configuration();
    UART1_DMA_Init();

    UART_SendStr("\r\n\r\n");
    UART_SendStr("========================================\r\n");
    UART_SendStr("  STM32 Attitude & Servo Control System\r\n");
    UART_SendStr("  MCU: STM32F103C8T6 @ 72MHz\r\n");
    UART_SendStr("========================================\r\n");

    CheckAndReportResetSource();

    /* --- watchdog --- */
    UART_SendStr("[INIT] IWDG watchdog...\r\n");
    IWDG_Init();

    /* --- I2C bus (must be up before OLED / MPU6050) --- */
    UART_SendStr("[INIT] I2C Bus...\r\n");
    SoftI2C_Init();

    /* --- OLED --- */
    UART_SendStr("[INIT] OLED...\r\n");
    OLED_Init();                    /* contains 300ms power-on delay */
    IWDG_ReloadCounter();           /* feed: OLED init consumed ~350ms */
    OLED_Clear();                   /* framebuffer wipe (RAM-only) */
    OLED_ShowString(0, 0, "System Boot...");

    /* --- MPU6050 --- */
    UART_SendStr("[INIT] MPU6050...\r\n");
    MPU6050_Init();
    {
        uint8_t whoami = MPU6050_ReadID();
        if (whoami != 0x00 && whoami != 0xFF)
        {
            /* Clone chips (0x70, 0x98, ...) share the register map;
             * only 0x00 (floating bus) and 0xFF (bus stuck high)
             * are rejected. */
            UART_SendStr("[OK]   MPU6050 WHO_AM_I = 0x");
            UART_SendHex(whoami);
            UART_SendStr(" (clone, functional)\r\n");
            OLED_ShowString(0, 2, "MPU6050: OK");
        }
        else
        {
            UART_SendStr("[ERR]  MPU6050 WHO_AM_I = 0x");
            UART_SendHex(whoami);
            UART_SendStr(" (bus error: check wiring/pull-ups)\r\n");
            OLED_ShowString(0, 2, "MPU6050: FAIL!");
        }
    }

    /* --- servo --- */
    UART_SendStr("[INIT] Servo PWM (PA0)...\r\n");
    Servo_Init();
    Servo_SetAngle(90);     /* park at center */
    UART_SendStr("[OK]   Servo at 90 deg\r\n");

    /* --- encoder --- */
    UART_SendStr("[INIT] Encoder (PB12/PB13, SW=PB14)...\r\n");
    Encoder_Init();
    UART_SendStr("[OK]   Encoder ready\r\n");

    /* --- ADC --- */
    UART_SendStr("[INIT] ADC1_CH1 (PA1)...\r\n");
    ADC_GPIO_Config();
    ADC1_Mode_Config();
    UART_SendStr("[OK]   ADC ready\r\n");

    /* --- complementary filter (alpha=0.98, dt=10ms) --- */
    UART_SendStr("[INIT] Complementary Filter (alpha=0.98, dt=10ms)...\r\n");
    Filter_Init(&filter_pitch, FILTER_DEFAULT_ALPHA, FILTER_DT_S);
    Filter_Init(&filter_roll,  FILTER_DEFAULT_ALPHA, FILTER_DT_S);

    /* --- digital noise filters (VOFA ch2/ch6 cleanup) --- */
    UART_SendStr("[INIT] Digital Filters (MA + Median)...\r\n");
    MovAvg_Init(&ma_accel_pitch);
    MovAvg_Init(&ma_roll_display);
    MovAvg_Init(&ma_adc);

    /* --- PID --- */
    UART_SendStr("[INIT] PID (Kp=2.0, Ki=0.05, Kd=0.5)...\r\n");
    pid_params.Kp = PID_DEFAULT_KP;
    pid_params.Ki = PID_DEFAULT_KI;
    pid_params.Kd = PID_DEFAULT_KD;
    PID_Init(&pid_state, PID_INTEGRAL_LIMIT, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    PID_SetSetpoint(&pid_state, setpoint_angle);

    /* --- start watchdog: last step before entering main loop --- */
    IWDG_Start();

    /* --- ready --- */
    UART_SendStr("========================================\r\n");
    UART_SendStr("[READY] All modules initialized.\r\n");
    UART_SendStr("========================================\r\n\r\n");

    OLED_Clear();
    OLED_ShowString(0, 0, "System Ready");
    OLED_ShowString(0, 2, "P:--.- R:--.-");
    OLED_ShowString(0, 4, "Set: 90 Serv:90");
    OLED_ShowString(0, 6, "Mode: RUN");
    OLED_FlushAll();                /* push boot screen (one-time block OK) */
}

/* ================================================================
 *  TASK BODIES  (each invoked at its own deadline)
 * ================================================================ */

/**
 * @brief  Task_10ms (100 Hz): sensor read -> filter -> watchdog feed.
 */
static void Task_10ms(void)
{
    float acc_pitch, acc_roll;

    MPU6050_ReadRawData(&mpu_raw);
    MPU6050_ConvertToPhy(&mpu_raw, &mpu_phy);

    Filter_AccelAngle(mpu_raw.Accel_X,
                      mpu_raw.Accel_Y,
                      mpu_raw.Accel_Z,
                      &acc_pitch, &acc_roll);

    Filter_UpdateAttitude(&filter_pitch, &filter_roll,
                          mpu_phy.Gyro_X_dps,
                          mpu_phy.Gyro_Y_dps,
                          mpu_phy.Gyro_Z_dps,
                          acc_pitch, acc_roll,
                          &attitude);

    IWDG_ReloadCounter();
}

/**
 * @brief  Task_20ms (50 Hz): servo tracking loop.
 *
 * MODE_RUN: servo mirrors the sensor pitch directly (0 deg -> center
 * 90 deg; pitch > 0 -> > 90 deg; pitch < 0 -> < 90 deg) plus an
 * encoder bias. MODE_TUNE_*: PID output drives the servo around the
 * 90 deg center.
 */
static void Task_20ms(void)
{
    float servo_angle;

    if (tune_mode == MODE_RUN)
    {
        /* Direct tracking (open-loop): map pitch [-90..+90] to servo
         * [0..180] centered at 90 deg, plus encoder bias. */
        servo_angle = 90.0f + attitude.pitch + setpoint_angle;
    }
    else
    {
        /* PID path (tune modes): the PID drives the servo position;
         * setpoint is the desired angle, measurement is the sensor
         * pitch. The PID is bypassed in MODE_RUN (direct tracking). */
        pid_output = PID_Compute(&pid_state, &pid_params,
                                 attitude.pitch, 0.020f);
        servo_angle = 90.0f + pid_output;
    }

    /* clamp to valid servo range */
    if (servo_angle < (float)SERVO_MIN_ANGLE) servo_angle = (float)SERVO_MIN_ANGLE;
    if (servo_angle > (float)SERVO_MAX_ANGLE) servo_angle = (float)SERVO_MAX_ANGLE;

    servo_angle_current = servo_angle;
    Servo_SetAngle((int16_t)servo_angle);
}

/**
 * @brief  Task_50ms (20 Hz): build VOFA+ frame, send via DMA.
 */
static void Task_50ms(void)
{
    /* free the DMA channel if the previous frame finished */
    UART1_DMA_Update();

#ifdef VOFA_ENABLE
    {
        float vofa_data[7];
        float acc_pitch_now, acc_roll_now;
        float adc_raw_v;

        Filter_AccelAngle(mpu_raw.Accel_X,
                          mpu_raw.Accel_Y,
                          mpu_raw.Accel_Z,
                          &acc_pitch_now, &acc_roll_now);

        /* ch0: complementary-filtered pitch, already smooth */
        vofa_data[0] = attitude.pitch;

        /* ch1: roll, limit -> moving-average cascade */
        {
            float roll_limited = LimitFilter(attitude.roll, prev_roll_vofa, 5.0f);
            prev_roll_vofa = roll_limited;
            vofa_data[1] = MovAvg_Update(&ma_roll_display, roll_limited);
        }

        /* ch2: raw accel pitch, moving-average vs vibration noise */
        vofa_data[2] = MovAvg_Update(&ma_accel_pitch, acc_pitch_now);

        /* ch3/ch4: gyro rates, unfiltered */
        vofa_data[3] = mpu_phy.Gyro_X_dps;
        vofa_data[4] = mpu_phy.Gyro_Y_dps;

        /* ch5: PID output (near zero in MODE_RUN) */
        vofa_data[5] = pid_output;

        /* ch6: ADC voltage, median then moving-average */
        adc_raw_v    = (float)adc_mv / 1000.0f;
        vofa_data[6] = MovAvg_Update(&ma_adc, MedianFilter(adc_raw_v));

        VOFA_SendFrame(vofa_data, 7);
    }
#endif
}

/**
 * @brief  Task_200ms (5 Hz): ADC read, encoder poll, tune-mode switch.
 */
static void Task_200ms(void)
{
    /* --- ADC: latch the last EOC result and kick off the next conversion --- */
    if (adc_conversion_done)
    {
        adc_conversion_done = 0;
        adc_mv = (uint16_t)((uint32_t)adc_value * ADC_VREF_MV / ADC_RESOLUTION);
    }
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* --- encoder SW button: cycle through tune modes --- */
    if (Encoder_SW_Pressed())
    {
        tune_mode = (tune_mode + 1) % MODE_MAX;
        encoder_accum = 0;
    }

    /* --- encoder rotation --- */
    encoder_delta = Encoder_GetCount();
    Encoder_ResetCount();

    if (encoder_delta != 0)
    {
        switch (tune_mode)
        {
        case MODE_RUN:
            encoder_accum += encoder_delta;
            setpoint_angle = (float)encoder_accum * ENCODER_STEP_DEG;
            if (setpoint_angle < SETPOINT_MIN_DEG) setpoint_angle = SETPOINT_MIN_DEG;
            if (setpoint_angle > SETPOINT_MAX_DEG) setpoint_angle = SETPOINT_MAX_DEG;
            PID_SetSetpoint(&pid_state, setpoint_angle);
            break;

        case MODE_TUNE_KP:
            pid_params.Kp += (float)encoder_delta * TUNE_STEP_KP;
            if (pid_params.Kp < 0.0f) pid_params.Kp = 0.0f;
            break;

        case MODE_TUNE_KI:
            pid_params.Ki += (float)encoder_delta * TUNE_STEP_KI;
            if (pid_params.Ki < 0.0f) pid_params.Ki = 0.0f;
            break;

        case MODE_TUNE_KD:
            pid_params.Kd += (float)encoder_delta * TUNE_STEP_KD;
            if (pid_params.Kd < 0.0f) pid_params.Kd = 0.0f;
            break;
        }
    }

    /* OLED framebuffer: stream one page every 200 ms. Pages 0/2/4/6
     * carry the four display rows; a full update takes 800 ms and
     * no single block exceeds one page write (~50 ms). */
    OLED_FlushPage(oled_flush_pages[oled_flush_idx]);
    oled_flush_idx = (oled_flush_idx + 1) & 0x03;
}

/**
 * @brief  Task_1000ms (1 Hz): rebuild the OLED framebuffer.
 * @note   RAM-only (~us).  Pages are streamed to the panel by
 *         Task_200ms, so this task never blocks the loop on I2C.
 */
static void Task_1000ms(uint32_t now)
{
    heartbeat = !heartbeat;

    /* Wipe framebuffer and restamp every field (RAM-only); pages are
     * flushed in full, so a RAM wipe is sufficient. */
    OLED_Clear();

    /* row 0: pitch + roll (integer) + heartbeat */
    OLED_ShowString(0, 0, "P:");
    OLED_ShowNum(18, 0, (int32_t)attitude.pitch, 4);
    OLED_ShowString(48, 0, "R:");
    OLED_ShowNum(60, 0, (int32_t)attitude.roll, 4);
    OLED_ShowString(90, 0, heartbeat ? "*" : " ");

    /* row 2: setpoint + servo angle */
    OLED_ShowString(0, 2, "Set:");
    OLED_ShowNum(30, 2, (int32_t)setpoint_angle, 3);
    OLED_ShowString(60, 2, "Servo:");
    OLED_ShowNum(108, 2, (int32_t)servo_angle_current, 3);

    /* row 4: ADC voltage */
    OLED_ShowString(0, 4, "ADC:");
    OLED_ShowNum(30, 4, adc_mv, 4);
    OLED_ShowString(60, 4, "mV");

    /* row 6: mode + parameter / uptime */
    switch (tune_mode)
    {
    case MODE_RUN:
        OLED_ShowString(0, 6, "Mode: RUN  T:");
        OLED_ShowNum(90, 6, (int32_t)(now / 1000), 4);
        break;

    case MODE_TUNE_KP:
        OLED_ShowString(0, 6, "TUNE Kp=");
        OLED_ShowFloat(54, 6, pid_params.Kp, 2, 2);
        break;

    case MODE_TUNE_KI:
        OLED_ShowString(0, 6, "TUNE Ki=");
        OLED_ShowFloat(54, 6, pid_params.Ki, 2, 3);
        break;

    case MODE_TUNE_KD:
        OLED_ShowString(0, 6, "TUNE Kd=");
        OLED_ShowFloat(54, 6, pid_params.Kd, 2, 2);
        break;
    }
}

/* ================================================================
 *  MAIN DISPATCH LOOP
 * ================================================================ */

/**
 * @brief  Check every task deadline and dispatch those that are due.
 * @note   Call continuously from main(). Unsigned subtraction
 *         (now - last) stays correct across counter wrap-around
 *         (~49.7 days).
 */
void Scheduler_Run(void)
{
    uint32_t now = SysTick_Get();

    /* Unconditional feed: IWDG is serviced on every loop pass,
     * even when no task is due. */
    IWDG_ReloadCounter();

    /* ---- Task_10ms (100 Hz) ---- */
    if (now - last_10ms >= TASK_PERIOD_10MS)
    {
        last_10ms = now;
        Task_10ms();
    }

    /* ---- Task_20ms (50 Hz) ---- */
    if (now - last_20ms >= TASK_PERIOD_20MS)
    {
        last_20ms = now;
        Task_20ms();
    }

    /* ---- Task_50ms (20 Hz) ---- */
    if (now - last_50ms >= TASK_PERIOD_50MS)
    {
        last_50ms = now;
        Task_50ms();
    }

    /* ---- Task_200ms (5 Hz) ---- */
    if (now - last_200ms >= TASK_PERIOD_200MS)
    {
        last_200ms = now;
        Task_200ms();
    }

    /* ---- Task_1000ms (1 Hz) ---- */
    if (now - last_1000ms >= TASK_PERIOD_1000MS)
    {
        last_1000ms = now;
        Task_1000ms(now);
    }
}
