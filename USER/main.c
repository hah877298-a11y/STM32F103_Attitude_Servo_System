#include "stm32f10x.h"
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

/**
 * @file  main.c
 * @brief Multi-sensor attitude acquisition & servo control system
 *        MCU: STM32F103C8T6 @ 72MHz
 *
 * Non-blocking time-sliced tasks (SysTick based):
 *   10ms  IMU read + complementary filter + IWDG feed
 *   20ms  PID compute + servo update
 *   50ms  VOFA+ data frame (7 ch) + DMA update
 *   200ms ADC + encoder + tune mode switch
 *   1000ms OLED refresh
 */

static MPU6050_RawData  mpu_raw;
static MPU6050_PhyData  mpu_phy;
static AttitudeAngle    attitude;

static ComplementaryFilter filter_pitch;
static ComplementaryFilter filter_roll;

static PID_State  pid_state;
static PID_Params pid_params;
static float      pid_output;

static float      setpoint_angle = 0.0f;
static int16_t    encoder_delta;
static int32_t    encoder_accum;

/* Tune mode selected by encoder SW button (PB14):
 *   MODE_RUN:        encoder adjusts setpoint
 *   MODE_TUNE_KP/KI/KD: encoder adjusts the respective PID gain
 *   OLED bottom line shows the active parameter. */
#define MODE_RUN      0
#define MODE_TUNE_KP  1
#define MODE_TUNE_KI  2
#define MODE_TUNE_KD  3
#define MODE_MAX      4

static uint8_t  tune_mode = MODE_RUN;

/**
 * @brief  Initialize independent watchdog, timeout ~1s
 * @note   LSI ~40kHz / prescaler 64 = 625Hz, reload 625 -> 1s.
 *         Fed at 100Hz in the 10ms task, far below the timeout.
 */
static void IWDG_Init(void)
{
    /* enable LSI clock and wait until ready */
    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    /* unlock IWDG write protection */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    /* 40kHz / 64 = 625Hz */
    IWDG_SetPrescaler(IWDG_Prescaler_64);

    /* 625 counts -> ~1s timeout */
    IWDG_SetReload(625);

    /* reload before enabling, then start */
    IWDG_ReloadCounter();
    IWDG_Enable();

    UART_SendStr("[OK]   IWDG enabled, timeout ~1s\r\n");
}

/**
 * @brief  One-time initialization of all system modules
 */
static void System_InitAll(void)
{
    SysTick_Init();

    UART1_Configuration();
    UART1_DMA_Init();

    UART_SendStr("\r\n\r\n");
    UART_SendStr("========================================\r\n");
    UART_SendStr("  STM32 Attitude & Servo Control System\r\n");
    UART_SendStr("  MCU: STM32F103C8T6 @ 72MHz\r\n");
    UART_SendStr("========================================\r\n");

    CheckAndReportResetSource();

    UART_SendStr("[INIT] IWDG watchdog...\r\n");
    IWDG_Init();

    UART_SendStr("[INIT] I2C Bus...\r\n");
    SoftI2C_Init();

    UART_SendStr("[INIT] OLED...\r\n");
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "System Boot...");

    UART_SendStr("[INIT] MPU6050...\r\n");
    MPU6050_Init();
    {
        uint8_t whoami = MPU6050_ReadID();
        if (whoami == 0x68)
        {
            UART_SendStr("[OK]   MPU6050 WHO_AM_I = 0x68\r\n");
            OLED_ShowString(0, 2, "MPU6050: OK");
        }
        else
        {
            UART_SendStr("[ERR]  MPU6050 WHO_AM_I = 0x");
            UART_SendHex(whoami);
            UART_SendStr(" (Expected 0x68!)\r\n");
            OLED_ShowString(0, 2, "MPU6050: FAIL!");
        }
    }

    UART_SendStr("[INIT] Servo PWM (PA0)...\r\n");
    Servo_Init();
    Servo_SetAngle(90);
    UART_SendStr("[OK]   Servo at 90 deg\r\n");

    UART_SendStr("[INIT] Encoder (PB12/PB13, SW=PB14)...\r\n");
    Encoder_Init();
    UART_SendStr("[OK]   Encoder ready\r\n");

    UART_SendStr("[INIT] ADC1_CH1 (PA1)...\r\n");
    ADC_GPIO_Config();
    ADC1_Mode_Config();
    UART_SendStr("[OK]   ADC ready\r\n");

    UART_SendStr("[INIT] Complementary Filter (alpha=0.98, dt=10ms)...\r\n");
    Filter_Init(&filter_pitch, 0.98f, 0.010f);
    Filter_Init(&filter_roll,  0.98f, 0.010f);

    UART_SendStr("[INIT] PID (Kp=2.0, Ki=0.05, Kd=0.5)...\r\n");
    pid_params.Kp = 2.0f;
    pid_params.Ki = 0.05f;
    pid_params.Kd = 0.5f;
    PID_Init(&pid_state, 20.0f, 0.0f, 180.0f);
    PID_SetSetpoint(&pid_state, setpoint_angle);

    UART_SendStr("========================================\r\n");
    UART_SendStr("[READY] All modules initialized.\r\n");
    UART_SendStr("========================================\r\n\r\n");

    OLED_Clear();
    OLED_ShowString(0, 0, "System Ready");
    OLED_ShowString(0, 2, "P:--.- R:--.-");
    OLED_ShowString(0, 4, "Set: 90 Serv:90");
    OLED_ShowString(0, 6, "Mode: RUN");
}

/**
 * @brief  Application entry: init all modules, run task loop
 */
int main(void)
{
    System_InitAll();

    static uint32_t last_1000ms = 0;
    static uint32_t last_200ms  = 0;
    static uint32_t last_50ms   = 0;
    static uint32_t last_20ms   = 0;
    static uint32_t last_10ms   = 0;
    static uint8_t  heartbeat   = 0;

    static uint16_t adc_mv = 0;  /* ADC voltage (mV), updated at 5Hz */

    while (1)
    {
        uint32_t now = SysTick_Get();

        /* Task_10ms (100Hz): IMU + filter + feed watchdog */
        if (now - last_10ms >= 10)
        {
            last_10ms = now;

            IWDG_ReloadCounter();

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
        }

        /* Task_20ms (50Hz): PID + servo */
        if (now - last_20ms >= 20)
        {
            last_20ms = now;

            pid_output = PID_Compute(&pid_state, &pid_params,
                                     attitude.pitch, 0.020f);
            Servo_SetAngle((int16_t)pid_output);
        }

        /* Task_50ms (20Hz): VOFA+ data frame + DMA update */
        if (now - last_50ms >= 50)
        {
            last_50ms = now;

            UART1_DMA_Update();

            float vofa_data[7];
            float acc_pitch_now, acc_roll_now;

            Filter_AccelAngle(mpu_raw.Accel_X,
                              mpu_raw.Accel_Y,
                              mpu_raw.Accel_Z,
                              &acc_pitch_now, &acc_roll_now);

            vofa_data[0] = attitude.pitch;
            vofa_data[1] = attitude.roll;
            vofa_data[2] = acc_pitch_now;
            vofa_data[3] = mpu_phy.Gyro_X_dps;
            vofa_data[4] = mpu_phy.Gyro_Y_dps;
            vofa_data[5] = pid_output;
            vofa_data[6] = (float)adc_mv / 1000.0f;  /* ADC voltage (V) */

            VOFA_SendFrame(vofa_data, 7);
        }

        /* Task_200ms (5Hz): ADC + encoder + mode switch */
        if (now - last_200ms >= 200)
        {
            last_200ms = now;

            if (adc_conversion_done)
            {
                adc_conversion_done = 0;
                /* 12-bit ADC -> mV: V = ADC * 3300 / 4096 */
                adc_mv = (uint16_t)((uint32_t)adc_value * 3300 / 4096);
            }
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);

            /* encoder SW (PB14): cycle tune mode */
            if (Encoder_SW_Pressed())
            {
                tune_mode = (tune_mode + 1) % MODE_MAX;
                encoder_accum = 0;  /* restart tuning from current value */
            }

            encoder_delta = Encoder_GetCount();
            Encoder_ResetCount();

            if (encoder_delta != 0)
            {
                switch (tune_mode)
                {
                case MODE_RUN:
                    /* encoder adjusts setpoint */
                    encoder_accum += encoder_delta;
                    setpoint_angle = (float)encoder_accum * 0.5f;
                    if (setpoint_angle < -45.0f) setpoint_angle = -45.0f;
                    if (setpoint_angle >  45.0f) setpoint_angle =  45.0f;
                    PID_SetSetpoint(&pid_state, setpoint_angle);
                    break;

                case MODE_TUNE_KP:
                    pid_params.Kp += (float)encoder_delta * 0.1f;
                    if (pid_params.Kp < 0.0f) pid_params.Kp = 0.0f;
                    break;

                case MODE_TUNE_KI:
                    pid_params.Ki += (float)encoder_delta * 0.01f;
                    if (pid_params.Ki < 0.0f) pid_params.Ki = 0.0f;
                    break;

                case MODE_TUNE_KD:
                    pid_params.Kd += (float)encoder_delta * 0.05f;
                    if (pid_params.Kd < 0.0f) pid_params.Kd = 0.0f;
                    break;
                }
            }
        }

        /* Task_1000ms (1Hz): OLED refresh */
        if (now - last_1000ms >= 1000)
        {
            last_1000ms = now;
            heartbeat = !heartbeat;

            /* row 0: attitude + heartbeat */
            OLED_ShowString(0, 0, "P:");
            OLED_ShowFloat(12, 0, attitude.pitch, 3, 1);
            OLED_ShowString(54, 0, "R:");
            OLED_ShowFloat(66, 0, attitude.roll, 3, 1);
            if (heartbeat)
                OLED_ShowString(114, 0, "*");
            else
                OLED_ShowString(114, 0, " ");

            /* row 2: setpoint + servo angle */
            OLED_ShowString(0, 2, "Set:");
            OLED_ShowNum(30, 2, (int32_t)setpoint_angle, 3);
            OLED_ShowString(60, 2, "Servo:");
            OLED_ShowNum(108, 2, (int32_t)pid_output, 3);

            /* row 4: ADC voltage */
            OLED_ShowString(0, 4, "ADC:");
            OLED_ShowNum(30, 4, adc_mv, 4);
            OLED_ShowString(60, 4, "mV");

            /* row 6: mode + active parameter */
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
    }
}
