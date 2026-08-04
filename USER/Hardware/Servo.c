#include "Servo.h"

/* ============================================================
 *              SG90 舵 机 PWM 驱 动 实 现
 * ============================================================
 *
 *  舵机工作原理 (通俗版):
 *
 *  舵机内部有一个小直流电机 + 齿轮组 + 电位器(角度传感器) + 控制电路.
 *  控制电路不断比较"你给的 PWM 脉冲"和"电位器读到的当前位置":
 *    - 如果 PWM 脉宽 > 当前位置 → 电机正转, 角度增大
 *    - 如果 PWM 脉宽 < 当前位置 → 电机反转, 角度减小
 *    - 如果 PWM 脉宽 ≈ 当前位置 → 电机停转, 保持角度
 *
 *  这种"自己跟自己比较并修正"的机制就是闭环控制.
 *  舵机内部已经集成了这个闭环, 我们只需要告诉它"我要多少度"即可.
 *
 *  关键参数:
 *    SG90 的 PWM 周期必须是 20ms (50Hz).
 *    脉冲宽度在 0.5ms~2.5ms 之间线性对应 0°~180°.
 *
 *    角度 → 脉宽:  pulse = 500 + (angle * 2000 / 180)
 *    例: 90° → 500 + (90*2000/180) = 500 + 1000 = 1500µs
 *
 *  TIM2 定时器配置详解:
 *    TIM2 是 16 位通用定时器, 挂在 APB1 总线.
 *    STM32F103 的 APB1 最大频率 36MHz, 但当 APB1 预分频 ≠1 时,
 *    定时器时钟 = APB1 × 2 = 72MHz (硬件自动倍频).
 *
 *    预分频器 (PSC): 把 72MHz 降频到 1MHz → 每个计数 = 1µs
 *      72,000,000 / (PSC+1) = 72,000,000 / 72 = 1,000,000 Hz
 *      → PSC = 71
 *
 *    自动重装载 (ARR): 决定 PWM 周期
 *      周期 = (ARR+1) / 1MHz = 20000 / 1,000,000 = 20ms = 50Hz
 *      → ARR = 19999
 *
 *    比较寄存器 (CCR): 决定占空比 = 高电平持续时间
 *      CCR = 500  → 500µs 高电平 → 舵机转 0°
 *      CCR = 1500 → 1500µs 高电平 → 舵机转 90°
 *      CCR = 2500 → 2500µs 高电平 → 舵机转 180°
 *
 *  ★ 面试常考: 为什么用 PWM 而不用 GPIO 电平控制舵机?
 *    因为舵机内部电路靠"脉冲宽度"而非"电平高低"来判断目标角度.
 *    如果用 GPIO: 拉高 → 电机转 → 拉低 → 电机停 → 需要自己算时间.
 *    用 PWM: 定时器硬件自动产生精确脉冲, CPU 完全不参与.
 * ============================================================
 */

/* ============================================================
 *               初 始 化 舵 机 PWM
 * ============================================================
 */
/**
 * @brief  初始化 TIM2_CH1 (PA0) 为 SG90 舵机的 PWM 输出
 *
 *  步骤:
 *    1. 使能 GPIOA + TIM2 的时钟
 *    2. 配置 PA0 为复用推挽输出 (连接到 TIM2 内部)
 *    3. 配置 TIM2 的预分频器、自动重装载、比较值
 *    4. 配置 PWM 模式 (PWM1: CNT < CCR 时输出高电平)
 *    5. 启动 TIM2
 *
 *  初始化后舵机默认停在 90° (中间位置),
 *  这样上电时不会突然甩到极限位置.
 */
void Servo_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* ===== 第 1 步: 使能时钟 ===== */
    /*
     * GPIOA 挂在 APB2 (高速总线), TIM2 挂在 APB1 (低速总线).
     * STM32 所有外设默认时钟关闭以省电, 使用时必须手动打开.
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  /* GPIOA 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   /* TIM2 时钟 */

    /* ===== 第 2 步: 配置 PA0 为复用推挽输出 ===== */
    /*
     * GPIO_Mode_AF_PP = 复用功能推挽输出
     *   普通推挽输出: GPIO 输出自己的 ODR 寄存器值
     *   复用推挽输出: GPIO 输出外设(TIM2)的信号, ODR 寄存器被绕过
     *
     * 简单理解: PA0 被"借给"TIM2 使用了, TIM2 直接控制这个引脚.
     */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;             /* PA0 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;        /* ★ 复用推挽输出 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       /* 高速 */
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ===== 第 3 步: 配置 TIM2 的时基 ===== */
    /*
     * 时基 = 定时器的"心跳节拍", 决定了计数速度和周期.
     *
     * PSC = 71:  72MHz / (71+1) = 1MHz, 每个 tick = 1µs
     * ARR = 19999:  (19999+1) / 1MHz = 20ms = 50Hz
     *
     * 计数模式: Up (从 0 数到 ARR, 然后回到 0, 循环往复)
     */
    TIM_TimeBaseStructure.TIM_Period        = 20000 - 1;   /* ARR = 19999, 20ms 周期 */
    TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;      /* PSC = 71, 1MHz */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; /* 时钟不分频 */
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up; /* 向上计数 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* ===== 第 4 步: 配置 PWM 输出模式 (通道 1) ===== */
    /*
     * PWM 模式 1: 向上计数时, CNT < CCR → 输出高, CNT >= CCR → 输出低
     *
     * 直观理解:
     *   计数从 0 开始 → 输出高电平 (脉冲开始)
     *   计数到 CCR (比如 1500) → 输出变低 (脉冲结束)
     *   计数到 ARR (19999) → 回到 0, 重新开始下一周期
     *
     *   所以 CCR 的值就决定了高电平持续的时间 = 脉冲宽度!
     *
     * TIM_OCMode_PWM1: PWM 模式 1
     * TIM_OCPolarity_High: 高电平有效 (CNT<CCR → 高, CNT>=CCR → 低)
     * TIM_OutputState_Enable: 使能输出到引脚
     */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;          /* PWM 模式 1 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   /* 输出使能 */
    TIM_OCInitStructure.TIM_Pulse       = 1500;  /* 初始脉宽 1.5ms → 舵机 90° */
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;      /* 高电平有效 */
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);  /* 通道 1 */

    /*
     * 预装载: TIM_OC1PreloadConfig 使能 CCR 寄存器的预装载.
     * 这意味着你写 CCR 的新值不会立即生效, 而是等当前 PWM 周期结束后
     * 才自动加载. 避免在周期中途改 CCR 导致输出一个异常的短脉冲.
     */
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* ===== 第 5 步: 使能自动重装载的预装载 + 启动定时器 ===== */
    TIM_ARRPreloadConfig(TIM2, ENABLE);  /* ARR 预装载 */
    TIM_Cmd(TIM2, ENABLE);               /* ★ 启动 TIM2, PWM 开始输出! */
}

/* ============================================================
 *            设 置 舵 机 角 度 (0° ~ 180°)
 * ============================================================
 */
/**
 * @brief  设置舵机角度
 * @param  angle: 目标角度 (0 ~ 180)
 *
 *  角度到脉宽的线性映射:
 *    pulse = SERVO_MIN_PULSE + angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180
 *          = 500 + angle * 2000 / 180
 *
 *  边界保护:
 *    如果传入的角度超出 0~180 范围, 自动钳位到边界值.
 *    防止舵机接收到超出硬件承受范围的脉冲而损坏.
 *
 *  注意: 给舵机发指令后, 它需要一定时间才能转到目标角度.
 *  SG90 空载转速约 0.12 秒/60°, 所以从 0° 转到 180° 约需 0.36 秒.
 *  如果 PID 计算频率 > 20Hz, 实际机械运动跟不上电信号的变化.
 */
void Servo_SetAngle(int16_t angle)
{
    uint16_t pulse;

    /* 边界钳位保护 */
    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    /*
     * 线性映射: 角度 → 脉宽
     *
     * 为什么要分两步算?
     *   pulse = 500 + angle * 2000 / 180
     *
     *   直接写 pulse = 500 + angle * 2000 / 180:
     *     angle=180, pulse=500 + 360000/180 = 500+2000 = 2500 ✓
     *     angle=0,   pulse=500 + 0/180       = 500+0    = 500  ✓
     *     angle=90,  pulse=500 + 180000/180   = 500+1000 = 1500 ✓
     *
     *   注意: 先乘后除可以保留精度.
     *   如果先除后乘: 2000/180 ≈ 11.11 → 取整=11 → 误差!
     */
    pulse = (uint16_t)(SERVO_MIN_PULSE +
             (uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180);

    /* 写入比较寄存器, 下一周期自动生效 (因为有预装载) */
    TIM_SetCompare1(TIM2, pulse);
}

/* ============================================================
 *          直 接 设 置 脉 冲 宽 度 (µs)
 * ============================================================
 */
/**
 * @brief  直接设置 PWM 脉冲宽度 (高级用户接口)
 * @param  pulse_us: 脉冲宽度, 单位 µs, 范围 500~2500
 *
 *  这个函数绕过了角度转换, 适合:
 *    - 精确标定舵机 (有的舵机不是在 500~2500 严格对应 0~180°)
 *    - PID 直接输出脉冲值而非角度值
 *    - 测试舵机极限范围
 */
void Servo_SetPulse(uint16_t pulse_us)
{
    /* 边界保护 */
    if (pulse_us < SERVO_MIN_PULSE)
        pulse_us = SERVO_MIN_PULSE;
    if (pulse_us > SERVO_MAX_PULSE)
        pulse_us = SERVO_MAX_PULSE;

    TIM_SetCompare1(TIM2, pulse_us);
}
