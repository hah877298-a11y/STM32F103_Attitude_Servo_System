#include "Encoder.h"
#include "SysTick.h"

/* ============================================================
 *            旋 转 编 码 器 驱 动 实 现
 * ============================================================
 *
 *  本驱动使用 EXTI (外部中断) 方式检测编码器脉冲.
 *
 *  为什么用中断而不是轮询?
 *    编码器脉冲宽度可能只有几毫秒, 如果放在 10ms 周期的
 *    轮询任务中, 脉冲可能被漏掉.
 *    中断响应速度在微秒级, 几乎不会丢脉冲.
 *
 *  消抖机制:
 *    硬件消抖 (RC 滤波): A/B 相各接一个 0.1µF 电容到 GND,
 *      形成 RC 低通滤波器, 滤除机械触点的高频抖动.
 *
 *    软件消抖 (本文件): 记录上次中断的时间戳,
 *      如果两次中断间隔 < 消抖时间, 忽略本次触发.
 *      消抖时间 = 5ms (机械编码器抖动通常 1~3ms).
 *
 *  中断服务在哪?
 *    PB12 和 PB13 都配置为 EXTI 中断.
 *    当任一引脚电平变化 (下降沿) 触发中断时,
 *    EXTI 中断服务函数调用 Encoder_OnInterrupt().
 *
 *    我们只在 A 相 (PB12) 的下降沿触发中断并读取 B 相电平来判断方向.
 *    也可以使用双边沿触发获得更高精度 (但需要配合定时器捕获模式).
 *    单边沿方案简单可靠, 足够本项目使用.
 *
 *  NVIC 配置说明:
 *    PB12 → EXTI12 (EXTI15_10_IRQn)
 *    PB13 → EXTI13 (EXTI15_10_IRQn)
 *    两者共用一个中断向量 EXTI15_10_IRQHandler.
 * ============================================================
 */

/* 软件消抖间隔 (毫秒) */
#define ENCODER_DEBOUNCE_MS   5

/* 编码器状态变量 */
static volatile int16_t encoder_count = 0;         /* 累计脉冲数 */
static volatile uint32_t last_interrupt_time = 0;  /* 上次中断时间 (消抖用) */

/* ============================================================
 *              初 始 化 编 码 器
 * ============================================================
 */
/**
 * @brief  初始化编码器的 GPIO 和 EXTI 中断
 *
 *  步骤:
 *    1. 使能 GPIOB + AFIO 时钟
 *    2. 配置 PB12, PB13 为上拉输入
 *    3. 配置 EXTI 中断线 (PB12 → EXTI12, PB13 → EXTI13)
 *    4. 配置下降沿触发 (A 相下降沿 = 编码器转动一步)
 *    5. 配置 NVIC 中断优先级
 */
void Encoder_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    /* ===== 第 1 步: 使能时钟 ===== */
    /*
     * GPIOB 在 APB2, AFIO 也在 APB2.
     * AFIO (Alternate Function I/O) 用于配置 EXTI 中断线来源,
     * 必须使能, 否则 EXTI 不会响应.
     */
    RCC_APB2PeriphClockCmd(ENCODER_RCC_CLOCK | RCC_APB2Periph_AFIO, ENABLE);

    /* ===== 第 2 步: 配置 PB12, PB13 为上拉输入 ===== */
    /*
     * 编码器的 A/B 相在不导通时为高阻态,
     * 上拉电阻确保不导通时读到高电平 (逻辑 1).
     * 导通时引脚被拉到 GND → 读到低电平 (逻辑 0).
     *
     * 如果编码器模块自带硬件上拉, 可以用浮空输入 (GPIO_Mode_IN_FLOATING).
     * 使用上拉输入更保险, 即使模块没有上拉也能正常工作.
     */
    GPIO_InitStructure.GPIO_Pin   = ENCODER_A_PIN | ENCODER_B_PIN | ENCODER_SW_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;   /* 上拉输入 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENCODER_PORT, &GPIO_InitStructure);

    /* ===== 第 3 步: 将 GPIO 引脚映射到 EXTI 中断线 ===== */
    /*
     * STM32F103 的 EXTI 中断线和 GPIO 引脚的对应关系:
     *   EXTI0  ← PA0/PB0/PC0...   (任选一个, 由 AFIO_EXTICR 选择)
     *   EXTI1  ← PA1/PB1/PC1...
     *   ...
     *   EXTI12 ← PA12/PB12/PC12...
     *   EXTI13 ← PA13/PB13/PC13...
     *
     * 我们选择 PB12 → EXTI12, PB13 → EXTI13.
     * 这样两个引脚触发同一个中断向量 EXTI15_10_IRQHandler.
     * 在中断中通过检查 EXTI->PR 寄存器判断是哪个引脚触发的.
     */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);  /* PB12 → EXTI12 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);  /* PB13 → EXTI13 */

    /* ===== 第 4 步: 配置 EXTI 中断 ===== */
    /*
     * 触发方式: 下降沿触发 (Falling Trigger)
     *   编码器旋转时 A 相从高变为低 → 产生下降沿 → 触发中断
     *   此时读取 B 相的电平即可判断旋转方向.
     *
     * 为什么用下降沿而不是上升沿?
     *   都可以, 只是读 B 相的逻辑相反而已.
     *   下降沿更常用, 因为机械触点从悬空到接触的上升沿抖动更大.
     */
    EXTI_InitStructure.EXTI_Line    = EXTI_Line12 | EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;     /* 中断模式 */
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;     /* 下降沿触发 */
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* ===== 第 5 步: 配置 NVIC 中断优先级 ===== */
    /*
     * EXTI15_10 共用一个中断通道, 优先级设为中等.
     * 抢占优先级 2, 子优先级 0.
     *
     * 注意: 编码器中断优先级应低于 SysTick (硬件决定),
     *       但高于主循环中的任务.
     */
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ============================================================
 *        在 中 断 中 调 用: 处 理 编 码 器 脉 冲
 * ============================================================
 */
/**
 * @brief  编码器中断处理 (在 EXTI15_10_IRQHandler 中调用)
 *
 *  方向判断逻辑:
 *    A 相下降沿触发中断 → 读取 B 相电平
 *      B = 0 (低电平) → A 比 B 先变化 → 顺时针 (CW)  → count++
 *      B = 1 (高电平) → B 比 A 先变化 → 逆时针 (CCW) → count--
 *
 *  消抖逻辑:
 *    记录每次中断发生的时间, 如果两次中断间隔 < ENCODER_DEBOUNCE_MS,
 *    说明是抖动, 忽略本次触发.
 *
 *  ★ 中断中要尽量快速!
 *    本函数只做: 读时间 → 比较消抖 → 读 B 电平 → 更新计数.
 *    总共几十个 CPU 周期, 完全不会影响系统实时性.
 */
void Encoder_OnInterrupt(void)
{
    uint32_t now = SysTick_Get();

    /* 软件消抖: 间隔太短 = 抖动, 忽略 */
    if (now - last_interrupt_time < ENCODER_DEBOUNCE_MS)
    {
        /*
         * 虽然是抖动, 但也要更新 last_interrupt_time.
         * 为什么?  因为抖动是一连串快速脉冲, 如果只判断不更新,
         * 那抖动期间的每一次触发都会和"上一次有效触发"比较,
         * 间隔可能已经 > 5ms → 全部被当成有效脉冲 → 消抖失败!
         */
        last_interrupt_time = now;
        return;
    }

    last_interrupt_time = now;

    /*
     * 方向判断: 读 B 相电平
     *
     * 注意: 这里读取的是 PB13 的输入数据寄存器 (IDR).
     *       BIT_RESET = 0 (低电平) → B 相导通到 GND
     *       BIT_SET   = 1 (高电平) → B 相通过上拉电阻到 VCC
     */
    if (GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_B_PIN) == Bit_RESET)
    {
        /* B = 低 → A 先于 B → 顺时针 → count++ */
        encoder_count++;
    }
    else
    {
        /* B = 高 → B 先于 A → 逆时针 → count-- */
        encoder_count--;
    }
}

/* ============================================================
 *              获 取 / 清 零 编 码 器 计 数
 * ============================================================
 */
/**
 * @brief  获取编码器累计脉冲数
 * @return 累计值 (正向 = 顺时针, 负向 = 逆时针)
 *
 *  主循环读取这个值, 转换为角度变化:
 *    delta_angle = count * DEGREE_PER_PULSE
 *
 *  典型 EC11 编码器: 每圈 20 个脉冲 (20 个定位点),
 *  每转一圈 A 相产生 20 个下降沿 → 每个脉冲 = 360/20 = 18°
 *
 *  但实际使用时不需要计算角度, 直接用 count 值:
 *    读取一次 → 清零 → 用 count 值调整 setpoint
 *    例: count = 3 → 目标角度 +3°  (每个脉冲 ±1°)
 */

/**
 *  关于 volatile:
 *    encoder_count 被 ISR 修改, 主循环读取.
 *    volatile 告诉编译器"这个变量的值可能在任何时候被中断改变",
 *    防止编译器将它优化到寄存器中反复读取.
 *    如果不用 volatile, 编译器可能把值缓存到 R0 寄存器然后一直读 R0,
 *    永远看不到 ISR 更新的内存值 → 编码器"失灵".
 */
int16_t Encoder_GetCount(void)
{
    return encoder_count;
}

/**
 * @brief  清零累计脉冲数
 *
 *  典型用法: 主循环读取 count 并处理后清零, 准备累积下一批脉冲:
 *    int16_t delta = Encoder_GetCount();
 *    Encoder_ResetCount();
 *    if (delta != 0) { ... do something ... }
 *
 *  Note: there is a race condition between reading count and resetting it.
 *  If an interrupt fires between GetCount() and ResetCount(),
 *  that pulse will be lost.
 *
 *  Solution: disable interrupts -> read -> reset -> enable interrupts.
 *  But disabling interrupts affects real-time performance.
 *
 *  Compromise (used here): the encoder operates at low frequency (~tens of Hz),
 *  so the race probability is extremely low. Losing one pulse occasionally
 *  has negligible impact on user experience.
 */
void Encoder_ResetCount(void)
{
    encoder_count = 0;
}

/*
 * SW 按键读取, 带简单消抖.
 * PB14 配置为上拉输入, 按下时读到低电平.
 * 返回值: 1 = 检测到一次有效按下, 0 = 未按下
 *
 * 消抖逻辑: 连续两次读到的电平都是低, 才认为是有效按下.
 * 两次读数之间延时约 20ms, 足以滤除机械触点抖动.
 * 注意这里用了阻塞延时, 但 SW 只在 200ms 任务中调用,
 * 20ms 的阻塞在 200ms 周期中可以接受.
 */
uint8_t Encoder_SW_Pressed(void)
{
    /* 第一次读: 是否为低 (按下)? */
    if (GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_SW_PIN) == Bit_RESET)
    {
        /* 消抖延时 ~20ms */
        {
            volatile uint32_t i;
            for (i = 0; i < 200000; i++) __NOP();
        }

        /* 第二次读: 还是低? 确认不是抖动 */
        if (GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_SW_PIN) == Bit_RESET)
        {
            /*
             * 等待释放, 但加超时保护 (~500ms).
             * 防止按键被长按时阻塞整个 200ms 任务,
             * 进而导致 IWDG 在 1s 内得不到喂狗而复位.
             */
            volatile uint32_t timeout = 5000000;
            while (GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_SW_PIN) == Bit_RESET
                   && --timeout);
            return 1;
        }
    }
    return 0;
}
