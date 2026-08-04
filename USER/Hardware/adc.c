#include "adc.h"

/*
 * ============================================================
 *  adc_value - ADC 转换结果（由中断服务函数更新）
 *  adc_conversion_done - 转换完成标志（ISR 置 1，主循环清零）
 * ============================================================
 */
volatile uint16_t adc_value = 0;
volatile uint8_t  adc_conversion_done = 0;

/*
 * ============================================================
 *  ADC_GPIO_Config - 配置 PA1 为模拟输入
 * ============================================================
 *
 *  ADC1 通道 1 对应的引脚是 PA1。
 *  模拟输入模式让引脚连接到 ADC 模块内部，
 *  而不是数字 GPIO 电路。
 *
 *  同时也使能了 ADC1 的外设时钟。
 */
void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能 GPIOA 和 ADC1 的时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    /*
     * ADC 时钟预分频: PCLK2=72MHz → 6分频 = 12MHz
     *
     * 必须设! SystemInit() 把 ADCPRE 清零了（默认2分频=36MHz），
     * 超过 STM32F103 ADC 最大允许的 14MHz，超频会导致校准失败、
     * 转换不触发、读数异常等随机问题。
     */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* PA1 = 模拟输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*
 * ============================================================
 *  ADC1_Mode_Config - 配置 ADC1 工作模式与中断
 * ============================================================
 *
 *  配置要点:
 *    1. 独立模式 (单 ADC 工作)
 *    2. 禁止扫描模式 (只转换一个通道)
 *    3. 单次转换模式 (触发一次，转换一次)
 *    4. 软件触发 (不依赖定时器等外部信号)
 *    5. 右对齐 (12位数据低12位有效)
 *    6. 使能 EOC (End of Conversion) 中断
 *    7. 配置 NVIC 中断向量
 *    8. ADC 校准
 *
 *  中断流程:
 *    软件触发 → ADC 采样+转换 → 完成时硬件置 EOC 标志
 *    → NVIC 触发 ADC1_2_IRQHandler → 中断里读数据 + 清标志
 */
void ADC1_Mode_Config(void)
{
    ADC_InitTypeDef   ADC_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /*
     * ====== 第 1 步: ADC 工作模式配置 ======
     */

    /* 先填充默认值，再修改需要的字段 */
    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;       // 独立模式
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;                    // 单通道，不扫描
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                    // 单次转换
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;  // 软件触发
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;        // 右对齐
    ADC_InitStructure.ADC_NbrOfChannel       = 1;                          // 只转换 1 个通道
    ADC_Init(ADC1, &ADC_InitStructure);

    /*
     * ====== 第 2 步: 配置通道 1 的采样时间 ======
     *
     * 采样时间越长，信号越稳定，但转换越慢。
     * 55.5 个时钟周期 (约 4.6us @ 12MHz ADCCLK) 是常用折衷值。
     *
     * Rank=1 表示在其所在序列中第一个被转换。
     */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);

    /*
     * ====== 第 3 步: 使能 EOC 中断 ======
     *
     * 每次单次转换完成后，ADC 的 EOC 标志位置位，
     * 同时触发中断，通知 CPU 去读数据。
     */
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    /*
     * ====== 第 4 步: 使能 ADC 并校准 ======
     *
     * 校准是必须的！未校准的 ADC 读数不准。
     *
     * 注意: 校准前必须确保 ADC 时钟 ≤ 14MHz，
     * 否则校准可能永远不完成导致死循环。
     * ADC_GPIO_Config() 中已设置 ADCCLK=12MHz（PCLK2/6）。
     */
    ADC_Cmd(ADC1, ENABLE);

    /* 等待 ADC 上电稳定 (tSTAB ≈ 1μs，这里多等一会) */
    {
        volatile uint32_t t = 10000;
        while (t--);
    }

    /* 复位校准（带超时保护） */
    ADC_ResetCalibration(ADC1);
    {
        volatile uint32_t cal_timeout = 100000;
        while (ADC_GetResetCalibrationStatus(ADC1) && --cal_timeout);
    }

    /* 执行校准（带超时保护） */
    ADC_StartCalibration(ADC1);
    {
        volatile uint32_t cal_timeout = 100000;
        while (ADC_GetCalibrationStatus(ADC1) && --cal_timeout);
    }

    /*
     * ====== 第 5 步: 配置 NVIC 中断向量 ======
     *
     * ADC1 和 ADC2 共用一个中断通道: ADC1_2_IRQn。
     * 中断优先级: 抢占优先级 1，子优先级 1。
     */
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
