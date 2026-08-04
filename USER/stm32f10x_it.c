#include "stm32f10x.h"
#include "SysTick.h"
#include "Encoder.h"
#include "adc.h"

/*
 * ============================================================
 *  stm32f10x_it.c - 中断服务函数
 * ============================================================
 *
 *  这个文件存放所有中断服务函数 (ISR, Interrupt Service Routine).
 *  中断向量的入口地址在启动文件 startup_stm32f10x_md.s 中定义,
 *  中断发生时 CPU 自动跳转到对应的函数名下.
 *
 *  ISR 编写的铁律:
 *    1. 函数名必须与启动文件中定义的完全一致 (大小写敏感!)
 *    2. 中断里只做最轻量的标志位置位, 复杂处理放主循环
 *    3. 不要在中断里用 printf / OLED 等耗时操作
 * ============================================================
 */

/*
 * wwdg_ewi_triggered - EWI 中断触发标志
 *
 * 当 WWDG 计数器减到 0x40 时，WWDG_IRQHandler 被调用，
 * 这个变量被设为 1。主循环检测到它就知道中断发生了。
 *
 * 为什么不在中断里直接打印？
 * 因为从 0x40 到 0x3F (复位) 只有约 910 微秒，
 * 串口发一个字符就要约 87 微秒，根本来不及。
 * 设置一个变量只需要几十纳秒。
 */
volatile uint8_t wwdg_ewi_triggered = 0;

/* ====== 系统异常处理 ====== */
void NMI_Handler(void)           {}
void HardFault_Handler(void)     { while (1); }
void MemManage_Handler(void)     { while (1); }
void BusFault_Handler(void)      { while (1); }
void UsageFault_Handler(void)    { while (1); }
void SVC_Handler(void)           {}
void DebugMon_Handler(void)      {}
void PendSV_Handler(void)        {}
/**
 * @brief  SysTick 中断服务函数 (每 1ms 触发一次)
 *
 *  这是整个非阻塞调度器的"心跳"!
 *  每 1ms 硬件自动产生一次中断, 我们只做一件事:
 *    把全局毫秒计数器 +1.
 *
 *  为什么不在中断里做更多的事?
 *    因为 ISR 会打断主循环的执行. 如果中断里耗时太长,
 *    主循环就没有足够的时间运行, 系统响应变慢.
 *    极端情况下, 中断还没执行完, 下一个中断又来了
 *    → "中断风暴" → CPU 什么都做不了.
 *
 *  良好的中断设计: ISR 只设置标志/递增计数, 立即退出.
 *                   主循环检查标志/计数, 执行实际任务.
 */
void SysTick_Handler(void)
{
    SysTick_IncTick();
}

/*
 * ============================================================
 *  WWDG_IRQHandler - WWDG 早期唤醒中断
 * ============================================================
 *
 *  触发条件: WWDG 计数器 == 0x40 (十进制 64)
 *
 *  必须做的:
 *    1. 清除中断标志 (否则会反复触发)
 *    2. 设置通知变量 (告诉主循环)
 *
 *  触发后还剩约 910us 就复位了。
 */
void WWDG_IRQHandler(void)
{
    if (WWDG_GetFlagStatus() != RESET)
    {
        WWDG_ClearFlag();
        wwdg_ewi_triggered = 1;
    }
}

/*
 * ============================================================
 *  EXTI15_10_IRQHandler - 旋转编码器中断 (PB12/PB13)
 * ============================================================
 *
 *  触发条件: 编码器旋转时 A 相 (PB12) 或 B 相 (PB13) 下降沿
 *
 *  处理逻辑:
 *    1. 检查是哪个引脚触发了中断
 *    2. 清除对应的中断挂起位 (必须清! 否则会反复触发)
 *    3. 调用 Encoder_OnInterrupt() 处理方向判断和计数
 *
 *  EXTI 中断清除位的特性:
 *    写 1 到 EXTI->PR 的对应位来清除挂起标志.
 *    注意是写 1 清除, 不是写 0! 这是 STM32 的设计.
 *    EXTI_ClearITPendingBit() 库函数封装了这个操作.
 */
void EXTI15_10_IRQHandler(void)
{
    /*
     * 检查 EXTI Line12 (PB12, A 相)
     * 只在 A 相的下降沿做方向判断,
     * B 相的中断只用来做更精细的计数 (本工程未启用).
     * 如果想启用 B 相中断实现双边沿检测,
     * 取消下面 EXTI_Line13 的判断并处理即可.
     */
    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line12);  /* ★ 必须先清标志! 否则反复触发 */
        Encoder_OnInterrupt();                 /* 处理编码器脉冲 */
    }

    /* B 相中断: 暂不处理 (单边沿方案已满足需求) */
    if (EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line13);
        /*
         * 如果启用双边沿检测, 这里也调用 Encoder_OnInterrupt(),
         * 但需要在 Encoder_OnInterrupt() 中区分 A/B 相.
         * 双边沿可以提升编码器分辨率 (每步 1 个脉冲 → 2 个脉冲).
         */
    }
}

/*
 * ============================================================
 *  ADC1_2_IRQHandler - ADC 转换完成中断
 * ============================================================
 *
 *  触发条件: ADC1 完成一次单次转换 (EOC 标志置位)
 *
 *  ISR 只做一件事: 读取转换结果并设置完成标志.
 *  耗时操作 (如打印/显示) 放在主循环中.
 *
 *  注意: 如果 ADC 配置为连续转换模式 (ContinuousConvMode=ENABLE),
 *        中断会不停地触发, 可能占用大量 CPU 时间.
 *        本项目使用单次转换模式, 每次在任务中软件触发,
 *        转换完成后进一次中断.
 */
void ADC1_2_IRQHandler(void)
{
    /*
     * 检查是否是 ADC1 的 EOC (End of Conversion) 中断
     * 因为 ADC1 和 ADC2 共用这个中断向量,
     * 需要判断具体是哪个 ADC 触发的.
     */
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);  /* ★ 清除中断标志 */

        adc_value = ADC_GetConversionValue(ADC1);  /* 读取 12 位转换结果 */
        adc_conversion_done = 1;                    /* 通知主循环: 数据就绪 */
    }
}

/*
 * ============================================================
 *  以下为空桩函数 (占位), 防止未使能的中断意外触发时
 *  跳转到启动文件的默认死循环.
 *  如果以后用到这些中断, 在这里添加实际的处理逻辑.
 * ============================================================
 */

void EXTI4_IRQHandler(void)
{
    /* 预留: 按键中断或外部触发 */
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    /* 预留: DMA1 Channel1 传输完成中断 */
    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
    }
}
