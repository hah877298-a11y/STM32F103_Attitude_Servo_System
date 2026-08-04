#include "usart.h"
#include <stddef.h>

/* ============================================================
 *          USART1 + DMA 串 口 高 级 驱 动 实 现
 * ============================================================
 *
 *  DMA (Direct Memory Access) 工作原理:
 *
 *    ┌─────────┐          ┌─────────────┐          ┌──────────┐
 *    │  CPU    │──指令──► │ DMA 控制器  │──搬运──► │ USART1   │
 *    │ "帮我把 │          │ (7通道)     │          │ TX寄存器 │
 *    │  这包数  │          │ 自动从内存  │          │          │
 *    │  据发了" │          │ 搬到串口    │          │          │
 *    └─────────┘          └─────────────┘          └──────────┘
 *                               ↑
 *                          CPU 可以去做别的事了!
 *                          比如刷新 OLED / 读 MPU6050
 *
 *  对比:
 *    阻塞发送: CPU 每发 1 字节都要死等 "发送完没有?"  (忙查询)
 *    DMA 发送: CPU 说"发这个数组", DMA 控制器自动搬运, 发完通知
 *
 *  VOFA+ JustFloat 协议详解:
 *    VOFA+ 是开源的串口波形显示上位机 (类似 SerialPlot),
 *    支持多种协议, 其中 JustFloat 最简单——直接发原始 float 字节:
 *
 *    帧结构 (N 通道):
 *      [通道1: 4字节 float][通道2: 4字节 float]...[通道N: 4字节 float][帧尾: 4字节]
 *
 *    帧尾 = 0x7F800000 (float +Inf 的大端表示)
 *    注意: float 在 ARM Cortex-M3 (little-endian) 中的字节序!
 *    例: float 值 3.14f 在内存中是: [0xC3][0xF5][0x48][0x40]
 *    串口发送顺序: 先发低地址 → 0xC3, 0xF5, 0x48, 0x40
 *    VOFA+ 端配置为 little-endian 即可正确解析.
 * ============================================================
 */

/* DMA 传输中标志: 防止上一帧还没发完就写下一帧 */
static volatile uint8_t dmaTxBusy = 0;

/* ============================================================
 *         USART1 基 础 初 始 化 (直 接 寄 存 器)
 * ============================================================
 */
/**
 * @brief  初始化 USART1: PA9/TX, PA10/RX, 115200-8-N-1
 *
 *  使用直接寄存器操作而非库函数, 代码更紧凑且已验证.
 *
 *  波特率计算 (面试常考):
 *    USARTDIV = PCLK2 / (16 × BaudRate)
 *             = 72,000,000 / (16 × 115,200)
 *             = 39.0625
 *    DIV_Mantissa = 39  (整数部分, 12 位)
 *    DIV_Fraction = 0.0625 × 16 = 1  (小数部分, 4 位)
 *    BRR = (39 << 4) | 1 = 0x0271
 *
 *  波特率误差 = |实际值 - 期望值| / 期望值
 *             = |115384 - 115200| / 115200 = 0.16% < 2% → 通信可靠
 */
void UART1_Configuration(void)
{
    /* 1. 使能 GPIOA + USART1 时钟 (APB2 总线) */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* 2. PA9 = 50MHz 复用推挽输出 (Alternate Function Push-Pull)
     *    PA9 处于 CRH 的 bits[7:4] (因为 PA9 是 GPIOA 第 9 引脚)
     *    每个引脚占 4 位配置: CNF[1:0] + MODE[1:0]
     *    0xB = 1011 = CNF=10(复用输出) + MODE=11(50MHz)
     */
    GPIOA->CRH &= ~(0xF << 4);      /* 清除 PA9 的 4 个配置位 */
    GPIOA->CRH |=  (0xB << 4);      /* PA9 = AF Push-Pull, 50MHz */

    /* PA10 = 浮空输入 (RX 引脚, 不配置也行, 复位默认就是浮空输入) */
    GPIOA->CRH &= ~(0xF << 8);      /* 清除 PA10 配置位 */
    GPIOA->CRH |=  (0x4 << 8);      /* CNF=01(浮空输入), MODE=00 */

    /* 3. 波特率 115200 */
    USART1->BRR = (39 << 4) | 1;

    /* 4. 使能 USART: TX + RX + USART模块 */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}

/* ============================================================
 *        DMA 初 始 化 (DMA1_Channel4 → USART1_TX)
 * ============================================================
 */
/**
 * @brief  配置 DMA1_Channel4 用于 USART1 的 TX 发送
 *
 *  STM32F103 的 DMA 资源分配:
 *    DMA1_Channel4 → USART1_TX (这个通道是硬件固定的, 不能改)
 *    DMA1_Channel5 → USART1_RX
 *    DMA1_Channel6 → USART2_TX (如果用了 USART2)
 *    DMA1_Channel7 → USART2_RX
 *
 *  DMA 关键配置:
 *    传输方向:   内存 → 外设 (Memory-to-Peripheral)
 *    内存地址:   数据缓冲区 (每发一字节, 地址自动 +1)
 *    外设地址:   &USART1->DR (固定不变)
 *    传输单位:   Byte × Byte
 *    模式:       Normal (单次, 非循环)
 *    优先级:     Medium (不高不低, 串口对延迟不敏感)
 */
void UART1_DMA_Init(void)
{
    /* 1. 使能 DMA1 时钟 (挂在 AHB 总线) */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* 2. 配置 DMA1_Channel4 */
    /*
     * DMA 结构体各成员含义:
     *   DMA_PeripheralBaseAddr = 外设寄存器的地址 (数据写到这里)
     *   DMA_MemoryBaseAddr     = 内存缓冲区的地址 (数据从这里来)
     *                           → 实际发送时在 UART1_DMA_Send() 中动态设置
     *   DMA_DIR                = 方向: 内存→外设 (PeripheralDST)
     *   DMA_BufferSize         = 传输字节数, 每次递减到 0 时传输完成
     *   DMA_PeripheralInc      = 外设地址是否自增 → DISABLE (固定写 DR 寄存器)
     *   DMA_MemoryInc          = 内存地址是否自增 → ENABLE (逐字节读取缓冲区)
     *   DMA_PeripheralDataSize = 外设数据宽度 → Byte (USART DR 是 8 位)
     *   DMA_MemoryDataSize     = 内存数据宽度 → Byte
     *   DMA_Mode               = Normal / Circular → Normal (一次性传输)
     *   DMA_Priority           = 通道优先级 → Medium
     *   DMA_M2M                = 存储器到存储器 → DISABLE (我们是用外设触发)
     */
    DMA_InitTypeDef dma;

    DMA_DeInit(DMA1_Channel4);              /* 先复位通道到默认状态 */

    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;  /* ★ 外设 = 串口数据寄存器 */
    dma.DMA_MemoryBaseAddr     = 0;                      /* 待填充, 发送时再设 */
    dma.DMA_DIR                = DMA_DIR_PeripheralDST;  /* 内存 → 外设 */
    dma.DMA_BufferSize         = 0;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;  /* 外设地址不变 */
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;       /* 内存地址递增 */
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /* 8 位传输 */
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;            /* 单次模式, 非循环 */
    dma.DMA_Priority           = DMA_Priority_Medium;
    dma.DMA_M2M                = DMA_M2M_Disable;            /* 非内存到内存 */
    DMA_Init(DMA1_Channel4, &dma);

    /* 3. 使能 USART1 的 DMA 发送功能
     *    USART_CR3_DMAT = bit7: 当 TXE=1 (发送寄存器空) 时, 自动触发 DMA 请求
     *    DMA 控制器看到请求 → 从内存搬 1 字节到 USART1->DR → 串口自动发出
     */
    USART1->CR3 |= USART_CR3_DMAT;

    /*
     * DMA 传输完成中断 (可选):
     *   发完一批数据后 DMA 会产生 TC (Transfer Complete) 中断.
     *   本工程暂时不用中断, 在 UART1_DMA_Send() 中通过 dmaTxBusy 标志
     *   阻止重复发送.
     *   如果后续需要"发完回调", 可以在此处使能 NVIC.
     */
}

/* ============================================================
 *              DMA 非 阻 塞 发 送
 * ============================================================
 */
/**
 * @brief  使用 DMA 发送一个数据缓冲区 (非阻塞, CPU 立即返回)
 * @param  buf: 数据缓冲区指针
 * @param  len: 要发送的字节数
 *
 *  工作流程:
 *    1. 检查上一帧是否发完 → 如果没完, 直接跳过 (防止数据错乱)
 *    2. 禁用 DMA 通道 → 修改配置 → 重新启用
 *    3. 设置内存基地址 + 传输长度
 *    4. 启动 DMA 传输 → 函数立即返回!
 *    5. DMA 在后台逐字节搬运, 发完自动停
 *
 *  "非阻塞" 的含义:
 *    调用这个函数后, CPU 立刻继续执行后面的代码,
 *    不需要等数据发完. DMA 在后台自动完成传输.
 *    相比 UART_SendStr() 的死等, 效率提升巨大.
 */
void UART1_DMA_Send(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
        return;

    /* 如果上一帧还在发送中, 放弃本次发送.
     * (简单策略: 宁可丢帧, 也不能让 DMA 配置混乱)
     * 后续可以改为环形缓冲区 + 等待完成.
     */
    if (dmaTxBusy)
        return;

    dmaTxBusy = 1;

    /* 禁用通道, 修改配置 */
    DMA_Cmd(DMA1_Channel4, DISABLE);

    /* 设置内存源地址 + 传输长度 */
    DMA1_Channel4->CMAR  = (uint32_t)buf;       /* CMAR = 通道内存地址寄存器 */
    DMA1_Channel4->CNDTR = len;                  /* CNDTR = 通道传输数量寄存器 */

    /* 清除上次传输的完成标志, 重新启动 */
    DMA_ClearFlag(DMA1_FLAG_TC4);               /* TC = Transfer Complete */
    DMA_Cmd(DMA1_Channel4, ENABLE);

    /*
     * 此时 DMA 已经开始工作!
     * 每当 USART1->DR 为空 (TXE=1), 硬件自动触发 DMA 请求,
     * DMA 从 buf 取 1 字节写入 DR, 然后 CNDTR--,
     * 重复直到 CNDTR 减到 0 → 传输完成 → DMA 自动停止.
     *
     * 但 dmaTxBusy 标志怎么清除?
     *   在 UART1_DMA_Update() 中检查 DMA_GetFlagStatus(),
     *   这个函数应在主循环中周期性调用.
     */
}

/**
 * @brief  检查 DMA 发送是否完成, 完成则清除忙碌标志
 *
 *  必须在主循环中周期性调用 (比如每 10ms 一次).
 *  如果不用这个函数, dmaTxBusy 永远为 1,
 *  后续的 DMA 帧全部被丢弃!
 */
void UART1_DMA_Update(void)
{
    if (dmaTxBusy && DMA_GetFlagStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag(DMA1_FLAG_TC4);
        dmaTxBusy = 0;
    }
}

/* ============================================================
 *            VOFA+ JustFloat 协 议 发 送
 * ============================================================
 */
/**
 * @brief  按 VOFA+ JustFloat 格式发送一帧多通道浮点数据
 * @param  data:     浮点数组指针, data[0]~data[channels-1] 是有效数据
 * @param  channels: 通道数 (1 ~ VOFA_MAX_CHANNELS)
 *
 *  VOFA+ 软件配置:
 *    1. 打开 VOFA+ → 选择串口 → 波特率 115200
 *    2. 左下角协议选择 "JustFloat"
 *    3. 通道数设为与 channels 一致
 *    4. 字节序选择 "Little Endian" (ARM 默认)
 *    5. 点击连接 → 自动显示波形!
 *
 *  帧尾标记 (float +Inf):
 *    在 IEEE 754 标准中:
 *      0x7F800000 = 0 11111111 00000000000000000000000
 *                    ↑ 符号=0  指数=255  尾数=0
 *    指数全 1 + 尾数全 0 = 正无穷大 (+Inf)
 *    VOFA+ 看到 +Inf 就知道 "上一帧结束了, 下一帧开始了".
 *
 *  缓冲区分配:
 *    channels 个 float × 4 字节 + 4 字节帧尾
 *    = channels × 4 + 4
 *    最大 6 通道 × 4 + 4 = 28 字节, 放栈上完全没问题.
 */
void VOFA_SendFrame(const float *data, uint8_t channels)
{
    uint8_t buf[VOFA_MAX_CHANNELS * 4 + 4];  /* 最大 28 字节 */
    uint8_t i, pos = 0;
    uint32_t tail = VOFA_TAIL;               /* float +Inf 的整数表示 */

    if (channels == 0 || channels > VOFA_MAX_CHANNELS)
        return;

    /*
     * 逐通道编码:
     *   每个 float 占 4 字节, 在 little-endian 的 STM32 上,
     *   低字节在前, 高字节在后.
     *   直接通过 uint32_t 指针读取 float 的底层表示.
     */
    for (i = 0; i < channels; i++)
    {
        uint32_t raw = *(const uint32_t *)(&data[i]);  /* 取出 float 的 IEEE 754 编码 */

        buf[pos++] = (uint8_t)(raw);        /* byte0: LSB */
        buf[pos++] = (uint8_t)(raw >> 8);   /* byte1 */
        buf[pos++] = (uint8_t)(raw >> 16);  /* byte2 */
        buf[pos++] = (uint8_t)(raw >> 24);  /* byte3: MSB */
    }

    /* 帧尾: float +Inf = 0x7F800000, 小端序: 0x00, 0x00, 0x80, 0x7F */
    buf[pos++] = (uint8_t)(tail);
    buf[pos++] = (uint8_t)(tail >> 8);
    buf[pos++] = (uint8_t)(tail >> 16);
    buf[pos++] = (uint8_t)(tail >> 24);

    /* 通过 DMA 发送整帧 (非阻塞!) */
    UART1_DMA_Send(buf, pos);
}

/* ============================================================
 *         阻 塞 发 送 函 数 (调 试 / 启 动 信 息 用)
 * ============================================================
 */
/*
 * 以下函数用于开机打印调试信息、错误提示等少量字符串.
 * 优点是简单可靠 (不需要 DMA), 缺点是一次只能发一个字节 CPU 要等着.
 * 对调试信息来说这不是问题——开机只打印一次, 占不到 1ms.
 *
 * 高频传感器数据不要用这些函数! 应该用 VOFA_SendFrame().
 */

/* 发送一个字节 (阻塞: 等 TXE) */
void UART_SendByte(uint8_t byte)
{
    while (!(USART1->SR & USART_SR_TXE));   /* 等发送寄存器空 */
    USART1->DR = byte;                        /* 写入数据 */
}

/* 发送字符串 (阻塞) */
void UART_SendStr(const char *str)
{
    while (*str)
    {
        UART_SendByte((uint8_t)(*str++));
    }
}

/* 发送十进制无符号整数 (阻塞) */
void UART_SendDec(uint32_t num)
{
    char buf[12];
    uint8_t i = 0;

    if (num == 0)
    {
        UART_SendByte('0');
        return;
    }

    while (num > 0)
    {
        buf[i++] = '0' + (num % 10);
        num /= 10;
    }

    while (i > 0)
        UART_SendByte(buf[--i]);
}

/* 发送十六进制 2 位 (阻塞) */
void UART_SendHex(uint8_t num)
{
    static const char hex[] = "0123456789ABCDEF";
    UART_SendByte(hex[(num >> 4) & 0x0F]);
    UART_SendByte(hex[num & 0x0F]);
}

/* ============================================================
 *                 复 位 源 检 查
 * ============================================================
 */
/**
 * @brief  打印系统复位来源 (调试利器!)
 *
 *  常见复位原因:
 *    Power-On Reset  → 正常上电, 每块板子开机都有
 *    NRST Pin Reset  → 按了复位按钮, 或者仿真器复位
 *    Software Reset  → 软件主动调用 NVIC_SystemReset()
 *    IWDG Reset      → 独立看门狗复位 (程序跑飞了!)
 *    WWDG Reset      → 窗口看门狗复位 (程序逻辑异常!)
 *
 *  调试技巧:
 *    如果程序频繁重启, 先看这个输出.
 *    如果出现 IWDG/WWDG 复位 → 程序卡死在某个地方, 看门狗没喂.
 */
void CheckAndReportResetSource(void)
{
    UART_SendStr("\r\n========== Reset Source ==========\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET)
        UART_SendStr("[!] WWDG Reset Detected\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
        UART_SendStr("[!] IWDG Reset Detected\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        UART_SendStr("[!] NRST Pin Reset Detected\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        UART_SendStr("[!] Power-On Reset Detected\r\n");

    if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET)
        UART_SendStr("[!] Software Reset Detected\r\n");

    RCC_ClearFlag();

    UART_SendStr("==================================\r\n\r\n");
}
