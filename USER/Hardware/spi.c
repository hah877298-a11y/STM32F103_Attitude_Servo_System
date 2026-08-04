#include "spi.h"


void MySPI_Init(void)
{
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE); 

   GPIO_InitTypeDef GPIO_InitStruct;
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init( GPIOA, &GPIO_InitStruct);
   
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 |  GPIO_Pin_7;
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init( GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init( GPIOA, &GPIO_InitStruct);
  
   SPI_InitTypeDef SPI_InitStruct;
   SPI_InitStruct.SPI_Mode = SPI_Mode_Master ;
   SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
   SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
   SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
   SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
   SPI_InitStruct.SPI_CRCPolynomial = 7;
   SPI_Init( SPI1, &SPI_InitStruct);
   
   SPI_Cmd(SPI1, ENABLE);

   GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    SPI_I2S_SendData(SPI1, ByteSend);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    return SPI_I2S_ReceiveData(SPI1);
}

