 #ifndef __W25Q64_H
 #define __W25Q64_H
 
 #include "stm32f10x.h"
 
  #define W25Q64_CS_LOW()   GPIO_ResetBits(GPIOA, GPIO_Pin_4)
  #define W25Q64_CS_HIGH()  GPIO_SetBits(GPIOA, GPIO_Pin_4) 

  void W25Q64_Init(void);
  uint16_t W25Q64_ReadID(void);
  void W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint16_t len);
  void W25Q64_PageProgram(uint32_t addr, uint8_t *data, uint16_t len);
  void W25Q64_SectorErase(uint32_t addr);

  #endif
