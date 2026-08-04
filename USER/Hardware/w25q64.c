  #include "w25q64.h"
  #include "w25q64_Ins.h"
  #include "spi.h"

  void W25Q64_Init(void)
  {
    MySPI_Init();

    W25Q64_CS_HIGH();


}

 uint16_t W25Q64_ReadID(void)
 {
   uint8_t mf, dev;

   W25Q64_CS_LOW();

   MySPI_SwapByte(W25Q64_JEDEC_ID);        // 0x9F: JEDEC ID 指令

   mf  = MySPI_SwapByte(W25Q64_DUMMY_BYTE); // byte1: 厂商 ID  (W25Q64 = 0xEF)
   MySPI_SwapByte(W25Q64_DUMMY_BYTE);       // byte2: 存储类型 (0x40), 跳过
   dev = MySPI_SwapByte(W25Q64_DUMMY_BYTE); // byte3: 容量 ID  (W25Q64 = 0x16)

   W25Q64_CS_HIGH();

   return (mf << 8) | dev;
 }
 
 void W25Q64_WriteEnable()
 {
   W25Q64_CS_LOW();
   MySPI_SwapByte(W25Q64_WRITE_ENABLE);
   W25Q64_CS_HIGH();
 }

 static void W25Q64_WaitBusy()
{
  uint8_t status = 0 ;

  W25Q64_CS_LOW();

  MySPI_SwapByte(W25Q64_READ_STATUS_REGISTER_1);

  do {
          status = MySPI_SwapByte(W25Q64_DUMMY_BYTE); // 读状态
      } while (status & 0x01);
  W25Q64_CS_HIGH();
}

void W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint16_t len)
  {
      uint16_t i;

      W25Q64_CS_LOW();
      MySPI_SwapByte(W25Q64_READ_DATA);           // ① 发 0x03
      MySPI_SwapByte((addr >> 16) & 0xFF);        // ② 地址高字节
      MySPI_SwapByte((addr >> 8) & 0xFF);         // ③ 地址中字节
      MySPI_SwapByte((addr >> 0) & 0xFF);         // ④ 地址低字节
      for (i = 0; i < len; i++) {
          buf[i] = MySPI_SwapByte(W25Q64_DUMMY_BYTE);  // ⑤ 读数据
      }
      W25Q64_CS_HIGH();
  }

   void W25Q64_PageProgram(uint32_t addr, uint8_t *data, uint16_t len)
  {
      uint16_t i;

      W25Q64_WriteEnable();                       
      W25Q64_CS_LOW();
      MySPI_SwapByte(W25Q64_PAGE_PROGRAM);       
      MySPI_SwapByte((addr >> 16) & 0xFF);         
      MySPI_SwapByte((addr >> 8)  & 0xFF);
      MySPI_SwapByte((addr >> 0)  & 0xFF);
      for (i = 0; i < len; i++)
          MySPI_SwapByte(data[i]);                
      W25Q64_CS_HIGH();
      W25Q64_WaitBusy();                           
  }

  void W25Q64_SectorErase(uint32_t addr)
  {
      W25Q64_WriteEnable();  

      W25Q64_CS_LOW();
      
      MySPI_SwapByte(W25Q64_SECTOR_ERASE_4KB);    

      MySPI_SwapByte((addr >> 16) & 0xFF);
      MySPI_SwapByte((addr >> 8)  & 0xFF);
      MySPI_SwapByte((addr >> 0)  & 0xFF);

      W25Q64_CS_HIGH();

      W25Q64_WaitBusy();   
  }
