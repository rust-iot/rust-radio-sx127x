MEMORY
{
  /* Define memory regions for STM32F303VCT6 */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 40K   
}
