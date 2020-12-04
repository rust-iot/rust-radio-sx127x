MEMORY 
{
  /* Define memory regions. STM32L100RCT6 */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 16K
}
