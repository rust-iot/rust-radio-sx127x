MEMORY 
{
  /* Define memory regions.  STM32F411RE */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
}
