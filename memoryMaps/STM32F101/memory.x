MEMORY
{
  /* Define memory regions for STM32F103C8T6 */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 20K
}
