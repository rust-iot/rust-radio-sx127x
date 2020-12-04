MEMORY
{
  /* Define memory regions for STM32F030 */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 16K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 4K
}
/* see  https://github.com/stm32-rs/stm32f0xx-hal/memory.x  */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
