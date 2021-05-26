MEMORY
{
  /* see https://github.com/stm32-rs/stm32h7xx-hal/blob/master/memory.x 
     for much better information
  */

  FLASH  : ORIGIN = 0x08000000, LENGTH = 2M
  RAM    : ORIGIN = 0x20000000, LENGTH = 128K

}
