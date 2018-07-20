//! SX127x Radio Driver
//! Copyright 2018 Ryan Kurte

#include <stdint.h>
#include <stdbool.h>

#include "radio.h"
#include "sx1276/sx1276.h"

extern void DelayMs(uint32_t ms);
extern void SX1276Reset( SX1276_t* sx1276 );
extern void SX1276WriteBuffer( SX1276_t* sx1276, uint8_t addr, uint8_t *buffer, uint8_t size );
extern void SX1276ReadBuffer( SX1276_t* sx1276, uint8_t addr, uint8_t *buffer, uint8_t size );


