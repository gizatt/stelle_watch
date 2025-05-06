#pragma once
#include <math.h>

// courtesy of
// https://stackoverflow.com/questions/18937701/combining-two-16-bits-rgb-colors-with-alpha-blending
#define MASK_RB 63519       // 0b1111100000011111
#define MASK_G 2016         // 0b0000011111100000
#define MASK_B 0x1F
#define MASK_R 0xF800
#define MASK_MUL_RB 4065216 // 0b1111100000011111000000
#define MASK_MUL_G 129024   // 0b0000011111100000000000
#define MAX_ALPHA 64        // 6bits+1 with rounding

uint16_t alphablend(uint16_t fg, uint16_t bg, uint8_t alpha, uint8_t transmissivity_factor) {

  // alpha for foreground multiplication
  // convert from 8bit to (6bit+1) with rounding
  // will be in [0..64] inclusive
  alpha = (alpha + 2) >> 2;
  // "beta" for background multiplication; (6bit+1);
  // will be in [0..64] inclusive
  uint8_t beta = (MAX_ALPHA - alpha * transmissivity_factor);
  // so (0..64)*alpha + (0..64)*beta always in 0..64

  return (uint16_t)((((alpha * (uint32_t)(fg & MASK_RB) +
                       beta * (uint32_t)(bg & MASK_RB)) &
                      MASK_MUL_RB) |
                     ((alpha * (fg & MASK_G) + beta * (bg & MASK_G)) &
                      MASK_MUL_G)) >>
                    6);
}