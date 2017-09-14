/*
 *
 * Copyright (C) 2017 Xavier Paris
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */
/** \file ws2812_led.c
 *  \brief WS2812 RGB led driver
 *
 */

#include "modules/light/ws2812_led.h"
#include "generated/modules.h"
#include "subsystems/actuators/actuators_pwm.h"

#include "subsystems/electrical.h"

/****************************************************************************************
https://github.com/joewa/WS2812-LED-Driver_ChibiOS/
****************************************************************************************/

//uint32_t s=0;

uint8_t clock;

uint8_t maxdecivolt=120;
uint8_t mindecivolt=100;

uint8_t freqratio   = (WS2812_LED_PERIODIC_FREQ/8);
uint8_t freqratio_3 = 3*(WS2812_LED_PERIODIC_FREQ/8);
uint8_t freqratio_4 = 4*(WS2812_LED_PERIODIC_FREQ/8);
uint8_t freqratio_5 = 5*(WS2812_LED_PERIODIC_FREQ/8);

PRINT_CONFIG_VAR(WS2812_LED_PERIODIC_FREQ)
void ws2812_led_init(void)
{
}

void ws2812_led_periodic(void)
{
  bool ledOn=false;
  uint8_t n,color,r,g,b;
  uint16_t current=electrical.vsupply;

  if(clock==WS2812_LED_PERIODIC_FREQ)clock=0;
  else clock++;

  if(current>=maxdecivolt)color=255;
  else if(current<=mindecivolt)color=0;
  else color =(current*255/(maxdecivolt-mindecivolt));

  if(color==0) ledOn=true;
  else { 
    if(clock<freqratio_3) ledOn=true;
    else if(clock<freqratio_4) ledOn=false;
    else if(clock<freqratio_5) ledOn=true;
  }

  if(ledOn) {r=255-color;g=color;b=0;}
  else {r=0;g=0;b=0;}
  for (n = 0; n < WS2812_LED_N; n++)
    ws2812_write_led(n, r,g,b);
}

/*
  uint32_t n,s0;
  for (n = 0; n < WS2812_LED_N; n++) {
    s0 = s + 10*n;
    ws2812_write_led(n, s0%255, (s0+85)%255, (s0+170)%255);
  }
  s+=10;
*/
