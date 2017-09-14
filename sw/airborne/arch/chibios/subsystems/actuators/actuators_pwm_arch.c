/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 */
/**
 * @file arch/chibios/subsystems/actuators/actuators_pwm_arch.c
 * Interface from actuators to ChibiOS PWM driver
 *
 * PWM configuration files are defined in the board file,
 * so maximal architecture independence is ensured.
 */
#include "subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/actuators/actuators_pwm.h"
#include "mcu_periph/gpio.h"


/**
 * CMD_TO_US() is depending on architecture (e.g STM32 vs LPC),
 * and on the hardware settings (clock speed etc.). Hence it has to be
 * defined separately for each board.
 *
 * It converts the actuator command from paparazzi. which is in pulse width
 * in milliseconds to microseconds (required by pwmEnableChannel())
 */
#ifndef PWM_CMD_TO_US
#define PWM_CMD_TO_US(_t) (1000000 * _t / PWM_FREQUENCY)
#endif

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];


/**
 * Configuration and interface for WS2812 protocol
 */
#ifdef WS2812_PORT
#define WS2812_PWM_FREQUENCY (STM32_SYSCLK/2)
#define WS2812_SERVO_HZ 800000
#define PWM_CONF_DEF_WS2812 { \
  WS2812_PWM_FREQUENCY, \
  WS2812_PWM_FREQUENCY/WS2812_SERVO_HZ, \
  NULL, \
  { \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
    { PWM_OUTPUT_DISABLED, NULL }, \
  }, \
  0, \
  TIM_DIER_UDE \
}
#define WS2812_RESET_BIT_N	(50)
#define WS2812_COLOR_BIT_N	(WS2812_LED_N*24)                           
#define WS2812_BIT_N		(WS2812_COLOR_BIT_N + WS2812_RESET_BIT_N)   
#define WS2812_DUTYCYCLE_0 	(WS2812_PWM_FREQUENCY/(1000000000/350))
#define WS2812_DUTYCYCLE_1 	(WS2812_PWM_FREQUENCY/(1000000000/800))
static uint32_t ws2812_frame_buffer[WS2812_BIT_N + 1];

#define CAT(x, y, z) PRIMITIVE_CAT(x, y, z)
#define PRIMITIVE_CAT(x, y, z) x ## y ## z

#define WS2812_DRIVER 	CAT(PWM_,WS2812_PORT,_DRIVER)
#define WS2812_TIMER_NB	CAT(PWM_,WS2812_PORT,_TIMER_NB)
#define WS2812_TIM_CH 	CAT(PWM_,WS2812_PORT,_CHANNEL)

#define WS2812_BIT(led, byte, bit) (24*(led) + 8*(byte) + (7 - (bit)))
#define WS2812_RED_BIT(led, bit)   WS2812_BIT((led), 1, (bit))
#define WS2812_GREEN_BIT(led, bit) WS2812_BIT((led), 0, (bit))
#define WS2812_BLUE_BIT(led, bit)  WS2812_BIT((led), 2, (bit))
void ws2812_write_led(uint32_t led_number, uint8_t r, uint8_t g, uint8_t b) 
{
  uint32_t bit;
  if (led_number < WS2812_LED_N) { 
    for (bit = 0; bit < 8; bit++) {
      ws2812_frame_buffer[WS2812_RED_BIT(led_number, bit)]      = ((r >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
      ws2812_frame_buffer[WS2812_GREEN_BIT(led_number, bit)]    = ((g >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
      ws2812_frame_buffer[WS2812_BLUE_BIT(led_number, bit)]     = ((b >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
    }
  }
}
#endif 


/**
 * PWM callback function
 *
 * Called after each period. All PWM configurations (from board.h)
 * should reference to this callback. Empty for now, can be used
 * later for fail safe monitoring (i.e. reset counter or something).
 *
 * @param[in] pwmp pointer to a @p PWMDriver object
 */
 __attribute__((unused)) static void pwmpcb(PWMDriver *pwmp __attribute__((unused))) {}

#if PWM_CONF_TIM1
static PWMConfig pwmcfg1 = PWM_CONF1_DEF;
#endif

#if PWM_CONF_TIM2
#if (WS2812_TIMER_NB==2)
static PWMConfig pwmcfg2 = PWM_CONF_DEF_WS2812;
#else
static PWMConfig pwmcfg2 = PWM_CONF2_DEF;
#endif
#endif

#if PWM_CONF_TIM3
#if (WS2812_TIMER_NB==3)
static PWMConfig pwmcfg3 = PWM_CONF_DEF_WS2812;
#else
static PWMConfig pwmcfg3 = PWM_CONF3_DEF;
#endif
#endif

#if PWM_CONF_TIM4
static PWMConfig pwmcfg4 = PWM_CONF4_DEF;
#endif
#if PWM_CONF_TIM5
static PWMConfig pwmcfg5 = PWM_CONF5_DEF;
#endif
#if PWM_CONF_TIM8
static PWMConfig pwmcfg8 = PWM_CONF8_DEF;
#endif
#if PWM_CONF_TIM9
static PWMConfig pwmcfg9 = PWM_CONF9_DEF;
#endif


void actuators_pwm_arch_init(void)
{
  /*----------------
   * Configure GPIO
   *----------------*/
#ifdef PWM_SERVO_0
  gpio_setup_pin_af(PWM_SERVO_0_GPIO, PWM_SERVO_0_PIN, PWM_SERVO_0_AF, true);
#endif
#ifdef PWM_SERVO_1
  gpio_setup_pin_af(PWM_SERVO_1_GPIO, PWM_SERVO_1_PIN, PWM_SERVO_1_AF, true);
#endif
#ifdef PWM_SERVO_2
  gpio_setup_pin_af(PWM_SERVO_2_GPIO, PWM_SERVO_2_PIN, PWM_SERVO_2_AF, true);
#endif
#ifdef PWM_SERVO_3
  gpio_setup_pin_af(PWM_SERVO_3_GPIO, PWM_SERVO_3_PIN, PWM_SERVO_3_AF, true);
#endif
#ifdef PWM_SERVO_4
  gpio_setup_pin_af(PWM_SERVO_4_GPIO, PWM_SERVO_4_PIN, PWM_SERVO_4_AF, true);
#endif
#ifdef PWM_SERVO_5
  gpio_setup_pin_af(PWM_SERVO_5_GPIO, PWM_SERVO_5_PIN, PWM_SERVO_5_AF, true);
#endif
#ifdef PWM_SERVO_6
  gpio_setup_pin_af(PWM_SERVO_6_GPIO, PWM_SERVO_6_PIN, PWM_SERVO_6_AF, true);
#endif
#ifdef PWM_SERVO_7
  gpio_setup_pin_af(PWM_SERVO_7_GPIO, PWM_SERVO_7_PIN, PWM_SERVO_7_AF, true);
#endif
#ifdef PWM_SERVO_8
  gpio_setup_pin_af(PWM_SERVO_8_GPIO, PWM_SERVO_8_PIN, PWM_SERVO_8_AF, true);
#endif
#ifdef PWM_SERVO_9
  gpio_setup_pin_af(PWM_SERVO_9_GPIO, PWM_SERVO_9_PIN, PWM_SERVO_9_AF, true);
#endif
#ifdef PWM_SERVO_10
  gpio_setup_pin_af(PWM_SERVO_10_GPIO, PWM_SERVO_10_PIN, PWM_SERVO_10_AF, true);
#endif
#ifdef PWM_SERVO_11
  gpio_setup_pin_af(PWM_SERVO_11_GPIO, PWM_SERVO_11_PIN, PWM_SERVO_11_AF, true);
#endif

  /*---------------
   * Configure WS2812 PWM and DMA
   *---------------*/
#ifdef WS2812_PORT
  #define WS2812_DMA_STREAM     CAT(STM32_PWM_TIMER,WS2812_TIMER_NB,_DMA_STREAM)
  #define WS2812_DMA_CHANNEL    CAT(STM32_PWM_TIMER,WS2812_TIMER_NB,_DMA_CHANNEL)
  #define WS2812_DMA_PRIORITY   CAT(STM32_PWM_TIMER,WS2812_TIMER_NB,_DMA_PRIORITY)
  uint32_t i;
  for (i = 0; i < WS2812_COLOR_BIT_N; i++) ws2812_frame_buffer[i]                       = WS2812_DUTYCYCLE_0;   // All color bits are zero duty cycle
  for (i = 0; i < WS2812_RESET_BIT_N; i++) ws2812_frame_buffer[i + WS2812_COLOR_BIT_N]  = 0;                    // All reset bits are zero
  dmaStreamAllocate(WS2812_DMA_STREAM, 10, NULL, NULL);
  dmaStreamSetPeripheral(WS2812_DMA_STREAM, &(WS2812_DRIVER.tim->CCR[WS2812_TIM_CH]));
  dmaStreamSetMemory0(WS2812_DMA_STREAM, ws2812_frame_buffer);
  dmaStreamSetTransactionSize(WS2812_DMA_STREAM, WS2812_BIT_N);
  dmaStreamSetMode(WS2812_DMA_STREAM,
    STM32_DMA_CR_CHSEL(WS2812_DMA_CHANNEL) | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD |
					        STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(WS2812_DMA_PRIORITY));
  dmaStreamEnable(WS2812_DMA_STREAM);
#endif
  
  /*---------------
   * Configure PWM
   *---------------*/
#if PWM_CONF_TIM1
  pwmStart(&PWMD1, &pwmcfg1);
#endif

#if PWM_CONF_TIM2
#if (WS2812_TIMER_NB==2)
  pwmcfg2.channels[WS2812_TIM_CH].mode = PWM_OUTPUT_ACTIVE_HIGH;
  pwmStart(&WS2812_DRIVER, &pwmcfg2);
  pwmEnableChannel(&WS2812_DRIVER, WS2812_TIM_CH, 0);
#else
  pwmStart(&PWMD2, &pwmcfg2);
#endif
#endif

#if PWM_CONF_TIM3
#if (WS2812_TIMER_NB==3)
  pwmcfg3.channels[WS2812_TIM_CH].mode = PWM_OUTPUT_ACTIVE_HIGH;
  pwmStart(&WS2812_DRIVER, &pwmcfg3);
  pwmEnableChannel(&WS2812_DRIVER, WS2812_TIM_CH, 0);
#else
  pwmStart(&PWMD3, &pwmcfg3);
#endif
#endif

#if PWM_CONF_TIM4
  pwmStart(&PWMD4, &pwmcfg4);
#endif
#if PWM_CONF_TIM5
  pwmStart(&PWMD5, &pwmcfg5);
#endif
#if PWM_CONF_TIM8
  pwmStart(&PWMD8, &pwmcfg8);
#endif
#if PWM_CONF_TIM9
  pwmStart(&PWMD9, &pwmcfg9);
#endif
}

void actuators_pwm_commit(void)
{
#ifdef PWM_SERVO_0
#if(WS2812_TIMER_NB!=PWM_SERVO_0_TIMER_NB)
  pwmEnableChannel(&PWM_SERVO_0_DRIVER, PWM_SERVO_0_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_0]));
#endif
#endif
#ifdef PWM_SERVO_1
#if(WS2812_TIMER_NB!=PWM_SERVO_1_TIMER_NB)
  pwmEnableChannel(&PWM_SERVO_1_DRIVER, PWM_SERVO_1_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_1]));
#endif
#endif
#ifdef PWM_SERVO_2
#if(WS2812_TIMER_NB!=PWM_SERVO_2_TIMER_NB)
  pwmEnableChannel(&PWM_SERVO_2_DRIVER, PWM_SERVO_2_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_2]));
#endif
#endif
#ifdef PWM_SERVO_3
#if(WS2812_TIMER_NB!=PWM_SERVO_3_TIMER_NB)
  pwmEnableChannel(&PWM_SERVO_3_DRIVER, PWM_SERVO_3_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_3]));
#endif
#endif
#ifdef PWM_SERVO_4
#if(WS2812_TIMER_NB!=PWM_SERVO_4_TIMER_NB)
  pwmEnableChannel(&PWM_SERVO_4_DRIVER, PWM_SERVO_4_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_4]));
#endif
#endif
#ifdef PWM_SERVO_5
#if(WS2812_TIMER_NB!=PWM_SERVO_5_TIMER_NB)
  pwmEnableChannel(&PWM_SERVO_5_DRIVER, PWM_SERVO_5_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_5]));
#endif
#endif
#ifdef PWM_SERVO_6
#if(WS2812_TIMER_NB!=PWM_SERVO_6_TIMER_NB)
  pwmEnableChannel(&PWM_SERVO_6_DRIVER, PWM_SERVO_6_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_6]));
#endif
#endif
#ifdef PWM_SERVO_7
  pwmEnableChannel(&PWM_SERVO_7_DRIVER, PWM_SERVO_7_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_7]));
#endif
#ifdef PWM_SERVO_8
  pwmEnableChannel(&PWM_SERVO_8_DRIVER, PWM_SERVO_8_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_8]));
#endif
#ifdef PWM_SERVO_9
  pwmEnableChannel(&PWM_SERVO_9_DRIVER, PWM_SERVO_9_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_9]));
#endif
#ifdef PWM_SERVO_10
  pwmEnableChannel(&PWM_SERVO_10_DRIVER, PWM_SERVO_10_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_10]));
#endif
#ifdef PWM_SERVO_11
  pwmEnableChannel(&PWM_SERVO_11_DRIVER, PWM_SERVO_11_CHANNEL, PWM_CMD_TO_US(actuators_pwm_values[PWM_SERVO_11]));
#endif
}
