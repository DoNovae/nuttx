/****************************************************************************
 * boards/xtensa/esp32/esp32-wrover-kit/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_XTENSA_ESP32_ESP32_WROVER_KIT_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32_ESP32_WROVER_KIT_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32 Wrover kit board uses a 40MHz crystal oscillator. */

#define BOARD_XTAL_FREQUENCY  40000000

#ifdef CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* GPIO definitions *********************************************************/

/* Display */
// To get CONFIG_ESP32_SPI2 or CONFIG_ESP32_SPI3 configuration
#define DISPLAY_SPI       2
#define DISPLAY_DC        15
#define ILI9341_AXP_I2C_FREQ 400000


/*
 * Touch
 */
#define CHSC6540_INT_PIN 39
#define CHSC6540_I2C_NUM 1


/* GPIO pins used by the GPIO Subsystem */
#define BOARD_NGPIOIN     5 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    5 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    5 /* Amount of GPIO Input w/ Interruption pins */

#endif /* __BOARDS_XTENSA_ESP32_ESP32_WROVER_KIT_INCLUDE_BOARD_H */
