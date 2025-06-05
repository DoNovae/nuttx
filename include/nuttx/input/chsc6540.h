/****************************************************************************
 * include/nuttx/input/cypress_chsc6540.h
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

#ifndef __INCLUDE_NUTTX_INPUT_M5TOUGH_CHSC6540_H
#define __INCLUDE_NUTTX_INPUT_M5TOUGH_CHSC6540_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <stdbool.h>
#include <stdint.h>

#define CHSC6540_I2C_ADDR 0x2E
#define CHSC6540_I2C_FREQ 400000
#define CHSC6540_W 320
#define CHSC6540_H 280
#define CHSC6540_OULEN SIZEOF_TOUCH_SAMPLE_S(1)


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Device registration */

int chsc6540_register(FAR const char *devpath,FAR struct i2c_master_s *i2c_dev,uint8_t i2c_devaddr, int16_t irq_i16);

#endif /* __INCLUDE_NUTTX_INPUT_M5TOUGH_CHSC6540_H */
