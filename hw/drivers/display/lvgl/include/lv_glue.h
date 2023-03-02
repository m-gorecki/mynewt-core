/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef LV_GLUE_H
#define LV_GLUE_H

#include <stdint.h>
#include <hal/lv_hal_disp.h>

void lv_touch_handler(void);
void mynewt_lv_drv_init(lv_disp_drv_t *driver);

void mynewt_lv_drv_delay_us(uint32_t us);
void mynewt_lv_drv_delay_ms(uint32_t us);

#define LV_DRV_DELAY_US(us)  mynewt_lv_drv_delay_us(us)
#define LV_DRV_DELAY_MS(ms)  mynewt_lv_drv_delay_ms(ms)

#endif /* LV_GLUE_H */
