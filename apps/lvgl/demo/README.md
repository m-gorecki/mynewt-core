<!--
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#
-->

# LVGL demo

## Overview

This application allows to run LVGL demos in mynewt environment.
Following LVGL demos can be selected in the syscfg.yml of the target by settings **LVGL_DEMO** variable to:
- benchmark
- flex_layout
- keypad_encoder
- music
- stress
- widgets

#### Several displays are supported:
- ILI9341 (240x320)
- ILI9486 (320x480)
- ST7735S (128x160)

#### Two touch screen controllers:
- XPT2046
- STMPE610

### Example display boards with configuration
- Adafruit 2.8" https://www.adafruit.com/product/1651
suscfg.yml values
```yaml
    LVGL_ILI9341: 1
    LVGL_STMPE610: 1
    ILI9341_SPI_DEV_NAME: '"spi0"'
    ILI9341_CS_PIN: ARDUINO_PIN_D10
    ILI9341_DC_PIN: ARDUINO_PIN_D9
    ILI9341_MCU_INTERFACE: 4_wire_spi
    ILI9341_SPI_FREQ: 16000
    ILI9341_USE_BGR: 1
    STMPE610_SPI_DEV_NAME: '"spi0"'
    STMPE610_SPI_CS_PIN: ARDUINO_PIN_D8
    STMPE610_SPI_FREQ: 1200

```
- Waveshare 3.5" https://www.waveshare.com/3.5inch-tft-touch-shield.htm
  suscfg.yml values
```yaml
    LVGL_ILI9486: 1
    LVGL_XPT2046: 1
    ILI9486_MCU_INTERFACE: 4_wire_spi_with_shift_register
    ILI9486_SPI_DEV_NAME: '"spi0"'
    ILI9486_CS_PIN: ARDUINO_PIN_D10
    ILI9486_DC_PIN: ARDUINO_PIN_D7
    ILI9486_BL_PIN: ARDUINO_PIN_D9
    ILI9486_RESET_PIN: ARDUINO_PIN_D8
    ILI9486_SPI_FREQ: 32000
    XPT2046_SPI_DEV_NAME: '"spi0"'
    XPT2046_SPI_CS_PIN: ARDUINO_PIN_D4
    XPT2046_SPI_FREQ: 1200
    XPT2046_INT_PIN: ARDUINO_PIN_D3
    XPT2046_XY_SWAP: 1
    XPT2046_X_INV: 1
    XPT2046_Y_INV: 1

```

