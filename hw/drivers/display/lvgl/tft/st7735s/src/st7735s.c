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

#include <stdio.h>
#include <lv_glue.h>
#include <hal/lv_hal_disp.h>
#include <hal/hal_gpio.h>
#include <bus/drivers/spi_common.h>

#ifndef LV_COLOR_16_SWAP
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ && \
    (MYNEWT_VAL_CHOICE(ST7735S_MCU_INTERFACE, 4_wire_spi))
#define LV_COLOR_16_SWAP 1
#else
#define LV_COLOR_16_SWAP 0
#endif
#endif

#define ST7735S_TFTWIDTH        128
#define ST7735S_TFTHEIGHT       160

#define ST7735S_NOP             0x00
#define ST7735S_SWRESET         0x01
#define ST7735S_RDDID           0x04
#define ST7735S_RDDST           0x09
#define ST7735S_RDDPM           0x0A
#define ST7735S_RDDDMADCTL      0x0B
#define ST7735S_RDDCOLMOD       0x0C
#define ST7735S_RDDIM           0x0D
#define ST7735S_RDDSM           0x0E
#define ST7735S_RDDSDR          0x0F

#define ST7735S_SLPIN           0x10
#define ST7735S_SLPOUT          0x11
#define ST7735S_PTLON           0x12
#define ST7735S_NORON           0x13

#define ST7735S_INVOFF          0x20
#define ST7735S_INVON           0x21
#define ST7735S_GAMSET          0x26
#define ST7735S_DISPOFF         0x28
#define ST7735S_DISPON          0x29
#define ST7735S_CASET           0x2A
#define ST7735S_RASET           0x2B
#define ST7735S_RAMWR           0x2C
#define ST7735S_RAMRD           0x2E

#define ST7735S_PTLAR           0x30
#define ST7735S_SCRLAR          0x33
#define ST7735S_TEOFF           0x34
#define ST7735S_TEON            0x35
#define ST7735S_MADCTL          0x36
#define ST7735S_VSCSAD          0x37
#define ST7735S_IDMOFF          0x38
#define ST7735S_IDMON           0x39
#define ST7735S_COLMOD          0x3A

#define ST7735S_FRMCTR1         0xB1
#define ST7735S_FRMCTR2         0xB2
#define ST7735S_FRMCTR3         0xB3
#define ST7735S_INVCTR          0xB4

#define ST7735S_PWCTR1          0xC0
#define ST7735S_PWCTR2          0xC1
#define ST7735S_PWCTR3          0xC2
#define ST7735S_PWCTR4          0xC3
#define ST7735S_PWCTR5          0xC4
#define ST7735S_VMCTR1          0xC5
#define ST7735S_VMOFCTR         0xC7

#define ST7735S_WRID2           0xD1
#define ST7735S_WRID3           0xD2
#define ST7735S_NVCTR1          0xD9
#define ST7735S_RDID1           0xDA
#define ST7735S_RDID2           0xDB
#define ST7735S_RDID3           0xDC
#define ST7735S_RDID4           0xDD
#define ST7735S_NVFCTR2         0xDE
#define ST7735S_NVFCTR3         0xDF

#define ST7735S_GMCTRP1         0xE0
#define ST7735S_GMCTRN1         0xE1

#define ST7735S_GCV             0xFC

#define ST7735S_MADCTL_MY       0x80
#define ST7735S_MADCTL_MX       0x40
#define ST7735S_MADCTL_MV       0x20
#define ST7735S_MADCTL_ML       0x10
#define ST7735S_MADCTL_RGB      0x00
#define ST7735S_MADCTL_BGR      0x08

#define ST7735S_HOR_RES       ST7735S_TFTWIDTH
#define ST7735S_VER_RES       ST7735S_TFTHEIGHT

#ifndef ST7735S_CS_PIN_ACTIVE
#define ST7735S_CS_PIN_ACTIVE() st7735s_cs_pin_set(0)
#endif
#ifndef ST7735S_CS_PIN_INACTIVE
#define ST7735S_CS_PIN_INACTIVE() st7735s_cs_pin_set(1)
#endif

#ifndef ST7735S_DC_PIN_DATA
#define ST7735S_DC_PIN_DATA() st7735s_dc_pin_set(1)
#endif

#ifndef ST7735S_DC_PIN_COMMAND
#define ST7735S_DC_PIN_COMMAND() st7735s_dc_pin_set(0)
#endif

#ifndef ST7735S_RESET_PIN_INACTIVE
#define ST7735S_RESET_PIN_INACTIVE() st7735s_reset_pin_set(1)
#endif

#ifndef ST7735S_RESET_PIN_ACTIVE
#define ST7735S_RESET_PIN_ACTIVE() st7735s_reset_pin_set(0)
#endif

#ifndef ST7735S_WRITE_BYTE
#define ST7735S_WRITE_BYTE(buf) st7735s_write_byte(buf)
#endif

#ifndef ST7735S_WRITE_BYTES
#define ST7735S_WRITE_BYTES(buf, count) st7735s_write_bytes(buf, count)
#endif

#ifndef ST7735S_WRITE_COLORS
#define ST7735S_WRITE_COLORS(buf, count) st7735s_write_colors(buf, count)
#endif

void
st7735s_cs_pin_set(int val)
{
    hal_gpio_write(MYNEWT_VAL(ST7735S_CS_PIN), val);
}

void
st7735s_dc_pin_set(int val)
{
    hal_gpio_write(MYNEWT_VAL(ST7735S_DC_PIN), val);
}

void
st7735s_reset_pin_set(int val)
{
    if (MYNEWT_VAL(ST7735S_RESET_PIN) >= 0) {
        hal_gpio_write(MYNEWT_VAL(ST7735S_RESET_PIN), val);
    }
}

#if MYNEWT_VAL_CHOICE(ST7735S_MCU_INTERFACE, 4_wire_spi)
static struct bus_spi_node lcd;
static struct bus_spi_node_cfg lcd_spi_cfg = {
    .node_cfg.bus_name = MYNEWT_VAL(ST7735S_SPI_DEV_NAME),
    .pin_cs = MYNEWT_VAL(ST7735S_CS_PIN),
    .mode = BUS_SPI_MODE_0,
    .data_order = HAL_SPI_MSB_FIRST,
    .freq = MYNEWT_VAL(ST7735S_SPI_FREQ),
};
static struct os_dev *lcd_dev;
#endif

void
st7735s_write_byte(uint8_t val)
{
#if  MYNEWT_VAL_CHOICE(ST7735S_MCU_INTERFACE, 4_wire_spi)
    bus_node_write(lcd_dev, &val, 1, 1000, BUS_F_NOSTOP);
#endif
}

void
st7735s_write_bytes(const uint8_t *buf, int count)
{
#if  MYNEWT_VAL_CHOICE(ST7735S_MCU_INTERFACE, 4_wire_spi)
    bus_node_write(lcd_dev, buf, count, 1000, BUS_F_NOSTOP);
#endif
}

void
st7735s_write_colors(const lv_color_t *pixels, int count)
{
#if MYNEWT_VAL_CHOICE(ST7735S_MCU_INTERFACE, 8080_8_bit)
    int i;
    for (i = 0; i < count; ++i) {
        ST7735S_WRITE_BYTE(pixels[i].full >> 8);
        ST7735S_WRITE_BYTE((uint8_t)pixels[i].full);
    }
#else
    bus_node_write(lcd_dev, pixels, count * 2, 1000, BUS_F_NOSTOP);
#endif
}

static inline void
st7735s_cmd(uint8_t cmd)
{
    ST7735S_DC_PIN_COMMAND();
    ST7735S_CS_PIN_ACTIVE();
    ST7735S_WRITE_BYTE(cmd);
    ST7735S_CS_PIN_INACTIVE();
}

static inline void
st7735s_cmd_with_args(uint8_t cmd, const uint8_t *args, int count)
{
    ST7735S_DC_PIN_COMMAND();
    ST7735S_CS_PIN_ACTIVE();
    ST7735S_WRITE_BYTE(cmd);
    if (count) {
        ST7735S_DC_PIN_DATA();
        ST7735S_WRITE_BYTES(args, count);
    }
    ST7735S_CS_PIN_INACTIVE();
}

static inline void
st7735s_cmd1(uint8_t cmd, uint8_t arg1)
{
    st7735s_cmd_with_args(cmd, &arg1, 1);
}

static inline void
st7735s_cmd2(uint8_t cmd, uint8_t arg1, uint8_t arg2)
{
    uint8_t args[2] = { arg1, arg2 };
    st7735s_cmd_with_args(cmd, args, 2);
}

static inline void
st7735s_cmd3(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
    uint8_t args[3] = { arg1, arg2, arg3 };
    st7735s_cmd_with_args(cmd, args, 3);
}

static inline void
st7735s_cmd4(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4)
{
    uint8_t args[4] = { arg1, arg2, arg3, arg4 };
    st7735s_cmd_with_args(cmd, args, 4);
}

/**
 * Write byte array
 * @param mode sets command or data mode for write
 * @param data the byte array to write
 * @param len the length of the byte array
 */
static inline void
st7735s_data(const uint8_t *data, uint16_t len)
{
    ST7735S_CS_PIN_ACTIVE();
    ST7735S_DC_PIN_DATA();
    ST7735S_WRITE_BYTES(data, len);
    ST7735S_CS_PIN_INACTIVE();
}

void
st7735s_rotate(lv_disp_rot_t rotation)
{
    uint8_t madctl = 0;

    switch (rotation) {
    case LV_DISP_ROT_270:
        madctl |= ST7735S_MADCTL_MV | ST7735S_MADCTL_MY | ST7735S_MADCTL_ML;
        break;
    case LV_DISP_ROT_180:
        madctl |= ST7735S_MADCTL_MX | ST7735S_MADCTL_MY;
        break;
    case LV_DISP_ROT_90:
        madctl |= ST7735S_MADCTL_MX | ST7735S_MADCTL_MV;
        break;
    case LV_DISP_ROT_NONE:
        break;
    }
    st7735s_cmd1(ST7735S_MADCTL, madctl);
}

/**
 * Initialize the ST7735S display controller
 */
void
st7735s_init(lv_disp_drv_t *driver)
{
    static const uint8_t pgamma_ctrl[] = { 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
                                           0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10 };
    static const uint8_t ngamma_ctrl[] = { 0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                                           0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10 };
    static const uint8_t frmctr3[] = { 0x01, 0x2C, 0x2D,0x01, 0x2C, 0x2D };

    /* hardware reset */
    ST7735S_CS_PIN_INACTIVE();
    ST7735S_DC_PIN_DATA();
    ST7735S_RESET_PIN_ACTIVE();
    LV_DRV_DELAY_US(10);
    ST7735S_RESET_PIN_INACTIVE();
    LV_DRV_DELAY_MS(5);

    /* software reset */
    st7735s_cmd(ST7735S_SWRESET);
    LV_DRV_DELAY_MS(5);

    st7735s_cmd(ST7735S_SLPOUT);
    LV_DRV_DELAY_MS(5);

    /* Sequence from esp32 driver port */
    st7735s_cmd3(ST7735S_FRMCTR1, 0x01, 0x2C, 0x2D);
    st7735s_cmd3(ST7735S_FRMCTR2, 0x01, 0x2C, 0x2D);
    st7735s_cmd_with_args(ST7735S_FRMCTR2, frmctr3, sizeof(frmctr3));
    st7735s_cmd1(ST7735S_INVCTR, 0x07);
    st7735s_cmd3(ST7735S_PWCTR1, 0xA2,0x02, 0x84);
    st7735s_cmd1(ST7735S_PWCTR2, 0xC5);
    st7735s_cmd2(ST7735S_PWCTR3, 0x0A, 0x00);
    st7735s_cmd2(ST7735S_PWCTR4, 0x8A, 0x2A);
    st7735s_cmd2(ST7735S_PWCTR5, 0x8A, 0xEE);
    st7735s_cmd1(ST7735S_VMCTR1, 0x0E);
#if invert_colors
    st7735s_cmd(ST7735S_INVON);
#else
    st7735s_cmd(ST7735S_INVOFF);
#endif
    st7735s_cmd1(ST7735S_COLMOD, 0x05);
    /* positive gamma correction */
    st7735s_cmd_with_args(ST7735S_GMCTRP1, pgamma_ctrl, sizeof(pgamma_ctrl));

    /* negative gamma correction */
    st7735s_cmd_with_args(ST7735S_GMCTRN1, ngamma_ctrl, sizeof(ngamma_ctrl));
    st7735s_cmd(ST7735S_NORON);
    LV_DRV_DELAY_MS(10);

    /* set orientation */
    st7735s_rotate(LV_DISP_ROT_NONE);

    LV_DRV_DELAY_MS(100);

    /* display on */
    st7735s_cmd(ST7735S_DISPON);
}

static void
st7735s_drv_update(struct _lv_disp_drv_t *drv)
{
    st7735s_rotate(drv->rotated);
}

void
st7735s_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    int32_t y;
    lv_coord_t w;

    if (area->x2 < 0 || area->y2 < 0 || area->x1 >= ST7735S_HOR_RES || area->y1 >= ST7735S_VER_RES) {
        lv_disp_flush_ready(drv);
        return;
    }

    /* Truncate the area to the screen */
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 >= ST7735S_HOR_RES ? ST7735S_HOR_RES - 1 : area->x2;
    int32_t act_y2 = area->y2 >= ST7735S_VER_RES ? ST7735S_VER_RES - 1 : area->y2;

    w = lv_area_get_width(area);

    /* Column address */
    st7735s_cmd4(ST7735S_CASET, act_x1 >> 8, (uint8_t)act_x1, act_x2 >> 8, (uint8_t)act_x2);

    /* Page address */
    st7735s_cmd4(ST7735S_RASET, act_y1 >> 8, (uint8_t)act_y1, act_y2 >> 8, (uint8_t)act_y2);

    ST7735S_DC_PIN_COMMAND();
    ST7735S_CS_PIN_ACTIVE();
    ST7735S_WRITE_BYTE(ST7735S_RAMWR);
    ST7735S_DC_PIN_DATA();

    for (y = act_y1; y <= act_y2; y++) {
        ST7735S_WRITE_COLORS(color_p, w);
        color_p += w;
    }
    ST7735S_CS_PIN_INACTIVE();

    lv_disp_flush_ready(drv);
}

void
mynewt_lv_drv_init(lv_disp_drv_t *driver)
{
    int rc;
    struct bus_node_callbacks cbs = {};

    hal_gpio_init_out(MYNEWT_VAL(ST7735S_CS_PIN), 1);
    hal_gpio_init_out(MYNEWT_VAL(ST7735S_DC_PIN), 0);
    hal_gpio_init_out(MYNEWT_VAL(ST7735S_BL_PIN), 1);
    if (MYNEWT_VAL(ST7735S_RESET_PIN) >= 0) {
        hal_gpio_init_out(MYNEWT_VAL(ST7735S_RESET_PIN), 1);
    }
#if LV_COLOR_16_SWAP
    driver->color_format = LV_COLOR_FORMAT_NATIVE_REVERSE;
#endif
#if MYNEWT_VAL_CHOICE(ST7735S_MCU_INTERFACE, 4_wire_spi) || SHIFT_REGISTER_PRESENT

    bus_node_set_callbacks((struct os_dev *)&lcd, &cbs);

    rc = bus_spi_node_create("st7735s", &lcd, &lcd_spi_cfg, NULL);
    assert(rc == 0);
    lcd_dev = os_dev_open("st7735s", 0, NULL);

#elif MYNEWT_VAL_CHOICE(ST7735S_MCU_INTERFACE, 8080_8_bit)
#else
#error Unsupported/unset ST7735S MCU interface
#endif
    driver->flush_cb = st7735s_flush;
    driver->drv_update_cb = st7735s_drv_update;
    driver->hor_res = ST7735S_TFTWIDTH;
    driver->ver_res = ST7735S_TFTHEIGHT;

    st7735s_init(driver);
}
