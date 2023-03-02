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
#include <bsp/bsp.h>
#include <bus/drivers/spi_common.h>
#include <hal/hal_gpio.h>

#include <lv_glue.h>
#include <hal/lv_hal_disp.h>

#define SHIFT_REGISTER_PRESENT MYNEWT_VAL_CHOICE(ILI9486_MCU_INTERFACE, 4_wire_spi_with_shift_register)

#ifndef LV_COLOR_16_SWAP
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ && \
    (MYNEWT_VAL_CHOICE(ILI9486_MCU_INTERFACE, 4_wire_spi) || SHIFT_REGISTER_PRESENT)
#define LV_COLOR_16_SWAP 1
#else
#define LV_COLOR_16_SWAP 0
#endif
#endif

#define ILI9486_TFTWIDTH    320
#define ILI9486_TFTHEIGHT   480

/* Level 1 Commands -------------- [section] Description */

#define ILI9486_NOP         0x00 /* [8.2.1 ] No Operation / Terminate Frame Memory Write */
#define ILI9486_SWRESET     0x01 /* [8.2.2 ] Software Reset */
#define ILI9486_RDDIDIF     0x04 /* [8.2.3 ] Read Display Identification Information */
#define ILI9486_RDNOE       0x05 /* [8.2.4 ] Read Number of the Errors on DSI */
#define ILI9486_RDDST       0x09 /* [8.2.5 ] Read Display Status */
#define ILI9486_RDDPM       0x0A /* [8.2.6 ] Read Display Power Mode */
#define ILI9486_RDDMADCTL   0x0B /* [8.2.7 ] Read Display MADCTL */
#define ILI9486_RDDCOLMOD   0x0C /* [8.2.8 ] Read Display Pixel Format */
#define ILI9486_RDDIM       0x0D /* [8.2.9 ] Read Display Image Mode */
#define ILI9486_RDDSM       0x0E /* [8.2.10] Read Display Signal Mode */
#define ILI9486_RDDSDR      0x0F /* [8.2.11] Read Display Self-Diagnostic Result */
#define ILI9486_SLPIN       0x10 /* [8.2.12] Enter Sleep Mode */
#define ILI9486_SLPOUT      0x11 /* [8.2.13] Leave Sleep Mode */
#define ILI9486_PTLON       0x12 /* [8.2.14] Partial Display Mode ON */
#define ILI9486_NORON       0x13 /* [8.2.15] Normal Display Mode ON */
#define ILI9486_DINVOFF     0x20 /* [8.2.16] Display Inversion OFF */
#define ILI9486_DINVON      0x21 /* [8.2.17] Display Inversion ON */
#define ILI9486_DISPOFF     0x28 /* [8.2.18] Display OFF */
#define ILI9486_DISPON      0x29 /* [8.2.19] Display ON */
#define ILI9486_CASET       0x2A /* [8.2.20] Column Address Set */
#define ILI9486_PASET       0x2B /* [8.2.21] Page Address Set */
#define ILI9486_RAMWR       0x2C /* [8.2.22] Memory Write */
#define ILI9486_RAMRD       0x2E /* [8.2.23] Memory Read */
#define ILI9486_PTLAR       0x30 /* [8.2.24] Partial Area */
#define ILI9486_VSCRDEF     0x33 /* [8.2.25] Vertical Scrolling Definition */
#define ILI9486_TEOFF       0x34 /* [8.2.26] Tearing Effect Line OFF */
#define ILI9486_TEON        0x35 /* [8.2.27] Tearing Effect Line ON */
#define ILI9486_MADCTL      0x36 /* [8.2.28] Memory Access Control */
#define     MADCTL_MY       0x80 /*          MY row address order */
#define     MADCTL_MX       0x40 /*          MX column address order */
#define     MADCTL_MV       0x20 /*          MV row / column exchange */
#define     MADCTL_ML       0x10 /*          ML vertical refresh order */
#define     MADCTL_MH       0x04 /*          MH horizontal refresh order */
#define     MADCTL_RGB      0x00 /*          RGB Order [default] */
#define     MADCTL_BGR      0x08 /*          BGR Order */
#define ILI9486_VSCRSADD    0x37 /* [8.2.29] Vertical Scrolling Start Address */
#define ILI9486_IDMOFF      0x38 /* [8.2.30] Idle Mode OFF */
#define ILI9486_IDMON       0x39 /* [8.2.31] Idle Mode ON */
#define ILI9486_PIXSET      0x3A /* [8.2.32] Pixel Format Set */
#define ILI9486_WRMEMCONT   0x3C /* [8.2.33] Write Memory Continue */
#define ILI9486_RDMEMCONT   0x3E /* [8.2.34] Read Memory Continue */
#define ILI9486_SETSCANTE   0x44 /* [8.2.35] Set Tear Scanline */
#define ILI9486_GETSCAN     0x45 /* [8.2.36] Get Scanline */
#define ILI9486_WRDISBV     0x51 /* [8.2.37] Write Display Brightness Value */
#define ILI9486_RDDISBV     0x52 /* [8.2.38] Read Display Brightness Value */
#define ILI9486_WRCTRLD     0x53 /* [8.2.39] Write Control Display */
#define ILI9486_RDCTRLD     0x54 /* [8.2.40] Read Control Display */
#define ILI9486_WRCABC      0x55 /* [8.2.41] Write Content Adaptive Brightness Control Value */
#define ILI9486_RDCABC      0x56 /* [8.2.42] Read Content Adaptive Brightness Control Value */
#define ILI9486_WRCABCMIN   0x5E /* [8.2.43] Write CABC Minimum Brightness */
#define ILI9486_RDCABCMIN   0x5F /* [8.2.44] Read CABC Minimum Brightness */
#define ILI9486_RDID1       0xDA /* [8.2.47] Read ID1 - Manufacturer ID (user) */
#define ILI9486_RDID2       0xDB /* [8.2.48] Read ID2 - Module/Driver version (supplier) */
#define ILI9486_RDID3       0xDC /* [8.2.49] Read ID3 - Module/Driver version (user) */

/* Level 2 Commands -------------- [section] Description */

#define ILI9486_IFMODE      0xB0 /* [8.2.50] Interface Mode Control */
#define ILI9486_FRMCTR1     0xB1 /* [8.2.51] Frame Rate Control (In Normal Mode/Full Colors) */
#define ILI9486_FRMCTR2     0xB2 /* [8.2.52] Frame Rate Control (In Idle Mode/8 colors) */
#define ILI9486_FRMCTR3     0xB3 /* [8.2.53] Frame Rate control (In Partial Mode/Full Colors) */
#define ILI9486_INVTR       0xB4 /* [8.2.54] Display Inversion Control */
#define ILI9486_PRCTR       0xB5 /* [8.2.55] Blanking Porch Control */
#define ILI9486_DISCTRL     0xB6 /* [8.2.56] Display Function Control */
#define ILI9486_ETMOD       0xB7 /* [8.2.57] Entry Mode Set */
#define ILI9486_PWCTRL1     0xC0 /* [8.2.58] Power Control 1 - GVDD */
#define ILI9486_PWCTRL2     0xC1 /* [8.2.59] Power Control 2 - step-up factor for operating voltage */
#define ILI9486_PWCTRL3     0xC2 /* [8.2.60] Power Control 3 - for normal mode */
#define ILI9486_PWCTRL4     0xC3 /* [8.2.61] Power Control 4 - for idle mode */
#define ILI9486_PWCTRL5     0xC4 /* [8.2.62] Power Control 5 - for partial mode */
#define ILI9486_VMCTRL      0xC5 /* [8.2.63] VCOM Control */
#define ILI9486_CABCCTRL1   0xC6 /* [8.2.64] VCOM Control */
#define ILI9486_CABCCTRL2   0xC8 /* [8.2.65] VCOM Control */
#define ILI9486_CABCCTRL3   0xC9 /* [8.2.66] VCOM Control */
#define ILI9486_CABCCTRL4   0xCA /* [8.2.67] VCOM Control */
#define ILI9486_CABCCTRL5   0xCB /* [8.2.68] VCOM Control */
#define ILI9486_CABCCTRL6   0xCC /* [8.2.69] VCOM Control */
#define ILI9486_CABCCTRL7   0xCD /* [8.2.70] VCOM Control */
#define ILI9486_CABCCTRL8   0xCE /* [8.2.71] VCOM Control */
#define ILI9486_CABCCTRL9   0xCF /* [8.2.72] VCOM Control */
#define ILI9486_NVMWR       0xD0 /* [8.2.73] NV Memory Write */
#define ILI9486_NVMPKEY     0xD1 /* [8.2.74] NV Memory Protection Key */
#define ILI9486_RDNVM       0xD2 /* [8.2.75] NV Memory Status Read */
#define ILI9486_RDID4       0xD3 /* [8.2.76] Read ID4 - IC Device Code */
#define ILI9486_PGAMCTRL    0xE0 /* [8.2.77] Positive Gamma Control */
#define ILI9486_NGAMCTRL    0xE1 /* [8.2.78] Negative Gamma Correction */
#define ILI9486_DGAMCTRL1   0xE2 /* [8.2.79] Digital Gamma Control 1 */
#define ILI9486_DGAMCTRL2   0xE3 /* [8.2.80] Digital Gamma Control 2 */
#define ILI9486_SPIRCS      0xFB /* [8.2.81] SPI read command settings */

#define ILI9486_HOR_RES       ILI9486_TFTWIDTH
#define ILI9486_VER_RES       ILI9486_TFTHEIGHT

#ifndef ILI9486_CS_PIN_ACTIVE
#define ILI9486_CS_PIN_ACTIVE() ili9486_cs_pin_set(0)
#endif
#ifndef ILI9486_CS_PIN_INACTIVE
#define ILI9486_CS_PIN_INACTIVE() ili9486_cs_pin_set(1)
#endif

#ifndef ILI9486_DC_PIN_DATA
#define ILI9486_DC_PIN_DATA() ili9486_dc_pin_set(1)
#endif

#ifndef ILI9486_DC_PIN_COMMAND
#define ILI9486_DC_PIN_COMMAND() ili9486_dc_pin_set(0)
#endif

#ifndef ILI9486_RESET_PIN_INACTIVE
#define ILI9486_RESET_PIN_INACTIVE() ili9486_reset_pin_set(1)
#endif

#ifndef ILI9486_RESET_PIN_ACTIVE
#define ILI9486_RESET_PIN_ACTIVE() ili9486_reset_pin_set(0)
#endif

#ifndef ILI9486_WRITE_BYTE
#define ILI9486_WRITE_BYTE(buf) ili9486_write_byte(buf)
#endif

#ifndef ILI9486_WRITE_BYTES
#define ILI9486_WRITE_BYTES(buf, count) ili9486_write_bytes(buf, count)
#endif

#ifndef ILI9486_WRITE_COLORS
#define ILI9486_WRITE_COLORS(buf, count) ili9486_write_colors(buf, count)
#endif

void
ili9486_cs_pin_set(int val)
{
    hal_gpio_write(MYNEWT_VAL(ILI9486_CS_PIN), val);
}

void
ili9486_dc_pin_set(int val)
{
    hal_gpio_write(MYNEWT_VAL(ILI9486_DC_PIN), val);
}

void
ili9486_reset_pin_set(int val)
{
    if (MYNEWT_VAL(ILI9486_RESET_PIN) >= 0) {
        hal_gpio_write(MYNEWT_VAL(ILI9486_RESET_PIN), val);
    }
}

#if MYNEWT_VAL_CHOICE(ILI9486_MCU_INTERFACE, 4_wire_spi) || SHIFT_REGISTER_PRESENT
static struct bus_spi_node lcd;
static struct bus_spi_node_cfg lcd_spi_cfg = {
    .node_cfg.bus_name = MYNEWT_VAL(ILI9486_SPI_DEV_NAME),
    .pin_cs = MYNEWT_VAL(ILI9486_CS_PIN),
    .mode = BUS_SPI_MODE_0,
    .data_order = HAL_SPI_MSB_FIRST,
    .freq = MYNEWT_VAL(ILI9486_SPI_FREQ),
};
static struct os_dev *lcd_dev;
#endif

void
ili9486_write_byte(uint8_t val)
{
#if SHIFT_REGISTER_PRESENT
    uint8_t buf[2] = { 0, val };
    bus_node_write(lcd_dev, buf, 2, 1000, BUS_F_NOSTOP);
#elif MYNEWT_VAL_CHOICE(ILI9486_MCU_INTERFACE, 4_wire_spi)
    bus_node_write(lcd_dev, &val, 1, 1000, BUS_F_NOSTOP);
#else
#endif
}

void
ili9486_write_bytes(const uint8_t *buf, int count)
{
#if SHIFT_REGISTER_PRESENT
    uint16_t buf2[count];
    int i;
    for (i = 0; i < count; ++i) {
        buf2[i] = htobe16(buf[i]);
    }
    bus_node_write(lcd_dev, buf2, count * 2, 1000, BUS_F_NOSTOP);
#elif MYNEWT_VAL_CHOICE(ILI9486_MCU_INTERFACE, 4_wire_spi)
    bus_node_write(lcd_dev, buf, count, 1000, BUS_F_NOSTOP);
#else
#endif
}

void
ili9486_write_colors(const lv_color_t *pixels, int count)
{
#if MYNEWT_VAL_CHOICE(ILI9486_MCU_INTERFACE, 8080_8_bit)
    int i;
    for (i = 0; i < count; ++i) {
        ILI9486_WRITE_BYTE(pixels[i].full >> 8);
        ILI9486_WRITE_BYTE((uint8_t)pixels[i].full);
    }
#else
    bus_node_write(lcd_dev, pixels, count * 2, 1000, BUS_F_NOSTOP);
#endif
}

static inline void
ili9486_cmd(uint8_t cmd)
{
    ILI9486_DC_PIN_COMMAND();
    ILI9486_CS_PIN_ACTIVE();
    ILI9486_WRITE_BYTE(cmd);
    ILI9486_CS_PIN_INACTIVE();
}

static inline void
ili9486_cmd_with_args(uint8_t cmd, const uint8_t *args, int count)
{
    ILI9486_DC_PIN_COMMAND();
    ILI9486_CS_PIN_ACTIVE();
    ILI9486_WRITE_BYTE(cmd);
    if (count) {
        ILI9486_DC_PIN_DATA();
        ILI9486_WRITE_BYTES(args, count);
    }
    ILI9486_CS_PIN_INACTIVE();
}

static inline void
ili9486_cmd1(uint8_t cmd, uint8_t arg1)
{
    ili9486_cmd_with_args(cmd, &arg1, 1);
}

static inline void
ili9486_cmd2(uint8_t cmd, uint8_t arg1, uint8_t arg2)
{
    uint8_t args[2] = { arg1, arg2 };
    ili9486_cmd_with_args(cmd, args, 2);
}

static inline void
ili9486_cmd3(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
    uint8_t args[3] = { arg1, arg2, arg3 };
    ili9486_cmd_with_args(cmd, args, 3);
}

static inline void
ili9486_cmd4(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t arg4)
{
    uint8_t args[4] = { arg1, arg2, arg3, arg4 };
    ili9486_cmd_with_args(cmd, args, 4);
}

/**
 * Write byte array
 * @param mode sets command or data mode for write
 * @param data the byte array to write
 * @param len the length of the byte array
 */
static inline void
ili9486_data(const uint8_t *data, uint16_t len)
{
    ILI9486_CS_PIN_ACTIVE();
    ILI9486_DC_PIN_DATA();
    ILI9486_WRITE_BYTES(data, len);
    ILI9486_CS_PIN_INACTIVE();
}

static inline void ili9486_cmd(uint8_t data);
static inline void ili9486_data(const uint8_t *data, uint16_t len);

void
ili9486_rotate(lv_disp_rot_t rotation)
{
    uint8_t madctl = MADCTL_BGR;

    switch (rotation) {
    case LV_DISP_ROT_270:
        madctl |= MADCTL_MV | MADCTL_MY | MADCTL_ML;
        break;
    case LV_DISP_ROT_180:
        madctl |= MADCTL_MX | MADCTL_MY;
        break;
    case LV_DISP_ROT_90:
        madctl |= MADCTL_MX | MADCTL_MV;
        break;
    case LV_DISP_ROT_NONE:
        break;
    }
    ili9486_cmd1(ILI9486_MADCTL, madctl);
}

/**
 * Initialize the ILI9486 display controller
 */
void
ili9486_init(lv_disp_drv_t *driver)
{
    static const uint8_t pgamma_ctrl[] = { 0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98,
                                           0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00};
    static const uint8_t ngamma_ctrl[] = { 0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75,
                                           0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00};

    /* hardware reset */
    ILI9486_CS_PIN_INACTIVE();
    ILI9486_DC_PIN_DATA();
    ILI9486_RESET_PIN_ACTIVE();
    LV_DRV_DELAY_US(10);
    ILI9486_RESET_PIN_INACTIVE();
    LV_DRV_DELAY_MS(5);

    /* software reset */
    ili9486_cmd(ILI9486_SWRESET);
    LV_DRV_DELAY_MS(5);

    /* Sequence from Waveshare arduino */
    ili9486_cmd2(ILI9486_PWCTRL1, 0x19, 0x1A);
    ili9486_cmd2(ILI9486_PWCTRL2, 0x45, 0x00);
    ili9486_cmd1(ILI9486_PWCTRL3, 0x33);
    ili9486_cmd2(ILI9486_VMCTRL, 0x00, 0x28);
    ili9486_cmd2(ILI9486_FRMCTR1, 0xA0, 0x11);
    ili9486_cmd1(ILI9486_INVTR, 0x02);
    ili9486_cmd3(ILI9486_DISCTRL, 0x00, 0x42, 0x3B);

    /* positive gamma correction */
    ili9486_cmd_with_args(ILI9486_PGAMCTRL, pgamma_ctrl, 15);

    /* negative gamma correction */
    ili9486_cmd_with_args(ILI9486_NGAMCTRL, ngamma_ctrl, 15);

    /* 16 bit pixel */
    ili9486_cmd1(ILI9486_PIXSET, 0x55);

    ili9486_cmd2(ILI9486_DISCTRL, 0x00, 0x22);

    /* set orientation */
    ili9486_rotate(LV_DISP_ROT_NONE);

    ili9486_cmd(ILI9486_SLPOUT);

    LV_DRV_DELAY_MS(100);

    /* display on */
    ili9486_cmd(ILI9486_DISPON);
}

static void
ili9486_drv_update(struct _lv_disp_drv_t *drv)
{
    ili9486_rotate(drv->rotated);
}

void
ili9486_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    int32_t y;
    lv_coord_t w;

    if (area->x2 < 0 || area->y2 < 0 || area->x1 >= ILI9486_HOR_RES || area->y1 >= ILI9486_VER_RES) {
        lv_disp_flush_ready(drv);
        return;
    }

    /* Truncate the area to the screen */
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 >= ILI9486_HOR_RES ? ILI9486_HOR_RES - 1 : area->x2;
    int32_t act_y2 = area->y2 >= ILI9486_VER_RES ? ILI9486_VER_RES - 1 : area->y2;

    w = lv_area_get_width(area);

    /* Column address */
    ili9486_cmd4(ILI9486_CASET, act_x1 >> 8, (uint8_t)act_x1, act_x2 >> 8, (uint8_t)act_x2);

    /* Page address */
    ili9486_cmd4(ILI9486_PASET, act_y1 >> 8, (uint8_t)act_y1, act_y2 >> 8, (uint8_t)act_y2);

    ILI9486_DC_PIN_COMMAND();
    ILI9486_CS_PIN_ACTIVE();
    ILI9486_WRITE_BYTE(ILI9486_RAMWR);
    ILI9486_DC_PIN_DATA();

    for (y = act_y1; y <= act_y2; y++) {
        ILI9486_WRITE_COLORS(color_p, w);
        color_p += w;
    }
    ILI9486_CS_PIN_INACTIVE();

    lv_disp_flush_ready(drv);
}

void
mynewt_lv_drv_init(lv_disp_drv_t *driver)
{
    int rc;
    struct bus_node_callbacks cbs = {};

    hal_gpio_init_out(MYNEWT_VAL(ILI9486_CS_PIN), 1);
    hal_gpio_init_out(MYNEWT_VAL(ILI9486_DC_PIN), 0);
    hal_gpio_init_out(MYNEWT_VAL(ILI9486_BL_PIN), 1);
    if (MYNEWT_VAL(ILI9486_RESET_PIN) >= 0) {
        hal_gpio_init_out(MYNEWT_VAL(ILI9486_RESET_PIN), 1);
    }
#if LV_COLOR_16_SWAP
    driver->color_format = LV_COLOR_FORMAT_NATIVE_REVERSE;
#endif
#if MYNEWT_VAL_CHOICE(ILI9486_MCU_INTERFACE, 4_wire_spi) || SHIFT_REGISTER_PRESENT

    bus_node_set_callbacks((struct os_dev *)&lcd, &cbs);

    rc = bus_spi_node_create("ili9486", &lcd, &lcd_spi_cfg, NULL);
    assert(rc == 0);
    lcd_dev = os_dev_open("ili9486", 0, NULL);

#elif MYNEWT_VAL_CHOICE(ILI9486_MCU_INTERFACE, 8080_8_bit)
#else
#error Unsupported/unset ILI9486 MCU interface
#endif
    driver->flush_cb = ili9486_flush;
    driver->drv_update_cb = ili9486_drv_update;
    driver->hor_res = ILI9486_TFTWIDTH;
    driver->ver_res = ILI9486_TFTHEIGHT;

    ili9486_init(driver);
}
