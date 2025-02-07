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

#include <assert.h>
#include <os/mynewt.h>
#include <hal/hal_uart.h>
#include <nrf.h>
#include <nrf_hal.h>
#include <nrfx_uarte.h>

/* There is no UARTE peripheral on nRF51 */
#ifndef NRF51
#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_1) || MYNEWT_VAL(UART_2) || MYNEWT_VAL(UART_3)
#if defined(NRF52832_XXAA)
#define UARTE0_IRQn  UARTE0_UART0_IRQn
#elif defined(NRF52840_XXAA)
#define UARTE0_IRQn  UARTE0_UART0_IRQn
#define UARTE1_IRQn  UARTE1_IRQn
#elif defined(NRF5340_XXAA_APPLICATION) || defined(NRF9160_XXAA)
#define UARTE0_IRQn  SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn
#define UARTE1_IRQn  SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn
#define UARTE2_IRQn  SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn
#define UARTE3_IRQn  SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn
#elif defined(NRF5340_XXAA_NETWORK)
#define UARTE0_IRQn  SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn
#elif defined(NRF54L15_XXAA)
#define NRF_UARTE0   NRF_UARTE20
#define NRF_UARTE1   NRF_UARTE21
#define NRF_UARTE2   NRF_UARTE22
#define UARTE0_IRQn  UARTE20_IRQn
#define UARTE1_IRQn  UARTE21_IRQn
#define UARTE2_IRQn  UARTE22_IRQn
#define nrfx_uarte_0_irq_handler nrfx_uarte_20_irq_handler
#define nrfx_uarte_1_irq_handler nrfx_uarte_21_irq_handler
#define nrfx_uarte_2_irq_handler nrfx_uarte_22_irq_handler
#endif

struct hal_uart {
    uint8_t u_open : 1;
    uint8_t u_rx_stall : 1;
    uint8_t u_rx_buf;
    uint8_t u_tx_buf[8];
    hal_uart_rx_char u_rx_func;
    hal_uart_tx_char u_tx_func;
    hal_uart_tx_done u_tx_done;
    void *u_func_arg;

    int8_t suc_pin_tx;
    int8_t suc_pin_rx;
    int8_t suc_pin_rts;
    int8_t suc_pin_cts;

    nrfx_uarte_t nrfx_uarte;
};


#if MYNEWT_VAL(UART_0)
static struct hal_uart uart0;
#endif
#if MYNEWT_VAL(UART_1)
static struct hal_uart uart1;
#endif
#if MYNEWT_VAL(UART_2)
static struct hal_uart uart2;
#endif
#if MYNEWT_VAL(UART_3)
static struct hal_uart uart3;
#endif

static struct hal_uart *
hal_uart_get(int port)
{
    switch (port) {
#if MYNEWT_VAL(UART_0)
    case 0:
        return &uart0;
#endif
#if MYNEWT_VAL(UART_1)
    case 1:
        return &uart1;
#endif
#if MYNEWT_VAL(UART_2)
    case 2:
        return &uart2;
#endif
#if MYNEWT_VAL(UART_3)
    case 3:
        return &uart3;
#endif
    default:
        return NULL;
    }
}

static void
hal_uart_set_irqh(int port)
{
    IRQn_Type irqn;
    uint32_t irqh;

    switch (port) {
#if MYNEWT_VAL(UART_0)
    case 0:
        irqn = UARTE0_IRQn;
        irqh = (uint32_t) nrfx_uarte_0_irq_handler;
        break;
#endif

#if MYNEWT_VAL(UART_1)
        case 1:
        irqn = UARTE1_IRQn;
        irqh = (uint32_t) nrfx_uarte_1_irq_handler;
        break;
#endif

#if MYNEWT_VAL(UART_2)
        case 2:
        irqn = UARTE2_IRQn;
        irqh = (uint32_t) nrfx_uarte_2_irq_handler;
        break;
#endif

#if MYNEWT_VAL(UART_3)
        case 3:
        irqn = UARTE3_IRQn;
        irqh = (uint32_t) nrfx_uarte_3_irq_handler;
        break;
#endif
    default:
        assert(0);
    }

    NVIC_SetVector(irqn, irqh);
}

int
hal_uart_init_cbs(int port, hal_uart_tx_char tx_func, hal_uart_tx_done tx_done,
                  hal_uart_rx_char rx_func, void *arg)
{
    struct hal_uart *u = hal_uart_get(port);

    if (!u || u->u_open) {
        return -1;
    }
    u->u_rx_func = rx_func;
    u->u_tx_func = tx_func;
    u->u_tx_done = tx_done;
    u->u_func_arg = arg;

    return 0;
}

static int
hal_uart_tx_fill_buf(struct hal_uart *u)
{
    int data;
    int i;

    for (i = 0; i < sizeof(u->u_tx_buf); i++) {
        data = u->u_tx_func(u->u_func_arg);
        if (data < 0) {
            break;
        }
        u->u_tx_buf[i] = data;
    }
    return i;
}

void
hal_uart_start_tx(int port)
{
    struct hal_uart *u = hal_uart_get(port);
    int rc;

    if (!u) {
        return;
    }

    if (!nrfx_uarte_tx_in_progress(&u->nrfx_uarte)) {
        rc = hal_uart_tx_fill_buf(u);
        if (rc > 0) {
            nrfx_uarte_tx(&u->nrfx_uarte, u->u_tx_buf, rc, 0);
        }
    }
}

void
hal_uart_start_rx(int port)
{
    struct hal_uart *u = hal_uart_get(port);
    int sr;
    int rc;

    if (!u) {
        return;
    }

    if (u->u_rx_stall) {
        __HAL_DISABLE_INTERRUPTS(sr);
        rc = u->u_rx_func(u->u_func_arg, u->u_rx_buf);
        if (rc == 0) {
            u->u_rx_stall = 0;
            nrfx_uarte_rx(&u->nrfx_uarte, &u->u_rx_buf, sizeof(u->u_rx_buf));
        }

        __HAL_ENABLE_INTERRUPTS(sr);
    }
}

void
hal_uart_blocking_tx(int port, uint8_t data)
{
    struct hal_uart *u = hal_uart_get(port);

    if (!u || !u->u_open) {
        return;
    }

    /* If we have started, wait until the current uart dma buffer is done */
    while (nrfx_uarte_tx_in_progress(&u->nrfx_uarte)) {
    }
    nrfx_uarte_tx(&u->nrfx_uarte, &data, 1, NRFX_UARTE_TX_BLOCKING);
}

void hal_uart_handler(nrfx_uarte_event_t const *p_event, void *p_context)
{
    struct hal_uart *u = p_context;
    int rc;

    if (p_event->type == NRFX_UARTE_EVT_TX_DONE) {
        rc = hal_uart_tx_fill_buf(u);
        if (rc > 0) {
            nrfx_uarte_tx(&u->nrfx_uarte, u->u_tx_buf, rc, 0);
        } else {
            if (u->u_tx_done) {
                u->u_tx_done(u->u_func_arg);
            }
        }
    }
    if (p_event->type == NRFX_UARTE_EVT_RX_DONE) {
        rc = u->u_rx_func(u->u_func_arg, u->u_rx_buf);
        if (rc < 0) {
            u->u_rx_stall = 1;
        } else {
            nrfx_uarte_rx(&u->nrfx_uarte, &u->u_rx_buf, sizeof(u->u_rx_buf));
        }
    }
}

static nrf_uarte_baudrate_t
hal_uart_baudrate(int baudrate)
{
    switch (baudrate) {
    case 1200:
        return NRF_UARTE_BAUDRATE_1200;
    case 2400:
        return NRF_UARTE_BAUDRATE_2400;
    case 4800:
        return NRF_UARTE_BAUDRATE_4800;
    case 9600:
        return NRF_UARTE_BAUDRATE_9600;
    case 14400:
        return NRF_UARTE_BAUDRATE_14400;
    case 19200:
        return NRF_UARTE_BAUDRATE_19200;
    case 28800:
        return NRF_UARTE_BAUDRATE_28800;
    case 38400:
        return NRF_UARTE_BAUDRATE_38400;
    case 56000:
        return NRF_UARTE_BAUDRATE_56000;
    case 57600:
        return NRF_UARTE_BAUDRATE_57600;
    case 76800:
        return NRF_UARTE_BAUDRATE_76800;
    case 115200:
        return NRF_UARTE_BAUDRATE_115200;
    case 230400:
        return NRF_UARTE_BAUDRATE_230400;
    case 250000:
        return NRF_UARTE_BAUDRATE_250000;
    case 460800:
        return NRF_UARTE_BAUDRATE_460800;
    case 921600:
        return NRF_UARTE_BAUDRATE_921600;
    case 1000000:
        return NRF_UARTE_BAUDRATE_1000000;
    default:
        return 0;
    }
}

int
hal_uart_init(int port, void *arg)
{
    struct hal_uart *u = hal_uart_get(port);
    struct nrf_uart_cfg *cfg = arg;

    if (!u) {
        return -1;
    }

    switch (port) {
#if MYNEWT_VAL(UART_0)
    case 0:
        u->nrfx_uarte.p_reg = NRF_UARTE0;
        break;
#endif
#if MYNEWT_VAL(UART_1)
    case 1:
        u->nrfx_uarte.p_reg = NRF_UARTE1;
        break;
#endif
#if MYNEWT_VAL(UART_2)
    case 2:
        u->nrfx_uarte.p_reg = NRF_UARTE2;
        break;
#endif
#if MYNEWT_VAL(UART_3)
        case 3:
        u->nrfx_uarte.p_reg = NRF_UARTE3;
        break;
#endif
    default:
        assert(false);
    }

    u->suc_pin_tx = cfg->suc_pin_tx;
    u->suc_pin_rx = cfg->suc_pin_rx;
    u->suc_pin_rts = cfg->suc_pin_rts;
    u->suc_pin_cts = cfg->suc_pin_cts;

    return 0;
}

int
hal_uart_config(int port, int32_t baudrate, uint8_t databits, uint8_t stopbits,
                enum hal_uart_parity parity, enum hal_uart_flow_ctl flow_ctl)
{
    struct hal_uart *u = hal_uart_get(port);
    nrfx_uarte_config_t nrfx_uart_config;
    int rc;

    if (!u || u->u_open) {
        return -1;
    }

    if (databits != 8) {
        return -1;
    }
    if (stopbits != 1) {
        return -1;
    }

    memset(&nrfx_uart_config, 0, sizeof(nrfx_uart_config));

    nrfx_uart_config.baudrate = hal_uart_baudrate(baudrate);
    if (nrfx_uart_config.baudrate == 0) {
        return -1;
    }
#if defined(UARTE_CONFIG_STOP_Msk)
    nrfx_uart_config.config.stop = NRF_UARTE_STOP_ONE;
#endif

    nrfx_uart_config.txd_pin = (uint32_t) u->suc_pin_tx;
    nrfx_uart_config.rxd_pin = (uint32_t) u->suc_pin_rx;

    switch (parity) {
    case HAL_UART_PARITY_NONE:
        nrfx_uart_config.config.parity = NRF_UARTE_PARITY_EXCLUDED;
        break;
    case HAL_UART_PARITY_ODD:
        nrfx_uart_config.config.parity = NRF_UARTE_PARITY_INCLUDED;
#if defined(UARTE_CONFIG_PARITYTYPE_Msk)
        nrfx_uart_config.config.paritytype = NRF_UARTE_PARITYTYPE_ODD;
#endif
        break;
    case HAL_UART_PARITY_EVEN:
        nrfx_uart_config.config.parity = NRF_UARTE_PARITY_INCLUDED;
#if defined(UARTE_CONFIG_PARITYTYPE_Msk)
        nrfx_uart_config.config.paritytype = NRF_UARTE_PARITYTYPE_EVEN;
#endif
        break;
    }

    switch (flow_ctl) {
    case HAL_UART_FLOW_CTL_NONE:
        nrfx_uart_config.config.hwfc = NRF_UARTE_HWFC_DISABLED;
        break;
    case HAL_UART_FLOW_CTL_RTS_CTS:
        nrfx_uart_config.config.hwfc = NRF_UARTE_HWFC_ENABLED;
        nrfx_uart_config.rts_pin = u->suc_pin_rts;
        nrfx_uart_config.cts_pin = u->suc_pin_cts;
        break;
    }

    hal_uart_set_irqh(port);

    nrfx_uart_config.p_context = u;
    rc = nrfx_uarte_init(&u->nrfx_uarte, &nrfx_uart_config, hal_uart_handler);
    if (rc != NRFX_SUCCESS) {
        return -1;
    }

    rc = nrfx_uarte_rx_buffer_set(&u->nrfx_uarte, &u->u_rx_buf, sizeof(u->u_rx_buf));
    if (rc != NRFX_SUCCESS) {
        return -1;
    }

    rc = nrfx_uarte_rx_enable(&u->nrfx_uarte, 0);
    if (rc != NRFX_SUCCESS) {
        return -1;
    }

    u->u_open = 1;

    return 0;
}

int
hal_uart_close(int port)
{
    struct hal_uart *u = hal_uart_get(port);

    if (!u) {
        return -1;
    }

    u->u_open = 0;
    while (nrfx_uarte_tx_in_progress(&u->nrfx_uarte)) {
        /* Wait here until the dma is finished */
    }
    nrfx_uarte_uninit(&u->nrfx_uarte);

    return 0;
}

#endif
#endif
