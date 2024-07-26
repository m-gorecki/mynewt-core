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

#include <string.h>
#include <errno.h>
#include <assert.h>
#include <os/mynewt.h>
#include <hal/hal_spi.h>
#include <mcu/nrf5340_hal.h>
#include <nrf.h>
#include <nrfx_config.h>
#include <nrfx_common.h>
#include <nrfx_spis.h>
#include <nrfx_spim.h>

//TODO: ifdef dla innych nrfow
#define SPIM_TXD_MAXCNT_MAX 0xffff

/* The maximum number of SPI interfaces we will allow */
#define NRF5340_HAL_SPI_MAX (5)

/*
 *  Slave states
 *
 *  IDLE: Slave not ready to be used. If master attempts to access
 *        slave it will receive the default character
 *  ACQ_SEM: Slave is attempting to acquire semaphore.
 *  READY: Slave is ready for master to send it data
 *
 */
#define HAL_SPI_SLAVE_STATE_IDLE        (0)
#define HAL_SPI_SLAVE_STATE_ACQ_SEM     (1)
#define HAL_SPI_SLAVE_STATE_READY       (2)

struct txrx_cb_data {
    hal_spi_txrx_cb func;
    void            *arg;
};

struct nrf5340_hal_spi
{
    uint8_t spi_type;
    uint8_t slave_state;    /* Slave only */
    uint8_t def_tx_val;

    int8_t sck_pin;
    int8_t mosi_pin;
    int8_t miso_pin;
    int8_t ss_pin;

    union {
        nrfx_spim_t spim;
        nrfx_spis_t spis;
    } nrfx_spi;

    /* Callback and arguments */
    struct txrx_cb_data cb_data;
};

#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE)
struct nrf5340_hal_spi nrf5340_hal_spi0;
#endif
#if MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_1_SLAVE)
struct nrf5340_hal_spi nrf5340_hal_spi1;
#endif
#if MYNEWT_VAL(SPI_2_MASTER) || MYNEWT_VAL(SPI_2_SLAVE)
struct nrf5340_hal_spi nrf5340_hal_spi2;
#endif
#if MYNEWT_VAL(SPI_3_MASTER) || MYNEWT_VAL(SPI_3_SLAVE)
struct nrf5340_hal_spi nrf5340_hal_spi3;
#endif
#if MYNEWT_VAL(SPI_4_MASTER)
struct nrf5340_hal_spi nrf5340_hal_spi4;
#endif

static const struct nrf5340_hal_spi *nrf5340_hal_spis[NRF5340_HAL_SPI_MAX] = {
#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE)
    &nrf5340_hal_spi0,
#else
    NULL,
#endif
#if MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_1_SLAVE)
    &nrf5340_hal_spi1,
#else
    NULL,
#endif
#if MYNEWT_VAL(SPI_2_MASTER) || MYNEWT_VAL(SPI_2_SLAVE)
    &nrf5340_hal_spi2,
#else
    NULL,
#endif
#if MYNEWT_VAL(SPI_3_MASTER) || MYNEWT_VAL(SPI_3_SLAVE)
    &nrf5340_hal_spi3,
#else
    NULL,
#endif
#if MYNEWT_VAL(SPI_4_MASTER)
    &nrf5340_hal_spi4,
#else
    NULL,
#endif
};

#define NRF5340_HAL_SPI_RESOLVE(__n, __v)                     \
    if ((__n) >= NRF5340_HAL_SPI_MAX) {                       \
        rc = EINVAL;                                        \
        goto err;                                           \
    }                                                       \
    (__v) = (struct nrf5340_hal_spi *) nrf5340_hal_spis[(__n)]; \
    if ((__v) == NULL) {                                    \
        rc = EINVAL;                                        \
        goto err;                                           \
    }

static void
hal_spim_nrfx_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
    struct txrx_cb_data *cb_arg = p_context;

    assert(cb_arg != NULL);
    cb_arg->func(cb_arg->arg, p_event->xfer_desc.rx_length);
}

static int
hal_spi_config_master(struct nrf5340_hal_spi *spi,
                      struct hal_spi_settings *settings)
{
    int rc;
    int frequency;
    nrfx_spim_config_t spim_config;

    /* 16 and 32 MHz is only supported on SPI_4_MASTER */
#if defined(SPIM_FREQUENCY_FREQUENCY_M32) && MYNEWT_VAL(SPI_4_MASTER)
    if (settings->baudrate >= 32000 && spim == NRF_SPIM4) {
        frequency = 32000000;
    } else
#endif
#if defined(SPIM_FREQUENCY_FREQUENCY_M16) && MYNEWT_VAL(SPI_4_MASTER)
    if (settings->baudrate >= 16000 && spim == NRF_SPIM4) {
        frequency = 16000000;
    } else
#endif
    if (settings->baudrate >= 8000) {
        frequency = 8000000;
    } else if (settings->baudrate >= 4000) {
        frequency = 4000000;
    } else if (settings->baudrate >= 2000) {
        frequency = 2000000;
    } else if (settings->baudrate >= 1000) {
        frequency = 1000000;
    } else if (settings->baudrate >= 500) {
        frequency = 500000;
    } else if (settings->baudrate >= 250) {
        frequency = 250000;
    } else if (settings->baudrate >= 125) {
        frequency = 125000;
    } else {
        frequency = 0;
        return EINVAL;
    }

    if (settings->word_size != HAL_SPI_WORD_SIZE_8BIT) {
        return EINVAL;
    }

    spim_config.miso_pin = spi->miso_pin;
    spim_config.mosi_pin = spi->mosi_pin;
    spim_config.ss_pin = spi->ss_pin;
    spim_config.sck_pin = spi->sck_pin;
    spim_config.frequency = frequency;
    spim_config.mode = settings->data_mode;
    spim_config.bit_order = settings->data_order;
    spim_config.miso_pull = NRF_GPIO_PIN_NOPULL;
    spim_config.irq_priority = (1 << __NVIC_PRIO_BITS) - 1;
    spim_config.skip_psel_cfg = false;
    spim_config.skip_gpio_cfg = false;
    spim_config.ss_active_high = false;

    rc = nrfx_spim_init(&spi->nrfx_spi.spim, &spim_config, hal_spim_nrfx_handler, &spi->cb_data);
    if (rc != NRFX_SUCCESS) {
        return EINVAL;
    }

    return 0;
}

#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_1_SLAVE) || \
    MYNEWT_VAL(SPI_2_SLAVE)
static void
hal_spis_nrfx_handler(nrfx_spis_evt_t const *p_event, void *p_context)
{
    struct txrx_cb_data *cb_arg = p_context;
    int len;

    assert(cb_arg != NULL);
    if (p_event->evt_type == NRFX_SPIS_XFER_DONE) {
        /* Driver only allows setting mutual len for both tx and rx.
         * tx_amount == 0 means that tx_buffer was set to NULL and
         * len value specified while calling hal_spi_txrx_noblock is stored
         * in rx_amount. */
        if (p_event->tx_amount == 0) {
            len = p_event->rx_amount;
        } else {
            len = p_event->tx_amount;
        }

        cb_arg->func(cb_arg->arg, len);
    }
}
#endif

static int
hal_spi_config_slave(struct nrf5340_hal_spi *spi,
                     struct hal_spi_settings *settings)
{
#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_1_SLAVE) || \
    MYNEWT_VAL(SPI_2_SLAVE) || MYNEWT_VAL(SPI_3_SLAVE)
    int rc;
    nrfx_spis_config_t spis_config;

    /* Only 8-bit word sizes supported. */
    if (settings->word_size != HAL_SPI_WORD_SIZE_8BIT) {
        return EINVAL;
    }

    spis_config.miso_pin = spi->miso_pin;
    spis_config.mosi_pin = spi->mosi_pin;
    spis_config.sck_pin = spi->sck_pin;
    spis_config.csn_pin = spi->ss_pin;
    spis_config.mode = settings->data_mode;
    spis_config.bit_order = settings->data_order;
    spis_config.miso_drive = NRF_GPIO_PIN_S0S1;
    spis_config.csn_pullup = NRF_GPIO_PIN_PULLUP;
    spis_config.def = spi->def_tx_val;
    spis_config.orc = spi->def_tx_val;
    spis_config.irq_priority = (1 << __NVIC_PRIO_BITS) - 1;
    spis_config.skip_gpio_cfg = false;
    spis_config.skip_psel_cfg = false;

    rc = nrfx_spis_init(&spi->nrfx_spi.spis, &spis_config, hal_spis_nrfx_handler, &spi->cb_data);
    if (rc != NRFX_SUCCESS) {
        return EINVAL;
    }
    return 0;
#else
    return SYS_ENOTSUP;
#endif
}

/**
 * Initialize the SPI, given by spi_num.
 *
 * @param spi_num The number of the SPI to initialize
 * @param cfg HW/MCU specific configuration,
 *            passed to the underlying implementation, providing extra
 *            configuration.
 * @param spi_type SPI type (master or slave)
 *
 * @return int 0 on success, non-zero error code on failure.
 */
int
hal_spi_init(int spi_num, void *cfg, uint8_t spi_type)
{
    int rc;
    struct nrf5340_hal_spi *spi;
    struct nrf5340_hal_spi_cfg *pins;

    NRF5340_HAL_SPI_RESOLVE(spi_num, spi);

    /* Check for valid arguments */
    rc = EINVAL;
    if (cfg == NULL) {
        goto err;
    }

    if ((spi_type != HAL_SPI_TYPE_MASTER) && (spi_type != HAL_SPI_TYPE_SLAVE)) {
        goto err;
    }

    spi->spi_type  = spi_type;
    if (spi_num == 0) {
#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE)
        if (spi_type == HAL_SPI_TYPE_MASTER) {
#if MYNEWT_VAL(SPI_0_MASTER)
            spi->nrfx_spi.spim.p_reg = NRF_SPIM0_S;
#else
            assert(0);
#endif
        } else {
#if MYNEWT_VAL(SPI_0_SLAVE)

            NVIC_SetVector(nrfx_get_irq_number(NRF_SPIS0_S),
                           (uint32_t)nrfx_spis_0_irq_handler);
            spi->nrfx_spi.spis.p_reg = NRF_SPIS0_S;
#else
            assert(0);
#endif
        }
#else
        goto err;
#endif
    } else if (spi_num == 1) {
#if MYNEWT_VAL(SPI_1_MASTER)  || MYNEWT_VAL(SPI_1_SLAVE)
        spi->irq_num = SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn;
        if (spi_type == HAL_SPI_TYPE_MASTER) {
#if MYNEWT_VAL(SPI_1_MASTER)
            spi->nrfx_spi.spim.p_reg = NRF_SPIM1_S;
#else
            assert(0);
#endif
        } else {
#if MYNEWT_VAL(SPI_1_SLAVE)
            spi->nrfx_spi.spis.p_reg = NRF_SPIS1_S;
#else
            assert(0);
#endif
        }
#else
        goto err;
#endif
    } else if (spi_num == 2) {
#if MYNEWT_VAL(SPI_2_MASTER)  || MYNEWT_VAL(SPI_2_SLAVE)
        spi->irq_num = SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn;
        if (spi_type == HAL_SPI_TYPE_MASTER) {
#if MYNEWT_VAL(SPI_2_MASTER)
            spi->nrfx_spi.spim.p_reg = NRF_SPIM2_S;
#else
            assert(0);
#endif
        } else {
#if MYNEWT_VAL(SPI_2_SLAVE)
            spi->nrfx_spi.spis.p_reg = NRF_SPIS2_S;
#else
            assert(0);
#endif
        }
#else
        goto err;
#endif
    } else if (spi_num == 3) {
#if MYNEWT_VAL(SPI_3_MASTER)  || MYNEWT_VAL(SPI_3_SLAVE)
        if (spi_type == HAL_SPI_TYPE_MASTER) {
#if MYNEWT_VAL(SPI_3_MASTER)
            spi->nrfx_spi.spim.p_reg = NRF_SPIM2_S;
#else
            assert(0);
#endif
        } else {
#if MYNEWT_VAL(SPI_3_SLAVE)
            spi->nrfx_spi.spis.p_reg = NRF_SPIS2_S;
#else
            assert(0);
#endif
        }
#else
        goto err;
#endif
    } else if (spi_num == 4) {
#if MYNEWT_VAL(SPI_4_MASTER)
        if (spi_type == HAL_SPI_TYPE_MASTER) {
            spi->nrfx_spi.spim.p_reg = NRF_SPIM4_S;
        } else {
            assert(0);
        }
#else
        goto err;
#endif
    } else {
        goto err;
    }

    rc = hal_spi_disable(spi_num);
    if (rc) {
        goto err;
    }

    pins = (struct nrf5340_hal_spi_cfg *)cfg;
    spi->sck_pin = pins->sck_pin;
    spi->ss_pin = pins->ss_pin;
    spi->mosi_pin = pins->mosi_pin;
    spi->miso_pin = pins->miso_pin;

err:
    return (rc);
}

int
hal_spi_init_hw(uint8_t spi_num, uint8_t spi_type,
                const struct hal_spi_hw_settings *cfg)
{
    struct nrf5340_hal_spi_cfg hal_cfg;

    hal_cfg.sck_pin = cfg->pin_sck;
    hal_cfg.mosi_pin = cfg->pin_mosi;
    hal_cfg.miso_pin = cfg->pin_miso;
    hal_cfg.ss_pin = cfg->pin_ss;

    return hal_spi_init(spi_num, &hal_cfg, spi_type);
}

/**
 * Configure the spi. Must be called after the spi is initialized (after
 * hal_spi_init is called) and when the spi is disabled (user must call
 * hal_spi_disable if the spi has been enabled through hal_spi_enable prior
 * to calling this function). Can also be used to reconfigure an initialized
 * SPI (assuming it is disabled as described previously).
 *
 * @param spi_num The number of the SPI to configure.
 * @param psettings The settings to configure this SPI with
 *
 * @return int 0 on success, non-zero error code on failure.
 */
int
hal_spi_config(int spi_num, struct hal_spi_settings *settings)
{
    int rc;
    struct nrf5340_hal_spi *spi;

    NRF5340_HAL_SPI_RESOLVE(spi_num, spi);

    if (spi->spi_type == HAL_SPI_TYPE_MASTER) {
        if (nrf_spim_enable_check(spi->nrfx_spi.spim.p_reg)) {
            return -1;
        }
    } else {
        if (nrf_spis_enable_check(spi->nrfx_spi.spis.p_reg)) {
            return -1;
        }
    }

    if (settings->word_size != HAL_SPI_WORD_SIZE_8BIT) {
        return EINVAL;
    }

    if (spi->spi_type  == HAL_SPI_TYPE_MASTER) {
        rc = hal_spi_config_master(spi, settings);
    } else {
        rc = hal_spi_config_slave(spi, settings);
    }

    hal_spi_disable(spi_num);

err:
    return (rc);
}

/**
 * Enables the SPI. This does not start a transmit or receive operation;
 * it is used for power mgmt. Cannot be called when a SPI transfer is in
 * progress.
 *
 * @param spi_num
 *
 * @return int 0 on success, non-zero error code on failure.
 */
int
hal_spi_enable(int spi_num)
{
    int rc;
    NRF_SPIS_Type *spis;
    NRF_SPIM_Type *spim;
    struct nrf5340_hal_spi *spi;

    NRF5340_HAL_SPI_RESOLVE(spi_num, spi);

    if (spi->cb_data.func == NULL) {
        rc = EINVAL;
        goto err;
    }

    if (spi->spi_type  == HAL_SPI_TYPE_MASTER) {
        spim = spi->nrfx_spi.spim.p_reg;
        nrf_spim_event_clear(spim, NRF_SPIM_EVENT_END);
        nrf_spim_enable(spim);
    } else {
        spis = spi->nrfx_spi.spis.p_reg;
        nrf_spis_event_clear(spis, NRF_SPIS_EVENT_END);
        nrf_spis_event_clear(spis, NRF_SPIS_EVENT_ACQUIRED);
        nrf_spis_int_enable(spis, NRF_SPIS_INT_END_MASK | NRF_SPIS_INT_ACQUIRED_MASK);
        nrf_spis_enable(spis);
    }
    rc = 0;

err:
    return rc;
}

/**
 * Disables the SPI. Used for power mgmt. It will halt any current SPI transfers
 * in progress.
 *
 * @param spi_num
 *
 * @return int 0 on success, non-zero error code on failure.
 */
int
hal_spi_disable(int spi_num)
{
    int rc;
    NRF_SPIS_Type *spis;
    struct nrf5340_hal_spi *spi;

    NRF5340_HAL_SPI_RESOLVE(spi_num, spi);

    if (spi->spi_type  == HAL_SPI_TYPE_MASTER) {
        if (nrfx_spim_init_check(&spi->nrfx_spi.spim)) {
            nrfx_spim_abort(&spi->nrfx_spi.spim);
        }
    } else {
        spis = spi->nrfx_spi.spis.p_reg;
        nrf_spis_int_disable(spis, NRF_SPIS_ALL_INTS_MASK);
        nrf_spis_event_clear(spis, NRF_SPIS_EVENT_END);
        nrf_spis_event_clear(spis, NRF_SPIS_EVENT_ACQUIRED);
        nrf_spis_disable(spis);
        spi->slave_state = HAL_SPI_SLAVE_STATE_IDLE;
    }

    rc = 0;

err:
    return rc;
}

/**
 * Blocking call to send a value on the SPI. Returns the value received from the
 * SPI slave.
 *
 * MASTER: Sends the value and returns the received value from the slave.
 * SLAVE: Invalid API. Returns 0xFFFF
 *
 * @param spi_num   Spi interface to use
 * @param val       Value to send
 *
 * @return uint16_t Value received on SPI interface from slave. Returns 0xFFFF
 * if called when the SPI is configured to be a slave
 */
uint16_t hal_spi_tx_val(int spi_num, uint16_t val)
{
    int rc;
    uint16_t txval;
    uint16_t retval;
    struct nrf5340_hal_spi *hal_spi;

    NRF5340_HAL_SPI_RESOLVE(spi_num, hal_spi);

    if (hal_spi->spi_type  == HAL_SPI_TYPE_MASTER) {
        txval = val;
        rc = hal_spi_txrx(spi_num, &txval, &retval, 2);
        if (rc) {
            goto err;
        }
    } else {
        retval = 0xFFFF;
    }

    return retval;

    err:
    return rc;
}

int
hal_spi_txrx(int spi_num, void *txbuf, void *rxbuf, int len)
{
    int rc;
    int nrfx_rxlen;
    int nrfx_txlen;
    nrfx_spim_t *spim;
    struct nrf5340_hal_spi *hal_spi;
    nrfx_spim_xfer_desc_t xfer_desc;

    rc = -1;
    /* Must have txbuf or rxbuf */
    if ((txbuf == NULL) && (rxbuf == NULL)) {
        goto err;
    }

    if ((rxbuf != NULL && !nrfx_is_in_ram(rxbuf)) ||
        (txbuf != NULL && !nrfx_is_in_ram(txbuf))) {
        goto err;
    }

    if ((len == 0) || (len > SPIM_TXD_MAXCNT_MAX)) {
        goto err;
    }

    if (rxbuf == NULL) {
        nrfx_rxlen = 0;
    } else {
        nrfx_rxlen = len;
    }
    if (txbuf == NULL) {
        nrfx_txlen = 0;
    } else {
        nrfx_txlen = len;
    }

    NRF5340_HAL_SPI_RESOLVE(spi_num, hal_spi);

    if (hal_spi->spi_type  == HAL_SPI_TYPE_MASTER) {
        spim = &hal_spi->nrfx_spi.spim;

        xfer_desc.p_tx_buffer = txbuf;
        xfer_desc.p_rx_buffer = rxbuf;
        xfer_desc.rx_length = nrfx_rxlen;
        xfer_desc.tx_length = nrfx_txlen;
        nrfx_spim_xfer(spim, &xfer_desc, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
        while (nrf_spim_event_check(spim->p_reg, NRF_SPIM_EVENT_END)) {};

        return 0;
    }

    err:
    return rc;
}

/**
 * Sets the txrx callback (executed at interrupt context) when the
 * buffer is transferred by the master or the slave using the non-blocking API.
 * Cannot be called when the spi is enabled. This callback will also be called
 * when chip select is de-asserted on the slave.
 *
 * NOTE: This callback is only used for the non-blocking interface and must
 * be called prior to using the non-blocking API.
 *
 * @param spi_num   SPI interface on which to set callback
 * @param txrx      Callback function
 * @param arg       Argument to be passed to callback function
 *
 * @return int 0 on success, non-zero error code on failure.
 */
int
hal_spi_set_txrx_cb(int spi_num, hal_spi_txrx_cb txrx_cb, void *arg)
{
    int rc;
    struct nrf5340_hal_spi *spi;

    NRF5340_HAL_SPI_RESOLVE(spi_num, spi);

    if (spi->spi_type == HAL_SPI_TYPE_MASTER) {
        if (nrf_spim_enable_check(spi->nrfx_spi.spim.p_reg)) {
            rc = -1;
            goto err;
        }
    } else {
        if (nrf_spis_enable_check(spi->nrfx_spi.spis.p_reg)) {
            rc = -1;
            goto err;
        }
    }

    spi->cb_data.func = txrx_cb;
    spi->cb_data.arg = arg;
    rc = 0;
err:
    return rc;
}

static inline nrfx_err_t
hal_spis_txrx_noblock(nrfx_spis_t *spis, void *txbuf, int txlen,
                      void *rxbuf, int rxlen)
{
#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_1_SLAVE) || \
    MYNEWT_VAL(SPI_2_SLAVE) || MYNEWT_VAL(SPI_3_SLAVE)
    return nrfx_spis_buffers_set(spis, txbuf, txlen, rxbuf, rxlen);
#else
    return ENOTSUP;
#endif
}

static inline nrfx_err_t
hal_spim_txrx_noblock(nrfx_spim_t *spim, void *txbuf, int txlen,
                      void *rxbuf, int rxlen)
{
    nrfx_spim_xfer_desc_t xfer_desc;

#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_1_MASTER) || \
    MYNEWT_VAL(SPI_2_MASTER) || MYNEWT_VAL(SPI_3_MASTER) || MYNEWT_VAL(SPI_4_MASTER)
    xfer_desc.p_tx_buffer = txbuf;
    xfer_desc.p_rx_buffer = rxbuf;
    xfer_desc.rx_length = rxlen;
    xfer_desc.tx_length = txlen;
    return nrfx_spim_xfer(spim, &xfer_desc, 0);
#else
    return ENOTSUP;
#endif
}

/**
 * Non-blocking interface to send a buffer and store received values. Can be
 * used for both master and slave SPI types. The user must configure the
 * callback (using hal_spi_set_txrx_cb); the txrx callback is executed at
 * interrupt context when the buffer is sent.
 *
 * The transmit and receive buffers are either arrays of 8-bit (uint8_t)
 * values or 16-bit values depending on whether the spi is configured for 8 bit
 * data or more than 8 bits per value. The 'cnt' parameter is the number of
 * 8-bit or 16-bit values. Thus, if 'cnt' is 10, txbuf/rxbuf would point to an
 * array of size 10 (in bytes) if the SPI is using 8-bit data; otherwise
 * txbuf/rxbuf would point to an array of size 20 bytes (ten, uint16_t values).
 *
 * NOTE: these buffers are in the native endian-ness of the platform.
 *
 *     MASTER: master sends all the values in the buffer and stores the
 *             stores the values in the receive buffer if rxbuf is not NULL.
 *             The txbuf parameter cannot be NULL
 *     SLAVE: Slave "preloads" the data to be sent to the master (values
 *            stored in txbuf) and places received data from master in rxbuf
 *            (if not NULL). The txrx callback occurs when len values are
 *            transferred or master de-asserts chip select. If txbuf is NULL,
 *            the slave transfers its default byte. Both rxbuf and txbuf cannot
 *            be NULL.
 *
 * @param spi_num   SPI interface to use
 * @param txbuf     Pointer to buffer where values to transmit are stored.
 * @param rxbuf     Pointer to buffer to store values received from peer.
 * @param cnt       Number of 8-bit or 16-bit values to be transferred.
 *
 * @return int 0 on success, non-zero error code on failure.
 */
int
hal_spi_txrx_noblock(int spi_num, void *txbuf, void *rxbuf, int len)
{
    int rc;
    int nrfx_rxlen;
    int nrfx_txlen;
    nrfx_spim_t *spim;
    nrfx_spis_t *spis;
    struct nrf5340_hal_spi *spi;

    rc = EINVAL;
    NRF5340_HAL_SPI_RESOLVE(spi_num, spi);

    /* Must have txbuf or rxbuf */
    if ((txbuf == NULL) && (rxbuf == NULL)) {
        goto err;
    }

    if ((rxbuf != NULL && !nrfx_is_in_ram(rxbuf)) ||
        (txbuf != NULL && !nrfx_is_in_ram(txbuf))) {
        goto err;
    }

    if ((len == 0) || (len > SPIM_TXD_MAXCNT_MAX)) {
        goto err;
    }

    if (spi->cb_data.func == NULL) {
        goto err;
    }

    if (rxbuf == NULL) {
        nrfx_rxlen = 0;
    } else {
        nrfx_rxlen = len;
    }
    if (txbuf == NULL) {
        nrfx_txlen = 0;
    } else {
        nrfx_txlen = len;
    }

    if (spi->spi_type  == HAL_SPI_TYPE_MASTER) {
        spim = &spi->nrfx_spi.spim;

        rc = hal_spim_txrx_noblock(spim, txbuf, nrfx_txlen, rxbuf, nrfx_rxlen);
        if (rc != NRFX_SUCCESS) {
            rc = -1;
            goto err;
        }
    } else {
        /*
         * Ready the slave for a transfer. Do not allow this to be called
         * if the slave has already been readied or is requesting the
         * semaphore
         */
        if (spi->slave_state != HAL_SPI_SLAVE_STATE_IDLE) {
            rc = -1;
            goto err;
        }
        spis = &spi->nrfx_spi.spis;
        rc = hal_spis_txrx_noblock(spis, txbuf, nrfx_txlen, rxbuf, nrfx_rxlen);
        if (rc != NRFX_SUCCESS) {
            rc = -1;
            goto err;
        }
    }
    return 0;

err:
    return rc;
}

/**
 * Sets the default value transferred by the slave. Not valid for master
 *
 * @param spi_num SPI interface to use
 *
 * @return int 0 on success, non-zero error code on failure.
 */
int
hal_spi_slave_set_def_tx_val(int spi_num, uint16_t val)
{
    int rc;
    nrfx_spis_t *spis;
    struct nrf5340_hal_spi *spi;

    NRF5340_HAL_SPI_RESOLVE(spi_num, spi);
    if (spi->spi_type == HAL_SPI_TYPE_SLAVE) {
        spis = &spi->nrfx_spi.spis;
        spi->def_tx_val = (uint8_t)val;
        nrf_spis_orc_set(spis->p_reg, spi->def_tx_val);
        nrf_spis_def_set(spis->p_reg, spi->def_tx_val);
        rc = 0;
    } else {
        rc = EINVAL;
    }

err:
    return rc;
}

/**
 * This aborts the current transfer but keeps the spi enabled.
 *
 * @param spi_num   SPI interface on which transfer should be aborted.
 *
 * @return int 0 on success, non-zero error code on failure.
 *
 * NOTE: does not return an error if no transfer was in progress.
 */
int
hal_spi_abort(int spi_num)
{
    int rc;
    nrfx_spim_t *spim;
    struct nrf5340_hal_spi *spi;

    NRF5340_HAL_SPI_RESOLVE(spi_num, spi);

    rc = 0;
    if (spi->spi_type  == HAL_SPI_TYPE_MASTER) {
        spim = &spi->nrfx_spi.spim;
        nrfx_spim_abort(spim);
    }

err:
    return rc;
}
