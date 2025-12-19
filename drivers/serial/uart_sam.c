/*
 * Copyright (c) 2017 Piotr Mienkowski
 * Copyright (c) 2018 Justin Watson
 * Copyright (c) 2023 Gerson Fernando Budke
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/cache.h"
#include <stdint.h>
#define DT_DRV_COMPAT atmel_sam_uart

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/atmel_sam_pmc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#include <zephyr/sys/ring_buffer.h>
#endif

#include <soc.h>

LOG_MODULE_REGISTER(uart_sam, CONFIG_UART_LOG_LEVEL);

/* Device constant configuration parameters */
struct uart_sam_dev_cfg {
    Uart *regs;
    const struct atmel_sam_pmc_config clock_cfg;
    const struct pinctrl_dev_config *pcfg;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_SAM_ASYNC
    uart_irq_config_func_t irq_config_func;
#endif

#ifdef CONFIG_UART_ASYNC_API
    const struct device *dma_dev;
    uint32_t rx_dma_channel;
    uint32_t tx_dma_channel;
    uint32_t rx_dma_request;  /* Peripheral ID for DMA handshaking */
    uint32_t tx_dma_request;
#endif
};

/* RX completion reasons */
enum uart_sam_rx_completion_reason {
    UART_SAM_RX_TIMEOUT,      /* Inter-byte timeout - packet complete */
    UART_SAM_RX_BUFFER_FULL,  /* DMA buffer full */
    UART_SAM_RX_DMA_ERROR,        /* DMA error occurred */
    UART_SAM_RX_SR_ERROR,        /* DMA error occurred */
};

/* RX completion event - self-contained work item */
struct uart_sam_rx_event {
    struct k_work work;                           /* Must be first for CONTAINER_OF */
    const struct device *dev;                     /* Device reference */
    enum uart_sam_rx_completion_reason reason;    /* Why we're completing */
    int error_code;                               /* DMA error code if applicable */
};

/* Device run time data */
struct uart_sam_dev_data {
    const struct device *dev;
    uint32_t baud_rate;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_SAM_ASYNC
    uart_irq_callback_user_data_t irq_cb;
    void *irq_cb_data;
#endif

#ifdef CONFIG_UART_ASYNC_API
    /* RX */
    uint8_t *rx_buf;
    size_t rx_len;
    size_t rx_offset;  /* Current position in buffer */
    uint8_t *rx_next_buf;
    size_t rx_next_len;
    /* RX timeout configuration */
    uint32_t rx_inter_byte_timeout;  // Microseconds of silence = packet done
    size_t rx_last_position;         // Last known DMA position
    /* Work items */
    struct k_work_delayable rx_timeout_work;
    /* Event pool for RX completion events */
    struct uart_sam_rx_event rx_event_pool[3];   /* DMA done, timeout, spare */

    bool rx_enabled;

    /* TX */
    const uint8_t *tx_buf;
    size_t tx_len;
    struct k_work_delayable tx_timeout_work;

    /* Callback */
    uart_callback_t async_cb;
    void *async_cb_data;
#endif
};

/**
 * Notify application of RX data up to the specified position
 * Only notifies data that hasn't been reported yet (beyond rx_offset)
 *
 * @param dev Device instance
 * @param position Absolute position in buffer (total bytes received)
 */
static void uart_sam_notify_rx_data(const struct device *dev, size_t position)
{
    struct uart_sam_dev_data *data = dev->data;

    if (!data->async_cb) {
        return;
    }

    /* Nothing new to report */
    if (data->rx_offset >= position) {
        return;
    }

    size_t new_bytes = position - data->rx_offset;

    struct uart_event evt = {
        .type = UART_RX_RDY,
        .data.rx = {
            .buf = data->rx_buf,
            .offset = data->rx_offset,
            .len = new_bytes,
        },
    };

    data->rx_offset += position;
    if (data->rx_offset + position > data->rx_len) {
        data->rx_offset = 0;
    }
    else {
        data->rx_offset += position;
    }
    data->async_cb(dev, &evt, data->async_cb_data);
}

static int uart_sam_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	Uart * const uart = cfg->regs;

	if (!(uart->UART_SR & UART_SR_RXRDY)) {
		return -1;
	}

	/* got a character */
	*c = (unsigned char)uart->UART_RHR;

	return 0;
}

static void uart_sam_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	Uart * const uart = cfg->regs;

	/* Wait for transmitter to be ready */
	while (!(uart->UART_SR & UART_SR_TXRDY)) {
	}

	/* send a character */
	uart->UART_THR = (uint32_t)c;
}

static int uart_sam_err_check(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;
	int errors = 0;

	if (uart->UART_SR & UART_SR_OVRE) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (uart->UART_SR & UART_SR_PARE) {
		errors |= UART_ERROR_PARITY;
	}

	if (uart->UART_SR & UART_SR_FRAME) {
		errors |= UART_ERROR_FRAMING;
	}

	uart->UART_CR = UART_CR_RSTSTA;

	return errors;
}

static int uart_sam_baudrate_set(const struct device *dev, uint32_t baudrate)
{
	struct uart_sam_dev_data *const dev_data = dev->data;

	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	uint32_t divisor;

	__ASSERT(baudrate,
		 "baud rate has to be bigger than 0");
	__ASSERT(SOC_ATMEL_SAM_MCK_FREQ_HZ/16U >= baudrate,
		 "MCK frequency is too small to set required baud rate");

	divisor = SOC_ATMEL_SAM_MCK_FREQ_HZ / 16U / baudrate;

	if (divisor > 0xFFFF) {
		return -EINVAL;
	}

	uart->UART_BRGR = UART_BRGR_CD(divisor);
	dev_data->baud_rate = baudrate;

	return 0;
}

#if CONFIG_UART_SAM_ASYNC

// DMA callback - triggered when DMA finishes writing to UART_THR
static void uart_sam_dma_tx_done(const struct device *dma_dev, void *arg,
                                 uint32_t id, int error_code)
{
    ARG_UNUSED(dma_dev);
    ARG_UNUSED(id);

    struct uart_sam_dev_data *const dev_data =
        (struct uart_sam_dev_data *const)arg;
    const struct device *dev = dev_data->dev;

    struct uart_event evt = {
        .type = UART_TX_DONE,
        .data.tx = {
            .buf = dev_data->tx_buf,
            .len = dev_data->tx_len,
        },
    };

    if (evt.data.tx.len != 0U && dev_data->async_cb) {
        dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
    }

    dev_data->tx_buf = NULL;
    dev_data->tx_len = 0U;
}

static int uart_sam_tx_halt(struct uart_sam_dev_data *dev_data)
{
    const struct device *dev = dev_data->dev;
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	unsigned int key = irq_lock();
	size_t tx_active = dev_data->tx_len;
	struct dma_status st;

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx = {
			.buf = dev_data->tx_buf,
			.len = 0U,
		},
	};

	dev_data->tx_buf = NULL;
	dev_data->tx_len = 0U;

	dma_stop(cfg->dma_dev, cfg->tx_dma_channel);

	irq_unlock(key);

	if (dma_get_status(cfg->dma_dev, cfg->tx_dma_channel, &st) == 0) {
		evt.data.tx.len = tx_active - st.pending_length;
	}

	if (tx_active) {
		if (dev_data->async_cb) {
			dev_data->async_cb(dev_data->dev,
					   &evt, dev_data->async_cb_data);
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

static void uart_sam_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_sam_dev_data *dev_data = CONTAINER_OF(dwork,
							   struct uart_sam_dev_data, tx_timeout_work);

	uart_sam_tx_halt(dev_data);
}

/* Helper functions for better readability */
static void uart_sam_release_rx_buffer(const struct device *dev, uint8_t *buf)
{
    struct uart_sam_dev_data *data = dev->data;

    if (!data->async_cb) {
        return;
    }

    struct uart_event evt = {
        .type = UART_RX_BUF_RELEASED,
        .data.rx_buf = {
            .buf = buf,
        },
    };

    data->async_cb(dev, &evt, data->async_cb_data);

    /* Clear current buffer state */
    data->rx_buf = NULL;
    data->rx_len = 0;
    data->rx_offset = 0;
}

static void uart_sam_switch_to_next_rx_buffer(struct uart_sam_dev_data *data)
{
    data->rx_buf = data->rx_next_buf;
    data->rx_len = data->rx_next_len;
    data->rx_next_buf = NULL;
    data->rx_next_len = 0U;
    data->rx_offset = 0U;
}

static void uart_sam_request_rx_buffer(const struct device *dev)
{
    struct uart_sam_dev_data *data = dev->data;

    if (data->async_cb) {
        struct uart_event evt = {
            .type = UART_RX_BUF_REQUEST,
        };
        data->async_cb(dev, &evt, data->async_cb_data);
    }
}

static void uart_sam_abort_rx(const struct device *dev, enum uart_rx_stop_reason reason)
{
    struct uart_sam_dev_data *data = dev->data;
    const struct uart_sam_dev_cfg *cfg = dev->config;

    /* Stop DMA */
    dma_stop(cfg->dma_dev, cfg->rx_dma_channel);

    if (data->async_cb) {
        struct uart_event evt = {
            .type = UART_RX_STOPPED,
            .data.rx_stop = {
                .reason = reason,
                .data.len = data->rx_offset,
                .data.buf = data->rx_buf,
            },
        };
        data->async_cb(dev, &evt, data->async_cb_data);
    }

    data->rx_enabled = false;
    data->rx_buf = NULL;
    data->rx_len = 0U;
    data->rx_offset = 0U;
}

/* Allocate an RX event from the pool */
static struct uart_sam_rx_event *uart_sam_alloc_rx_event(
    struct uart_sam_dev_data *data)
{
    for (int i = 0; i < ARRAY_SIZE(data->rx_event_pool); i++) {
        struct uart_sam_rx_event *event = &data->rx_event_pool[i];

        /* Check if this work item is idle (not queued/running) */
        if (k_work_busy_get(&event->work) == 0) {
            /* This slot is free - claim it */
            return event;
        }
    }

    /* Pool exhausted - this shouldn't happen with proper sizing */
    LOG_ERR("RX event pool exhausted!");
    return NULL;
}

static void uart_sam_dma_rx_done(const struct device *dma_dev, void *arg,
                  uint32_t id, int error_code)
{
    ARG_UNUSED(dma_dev);
    ARG_UNUSED(id);

    struct uart_sam_dev_data *const data = (struct uart_sam_dev_data *)arg;
    const struct device *dev = data->dev;

    /* Quick validation */
    if (data->rx_len == 0U) {
        return;  /* Spurious callback */
    }

    /* Allocate event from pool */
    struct uart_sam_rx_event *event = uart_sam_alloc_rx_event(data);
    if (!event) {
        LOG_ERR("Cannot allocate RX event in DMA callback");
        // TODO provide better error propagation
        error_code = -1;
        return;
    }

    /* Populate event */
    event->dev = dev;
    event->error_code = error_code;

    if (error_code < 0) {
        /* DMA error */
        event->reason = UART_SAM_RX_DMA_ERROR;
    } else {
        /* DMA completed successfully = buffer full */
        event->reason = UART_SAM_RX_BUFFER_FULL;
    }

    /* Cancel timeout work - we're handling completion now */
    k_work_cancel_delayable(&data->rx_timeout_work);

    /*
     * Submit to work queue - k_work_submit will mark this work as busy,
     * preventing it from being allocated again until handler completes
     */
    k_work_submit(&event->work);

    const struct uart_sam_dev_cfg *const cfg = dev->config;
    Uart *regs = cfg->regs;
    regs->UART_IER = UART_IER_RXRDY;
}

static void uart_sam_rx_timeout(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct uart_sam_dev_data *dev_data = CONTAINER_OF(dwork,
        struct uart_sam_dev_data, rx_timeout_work);
    const struct device *dev = dev_data->dev;
    const struct uart_sam_dev_cfg *cfg = dev->config;
    struct dma_status st;

    /* Sanity check */
    if (dev_data->rx_len == 0U || !dev_data->rx_enabled) {
        return;
    }

    /* Get current DMA status */
    if (dma_get_status(cfg->dma_dev, cfg->rx_dma_channel, &st) != 0) {
        LOG_ERR("Failed to get DMA status");
        return;
    }

    // If the dma is busy it means the channel is initializing then the pending length isn't reliable
    // so reschedule this and check later
    if (st.busy) {
        k_work_reschedule(&dev_data->rx_timeout_work,
                         K_USEC(dev_data->rx_inter_byte_timeout));
        return;
    }

    size_t current_position = dev_data->rx_len - st.pending_length;

    /* Check if DMA is initializing or position has advanced */
    if (current_position != dev_data->rx_last_position) {
        /* New data received - restart timeout */
        dev_data->rx_last_position = current_position;

        /* Restart timeout */
        k_work_reschedule(&dev_data->rx_timeout_work,
                         K_USEC(dev_data->rx_inter_byte_timeout));
        return;
    }

    /* Position unchanged - idle detected, packet complete */

    /* Allocate event from pool */
    struct uart_sam_rx_event *event = uart_sam_alloc_rx_event(dev_data);
    if (!event) {
        LOG_ERR("Cannot allocate RX event in timeout");
        /* Try again on next timeout */
        k_work_reschedule(&dev_data->rx_timeout_work,
                         K_USEC(dev_data->rx_inter_byte_timeout));
        return;
    }

    /* Populate event */
    event->dev = dev;
    event->reason = UART_SAM_RX_TIMEOUT;
    event->error_code = 0;

    /*
     * Submit to unified completion handler
     * k_work_submit marks work as busy until handler completes
     */
    k_work_submit(&event->work);
}

#endif

static uint32_t uart_sam_cfg2sam_parity(uint8_t parity)
{
	switch (parity) {
	case UART_CFG_PARITY_EVEN:
		return UART_MR_PAR_EVEN;
	case UART_CFG_PARITY_ODD:
		return UART_MR_PAR_ODD;
	case UART_CFG_PARITY_SPACE:
		return UART_MR_PAR_SPACE;
	case UART_CFG_PARITY_MARK:
		return UART_MR_PAR_MARK;
	case UART_CFG_PARITY_NONE:
	default:
		return UART_MR_PAR_NO;
	}
}

static uint8_t uart_sam_get_parity(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	switch (uart->UART_MR & UART_MR_PAR_Msk) {
	case UART_MR_PAR_EVEN:
		return UART_CFG_PARITY_EVEN;
	case UART_MR_PAR_ODD:
		return UART_CFG_PARITY_ODD;
	case UART_MR_PAR_SPACE:
		return UART_CFG_PARITY_SPACE;
	case UART_MR_PAR_MARK:
		return UART_CFG_PARITY_MARK;
	case UART_MR_PAR_NO:
	default:
		return UART_CFG_PARITY_NONE;
	}
}

static int uart_sam_configure(const struct device *dev,
				const struct uart_config *cfg)
{
	int retval;

	const struct uart_sam_dev_cfg *const config = dev->config;

	volatile Uart * const uart = config->regs;

	/* Driver only supports 8 data bits, 1 stop bit, and no flow control */
	if (cfg->stop_bits != UART_CFG_STOP_BITS_1 ||
		cfg->data_bits != UART_CFG_DATA_BITS_8 ||
		cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	/* Reset and disable UART */
	uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
		      | UART_CR_RXDIS | UART_CR_TXDIS
		      | UART_CR_RSTSTA;

	/* baud rate driven by the peripheral clock, UART does not filter
	 * the receive line, parity chosen by config
	 */
	uart->UART_MR = UART_MR_CHMODE_NORMAL
		      | uart_sam_cfg2sam_parity(cfg->parity);

	/* Set baud rate */
	retval = uart_sam_baudrate_set(dev, cfg->baudrate);
	if (retval != 0) {
		return retval;
	}

	/* Enable receiver and transmitter */
	uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

	return 0;
}

static int uart_sam_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	struct uart_sam_dev_data *const dev_data = dev->data;

	cfg->baudrate = dev_data->baud_rate;
	cfg->parity = uart_sam_get_parity(dev);
	/* only supported mode for this peripheral */
	cfg->stop_bits = UART_CFG_STOP_BITS_1;
	cfg->data_bits = UART_CFG_DATA_BITS_8;
	cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_sam_fifo_fill(const struct device *dev,
			      const uint8_t *tx_data,
			      int size)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	/* Wait for transmitter to be ready. */
	while ((uart->UART_SR & UART_SR_TXRDY) == 0) {
	}

	uart->UART_THR = *tx_data;

	return 1;
}

static int uart_sam_fifo_read(const struct device *dev, uint8_t *rx_data,
			      const int size)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;
	int bytes_read;

	bytes_read = 0;

	while (bytes_read < size) {
		if (uart->UART_SR & UART_SR_RXRDY) {
			rx_data[bytes_read] = uart->UART_RHR;
			bytes_read++;
		} else {
			break;
		}
	}

	return bytes_read;
}

static void uart_sam_irq_tx_enable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	uart->UART_IER = UART_IER_TXRDY;
}

static void uart_sam_irq_tx_disable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	uart->UART_IDR = UART_IDR_TXRDY;
}

static int uart_sam_irq_tx_ready(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	/* Check that the transmitter is ready but only
	 * return true if the interrupt is also enabled
	 */
	return (uart->UART_SR & UART_SR_TXRDY &&
		uart->UART_IMR & UART_IMR_TXRDY);
}

static void uart_sam_irq_rx_enable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	uart->UART_IER = UART_IER_RXRDY;
}

static void uart_sam_irq_rx_disable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	uart->UART_IDR = UART_IDR_RXRDY;
}

static int uart_sam_irq_tx_complete(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	return (uart->UART_SR & UART_SR_TXRDY &&
		uart->UART_IMR & UART_IMR_TXEMPTY);
}

static int uart_sam_irq_rx_ready(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	return (uart->UART_SR & UART_SR_RXRDY);
}

static void uart_sam_irq_err_enable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	uart->UART_IER = UART_IER_OVRE | UART_IER_FRAME | UART_IER_PARE;
}

static void uart_sam_irq_err_disable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	uart->UART_IDR = UART_IDR_OVRE | UART_IDR_FRAME | UART_IDR_PARE;
}

static int uart_sam_irq_is_pending(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart * const uart = cfg->regs;

	return (uart->UART_IMR & (UART_IMR_TXRDY | UART_IMR_RXRDY)) &
		(uart->UART_SR & (UART_SR_TXRDY | UART_SR_RXRDY));
}

static int uart_sam_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static void uart_sam_irq_callback_set(const struct device *dev,
				      uart_irq_callback_user_data_t cb,
				      void *cb_data)
{
	struct uart_sam_dev_data *const dev_data = dev->data;

	dev_data->irq_cb = cb;
	dev_data->irq_cb_data = cb_data;
#if defined(CONFIG_UART_SAM_ASYNC) && defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	dev_data->async_cb = NULL;
	dev_data->async_cb_data = NULL;
#endif
}

#endif
#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_SAM_ASYNC

// UART ISR - handle actual transmission completion
static void uart_sam_isr(const struct device *dev)
{
    struct uart_sam_dev_data *const data = dev->data;
    const struct uart_sam_dev_cfg *const cfg = dev->config;
    Uart *regs = cfg->regs;

    uint32_t status = regs->UART_SR;
    uint32_t imr = regs->UART_IMR;
    uint32_t pending = status & imr;

#if CONFIG_UART_SAM_ASYNC
    /* Handle errors first */
    if (pending & (UART_SR_OVRE | UART_SR_FRAME | UART_SR_PARE)) {
        /* Clear error flags by resetting */
        regs->UART_CR = UART_CR_RSTSTA;
        // Ensure that the RXRDY is enabled
        regs->UART_IER = UART_IER_RXRDY;
        struct uart_sam_rx_event *event = uart_sam_alloc_rx_event(data);
        // TODO handle null event better
        if (event == NULL) {
            return;
        }
        event->dev = dev;
        event->reason = UART_SAM_RX_TIMEOUT;

        if (pending & UART_SR_OVRE) {
            event->error_code |= UART_ERROR_OVERRUN;
        }
        if (pending & UART_SR_FRAME) {
            event->error_code |= UART_ERROR_FRAMING;
        }
        if (pending & UART_SR_PARE) {
            event->error_code |= UART_ERROR_PARITY;
        }

        k_work_submit(&event->work);
        return;
    }

    /* RXRDY: First byte of new packet detected - start DMA and timeout */
    if (status & UART_SR_RXRDY) {
        if (data->rx_enabled) {
            /* Disable RXRDY - only needed for packet start detection */
            regs->UART_IDR = UART_IDR_RXRDY;

            int ret = dma_start(cfg->dma_dev, cfg->rx_dma_channel);
            if (ret < 0) {
                struct uart_sam_rx_event *event = uart_sam_alloc_rx_event(data);
                if (event == NULL) {
                    return;
                }
                // TODO handle the condition where this doesn't return
                event->dev = dev;
                event->reason = UART_SAM_RX_TIMEOUT;
                event->error_code |= UART_ERROR_OVERRUN;
                k_work_submit(&event->work);
                return;
            }

            /* Start inter-byte timeout */
            k_work_reschedule(&data->rx_timeout_work,
                             K_USEC(data->rx_inter_byte_timeout));
        }
    }
#endif

#if CONFIG_UART_INTERRUPT_DRIVEN
    if (data->irq_cb) {
        data->irq_cb(dev, data->irq_cb_data);
    }
#endif
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_SAM_ASYNC

static int uart_sam_callback_set(const struct device *dev,
				  uart_callback_t callback,
				  void *user_data)
{
	struct uart_sam_dev_data *const dev_data = dev->data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	dev_data->cb = NULL;
	dev_data->cb_data = NULL;
#else
	dev_data->async_cb = callback;
	dev_data->async_cb_data = user_data;
#endif

	return 0;
}

static int uart_sam_tx(const struct device *dev, const uint8_t *buf,
			size_t len,
			int32_t timeout)
{
	struct uart_sam_dev_data *const dev_data = dev->data;
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	Uart *regs = cfg->regs;


	int retval;

	if (cfg->tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	if (dev_data->tx_len != 0U) {
		retval = -EBUSY;
		goto err;
	}
	dev_data->tx_buf = buf;
	dev_data->tx_len = len;
/* #ifdef CONFIG_CACHE_MANAGEMENT */
/*     sys_cache_data_flush_range((void *)buf, len); */
/* #endif */
	retval = dma_reload(cfg->dma_dev, cfg->tx_dma_channel, (uintptr_t)buf,
						(uintptr_t)(&(regs->UART_THR)), len);
	if (retval != 0U) {
        dev_data->tx_buf = NULL;
        dev_data->tx_len = 0;
        irq_unlock(key);
		return retval;
	}

	if (timeout != SYS_FOREVER_US) {
		k_work_reschedule(&dev_data->tx_timeout_work,
				      K_USEC(timeout));
	}

	retval = dma_start(cfg->dma_dev, cfg->tx_dma_channel);
	if (retval != 0U) {
        k_work_cancel_delayable(&dev_data->tx_timeout_work);
        dev_data->tx_buf = NULL;
        dev_data->tx_len = 0;
        irq_unlock(key);
		return retval;
	}
	/* regs->UART_IER = UART_IER_TXRDY; */

    irq_unlock(key);
    return retval;
err:
	irq_unlock(key);
	return retval;
}

static int uart_sam_tx_abort(const struct device *dev)
{
	struct uart_sam_dev_data *const dev_data = dev->data;
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	if (cfg->tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	k_work_cancel_delayable(&dev_data->tx_timeout_work);

	return uart_sam_tx_halt(dev_data);
}

/**
 * @brief Configure DMA for RX transfer
 */
static int uart_sam_configure_rx_dma(const struct device *dev,
                      uint8_t *buf, size_t len)
{
    const struct uart_sam_dev_cfg *const cfg = dev->config;
    struct uart_sam_dev_data *const data = dev->data;
    Uart *regs = cfg->regs;

    /* Validate DMA is available */
    if (!device_is_ready(cfg->dma_dev)) {
        LOG_ERR("DMA device not ready");
        return -ENODEV;
    }

    struct dma_block_config dma_blk = {
        .source_address = (uintptr_t)&regs->UART_RHR,
        .dest_address = (uintptr_t)buf,
        .block_size = len,
        .source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
        .dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
    };

    struct dma_config dma_cfg = {
        .channel_direction = PERIPHERAL_TO_MEMORY,
        .source_data_size = 1,
        .dest_data_size = 1,
        .source_burst_length = 1,
        .dest_burst_length = 1,
        .head_block = &dma_blk,
        .dma_callback = uart_sam_dma_rx_done,
        .complete_callback_en = 1,
        .user_data = data,
        .dma_slot = cfg->rx_dma_request,
        .block_count = 1,
    };

    int ret = dma_config(cfg->dma_dev, cfg->rx_dma_channel, &dma_cfg);
    if (ret != 0) {
        LOG_ERR("RX DMA config failed: %d", ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Flush any stale data from the RX FIFO
 */
static void uart_sam_flush_rx_fifo(Uart *regs)
{
    uint32_t status;
    int flush_count = 0;
    const int max_flushes = 16; /* Prevent infinite loop */

    /* Read and discard any pending data */
    while (flush_count++ < max_flushes) {
        status = regs->UART_SR;
        if (!(status & UART_SR_RXRDY)) {
            break;
        }
        (void)regs->UART_RHR;
    }

    /* Clear any error flags */
    regs->UART_CR = UART_CR_RSTSTA;
}

/*
* Calculate inter-byte timeout based on actual frame configuration.
*/
static uint32_t get_dev_interbyte_timeout(const struct device *dev) {
    struct uart_config config;
    uart_sam_config_get(dev, &config);

    /* Calculate bits per frame based on configuration */
    uint32_t bits_per_byte = 1;  /* Start bit */

    /* Add data bits */
    switch (config.data_bits) {
    case UART_CFG_DATA_BITS_5:
        bits_per_byte += 5;
        break;
    case UART_CFG_DATA_BITS_6:
        bits_per_byte += 6;
        break;
    case UART_CFG_DATA_BITS_7:
        bits_per_byte += 7;
        break;
    case UART_CFG_DATA_BITS_8:
    default:
        bits_per_byte += 8;
        break;
    }

    /* Add parity bit if enabled */
    if (config.parity != UART_CFG_PARITY_NONE) {
        bits_per_byte += 1;
    }

    /* Add stop bits */
    switch (config.stop_bits) {
    case UART_CFG_STOP_BITS_0_5:
        /* 0.5 stop bits - round up */
        bits_per_byte += 1;
        break;
    case UART_CFG_STOP_BITS_1:
        bits_per_byte += 1;
        break;
    case UART_CFG_STOP_BITS_1_5:
        bits_per_byte += 2;  /* Conservative: round up */
        break;
    case UART_CFG_STOP_BITS_2:
        bits_per_byte += 2;
        break;
    }

    /* Calculate byte time in microseconds */
    uint32_t byte_time_us = (bits_per_byte * 1000000U) / config.baudrate;


    // As per data sheet 46.5.2.2 start is dected 16 times the baud rate, so stop is also
    uint32_t safety_multiplier = 16;
    uint32_t calculated_timeout = byte_time_us * safety_multiplier;

    /* Enforce reasonable bounds */
    const uint32_t MIN_TIMEOUT_US = 500;   /* 500µs minimum */
    const uint32_t MAX_TIMEOUT_US = 50000; /* 50ms maximum */

    uint32_t rx_inter_byte_timeout = CLAMP(calculated_timeout, MIN_TIMEOUT_US, MAX_TIMEOUT_US);
    return rx_inter_byte_timeout;
}

/**
 * @brief Enable asynchronous UART reception with DMA
 *
 * @param dev UART device
 * @param buf Buffer to receive data into
 * @param len Length of buffer
 * @param timeout Timeout in microseconds (SYS_FOREVER_US for no timeout)
 * @return 0 on success, negative error code on failure
 */
static int uart_sam_rx_enable(const struct device *dev, uint8_t *buf,
                   size_t len, int32_t timeout)
{
    struct uart_sam_dev_data *const data = dev->data;
    const struct uart_sam_dev_cfg *const cfg = dev->config;
    Uart *regs = cfg->regs;
    int ret;

    if (cfg->rx_dma_channel == 0xFFU) {
        return -ENOTSUP;
    }

    if (!buf || len == 0) {
        return -EINVAL;
    }

    if (len > UINT16_MAX) {
        return -EINVAL;
    }

    unsigned int key = irq_lock();

    if (data->rx_enabled || data->rx_len != 0U) {
        irq_unlock(key);
        return -EBUSY;
    }

    uart_sam_flush_rx_fifo(regs);

    /* Initialize RX state */
    data->rx_buf = buf;
    data->rx_len = len;
    data->rx_offset = 0U;
    data->rx_last_position = 0U;
    data->rx_enabled = true;

    data->rx_inter_byte_timeout = get_dev_interbyte_timeout(dev);

    /* Configure DMA transfer */
    ret = uart_sam_configure_rx_dma(dev, buf, len);
    if (ret != 0) {
        goto error_cleanup;
    }

    // NOTE this reload might be redundant after coonfiguring it
    ret = dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
                    (uintptr_t)(&regs->UART_RHR),
                    (uintptr_t)data->rx_buf, data->rx_len);
    if (ret < 0) {
        LOG_ERR("DMA reload failed: %d", ret);
        goto error_cleanup;
    }

    /* Enable UART receiver and error interrupts */
    regs->UART_CR = UART_CR_RSTSTA | UART_CR_RXEN;
    regs->UART_IER = UART_SR_OVRE | UART_SR_FRAME | UART_SR_PARE | UART_IER_RXRDY;

    /*
     * Since we're using timeout-based packet detection, wait for RXRDY
     * before starting DMA
     */
    // NOTE may have to start this ahead of time so that we don't miss the first character
    /* ret = dma_start(cfg->dma_dev, cfg->rx_dma_channel); */
    /* if (ret != 0) { */
    /*     LOG_ERR("RX DMA start failed: %d", ret); */
    /*     goto error_cleanup; */
    /* } */

    irq_unlock(key);
    return 0;

error_cleanup:
    data->rx_buf = NULL;
    data->rx_len = 0U;
    data->rx_offset = 0U;
    data->rx_enabled = false;
    regs->UART_CR = UART_CR_RXDIS;
    regs->UART_IDR = UART_SR_OVRE | UART_SR_FRAME | UART_SR_PARE;

    irq_unlock(key);
    return ret;
}

static int uart_sam_rx_buf_rsp(const struct device *dev, uint8_t *buf,
				size_t len)
{
	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	struct uart_sam_dev_data *const dev_data = dev->data;
	unsigned int key = irq_lock();
	int retval = 0;

	if (dev_data->rx_len == 0U) {
		retval = -EACCES;
		goto err;
	}

	if (dev_data->rx_next_len != 0U) {
		retval = -EBUSY;
		goto err;
	}

	dev_data->rx_next_buf = buf;
	dev_data->rx_next_len = len;

	irq_unlock(key);
	return 0;

err:
	irq_unlock(key);
	return retval;
}

static int uart_sam_rx_disable(const struct device *dev)
{
    struct uart_sam_dev_data *data = dev->data;

    data->rx_buf = NULL;
    data->rx_len = 0U;
    data->rx_offset = 0U;
    data->rx_enabled = false;

    if (data->async_cb) {
        struct uart_event evt = {
            .type = UART_RX_DISABLED,
        };
        data->async_cb(dev, &evt, data->async_cb_data);
    }

}

#endif

static void uart_sam_rx_completion_handler(struct k_work *work)
{
    struct uart_sam_rx_event *event = CONTAINER_OF(work,
                                                    struct uart_sam_rx_event,
                                                    work);
    const struct device *dev = event->dev;
    const struct uart_sam_dev_cfg *cfg = dev->config;
    struct uart_sam_dev_data *dev_data = dev->data;
    Uart *regs = cfg->regs;
    struct dma_status st;
    int ret;

    unsigned int key = irq_lock();

    /* Validate RX is still active */
    if (dev_data->rx_len == 0U || !dev_data->rx_enabled) {
        irq_unlock(key);
        return;
    }

    /* Handle error case first - different flow */
    // /* TODO refactor with a switch case on reason
    if (event->reason == UART_SAM_RX_DMA_ERROR) {
        LOG_ERR("RX DMA error: %d", event->error_code);
        uart_sam_abort_rx(dev, UART_ERROR_OVERRUN);
        irq_unlock(key);
        return;
    }

    if (event->reason == UART_SAM_RX_SR_ERROR) {
        LOG_ERR("RX SR error: %d", event->error_code);
        uart_sam_abort_rx(dev, event->error_code);
        irq_unlock(key);
        return;
    }

    /* For timeout case, ensure DMA is stopped FIRST */
    if (event->reason == UART_SAM_RX_TIMEOUT) {
        /* CRITICAL: Add memory barrier to ensure DMA writes are visible */
        #ifdef CONFIG_DCACHE
        /* Ensure all pending DMA writes complete before reading status */
        __DSB();  /* Data Synchronization Barrier */
        __ISB();  /* Instruction Synchronization Barrier */
        #endif
    }

    dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
    /* NOW get the status after DMA is stopped */
    size_t bytes_received;
    ret = dma_get_status(cfg->dma_dev, cfg->rx_dma_channel, &st);
    if (ret != 0) {
        LOG_ERR("Failed to get DMA status: %d", ret);
        uart_sam_abort_rx(dev, UART_ERROR_OVERRUN);
        irq_unlock(key);
        return;
    }

    bytes_received = dev_data->rx_len - st.pending_length;
    dev_data->rx_last_position = bytes_received;

    if (bytes_received == 0) {
        LOG_WRN("RX completion with no data");

        /* Restart DMA */
        dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
                  (uintptr_t)(&regs->UART_RHR),
                  (uintptr_t)dev_data->rx_buf,
                  dev_data->rx_len);
        dev_data->rx_last_position = 0;
        regs->UART_IER = UART_IER_RXRDY;
        irq_unlock(key);
        return;
    }

    #ifdef CONFIG_DCACHE
    /*
     * CRITICAL CACHE INVALIDATION
     * Must invalidate AFTER DMA stops and BEFORE reading buffer
     */

    /* Ensure DMA writes are complete */
    __DSB();

    /* Calculate cache-aligned region */
    uintptr_t buf_start = (uintptr_t)dev_data->rx_buf;
    uintptr_t buf_end = buf_start + bytes_received;

    /* Align to cache line boundaries (32 bytes on Cortex-M7) */
    uintptr_t cache_start = buf_start & ~(CONFIG_DCACHE_LINE_SIZE - 1);
    uintptr_t cache_end = (buf_end + CONFIG_DCACHE_LINE_SIZE - 1) &
                          ~(CONFIG_DCACHE_LINE_SIZE - 1);

    /* Invalidate the cache */
    sys_cache_data_invd_range((void *)cache_start, cache_end - cache_start);

    /* Ensure invalidation completes before reading */
    __DSB();
    __ISB();
    #endif

    /* Determine if buffer is complete (full or partial packet done) */
    bool buffer_complete = event->reason == UART_SAM_RX_BUFFER_FULL ||
        dev_data->rx_len == st.pending_length;

    /* Notify application of received data */
    uart_sam_notify_rx_data(dev, bytes_received);

    /* Only release buffer if it's actually complete */
    if (buffer_complete) {
        uart_sam_release_rx_buffer(dev, dev_data->rx_buf);

        /* Request next buffer from user */
        uart_sam_request_rx_buffer(dev);

        /* Check for next buffer */
        if (!dev_data->rx_next_buf) {
            uart_sam_rx_disable(dev);
            irq_unlock(key);
            return;
        }

        /* Switch to next buffer */
        uart_sam_switch_to_next_rx_buffer(dev_data);

        /* Reset state for next packet */
        dev_data->rx_offset = 0;
        dev_data->rx_last_position = 0;

        /* Reload DMA with new buffer */
        ret = dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
                        (uintptr_t)(&regs->UART_RHR),
                        (uintptr_t)dev_data->rx_buf,
                        dev_data->rx_len);
        if (ret < 0) {
            LOG_ERR("DMA reload failed: %d", ret);
            uart_sam_abort_rx(dev, UART_ERROR_OVERRUN);
            irq_unlock(key);
            return;
        }
    }

    // Enable detection of the next incoming packet
    regs->UART_IER = UART_IER_RXRDY;
    irq_unlock(key);
}

static int uart_sam_init(const struct device *dev)
{
    int retval;
    const struct uart_sam_dev_cfg *const cfg = dev->config;
    struct uart_sam_dev_data *const dev_data = dev->data;
    Uart *const uart = cfg->regs;

    /* Enable UART clock in PMC */
    (void)clock_control_on(SAM_DT_PMC_CONTROLLER,
                   (clock_control_subsys_t)&cfg->clock_cfg);

    /* Connect pins to the peripheral */
    retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (retval < 0) {
        return retval;
    }

    /* Disable all interrupts */
    uart->UART_IDR = 0xFFFFFFFF;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_SAM_ASYNC
    cfg->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_SAM_ASYNC
    dev_data->dev = dev;

    /* Verify DMA device is ready */
    if (!device_is_ready(cfg->dma_dev)) {
        LOG_ERR("DMA device not ready");
        return -ENODEV;
    }

    /* Initialize work items */
    k_work_init_delayable(&dev_data->rx_timeout_work, uart_sam_rx_timeout);

    /* Initialize event pool - all work items start idle */
    for (int i = 0; i < ARRAY_SIZE(dev_data->rx_event_pool); i++) {
        k_work_init(&dev_data->rx_event_pool[i].work,
                   uart_sam_rx_completion_handler);
        dev_data->rx_event_pool[i].dev = dev;
    }

    k_work_init_delayable(&dev_data->tx_timeout_work, uart_sam_tx_timeout);

	// Since we don't "enable" tx we need to do configure the dma on startup
	if (cfg->tx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = { 0 };
		struct dma_block_config dma_blk = { 0 };

		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = uart_sam_dma_tx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->tx_dma_request;
        dma_cfg.source_burst_length = 1,
        dma_cfg.dest_burst_length = 1,
        dma_cfg.complete_callback_en = 1,

        // Set's the following as sane defaults
		dma_blk.block_size = 1;
		dma_blk.dest_address = (uintptr_t)(&(uart->UART_THR));
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel,
				    &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

    /* Initialize async state with null DMA configuration */
    dev_data->rx_enabled = false;
    dev_data->tx_buf = NULL;
    dev_data->rx_buf = NULL;
    dev_data->tx_len = 0;
    dev_data->rx_len = 0;
#endif

    /* Configure UART parameters */
    struct uart_config uart_config = {
        .baudrate = dev_data->baud_rate,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };

    return uart_sam_configure(dev, &uart_config);
}

static DEVICE_API(uart, uart_sam_driver_api) = {
	.poll_in = uart_sam_poll_in,
	.poll_out = uart_sam_poll_out,
	.err_check = uart_sam_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_sam_configure,
	.config_get = uart_sam_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_sam_fifo_fill,
	.fifo_read = uart_sam_fifo_read,
	.irq_tx_enable = uart_sam_irq_tx_enable,
	.irq_tx_disable = uart_sam_irq_tx_disable,
	.irq_tx_ready = uart_sam_irq_tx_ready,
	.irq_rx_enable = uart_sam_irq_rx_enable,
	.irq_rx_disable = uart_sam_irq_rx_disable,
	.irq_tx_complete = uart_sam_irq_tx_complete,
	.irq_rx_ready = uart_sam_irq_rx_ready,
	.irq_err_enable = uart_sam_irq_err_enable,
	.irq_err_disable = uart_sam_irq_err_disable,
	.irq_is_pending = uart_sam_irq_is_pending,
	.irq_update = uart_sam_irq_update,
	.irq_callback_set = uart_sam_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
#if CONFIG_UART_SAM_ASYNC
	.callback_set = uart_sam_callback_set,
	.tx = uart_sam_tx,
	.tx_abort = uart_sam_tx_abort,
	.rx_enable = uart_sam_rx_enable,
	.rx_buf_rsp = uart_sam_rx_buf_rsp,
	.rx_disable = uart_sam_rx_disable,
#endif
};

#define UART_SAM_DMA_INIT(n)                                            \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                        \
        (.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)),   \
         .rx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),  \
         .tx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),  \
         .rx_dma_request = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),    \
         .tx_dma_request = DT_INST_DMAS_CELL_BY_NAME(n, tx, perid)),   \
        ())

/* Device instantiation macros */
#define UART_SAM_DECLARE_CFG(n, IRQ_FUNC_INIT)                          \
    static const struct uart_sam_dev_cfg uart_sam_cfg_##n = {          \
        .regs = (Uart *)DT_INST_REG_ADDR(n),                           \
        .clock_cfg = SAM_DT_INST_CLOCK_PMC_CFG(n),                     \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                     \
        IRQ_FUNC_INIT                                                   \
        UART_SAM_DMA_INIT(n)                                           \
    };

#ifdef CONFIG_UART_ASYNC_API
#define UART_SAM_CONFIG_FUNC(n)                                         \
    static void uart_sam_irq_config_func_##n(const struct device *dev) \
    {                                                                   \
        IRQ_CONNECT(DT_INST_IRQN(n),                                   \
                   DT_INST_IRQ(n, priority),                           \
                   uart_sam_isr,                                       \
                   DEVICE_DT_INST_GET(n),                              \
                   0);                                                  \
        irq_enable(DT_INST_IRQN(n));                                   \
    };

#define UART_SAM_IRQ_CFG_FUNC_INIT(n) \
    .irq_config_func = uart_sam_irq_config_func_##n,

#else /* !CONFIG_UART_ASYNC_API */

#define UART_SAM_CONFIG_FUNC(n)
#define UART_SAM_IRQ_CFG_FUNC_INIT(n)

#endif /* CONFIG_UART_ASYNC_API */

#define UART_SAM_INIT_CFG(n) \
    UART_SAM_DECLARE_CFG(n, UART_SAM_IRQ_CFG_FUNC_INIT(n))

#define UART_SAM_INIT(n)                                               \
    PINCTRL_DT_INST_DEFINE(n);                                         \
                                                                        \
    static struct uart_sam_dev_data uart##n##_sam_data = {             \
        .baud_rate = DT_INST_PROP(n, current_speed),                   \
    };                                                                  \
                                                                        \
    UART_SAM_CONFIG_FUNC(n);                                            \
    UART_SAM_INIT_CFG(n);                                               \
                                                                        \
    DEVICE_DT_INST_DEFINE(n,                                           \
        uart_sam_init,                                                 \
        NULL,                                                          \
        &uart##n##_sam_data,                                           \
        &uart_sam_cfg_##n,                                             \
        POST_KERNEL,                                                   \
        CONFIG_SERIAL_INIT_PRIORITY,                                   \
        &uart_sam_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_SAM_INIT)
