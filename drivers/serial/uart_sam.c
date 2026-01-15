/*
 * Copyright (c) 2017 Piotr Mienkowski
 * Copyright (c) 2018 Justin Watson
 * Copyright (c) 2023 Gerson Fernando Budke
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_uart

/** @file
 * @brief UART driver for Atmel SAM MCU family.
 */

#include <stdint.h>
#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/atmel_sam_pmc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_DCACHE
#include <zephyr/cache.h>
#endif

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#endif

LOG_MODULE_REGISTER(uart_sam, CONFIG_UART_LOG_LEVEL);

#define RX_POLL_INTERVAL_MS 25

/* Device constant configuration parameters */
struct uart_sam_dev_cfg {
	Uart *regs;
	const struct atmel_sam_pmc_config clock_cfg;
	const struct pinctrl_dev_config *pcfg;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif

#ifdef CONFIG_UART_ASYNC_API
	const struct device *dma_dev;
	uint32_t rx_dma_channel;
	uint32_t tx_dma_channel;
	uint32_t rx_dma_request; /* Peripheral ID for DMA handshaking */
	uint32_t tx_dma_request;
#endif
};

/* Device run time data */
struct uart_sam_dev_data {
	const struct device *dev;
	uint32_t baud_rate;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t irq_cb; /* Interrupt Callback */
	void *irq_cb_data;                    /* Interrupt Callback Arg */
#endif                                        /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
	/* RX ring buffer */
	uint8_t *rx_ring;
	size_t rx_ring_len; /* total length of rx_ring */
	size_t rx_rd_ptr;   /* read pointer (software) */

	/* Periodic work for RX processing */
	struct k_work_delayable rx_poll_work;

	/* Async callback */
	uart_callback_t async_cb;
	void *async_cb_data;
	bool rx_enabled;

	/* TX state */
	const uint8_t *tx_buf;
	size_t tx_len;
	struct k_work tx_complete_work;
#endif
};

// =============================================================================
// Basic UART Functions (Polling)
// =============================================================================

static int uart_sam_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	Uart *const uart = cfg->regs;

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
	Uart *const uart = cfg->regs;

	/* Wait for transmitter to be ready */
	while (!(uart->UART_SR & UART_SR_TXRDY)) {
	}

	/* send a character */
	uart->UART_THR = (uint32_t)c;
}

static int uart_sam_err_check(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	volatile Uart *const uart = cfg->regs;
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

// =============================================================================
// UART Configuration
// =============================================================================

static int uart_sam_baudrate_set(const struct device *dev, uint32_t baudrate)
{
	struct uart_sam_dev_data *const dev_data = dev->data;
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	volatile Uart *const uart = cfg->regs;
	uint32_t divisor;

	__ASSERT(baudrate, "baud rate has to be bigger than 0");
	__ASSERT(SOC_ATMEL_SAM_MCK_FREQ_HZ / 16U >= baudrate,
		 "MCK frequency is too small to set required baud rate");

	divisor = SOC_ATMEL_SAM_MCK_FREQ_HZ / 16U / baudrate;
	if (divisor > 0xFFFF) {
		return -EINVAL;
	}

	uart->UART_BRGR = UART_BRGR_CD(divisor);
	dev_data->baud_rate = baudrate;
	return 0;
}

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

	volatile Uart *const uart = cfg->regs;

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

static int uart_sam_configure(const struct device *dev, const struct uart_config *cfg)
{
	int retval;
	const struct uart_sam_dev_cfg *const config = dev->config;
	volatile Uart *const uart = config->regs;

	/* Driver only supports 8 data bits, 1 stop bit, and no flow control */
	if (cfg->stop_bits != UART_CFG_STOP_BITS_1 || cfg->data_bits != UART_CFG_DATA_BITS_8 ||
	    cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	/* Reset and disable UART */
	uart->UART_CR =
		UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

	/* baud rate driven by the peripheral clock, UART does not filter
	 * the receive line, parity chosen by config
	 */
	uart->UART_MR = UART_MR_CHMODE_NORMAL | uart_sam_cfg2sam_parity(cfg->parity);

	/* Set baud rate */
	retval = uart_sam_baudrate_set(dev, cfg->baudrate);
	if (retval != 0) {
		return retval;
	}

	/* Enable receiver and transmitter */
	uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

	return 0;
}

static int uart_sam_config_get(const struct device *dev, struct uart_config *cfg)
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

static int uart_sam_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	/* Wait for transmitter to be ready. */
	while ((uart->UART_SR & UART_SR_TXRDY) == 0) {
	}

	uart->UART_THR = *tx_data;

	return 1;
}

static int uart_sam_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;
	int bytes_read = 0;

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

	volatile Uart *const uart = cfg->regs;

	uart->UART_IER = UART_IER_TXRDY;
}

static void uart_sam_irq_tx_disable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	uart->UART_IDR = UART_IDR_TXRDY;
}

static int uart_sam_irq_tx_ready(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	/* Check that the transmitter is ready but only
	 * return true if the interrupt is also enabled
	 */
	return (uart->UART_SR & UART_SR_TXRDY && uart->UART_IMR & UART_IMR_TXRDY);
}

static void uart_sam_irq_rx_enable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	uart->UART_IER = UART_IER_RXRDY;
}

static void uart_sam_irq_rx_disable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	uart->UART_IDR = UART_IDR_RXRDY;
}

static int uart_sam_irq_tx_complete(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	return (uart->UART_SR & UART_SR_TXRDY && uart->UART_IMR & UART_IMR_TXEMPTY);
}

static int uart_sam_irq_rx_ready(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	return (uart->UART_SR & UART_SR_RXRDY);
}

static void uart_sam_irq_err_enable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	uart->UART_IER = UART_IER_OVRE | UART_IER_FRAME | UART_IER_PARE;
}

static void uart_sam_irq_err_disable(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	uart->UART_IDR = UART_IDR_OVRE | UART_IDR_FRAME | UART_IDR_PARE;
}

static int uart_sam_irq_is_pending(const struct device *dev)
{
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	volatile Uart *const uart = cfg->regs;

	return (uart->UART_IMR & (UART_IMR_TXRDY | UART_IMR_RXRDY)) &
	       (uart->UART_SR & (UART_SR_TXRDY | UART_SR_RXRDY));
}

static int uart_sam_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static void uart_sam_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				      void *cb_data)
{
	struct uart_sam_dev_data *const dev_data = dev->data;

	dev_data->irq_cb = cb;
	dev_data->irq_cb_data = cb_data;
}

static void uart_sam_isr(const struct device *dev)
{
	struct uart_sam_dev_data *const dev_data = dev->data;

	if (dev_data->irq_cb) {
		dev_data->irq_cb(dev, dev_data->irq_cb_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

// =============================================================================
// Async UART API Implementation
// =============================================================================

#ifdef CONFIG_UART_SAM_ASYNC

// =============================================================================
// DMA TX Implementation
// =============================================================================

static void uart_sam_dma_tx_done(const struct device *dma_dev, void *arg, uint32_t id,
				 int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	struct uart_sam_dev_data *const dev_data = (struct uart_sam_dev_data *)arg;
	k_work_submit(&dev_data->tx_complete_work);
}

static void uart_sam_tx_complete_handler(struct k_work *work)
{
	struct uart_sam_dev_data *dev_data =
		CONTAINER_OF(work, struct uart_sam_dev_data, tx_complete_work);
	const struct device *dev = dev_data->dev;

	struct uart_event evt = {
		.type = UART_TX_DONE,
		.data.tx =
			{
				.buf = dev_data->tx_buf,
				.len = dev_data->tx_len,
			},
	};

	dev_data->tx_buf = NULL;
	dev_data->tx_len = 0;

	if (dev_data->async_cb) {
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}
}

// =============================================================================
// DMA RX Ring Buffer Implementation
// =============================================================================

/**
 * @brief Flush any stale data from the RX FIFO
 */
static void uart_sam_flush_rx_fifo(Uart *regs)
{
	int flush_count = 0;
	const int max_flushes = 16; /* Prevent infinite loop */

	/* Read and discard any pending data */
	while (flush_count++ < max_flushes) {
		if (!(regs->UART_SR & UART_SR_RXRDY_Msk)) {
			break;
		}
		(void)regs->UART_RHR;
	}

	/* Clear any error flags */
	regs->UART_CR = UART_CR_RSTSTA_Msk;
}

/**
 * @brief RX polling work handler - implements the ring buffer algorithm
 *
 * Definitions:
 *  - rd_ptr: The next location that should be read from
 *  - wr_ptr: The next location that will be written to
 *
 * Algorithm:
 * 1. Flush DMA FIFO
 * 2. Calculate wr_ptr from DMA status: wr_ptr = ublen_max - ublen
 * 3. Compare wr_ptr with rd_ptr:
 *    - if wr_ptr > rd_ptr: read from rd_ptr to wr_ptr
 *    - if wr_ptr < rd_ptr: read to end, then from start to wr_ptr
 *    - if wr_ptr == rd_ptr: buffer empty, do nothing
 */
static void uart_sam_rx_poll_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_sam_dev_data *dev_data =
		CONTAINER_OF(dwork, struct uart_sam_dev_data, rx_poll_work);
	const struct device *dev = dev_data->dev;
	const struct uart_sam_dev_cfg *cfg = dev->config;
	struct dma_status st;
	size_t wr_ptr, rd_ptr;
	size_t bytes_to_read;

	if (!dev_data->rx_enabled) {
		return;
	}

	/* Flush DMA FIFO - ensure all pending writes are visible */
	__DSB();
	__ISB();

	/* Get DMA status to determine write pointer */
	if (dma_get_status(cfg->dma_dev, cfg->rx_dma_channel, &st) != 0) {
		LOG_ERR("Failed to get DMA status");
		goto reschedule;
	}

	/* Clamp pending length to the size of the ring buffer*/
	if (st.pending_length > dev_data->rx_ring_len) {
		LOG_WRN("DMA status pending_length (%u) > rx_ring_len (%zu); clamping",
			(unsigned int)st.pending_length, dev_data->rx_ring_len);
		st.pending_length = dev_data->rx_ring_len;
	}

	/* Calculate write pointer: wr_ptr = ublen_max - ublen (pending_length) */
	wr_ptr = dev_data->rx_ring_len - st.pending_length;
	rd_ptr = dev_data->rx_rd_ptr;

#ifdef CONFIG_DCACHE
	/* Invalidate cache for the ring buffer to ensure CPU sees DMA data */
	sys_cache_data_invd_range((void *)dev_data->rx_ring, dev_data->rx_ring_len);
	__DSB();
	__ISB();
#endif

	if (wr_ptr > rd_ptr) {
		/* Case 1: Simple read from rd_ptr to wr_ptr (no wrap-around) */
		bytes_to_read = wr_ptr - rd_ptr;

		if (bytes_to_read > 0 && dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_RDY,
				.data.rx =
					{
						.buf = dev_data->rx_ring,
						.offset = rd_ptr,
						.len = bytes_to_read,
					},
			};
			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
			dev_data->rx_rd_ptr = wr_ptr;
		}
	} else if (wr_ptr < rd_ptr) {
		/* Case 2: Wrap-around */

		/* First chunk: rd_ptr to end of buffer */
		bytes_to_read = dev_data->rx_ring_len - rd_ptr;
		if (bytes_to_read > 0 && dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_RDY,
				.data.rx =
					{
						.buf = dev_data->rx_ring,
						.offset = rd_ptr,
						.len = bytes_to_read,
					},
			};
			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		/* Second chunk: start of buffer to wr_ptr */
		if (wr_ptr > 0 && dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_RDY,
				.data.rx =
					{
						.buf = dev_data->rx_ring,
						.offset = 0,
						.len = wr_ptr,
					},
			};
			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		dev_data->rx_rd_ptr = wr_ptr;
	}
	/* Case 3: Buffer is empty, do nothing */

reschedule:
	/* Reschedule for next poll (25ms interval) */
	k_work_reschedule(&dev_data->rx_poll_work, K_MSEC(RX_POLL_INTERVAL_MS));
}

// =============================================================================
// Async UART API Implementation
// =============================================================================

static int uart_sam_callback_set(const struct device *dev, uart_callback_t callback,
				 void *user_data)
{
	struct uart_sam_dev_data *const dev_data = dev->data;

	dev_data->async_cb = callback;
	dev_data->async_cb_data = user_data;

	return 0;
}

static int uart_sam_tx(const struct device *dev, const uint8_t *buf, size_t len, int32_t timeout)
{
	ARG_UNUSED(timeout);

	struct uart_sam_dev_data *const dev_data = dev->data;
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	Uart *regs = cfg->regs;
	int retval;

	if (cfg->tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFF) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	if (dev_data->tx_len != 0U) {
		irq_unlock(key);
		return -EBUSY;
	}

	dev_data->tx_buf = buf;
	dev_data->tx_len = len;

#ifdef CONFIG_DCACHE
	/* Flush cache to ensure DMA sees the latest data */
	sys_cache_data_flush_range((void *)buf, len);
#endif

	retval = dma_reload(cfg->dma_dev, cfg->tx_dma_channel, (uintptr_t)buf,
			    (uintptr_t)(&(regs->UART_THR)), len);
	if (retval != 0) {
		dev_data->tx_buf = NULL;
		dev_data->tx_len = 0;
		irq_unlock(key);
		return retval;
	}

	retval = dma_start(cfg->dma_dev, cfg->tx_dma_channel);
	if (retval != 0) {
		dev_data->tx_buf = NULL;
		dev_data->tx_len = 0;
		irq_unlock(key);
		return retval;
	}

	irq_unlock(key);
	return 0;
}

static int uart_sam_tx_abort(const struct device *dev)
{
	struct uart_sam_dev_data *const dev_data = dev->data;
	const struct uart_sam_dev_cfg *const cfg = dev->config;

	if (cfg->tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	unsigned int key = irq_lock();

	dma_stop(cfg->dma_dev, cfg->tx_dma_channel);

	if (dev_data->tx_len > 0 && dev_data->async_cb) {
		struct uart_event evt = {
			.type = UART_TX_ABORTED,
			.data.tx =
				{
					.buf = dev_data->tx_buf,
					.len = dev_data->tx_len,
				},
		};
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	dev_data->tx_buf = NULL;
	dev_data->tx_len = 0;

	irq_unlock(key);
	return 0;
}

static int uart_sam_rx_enable(const struct device *dev, uint8_t *buf, size_t len, int32_t timeout)
{
	/* Note: buf and len are not used in ring buffer implementation */
	ARG_UNUSED(timeout);

	struct uart_sam_dev_data *const dev_data = dev->data;
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	Uart *regs = cfg->regs;

	if (cfg->rx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (!buf || len == 0) {
		return -EINVAL;
	}

	if (len > UINT16_MAX) {
		return -EINVAL;
	}

	if (dev_data->rx_enabled) {
		return -EBUSY;
	}

	/* Reset read pointer */
	dev_data->rx_rd_ptr = 0;

	/* Flush RX FIFO */
	uart_sam_flush_rx_fifo(regs);

	dev_data->rx_ring = buf;
	dev_data->rx_ring_len = len;

	/*
	 * Configure circular DMA with single descriptor pointing to itself
	 * This creates a ring buffer where DMA continuously writes
	 */
	struct dma_block_config dma_blk = {
		.source_address = (uintptr_t)&regs->UART_RHR,
		.dest_address = (uintptr_t)dev_data->rx_ring,
		.block_size = dev_data->rx_ring_len,
		.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	};

	struct dma_config dma_cfg = {
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_data_size = 1,
		.dest_data_size = 1,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.block_count = 1,
		.head_block = &dma_blk,
		.complete_callback_en = 0, /* No completion callback for circular */
		.user_data = dev_data,
		.dma_slot = cfg->rx_dma_request,
	};

	int ret = dma_config(cfg->dma_dev, cfg->rx_dma_channel, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("RX DMA config failed: %d", ret);
		return ret;
	}

	ret = dma_start(cfg->dma_dev, cfg->rx_dma_channel);
	if (ret != 0) {
		LOG_ERR("RX DMA start failed: %d", ret);
		return ret;
	}

	/* Enable receiver */
	regs->UART_CR = UART_CR_RSTSTA | UART_CR_RXEN;

	dev_data->rx_enabled = true;

	/* Start periodic polling (25ms interval) */
	k_work_reschedule(&dev_data->rx_poll_work, K_MSEC(RX_POLL_INTERVAL_MS));

	LOG_INF("RX enabled with %zu byte ring buffer, polling every %dms", dev_data->rx_ring_len,
		RX_POLL_INTERVAL_MS);

	return 0;
}

static int uart_sam_rx_disable(const struct device *dev)
{
	struct uart_sam_dev_data *dev_data = dev->data;
	const struct uart_sam_dev_cfg *cfg = dev->config;
	Uart *regs = cfg->regs;

	if (!dev_data->rx_enabled) {
		return -EFAULT;
	}

	/* Cancel polling work */
	k_work_cancel_delayable(&dev_data->rx_poll_work);

	/* Stop DMA */
	dma_stop(cfg->dma_dev, cfg->rx_dma_channel);

	/* Disable receiver */
	regs->UART_CR = UART_CR_RXDIS;

	dev_data->rx_enabled = false;
	dev_data->rx_rd_ptr = 0;

	/* Notify application */
	if (dev_data->async_cb) {
		struct uart_event evt = {
			.type = UART_RX_DISABLED,
		};
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	LOG_INF("RX disabled");

	return 0;
}
#endif /* CONFIG_UART_ASYNC_API */

// =============================================================================
// Driver Initialization
// =============================================================================

static int uart_sam_init(const struct device *dev)
{
	int retval;
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	struct uart_sam_dev_data *const dev_data = dev->data;
	Uart *const uart = cfg->regs;

	dev_data->dev = dev;

	/* Enable UART clock in PMC */
	(void)clock_control_on(SAM_DT_PMC_CONTROLLER, (clock_control_subsys_t)&cfg->clock_cfg);

	/* Connect pins to the peripheral */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	/* Disable all interrupts */
	uart->UART_IDR = 0xFFFFFFFF;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_SAM_ASYNC
	/* Verify DMA device is ready */
	if (!device_is_ready(cfg->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	/* Initialize work items */
	k_work_init_delayable(&dev_data->rx_poll_work, uart_sam_rx_poll_handler);
	k_work_init(&dev_data->tx_complete_work, uart_sam_tx_complete_handler);

	/* Configure TX DMA (one-time setup) */
	if (cfg->tx_dma_channel != 0xFFU) {
		struct dma_block_config dma_blk = {
			.block_size = 1,
			.dest_address = (uintptr_t)(&(uart->UART_THR)),
			.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
			.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		};

		struct dma_config dma_cfg = {
			.channel_direction = MEMORY_TO_PERIPHERAL,
			.source_data_size = 1,
			.dest_data_size = 1,
			.source_burst_length = 1,
			.dest_burst_length = 1,
			.block_count = 1,
			.head_block = &dma_blk,
			.complete_callback_en = 1,
			.dma_callback = uart_sam_dma_tx_done,
			.user_data = dev_data,
			.dma_slot = cfg->tx_dma_request,
		};

		retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel, &dma_cfg);
		if (retval != 0) {
			LOG_ERR("TX DMA config failed: %d", retval);
			return retval;
		}
	}
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

// =============================================================================
// Driver API Structure
// =============================================================================

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
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_SAM_ASYNC
	.callback_set = uart_sam_callback_set,
	.tx = uart_sam_tx,
	.tx_abort = uart_sam_tx_abort,
	.rx_enable = uart_sam_rx_enable,
	.rx_disable = uart_sam_rx_disable,
#endif /* CONFIG_UART_SAM_ASYNC */
};

// =============================================================================
// Device Instantiation Macros
// =============================================================================

#define UART_SAM_DMA_INIT(n)                                                                       \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                        \
        (.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)),   \
         .rx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),  \
         .tx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),  \
         .rx_dma_request = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),    \
         .tx_dma_request = DT_INST_DMAS_CELL_BY_NAME(n, tx, perid)),   \
        ())

#define UART_SAM_DECLARE_CFG(n, IRQ_FUNC_INIT)                                                     \
	static const struct uart_sam_dev_cfg uart##n##_sam_config = {                              \
		.regs = (Uart *)DT_INST_REG_ADDR(n),                                               \
		.clock_cfg = SAM_DT_INST_CLOCK_PMC_CFG(n),                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		IRQ_FUNC_INIT UART_SAM_DMA_INIT(n)};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SAM_CONFIG_FUNC(n)                                                                    \
	static void uart##n##_sam_irq_config_func(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_sam_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};

#define UART_SAM_IRQ_CFG_FUNC_INIT(n) .irq_config_func = uart##n##_sam_irq_config_func,

#else /* !CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_SAM_CONFIG_FUNC(n)
#define UART_SAM_IRQ_CFG_FUNC_INIT(n)

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_SAM_INIT_CFG(n) UART_SAM_DECLARE_CFG(n, UART_SAM_IRQ_CFG_FUNC_INIT(n))

#define UART_SAM_INIT(n)                                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static struct uart_sam_dev_data uart##n##_sam_data = {                                     \
		.baud_rate = DT_INST_PROP(n, current_speed),                                       \
	};                                                                                         \
                                                                                                   \
	UART_SAM_CONFIG_FUNC(n);                                                                   \
	UART_SAM_INIT_CFG(n);                                                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, uart_sam_init, NULL, &uart##n##_sam_data, &uart##n##_sam_config,  \
			      POST_KERNEL, CONFIG_SERIAL_INIT_PRIORITY, &uart_sam_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_SAM_INIT)
