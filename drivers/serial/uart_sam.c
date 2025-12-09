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

#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/atmel_sam_pmc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel_structs.h>

/* Device constant configuration parameters */
struct uart_sam_dev_cfg {
	Uart *regs;
	const struct atmel_sam_pmc_config clock_cfg;
	const struct pinctrl_dev_config *pcfg;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_SAM_ASYNC
	uart_irq_config_func_t	irq_config_func;
#endif
#if CONFIG_UART_SAM_ASYNC
	const struct device *dma_dev;
	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;
#endif
};

/* Device run time data */
struct uart_sam_dev_data {
	uint32_t baud_rate;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_SAM_ASYNC
	uart_irq_callback_user_data_t irq_cb;	/* Interrupt Callback */
	void *irq_cb_data;			/* Interrupt Callback Arg */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#if CONFIG_UART_SAM_ASYNC
	const struct device *dev;
	const struct uart_sam0_dev_cfg *cfg;

	uart_callback_t async_cb;
	void *async_cb_data;

	struct k_work_delayable tx_timeout_work;
	const uint8_t *tx_buf;
	size_t tx_len;

	struct k_work_delayable rx_timeout_work;
	size_t rx_timeout_time;
	size_t rx_timeout_chunk;
	uint32_t rx_timeout_start;
	uint8_t *rx_buf;
	size_t rx_len;
	size_t rx_processed_len;
	uint8_t *rx_next_buf;
	size_t rx_next_len;
	bool rx_waiting_for_irq;
	bool rx_timeout_from_isr;
#endif
};

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

static void uart_sam_dma_tx_done(const struct device *dma_dev, void *arg,
				  uint32_t id, int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	struct uart_sam_dev_data *const dev_data =
		(struct uart_sam_dev_data *const) arg;
	const struct uart_sam_dev_cfg *const cfg = dev_data->cfg;

	/* SercomUsart * const regs = cfg->regs; */

	/* regs->INTENSET.reg = SERCOM_USART_INTENSET_TXC; */
}

static int uart_sam_tx_halt(struct uart_sam_dev_data *dev_data)
{
	const struct uart_sam_dev_cfg *const cfg = dev_data->cfg;
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

static void uart_sam_notify_rx_processed(struct uart_sam_dev_data *dev_data,
					  size_t processed)
{
	if (!dev_data->async_cb) {
		return;
	}

	if (dev_data->rx_processed_len == processed) {
		return;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = dev_data->rx_buf,
			.offset = dev_data->rx_processed_len,
			.len = processed - dev_data->rx_processed_len,
		},
	};

	dev_data->rx_processed_len = processed;

	dev_data->async_cb(dev_data->dev,
			   &evt, dev_data->async_cb_data);
}

static void uart_sam_dma_rx_done(const struct device *dma_dev, void *arg,
				  uint32_t id, int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	struct uart_sam_dev_data *const dev_data =
		(struct uart_sam_dev_data *const)arg;
	const struct device *dev = dev_data->dev;
	const struct uart_sam_dev_cfg *const cfg = dev_data->cfg;
	/* SercomUsart * const regs = cfg->regs; */
	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return;
	}

	uart_sam_notify_rx_processed(dev_data, dev_data->rx_len);

	if (dev_data->async_cb) {
		struct uart_event evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf = {
				.buf = dev_data->rx_buf,
			},
		};

		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	/* No next buffer, so end the transfer */
	if (!dev_data->rx_next_len) {
		dev_data->rx_buf = NULL;
		dev_data->rx_len = 0U;

		if (dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_DISABLED,
			};

			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		irq_unlock(key);
		return;
	}

	dev_data->rx_buf = dev_data->rx_next_buf;
	dev_data->rx_len = dev_data->rx_next_len;
	dev_data->rx_next_buf = NULL;
	dev_data->rx_next_len = 0U;
	dev_data->rx_processed_len = 0U;

	/* dma_reload(cfg->dma_dev, cfg->rx_dma_channel, */
	/* 	   (uint32_t)(&(regs->DATA.reg)), */
	/* 	   (uint32_t)dev_data->rx_buf, dev_data->rx_len); */

	/*
	 * If there should be a timeout, handle starting the DMA in the
	 * ISR, since reception resets it and DMA completion implies
	 * reception.  This also catches the case of DMA completion during
	 * timeout handling.
	 */
	if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
		dev_data->rx_waiting_for_irq = true;
		/* regs->INTENSET.reg = SERCOM_USART_INTENSET_RXC; */
		irq_unlock(key);
		return;
	}

	/* Otherwise, start the transfer immediately. */
	dma_start(cfg->dma_dev, cfg->rx_dma_channel);

	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	dev_data->async_cb(dev, &evt, dev_data->async_cb_data);

	irq_unlock(key);
}

static void uart_sam_rx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_sam_dev_data *dev_data = CONTAINER_OF(dwork,
							   struct uart_sam_dev_data, rx_timeout_work);
	const struct uart_sam_dev_cfg *const cfg = dev_data->cfg;
	/* SercomUsart * const regs = cfg->regs; */
	struct dma_status st;
	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return;
	}

	/*
	 * Stop the DMA transfer and restart the interrupt read
	 * component (so the timeout restarts if there's still data).
	 * However, just ignore it if the transfer has completed (nothing
	 * pending) that means the DMA ISR is already pending, so just let
	 * it handle things instead when we re-enable IRQs.
	 */
	dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
	if (dma_get_status(cfg->dma_dev, cfg->rx_dma_channel,
			   &st) == 0 && st.pending_length == 0U) {
		irq_unlock(key);
		return;
	}

	uint8_t *rx_dma_start = dev_data->rx_buf + dev_data->rx_len -
			     st.pending_length;
	size_t rx_processed = rx_dma_start - dev_data->rx_buf;

	/*
	 * We know we still have space, since the above will catch the
	 * empty buffer, so always restart the transfer.
	 */
	/* dma_reload(cfg->dma_dev, cfg->rx_dma_channel, */
	/* 	   (uint32_t)(&(regs->DATA.reg)), */
	/* 	   (uint32_t)rx_dma_start, */
	/* 	   dev_data->rx_len - rx_processed); */

	dev_data->rx_waiting_for_irq = true;
	/* regs->INTENSET.reg = SERCOM_USART_INTENSET_RXC; */

	/*
	 * Never do a notify on a timeout started from the ISR: timing
	 * granularity means the first timeout can be in the middle
	 * of reception but still have the total elapsed time exhausted.
	 * So we require a timeout chunk with no data seen at all
	 * (i.e. no ISR entry).
	 */
	if (dev_data->rx_timeout_from_isr) {
		dev_data->rx_timeout_from_isr = false;
		k_work_reschedule(&dev_data->rx_timeout_work,
				      K_USEC(dev_data->rx_timeout_chunk));
		irq_unlock(key);
		return;
	}

	uint32_t now = USEC_PER_MSEC * k_uptime_get_32();
	uint32_t elapsed = now - dev_data->rx_timeout_start;

	if (elapsed >= dev_data->rx_timeout_time) {
		/*
		 * No time left, so call the handler, and let the ISR
		 * restart the timeout when it sees data.
		 */
		uart_sam_notify_rx_processed(dev_data, rx_processed);
	} else {
		/*
		 * Still have time left, so start another timeout.
		 */
		uint32_t remaining = MIN(dev_data->rx_timeout_time - elapsed,
				      dev_data->rx_timeout_chunk);

		k_work_reschedule(&dev_data->rx_timeout_work,
				      K_USEC(remaining));
	}

	irq_unlock(key);
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

static void uart_sam_isr(const struct device *dev)
{
	struct uart_sam_dev_data *const dev_data = dev->data;

	if (dev_data->irq_cb) {
		dev_data->irq_cb(dev, dev_data->irq_cb_data);
	}
#if CONFIG_UART_SAM_ASYNC
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	/* SercomUsart * const regs = cfg->regs; */

	/* if (dev_data->tx_len && regs->INTFLAG.bit.TXC) { */
	/* 	regs->INTENCLR.reg = SERCOM_USART_INTENCLR_TXC; */

	/* 	k_work_cancel_delayable(&dev_data->tx_timeout_work); */

	/* 	unsigned int key = irq_lock(); */

	/* 	struct uart_event evt = { */
	/* 		.type = UART_TX_DONE, */
	/* 		.data.tx = { */
	/* 			.buf = dev_data->tx_buf, */
	/* 			.len = dev_data->tx_len, */
	/* 		}, */
	/* 	}; */

	/* 	dev_data->tx_buf = NULL; */
	/* 	dev_data->tx_len = 0U; */

	/* 	if (evt.data.tx.len != 0U && dev_data->async_cb) { */
	/* 		dev_data->async_cb(dev, &evt, dev_data->async_cb_data); */
	/* 	} */

	/* 	irq_unlock(key); */
	/* } */

	/* if (dev_data->rx_len && regs->INTFLAG.bit.RXC && */
	/*     dev_data->rx_waiting_for_irq) { */
	/* 	dev_data->rx_waiting_for_irq = false; */
	/* 	regs->INTENCLR.reg = SERCOM_USART_INTENCLR_RXC; */

	/* 	/\* Receive started, so request the next buffer *\/ */
	/* 	if (dev_data->rx_next_len == 0U && dev_data->async_cb) { */
	/* 		struct uart_event evt = { */
	/* 			.type = UART_RX_BUF_REQUEST, */
	/* 		}; */

	/* 		dev_data->async_cb(dev, &evt, dev_data->async_cb_data); */
	/* 	} */

	/* 	/\* */
	/* 	 * If we have a timeout, restart the time remaining whenever */
	/* 	 * we see data. */
	/* 	 *\/ */
	/* 	if (dev_data->rx_timeout_time != SYS_FOREVER_US) { */
	/* 		dev_data->rx_timeout_from_isr = true; */
	/* 		dev_data->rx_timeout_start = USEC_PER_MSEC * k_uptime_get_32(); */
	/* 		k_work_reschedule(&dev_data->rx_timeout_work, */
	/* 				      K_USEC(dev_data->rx_timeout_chunk)); */
	/* 	} */

	/* 	/\* DMA will read the currently ready byte out *\/ */
	/* 	dma_start(cfg->dma_dev, cfg->rx_dma_channel); */
	/* } */
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
	/* SercomUsart *regs = cfg->regs; */
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

	irq_unlock(key);

	/* retval = dma_reload(cfg->dma_dev, cfg->tx_dma_channel, (uint32_t)buf, */
	/* 		    (uint32_t)(&(regs->DATA.reg)), len); */
	if (retval != 0U) {
		return retval;
	}

	if (timeout != SYS_FOREVER_US) {
		k_work_reschedule(&dev_data->tx_timeout_work,
				      K_USEC(timeout));
	}

	return dma_start(cfg->dma_dev, cfg->tx_dma_channel);
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

static int uart_sam_rx_enable(const struct device *dev, uint8_t *buf,
			       size_t len,
			       int32_t timeout)
{
	struct uart_sam_dev_data *const dev_data = dev->data;
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	/* SercomUsart *regs = cfg->regs; */
	int retval;

	if (cfg->rx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	if (dev_data->rx_len != 0U) {
		retval = -EBUSY;
		goto err;
	}

	/* Read off anything that was already there */
	/* while (regs->INTFLAG.bit.RXC) { */
	/* 	char discard = regs->DATA.reg; */

	/* 	(void)discard; */
	/* } */

	/* retval = dma_reload(cfg->dma_dev, cfg->rx_dma_channel, */
	/* 		    (uint32_t)(&(regs->DATA.reg)), */
	/* 		    (uint32_t)buf, len); */
	if (retval != 0) {
		return retval;
	}

	dev_data->rx_buf = buf;
	dev_data->rx_len = len;
	dev_data->rx_processed_len = 0U;
	dev_data->rx_waiting_for_irq = true;
	dev_data->rx_timeout_from_isr = true;
	dev_data->rx_timeout_time = timeout;
	dev_data->rx_timeout_chunk = MAX(timeout / 4U, 1);

	/* regs->INTENSET.reg = SERCOM_USART_INTENSET_RXC; */

	irq_unlock(key);
	return 0;

err:
	irq_unlock(key);
	return retval;
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
	struct uart_sam_dev_data *const dev_data = dev->data;
	const struct uart_sam_dev_cfg *const cfg = dev->config;
	/* SercomUsart * const regs = cfg->regs; */
	struct dma_status st;

	k_work_cancel_delayable(&dev_data->rx_timeout_work);

	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return -EINVAL;
	}

	/* regs->INTENCLR.reg = SERCOM_USART_INTENCLR_RXC; */
	dma_stop(cfg->dma_dev, cfg->rx_dma_channel);


	if (dma_get_status(cfg->dma_dev, cfg->rx_dma_channel,
			   &st) == 0 && st.pending_length != 0U) {
		size_t rx_processed = dev_data->rx_len - st.pending_length;

		uart_sam_notify_rx_processed(dev_data, rx_processed);
	}

	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf = {
			.buf = dev_data->rx_buf,
		},
	};

	dev_data->rx_buf = NULL;
	dev_data->rx_len = 0U;

	if (dev_data->async_cb) {
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	if (dev_data->rx_next_len) {
		struct uart_event next_evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf = {
				.buf = dev_data->rx_next_buf,
			},
		};

		dev_data->rx_next_buf = NULL;
		dev_data->rx_next_len = 0U;

		if (dev_data->async_cb) {
			dev_data->async_cb(dev, &next_evt, dev_data->async_cb_data);
		}
	}

	evt.type = UART_RX_DISABLED;
	if (dev_data->async_cb) {
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	irq_unlock(key);

	return 0;
}

#endif

static int uart_sam_init(const struct device *dev)
{
	int retval;

	const struct uart_sam_dev_cfg *const cfg = dev->config;

	struct uart_sam_dev_data *const dev_data = dev->data;

	Uart * const uart = cfg->regs;

	/* Enable UART clock in PMC */
	(void)clock_control_on(SAM_DT_PMC_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clock_cfg);

	/* Connect pins to the peripheral */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	/* Disable Interrupts */
	uart->UART_IDR = 0xFFFFFFFF;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_SAM_ASYNC
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_SAM_ASYNC
	dev_data->dev = dev;
	dev_data->cfg = cfg;
	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}

	k_work_init_delayable(&dev_data->tx_timeout_work, uart_sam_tx_timeout);
	k_work_init_delayable(&dev_data->rx_timeout_work, uart_sam_rx_timeout);

	if (cfg->tx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = { 0 };
		struct dma_block_config dma_blk = { 0 };

		/* dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL; */
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = uart_sam_dma_tx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->tx_dma_request;

		dma_blk.block_size = 1;
		/* dma_blk.dest_address = (uint32_t)(&(usart->DATA.reg)); */
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel,
				    &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

	if (cfg->rx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = { 0 };
		struct dma_block_config dma_blk = { 0 };

		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = uart_sam_dma_rx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->rx_dma_request;

		dma_blk.block_size = 1;
		/* dma_blk.source_address = (uint32_t)(&(usart->DATA.reg)); */
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		retval = dma_config(cfg->dma_dev, cfg->rx_dma_channel,
				    &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

#endif

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

#define UART_SAM_DECLARE_CFG(n, IRQ_FUNC_INIT)				\
	static const struct uart_sam_dev_cfg uart##n##_sam_config = {	\
		.regs = (Uart *)DT_INST_REG_ADDR(n),			\
		.clock_cfg = SAM_DT_INST_CLOCK_PMC_CFG(n),		\
									\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
									\
		IRQ_FUNC_INIT						\
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SAM_CONFIG_FUNC(n)						\
	static void uart##n##_sam_irq_config_func(const struct device *port)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    uart_sam_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}
#define UART_SAM_IRQ_CFG_FUNC_INIT(n)					\
	.irq_config_func = uart##n##_sam_irq_config_func
#define UART_SAM_INIT_CFG(n)						\
	UART_SAM_DECLARE_CFG(n, UART_SAM_IRQ_CFG_FUNC_INIT(n))
#else
#define UART_SAM_CONFIG_FUNC(n)
#define UART_SAM_IRQ_CFG_FUNC_INIT
#define UART_SAM_INIT_CFG(n)						\
	UART_SAM_DECLARE_CFG(n, UART_SAM_IRQ_CFG_FUNC_INIT)
#endif

#if CONFIG_UART_SAM_ASYNC
#define UART_SAM_DMA_CHANNELS(n)					\
	.dma_dev = DEVICE_DT_GET(ATMEL_SAM_DT_INST_DMA_CTLR(n, tx)),	\
	.tx_dma_request = ATMEL_SAM_DT_INST_DMA_TRIGSRC(n, tx),	\
	.tx_dma_channel = ATMEL_SAM_DT_INST_DMA_CHANNEL(n, tx),	\
	.rx_dma_request = ATMEL_SAM_DT_INST_DMA_TRIGSRC(n, rx),	\
	.rx_dma_channel = ATMEL_SAM_DT_INST_DMA_CHANNEL(n, rx),
#else
#define UART_SAM_DMA_CHANNELS(n)
#endif

#define UART_SAM_INIT(n)						\
	PINCTRL_DT_INST_DEFINE(n);					\
	static struct uart_sam_dev_data uart##n##_sam_data = {		\
		.baud_rate = DT_INST_PROP(n, current_speed),		\
	};								\
									\
	static const struct uart_sam_dev_cfg uart##n##_sam_config;	\
									\
	DEVICE_DT_INST_DEFINE(n, uart_sam_init,				\
			    NULL, &uart##n##_sam_data,			\
			    &uart##n##_sam_config, PRE_KERNEL_1,	\
			    CONFIG_SERIAL_INIT_PRIORITY,		\
			    &uart_sam_driver_api);			\
									\
	UART_SAM_CONFIG_FUNC(n)						\
									\
	UART_SAM_INIT_CFG(n);

DT_INST_FOREACH_STATUS_OKAY(UART_SAM_INIT)
