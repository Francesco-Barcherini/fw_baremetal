/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#include "reg.h"
#include <inttypes.h>

/*
 * Copyright (c) 2021 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_scif

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

static uint8_t uart_rcar_read_8(const struct device *dev, uint32_t offs)
{
	return sys_read8(DEVICE_MMIO_GET(dev) + offs);
}

static void uart_rcar_write_8(const struct device *dev,
			      uint32_t offs, uint8_t value)
{
	sys_write8(value, DEVICE_MMIO_GET(dev) + offs);
}

static uint16_t uart_rcar_read_16(const struct device *dev,
				  uint32_t offs)
{
	return sys_read16(DEVICE_MMIO_GET(dev) + offs);
}

static void uart_rcar_write_16(const struct device *dev,
			       uint32_t offs, uint16_t value)
{
	sys_write16(value, DEVICE_MMIO_GET(dev) + offs);
}

static void uart_rcar_set_baudrate(const struct device *dev,
				   uint32_t baud_rate)
{
	struct uart_rcar_data *data = dev->data;
	const struct uart_rcar_cfg *cfg = dev->config;
	uint8_t reg_val;

	if (cfg->is_hscif) {
		reg_val = data->clk_rate / (2 * (HSSRR_SRCYC_DEF_VAL + 1) * baud_rate) - 1;
	} else {
		reg_val = ((data->clk_rate + 16 * baud_rate) / (32 * baud_rate) - 1);
	}
	uart_rcar_write_8(dev, SCBRR, reg_val);
}

static int uart_rcar_poll_in(const struct device *dev, unsigned char *p_char)
{
	struct uart_rcar_data *data = dev->data;
	uint16_t reg_val;
	int ret = 0;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Receive FIFO empty */
	if (!((uart_rcar_read_16(dev, SCFSR)) & SCFSR_RDF)) {
		ret = -1;
		goto unlock;
	}

	*p_char = uart_rcar_read_8(dev, SCFRDR);

	reg_val = uart_rcar_read_16(dev, SCFSR);
	reg_val &= ~SCFSR_RDF;
	uart_rcar_write_16(dev, SCFSR, reg_val);

unlock:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static void uart_rcar_poll_out(const struct device *dev, unsigned char out_char)
{
	struct uart_rcar_data *data = dev->data;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Wait for empty space in transmit FIFO */
	while (!(uart_rcar_read_16(dev, SCFSR) & SCFSR_TDFE)) {
	}

	uart_rcar_write_8(dev, SCFTDR, out_char);

	reg_val = uart_rcar_read_16(dev, SCFSR);
	reg_val &= ~(SCFSR_TDFE | SCFSR_TEND);
	uart_rcar_write_16(dev, SCFSR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_rcar_configure(const struct device *dev,
			       const struct uart_config *cfg)
{
	struct uart_rcar_data *data = dev->data;
	const struct uart_rcar_cfg *cfg_drv = dev->config;

	uint16_t reg_val;
	k_spinlock_key_t key;

	if (cfg->parity != UART_CFG_PARITY_NONE ||
	    cfg->stop_bits != UART_CFG_STOP_BITS_1 ||
	    cfg->data_bits != UART_CFG_DATA_BITS_8 ||
	    cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

	/* Disable Transmit and Receive */
	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_TE | SCSCR_RE);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	/* Emptying Transmit and Receive FIFO */
	reg_val = uart_rcar_read_16(dev, SCFCR);
	reg_val |= (SCFCR_TFRST | SCFCR_RFRST);
	uart_rcar_write_16(dev, SCFCR, reg_val);

	/* Resetting Errors Registers */
	reg_val = uart_rcar_read_16(dev, SCFSR);
	reg_val &= ~(SCFSR_ER | SCFSR_DR | SCFSR_BRK | SCFSR_RDF);
	uart_rcar_write_16(dev, SCFSR, reg_val);

	reg_val = uart_rcar_read_16(dev, SCLSR);
	reg_val &= ~(SCLSR_TO | SCLSR_ORER);
	uart_rcar_write_16(dev, SCLSR, reg_val);

	/* Select internal clock */
	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_CKE1 | SCSCR_CKE0);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	/* Serial Configuration (8N1) & Clock divider selection */
	reg_val = uart_rcar_read_16(dev, SCSMR);
	reg_val &= ~(SCSMR_C_A | SCSMR_CHR | SCSMR_PE | SCSMR_O_E | SCSMR_STOP |
		     SCSMR_CKS1 | SCSMR_CKS0);
	uart_rcar_write_16(dev, SCSMR, reg_val);

	if (cfg_drv->is_hscif) {
		/* TODO: calculate the optimal sampling and bit rates based on error rate */
		uart_rcar_write_16(dev, HSSRR, HSSRR_SRE | HSSRR_SRCYC_DEF_VAL);
	}

	/* Set baudrate */
	uart_rcar_set_baudrate(dev, cfg->baudrate);

	/* FIFOs data count trigger configuration */
	reg_val = uart_rcar_read_16(dev, SCFCR);
	reg_val &= ~(SCFCR_RTRG1 | SCFCR_RTRG0 | SCFCR_TTRG1 | SCFCR_TTRG0 |
		     SCFCR_MCE | SCFCR_TFRST | SCFCR_RFRST);
	uart_rcar_write_16(dev, SCFCR, reg_val);

	/* Enable Transmit & Receive + disable Interrupts */
	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val |= (SCSCR_TE | SCSCR_RE);
	reg_val &= ~(SCSCR_TIE | SCSCR_RIE | SCSCR_TEIE | SCSCR_REIE |
		     SCSCR_TOIE);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	data->current_config = *cfg;

	k_spin_unlock(&data->lock, key);

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_rcar_config_get(const struct device *dev,
				struct uart_config *cfg)
{
	struct uart_rcar_data *data = dev->data;

	*cfg = data->current_config;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_rcar_init(const struct device *dev)
{
	const struct uart_rcar_cfg *config = dev->config;
	struct uart_rcar_data *data = dev->data;
	int ret;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev,
			       (clock_control_subsys_t)&config->mod_clk);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev,
				     (clock_control_subsys_t)&config->bus_clk,
				     &data->clk_rate);
	if (ret < 0) {
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	ret = uart_rcar_configure(dev, &data->current_config);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static bool uart_rcar_irq_is_enabled(const struct device *dev,
				     uint32_t irq)
{
	return !!(uart_rcar_read_16(dev, SCSCR) & irq);
}

static int uart_rcar_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data,
			       int len)
{
	struct uart_rcar_data *data = dev->data;
	int num_tx = 0;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while (((len - num_tx) > 0) &&
	       (uart_rcar_read_16(dev, SCFSR) & SCFSR_TDFE)) {
		/* Send current byte */
		uart_rcar_write_8(dev, SCFTDR, tx_data[num_tx]);

		reg_val = uart_rcar_read_16(dev, SCFSR);
		reg_val &= ~(SCFSR_TDFE | SCFSR_TEND);
		uart_rcar_write_16(dev, SCFSR, reg_val);

		num_tx++;
	}

	k_spin_unlock(&data->lock, key);

	return num_tx;
}

static int uart_rcar_fifo_read(const struct device *dev, uint8_t *rx_data,
			       const int size)
{
	struct uart_rcar_data *data = dev->data;
	int num_rx = 0;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while (((size - num_rx) > 0) &&
	       (uart_rcar_read_16(dev, SCFSR) & SCFSR_RDF)) {
		/* Receive current byte */
		rx_data[num_rx++] = uart_rcar_read_8(dev, SCFRDR);

		reg_val = uart_rcar_read_16(dev, SCFSR);
		reg_val &= ~(SCFSR_RDF);
		uart_rcar_write_16(dev, SCFSR, reg_val);

	}

	k_spin_unlock(&data->lock, key);

	return num_rx;
}

static void uart_rcar_irq_tx_enable(const struct device *dev)
{
	struct uart_rcar_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val |= (SCSCR_TIE);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_rcar_irq_tx_disable(const struct device *dev)
{
	struct uart_rcar_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_TIE);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_rcar_irq_tx_ready(const struct device *dev)
{
	return !!(uart_rcar_read_16(dev, SCFSR) & SCFSR_TDFE);
}

static void uart_rcar_irq_rx_enable(const struct device *dev)
{
	struct uart_rcar_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val |= (SCSCR_RIE);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_rcar_irq_rx_disable(const struct device *dev)
{
	struct uart_rcar_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_RIE);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_rcar_irq_rx_ready(const struct device *dev)
{
	return !!(uart_rcar_read_16(dev, SCFSR) & SCFSR_RDF);
}

static void uart_rcar_irq_err_enable(const struct device *dev)
{
	struct uart_rcar_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val |= (SCSCR_REIE);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_rcar_irq_err_disable(const struct device *dev)
{
	struct uart_rcar_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_rcar_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_REIE);
	uart_rcar_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_rcar_irq_is_pending(const struct device *dev)
{
	return (uart_rcar_irq_rx_ready(dev) && uart_rcar_irq_is_enabled(dev, SCSCR_RIE)) ||
	       (uart_rcar_irq_tx_ready(dev) && uart_rcar_irq_is_enabled(dev, SCSCR_TIE));
}

static int uart_rcar_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_rcar_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_rcar_data *data = dev->data;

	data->callback = cb;
	data->cb_data = cb_data;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 */
void uart_rcar_isr(const struct device *dev)
{
	struct uart_rcar_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, data->cb_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_rcar_driver_api = {
	.poll_in = uart_rcar_poll_in,
	.poll_out = uart_rcar_poll_out,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_rcar_configure,
	.config_get = uart_rcar_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_rcar_fifo_fill,
	.fifo_read = uart_rcar_fifo_read,
	.irq_tx_enable = uart_rcar_irq_tx_enable,
	.irq_tx_disable = uart_rcar_irq_tx_disable,
	.irq_tx_ready = uart_rcar_irq_tx_ready,
	.irq_rx_enable = uart_rcar_irq_rx_enable,
	.irq_rx_disable = uart_rcar_irq_rx_disable,
	.irq_rx_ready = uart_rcar_irq_rx_ready,
	.irq_err_enable = uart_rcar_irq_err_enable,
	.irq_err_disable = uart_rcar_irq_err_disable,
	.irq_is_pending = uart_rcar_irq_is_pending,
	.irq_update = uart_rcar_irq_update,
	.irq_callback_set = uart_rcar_irq_callback_set,
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */
};

/* Device Instantiation */
#define UART_RCAR_DECLARE_CFG(n, IRQ_FUNC_INIT, compat)					\
	PINCTRL_DT_INST_DEFINE(n);							\
	static const struct uart_rcar_cfg uart_rcar_cfg_##compat##n = {			\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),					\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),			\
		.mod_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),		\
		.mod_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),		\
		.bus_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),		\
		.bus_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		.is_hscif = DT_INST_NODE_HAS_COMPAT(n, renesas_rcar_hscif),	        \
		IRQ_FUNC_INIT								\
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_RCAR_CONFIG_FUNC(n, compat)					 \
	static void irq_config_func_##compat##n(const struct device *dev)	 \
	{									 \
		IRQ_CONNECT(DT_INST_IRQN(n),					 \
			    DT_INST_IRQ(n, priority),				 \
			    uart_rcar_isr,					 \
			    DEVICE_DT_INST_GET(n), 0);				 \
										 \
		irq_enable(DT_INST_IRQN(n));					 \
	}
#define UART_RCAR_IRQ_CFG_FUNC_INIT(n, compat) \
	.irq_config_func = irq_config_func_##compat##n
#define UART_RCAR_INIT_CFG(n, compat) \
	UART_RCAR_DECLARE_CFG(n, UART_RCAR_IRQ_CFG_FUNC_INIT(n, compat), compat)
#else
#define UART_RCAR_CONFIG_FUNC(n, compat)
#define UART_RCAR_IRQ_CFG_FUNC_INIT
#define UART_RCAR_INIT_CFG(n, compat) \
	UART_RCAR_DECLARE_CFG(n, UART_RCAR_IRQ_CFG_FUNC_INIT, compat)
#endif

#define UART_RCAR_INIT(n, compat)						\
	static struct uart_rcar_data uart_rcar_data_##compat##n = {		\
		.current_config = {						\
			.baudrate = DT_INST_PROP(n, current_speed),		\
			.parity = UART_CFG_PARITY_NONE,				\
			.stop_bits = UART_CFG_STOP_BITS_1,			\
			.data_bits = UART_CFG_DATA_BITS_8,			\
			.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,			\
		},								\
	};									\
										\
	static const struct uart_rcar_cfg uart_rcar_cfg_##compat##n;		\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			      uart_rcar_init,					\
			      NULL,						\
			      &uart_rcar_data_##compat##n,			\
			      &uart_rcar_cfg_##compat##n,			\
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,	\
			      &uart_rcar_driver_api);				\
										\
	UART_RCAR_CONFIG_FUNC(n, compat)					\
										\
	UART_RCAR_INIT_CFG(n, compat);

DT_INST_FOREACH_STATUS_OKAY_VARGS(UART_RCAR_INIT, DT_DRV_COMPAT)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT renesas_rcar_hscif

DT_INST_FOREACH_STATUS_OKAY_VARGS(UART_RCAR_INIT, DT_DRV_COMPAT)


// inline function to swap two numbers
static inline void swap(char *x, char *y) {
  char t = *x; *x = *y; *y = t;
}

// function to reverse buffer[i..j]
char* reverse(char *buffer, int i, int j)
{
  while (i < j)
    swap(&buffer[i++], &buffer[j--]);

  return buffer;
}

// Iterative function to implement itoa() function in C
void _itoa(char* buffer, int base, uint64_t value)
{
  if(base == 'd'){
    base = 10;
  } else if(base == 'x'){
    base = 16;
  } else {
    return;
  }

  // consider absolute value of number
  // uint64_t n = (uint64_t)abs(value);
  uint64_t n  = value;

  int i = 0;
  while (n)
  {
    uint64_t r = n % base;

    if (r >= 10) 
      buffer[i++] = 65 + (r - 10);
    else
      buffer[i++] = 48 + r;

    n = n / base;
  }

  // if number is 0
  if (i == 0)
    buffer[i++] = '0';

  // If base is 10 and value is negative, the resulting string 
  // is preceded with a minus sign (-)
  // With any other base, value is always considered unsigned
  if (value < 0 && base == 10)
    buffer[i++] = '-';

  buffer[i] = '\0'; // null terminate string

  // reverse the string and return it
  buffer = reverse(buffer, 0, i - 1);
}

int put(const char str)
{
  if(0xA == str){
    *((uint32_t *) UART_BASE_REG) = 0xD;
  }
  *((unsigned int *) UART_BASE_REG) = str;
  return 0;
}

int puts_no_lock(const char *str)
{

  while (*str)
    put(*str++);

  return 0;
}

int puts(const char *str)
{

  while (*str)
    put(*str++);

  return 0;
}

