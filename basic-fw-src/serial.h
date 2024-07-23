/* 
 * This file is part of CLARE-BasicFirmware
 * 
 * Author: Accelerat S.r.l.
 * 
 * This program is confidential.
 * Any unauthorized use of this file via any medium is strictly prohibited.
 * 
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <inttypes.h>


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

struct uart_rcar_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *clock_dev;
	struct rcar_cpg_clk mod_clk;
	struct rcar_cpg_clk bus_clk;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
	bool is_hscif;
};

struct uart_rcar_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct uart_config current_config;
	uint32_t clk_rate;
	struct k_spinlock lock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

/* Registers */
#define SCSMR           0x00    /* Serial Mode Register */
#define SCBRR           0x04    /* Bit Rate Register */
#define SCSCR           0x08    /* Serial Control Register */
#define SCFTDR          0x0c    /* Transmit FIFO Data Register */
#define SCFSR           0x10    /* Serial Status Register */
#define SCFRDR          0x14    /* Receive FIFO Data Register */
#define SCFCR           0x18    /* FIFO Control Register */
#define SCFDR           0x1c    /* FIFO Data Count Register */
#define SCSPTR          0x20    /* Serial Port Register */
#define SCLSR           0x24    /* Line Status Register */
#define DL              0x30    /* Frequency Division Register */
#define CKS             0x34    /* Clock Select Register */
#define HSSRR           0x40    /* Sampling Rate Register */

/* SCSMR (Serial Mode Register) */
#define SCSMR_C_A       BIT(7)  /* Communication Mode */
#define SCSMR_CHR       BIT(6)  /* 7-bit Character Length */
#define SCSMR_PE        BIT(5)  /* Parity Enable */
#define SCSMR_O_E       BIT(4)  /* Odd Parity */
#define SCSMR_STOP      BIT(3)  /* Stop Bit Length */
#define SCSMR_CKS1      BIT(1)  /* Clock Select 1 */
#define SCSMR_CKS0      BIT(0)  /* Clock Select 0 */

/* SCSCR (Serial Control Register) */
#define SCSCR_TEIE      BIT(11) /* Transmit End Interrupt Enable */
#define SCSCR_TIE       BIT(7)  /* Transmit Interrupt Enable */
#define SCSCR_RIE       BIT(6)  /* Receive Interrupt Enable */
#define SCSCR_TE        BIT(5)  /* Transmit Enable */
#define SCSCR_RE        BIT(4)  /* Receive Enable */
#define SCSCR_REIE      BIT(3)  /* Receive Error Interrupt Enable */
#define SCSCR_TOIE      BIT(2)  /* Timeout Interrupt Enable */
#define SCSCR_CKE1      BIT(1)  /* Clock Enable 1 */
#define SCSCR_CKE0      BIT(0)  /* Clock Enable 0 */

/* SCFCR (FIFO Control Register) */
#define SCFCR_RTRG1     BIT(7)  /* Receive FIFO Data Count Trigger 1 */
#define SCFCR_RTRG0     BIT(6)  /* Receive FIFO Data Count Trigger 0 */
#define SCFCR_TTRG1     BIT(5)  /* Transmit FIFO Data Count Trigger 1 */
#define SCFCR_TTRG0     BIT(4)  /* Transmit FIFO Data Count Trigger 0 */
#define SCFCR_MCE       BIT(3)  /* Modem Control Enable */
#define SCFCR_TFRST     BIT(2)  /* Transmit FIFO Data Register Reset */
#define SCFCR_RFRST     BIT(1)  /* Receive FIFO Data Register Reset */
#define SCFCR_LOOP      BIT(0)  /* Loopback Test */

/* SCFSR (Serial Status Register) */
#define SCFSR_PER3      BIT(15) /* Parity Error Count 3 */
#define SCFSR_PER2      BIT(14) /* Parity Error Count 2 */
#define SCFSR_PER1      BIT(13) /* Parity Error Count 1 */
#define SCFSR_PER0      BIT(12) /* Parity Error Count 0 */
#define SCFSR_FER3      BIT(11) /* Framing Error Count 3 */
#define SCFSR_FER2      BIT(10) /* Framing Error Count 2 */
#define SCFSR_FER_1     BIT(9)  /* Framing Error Count 1 */
#define SCFSR_FER0      BIT(8)  /* Framing Error Count 0 */
#define SCFSR_ER        BIT(7)  /* Receive Error */
#define SCFSR_TEND      BIT(6)  /* Transmission ended */
#define SCFSR_TDFE      BIT(5)  /* Transmit FIFO Data Empty */
#define SCFSR_BRK       BIT(4)  /* Break Detect */
#define SCFSR_FER       BIT(3)  /* Framing Error */
#define SCFSR_PER       BIT(2)  /* Parity Error */
#define SCFSR_RDF       BIT(1)  /* Receive FIFO Data Full */
#define SCFSR_DR        BIT(0)  /* Receive Data Ready */

/* SCLSR (Line Status Register) on (H)SCIF */
#define SCLSR_TO        BIT(2)  /* Timeout */
#define SCLSR_ORER      BIT(0)  /* Overrun Error */

/* HSSRR (Sampling Rate Register) */
#define HSSRR_SRE            BIT(15)      /* Sampling Rate Register Enable */
#define HSSRR_SRCYC_DEF_VAL  0x7          /* Sampling rate default value */

static uint8_t uart_rcar_read_8(const struct device *dev, uint32_t offs);

static void uart_rcar_write_8(const struct device *dev,
			      uint32_t offs, uint8_t value);

static uint16_t uart_rcar_read_16(const struct device *dev,
				  uint32_t offs);

static void uart_rcar_write_16(const struct device *dev,
			       uint32_t offs, uint16_t value);

static void uart_rcar_set_baudrate(const struct device *dev,
				   uint32_t baud_rate);

static int uart_rcar_poll_in(const struct device *dev, unsigned char *p_char);

static void uart_rcar_poll_out(const struct device *dev, unsigned char out_char);

static int uart_rcar_configure(const struct device *dev,
			       const struct uart_config *cfg);

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_rcar_config_get(const struct device *dev,
				struct uart_config *cfg);
{
	struct uart_rcar_data *data = dev->data;

	*cfg = data->current_config;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_rcar_init(const struct device *dev);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static bool uart_rcar_irq_is_enabled(const struct device *dev,
				     uint32_t irq);

static int uart_rcar_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data,
			       int len);

static int uart_rcar_fifo_read(const struct device *dev, uint8_t *rx_data,
			       const int size);

static void uart_rcar_irq_tx_enable(const struct device *dev);

static void uart_rcar_irq_tx_disable(const struct device *dev);

static int uart_rcar_irq_tx_ready(const struct device *dev);

static void uart_rcar_irq_rx_enable(const struct device *dev);

static void uart_rcar_irq_rx_disable(const struct device *dev);

static int uart_rcar_irq_rx_ready(const struct device *dev);

static void uart_rcar_irq_err_enable(const struct device *dev);

static void uart_rcar_irq_err_disable(const struct device *dev);

static int uart_rcar_irq_is_pending(const struct device *dev);

static int uart_rcar_irq_update(const struct device *dev);

static void uart_rcar_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data);

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 */
void uart_rcar_isr(const struct device *dev);

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


void _itoa(char* buffer, int base, uint64_t value);

int puts_no_lock(const char *str);

int puts(const char *str);

int put(const char str);

#endif
