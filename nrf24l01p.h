// SPDX-License-Identifier: GPL-2.0
/*            
 * Copyright (C) 2022 Jagath Jog J <jagathjog1996@gmail.com@gmail.com>
 *
 */

#ifndef _NRF24L01P_H
#define _NRF24L01P_H

#include <linux/bits.h>
#include <linux/regmap.h>

#define NRF24_CLASS_NAME	"radio"
#define FIFO_SIZE		65536

/* Register Map */
#define NRF24L0P_FEATURE_REG	0x1d
/* nRF24L01 Register map */

#define NRF24L0P_CONFIG			0x00
#define NRF24L0P_EN_AA			0x01
#define NRF24L0P_EN_RXADDR		0x02
#define NRF24L0P_SETUP_AW		0x03
#define NRF24L0P_SETUP_RETR		0x04
#define NRF24L0P_RF_CH			0x05
#define NRF24L0P_RF_SETUP		0x06
#define NRF24L0P_STATUS			0x07
#define NRF24L0P_OBSERVE_TX		0x08
#define	NRF24L0P_CD			0x09
#define NRF24L0P_RX_ADDR_P0		0x0A
#define NRF24L0P_RX_ADDR_P1		0x0B
#define NRF24L0P_RX_ADDR_P2		0x0C
#define NRF24L0P_RX_ADDR_P3		0x0D
#define NRF24L0P_RX_ADDR_P4		0x0E
#define NRF24L0P_RX_ADDR_P5		0x0F
#define NRF24L0P_TX_ADDR			0x10
#define NRF24L0P_RX_PW_P0		0x11
#define NRF24L0P_RX_PW_P1		0x12
#define NRF24L0P_RX_PW_P2		0x13
#define NRF24L0P_RX_PW_P3		0x14
#define NRF24L0P_RX_PW_P4		0x15
#define NRF24L0P_RX_PW_P5		0x16
#define NRF24L0P_FIFO_STATUS		0x17
#define NRF24L0P_DYNPD			0x1C
#define NRF24L0P_FEATURE			0x1D

/* nRF24L01 Instruction Definitions */
#define NRF24L0P_W_REGISTER		0x20
#define NRF24L0P_R_RX_PL_WID		0x60
#define NRF24L0P_R_RX_PAYLOAD		0x61
#define NRF24L0P_W_TX_PAYLOAD		0xA0
#define NRF24L0P_W_ACK_PAYLOAD		0xA8
#define NRF24L0P_W_TX_PAYLOAD_NOACK	0xB0
#define NRF24L0P_FLUSH_TX		0xE1
#define NRF24L0P_FLUSH_RX		0xE2
#define NRF24L0P_REUSE_TX_PL		0xE3
#define NRF24L0P_LOCK_UNLOCK		0x50
#define NRF24L0P_NOP			0xFF

/* CONFIG 0x00 */
#define NRF24L0P_MASK_RX_DR		0x40
#define NRF24L0P_MASK_TX_DS		0x20
#define NRF24L0P_MASK_MAX_RT		0x10
#define NRF24L0P_EN_CRC			0x08
#define NRF24L0P_CRC_MSK		GENMASK(3, 2)
#define NRF24L0P_CRCO			0x04
#define NRF24L0P_PWR_UP			0x02
#define NRF24L0P_PRIM_RX		0x01

/* RF_SETUP 0x06 */
#define NRF24L0P_RF_DR_LO		0x20
#define NRF24L0P_PLL_LOCK		0x10
#define NRF24L0P_RF_DR_HI		0x08
#define NRF24L0P_RF_PWR1			0x04
#define NRF24L0P_RF_PWR0			0x02

/* STATUS 0x07 */
#define NRF24L0P_RX_DR			0x40
#define NRF24L0P_TX_DS			0x20
#define NRF24L0P_MAX_RT			0x10
#define NRF24L0P_TX_FULL			0x01

/* FEATURE 0x1D */
#define NRF24L0P_EN_DPL			0x04
#define NRF24L0P_EN_ACK_PAY		0x02
#define NRF24L0P_EN_DYN_ACK		0x01

#define NRF24L0P_PLOAD_MAX		32

enum nrf24_mode {
	NRF24_MODE_TX,
	NRF24_MODE_RX
};

extern struct attribute *nrf24_attrs[];
extern struct attribute *nrf24_pipe_attrs[];

struct nrf24l0_pipe {
	dev_t			devt;
	struct device		*dev;
	struct cdev		cdev;

	u64			address;
	u8			ack;
	ssize_t			plw;

	DECLARE_KFIFO(rx_fifo, u8, FIFO_SIZE);
	struct mutex		rx_fifo_mutex;
	wait_queue_head_t	read_wait_queue;
	wait_queue_head_t	write_wait_queue;
	struct task_struct	*tx_task_struct;

};

struct nrf24l0_data {
        struct device parent;
        struct device dev;
	struct regmap *regmap;
	int irq;

	struct work_struct	isr_work;
	struct work_struct	rx_work;

	spinlock_t		lock;
	STRUCT_KFIFO_REC_1(FIFO_SIZE) tx_fifo;
	struct task_struct	*tx_task_struct;
	wait_queue_head_t	tx_wait_queue;
	wait_queue_head_t	tx_done_wait_queue;

	struct gpio_desc	*ce;
	struct nrf24l0_pipe *pipe0;
	bool			rx_active;
};

struct nrf24l0_tx_data {
	u8			size;
	u8			pload[NRF24L0P_PLOAD_MAX];
};

int nrf24l01p_configure(struct nrf24l0_data *nrf24l01p);
int nrf24l01p_get_status(struct nrf24l0_data *nrf24l01p, uint8_t *status);
int nrf24l01p_clear_irq(struct nrf24l0_data *nrf24l01p, u8 irq);
int nrf24l01p_is_rx_fifo_empty(struct nrf24l0_data *nrf24l01p);
int nrf24l01_get_rx_payload(struct nrf24l0_data *nrf24l01p, uint8_t *pload);
int nrf24l01p_set_mode(struct nrf24l0_data *nrf24l01p, enum nrf24_mode mode);
int nrf24l01p_write_tx_pload(struct nrf24l0_data *nrf24l01p, uint8_t *pload, uint8_t size);
int nrf24_print_status(struct nrf24l0_data *nrf24l01p);
int nrf24l01p_read_register(struct nrf24l0_data *nrf24l01p, uint8_t reg,
			    uint8_t *data ,uint8_t count);
int nrf24l01p_flush_fifo(struct nrf24l0_data *nrf24l01p);

#define to_nrf24_device(nrf24) container_of(nrf24, struct nrf24l0_data, dev)
#endif
