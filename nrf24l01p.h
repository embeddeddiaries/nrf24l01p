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

extern struct attribute *nrf24_attrs[];

struct nrf24l0_data {
        struct device dev;
	struct regmap *regmap;

	struct work_struct	isr_work;
	struct work_struct	rx_work;

	STRUCT_KFIFO_REC_1(FIFO_SIZE) tx_fifo;
	struct task_struct	*tx_task_struct;
	wait_queue_head_t	tx_wait_queue;
	wait_queue_head_t	tx_done_wait_queue;
};

#define to_nrf24_device(nrf24) container_of(nrf24, struct nrf24l0_data, dev)
#endif
