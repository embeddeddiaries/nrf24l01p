// SPDX-License-Identifier: GPL-2.0
/*            
 * Copyright (C) 2022 Jagath Jog J <jagathjog1996@gmail.com@gmail.com>
 *
 */

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/module.h>

#include "nrf24l01p.h"

struct attribute *nrf24_attrs[] = {
	
};


MODULE_AUTHOR("Jagath Jog J <jagathjog1996@gmail.com>");
MODULE_DESCRIPTION("NRF24L01P transceiver device driver");
MODULE_LICENSE("GPL");
