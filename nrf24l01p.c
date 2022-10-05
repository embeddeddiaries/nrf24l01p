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
#include <linux/regmap.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include "nrf24l01p.h"

int nrf24_get_status(struct nrf24l0_data *nrf24l01p, uint8_t *status)
{
	return nrf24l01p_read_register(nrf24l01p, NRF24L0P_STATUS, status, 1);
}


static int registers_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	int ret;
	uint8_t status;
	struct nrf24l0_data *nrf24l01p  = to_nrf24_device(dev);

	nrf24_print_registers(nrf24l01p);
	ret = nrf24_get_status(nrf24l01p, &status);
	if (ret < 0)
		return ret;
	return snprintf(buf, 16, "STATUS = 0x%02X\n", status);
}

static DEVICE_ATTR_RO(registers);

struct attribute *nrf24_attrs[] = {
	&dev_attr_registers.attr,
	NULL,
};

struct attribute *nrf24_pipe_attrs[] = {
	
};

int nrf24_print_registers(struct nrf24l0_data *nrf24l01p)
{
const u8 nrf_reg[] = {
	NRF24L0P_CONFIG,
	NRF24L0P_EN_AA,
	NRF24L0P_EN_RXADDR,
	NRF24L0P_SETUP_AW,
	NRF24L0P_SETUP_RETR,
	NRF24L0P_RF_CH,
	NRF24L0P_RF_SETUP,
	NRF24L0P_STATUS,
	NRF24L0P_OBSERVE_TX,
	NRF24L0P_CD,
	NRF24L0P_RX_PW_P0,
	NRF24L0P_FIFO_STATUS,
	NRF24L0P_DYNPD,
	NRF24L0P_FEATURE
};

char *nrf_reg_name[] = {
	"CONFIG",
	"EN_AA",
	"EN_RXADDR",
	"SETUP_AW",
	"SETUP_RETR",
	"RF_CH",
	"RF_SETUP",
	"STATUS",
	"OBSERVE_TX",
	"CD",
	"RX_PW_P0",
	"FIFO_STATUS",
	"DYNPD",
	"FEATURE"
};

	ssize_t loop;
	int ret;
	uint8_t value;

	for (loop = 0; loop < 14; loop++) {
		//ret = spi_w8r8(spi, nrf_reg[loop]);
		ret = nrf24l01p_read_register(nrf24l01p, nrf_reg[loop],
					      &value, 1);
		if (ret < 0)
			return ret;

		dev_info(&nrf24l01p->dev,
			"%s: %s = 0%02zx\n",
			__func__,
			nrf_reg_name[loop],
			value);
	}

	return 0;
}


int nrf24l01p_read_register(struct nrf24l0_data *nrf24l01p, uint8_t reg,
			    uint8_t *data ,uint8_t count)
{
	reg &= 0x1F;
	//if (count == 1)
	//	return regmap_read(nrf24l01p->regmap, reg, (u32 *)data);

	return regmap_bulk_read(nrf24l01p->regmap, reg, data, count);
}

int nrf24l01p_write_register(struct nrf24l0_data *nrf24l01p, uint8_t reg,
			     uint8_t *data ,uint8_t count)
{

	reg = 0x20 | reg;
//	if (count == 1)
		//return regmap_write(nrf24l01p->regmap, reg, *(u32*)data);

	return regmap_bulk_write(nrf24l01p->regmap, reg, data, count);

}

int nrf24l0p_get_tx_address(struct nrf24l0_data *nrf24l01p, uint8_t *addr)
{
	return nrf24l01p_read_register(nrf24l01p, NRF24L0P_TX_ADDR, addr, 5);
}

int nrf24l0p_get_rx_address(struct nrf24l0_data *nrf24l01p, uint8_t *addr)
{
	return nrf24l01p_read_register(nrf24l01p, NRF24L0P_RX_ADDR_P0, addr, 5);
}

int nrf24l01p_set_mode(struct nrf24l0_data *nrf24l01p, enum nrf24_mode mode)
{
	uint8_t config;
	int ret;

	ret = nrf24l01p_read_register(nrf24l01p, NRF24L0P_CONFIG, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Power config read fail\n");
		return ret;
	}

	if (mode == NRF24_MODE_RX)
		config |= NRF24L0P_PRIM_RX;
	else
		config &= ~NRF24L0P_PRIM_RX;

	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_CONFIG, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Power config write fail\n");
		return ret;
	}
	return 0;
}

int nrf24l01p_get_status(struct nrf24l0_data *nrf24l01p, uint8_t *status)
{
	return nrf24l01p_read_register(nrf24l01p, NRF24L0P_STATUS, status, 1);
}

int nrf24l01p_clear_irq(struct nrf24l0_data *nrf24l01p, u8 irq)
{
	return nrf24l01p_write_register(nrf24l01p, NRF24L0P_STATUS, &irq, 1);
}

int nrf24l01p_flush_fifo(struct nrf24l0_data *nrf24l01p)
{
	int ret;
	uint8_t flush = 0;

	ret = regmap_bulk_write(nrf24l01p->regmap, NRF24L0P_FLUSH_TX, &flush, 1);
	if (ret < 0) {
		dev_err(&nrf24l01p->dev, "Tx flush write fail\n");
		return ret;
	}
	ret = regmap_bulk_write(nrf24l01p->regmap, NRF24L0P_FLUSH_RX, &flush, 1);
	if (ret < 0) {
		dev_err(&nrf24l01p->dev, "Rx flush write fail\n");
		return ret;
	}
	return 0;
}

int nrf24l01p_is_rx_fifo_empty(struct nrf24l0_data *nrf24l01p)
{
	int ret;
	uint8_t fifo;

	ret = nrf24l01p_read_register(nrf24l01p, NRF24L0P_FIFO_STATUS, &fifo, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Error reading fifo status\n");
		return ret;
	}
	return fifo & 0x01;
}

int nrf24l01_get_rx_payload(struct nrf24l0_data *nrf24l01p, uint8_t *pload)
{
	u8 status, length, index;
	int ret;

	ret = nrf24l01p_get_status(nrf24l01p, &status);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Status read failed");
		return ret;
	}
	if ((status & 0x0E) != 0x00)
		dev_err(&nrf24l01p->dev, "Rx data invalid\n");
	
	ret = regmap_bulk_read(nrf24l01p->regmap, NRF24L0P_R_RX_PL_WID,
				       &length, 1);

	if (ret) {
		dev_err(&nrf24l01p->dev, "Payload length read failed");
		return ret;
	}

	if (length > 0 && length <= NRF24L0P_PLOAD_MAX) {
		dev_info(&nrf24l01p->dev, "%s: length = %zd\n", __func__, length);
		ret = regmap_bulk_read(nrf24l01p->regmap, NRF24L0P_R_RX_PAYLOAD,
				       pload, length);
		if (ret < 0) {
			dev_err(&nrf24l01p->dev, "Payload read failed");
			return ret;
		}
		dev_info(&nrf24l01p->dev, "Rx data: ");
		for (index = 0; index < length; index++) {
			dev_info(&nrf24l01p->dev, "0x%02x ", pload[index]);
		}
		dev_info(&nrf24l01p->dev, "\n");
	}

	return length;
}

int nrf24l01p_write_tx_pload(struct nrf24l0_data *nrf24l01p, uint8_t *pload,
			     uint8_t size)
{
	return regmap_bulk_write(nrf24l01p->regmap, NRF24L0P_W_TX_PAYLOAD,
				 pload, size);
}

int nrf24l01p_configure(struct nrf24l0_data *nrf24l01p)
{
	uint8_t config;
	int ret;

	//configure CRC to 1 byte
	config = 0x0C;
	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_CONFIG, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Config write error\n");
	}

	//configure ARD to 4000us and ARC 15 retires.
	config = 0xff;
	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_SETUP_RETR, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "RETR write error\n");
	}

	//configure PWR to 0dBm and data rate 2mbps.
	config = 0x07;
	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_SETUP_RETR, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "RETR write error\n");
	}

	//Enable auto ack
	config = 0x01;
	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_EN_AA, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Auto ack write error\n");
	}

	//Enable dpl
	config = 0x04;
	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_FEATURE, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Auto ack write error\n");
	}

	//Enable DPL_P0
	config = 0x01;
	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_DYNPD, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Auto ack write error\n");
	}

	//Enable ERX_P0
	config = 0x01;
	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_EN_RXADDR, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Enable rx write error\n");
	}

	nrf24l01p_flush_fifo(nrf24l01p);

	//Power Up
	ret = nrf24l01p_read_register(nrf24l01p, NRF24L0P_CONFIG, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Power config read fail\n");
		return ret;
	}
	config |= NRF24L0P_PWR_UP;
	ret = nrf24l01p_write_register(nrf24l01p, NRF24L0P_CONFIG, &config, 1);
	if (ret) {
		dev_err(&nrf24l01p->dev, "Power config write fail\n");
		return ret;
	}

	//Set Rx mode
	ret = nrf24l01p_set_mode(nrf24l01p, NRF24_MODE_RX);
	if (ret) {
		dev_err(&nrf24l01p->dev,"Mode set failed\n");
	}
	return ret;
}

MODULE_AUTHOR("Jagath Jog J <jagathjog1996@gmail.com>");
MODULE_DESCRIPTION("NRF24L01P transceiver device driver");
MODULE_LICENSE("GPL");
