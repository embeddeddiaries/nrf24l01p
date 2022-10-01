// SPDX-License-Identifier: GPL-2.0
/*            
 * Copyright (C) 2022 Jagath Jog J <jagathjog1996@gmail.com@gmail.com>
 *
 */

#include <linux/bits.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/kfifo.h>
#include <linux/list.h>

#include "nrf24l01p.h"

static dev_t nrf24_devt;
static struct class *nrf24_class;
ATTRIBUTE_GROUPS(nrf24);

const struct regmap_config nrf24l0p_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int nrf24l01p_regmap_spi_read(void *context, const void *reg, size_t reg_size,
				  void *val, size_t val_size)
{
	struct spi_device *spi = context;
	uint8_t cmd = *(u8*)reg;
	
	return spi_write_then_read(spi, &cmd, 1, val, val_size);
}

static int nrf24l01p_regmap_spi_write(void *context, const void *data, size_t count)
{
	return 0;
}

static struct regmap_bus nrf24l0p_regmap_bus = {
	.read = nrf24l01p_regmap_spi_read,
	.write = nrf24l01p_regmap_spi_write,
};

static void nrf24_dev_release(struct device *dev)
{
	struct nrf24l0_data *nrf24l01p  = to_nrf24_device(dev);
	kfree(nrf24l01p);
}

static struct device_type nrf24_dev_type = {
	.name = "nrf24_transceiver",
	.release = nrf24_dev_release,
};

static void nrf24_isr_work_handler(struct work_struct *work)
{

}

static void nrf24_rx_work_handler(struct work_struct *work)
{

}

static int nrf24l01p_probe(struct spi_device *spi)
{
	//const struct spi_device_id *id = spi_get_device_id(spi);
	uint8_t known_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	struct regmap *regmap;
	uint8_t addr[5] = {0, };
	struct nrf24l0_data *nrf24l01p;
	int ret;

	regmap = devm_regmap_init(&spi->dev, &nrf24l0p_regmap_bus,
				  spi, &nrf24l0p_regmap_config);
        if (IS_ERR(regmap)) {
                dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
                        PTR_ERR(regmap));
                return -EINVAL;
        }
	ret = regmap_bulk_read(regmap, 0x0A, addr, 5);
	if (ret) {
		dev_err(&spi->dev, "Regmap read fail\n");
		return ret;
	}
	if (memcmp(addr, known_addr, 5)) {
		dev_err(&spi->dev, "NRF24L01p not detected\n");
		return ret;
	}

	nrf24l01p = kzalloc(sizeof(*nrf24l01p), GFP_KERNEL);
	if (!nrf24l01p) {
		return -ENOMEM;
	}
	dev_set_name(&nrf24l01p->dev, "nrf24l%d", 0);
	nrf24l01p->regmap = regmap;
	nrf24l01p->dev.class = nrf24_class;
	nrf24l01p->dev.type = &nrf24_dev_type;
	nrf24l01p->dev.groups = nrf24_groups;

	ret = device_register(&nrf24l01p->dev);
	if (ret < 0) {
		put_device(&nrf24l01p->dev);
		return ret;
	}

	init_waitqueue_head(&nrf24l01p->tx_wait_queue);
	init_waitqueue_head(&nrf24l01p->tx_done_wait_queue);
	INIT_WORK(&nrf24l01p->isr_work, nrf24_isr_work_handler);
	INIT_WORK(&nrf24l01p->rx_work, nrf24_rx_work_handler);
	INIT_KFIFO(nrf24l01p->tx_fifo);

	dev_info(&nrf24l01p->dev, "Probe success\n");
	spi_set_drvdata(spi, nrf24l01p);
	return 0;
}

static int nrf24l01p_remove(struct spi_device *spi)
{
	struct nrf24l0_data *nrf24l01p = spi_get_drvdata(spi);
	device_unregister(&nrf24l01p->dev);
	return 0;
}

static const struct spi_device_id nrf24l01p_spi_ids[] = {
        { "nrf24l01p", 0 },
        { }
};
MODULE_DEVICE_TABLE(spi, nrf24l01p_spi_ids);

static const struct of_device_id nrf24l01p_dt_ids[] = {
	{ .compatible = "nordic,nrf24l01p" },
	{},
};

MODULE_DEVICE_TABLE(of, nrf24l01p_dt_ids);

struct spi_driver nrf24l01p_spi_driver = {
	.driver = {
		.name = "nrf24l01p",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nrf24l01p_dt_ids),
	},
	.probe = nrf24l01p_probe,
	.remove = nrf24l01p_remove,
	.id_table = nrf24l01p_spi_ids,
};

static int __init nrf24l01_init(void)
{
	int ret;

	//Dynamically allocate major number and reserve range of minor numbers
	ret = alloc_chrdev_region(&nrf24_devt, 0, 8, NRF24_CLASS_NAME);
	if (ret < 0) {
		pr_err("Unable to alloc chrdev region\n");
		goto chrdev_destroy;
	}

	//create device's class, visible in /sys/class */
	nrf24_class = class_create(THIS_MODULE, NRF24_CLASS_NAME);
	if (IS_ERR(nrf24_class)) {
		pr_err("Unable to create class\n");
		ret = PTR_ERR(nrf24_class);
		goto class_remove;
	}

	//Register device driver
	ret = spi_register_driver(&nrf24l01p_spi_driver);
	if (ret < 0) {
		pr_err("Unable to register spi driver\n");
		goto class_remove;
	}

	return 0;

class_remove:
	class_destroy(nrf24_class);
chrdev_destroy:
	unregister_chrdev(MAJOR(nrf24_devt), NRF24_CLASS_NAME);

	return ret;
}
module_init(nrf24l01_init);

static void __exit nrf24l01_exit(void)
{
	spi_unregister_driver(&nrf24l01p_spi_driver);
	class_destroy(nrf24_class);
	unregister_chrdev(MAJOR(nrf24_devt), NRF24_CLASS_NAME);
}

module_exit(nrf24l01_exit);

MODULE_AUTHOR("Jagath Jog J <jagathjog1996@gmail.com>");
MODULE_DESCRIPTION("NRF24L01P transceiver device driver");
MODULE_LICENSE("GPL");
