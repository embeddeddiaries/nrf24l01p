// SPDX-License-Identifier: GPL-2.0
/*            
 * Copyright (C) 2022 Jagath Jog J <jagathjog1996@gmail.com@gmail.com>
 *
 */

#include <linux/bits.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/poll.h>

#include "nrf24l01p.h"

static dev_t nrf24_devt;
static struct class *nrf24_class;
ATTRIBUTE_GROUPS(nrf24);
ATTRIBUTE_GROUPS(nrf24_pipe);

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
	struct spi_device *spi = context;

	return spi_write(spi, data, count);
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

static void nrf24_gpio_free(struct nrf24l0_data *nrf24l01p)
{
	if (!IS_ERR(nrf24l01p->ce))
		gpiod_put(nrf24l01p->ce);

	free_irq(nrf24l01p->irq, nrf24l01p);
}

static void nrf24_ce_pin(struct nrf24l0_data *nrf24l01p, int hi_low)
{
	if (hi_low)
		gpiod_set_value(nrf24l01p->ce, 1);
	else
		gpiod_set_value(nrf24l01p->ce, 0);

}

static int nrf24l01p_open(struct inode *inode, struct file *filp)
{
	struct nrf24l0_pipe *pipe0;

	pipe0 = container_of(inode->i_cdev, struct nrf24l0_pipe, cdev);
	if (!pipe0) {
		pr_err("device: minor %d unknown.\n", iminor(inode));
		return -ENODEV;
	}

	filp->private_data = pipe0;

	return 0;
}

static int nrf24l01p_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static ssize_t nrf24l01p_read(struct file *filp,
			  char __user *buf,
			  size_t size,
			  loff_t *f_pos)
{
	struct nrf24l0_data *nrf24l01p;
	struct nrf24l0_pipe *pipe0;
	unsigned int copied, n;
	int ret;

	pipe0 = filp->private_data;
	nrf24l01p = to_nrf24_device(pipe0->dev->parent);

	if (kfifo_is_empty(&pipe0->rx_fifo)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		wait_event_interruptible(pipe0->read_wait_queue,
					 !kfifo_is_empty(&pipe0->rx_fifo));
	}

	ret = mutex_lock_interruptible(&pipe0->rx_fifo_mutex);
	if (ret)
		return ret;

	n = kfifo_to_user(&pipe0->rx_fifo, buf, size, &copied);

	mutex_unlock(&pipe0->rx_fifo_mutex);

	return n ? n : copied;
}

static ssize_t nrf24l01p_write(struct file *filp,
			   const char __user *buf,
			   size_t size,
			   loff_t *f_pos)
{
	struct nrf24l0_data *nrf24l01p;
	struct nrf24l0_pipe *pipe0;
	struct nrf24l0_tx_data txdata;

	pipe0 = filp->private_data;
	nrf24l01p = to_nrf24_device(pipe0->dev->parent);

	txdata.size = min_t(size_t, size, NRF24L0P_PLOAD_MAX);
	txdata.pipe0 = pipe0;

	memset(txdata.pload, 0, NRF24L0P_PLOAD_MAX);
	if (copy_from_user(txdata.pload, buf, txdata.size)) {
		dev_err(&nrf24l01p->dev,"%s\n","Data copy error\n");
		goto exit_lock;
	}

	if (mutex_lock_interruptible(&nrf24l01p->tx_fifo_mutex))
		goto exit_lock;

	if (kfifo_in(&nrf24l01p->tx_fifo, &txdata, sizeof(txdata)) != sizeof(txdata)) {
		dev_err(&nrf24l01p->dev,"%s\n","Fifo write error\n");
		goto exit_kfifo;
	}
	mutex_unlock(&nrf24l01p->tx_fifo_mutex);
	wake_up_interruptible(&nrf24l01p->tx_wait_queue);

	pipe0->write_done = false;
	wait_event_interruptible(pipe0->write_wait_queue, pipe0->write_done);

	return size;
exit_kfifo:
	mutex_unlock(&nrf24l01p->tx_fifo_mutex);

exit_lock:
	//if (filp->f_flags & O_NONBLOCK)
	wake_up_interruptible(&nrf24l01p->tx_wait_queue);
	return size;
}

static __poll_t nrf24l01p_poll(struct file *filp, struct poll_table_struct *wait)
{
	__poll_t events = 0;
	struct nrf24l0_data *nrf24l01p;
	struct nrf24l0_pipe *pipe0;

	pipe0 = filp->private_data;
	nrf24l01p = to_nrf24_device(pipe0->dev->parent);

	poll_wait(filp, &pipe0->read_wait_queue, wait);
	if (!kfifo_is_empty(&pipe0->rx_fifo))
		events |= (EPOLLIN | EPOLLRDNORM);

	if (!kfifo_is_full(&nrf24l01p->tx_fifo))
		events |= (EPOLLOUT | EPOLLWRNORM);

	return events;
}

static void nrf24l01_remove_pipes(struct nrf24l0_data *nrf24l01p)
{
	struct nrf24l0_pipe *pipe = nrf24l01p->pipe0;

	cdev_del(&pipe->cdev);
	device_destroy(nrf24_class, pipe->devt);
	kfree(pipe);
}

static const struct file_operations nrf24l01p_fops = {
	.owner = THIS_MODULE,
	.open = nrf24l01p_open,
	.release = nrf24l01p_release,
	.read = nrf24l01p_read,
	.write = nrf24l01p_write,
	.poll = nrf24l01p_poll,
};

static void nrf24_isr_work_handler(struct work_struct *work)
{
	struct nrf24l0_data *nrf24l01p;
	uint8_t status;
	int ret;

	nrf24l01p = container_of(work, struct nrf24l0_data, isr_work);

	ret = nrf24l01p_get_status(nrf24l01p, &status);
	if (ret) {
		dev_info(&nrf24l01p->dev, "Status read failed");
		return;
	}

	if (status & NRF24L0P_RX_DR) {
		dev_info(&nrf24l01p->dev, "%s: RX_DR\n", __func__);
		nrf24l01p->rx_active = true;
		mdelay(50);
		nrf24l01p_clear_irq(nrf24l01p, NRF24L0P_RX_DR);
		schedule_work(&nrf24l01p->rx_work);
	}

	if (status & NRF24L0P_TX_DS) {
		dev_info(&nrf24l01p->dev, "%s: TX_DS\n", __func__);
		nrf24l01p->tx_done = true;
		nrf24l01p_clear_irq(nrf24l01p, NRF24L0P_TX_DS);
		wake_up_interruptible(&nrf24l01p->tx_done_wait_queue);
	}

	if (status & NRF24L0P_MAX_RT) {
		dev_info_ratelimited(&nrf24l01p->dev, "%s: MAX_RT\n", __func__);
		nrf24l01p->tx_done = true;
		nrf24l01p_flush_fifo(nrf24l01p);
		nrf24l01p_clear_irq(nrf24l01p, NRF24L0P_TX_DS);
		wake_up_interruptible(&nrf24l01p->tx_done_wait_queue);
	}
}

static void nrf24_rx_work_handler(struct work_struct *work)
{
	struct nrf24l0_data *nrf24l01p;
	u8 payload[NRF24L0P_PLOAD_MAX];
	struct nrf24l0_pipe *pipe0;
	int length;

	nrf24l01p = container_of(work, struct nrf24l0_data, rx_work);
	pipe0 = nrf24l01p->pipe0;

	while(!nrf24l01p_is_rx_fifo_empty(nrf24l01p))
	{
		memset(payload, 0, NRF24L0P_PLOAD_MAX);
		length = nrf24l01_get_rx_payload(nrf24l01p, payload);
		if (length < 0) {
			dev_info(&nrf24l01p->dev, "payload invalid\n");
		}
		nrf24l01p->rx_active = false;
		if (mutex_lock_interruptible(&pipe0->rx_fifo_mutex))
			return;
		kfifo_in(&pipe0->rx_fifo, payload, length);
		mutex_unlock(&pipe0->rx_fifo_mutex);
		wake_up_interruptible(&pipe0->read_wait_queue);
	}
}

static irqreturn_t nrf24l01_isr(int irq, void *dev_id)
{
	unsigned long flags;
	struct nrf24l0_data *nrf24l01p = dev_id;

	spin_lock_irqsave(&nrf24l01p->lock, flags);

	schedule_work(&nrf24l01p->isr_work);

	spin_unlock_irqrestore(&nrf24l01p->lock, flags);

	return IRQ_HANDLED;
}

static struct nrf24l0_pipe *nrf24l01_create_pipe(struct nrf24l0_data *nrf24l01p)
{
	int ret;
	struct nrf24l0_pipe *pipe;

	pipe = kzalloc(sizeof(*pipe), GFP_KERNEL);
	if (!pipe) {
		ret = -ENOMEM;
		goto err_return;
	}

	pipe->devt = MKDEV(MAJOR(nrf24_devt), 0);
	INIT_KFIFO(pipe->rx_fifo);

	init_waitqueue_head(&pipe->read_wait_queue);
	init_waitqueue_head(&pipe->write_wait_queue);

	pipe->dev = device_create_with_groups(nrf24_class,
					   &nrf24l01p->dev,
					   pipe->devt,
					   pipe,
					   nrf24_pipe_groups,
					   "%s.%d",
					   dev_name(&nrf24l01p->dev),
					   0);

	if (IS_ERR(pipe->dev)) {
		dev_err(&nrf24l01p->dev,
			"%s: device_create of pipe %d failed\n",
			__func__,
			0);
		ret = PTR_ERR(pipe->dev);
		goto err_free_mem;
	}


	cdev_init(&pipe->cdev, &nrf24l01p_fops);
	pipe->cdev.owner = THIS_MODULE;
	ret = cdev_add(&pipe->cdev, pipe->devt, 1);

	if (ret < 0) {
		dev_err(&nrf24l01p->dev, "%s: cdev failed\n", __func__);
		goto err_dev_destroy;
	}

	dev_info(&nrf24l01p->dev,
		"%s: device created: major(%d), minor(%d)\n",
		__func__,
		MAJOR(pipe->devt),
		MINOR(pipe->devt));

	return pipe;

err_dev_destroy:
	device_destroy(nrf24_class, pipe->devt);
err_free_mem:
	kfree(pipe);
err_return:
	return ERR_PTR(ret);
}

static int nrf24l01_gpio_config(struct nrf24l0_data *nrf24l01p)
{
	int ret;
	nrf24l01p->ce = gpiod_get(&nrf24l01p->parent, "ce", 0);

	if (nrf24l01p->ce == ERR_PTR(-ENOENT))
		dev_dbg(&nrf24l01p->dev, "%s: no entry for CE\n", __func__);
	else if (nrf24l01p->ce == ERR_PTR(-EBUSY))
		dev_dbg(&nrf24l01p->dev, "%s: CE is busy\n", __func__);

	if (IS_ERR(nrf24l01p->ce)) {
		ret = PTR_ERR(nrf24l01p->ce);
		dev_err(&nrf24l01p->dev, "%s: CE gpio setup error\n", __func__);
		return ret;
	}

	ret = request_irq(nrf24l01p->irq, nrf24l01_isr,
			  0, dev_name(&nrf24l01p->dev), nrf24l01p);
	if (ret < 0) {
		gpiod_put(nrf24l01p->ce);
		return ret;
	}

	return 0;
}

static int nrf24l01p_tx_thread(void *data)
{
	struct nrf24l0_data *nrf24l01p = data;
	struct nrf24l0_tx_data txdata;
	struct nrf24l0_pipe *pipe0;
	int ret;

	while(true) {
		wait_event_interruptible(nrf24l01p->tx_wait_queue,
					 kthread_should_stop() ||
					 (!nrf24l01p->rx_active && !kfifo_is_empty(&nrf24l01p->tx_fifo)));

		if (kthread_should_stop())
			return 0;
		
		if (mutex_lock_interruptible(&nrf24l01p->tx_fifo_mutex))
			continue;

		ret = kfifo_out(&nrf24l01p->tx_fifo, &txdata, sizeof(txdata));
		if (ret != sizeof(txdata)) {
			dev_err(&nrf24l01p->dev, "get tx_data from fifo failed\n");
			continue;
		}
		mutex_unlock(&nrf24l01p->tx_fifo_mutex);
		pipe0 = txdata.pipe0;
	
		nrf24_ce_pin(nrf24l01p, 0);

		ret = nrf24l01p_set_mode(nrf24l01p, NRF24_MODE_TX);
		if (ret) {
			dev_err(&nrf24l01p->dev, "Mode set to Tx failed\n");
			goto gotoRX;
		}

		//auto ack

		ret = nrf24l01p_write_tx_pload(nrf24l01p, txdata.pload, txdata.size);
		if (ret < 0) {
			dev_err(&nrf24l01p->dev,
				"write TX PLOAD failed (%d)\n",
				ret);
			goto gotoRX;
		}

		nrf24_ce_pin(nrf24l01p, 1);

		//Wait for ack
		nrf24l01p->tx_done = false;
		wait_event_interruptible(nrf24l01p->tx_done_wait_queue,
					 (nrf24l01p->tx_done ||
					 kthread_should_stop()));

		if (kthread_should_stop())
			return 0;

		pipe0->write_done = true;
		wake_up_interruptible(&pipe0->write_wait_queue);

gotoRX:
		if (kfifo_is_empty(&nrf24l01p->tx_fifo) || nrf24l01p->rx_active) {

			//enter Standby-I
			nrf24_ce_pin(nrf24l01p, 0);
			ret = nrf24l01p_set_mode(nrf24l01p, NRF24_MODE_RX);
			if (ret < 0) {
				dev_err(&nrf24l01p->dev,
						"Set rx mode failed (%d)\n",
						ret);
			}
			nrf24_ce_pin(nrf24l01p, 1);
		}
	}
	return 0;
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
		//return ret;
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
	nrf24l01p->irq = spi->irq;
	nrf24l01p->parent = spi->dev;

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

	ret = nrf24l01_gpio_config(nrf24l01p);
	if (ret)
	{
		dev_err(&nrf24l01p->dev, "gpio_setup failed\n");
		goto unregister_device;
	}

	nrf24l01p->pipe0 = nrf24l01_create_pipe(nrf24l01p);
	if (IS_ERR(nrf24l01p->pipe0)) {
		ret = PTR_ERR(nrf24l01p->pipe0);
		goto remove_device;
	}

	ret = nrf24l01p_configure(nrf24l01p);
	if (ret)
	{
		dev_err(&nrf24l01p->dev, "Configure failed\n");
		goto remove_device;
	}

	nrf24_ce_pin(nrf24l01p, 1);

	nrf24l01p->tx_task_struct = kthread_run(nrf24l01p_tx_thread,
						nrf24l01p,
						"nrf24l_tx_thread");
	if (IS_ERR(nrf24l01p->tx_task_struct)) {
		dev_err(&nrf24l01p->dev, "start of tx thread failed\n");
		goto remove_device;
	}

	spi_set_drvdata(spi, nrf24l01p);
	return 0;

remove_device:
	nrf24_gpio_free(nrf24l01p);

unregister_device:
	device_unregister(&nrf24l01p->dev);
	return ret;
}

static int nrf24l01p_remove(struct spi_device *spi)
{
	struct nrf24l0_data *nrf24l01p = spi_get_drvdata(spi);
	nrf24_gpio_free(nrf24l01p);
	kthread_stop(nrf24l01p->tx_task_struct);
	nrf24l01_remove_pipes(nrf24l01p);
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
