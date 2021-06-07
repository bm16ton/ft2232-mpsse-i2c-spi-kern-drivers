// SPDX-License-Identifier: GPL-2.0
/*
 * FTDI FT232H MPSSE SPI controller driver
 *
 * Copyright (C) 2017 DENX Software Engineering
 * Anatolij Gustschin <agust@denx.de>
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/usb.h>
#include <linux/usb/ft232h-intf.h>

enum gpiol {
	SK	= BIT(0),
	DO	= BIT(1),
	DI	= BIT(2),
	CS	= BIT(3),
};

struct ftdi_spi {
	struct platform_device *pdev;
	struct usb_interface *intf;
	struct spi_master *master;
	const struct ft232h_intf_ops *iops;
	struct gpiod_lookup_table *lookup[13];
	struct gpio_desc **cs_gpios;

	u8 txrx_cmd;
	u8 rx_cmd;
	u8 tx_cmd;
	u8 xfer_buf[SZ_64K];
	u16 last_mode;
};

static void ftdi_spi_chipselect(struct ftdi_spi *priv, struct spi_device *spi,
				bool value)
{
	int cs = spi->chip_select;

	dev_dbg(&priv->master->dev, "%s: CS %d, mode(%d), val %d\n",
		__func__, cs, (spi->mode & SPI_CS_HIGH), value);

	gpiod_set_raw_value_cansleep(priv->cs_gpios[cs], value);
}

static inline u8 ftdi_spi_txrx_byte_cmd(struct spi_device *spi)
{
	u8 mode = spi->mode & (SPI_CPOL | SPI_CPHA);
	u8 cmd;

	if (spi->mode & SPI_LSB_FIRST) {
		switch (mode) {
		case SPI_MODE_0:
		case SPI_MODE_1:
			cmd = TXF_RXR_BYTES_LSB;
			break;
		case SPI_MODE_2:
		case SPI_MODE_3:
			cmd = TXR_RXF_BYTES_LSB;
			break;
		}
	} else {
		switch (mode) {
		case SPI_MODE_0:
		case SPI_MODE_1:
			cmd = TXF_RXR_BYTES_MSB;
			break;
		case SPI_MODE_2:
		case SPI_MODE_3:
			cmd = TXR_RXF_BYTES_MSB;
			break;
		}
	}
	return cmd;
}

static inline int ftdi_spi_loopback_cfg(struct ftdi_spi *priv, int on)
{
	int ret;

	priv->xfer_buf[0] = on ? LOOPBACK_ON : LOOPBACK_OFF;

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 1);
	if (ret < 0)
		dev_warn(&priv->master->dev, "loopback %d failed\n", on);
	return ret;
}

static int ftdi_spi_tx_rx(struct ftdi_spi *priv, struct spi_device *spi,
			  struct spi_transfer *t)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct spi_master *master = priv->master;
	struct device *dev = &master->dev;
	void *rx_offs;
	const void *tx_offs;
	size_t remaining, stride;
	size_t rx_stride;
	int ret, tout = 10;
	const u8 *tx_data = t->tx_buf;
	u8 *rx_data = t->rx_buf;

	if (!t->len)
		return 0;

	ops->lock(priv->intf);

	if (spi->mode & SPI_LOOP) {
		ret = ftdi_spi_loopback_cfg(priv, 1);
		if (ret < 0)
			goto err;
	}

	remaining = t->len;
	rx_offs = rx_data;
	tx_offs = tx_data;

	while (remaining) {
		stride = min_t(size_t, remaining, SZ_512 - 3);

		priv->xfer_buf[0] = priv->txrx_cmd;
		priv->xfer_buf[1] = stride - 1;
		priv->xfer_buf[2] = (stride - 1) >> 8;
		memcpy(&priv->xfer_buf[3], tx_offs, stride);
		print_hex_dump_debug("WR: ", DUMP_PREFIX_OFFSET, 16, 1,
				     priv->xfer_buf, stride + 3, 1);

		ret = ops->write_data(priv->intf, priv->xfer_buf, stride + 3);
		if (ret < 0) {
			dev_err(dev, "%s: xfer failed %d\n", __func__, ret);
			goto fail;
		}
		dev_dbg(dev, "%s: WR %zu byte(s), TXRX CMD 0x%02x\n",
			__func__, stride, priv->txrx_cmd);

		rx_stride = min_t(size_t, stride, SZ_512);

		ret = ops->read_data(priv->intf, priv->xfer_buf, rx_stride);
		while (ret == 0) {
			/* If no data yet, wait and repeat */
			usleep_range(5000, 5100);
			ret = ops->read_data(priv->intf, priv->xfer_buf,
					     rx_stride);
			dev_dbg(dev, "Waiting data ready, read: %d\n", ret);
			if (!--tout) {
				dev_err(dev, "Read timeout\n");
				ret = -ETIMEDOUT;
				goto fail;
			}
		}

		if (ret < 0)
			goto fail;

		print_hex_dump_debug("RD: ", DUMP_PREFIX_OFFSET, 16, 1,
				     priv->xfer_buf, rx_stride, 1);
		memcpy(rx_offs, priv->xfer_buf, ret);
		rx_offs += ret;

		remaining -= stride;
		tx_offs += stride;
		dev_dbg(dev, "%s: WR remains %zu\n", __func__, remaining);
	}

	ret = 0;

fail:
	if (spi->mode & SPI_LOOP)
		ftdi_spi_loopback_cfg(priv, 0);

err:
	ops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_tx(struct ftdi_spi *priv, const u8 *tx_data, size_t len)
{
	struct spi_master *master = priv->master;
	size_t remaining, stride;
	int ret;

	if (!len)
		return 0;

	priv->iops->lock(priv->intf);

	remaining = len;
	do {
		stride = min_t(size_t, remaining, SZ_64K - 3);

		priv->xfer_buf[0] = priv->tx_cmd;
		priv->xfer_buf[1] = stride - 1;
		priv->xfer_buf[2] = (stride - 1) >> 8;

		memcpy(&priv->xfer_buf[3], tx_data, stride);

		ret = priv->iops->write_data(priv->intf, priv->xfer_buf,
					     stride + 3);
		if (ret < 0) {
			dev_dbg(&master->dev, "%s: tx failed %d\n",
				__func__, ret);
			goto err;
		}
		dev_dbg(&master->dev, "%s: %zu byte(s) done\n",
			__func__, stride);
		remaining -= stride;
	} while (remaining);

	ret = 0;
err:
	priv->iops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_rx(struct ftdi_spi *priv, u8 *rx_data, size_t len)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct spi_master *master = priv->master;
	size_t remaining, stride;
	int ret, tout = 10;
	void *rxbuf;

	dev_dbg(&master->dev, "%s: CMD 0x%02x, len %zu\n",
		__func__, priv->rx_cmd, len);

	if (!len)
		return 0;

	priv->xfer_buf[0] = priv->rx_cmd;
	priv->xfer_buf[1] = len - 1;
	priv->xfer_buf[2] = (len - 1) >> 8;

	ops->lock(priv->intf);

	ret = ops->write_data(priv->intf, priv->xfer_buf, 3);
	if (ret < 0)
		goto err;

	remaining = len;
	rxbuf = rx_data;

	do {
		stride = min_t(size_t, remaining, SZ_512);

		ret = ops->read_data(priv->intf, priv->xfer_buf, stride);
		if (ret < 0)
			goto err;

		if (!ret) {
			dev_dbg(&master->dev,
				"Waiting for data (read : %02X), tout %d\n",
				 ret, tout);
			if (--tout)
				continue;

			dev_dbg(&master->dev, "read timeout...\n");
			ret = -ETIMEDOUT;
			goto err;
		}

		memcpy(rxbuf, priv->xfer_buf, ret);

		dev_dbg(&master->dev, "%s: %d byte(s)\n", __func__, ret);
		rxbuf += ret;
		remaining -= ret;
	} while (remaining);

	ret = 0;
err:
	ops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_transfer_one(struct spi_master *master,
				 struct spi_message *msg)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(master);
	struct spi_device *spi = msg->spi;
	struct spi_transfer *t;
	bool keep_cs = false;
	int ret = 0;

	if (!priv)
		return -ENODEV;

	msg->actual_length = 0;
	msg->state = NULL;
	msg->status = 0;

	if (priv->last_mode != spi->mode) {
		u8 spi_mode = spi->mode & (SPI_CPOL | SPI_CPHA);
		u8 pins = 0;

		dev_dbg(&master->dev, "%s: MODE %d\n", __func__, spi->mode);
		if (spi->mode & SPI_LSB_FIRST) {
			switch (spi_mode) {
			case SPI_MODE_0:
			case SPI_MODE_3:
				priv->tx_cmd = TX_BYTES_FE_LSB;
				priv->rx_cmd = RX_BYTES_RE_LSB;
				break;
			case SPI_MODE_1:
			case SPI_MODE_2:
				priv->tx_cmd = TX_BYTES_RE_LSB;
				priv->rx_cmd = RX_BYTES_FE_LSB;
				break;
			}
		} else {
			switch (spi_mode) {
			case SPI_MODE_0:
			case SPI_MODE_3:
				priv->tx_cmd = TX_BYTES_FE_MSB;
				priv->rx_cmd = RX_BYTES_RE_MSB;
				break;
			case SPI_MODE_1:
			case SPI_MODE_2:
				priv->tx_cmd = TX_BYTES_RE_MSB;
				priv->rx_cmd = RX_BYTES_FE_MSB;
				break;
			}
		}

		priv->txrx_cmd = ftdi_spi_txrx_byte_cmd(spi);

		switch (spi_mode) {
		case SPI_MODE_2:
		case SPI_MODE_3:
			pins |= SK;
			break;
		}

		ret = priv->iops->cfg_bus_pins(priv->intf, SK | DO, pins);
		if (ret < 0) {
			dev_err(&master->dev, "IO cfg failed: %d\n", ret);
			return ret;
		}
		priv->last_mode = spi->mode;
	}

	dev_dbg(&master->dev, "%s: mode 0x%x, CMD RX/TX 0x%x/0x%x\n",
		__func__, spi->mode, priv->rx_cmd, priv->tx_cmd);

	ftdi_spi_chipselect(priv, spi, spi->mode & SPI_CS_HIGH);

	ret = -EINVAL;

	list_for_each_entry(t, &msg->transfers, transfer_list) {
		dev_dbg(&master->dev, "%s: cs_change %d, cs %d, len %d\n",
			__func__, t->cs_change, spi->chip_select, t->len);
		dev_dbg(&master->dev, "%s: txb 0x%p, rxb 0x%p, bpw %d\n",
			__func__, t->tx_buf, t->rx_buf, t->bits_per_word);

		if (t->tx_buf && t->rx_buf)
			ret = ftdi_spi_tx_rx(priv, spi, t);
		else if (t->tx_buf)
			ret = ftdi_spi_tx(priv, t->tx_buf, t->len);
		else if (t->rx_buf)
			ret = ftdi_spi_rx(priv, t->rx_buf, t->len);

		dev_dbg(&master->dev, "%s: xfer ret %d\n", __func__, ret);
		if (ret)
			break;

		msg->actual_length += t->len;

		if (t->delay_usecs) {
			u16 us = t->delay_usecs;

			if (us <= 10)
				udelay(us);
			else
				usleep_range(us, us + DIV_ROUND_UP(us, 10));
		}

		if (!t->cs_change)
			continue;

		/* Last transfer with cs_change set, stop keeping CS */
		if (list_is_last(&t->transfer_list, &msg->transfers)) {
			keep_cs = true;
			break;
		}
		ftdi_spi_chipselect(priv, spi, !(spi->mode & SPI_CS_HIGH));
		usleep_range(10, 15);
		ftdi_spi_chipselect(priv, spi, spi->mode & SPI_CS_HIGH);
	}

	dev_dbg(&master->dev, "%s: status %d\n", __func__, ret);
	msg->status = ret;
	spi_finalize_current_message(master);

	if (!keep_cs)
		ftdi_spi_chipselect(priv, spi, !(spi->mode & SPI_CS_HIGH));

	return ret;
}

static int ftdi_mpsse_init(struct ftdi_spi *priv)
{
	struct platform_device *pdev = priv->pdev;
	int ret;

	dev_dbg(&pdev->dev, "MPSSE init\n");

	/* Setup and send off the Hi-Speed specific commands for the FTx232H */
	priv->xfer_buf[0] = DIS_DIV_5;      /* Use 60MHz master clock */
	priv->xfer_buf[1] = DIS_ADAPTIVE;   /* Turn off adaptive clocking */
	priv->xfer_buf[2] = DIS_3_PHASE;    /* Disable three-phase clocking */

	priv->iops->lock(priv->intf);

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 3);
	if (ret < 0) {
		dev_err(&pdev->dev, "Clk cfg failed: %d\n", ret);
		priv->iops->unlock(priv->intf);
		return ret;
	}

	priv->xfer_buf[0] = TCK_DIVISOR;
	priv->xfer_buf[1] = div_value(60000000);
	priv->xfer_buf[2] = div_value(60000000) >> 8;
	dev_dbg(&pdev->dev, "TCK_DIVISOR: 0x%04x 0x%04x\n",
		priv->xfer_buf[1], priv->xfer_buf[2]);

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 3);
	if (ret < 0) {
		dev_err(&pdev->dev, "Clk cfg failed: %d\n", ret);
		priv->iops->unlock(priv->intf);
		return ret;
	}

	priv->iops->unlock(priv->intf);

	ret = priv->iops->cfg_bus_pins(priv->intf, SK | DO, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't init SPI bus pins: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_spi_init_io(struct spi_master *master, int cs)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(master);
	const struct mpsse_spi_platform_data *pd;
	struct platform_device *pdev = priv->pdev;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size;
	char *label;
	int i, size;

	pd = pdev->dev.platform_data;
	size = pd->io_data_len + 1;

	lookup_size = sizeof(*lookup) + size * sizeof(struct gpiod_lookup);
	lookup = devm_kzalloc(&pdev->dev, lookup_size, GFP_KERNEL);
	if (!lookup)
		return -ENOMEM;

	lookup->dev_id = devm_kasprintf(&pdev->dev, GFP_KERNEL, "spi%d.%d",
					master->bus_num, cs);
	if (!lookup->dev_id) {
		devm_kfree(&pdev->dev, lookup);
		return -ENOMEM;
	}
	dev_dbg(&pdev->dev, "LOOKUP ID '%s'\n", lookup->dev_id);

	label = devm_kasprintf(&pdev->dev, GFP_KERNEL, "ftdi-mpsse-gpio.%d",
			       pdev->id);
	if (!label) {
		devm_kfree(&pdev->dev, (void *)lookup->dev_id);
		devm_kfree(&pdev->dev, lookup);
		return -ENOMEM;
	}

	for (i = 0; i < pd->io_data_len; i++) {
		dev_dbg(&pdev->dev, "con_id: '%s' idx: %d, flags: 0x%x\n",
			pd->io_data[i].con_id, pd->io_data[i].idx,
			pd->io_data[i].flags);
		lookup->table[i].key = label;
		lookup->table[i].chip_hwnum = pd->io_data[i].idx;
		lookup->table[i].idx = 0;
		lookup->table[i].con_id = pd->io_data[i].con_id;
		lookup->table[i].flags = pd->io_data[i].flags;
	}

	priv->lookup[cs] = lookup;
	gpiod_add_lookup_table(lookup);
	return 0;
}

static int ftdi_spi_probe(struct platform_device *pdev)
{
	const struct mpsse_spi_platform_data *pd;
	struct device *dev = &pdev->dev;
	struct spi_master *master;
	struct ftdi_spi *priv;
	struct gpio_desc *desc;
	int num_cs, max_cs = 0;
	int i, ret;

	pd = dev->platform_data;
	if (!pd) {
		dev_err(dev, "Missing platform data.\n");
		return -EINVAL;
	}

	if (!pd->ops ||
	    !pd->ops->read_data || !pd->ops->write_data ||
	    !pd->ops->lock || !pd->ops->unlock ||
	    !pd->ops->set_bitmode || !pd->ops->set_baudrate ||
	    !pd->ops->disable_bitbang || !pd->ops->cfg_bus_pins)
		return -EINVAL;

	/* Find max. slave chipselect number */
	num_cs = pd->spi_info_len;
	for (i = 0; i < num_cs; i++) {
		if (max_cs < pd->spi_info[i].chip_select)
			max_cs = pd->spi_info[i].chip_select;
	}

	if (max_cs > 12) {
		dev_err(dev, "Invalid max CS in platform data: %d\n", max_cs);
		return -EINVAL;
	}
	dev_dbg(dev, "CS count %d, max CS %d\n", num_cs, max_cs);
	max_cs += 1; /* including CS0 */

	master = spi_alloc_master(&pdev->dev, sizeof(*priv));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	priv = spi_controller_get_devdata(master);
	priv->master = master;
	priv->pdev = pdev;
	priv->intf = to_usb_interface(dev->parent);
	priv->iops = pd->ops;

	master->bus_num = -1;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP |
			    SPI_CS_HIGH | SPI_LSB_FIRST;
	master->num_chipselect = max_cs;
	master->min_speed_hz = 450;
	master->max_speed_hz = 30000000;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->transfer_one_message = ftdi_spi_transfer_one;
	master->auto_runtime_pm = false;

	priv->cs_gpios = devm_kcalloc(&master->dev, max_cs, sizeof(desc),
				      GFP_KERNEL);
	if (!priv->cs_gpios) {
		spi_controller_put(master);
		return -ENOMEM;
	}

	for (i = 0; i < num_cs; i++) {
		int idx = pd->spi_info[i].chip_select;

		dev_dbg(&pdev->dev, "CS num: %d\n", idx);
		desc = devm_gpiod_get_index(&priv->pdev->dev, "spi-cs",
					    i, GPIOD_OUT_LOW);
		if (IS_ERR(desc)) {
			ret = PTR_ERR(desc);
			dev_err(&pdev->dev, "CS %d gpiod err: %d\n", i, ret);
			continue;
		}
		priv->cs_gpios[idx] = desc;
	}

	ret = spi_register_controller(master);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register spi master\n");
		spi_controller_put(master);
		return ret;
	}

	ret = priv->iops->set_bitmode(priv->intf, 0x00, BITMODE_MPSSE);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to set MPSSE mode\n");
		goto err;
	}

	priv->last_mode = 0xffff;

	ret = ftdi_mpsse_init(priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "MPSSE init failed\n");
		goto err;
	}

	for (i = 0; i < pd->spi_info_len; i++) {
		struct spi_device *sdev;
		int cs;

		dev_dbg(&pdev->dev, "slave: '%s', CS: %d\n",
			pd->spi_info[i].modalias, pd->spi_info[i].chip_select);
		ret = ftdi_spi_init_io(master, pd->spi_info[i].chip_select);
		if (ret < 0) {
			dev_warn(&pdev->dev, "Can't add slave IO: %d\n", ret);
			continue;
		}
		sdev = spi_new_device(master, &pd->spi_info[i]);
		if (!sdev) {
			cs = pd->spi_info[i].chip_select;
			dev_warn(&pdev->dev, "Can't add slave '%s', CS %d\n",
				 pd->spi_info[i].modalias, cs);
			if (priv->lookup[cs]) {
				gpiod_remove_lookup_table(priv->lookup[cs]);
				priv->lookup[cs] = NULL;
			}
		}
	}

	return 0;
err:
	platform_set_drvdata(pdev, NULL);
	spi_unregister_controller(master);
	return ret;
}

static int ftdi_spi_slave_release(struct device *dev, void *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct ftdi_spi *priv = data;
	int cs = spi->chip_select;

	dev_dbg(dev, "%s: remove CS %d\n", __func__, cs);
	spi_unregister_device(to_spi_device(dev));

	if (priv->lookup[cs])
		gpiod_remove_lookup_table(priv->lookup[cs]);
	return 0;
}

static int ftdi_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master;
	struct ftdi_spi *priv;

	master = platform_get_drvdata(pdev);
	priv = spi_controller_get_devdata(master);

	device_for_each_child(&master->dev, priv, ftdi_spi_slave_release);

	spi_unregister_controller(master);
	return 0;
}

static struct platform_driver ftdi_spi_driver = {
	.driver.name	= "ftdi-mpsse-spi",
	.probe		= ftdi_spi_probe,
	.remove		= ftdi_spi_remove,
};
module_platform_driver(ftdi_spi_driver);

MODULE_ALIAS("platform:ftdi-mpsse-spi");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de");
MODULE_DESCRIPTION("FTDI MPSSE SPI master driver");
MODULE_LICENSE("GPL v2");
