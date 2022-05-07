
// SPDX-License-Identifier: GPL-2.0
//
// FTDI FT232H MPSSE SPI controller driver
//
// Copyright (C) 2017 - 2018 DENX Software Engineering
// Anatolij Gustschin <agust@denx.de>
//

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/device.h>

#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/usb.h>
#include <linux/usb/ft232h-intf.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/stringify.h>

int spi_ftdi_mpsse_debug;
module_param_named(debug, spi_ftdi_mpsse_debug, int, 0444);
MODULE_PARM_DESC(debug, "Turn on tx/rx details");

enum gpiol {
	MPSSE_SK	= BIT(0),
	MPSSE_DO	= BIT(1),
	MPSSE_DI	= BIT(2),
	MPSSE_CS	= BIT(3),
};

// FTDI_MPSSE_GPIOS = ft232h_intf_get_numgpio(intf);

struct ftdi_spi {
	struct platform_device *pdev;
	struct usb_interface *intf;
	struct spi_controller *master;
	const struct ft232h_intf_ops *iops;
	struct gpiod_lookup_table *lookup[FTDI_MPSSE_GPIOS5];
	struct gpio_desc **cs_gpios;
	struct gpio_desc **dc_gpios;
	struct gpio_desc **reset_gpios;
	struct gpio_desc **interrupts_gpios;

	u8 txrx_cmd;
	u8 rx_cmd;
	u8 tx_cmd;
	u8 xfer_buf[SZ_64K];
	u16 last_mode;
	u32 last_speed_hz;
	int ftmodel;
	int gpionum;
};

static void ftdi_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(spi->master);
	u16 cs = spi->chip_select;

	if (spi_ftdi_mpsse_debug) {
	dev_dbg(&priv->pdev->dev, "%s: CS %u, cs mode %d, val %d\n",
		__func__, cs, (spi->mode & SPI_CS_HIGH), enable);
	}

	gpiod_set_raw_value_cansleep(priv->cs_gpios[cs], enable);
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
		dev_warn(&priv->pdev->dev, "loopback %d failed\n", on);
	return ret;
}

static int ftdi_spi_tx_rx(struct ftdi_spi *priv, struct spi_device *spi,
			  struct spi_transfer *t)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct device *dev = &priv->pdev->dev;
	void *rx_offs;
	const void *tx_offs;
	size_t remaining, stride;
        size_t rx_remaining;
	size_t rx_stride;
	int ret, tout = 10;
	const u8 *tx_data = t->tx_buf;
	u8 *rx_data = t->rx_buf;

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
		priv->xfer_buf[3 + stride] = SEND_IMMEDIATE;
		print_hex_dump_debug("WR: ", DUMP_PREFIX_OFFSET, 16, 1,
				     priv->xfer_buf, stride + 3, 1);

		ret = ops->write_data(priv->intf, priv->xfer_buf, stride + 4);
		if (ret < 0) {
			dev_err(dev, "%s: xfer failed %d\n", __func__, ret);
			goto fail;
		}
		if (spi_ftdi_mpsse_debug) {
		dev_dbg(dev, "%s: WR %zu byte(s), TXRX CMD 0x%02x\n",
			__func__, stride, priv->txrx_cmd);
		}
		rx_stride = min_t(size_t, stride, SZ_512);

		tout = 10;
		rx_remaining = stride;
		do {
			rx_stride = min_t(size_t, rx_remaining, SZ_512);
			ret = ops->read_data(priv->intf, priv->xfer_buf, rx_stride);
			if (ret < 0)
				goto fail;
			if (!ret) {
				if (--tout) {
					continue;
				}
				dev_err(dev, "Read timeout\n");
				ret = -ETIMEDOUT;
				goto fail;
			}
			print_hex_dump_debug("RD: ", DUMP_PREFIX_OFFSET, 16, 1,
				     priv->xfer_buf, ret, 1);
			memcpy(rx_offs, priv->xfer_buf, ret);
			rx_offs += ret;
			rx_remaining -= ret;
		} while (rx_remaining);

		remaining -= stride;
		tx_offs += stride;
		if (spi_ftdi_mpsse_debug) {
		dev_dbg(dev, "%s: WR remains %zu\n", __func__, remaining);
		}
	}

	ret = 0;

fail:
	if (spi->mode & SPI_LOOP)
		ftdi_spi_loopback_cfg(priv, 0);

err:
	ops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_push_buf(struct ftdi_spi *priv, const void *buf, size_t len)
{
	size_t bytesleft = len;
	int ret;

	do {
		ret = priv->iops->write_data(priv->intf, buf, bytesleft);
		if (ret < 0)
			return ret;

		buf += ret;
		bytesleft -= ret;
	} while (bytesleft);

	return len;
}

static int ftdi_spi_tx(struct ftdi_spi *priv, struct spi_transfer *xfer)
{
	const void *tx_offs;
	size_t remaining, stride;
	int ret;

	priv->iops->lock(priv->intf);

	tx_offs = xfer->tx_buf;
	remaining = xfer->len;

	do {
		stride = min_t(size_t, remaining, sizeof(priv->xfer_buf) - 3);

		priv->xfer_buf[0] = priv->tx_cmd;
		priv->xfer_buf[1] = stride - 1;
		priv->xfer_buf[2] = (stride - 1) >> 8;

		memcpy(&priv->xfer_buf[3], tx_offs, stride);

		ret = ftdi_spi_push_buf(priv, priv->xfer_buf, stride + 3);
		if (ret < 0) {
			dev_dbg(&priv->pdev->dev, "%s: tx failed %d\n",
				__func__, ret);
			goto err;
		}
		if (spi_ftdi_mpsse_debug) {
		dev_dbg(&priv->pdev->dev, "%s: %zu byte(s) done\n",
			__func__, stride);
		}
		remaining -= stride;
		tx_offs += stride;
	} while (remaining);

	ret = 0;
err:
	priv->iops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_rx(struct ftdi_spi *priv, struct spi_transfer *xfer)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct device *dev = &priv->pdev->dev;
	size_t remaining, stride;
	int ret, tout = 10;
	void *rx_offs;
	if (spi_ftdi_mpsse_debug) {
	dev_dbg(dev, "%s: CMD 0x%02x, len %u\n",
		__func__, priv->rx_cmd, xfer->len);
	}
	priv->xfer_buf[0] = priv->rx_cmd;
	priv->xfer_buf[1] = xfer->len - 1;
	priv->xfer_buf[2] = (xfer->len - 1) >> 8;
	priv->xfer_buf[3] = SEND_IMMEDIATE;
	ops->lock(priv->intf);

	ret = ops->write_data(priv->intf, priv->xfer_buf, 4);
	if (ret < 0)
		goto err;

	remaining = xfer->len;
	rx_offs = xfer->rx_buf;

	do {
		stride = min_t(size_t, remaining, SZ_512);

		ret = ops->read_data(priv->intf, priv->xfer_buf, stride);
		if (ret < 0)
			goto err;

		if (!ret) {
			dev_dbg(dev, "Waiting for data (read: %02X), tout %d\n",
				ret, tout);
			if (--tout)
				continue;

			dev_dbg(dev, "read timeout...\n");
			ret = -ETIMEDOUT;
			goto err;
		}

		memcpy(rx_offs, priv->xfer_buf, ret);
		if (spi_ftdi_mpsse_debug) {
		dev_dbg(dev, "%s: %d byte(s)\n", __func__, ret);
		}
		rx_offs += ret;
		remaining -= ret;
	} while (remaining);

	ret = 0;
err:
	ops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_transfer_one(struct spi_controller *ctlr,
				 struct spi_device *spi,
				 struct spi_transfer *xfer)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(ctlr);
	struct device *dev = &priv->pdev->dev;
	int ret = 0;

	if (!xfer->len)
		return 0;

	if (priv->last_speed_hz != xfer->speed_hz) {
		dev_dbg(dev, "%s: new speed %u\n", __func__, (int)xfer->speed_hz);
		ret = priv->iops->set_clock(priv->intf, xfer->speed_hz);
		if (ret < 0) {
			dev_err(dev, "Set clock(%u) failed: %d\n", xfer->speed_hz, ret);
			return ret;
		}
		priv->last_speed_hz = xfer->speed_hz;
	}

	if (priv->last_mode != spi->mode) {
		u8 spi_mode = spi->mode & (SPI_CPOL | SPI_CPHA);
		u8 pins = 0;

		if (spi_ftdi_mpsse_debug) {
		dev_dbg(dev, "%s: MODE 0x%x\n", __func__, spi->mode);
		}

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
			pins |= MPSSE_SK;
			break;
		}

		ret = priv->iops->cfg_bus_pins(priv->intf,
					       MPSSE_SK | MPSSE_DO, pins);
		if (ret < 0) {
			dev_err(dev, "IO cfg failed: %d\n", ret);
			return ret;
		}
		priv->last_mode = spi->mode;
	}
	if (spi_ftdi_mpsse_debug) {
	dev_dbg(dev, "%s: mode 0x%x, CMD RX/TX 0x%x/0x%x\n",
		__func__, spi->mode, priv->rx_cmd, priv->tx_cmd);
	}

	if (xfer->tx_buf && xfer->rx_buf)
		ret = ftdi_spi_tx_rx(priv, spi, xfer);
	else if (xfer->tx_buf)
		ret = ftdi_spi_tx(priv, xfer);
	else if (xfer->rx_buf)
		ret = ftdi_spi_rx(priv, xfer);

	if (spi_ftdi_mpsse_debug)
	dev_dbg(dev, "%s: xfer ret %d\n", __func__, ret);

	spi_finalize_current_transfer(ctlr);
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

	ret = priv->iops->cfg_bus_pins(priv->intf, MPSSE_SK | MPSSE_DO, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't init SPI bus pins: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_spi_init_io(struct spi_controller *master, unsigned int dev_idx)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(master);
	struct platform_device *pdev = priv->pdev;
	const struct mpsse_spi_platform_data *pd;
	const struct mpsse_spi_dev_data *data;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size, size;
	char *label;
	unsigned int i;
	u16 cs;

	pd = pdev->dev.platform_data;

	data = pd->spi_info[dev_idx].platform_data;
	if (!data || data->magic != FTDI_MPSSE_IO_DESC_MAGIC)
		return 0;

	size = data->desc_len + 1;

	lookup_size = sizeof(*lookup) + size * sizeof(struct gpiod_lookup);
	lookup = devm_kzalloc(&pdev->dev, lookup_size, GFP_KERNEL);
	if (!lookup)
		return -ENOMEM;

	cs = pd->spi_info[dev_idx].chip_select;


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

	for (i = 0; i < data->desc_len; i++) {
		dev_dbg(&pdev->dev, "con_id: '%s' idx: %d, flags: 0x%x\n",
			data->desc[i].con_id, data->desc[i].idx,
			data->desc[i].flags);
		lookup->table[i].key = label;
		lookup->table[i].chip_hwnum = data->desc[i].idx;
		lookup->table[i].idx = 0;
		lookup->table[i].con_id = data->desc[i].con_id;
		lookup->table[i].flags = data->desc[i].flags;
	}

	priv->lookup[cs] = lookup;
	gpiod_add_lookup_table(lookup);
	return 0;
}

static int ftdi_spi_probe(struct platform_device *pdev)
{
	const struct mpsse_spi_platform_data *pd;
	struct device *dev = &pdev->dev;
	struct spi_controller *master;
	struct ftdi_spi *priv;
	struct gpio_desc *desc;
	u16 dc, reset, interrupts, num_cs, max_cs = 0;
	unsigned int i;
	int ret;
//	int ret2;
	int model;
	int numgpio;

	int ftmod2;
	int ftmod4;

	ftmod2 = 2232;
	ftmod4 = 4232;

	pd = dev->platform_data;
	if (!pd) {
		dev_err(dev, "Missing platform data.\n");
		return -EINVAL;
	}

	if (!pd->ops ||
	    !pd->ops->read_data || !pd->ops->write_data ||
	    !pd->ops->lock || !pd->ops->unlock ||
	    !pd->ops->set_bitmode || !pd->ops->set_baudrate ||
	    !pd->ops->disable_bitbang || !pd->ops->cfg_bus_pins ||
	    !pd->ops->set_clock || !pd->ops->set_latency)
	    	return -EINVAL;

	if (pd->spi_info_len > FTDI_MPSSE_GPIOS13)
		return -EINVAL;

	/* Find max. slave chipselect number */
	num_cs = pd->spi_info_len;
	for (i = 0; i < num_cs; i++) {
		if (max_cs < pd->spi_info[i].chip_select)
			max_cs = pd->spi_info[i].chip_select;
	}

	if (max_cs > FTDI_MPSSE_GPIOS5 - 1) {
		dev_err(dev, "Invalid max CS in platform data: %u\n", max_cs);
		return -EINVAL;
	}
	dev_dbg(dev, "CS count %u, max CS %u\n", num_cs, max_cs);
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

	model = ft232h_intf_get_model(priv->intf);
	priv->ftmodel = model;
	dev_info(dev, "model num %d\n", priv->ftmodel);

    numgpio = ft232h_intf_get_numgpio(priv->intf);
	priv->gpionum = numgpio;
	dev_info(dev, "gpio num %d\n", priv->gpionum);

	master->bus_num = -1;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP |
			    SPI_CS_HIGH | SPI_LSB_FIRST;
	master->num_chipselect = max_cs;
	master->min_speed_hz = 450;
//	master->min_speed_hz = 1000000;
	master->max_speed_hz = 30000000;
//	master->max_speed_hz = 25000000;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->set_cs = ftdi_spi_set_cs;
	master->transfer_one = ftdi_spi_transfer_one;
	master->auto_runtime_pm = false;

	priv->cs_gpios = devm_kcalloc(&master->dev, max_cs, sizeof(desc),
				      GFP_KERNEL);
	if (!priv->cs_gpios) {
		spi_controller_put(master);
		return -ENOMEM;
	}

	priv->dc_gpios = devm_kcalloc(&master->dev, dc, sizeof(desc),
				      GFP_KERNEL);

	priv->reset_gpios = devm_kcalloc(&master->dev, reset, sizeof(desc),
				      GFP_KERNEL);

	priv->interrupts_gpios = devm_kcalloc(&master->dev, interrupts, sizeof(desc),
				      GFP_KERNEL);


	for (i = 0; i < num_cs; i++) {
		unsigned int idx = pd->spi_info[i].chip_select;

		dev_dbg(&pdev->dev, "CS num: %u\n", idx);
		desc = devm_gpiod_get_index(&priv->pdev->dev, "spi-cs",
					    i, GPIOD_OUT_LOW);
		if (IS_ERR(desc)) {
			ret = PTR_ERR(desc);
			dev_err(&pdev->dev, "CS %u gpiod err: %d\n", i, ret);
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

	ret = priv->iops->set_latency(priv->intf, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "Set latency failed\n");
		goto err;
        }

	for (i = 0; i < pd->spi_info_len; i++) {
		struct spi_device *sdev;
		u16 cs;

		dev_dbg(&pdev->dev, "slave: '%s', CS: %u\n",
			pd->spi_info[i].modalias, pd->spi_info[i].chip_select);

		ret = ftdi_spi_init_io(master, i);
		if (ret < 0) {
			dev_warn(&pdev->dev, "Can't add slave IO: %d\n", ret);
			continue;
		}
		sdev = spi_new_device(master, &pd->spi_info[i]);
		if (!sdev) {
			cs = pd->spi_info[i].chip_select;
			dev_warn(&pdev->dev, "Can't add slave '%s', CS %u\n",
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
	u16 cs = spi->chip_select;

	dev_dbg(dev, "%s: remove CS %u\n", __func__, cs);
	spi_unregister_device(to_spi_device(dev));

	if (priv->lookup[cs])
		gpiod_remove_lookup_table(priv->lookup[cs]);
	return 0;
}

static int ftdi_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *master;
	struct ftdi_spi *priv;

	master = platform_get_drvdata(pdev);
	priv = spi_controller_get_devdata(master);

	device_for_each_child(&master->dev, priv, ftdi_spi_slave_release);

	spi_unregister_controller(master);
	return 0;
}

static const struct of_device_id ftdi_of_match[] = {
	{ .compatible = "ftdi,ftdi-mpsse-spi", },
	{ .compatible = "ftdi,spi_ftdi_mpsse", },
	{},
};
MODULE_DEVICE_TABLE(of, ftdi_of_match);

static const struct spi_device_id ftdi_spi_ids[] = {
	{ .name = "ftdi-mpsse-spi", (unsigned long)ftdi_spi_probe },
	{ .name = "spi-ftdi-mpsse", (unsigned long)ftdi_spi_probe },
	{},
};
MODULE_DEVICE_TABLE(spi, ftdi_spi_ids);

static struct platform_driver spi_ftdi_mpsse = {
	.driver		= {
				.name	= "spi-ftdi-mpsse",
				.of_match_table = of_match_ptr(ftdi_of_match),
	},
	.probe		= ftdi_spi_probe,
	.remove		= ftdi_spi_remove,
};
module_platform_driver(spi_ftdi_mpsse);

MODULE_ALIAS("platform:spi_ftdi_mpsse");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de");
MODULE_DESCRIPTION("FTDI MPSSE SPI master driver");
MODULE_LICENSE("GPL v2");


