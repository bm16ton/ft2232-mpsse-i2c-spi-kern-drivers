// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/mutex.h>

#include "mpsse.h"

int i2c_ftdi_100k;
module_param_named(100k, i2c_ftdi_100k, int, 0444);
MODULE_PARM_DESC(100k, "Drop bus from 400k to 100k");

int i2c_ftdi_adaptive;
module_param_named(adaptive, i2c_ftdi_adaptive, int, 0444);
MODULE_PARM_DESC(adaptive, "clock stretching hack NOT WORKING");

const int FTDI_IO_TIMEOUT = 5000;
//const unsigned FTDI_I2C_FREQ = 400000;
const size_t FTDI_IO_BUFFER_SIZE = 65536;
//const u16 FTDI_BIT_MODE_RESET = 0x0000;
//const u16 FTDI_BIT_MODE_MPSSE = 0x0200;
const u16 FTDI_BIT_MODE_RESET = 0x00ff;
const u16 FTDI_BIT_MODE_MPSSE = 0x02ff;

struct ftdi_usb {
	struct usb_device *udev;
	struct usb_interface *interface;
	u8 *buffer;
	size_t buffer_size;
	struct i2c_adapter adapter;
	// Timeout in milliseconds for USB IO operations
	int io_timeout;
	// I2C bus frequency
	unsigned freq;
};

static int ftdi_mpsse_write(
	struct ftdi_usb *ftdi, u8 *data, size_t size, size_t *written)
{
	int actual_length;
	int ret;

	ret = usb_bulk_msg(
		ftdi->udev, usb_sndbulkpipe(ftdi->udev, 4),
		/* data = */data,
		/* len = */size,
		/* actual_length = */&actual_length,
		ftdi->io_timeout);
	*written = actual_length;
	return ret;
}

static int ftdi_mpsse_read(
	struct ftdi_usb *ftdi, u8 *data, size_t size, size_t *read)
{
	int actual_length;
	int ret;

	ret = usb_bulk_msg(
		ftdi->udev, usb_rcvbulkpipe(ftdi->udev, 3),
		/* data = */data,
		/* len = */size,
		/* actual_length = */&actual_length,
		ftdi->io_timeout);
	*read = actual_length;
	return ret;
}

static int ftdi_mpsse_submit(
	struct ftdi_usb *ftdi, const struct ftdi_mpsse_cmd *cmd)
{
	size_t written = 0;

	while (written < cmd->offset) {
		size_t actual_length;
		int ret;

		ret = ftdi_mpsse_write(ftdi,
				       cmd->buffer + written,
				       cmd->offset - written,
				       &actual_length);
		if (ret < 0)
			return ret;

		written += actual_length;
	}

	return 0;
}

static int ftdi_mpsse_receive(struct ftdi_usb *ftdi, u8 *data, size_t size)
{
	size_t read = 0;

	while (read < size) {
		size_t actual_length;
		int ret;

		ret = ftdi_mpsse_read(ftdi,
				      ftdi->buffer,
				      min(ftdi->buffer_size, size - read + 2),
				      &actual_length);
		if (ret < 0)
			return ret;

		if (actual_length < 2)
			return -EIO;

		memcpy(data + read, ftdi->buffer + 2, actual_length - 2);
		read += actual_length - 2;
	}

	return 0;
}

static int ftdi_i2c_idle(struct ftdi_usb *ftdi)
{
	struct ftdi_mpsse_cmd cmd;
	int ret;

	ftdi_mpsse_cmd_setup(&cmd, ftdi->buffer, ftdi->buffer_size);
	ret = ftdi_mpsse_set_output(&cmd, 0x40fb, 0xffff);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);
	return 0;
}

static int ftdi_i2c_start(struct ftdi_usb *ftdi)
{
	struct ftdi_mpsse_cmd cmd;
	int ret;

	ftdi_mpsse_cmd_setup(&cmd, ftdi->buffer, ftdi->buffer_size);
	ret = ftdi_mpsse_set_output(&cmd, 0x00fb, 0x00fd);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);

	ftdi_mpsse_cmd_reset(&cmd);
	ret = ftdi_mpsse_set_output(&cmd, 0x40fb, 0x00fc);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);
	return 0;
}

static int ftdi_i2c_stop(struct ftdi_usb *ftdi)
{
	struct ftdi_mpsse_cmd cmd;
	int ret;

	ftdi_mpsse_cmd_setup(&cmd, ftdi->buffer, ftdi->buffer_size);
	ret = ftdi_mpsse_set_output(&cmd, 0x00fb, 0x00fc);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);

	ftdi_mpsse_cmd_reset(&cmd);
	ret = ftdi_mpsse_set_output(&cmd, 0x00fb, 0x00fd);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);

	ftdi_mpsse_cmd_reset(&cmd);
	ret = ftdi_mpsse_set_output(&cmd, 0x40fb, 0xffff);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);
	return 0;
}

static int ftdi_i2c_write_byte(struct ftdi_usb *ftdi, u8 byte)
{
	struct ftdi_mpsse_cmd cmd;
	int ret;

	ftdi_mpsse_cmd_setup(&cmd, ftdi->buffer, ftdi->buffer_size);
	ret = ftdi_mpsse_write_bytes(&cmd, &byte, sizeof(byte));
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_set_output(&cmd, 0x00fb, 0x00fe);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_read_bits(&cmd, 1);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_complete(&cmd);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_receive(ftdi, &byte, sizeof(byte));
	if (ret < 0)
		return ret;

	if ((byte & 0x1) != 0)
		return -EIO;

	return 0;
}

static int ftdi_i2c_write_bytes(
	struct ftdi_usb *ftdi, const u8 *data, size_t size)
{
	size_t i;

	for (i = 0; i < size; ++i) {
		int ret = ftdi_i2c_write_byte(ftdi, data[i]);

		if (ret < 0)
			return ret;
	}
	return 0;
}

static int ftdi_i2c_write_addr(struct ftdi_usb *ftdi, u8 i2c_addr, int read)
{
	const u8 byte = read ? ((i2c_addr << 1) | 1) : (i2c_addr << 1);
	return ftdi_i2c_write_byte(ftdi, byte);
}

static int ftdi_i2c_read_bytes(struct ftdi_usb *ftdi, u8 *data, size_t size)
{
	struct ftdi_mpsse_cmd cmd;
	size_t i;
	int ret;

	ftdi_mpsse_cmd_setup(&cmd, ftdi->buffer, ftdi->buffer_size);
	ret = ftdi_mpsse_read_bytes(&cmd, 1);
	if (ret < 0)
		return ret;

	for (i = 1; i < size; ++i) {
		ret = ftdi_mpsse_write_bits(&cmd, 0x00, 1);
		if (ret < 0)
			return ret;

		ret = ftdi_mpsse_set_output(&cmd, 0x00fb, 0x00fe);
		if (ret < 0)
			return ret;

		ret = ftdi_mpsse_read_bytes(&cmd, 1);
		if (ret < 0)
			return ret;
	}

	ret = ftdi_mpsse_write_bits(&cmd, 0xff, 1);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_set_output(&cmd, 0x00fb, 0x00fe);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_complete(&cmd);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	return ftdi_mpsse_receive(ftdi, data, size);
}

static int ftdi_reset(struct ftdi_usb *ftdi);

static int ftdi_usb_i2c_xfer(struct i2c_adapter *adapter,
			     struct i2c_msg *msg, int num)
{
	struct ftdi_usb *ftdi = (struct ftdi_usb *)adapter->algo_data;
	int i;
	int ret;

	for (i = 0; i < num; ++i) {
		const int read = (msg[i].flags & I2C_M_RD) != 0;

		ret = ftdi_i2c_start(ftdi);
		if (ret < 0)
			goto err;

		ret = ftdi_i2c_write_addr(ftdi, msg[i].addr, read);
		if (ret < 0)
			goto err;

		usleep_range(100, 200);
		if (read)
			ret = ftdi_i2c_read_bytes(
				ftdi, msg[i].buf, msg[i].len);
		else
			ret = ftdi_i2c_write_bytes(
				ftdi, msg[i].buf, msg[i].len);
		if (ret < 0)
			goto err;

		ret = ftdi_i2c_stop(ftdi);
		if (ret < 0)
			goto err;

		usleep_range(100, 200);
	}

	return num;

err:
	ftdi_reset(ftdi);
	return ret;
}

static u32 ftdi_usb_i2c_func(struct i2c_adapter *adapter)
{
	(void) adapter;
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ftdi_usb_i2c_algo = {
	.master_xfer = ftdi_usb_i2c_xfer,
	.functionality = ftdi_usb_i2c_func,
};

static const struct usb_device_id ftdi_id_table[] = {
	{ USB_DEVICE(0x0403, 0x6011) },
	{ USB_DEVICE(0x0403, 0x6010) },
    { }
};
MODULE_DEVICE_TABLE(usb, ftdi_id_table);

static void ftdi_usb_delete(struct ftdi_usb *ftdi)
{
	usb_put_intf(ftdi->interface);
	usb_put_dev(ftdi->udev);
	kfree(ftdi->buffer);
	kfree(ftdi);
}

static int ftdi_mpsse_verify(struct ftdi_usb *ftdi)
{
	struct ftdi_mpsse_cmd cmd;
	size_t received;
	int ret;

	ftdi_mpsse_cmd_setup(&cmd, ftdi->buffer, ftdi->buffer_size);
	ret = ftdi_mpsse_command(&cmd, 0xaa);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_read(
		ftdi, ftdi->buffer, ftdi->buffer_size, &received);
	if (ret < 0)
		return ret;

	if (received != 4 || ftdi->buffer[2] != 0xfa || ftdi->buffer[3] != 0xaa)
		return -EIO;

	ftdi_mpsse_cmd_reset(&cmd);
	ret = ftdi_mpsse_command(&cmd, 0xab);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_submit(ftdi, &cmd);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_read(
		ftdi, ftdi->buffer, ftdi->buffer_size, &received);
	if (ret < 0)
		return ret;

	if (received != 4 || ftdi->buffer[2] != 0xfa || ftdi->buffer[3] != 0xab)
		return -EIO;

	return 0;
}

static int ftdi_mpsse_i2c_setup(struct ftdi_usb *ftdi)
{
	struct ftdi_mpsse_cmd cmd;
	int ret;

	ftdi_mpsse_cmd_setup(&cmd, ftdi->buffer, ftdi->buffer_size);
		if (i2c_ftdi_adaptive) {
	ret = ftdi_mpsse_enable_adaptive_clocking(&cmd);
	} else {
	ret = ftdi_mpsse_disable_adaptive_clocking(&cmd);
	}
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_disable_loopback(&cmd);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_set_drive0_pins(&cmd, 0x7);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_set_freq(&cmd, ftdi->freq);
	if (ret < 0)
		return ret;

	return ftdi_mpsse_submit(ftdi, &cmd);
}

static int ftdi_set_bit_mode(struct ftdi_usb *ftdi, u16 mode)
{
	int actual_length;
	int ret;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */0x0b,
		/* bRequestType = */0x40,
		/* wValue = */mode,
		/* wIndex =  */0x0002,
		/* data = */NULL,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	// After changing the mode we need to read 2 bytes of status, but since
	// I don't know the format I read them check that it's just 2 bytes and
	// ignore the actual values.
	ret = usb_bulk_msg(
		ftdi->udev, usb_rcvbulkpipe(ftdi->udev, 3),
		/* data = */ftdi->buffer,
		/* len = */ftdi->buffer_size,
		/* actual_length = */&actual_length,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	if (actual_length != 2)
		return -EIO;

	return 0;
}

static int ftdi_disable_special_characters(struct ftdi_usb *ftdi)
{
	int ret;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */0x06,
		/* bRequestType = */0x40,
		/* wValue = */0x0000,
		/* wIndex =  */0x0002,
		/* data = */NULL,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */0x07,
		/* bRequestType = */0x40,
		/* wValue = */0x0000,
		/* wIndex =  */0x0002,
		/* data = */NULL,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	return 0;
}

// The FTDI interface is not described in any publicly available document.
// Instead of the documentation I looked at what the FTDI userspace driver does
// and replicated it here. I didn't give names to the contstants used below
// because for me they are as much magic numbers as they are for the reader of
// the code. That being said in drivers/usb/serial/ftdi_sio.c we can find that:
//
//   * bRequestType for reset is 0x40 (FTDI_SIO_RESET_REQUEST_TYPE)
//   * bRequest for reset is 0x00 (FTDI_SIO_RESET_REQUEST)
//   * wIndex is the port number and on F232H it's always 0
//
// The meaning of wValue is not clear to me still. drivers/usb/serial/ftdi_sio.c
// uses only value 0 (FTDI_SIO_RESET_SIO), but the userspace driver when doing
// reset uses 0, 1 and 2.
static int ftdi_reset(struct ftdi_usb *ftdi)
{
	int ret;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */0x00,
		/* bRequestType = */0x40,
		/* wValue = */0x0000,
		/* wIndex =  */0x0002,
		/* data = */NULL,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */0x03,
		/* bRequestType = */0x40,
		/* wValue = */0x4138,
		/* wIndex =  */0x0002,
		/* data = */NULL,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */0x00,
		/* bRequestType = */0x40,
		/* wValue = */0x0000,
		/* wIndex =  */0x0002,
		/* data = */NULL,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */0x00,
		/* bRequestType = */0x40,
		/* wValue = */0x0001,
		/* wIndex =  */0x0002,
		/* data = */NULL,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */0x00,
		/* bRequestType = */0x40,
		/* wValue = */0x0002,
		/* wIndex =  */0x0002,
		/* data = */NULL,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		return ret;

	ret = ftdi_disable_special_characters(ftdi);
	if (ret < 0)
		return ret;

	ret = ftdi_set_bit_mode(ftdi, FTDI_BIT_MODE_RESET);
	if (ret < 0)
		return ret;

	ret = ftdi_set_bit_mode(ftdi, FTDI_BIT_MODE_MPSSE);
	if (ret < 0)
		return ret;

	ret = ftdi_mpsse_i2c_setup(ftdi);
	if (ret < 0)
		return ret;

//  16ton this verify errors out
	ret = ftdi_mpsse_verify(ftdi);
//	if (ret < 0)
//		return ret;

	return ftdi_i2c_idle(ftdi);
}

static int ftdi_usb_probe(struct usb_interface *interface,
			  const struct usb_device_id *id)
{
	struct usb_device *dev = interface_to_usbdev(interface);
	struct ftdi_usb *ftdi;
	int ret;
	int inf;
	(void) id;

	ftdi = kzalloc(sizeof(*ftdi), GFP_KERNEL);
	if (!ftdi)
		return -ENOMEM;

	ftdi->udev = usb_get_dev(dev);
	ftdi->interface = usb_get_intf(interface);
	inf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;
	if (inf > 1) {
		dev_info(&interface->dev, "Ignoring Interface\n");
		return -ENODEV;
		}
	if (inf < 1) {
		dev_info(&interface->dev, "Ignoring Interface\n");
		return -ENODEV;
		}
	ftdi->io_timeout = FTDI_IO_TIMEOUT;
	if (i2c_ftdi_100k) {
	ftdi->freq = 100000;
	} else {
	ftdi->freq  = 400000;
	}
//	ftdi->freq = FTDI_I2C_FREQ;
	ftdi->buffer = kzalloc(FTDI_IO_BUFFER_SIZE, GFP_KERNEL);
	ftdi->buffer_size = FTDI_IO_BUFFER_SIZE;
	if (!ftdi->buffer) {
		dev_err(&interface->dev,
			"Failed to initialize the FTDI-based device: %d\n",
			-ENOMEM);
		ftdi_usb_delete(ftdi);
		return -ENOMEM;
	}

	ret = ftdi_reset(ftdi);
	if (ret < 0) {
		dev_err(&interface->dev,
			"Failed to reset FTDI-based device: %d\n", ret);
		ftdi_usb_delete(ftdi);
		return ret;
	}

	ftdi->adapter.owner = THIS_MODULE;
	ftdi->adapter.algo = &ftdi_usb_i2c_algo;
	ftdi->adapter.algo_data = ftdi;
	ftdi->adapter.dev.parent = &interface->dev;
	snprintf(ftdi->adapter.name, sizeof(ftdi->adapter.name),
		 "FTDI USB-to-I2C at bus %03d device %03d",
		 dev->bus->busnum, dev->devnum);
	i2c_add_adapter(&ftdi->adapter);

	usb_set_intfdata(interface, ftdi);
	dev_info(&interface->dev, "Initialized FTDI-based device\n");
	return 0;
}

static void ftdi_usb_disconnect(struct usb_interface *interface)
{
	struct ftdi_usb *ftdi = usb_get_intfdata(interface);

	i2c_del_adapter(&ftdi->adapter);
	usb_set_intfdata(interface, NULL);
	ftdi_usb_delete(ftdi);
	dev_info(&interface->dev, "FTDI-based device has been disconnected\n");
}

static struct usb_driver ftdi_usb_driver = {
	.name = "ftdi_usb",
	.probe = ftdi_usb_probe,
	.disconnect = ftdi_usb_disconnect,
	.id_table = ftdi_id_table,
};

module_usb_driver(ftdi_usb_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FTDI USB-to-serial based something");
MODULE_AUTHOR("Ben Maddocks <bm16ton@gmail.com), Krinkin Mike <krinkin.m.u@gmail.com>");
