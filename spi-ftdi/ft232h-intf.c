// SPDX-License-Identifier: GPL-2.0
/*
 * FTDI FT232H interface driver for ARRI FPGA configuration
 *
 * Copyright (C) 2017 - 2018 DENX Software Engineering
 * Anatolij Gustschin <agust@denx.de>
 *
 *    Figure: FT232H, FPGA Devices and Drivers Relationship
 *
 *      +-------------+
 *      |             |
 *      |  STRATIX V  |PS-SPI         FT245 FIFO & GPIO
 *      |             +-----+    +-------------------+
 *      |  on Board 1 |     +    +                   |
 *      |             |                         +----+---+
 *      |  PCIe       |   ADBUS&ACBUS           |  CPLD  |
 *      +---+---------+ Connection Options      +----+---+
 *          ^          (MPSSE or FIFO&GPIO)          |
 *          +                  +              +------+-------+
 *     altera-cvp  +-----------+----------+   |     FPP      |
 *                 |        FT232H        |   |              |
 *                 |     0x0403:0x7148    |   |   ARRIA 10   |
 *                 |     0x0403:0x7149    |   |              |
 *                 +----------+-----------+   |  on Board 2  |
 *                            |               |              |
 *                +-----------+------------+  |        PCIe  |
 *        creates | ft232h-intf (USB misc) |  +----------+---+
 *       platform |     bulk/ctrl xfer     |             ^
 *        devices |ACBUS GPIO Ctrl (0x7148)|             |
 *         below  |MPSSE GPIO Ctrl (0x7149)|             |
 *                +-------+-------+--------+             |
 *                        |       |                      |
 *           for     +----+       +------+    for        |
 *        PID 0x7149 |                   | PID 0x7148    |
 *         +---------+--------+  +-------+---------+     |
 *         |  ftdi-mpsse-spi  |  |                 |     |
 *         | altera-ps-spi in |  |ftdi-fifo-fpp-mgr|     |
 *         |   spi_board_info |  |                 |     |
 *         +---------+--------+  +--------+--------+     |
 *                   ^                    ^              |
 *        Drivers:   |                    |              |
 *                   +                    |              |
 *      MPSSE SPI master(spi-ftdi-mpsse)  |              +
 *                   ^                    |              |
 *                   |                    +              +
 *             altera-ps-spi        ftdi-fifo-fpp    altera-cvp
 *              FPGA Manager         FPGA Manager   FPGA Manager
 *
 *
 * When using your custom USB product ID, this FT232H interface driver
 * also allows to register the GPIO controller for CBUS pins or for
 * MPSSE GPIO pins. Below are examples how to use the driver as CBUS-
 * or MPSSE-GPIO controller.
 *
 * For CBUS-GPIOs add new entry with your PID to ft232h_intf_table[]:
 * static const struct ft232h_intf_info ftdi_cbus_gpio_intf_info = {
 *	.use_cbus_gpio_ctrl = true,
 * };
 * { USB_DEVICE(FTDI_VID, PID),
 *   .driver_info = (kernel_ulong_t)&ftdi_cbus_gpio_intf_info },
 *
 * For MPSSE-GPIO add new entry with your PID to ft232h_intf_table[]:
 * static const struct ft232h_intf_info ftdi_mpsse_gpio_intf_info = {
 *	.use_mpsse_gpio_ctrl = true,
 * };
 * { USB_DEVICE(FTDI_VID, PID),
 *   .driver_info = (kernel_ulong_t)&ftdi_mpsse_gpio_intf_info },
 *
 * With custom USB product IDs it is also possible to use FT232H SPI bus
 * with different SPI slave devices attached (e.g. SPI-NOR flash chips,
 * spidev, etc.). Example below shows how to add a bus with two SPI slave
 * devices for your USB PID:
 *
 * static struct spi_board_info ftdi_spi_bus_info[] = {
 *	{
 *	.modalias	= "w25q32",
 *	.mode		= SPI_MODE_0,
 *	.max_speed_hz	= 60000000,
 *	.bus_num	= 0,
 *	.chip_select	= 0, // TCK/SK at ADBUS0
 *	},
 *	{
 *	.modalias	= "spidev",
 *	.mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
 *	.max_speed_hz	= 30000000,
 *	.bus_num	= 0,
 *	.chip_select	= 5, // GPIOH0 at ACBUS0
 *	},
 * };
 *
 * static const struct mpsse_spi_platform_data ftdi_spi_bus_plat_data = {
 *	.ops		= &ft232h_intf_ops,
 *	.spi_info	= ftdi_spi_bus_info,
 *	.spi_info_len	= ARRAY_SIZE(ftdi_spi_bus_info),
 * };
 *
 * static const struct ft232h_intf_info ftdi_spi_bus_intf_info = {
 *	.probe  = ft232h_intf_spi_probe,
 *	.remove  = ft232h_intf_spi_remove,
 *	.plat_data  = &ftdi_spi_bus_plat_data,
 * };
 * { USB_DEVICE(FTDI_VID, YOUR_PID),
 *	.driver_info = (kernel_ulong_t)&ftdi_spi_bus_intf_info },
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/gpio.h> //16ton
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/usb/ch9.h>
#include <linux/usb.h>
#include <linux/usb/ft232h-intf.h>
//16ton
#include <linux/property.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/kobject.h>
#include <linux/kdev_t.h>

#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>

#define IRQPIN 3
#define POLL_PERIOD_MS 10;

int usb_wait_msec = 0;
module_param(usb_wait_msec, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(usb_wait_msec, "Wait after USB transfer in msec");

int irqpoll;
module_param(irqpoll, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(irqpoll, "enabled poll based irq gpio pin AD6");

int bind232h;
module_param(bind232h, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(bind232h, "bind to 232h 16ton");

struct ft232h_intf_priv {
    char *name;
	struct usb_interface	*intf;
	struct usb_device	*udev;
	struct mutex		io_mutex; /* sync I/O with disconnect */
	struct mutex		ops_mutex;
	int			bitbang_enabled;
	int			id;
	int			index;
	u8			bulk_in;
	u8			bulk_out;
	size_t			bulk_in_sz;
	void			*bulk_in_buf;

	const struct usb_device_id	*usb_dev_id;
	struct ft232h_intf_info		*info;
	struct platform_device		*fifo_pdev;
	struct platform_device		*spi_pdev;
	struct gpiod_lookup_table	*lookup_fifo;
	struct gpiod_lookup_table	*lookup_cs;
	struct gpiod_lookup_table	*lookup_irq;
   

	struct gpio_chip	cbus_gpio;
	const char		*cbus_gpio_names[4];
	u8			cbus_pin_offsets[4];
	u8			cbus_mask;
	u8			pinbuf[4];

	struct gpio_chip	mpsse_gpio;

    struct gpio_desc *ce_gpio;
	struct gpio_desc *interrupt_gpio;
	char *interrupt_name;
	int old_value;
	u8			gpiol_mask;
	u8			gpioh_mask;
	u8			gpiol_dir;
	u8			gpioh_dir;
	u8			tx_buf[4];
	unsigned int offset;
     bool    hwirq;
     int                      gpio_irq_map[4]; // GPIO to IRQ map (gpio_num elements)

     struct irq_chip   irq;                                // chip descriptor for IRQs
     int               num;
     uint8_t           irq_num;                            // number of pins with IRQs
     int               irq_base;                           // base IRQ allocated
     const struct cpumask *aff_mask;
     int               irq_types    [5]; // IRQ types (irq_num elements)
     bool              irq_enabled  [5]; // IRQ enabled flag (irq_num elements)
     int               irq_gpio_map [5]; // IRQ to GPIO pin map (irq_num elements)
     int               irq_hw;                             // IRQ for GPIO with hardware IRQ (default -1)
     int irq_poll_interval;
    struct work_struct irq_work;

	int		ftmodel;
	int		numgpio;
	u8		eeprom[FTDI_MAX_EEPROM_SIZE];
};


/* Device info struct used for device specific init. */
struct ft232h_intf_info {
	unsigned int use_cbus_gpio_ctrl;
	unsigned int use_mpsse_gpio_ctrl;
	int (*probe)(struct usb_interface *intf, const void *plat_data);
	int (*remove)(struct usb_interface *intf);
	const void *plat_data; /* optional, passed to probe() */
	int			ftmodel;
	int 		numgpio;
};

struct ft232h_intf_device {
	struct ft232h_intf_priv irq_chip;

};

dev_t dev =0;
//static struct class *dev_class;
struct kobject *kobj_ref;



unsigned int gpio_no = 3;

int irqon;

int irqt = 2;

int poll_interval;

static DEFINE_IDA(ftdi_devid_ida);

static int ftdi_read_eeprom(struct ft232h_intf_priv *priv);

static int ftdi_mpsse_gpio_to_irq(struct gpio_chip *chip, unsigned offset);

static void usb_gpio_irq_enable(struct irq_data *irqd);

static void usb_gpio_irq_disable(struct irq_data *irqd);

static int usbirq_irq_set_type(struct irq_data *irqd, unsigned type);

static uint poll_period = POLL_PERIOD_MS;       // module parameter poll period

unsigned int GPIO_irqNumber;

static ssize_t eeprom_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
 //  struct ft232h_intf_priv *priv = container_of(*intf, struct ft232h_intf_priv,
 //                                     mpsse_gpio);
//	struct ft232h_intf_priv *priv = dev_get_drvdata(dev);
//    ftdi_read_eeprom(priv);
//    return sysfs_emit(buf, "%hh \n", priv->eeprom);
    return 0;
}


static ssize_t eeprom_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *valbuf, size_t count)
{
        return count;
}
struct kobj_attribute eeprom = __ATTR(eeprom, 0660, eeprom_show, eeprom_store);
//static DEVICE_ATTR_RW(eeprom);


static int create_sysfs_attrs(struct usb_interface *intf)
{
	int retval = 0;

    kobj_ref = kobject_create_and_add("mpsse",NULL); 
    

//			retval = device_create_file(&intf->dev,
//						    &dev_attr_eeprom);

            retval = sysfs_create_file(kobj_ref,&eeprom.attr);
						    
	return retval;
}

static void remove_sysfs_attrs(struct usb_interface *intf)
{

//			device_remove_file(&intf->dev, &dev_attr_eeprom);
            sysfs_remove_file(kobj_ref,&eeprom.attr);
}

/* Use baudrate calculation borrowed from libftdi */
static int ftdi_to_clkbits(int baudrate, unsigned int clk, int clk_div,
			   unsigned long *encoded_divisor)
{
	static const char frac_code[8] = { 0, 3, 2, 4, 1, 5, 6, 7 };
	int best_baud = 0;
	int div, best_div;

	if (baudrate >= clk / clk_div) {
		*encoded_divisor = 0;
		best_baud = clk / clk_div;
	} else if (baudrate >= clk / (clk_div + clk_div / 2)) {
		*encoded_divisor = 1;
		best_baud = clk / (clk_div + clk_div / 2);
	} else if (baudrate >= clk / (2 * clk_div)) {
		*encoded_divisor = 2;
		best_baud = clk / (2 * clk_div);
	} else {
		/*
		 * Divide by 16 to have 3 fractional bits and
		 * one bit for rounding
		 */
		div = clk * 16 / clk_div / baudrate;
		if (div & 1)	/* Decide if to round up or down */
			best_div = div / 2 + 1;
		else
			best_div = div / 2;
		if (best_div > 0x20000)
			best_div = 0x1ffff;
		best_baud = clk * 16 / clk_div / best_div;
		if (best_baud & 1)	/* Decide if to round up or down */
			best_baud = best_baud / 2 + 1;
		else
			best_baud = best_baud / 2;
		*encoded_divisor = (best_div >> 3) |
				   (frac_code[best_div & 0x7] << 14);
	}
	return best_baud;
}

#define H_CLK	120000000
#define C_CLK	48000000
static int ftdi_convert_baudrate(struct ft232h_intf_priv *priv, int baud,
				 u16 *value, u16 *index)
{
	unsigned long encoded_divisor = 0;
	int best_baud = 0;

	if (baud <= 0)
		return -EINVAL;

	/*
	 * On H Devices, use 12000000 baudrate when possible.
	 * We have a 14 bit divisor, a 1 bit divisor switch (10 or 16),
	 * three fractional bits and a 120 MHz clock. Assume AN_120
	 * "Sub-integer divisors between 0 and 2 are not allowed" holds
	 * for DIV/10 CLK too, so /1, /1.5 and /2 can be handled the same
	 */
	if (baud * 10 > H_CLK / 0x3fff) {
		best_baud = ftdi_to_clkbits(baud, H_CLK, 10, &encoded_divisor);
		encoded_divisor |= 0x20000;	/* switch on CLK/10 */
	} else {
		best_baud = ftdi_to_clkbits(baud, C_CLK, 16, &encoded_divisor);
	}

	if (best_baud <= 0) {
		pr_err("Invalid baudrate: %d\n", best_baud);
		return -EINVAL;
	}

	/* Check within tolerance (about 5%) */
	if ((best_baud * 2 < baud) ||
	    (best_baud < baud
		? (best_baud * 21 < baud * 20)
		: (baud * 21 < best_baud * 20))) {
		pr_err("Unsupported baudrate.\n");
		return -EINVAL;
	}

	/* Split into "value" and "index" values */
	*value = (u16)(encoded_divisor & 0xffff);
	*index = (u16)(((encoded_divisor >> 8) & 0xff00) | priv->index);

	dev_dbg(&priv->intf->dev, "best baud %d, v/i: %d, %d\n",
		best_baud, *value, *index);
	return best_baud;
}

/*
 * ftdi_ctrl_xfer - FTDI control endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for control transfer
 *
 * Return:
 * Return: If successful, the number of bytes transferred. Otherwise,
 * a negative error number.
 */
static int ftdi_ctrl_xfer(struct usb_interface *intf, struct ctrl_desc *desc)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (!desc->data && desc->size)
		desc->data = priv->bulk_in_buf;

	if (desc->dir_out)
		pipe = usb_sndctrlpipe(udev, 0);
	else
		pipe = usb_rcvctrlpipe(udev, 0);

	ret = usb_control_msg(udev, pipe, desc->request, desc->requesttype,
			      desc->value, desc->index, desc->data, desc->size,
			      desc->timeout);
	if (ret < 0)
		dev_dbg(&udev->dev, "ctrl msg failed: %d\n", ret);
exit:
	mutex_unlock(&priv->io_mutex);
	return ret;
}

/*
 * ftdi_bulk_xfer - FTDI bulk endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for bulk-in or bulk-out transfer
 *
 * Return:
 * If successful, 0. Otherwise a negative error number. The number of
 * actual bytes transferred will be stored in the @desc->act_len field
 * of the descriptor struct.
 */
static int ftdi_bulk_xfer(struct usb_interface *intf, struct bulk_desc *desc)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (desc->dir_out)
		pipe = usb_sndbulkpipe(udev, priv->bulk_out);
	else
		pipe = usb_rcvbulkpipe(udev, priv->bulk_in);

	ret = usb_bulk_msg(udev, pipe, desc->data, desc->len,
			   &desc->act_len, desc->timeout);
	if (ret)
		dev_dbg(&udev->dev, "bulk msg failed: %d\n", ret);

exit:
	mutex_unlock(&priv->io_mutex);
	if (usb_wait_msec > 0) {
		usleep_range(usb_wait_msec * 1000, usb_wait_msec * 1000 + 1000);
	}
	return ret;
}

/*
 * ftdi_set_baudrate - set the device baud rate
 * @intf: USB interface pointer
 * @baudrate: baud rate value to set
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_baudrate(struct usb_interface *intf, int baudrate)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	u16 index, value;
	int ret;

	if (priv->bitbang_enabled)
		baudrate *= 4;

	ret = ftdi_convert_baudrate(priv, baudrate, &value, &index);
	if (ret < 0)
		return ret;

	desc.dir_out = true;
	desc.request = FTDI_SIO_SET_BAUDRATE_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.value = value;
	desc.index = index;
	desc.data = NULL;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	if (ret < 0) {
		dev_dbg(&intf->dev, "failed to set baudrate: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_set_clock(struct usb_interface *intf, int clock_freq_hz)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	uint8_t *buf = priv->tx_buf;
	uint32_t value = 0;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	switch (priv->usb_dev_id->idProduct) {
	case 0x6001: /* FT232 */
		if (clock_freq_hz >= FTDI_CLK_6MHZ) {
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		}
		break;

	case 0x6010: /* FT2232 */
	case 0x6011: /* FT4232 */
	case 0x6041: /* FT4233 */
	case 0x6014: /* FT232H */
	case 0x0146: /* GW16146 */
		desc.len = 1;
		if (clock_freq_hz <= (FTDI_CLK_30MHZ/65535)) {
			buf[0] = EN_DIV_5;
			ret = ftdi_bulk_xfer(intf, &desc);
			if (ret) {
				return ret;
			}
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		}
		else {
			buf[0] = DIS_DIV_5;
			ret = ftdi_bulk_xfer(intf, &desc);
			if (ret) {
				return ret;
			}
			value = (FTDI_CLK_30MHZ/clock_freq_hz) - 1;
		}

		break;
	}

	buf[0] = TCK_DIVISOR;
	buf[1] = (uint8_t)(value & 0xff);
	buf[2] = (uint8_t)(value >> 8);
	desc.act_len = 0;
	desc.len = 3;
	ret = ftdi_bulk_xfer(intf, &desc);

	return ret;
}

/*
 * ftdi_set_latency - set the device latency (Bulk-In interval)
 * @intf: USB interface pointer
 * @latency_msec: latency value to set, 1-255
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_latency(struct usb_interface *intf, int latency_msec)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.request = FTDI_SIO_SET_LATENCY_TIMER_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.value = latency_msec;
	desc.index = priv->index;
	desc.data = NULL;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	if (ret < 0) {
		dev_dbg(&intf->dev, "failed to set latency: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * ftdi_read_data - read from FTDI bulk-in endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to read
 *
 * The two modem status bytes transferred in every read will
 * be removed and will not appear in the data buffer.
 *
 * Return:
 * If successful, the number of data bytes received (can be 0).
 * Otherwise, a negative error number.
 */
static int ftdi_read_data(struct usb_interface *intf, void *buf, size_t len)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = false;
	desc.data = priv->bulk_in_buf;
	/* Device sends 2 additional status bytes, read at least len + 2 */
	desc.len = min_t(size_t, len + 2, priv->bulk_in_sz);
	desc.timeout = FTDI_USB_READ_TIMEOUT;

	ret = ftdi_bulk_xfer(intf, &desc);
	if (ret)
		return ret;

	/* Only status bytes and no data? */
	if (desc.act_len <= 2)
		return 0;

	/* Skip first two status bytes */
	ret = desc.act_len - 2;
	if (ret > len)
		ret = len;
	memcpy(buf, desc.data + 2, ret);
	return ret;
}

/*
 * ftdi_write_data - write to FTDI bulk-out endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to send
 *
 * Return:
 * If successful, the number of bytes transferred. Otherwise a negative
 * error number.
 */
static int ftdi_write_data(struct usb_interface *intf,
			   const char *buf, size_t len)
{
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.len = len;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	ret = ftdi_bulk_xfer(intf, &desc);
	if (ret < 0)
		return ret;

	return desc.act_len;
}

/*
 * ftdi_set_bitmode - configure bitbang mode
 * @intf: USB interface pointer
 * @bitmask: line configuration bitmask
 * @mode: bitbang mode to set
 *
 * Return:
 * If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_bitmode(struct usb_interface *intf, unsigned char bitmask,
			    unsigned char mode)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.data = NULL;
	desc.request = FTDI_SIO_SET_BITMODE_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.index = 1;
	desc.value = (mode << 8) | bitmask;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	if (ret < 0)
		return ret;

	switch (mode) {
	case BITMODE_BITBANG:
	case BITMODE_CBUS:
	case BITMODE_SYNCBB:
	case BITMODE_SYNCFF:
		priv->bitbang_enabled = 1;
		break;
	case BITMODE_MPSSE:
	case BITMODE_RESET:
	default:
		priv->bitbang_enabled = 0;
		break;
	}

	return 0;
}

/*
 * ftdi_disable_bitbang - disable bitbang mode
 * @intf: USB interface pointer
 *
 * Return:
 * If successful, 0. Otherwise a negative error number.
 */
static int ftdi_disable_bitbang(struct usb_interface *intf)
{
	int ret;

	ret = ftdi_set_bitmode(intf, 0, BITMODE_RESET);
	if (ret < 0) {
		dev_dbg(&intf->dev, "disable bitbang failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_read_eeprom(struct ft232h_intf_priv *priv)
{
	struct ctrl_desc desc;
	unsigned int i;
	int ret;

	desc.dir_out = false;
	desc.request = FTDI_SIO_READ_EEPROM_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN;
	desc.value = 0;
	desc.size = 2;
	desc.timeout = USB_CTRL_GET_TIMEOUT;

	for (i = 0; i < FTDI_MAX_EEPROM_SIZE / 2; i++) {
		desc.index = i;
		desc.data = &priv->eeprom[i * 2];

		ret = ftdi_ctrl_xfer(priv->intf, &desc);
		if (ret < 0) {
			dev_dbg(&priv->intf->dev, "EEPROM read failed: %d\n",
				ret);
			return ret;
		}
	}


	print_hex_dump(KERN_DEBUG, "EEPROM: ", DUMP_PREFIX_OFFSET, 16, 1, priv->eeprom, sizeof(priv->eeprom), 1);

	return 0;
}

/*
 * ACBUS GPIO functions
 */
static const char *ftdi_acbus_names[5] = {
	"ACBUS5", "ACBUS6", NULL, "ACBUS8", "ACBUS9"
};

static int ftdi_cbus_gpio_read_pins(struct ft232h_intf_priv *priv,
				    unsigned char *pins)
{
	struct gpio_chip *chip = &priv->cbus_gpio;
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = false;
	desc.request = FTDI_SIO_READ_PINS_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN;
	desc.value = 0;
	desc.index = 1;
	desc.data = &priv->pinbuf[0];
	desc.size = 1;
	desc.timeout = USB_CTRL_GET_TIMEOUT;

	ret = ftdi_ctrl_xfer(priv->intf, &desc);
	if (ret < 0) {
		dev_dbg(chip->parent, "failed to get pin values: %d\n", ret);
		return ret;
	}

	*pins = priv->pinbuf[0];
	return 0;
}

static inline void ftdi_cbus_init_gpio_data(struct ft232h_intf_priv *priv,
					    int gpio_num, int cbus_num)
{
	switch (cbus_num) {
	case 5:
	case 6:
		priv->cbus_pin_offsets[gpio_num] = cbus_num - 5;
		break;
	case 8:
	case 9:
		priv->cbus_pin_offsets[gpio_num] = cbus_num - 6;
		break;
	default:
		return;
	}

	priv->cbus_gpio_names[gpio_num] = ftdi_acbus_names[cbus_num - 5];
}

static int ftdi_cbus_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	unsigned int offs;
	int ret;
	u8 pins = 0;

	ret = ftdi_cbus_gpio_read_pins(priv, &pins);
	if (ret)
		return ret;

	offs = priv->cbus_pin_offsets[offset];

	return !!(pins & BIT(offs));
}

static void ftdi_cbus_gpio_set(struct gpio_chip *chip,
			       unsigned int offset, int value)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	unsigned int offs;
	int ret;

	offs = priv->cbus_pin_offsets[offset];

	if (value)
		priv->cbus_mask |= BIT(offs);
	else
		priv->cbus_mask &= ~BIT(offs);

	ret = ftdi_set_bitmode(priv->intf, priv->cbus_mask, BITMODE_CBUS);
	if (ret < 0)
		dev_dbg(chip->parent, "setting pin value failed: %d\n", ret);
}

static int ftdi_cbus_gpio_direction_input(struct gpio_chip *chip,
					  unsigned int offset)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	unsigned int offs;

	offs = priv->cbus_pin_offsets[offset];
	/* Direction bits are in the upper nibble */
	priv->cbus_mask &= ~(BIT(offs) << 4);

	return ftdi_set_bitmode(priv->intf, priv->cbus_mask, BITMODE_CBUS);
}

static int ftdi_cbus_gpio_direction_output(struct gpio_chip *chip,
					   unsigned int offset, int value)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	unsigned int offs;

	offs = priv->cbus_pin_offsets[offset];
	priv->cbus_mask |= BIT(offs) << 4;

	if (value)
		priv->cbus_mask |= BIT(offs);
	else
		priv->cbus_mask &= ~BIT(offs);

	return ftdi_set_bitmode(priv->intf, priv->cbus_mask, BITMODE_CBUS);
}

static int ft232h_intf_add_cbus_gpio(struct ft232h_intf_priv *priv)
{
	struct device *dev = &priv->intf->dev;
	char **names, *label;
	int ngpio = 0;
	int i, ret;
	u8 val;
 
	ret = ftdi_read_eeprom(priv);
	if (ret < 0)
		return ret;

	/* Check if I/O mode is enabled for supported pins 5, 6, 8, 9 */
	for (i = 5; i < 10; i++) {
		val = priv->eeprom[0x18 + i / 2] >> (i % 2 ? 4 : 0);
		if ((val & 0x0f) == FTDI_232H_CBUS_IOMODE) {
			dev_dbg(dev, "gpio-%d @ ACBUS%d\n",
				priv->cbus_gpio.ngpio, i);
			priv->cbus_gpio.ngpio++;
			ftdi_cbus_init_gpio_data(priv, ngpio++, i);
		}
	}

	if (!priv->cbus_gpio.ngpio) {
		dev_warn(dev, "I/O mode disabled in EEPROM\n");
		return -ENODEV;
	}

	label = devm_kasprintf(dev, GFP_KERNEL, "ftdi-cbus-gpio.%d", priv->id);
	if (!label)
		return -ENOMEM;

	priv->cbus_gpio.label = label;
	priv->cbus_gpio.parent = dev;
	priv->cbus_gpio.owner = THIS_MODULE;
	priv->cbus_gpio.base = -1;
	priv->cbus_gpio.can_sleep = true;
	priv->cbus_gpio.set = ftdi_cbus_gpio_set;
	priv->cbus_gpio.get = ftdi_cbus_gpio_get;
	priv->cbus_gpio.direction_input = ftdi_cbus_gpio_direction_input;
	priv->cbus_gpio.direction_output = ftdi_cbus_gpio_direction_output;

	names = devm_kcalloc(dev, priv->cbus_gpio.ngpio, sizeof(char *),
			     GFP_KERNEL);
	if (!names)
		return -ENOMEM;

	for (i = 0; i < priv->cbus_gpio.ngpio; i++) {
		if (!priv->cbus_gpio_names[i])
			continue;
		names[i] = devm_kasprintf(dev, GFP_KERNEL, "cbus.%d-%s",
					  priv->id, priv->cbus_gpio_names[i]);
		if (!names[i])
			return -ENOMEM;
	}

	priv->cbus_gpio.names = (const char *const *)names;

	ret = devm_gpiochip_add_data(dev, &priv->cbus_gpio, priv);
	if (ret) {
		dev_warn(dev, "failed to add CBUS gpiochip: %d\n", ret);
		return ret;
	}

	dev_info(dev, "using %d CBUS pins\n", priv->cbus_gpio.ngpio);
	return 0;
}

/*
 * MPSSE CS and GPIO-L/-H support
 */
#define SET_BITS_LOW	0x80
#define GET_BITS_LOW	0x81
#define SET_BITS_HIGH	0x82
#define GET_BITS_HIGH	0x83

static int ftdi_mpsse_get_port_pins(struct ft232h_intf_priv *priv, bool low)
{
	struct device *dev = &priv->intf->dev;
//  int ret, tout = 10;
	int ret, tout = 25;
	u8 rxbuf[4];

	if (low)
		priv->tx_buf[0] = GET_BITS_LOW;
	else
		priv->tx_buf[0] = GET_BITS_HIGH;

	ret = ftdi_write_data(priv->intf, priv->tx_buf, 1);
	if (ret < 0) {
		dev_dbg_ratelimited(dev, "Writing port pins cmd failed: %d\n",
				    ret);
		return ret;
	}

	rxbuf[0] = 0;
	do {
		usleep_range(5000, 5200);
		ret = ftdi_read_data(priv->intf, rxbuf, 1);
		tout--;
		if (!tout) {
			dev_err(dev, "Timeout when getting port pins\n");
			return -ETIMEDOUT;
		}
	} while (ret == 0);

	if (ret < 0)
		return ret;

	if (ret != 1)
		return -EINVAL;

	if (low)
		priv->gpiol_mask = rxbuf[0];
	else
		priv->gpioh_mask = rxbuf[0];

	return 0;
}

static int ftdi_mpsse_set_port_pins(struct ft232h_intf_priv *priv, bool low)
{
	struct device *dev = &priv->intf->dev;
	int ret;

	if (low) {
		priv->tx_buf[0] = SET_BITS_LOW;
		priv->tx_buf[1] = priv->gpiol_mask;
		priv->tx_buf[2] = priv->gpiol_dir;
	} else {
		priv->tx_buf[0] = SET_BITS_HIGH;
		priv->tx_buf[1] = priv->gpioh_mask;
		priv->tx_buf[2] = priv->gpioh_dir;
	}

	ret = ftdi_write_data(priv->intf, priv->tx_buf, 3);
	if (ret < 0) {
		dev_dbg_ratelimited(dev, "Failed to set GPIO pins: %d\n",
				    ret);
		return ret;
	}

	return 0;
}

static int ftdi_mpsse_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	int ret, val;
	bool low;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d\n", __func__, offset);

	low = offset < 5;

	mutex_lock(&priv->ops_mutex);

	ret = ftdi_mpsse_get_port_pins(priv, low);
	if (ret < 0) {
		mutex_unlock(&priv->ops_mutex);
		return ret;
	}

	if (low)
		val = priv->gpiol_mask & (BIT(offset) << 3);
	else
		val = priv->gpioh_mask & BIT(offset - 5);

	mutex_unlock(&priv->ops_mutex);

	return !!val;
}

static void ftdi_mpsse_gpio_set(struct gpio_chip *chip, unsigned int offset,
				int value)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	bool low;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		if (value)
			priv->gpiol_mask |= (BIT(offset) << 3);
		else
			priv->gpiol_mask &= ~(BIT(offset) << 3);
	} else {
		low = false;
		if (value)
			priv->gpioh_mask |= BIT(offset - 5);
		else
			priv->gpioh_mask &= ~BIT(offset - 5);
	}

	ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);
}

static int ftdi_mpsse_gpio_direction_input(struct gpio_chip *chip,
					   unsigned int offset)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	bool low;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d\n", __func__, offset);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		priv->gpiol_dir &= ~(BIT(offset) << 3);
	} else {
		low = false;
		priv->gpioh_dir &= ~BIT(offset - 5);
	}

	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static int ftdi_mpsse_gpio_direction_output(struct gpio_chip *chip,
					    unsigned int offset, int value)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	bool low;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		priv->gpiol_dir |= BIT(offset) << 3;

		if (value)
			priv->gpiol_mask |= BIT(offset) << 3;
		else
			priv->gpiol_mask &= ~(BIT(offset) << 3);
	} else {
		low = false;
		priv->gpioh_dir |= BIT(offset - 5);

		if (value)
			priv->gpioh_mask |= BIT(offset - 5);
		else
			priv->gpioh_mask &= ~BIT(offset - 5);
	}

	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static int ftdi_mpsse_init_pins(struct usb_interface *intf, bool low,
				u8 bits, u8 direction)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&priv->ops_mutex);

	if (low) {
		priv->gpiol_mask = bits;
		priv->gpiol_dir = direction;
	} else {
		priv->gpioh_mask = bits;
		priv->gpioh_dir = direction;
	}
	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static int ftdi_mpsse_cfg_bus_pins(struct usb_interface *intf,
				   u8 dir_bits, u8 value_bits)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&priv->ops_mutex);

	priv->gpiol_dir &= ~7;
	priv->gpiol_dir |= (dir_bits & 7);

	priv->gpiol_mask &= ~7;
	priv->gpiol_mask |= (value_bits & 7);

	ret = ftdi_mpsse_set_port_pins(priv, true);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static void ftdix_read_gpios(struct work_struct *work)
{

	struct ft232h_intf_priv *priv = container_of(work,
	                                         struct ft232h_intf_priv,
	                                         irq_work);

    uint8_t oldio;
    uint8_t new_value;
    unsigned long flags;
    int type;
//	oldio = priv->old_value;
	oldio = 1;

          priv->aff_mask = irq_get_affinity_mask(IRQPIN);
             if (!priv->aff_mask) 
          {
              printk(KERN_ALERT "irq_to_desc failed\n");
          }

	printk(KERN_INFO "Saved oldio pin state on AD6 %d\n", oldio);


	printk(KERN_INFO "read gpio after init state AD6 %d\n", ftdi_mpsse_gpio_get(&priv->mpsse_gpio, 3));
	printk(KERN_INFO  "start read gpios\n");

	if (poll_period)
	{
	priv->irq_poll_interval = poll_period;
    }

loop:
        
    type = priv->irq_types[GPIO_irqNumber];
	new_value = ftdi_mpsse_gpio_get(&priv->mpsse_gpio, 3);

//    if (irqon == 1) {
	if (oldio != new_value) {
	printk(KERN_INFO "irq type  %d\n", irqt);
	         if (irqt == 3) {
                if (oldio != new_value) {
                	dev_dbg(&priv->udev->dev, "issue irq on %d\n", IRQPIN);
			        local_irq_save(flags);
                    generic_handle_irq(GPIO_irqNumber);
                    local_irq_disable();
                    local_irq_restore(flags);
                }
             }
             if (irqt == 2) {
                if (oldio > new_value) {
                	dev_dbg(&priv->udev->dev, "issue irq on %d\n", IRQPIN);
			        local_irq_save(flags);
                    generic_handle_irq(GPIO_irqNumber);
                    local_irq_disable();
                    local_irq_restore(flags);
                }
             }
            if (irqt == 1) { 
                if (new_value > oldio) {
        			dev_dbg(&priv->udev->dev, "issue irq on %d\n", IRQPIN);
			        local_irq_save(flags);
                    generic_handle_irq(GPIO_irqNumber);
                    local_irq_disable();
                    local_irq_restore(flags);
                }
             }
            oldio = new_value;
	}
//    }

    	if (priv->irq_poll_interval < 0) {
		    return;
        }

	usleep_range(priv->irq_poll_interval, priv->irq_poll_interval + 20);

    if (irqon == 1) {
	    goto loop;
    }
    
    priv->irq_enabled[3] = false;
    printk(KERN_INFO  "Leaving gpio irq poll loop \n");

}

const char *gpio_names[] = { "CS", "ce", "reset", "irq", "GPIOL3" };
const char *gpio_names2[] = { "CS", "ce", "reset", "irq", "GPIOL3", "GPIOH0", "GPIOH1", "GPIOH2", "GPIOH3", "GPIOH4", "GPIOH5", "GPIOH6", "GPIOH7" };

static int ft232h_intf_add_mpsse_gpio(struct ft232h_intf_priv *priv)
{
	struct device *dev = &priv->intf->dev;
	struct gpio_irq_chip *girq;

	int rc;
	char **names, *label;
	int MPSSE_GPIOS;
	int ret;
//    int status;
    
	ret = ftdi_read_eeprom(priv);
	if (ret < 0)
		return ret;

	MPSSE_GPIOS = ft232h_intf_get_numgpio(priv->intf);

	label = devm_kasprintf(dev, GFP_KERNEL, "ftdi-mpsse-gpio.%d", priv->id);
	if (!label)
		return -ENOMEM;

	priv->mpsse_gpio.label = label;
	priv->mpsse_gpio.parent = dev;
	priv->mpsse_gpio.owner = THIS_MODULE;
//	priv->mpsse_gpio.request= NULL;      //16ton?
//	priv->mpsse_gpio.free   = NULL;          //16ton?
	priv->mpsse_gpio.base = -1;
	priv->mpsse_gpio.ngpio = MPSSE_GPIOS;
	priv->mpsse_gpio.can_sleep = true;
	priv->mpsse_gpio.set = ftdi_mpsse_gpio_set;
	priv->mpsse_gpio.get = ftdi_mpsse_gpio_get;
	priv->mpsse_gpio.direction_input = ftdi_mpsse_gpio_direction_input;
 	priv->mpsse_gpio.direction_output = ftdi_mpsse_gpio_direction_output;
	if (irqpoll) {
	    priv->mpsse_gpio.to_irq = ftdi_mpsse_gpio_to_irq;
    	priv->irq.name = "usbgpio-irq";
    	priv->irq.irq_set_type = usbirq_irq_set_type;
        priv->irq.irq_enable = usb_gpio_irq_enable;
        priv->irq.irq_disable = usb_gpio_irq_disable;
	}
	
	names = devm_kcalloc(dev, priv->mpsse_gpio.ngpio, sizeof(char *),
			     GFP_KERNEL);
	if (!names)
		return -ENOMEM;

	if (MPSSE_GPIOS < 6) {
	priv->mpsse_gpio.names = gpio_names;
	}
	if (MPSSE_GPIOS > 12) {
	priv->mpsse_gpio.names = gpio_names2;
	}

    if (irqpoll) {
    	girq = &priv->mpsse_gpio.irq;
    	girq->chip = &priv->irq;
       	girq->parent_handler = NULL;
    	girq->num_parents = 0;
    	girq->parents = NULL;
    	girq->default_type = IRQ_TYPE_NONE;
    	girq->handler = handle_simple_irq;
    
    	
	    rc = irq_alloc_desc(0);
	    if (rc < 0) {
	    	dev_err(dev, "Cannot allocate an IRQ desc");
	    	return rc;
	    }
	    priv->irq_base = rc;
	}
	
    if (irqpoll) {
    irq_clear_status_flags(priv->irq_base + 3, IRQ_NOREQUEST | IRQ_NOPROBE);
    }
    
    if (irqpoll) {
    irq_set_status_flags(priv->irq_base + 3, IRQ_TYPE_EDGE_FALLING);
    }
    
	ret = ftdi_set_bitmode(priv->intf, 0x00, BITMODE_MPSSE);
	if (ret < 0) {
		dev_err(dev, "Failed to set MPSSE mode\n");
		return ret;
	}

	ret = devm_gpiochip_add_data(dev, &priv->mpsse_gpio, priv);
	if (ret < 0) {
		dev_err(dev, "Failed to add MPSSE GPIO chip: %d\n", ret);
		return ret;
	}
	
    if (irqpoll) {
    
    priv->interrupt_gpio = gpiochip_request_own_desc(&priv->mpsse_gpio, 
                                                3,
                                                "irq",
                                                GPIOD_IN,
                                                GPIO_ACTIVE_LOW);



    priv->interrupt_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);

    ret = devm_gpio_request_one(dev,  priv->mpsse_gpio.base + 3, GPIOD_IN, "irq");
		if (ret) {
			if (ret == -EPROBE_DEFER)
				pr_info("failed request one  = %s\n", priv->interrupt_name);
		}

    irqon = 1;
    priv->old_value = ftdi_mpsse_gpio_get(&priv->mpsse_gpio, 3);
       
    printk(KERN_INFO  "end of add gpio mpsse\n");
       
        ftdi_mpsse_gpio_to_irq(&priv->mpsse_gpio, 3);
        INIT_WORK(&priv->irq_work, ftdix_read_gpios);
	    schedule_work(&priv->irq_work);
        
    }
    
	return 0;
}

static void usb_gpio_irq_enable(struct irq_data *irqd)
{
    struct ft232h_intf_priv *priv = irq_data_get_irq_chip_data(irqd);

	/* Is that needed? */
	;

    if (priv->irq_enabled[3]) {
        printk(KERN_INFO  "IRQ already enabled returning\n");        
		return;
    }
    
    if (irqon == 0) {
    irqon = 1;
    INIT_WORK(&priv->irq_work, ftdix_read_gpios);
	schedule_work(&priv->irq_work);
	}
	
    priv->irq_enabled[3] = true;
    printk(KERN_INFO  "IRQ enabled\n");
   
}

static void usb_gpio_irq_disable(struct irq_data *irqd)
{
    struct ft232h_intf_priv *priv = irq_data_get_irq_chip_data(irqd);

	if (irqon == 1) {            
		irqon = 0;
		cancel_work_sync(&priv->irq_work);
	}
    
    priv->irq_enabled[3] = false;
    usleep_range(100, 200);
    printk(KERN_INFO  "IRQ disabled\n");

}


static int usbirq_irq_set_type(struct irq_data *irqd, unsigned type)
{
    struct ft232h_intf_priv *dev = irq_data_get_irq_chip_data(irqd);
    int irq;
    irqt = type;
    irq = irqd->irq - dev->irq_base;
    printk(KERN_INFO "Set  irq  %d\n", irq);
    printk(KERN_INFO "Set type irq  %d\n", type);
    dev->irq_types[irq] = type;

    printk(KERN_INFO "Set type, type  %d\n", dev->irq_types[irq]);
    
    return 0;
    
}

static void ftdi_lock(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);

	mutex_lock(&priv->ops_mutex);
}

static void ftdi_unlock(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);

	mutex_unlock(&priv->ops_mutex);
}

static const struct ft232h_intf_ops ft232h_intf_ops = {
	.ctrl_xfer = ftdi_ctrl_xfer,
	.bulk_xfer = ftdi_bulk_xfer,
	.read_data = ftdi_read_data,
	.write_data = ftdi_write_data,
	.lock = ftdi_lock,
	.unlock = ftdi_unlock,
	.set_bitmode = ftdi_set_bitmode,
	.set_baudrate = ftdi_set_baudrate,
	.disable_bitbang = ftdi_disable_bitbang,
	.init_pins = ftdi_mpsse_init_pins,
	.cfg_bus_pins = ftdi_mpsse_cfg_bus_pins,
	.set_clock = ftdi_set_clock,
	.set_latency = ftdi_set_latency,
};

static int
ftdi_mpsse_gpio_to_irq(struct gpio_chip *chip,
                  unsigned offset)
{
   struct ft232h_intf_priv *priv = container_of(chip, struct ft232h_intf_priv,
                                      mpsse_gpio);
   GPIO_irqNumber = irq_create_mapping(priv->mpsse_gpio.irq.domain, offset);
   pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);

   return GPIO_irqNumber;
}
/*
 * FPGA config interface: FPP via FT245 FIFO
 */
#define FPP_INTF_DEVNAME	"spi-ftdi-mpsse"

static struct dev_io_desc_data fpga_cfg_fpp_dev_io[2] = {
	{ "nconfig", 0, GPIO_ACTIVE_LOW },
	{ "conf_done", 1, GPIO_ACTIVE_HIGH },
};

static const struct fifo_fpp_mgr_platform_data fpga_cfg_fpp_plat_data = {
	.ops = &ft232h_intf_ops,
	.io_data = fpga_cfg_fpp_dev_io,
	.io_data_len = ARRAY_SIZE(fpga_cfg_fpp_dev_io),
	.nconfig_num = 8,
	.conf_done_num = 9,
};

static int ft232h_intf_fpp_probe(struct usb_interface *intf,
				 const void *plat_data)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	const struct fifo_fpp_mgr_platform_data *pd = plat_data;
	struct device *dev = &intf->dev;
	struct platform_device *pdev;
	struct gpiod_lookup_table *lookup;
	char *cfgdone, *ncfg;    /* 16ton to replace line above*/
	size_t lookup_size;


	dev_dbg(dev, "%s: plat_data %p\n", __func__, pd);
	if (!pd) {
		dev_err(dev, "%s: Missing platform data\n", __func__);
		return -EINVAL;
	}


	lookup_size = sizeof(*lookup) + 3 * sizeof(struct gpiod_lookup);
	lookup = devm_kzalloc(dev, lookup_size, GFP_KERNEL);
	if (!lookup)
		return -ENOMEM;

	lookup->dev_id = devm_kasprintf(dev, GFP_KERNEL, "%s.%d",
					FPP_INTF_DEVNAME, priv->id);
	if (!lookup->dev_id)
		return -ENOMEM;

	ncfg = devm_kasprintf(dev, GFP_KERNEL, "ACBUS%d", pd->nconfig_num);
	if (!ncfg)
		return -ENOMEM;

	cfgdone = devm_kasprintf(dev, GFP_KERNEL, "ACBUS%d", pd->conf_done_num);
	if (!cfgdone)
		return -ENOMEM;


	priv->lookup_fifo = lookup;


	pdev = platform_device_register_data(dev, FPP_INTF_DEVNAME,
					     priv->id, pd, sizeof(*pd));
	if (IS_ERR(pdev)) {
		gpiod_remove_lookup_table(priv->lookup_fifo);
		return PTR_ERR(pdev);
	}

	priv->fifo_pdev = pdev;

	dev_dbg(dev, "%s: fifo pdev %p\n", __func__, pdev);
	return 0;
}

static int ft232h_intf_fpp_remove(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;

	dev_dbg(dev, "%s\n", __func__);
	platform_device_unregister(priv->fifo_pdev);
	gpiod_remove_lookup_table(priv->lookup_fifo);
	return 0;
}

/*
 * FPGA config interface: PS-SPI via MPSSE
 */
#define SPI_INTF_DEVNAME	"spi-ftdi-mpsse"


static struct dev_io_desc_data ftdi_spi_bus_dev_io[] = {
	{ "ce", 1, GPIO_ACTIVE_HIGH },
	{ "csn", 2, GPIO_ACTIVE_LOW },
	{ "irq", 3, GPIO_ACTIVE_LOW },
};

static const struct mpsse_spi_dev_data ftdi_spi_dev_data[] = {
	{
	.magic		= FTDI_MPSSE_IO_DESC_MAGIC,
	.desc		= ftdi_spi_bus_dev_io,
	.desc_len	= ARRAY_SIZE(ftdi_spi_bus_dev_io),
	},
};

static const struct property_entry mcp2515_properties[] = {
	PROPERTY_ENTRY_U32("clock-frequency", 8000000),
//	PROPERTY_ENTRY_U32("xceiver", 1),
//	PROPERTY_ENTRY_U32("gpio-controller", 3),
	{}
};

static const struct property_entry nrf24_properties[] = {
	PROPERTY_ENTRY_U32("interrupts", 3),
	{}
};

static const struct property_entry eeprom_93xx46_properties[] = {
	PROPERTY_ENTRY_U32("spi-max-frequency", 1000000),
	PROPERTY_ENTRY_U32("data-size", 16),
//	PROPERTY_ENTRY_U32("xceiver", 1),
//	PROPERTY_ENTRY_U32("gpio-controller", 3),
	{}
};

static const struct property_entry ili9341_properties[] = {
	PROPERTY_ENTRY_U32("rotate", 270),
	PROPERTY_ENTRY_BOOL("bgr"),
	PROPERTY_ENTRY_U32("fps", 60),
	PROPERTY_ENTRY_U32("speed", 30000000),
	PROPERTY_ENTRY_U32("buswidth", 8),
	PROPERTY_ENTRY_U32("regwidth", 8),
	{}
};

static const struct software_node mcp2515_node = {
	.properties = mcp2515_properties,
};

static struct spi_board_info ftdi_spi_bus_info[] = {
    {
//    .modalias	= "yx240qv29",
//	.modalias	= "ili9341",
//	.modalias	= "mcp2515",
//    .modalias	= "spi-petra",
    .modalias	= "spi-petra",
//	.modalias	= "ili9341",
    .mode		= SPI_MODE_0,
//    .mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
    .max_speed_hz	= 30000000,
//    .max_speed_hz	= 30000000,
    .bus_num	= 0,
    .chip_select	= 4,
// 	.properties	= nrf24_properties,    //changed from properties to swnode i dunno aroun kernel 5.15ish
//    .properties	= mcp2515_properties,
//	.swnode  =  &mcp2515_node,
//	.irq     = 0,
    },
   {
    .modalias	= "nrf24",    //use instead of spidev for spidev no-longer enumerates
//    .modalias	= "w25q32",
//	  .modalias	= "spidev",
    .mode		= SPI_MODE_0,
//    .mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
    .max_speed_hz	= 4000000,
    .bus_num	= 0,
    .chip_select	= 0, // GPIOH0 at ACBUS0
    .platform_data	= ftdi_spi_dev_data,
    },
};

static const struct mpsse_spi_platform_data ftdi_spi_bus_plat_data = {
    .ops		= &ft232h_intf_ops,
    .spi_info	= ftdi_spi_bus_info,
    .spi_info_len	= ARRAY_SIZE(ftdi_spi_bus_info),
};

static struct dev_io_desc_data fpga_cfg_spi_dev_io[3] = {
	{ "confd", 1, GPIO_ACTIVE_HIGH },
	{ "nstat", 2, GPIO_ACTIVE_LOW },
	{ "nconfig", 3, GPIO_ACTIVE_LOW },
};

static const struct mpsse_spi_dev_data fpga_spi_dev_data[] = {
	{
	.magic		= FTDI_MPSSE_IO_DESC_MAGIC,
	.desc		= fpga_cfg_fpp_dev_io,
	.desc_len	= ARRAY_SIZE(fpga_cfg_spi_dev_io),
	},
};

static struct spi_board_info fpga_cfg_spi_info[] = {
	{
	.modalias	= "fpga-passive-serial",
	.mode		= SPI_MODE_0 | SPI_LSB_FIRST,
	.max_speed_hz	= 30000000,
	.bus_num	= 0,
	.chip_select	= 0,
	.platform_data	= fpga_spi_dev_data,
	},
};

static const struct mpsse_spi_platform_data fpga_cfg_spi_plat_data = {
	.ops		= &ft232h_intf_ops,
	.spi_info	= fpga_cfg_spi_info,
	.spi_info_len	= ARRAY_SIZE(fpga_cfg_spi_info),
};

static struct platform_device *mpsse_dev_register(struct ft232h_intf_priv *priv,
				const struct mpsse_spi_platform_data *pd)
{
	struct device *parent = &priv->intf->dev;
	struct platform_device *pdev;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size, tbl_size;
	int i, ret;
    
	ret = ft232h_intf_add_mpsse_gpio(priv);
	if (ret < 0)
		goto err;
		
        pd->spi_info[0].irq = GPIO_irqNumber;
          
        priv->ce_gpio = gpiochip_request_own_desc(&priv->mpsse_gpio, 
                                                1,
                                                "ce",
                                                GPIOD_OUT_LOW,
                                                GPIO_ACTIVE_HIGH);
                                                
        gpiod_direction_output(priv->ce_gpio, true);
    
        gpiochip_free_own_desc(priv->ce_gpio);
    
//	irq_set_irq_type(GPIO_irqNumber, IRQ_TYPE_EDGE_FALLING);
	irq_set_irq_type(GPIO_irqNumber, irqt);    
	pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
	if (!pdev)
		return NULL;

	pdev->dev.parent = parent;
	pdev->dev.fwnode = NULL;
	
	printk(KERN_INFO "spi-info irq struct = %d\n", pd->spi_info[0].irq);

	priv->spi_pdev = pdev;
    
	tbl_size = pd->spi_info_len + 1;
	lookup_size = sizeof(*lookup) + tbl_size * sizeof(struct gpiod_lookup);
	lookup = devm_kzalloc(parent, lookup_size, GFP_KERNEL);
	if (!lookup) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < pd->spi_info_len; i++) {
		dev_dbg(parent, "INFO: %s cs %d\n",
			pd->spi_info[i].modalias, pd->spi_info[i].chip_select);
	}


	ret = platform_device_add_data(pdev, pd, sizeof(*pd));
	if (ret)
		goto err;

	pdev->id = priv->id;


	lookup->dev_id = devm_kasprintf(parent, GFP_KERNEL, "%s.%d",
					pdev->name, pdev->id);
	if (!lookup->dev_id) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < pd->spi_info_len; i++) {
		lookup->table[i].key = priv->mpsse_gpio.label;
		lookup->table[i].chip_hwnum = pd->spi_info[i].chip_select;
		lookup->table[i].idx = i;
		lookup->table[i].con_id = NULL;
		if (pd->spi_info[i].mode & SPI_CS_HIGH)
			lookup->table[i].flags = GPIO_ACTIVE_HIGH;
		else
			lookup->table[i].flags = GPIO_ACTIVE_LOW;
	}

	priv->lookup_cs = lookup;

	gpiod_add_lookup_table(priv->lookup_cs);


	ret = platform_device_add(pdev);
	if (ret < 0)
		goto err_add;

	dev_dbg(&pdev->dev, "%s done\n", __func__);
	return pdev;

err_add:
	gpiod_remove_lookup_table(priv->lookup_cs);
err:
	platform_device_put(pdev);
	return ERR_PTR(ret);
}

static int ft232h_intf_spi_probe(struct usb_interface *intf,
				 const void *plat_data)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;
	struct platform_device *pdev;

	pdev = mpsse_dev_register(priv, plat_data);
	if (IS_ERR(pdev)) {
		dev_err(dev, "%s: Can't create MPSSE SPI device %ld\n",
			__func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	priv->spi_pdev = pdev;

	return 0;
}

static int ft232h_intf_spi_remove(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;

	dev_dbg(dev, "%s: spi pdev %p\n", __func__, priv->spi_pdev);
	gpiod_remove_lookup_table(priv->lookup_cs);
    priv->irq_enabled[3] = false;
	platform_device_unregister(priv->spi_pdev);
	return 0;
}

static const struct ft232h_intf_info ftdi_spi_bus_intf_info = {
    .probe  = ft232h_intf_spi_probe,
    .remove  = ft232h_intf_spi_remove,
    .plat_data  = &ftdi_spi_bus_plat_data,
};

static const struct ft232h_intf_info fpga_cfg_spi_intf_info = {
	.probe  = ft232h_intf_spi_probe,
	.remove  = ft232h_intf_spi_remove,
	.plat_data  = &fpga_cfg_spi_plat_data,
};

static const struct ft232h_intf_info fpga_cfg_fifo_intf_info = {
	.probe = ft232h_intf_fpp_probe,
	.remove = ft232h_intf_fpp_remove,
	.plat_data = &fpga_cfg_fpp_plat_data,
};

int ft232h_intf_get_model(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv;
	struct device *dev = &intf->dev;

	int ftmod1;
	int ftmod2;
	int ftmod4;
	int ret;

	ftmod2 = 2232;
	ftmod4 = 4232;
	ftmod1 = 232;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));
	if (priv->udev->product && !strcmp(priv->udev->product, "ft4232H-16ton")) {
	priv->ftmodel = ftmod4;
	} 
	if (priv->udev->product && !strcmp(priv->udev->product, "ft4233HPQ-16ton")) {
	priv->ftmodel = ftmod4;
	} 
	if (priv->udev->product && !strcmp(priv->udev->product, "ft2232H-16ton")) {
	priv->ftmodel = ftmod2;
	} 	
	if (priv->udev->product && !strcmp(priv->udev->product, "ft232H-16ton")) {
	priv->ftmodel = ftmod2;
	}
	if (priv->udev->product && !strcmp(priv->udev->product, "ft232H-16ton-spi")) {
	priv->ftmodel = ftmod1;
	} 	
	if (bind232h) {
	    if (priv->udev->product && !strcmp(priv->udev->product, "ft232H-16ton")) {
	    priv->ftmodel = ftmod1;
	    }
	}
	ret = priv->ftmodel;
	return ret;
}
EXPORT_SYMBOL_GPL(ft232h_intf_get_model);

int ft232h_intf_get_numgpio(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv;
	struct device *dev = &intf->dev;

	int ftgpio2;
	int ftgpio4;
	int ret;

	ftgpio2 = 13;
	ftgpio4 = 5;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));
	if (priv->udev->product && !strcmp(priv->udev->product, "ft4232H-16ton")) {
	priv->numgpio = ftgpio4;
	} 
	if (priv->udev->product && !strcmp(priv->udev->product, "ft4233HPQ-16ton")) {
	priv->numgpio = ftgpio4;
	} 
	if (priv->udev->product && !strcmp(priv->udev->product, "ft2232H-16ton")) {
	priv->numgpio = ftgpio2;
	} 	
	if (priv->udev->product && !strcmp(priv->udev->product, "ft232H-16ton-spi")) {
	priv->numgpio = ftgpio2;
	} 
	if (bind232h) {
	    if (priv->udev->product && !strcmp(priv->udev->product, "ft232H-16ton")) {
	    priv->numgpio = ftgpio2;
	    }
	}

	ret = priv->numgpio;
//	kfree (priv);
	return ret;
}
EXPORT_SYMBOL_GPL(ft232h_intf_get_numgpio);

static int ftx232h_jtag_probe(struct usb_interface *intf)
{
	int inf;
	inf = intf->cur_altsetting->desc.bInterfaceNumber;

	if (inf > 0) {
		dev_info(&intf->dev, "Ignoring interface reserved\n");
		return -ENODEV;
	} else {

	return 0;
  }
}

static int ft232h_intf_probe(struct usb_interface *intf,
			     const struct usb_device_id *id)
{
	struct ft232h_intf_priv *priv;
	struct device *dev = &intf->dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	const struct ft232h_intf_info *info;
	unsigned int i;
	int ret = 0;
	int inf;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));
	inf = intf->cur_altsetting->desc.bInterfaceNumber;

	ft232h_intf_get_model(intf);
	ft232h_intf_get_numgpio(intf);

     if (priv->udev->product && !strcmp(priv->udev->product, "ft4232H-16ton")) {
	 	ret = ftx232h_jtag_probe(intf);
		if (ret < 0) {
			return -ENODEV;
		}
    } else if (priv->udev->product && !strcmp(priv->udev->product, "ft4233HPQ-16ton")) {
	 	ret = ftx232h_jtag_probe(intf);
		if (ret < 0) {
			return -ENODEV;
		}
	} else if (priv->udev->product && !strcmp(priv->udev->product, "ft2232H-16ton")) {
		ret = ftx232h_jtag_probe(intf);
		if (ret < 0) {
			return -ENODEV;
		}
	} else if (bind232h) {
		if (priv->udev->product && !strcmp(priv->udev->product, "ft232H-16ton")) {
		    ret = ftx232h_jtag_probe(intf);
		    if (ret < 0) {
			return -ENODEV;
		}
	  }
	} else if (priv->udev->product && !strcmp(priv->udev->product, "ft232H-16ton-spi")) {
		ret = ftx232h_jtag_probe(intf);
		if (ret < 0) {
			return -ENODEV;
		}
	} else if (priv->udev->product && !strcmp(priv->udev->product, "ft232H-16ton-i2c")) {
		dev_info(&intf->dev, "Ignoring single I2C interface reserved\n");
		return -ENODEV;
	} else {
    return -ENODEV; 
    }	

	create_sysfs_attrs(intf);

	iface_desc = intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_out(endpoint))
			priv->bulk_out = endpoint->bEndpointAddress;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			priv->bulk_in = endpoint->bEndpointAddress;
			priv->bulk_in_sz = usb_endpoint_maxp(endpoint);
		}
	}

	priv->usb_dev_id = id;
	priv->index = 1;
	priv->intf = intf;
	priv->info = (struct ft232h_intf_info *)id->driver_info;

	info = priv->info;
	if (!info) {
		dev_err(dev, "Missing device specific driver info...\n");
		return -ENODEV;
	}

	mutex_init(&priv->io_mutex);
	mutex_init(&priv->ops_mutex);
	usb_set_intfdata(intf, priv);

	priv->bulk_in_buf = devm_kmalloc(dev, priv->bulk_in_sz, GFP_KERNEL);
	if (!priv->bulk_in_buf)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));

	priv->id = ida_simple_get(&ftdi_devid_ida, 0, 0, GFP_KERNEL);
	if (priv->id < 0)
		return priv->id;

	if (info->probe) {
		ret = info->probe(intf, info->plat_data);
		if (ret < 0)
			goto err;
		return 0;
	}

	/* for simple GPIO-only devices */
//	ret = -ENODEV;

	if (info->use_cbus_gpio_ctrl)
		ret = ft232h_intf_add_cbus_gpio(priv);
	else if (info->use_mpsse_gpio_ctrl)
		ret = ft232h_intf_add_mpsse_gpio(priv);
	if (!ret)
		return 0;

err:
	ida_simple_remove(&ftdi_devid_ida, priv->id);
	return ret;
}

static void ft232h_intf_disconnect(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	const struct ft232h_intf_info *info;
    
	if (irqon == 1) {            
		irqon = 0;
		cancel_work_sync(&priv->irq_work);
	}
    
    usleep_range(5000, 5200);
    
    kobject_put(kobj_ref);
	sysfs_remove_file(kernel_kobj, &eeprom.attr);
    
    
	info = (struct ft232h_intf_info *)priv->usb_dev_id->driver_info;
	if (info && info->remove)
		info->remove(intf);


   	if (info->use_mpsse_gpio_ctrl) {
          gpiochip_remove(&priv->mpsse_gpio);
    }
    
	if (info->use_cbus_gpio_ctrl)
		gpiochip_remove(&priv->cbus_gpio);

 
	mutex_lock(&priv->io_mutex);
 	priv->intf = NULL;
	usb_set_intfdata(intf, NULL);
	mutex_unlock(&priv->io_mutex);

	usb_put_dev(priv->udev);
	ida_simple_remove(&ftdi_devid_ida, priv->id);
//	mutex_destroy(&priv->io_mutex);
//	mutex_destroy(&priv->ops_mutex);
//	kfree (priv);
}

#define FTDI_VID			0x0403
#define ARRI_FPP_INTF_PRODUCT_ID	0x7148
#define ARRI_SPI_INTF_PRODUCT_ID	0x7149

static struct usb_device_id ft232h_intf_table[] = {
//	{ USB_DEVICE(FTDI_VID, 0x6010),
//		.driver_info = (kernel_ulong_t)&fpga_cfg_fifo_intf_info },
//	{ USB_DEVICE(FTDI_VID, ARRI_SPI_INTF_PRODUCT_ID),
//		.driver_info = (kernel_ulong_t)&fpga_cfg_spi_intf_info },
	{ USB_DEVICE(FTDI_VID, 0x6014),
        .driver_info = (kernel_ulong_t)&ftdi_spi_bus_intf_info },
	{ USB_DEVICE(FTDI_VID, 0x6010),
        .driver_info = (kernel_ulong_t)&ftdi_spi_bus_intf_info },
	{ USB_DEVICE(FTDI_VID, 0x6011),
        .driver_info = (kernel_ulong_t)&ftdi_spi_bus_intf_info },
	{ USB_DEVICE(FTDI_VID, 0x6041),
        .driver_info = (kernel_ulong_t)&ftdi_spi_bus_intf_info },
	{}
};
MODULE_DEVICE_TABLE(usb, ft232h_intf_table);

static struct usb_driver ft232h_intf_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= ft232h_intf_table,
	.probe		= ft232h_intf_probe,
	.disconnect	= ft232h_intf_disconnect,
};

module_usb_driver(ft232h_intf_driver);

MODULE_ALIAS("ft232h-intf");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de>");
MODULE_DESCRIPTION("FT232H to FPGA interface driver");
MODULE_LICENSE("GPL v2");

module_param(poll_period, uint, 0644);
MODULE_PARM_DESC(poll_period, "GPIO polling period in ms (default 10 ms)");
