/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common definitions for FTDI FT232H interface device
 *
 * Copyright (C) 2017 - 2018 DENX Software Engineering
 * Anatolij Gustschin <agust@denx.de>
 */

#ifndef __LINUX_FT232H_INTF_H
#define __LINUX_FT232H_INTF_H

#define FTDI_CLK_6MHZ	6000000
#define FTDI_CLK_30MHZ	30000000

/* Used FTDI USB Requests */
#define FTDI_SIO_RESET_REQUEST		0x00
#define FTDI_SIO_SET_BAUDRATE_REQUEST	0x03
#define FTDI_SIO_SET_LATENCY_TIMER_REQUEST	0x09
#define FTDI_SIO_SET_BITMODE_REQUEST	0x0B
#define FTDI_SIO_READ_PINS_REQUEST	0x0C
#define FTDI_SIO_READ_EEPROM_REQUEST	0x90

/* MPSSE Commands */
#define TX_BYTES_RE_MSB		0x10 /* tx on +ve clk (rising edge) */
#define TX_BYTES_FE_MSB		0x11 /* tx on -ve clk (falling edge) */
#define RX_BYTES_RE_MSB		0x20
#define RX_BYTES_FE_MSB		0x24
#define TXF_RXR_BYTES_MSB	0x31 /* tx on -ve clk, rx on +ve */
#define TXR_RXF_BYTES_MSB	0x34 /* tx on +ve clk, rx on -ve */

#define TX_BYTES_RE_LSB		0x18 /* tx on +ve clk */
#define TX_BYTES_FE_LSB		0x19 /* tx on -ve clk */
#define RX_BYTES_RE_LSB		0x28
#define RX_BYTES_FE_LSB		0x2C
#define TXF_RXR_BYTES_LSB	0x39 /* tx on -ve clk, rx on +ve */
#define TXR_RXF_BYTES_LSB	0x3C /* tx on +ve clk, rx on -ve */

#define LOOPBACK_ON		0x84
#define LOOPBACK_OFF		0x85
#define TCK_DIVISOR		0x86
#define SEND_IMMEDIATE		0x87
#define DIS_DIV_5		0x8A
#define EN_DIV_5		0x8B
#define EN_3_PHASE		0x8C
#define DIS_3_PHASE		0x8D
#define DIS_ADAPTIVE		0x97
#define EN_ADAPTIVE     0x96

/* For EEPROM I/O mode */
#define FTDI_MAX_EEPROM_SIZE	256
#define FTDI_232H_CBUS_IOMODE	0x08

#define FTDI_USB_READ_TIMEOUT	5000
#define FTDI_USB_WRITE_TIMEOUT	5000

/* Total number of MPSSE GPIOs: 4x GPIOL, 8x GPIOH, 1x CS on ADBUS3 */
#define FTDI_MPSSE_GPIOS5	5

#define FTDI_MPSSE_GPIOS13	13

/* MPSSE bitbang modes (copied from libftdi) */
enum ftdi_mpsse_mode {
	BITMODE_RESET	= 0x00,	/* switch off bitbang mode */
	BITMODE_BITBANG	= 0x01,	/* asynchronous bitbang mode */
	BITMODE_MPSSE	= 0x02,	/* MPSSE mode, on 2232x chips */
	BITMODE_SYNCBB	= 0x04,	/* synchronous bitbang mode  */
	BITMODE_MCU	= 0x08,	/* MCU Host Bus Emulation mode */
				/* CPU-style fifo mode gets set via EEPROM */
	BITMODE_OPTO	= 0x10,	/* Fast Opto-Isolated Serial Interface Mode */
	BITMODE_CBUS	= 0x20,	/* Bitbang on CBUS pins, EEPROM config needed */
	BITMODE_SYNCFF	= 0x40,	/* Single Channel Synchronous FIFO mode */
	BITMODE_FT1284	= 0x80,	/* FT1284 mode, available on 232H chips */
};

struct ctrl_desc {
	unsigned int dir_out;
	u8 request;
	u8 requesttype;
	u16 value;
	u16 index;
	u16 size;
	void *data;
	int timeout;
};

struct bulk_desc {
	unsigned int dir_out;
	void *data;
	int len;
	int act_len;
	int timeout;
};

/*
 * struct ft232h_intf_ops - FT232H interface operations for upper drivers
 *
 * @bulk_xfer: FTDI USB bulk transfer
 * @ctrl_xfer: FTDI USB control transfer
 * @read_data: read 'len' bytes from FTDI device to the given buffer
 * @write_data: write 'len' bytes from the given buffer to the FTDI device
 * @lock: lock the interface for an operation sequence. Used when multiple
 *	  command and/or data operations must be executed in a specific order
 *	  (when other intermediate command/data transfers may not interfere)
 * @unlock: unlock the previously locked interface
 * @set_bitmode: configure FTDI bit mode
 * @set_baudrate: configure FTDI baudrate
 * @disable_bitbang: turn off bitbang mode
 * @init_pins: initialize GPIOL/GPIOH port pins in MPSSE mode
 * @cfg_bus_pins: configure MPSSE SPI bus pins
 *
 * Common FT232H interface USB xfer and device configuration operations used
 * in FIFO-FPP, MPSSE-SPI or MPSSE-I2C drivers. Many of them are like FTDI
 * protocol functions, which I mainly borrowed from libftdi
 */
struct ft232h_intf_ops {
	int (*bulk_xfer)(struct usb_interface *intf, struct bulk_desc *desc);
	int (*ctrl_xfer)(struct usb_interface *intf, struct ctrl_desc *desc);
	int (*read_data)(struct usb_interface *intf, void *buf, size_t len);
	int (*write_data)(struct usb_interface *intf, const char *buf,
			  size_t len);
	void (*lock)(struct usb_interface *intf);
	void (*unlock)(struct usb_interface *intf);
	int (*set_bitmode)(struct usb_interface *intf, unsigned char bitmask,
			   unsigned char mode);
	int (*set_baudrate)(struct usb_interface *intf, int baudrate);
	int (*disable_bitbang)(struct usb_interface *intf);
	int (*init_pins)(struct usb_interface *intf, bool low, u8 bits, u8 dir);
	int (*cfg_bus_pins)(struct usb_interface *intf, u8 dir_bits,
			    u8 value_bits);
	int (*set_clock)(struct usb_interface *intf, int clock_freq_hz);
	int (*set_latency)(struct usb_interface *intf, int latency_msec);
};

/*
 * struct dev_io_desc_data - Descriptor of FT232H pin used by attached device
 * @con_id: Name of the GPIO pin
 * @idx: Index of the pin
 * @flags: GPIOD flags of the pin
 *
 * Description of a GPIO used by device connected to FT232H
 */
struct dev_io_desc_data {
	const char *con_id;
	unsigned int idx;
	unsigned int flags;
};

/*
 * struct fifo_fpp_mgr_platform_data - FIFO/FPP device platform data
 * @ops: USB interface operations used in FPP manager driver
 * @io_data: Array with descriptors of used I/O pins
 * @io_data_len: Length of io_data array
 * @nconfig_num: ACBUS pin number of the NCONFIG pin
 * @conf_done_num: ACBUS pin number of the CONF_DONE pin
 *
 * FIFO/FPP fpga manager specific platform data
 */
struct fifo_fpp_mgr_platform_data {
	const struct ft232h_intf_ops *ops;
	struct dev_io_desc_data *io_data;
	size_t io_data_len;
	int nconfig_num;
	int conf_done_num;
};

#define FTDI_MPSSE_IO_DESC_MAGIC	0x5345494F
/*
 * struct mpsse_spi_dev_data - MPSSE SPI device platform data
 * @magic: Special # indicating that this is a I/O descriptor struct
 * @io_data: Array with descriptors of used I/O pins
 * @io_data_len: Length of io_data array
 *
 * MPSSE SPI slave specific platform data describing additional
 * I/O pins (if any) of attached SPI slave. It is supposed to be
 * passed via .platform_data of spi_board_info struct.
 * To differentiate between MPSSE I/O descriptor data and other
 * driver-specific platform data we use FTDI_MPSSE_IO_DESC_MAGIC
 * in the header of this struct
 */
struct mpsse_spi_dev_data {
	u32 magic;
	struct dev_io_desc_data *desc;
	size_t desc_len;
};

/*
struct dev_info_desc_data {
	unsigned int buswidth;
	unsigned int backlight;
	unsigned int bgr;
};
*/
/*
 * struct mpsse_spi_platform_data - MPSSE SPI bus platform data
 * @ops: USB interface operations used in MPSSE SPI controller driver
 * @spi_info: Array with spi_board_info structures of attached SPI slaves
 * @spi_info_len: Length of spi_info array
 * @io_data: Array with descriptors of used I/O pins
 * @io_data_len: Length of io_data array
 *
 * MPSSE SPI specific platform data describing attached SPI slaves and
 * their additional I/O pins
 */
struct mpsse_spi_platform_data {
	const struct ft232h_intf_ops *ops;
	struct spi_board_info *spi_info;
	size_t spi_info_len;
	u32 magic;
	struct dev_io_desc_data *desc;
	size_t desc_len;
//	struct dev_info_desc_data *data;
//	size_t data_len;
//	struct dev_io_desc_data *io_data;
//	size_t io_data_len;
//	int dc;
};

/*
 * Value HIGH. rate is 12000000 / ((1 + value) * 2)
 */
static inline int div_value(int rate)
{
	int r;

	if (rate >= 6000000)
		return 0;
	r = 6000000 / rate - 1;
	if (r < 0xffff)
		return r;
	return 0xffff;
}

extern int ft232h_intf_get_model(struct usb_interface *intf);
extern int ft232h_intf_get_numgpio(struct usb_interface *intf);
#endif /* __LINUX_FT232H_INTF_H */
