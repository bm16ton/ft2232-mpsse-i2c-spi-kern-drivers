/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __FTDI_MPSSE_H
#define __FTDI_MPSSE_H

#include <linux/errno.h>
#include <linux/types.h>

#define EN_ADAPTIVE     0x96
#define DIS_ADAPTIVE     0x97

struct ftdi_mpsse_cmd {
	u8 *buffer;
	size_t offset;
	size_t size;
};

static inline void ftdi_mpsse_cmd_setup(
	struct ftdi_mpsse_cmd *cmd, u8 *buffer, size_t size)
{
	cmd->buffer = buffer;
	cmd->size = size;
	cmd->offset = 0;
}

static inline void ftdi_mpsse_cmd_reset(struct ftdi_mpsse_cmd *cmd)
{
	cmd->offset = 0;
}

static inline int ftdi_mpsse_command(struct ftdi_mpsse_cmd *cmd, u8 command)
{
	if (cmd->offset + 1 > cmd->size)
		return -ENOMEM;
	cmd->buffer[cmd->offset++] = command;
	return 0;
}

static inline int ftdi_mpsse_enable_adaptive_clocking(
	struct ftdi_mpsse_cmd *cmd)
{
	return ftdi_mpsse_command(cmd, 0x96);
}

static inline int ftdi_mpsse_disable_adaptive_clocking(
	struct ftdi_mpsse_cmd *cmd)
{
	return ftdi_mpsse_command(cmd, 0x97);
}

static inline int ftdi_mpsse_disable_loopback(struct ftdi_mpsse_cmd *cmd)
{
	return ftdi_mpsse_command(cmd, 0x85);
}

// For I2C when nobody drives the pin it should take the value 1 and all devices
// to set the pin to value 1 should just release it.
static inline int ftdi_mpsse_set_drive0_pins(
	struct ftdi_mpsse_cmd *cmd, unsigned pinmask)
{
	if (cmd->offset + 3 > cmd->size)
		return -ENOMEM;
	cmd->buffer[cmd->offset++] = 0x9e;
	cmd->buffer[cmd->offset++] = pinmask & 0xff;
	cmd->buffer[cmd->offset++] = (pinmask >> 8) & 0xff;
	return 0;
}

static inline int ftdi_mpsse_set_freq(struct ftdi_mpsse_cmd *cmd, unsigned freq)
{
	const unsigned div = 20000000 / freq - 1;

	if (cmd->offset + 5 > cmd->size)
		return -ENOMEM;
	// FTDI may be in a mode of operation when they divide the base clock
	// frequency by 5. For the formula above to be correct we need to make
	// sure that it's disabled.
	cmd->buffer[cmd->offset++] = 0x8a;
	// For I2C it's important that the data value stays stable when the
	// clock signal changes. In order to support that need FTDI has a
	// mode of operation that is called 3-phase clocking which extends the
	// clock period by half and as such affects frequency calculations.
	cmd->buffer[cmd->offset++] = 0x8c;
	// Finally, this sets the frequency divisor.
	cmd->buffer[cmd->offset++] = 0x86;
	cmd->buffer[cmd->offset++] = div & 0xff;
	cmd->buffer[cmd->offset++] = (div >> 8) & 0xff;
	return 0;
}

static inline int ftdi_mpsse_set_output_232h(
	struct ftdi_mpsse_cmd *cmd, unsigned pinmask, unsigned pinvals)
{
	if (cmd->offset + 6 > cmd->size)
		return -ENOMEM;

	cmd->buffer[cmd->offset++] = 0x80;
	cmd->buffer[cmd->offset++] = pinvals & 0xff;
	cmd->buffer[cmd->offset++] = pinmask & 0x03;
	cmd->buffer[cmd->offset++] = 0x82;
	cmd->buffer[cmd->offset++] = (pinvals >> 8) & 0xff;
	cmd->buffer[cmd->offset++] = (pinmask >> 8) & 0xff;
	return 0;
}

static inline int ftdi_mpsse_set_output(
	struct ftdi_mpsse_cmd *cmd, unsigned pinmask, unsigned pinvals)
{
	if (cmd->offset + 6 > cmd->size)
		return -ENOMEM;

	cmd->buffer[cmd->offset++] = 0x80;
	cmd->buffer[cmd->offset++] = pinvals & 0xff;
//	cmd->buffer[cmd->offset++] = pinmask & 0xff;
    cmd->buffer[cmd->offset++] = pinmask & 0x03;
	cmd->buffer[cmd->offset++] = 0x82;
	cmd->buffer[cmd->offset++] = (pinvals >> 8) & 0xff;
	cmd->buffer[cmd->offset++] = (pinmask >> 8) & 0xff;
	return 0;
}

static inline int ftdi_mpsse_write_bytes(
	struct ftdi_mpsse_cmd *cmd, const u8 *data, size_t size)
{
	if (size == 0)
		return 0;

	if (cmd->offset + 3 + size > cmd->size)
		return -ENOMEM;

	cmd->buffer[cmd->offset++] = 0x11;
	cmd->buffer[cmd->offset++] = (size - 1) & 0xff;
	cmd->buffer[cmd->offset++] = ((size - 1) >> 8) & 0xff;
	memcpy(cmd->buffer + cmd->offset, data, size);
	cmd->offset += size;
	return 0;
}

static inline int ftdi_mpsse_write_bits(
	struct ftdi_mpsse_cmd *cmd, u8 data, size_t bits)
{
	if (bits == 0)
		return 0;

	if (cmd->offset + 3 > cmd->size)
		return -ENOMEM;

	cmd->buffer[cmd->offset++] = 0x13;
	cmd->buffer[cmd->offset++] = (bits - 1) & 0xff;
	cmd->buffer[cmd->offset++] = data;
	return 0;
}

static inline int ftdi_mpsse_read_bytes(struct ftdi_mpsse_cmd *cmd, size_t size)
{
	if (size == 0)
		return 0;

	if (cmd->offset + 3 > cmd->size)
		return -ENOMEM;

	cmd->buffer[cmd->offset++] = 0x20;
	cmd->buffer[cmd->offset++] = (size - 1) & 0xff;
	cmd->buffer[cmd->offset++] = ((size - 1) >> 8) & 0xff;
	return 0;
}

static inline int ftdi_mpsse_read_bits(struct ftdi_mpsse_cmd *cmd, size_t bits)
{
	if (bits == 0)
		return 0;

	if (cmd->offset + 2 > cmd->size)
		return -ENOMEM;

	cmd->buffer[cmd->offset++] = 0x22;
	cmd->buffer[cmd->offset++] = (bits - 1) & 0xff;
	return 0;
}

// This command forces FTDI MPSSE chip to send the data it read back immediately
// without waiting for more data to arrive to fill the read buffer.
static inline int ftdi_mpsse_complete(struct ftdi_mpsse_cmd *cmd)
{
	return ftdi_mpsse_command(cmd, 0x87);
}

#endif  /* __FTDI_MPSSE_H */
