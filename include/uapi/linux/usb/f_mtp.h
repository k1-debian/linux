/*
 * Gadget Function Driver for MTP
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _UAPI_LINUX_USB_F_MTP_H
#define _UAPI_LINUX_USB_F_MTP_H

#include <linux/ioctl.h>
#include <linux/types.h>

struct mtp_file_range {
	/* file descriptor for file to transfer */
	int         fd;
	/* offset in file for start of transfer */
	loff_t      offset;
	/* number of bytes to transfer */
	int64_t     length;
	/* MTP command ID for data header,
	 * used only for MTP_SEND_FILE_WITH_HEADER
	 */
	uint16_t    command;
	/* MTP transaction ID for data header,
	 * used only for MTP_SEND_FILE_WITH_HEADER
	 */
	uint32_t    transaction_id;
};

struct mtp_event {
	/* size of the event */
	size_t      length;
	/* event data to send */
	void        *data;
};

/* Sends the specified file range to the host */
#define MTP_SEND_FILE              _IOW('M', 0, struct mtp_file_range)
/* Receives data from the host and writes it to a file.
 * The file is created if it does not exist.
 */
#define MTP_RECEIVE_FILE           _IOW('M', 1, struct mtp_file_range)
/* Sends an event to the host via the interrupt endpoint */
#define MTP_SEND_EVENT             _IOW('M', 3, struct mtp_event)
/* Sends the specified file range to the host,
 * with a 12 byte MTP data packet header at the beginning.
 */
#define MTP_SEND_FILE_WITH_HEADER  _IOW('M', 4, struct mtp_file_range)

#endif /* _UAPI_LINUX_USB_F_MTP_H */
