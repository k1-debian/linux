#ifndef _MCU_FIRMWARE_H_
#define _MCU_FIRMWARE_H_

struct mcu_firmware_header {
	unsigned long code[2];
	unsigned long version;
	unsigned long start_addr;
	unsigned long end_addr;
	unsigned long rsc_table_start;
	unsigned long rsc_table_end;
};


#endif /* _MCU_FIRMWARE_H_ */
