#ifndef __TCSM_LAYOUT_H__
#define __TCSM_LAYOUT_H__

/**
 *      |-------------|
 *      |    BANK0    |
 *      |-------------|     <--- SLEEP_TCSM_BOOTCODE_TEXT
 *      | BOOT CODE   |
 *      |-------------|     <--- SLEEP_TCSM_RESUMECODE_TEXT
 *      |    ...      |
 *      | RESUME CODE |
 *      |    ...      |
 *      |-------------|     <--- SLEEP_TCSM_RESUME_DATA
 *      |    ...      |
 *      | RESUME DATA |
 *      |    ...      |
 *      |_____________|     <--- SLEEP_TCSM_CPU_RESMUE_SP
 *
 *
 *      |-------------|
 *      |    BANK1    |
 *      |-------------|     <--- TO BE DEFINED
 *      |             |
 *      |             |
 *      |    ...      |
 *      |  VOICE DATA |
 *      |    ...      |
 *      |             |
 *      |             |
 *      |_____________|
 *
 */

#define SLEEP_TCSM_SPACE           0xb3422000
#define VOICE_TCSM_DATA_BUF    0xb3423000
#define TCSM_BANK_LEN              4096

#define SLEEP_TCSM_BOOT_TEXT       (SLEEP_TCSM_SPACE)
#define SLEEP_TCSM_BOOT_LEN        64
#define SLEEP_TCSM_BOOT_END        (SLEEP_TCSM_BOOT_TEXT + SLEEP_TCSM_BOOT_LEN)

#define SLEEP_TCSM_RESUME_TEXT     (SLEEP_TCSM_BOOT_END)
#define SLEEP_TCSM_RESUME_LEN      2048
#define SLEEP_TCSM_RESUME_END      (SLEEP_TCSM_RESUME_TEXT + SLEEP_TCSM_RESUME_LEN)

#define SLEEP_TCSM_RESUME_DATA     (SLEEP_TCSM_RESUME_END)

#define SLEEP_TCSM_SPACE_END       (SLEEP_TCSM_SPACE + TCSM_BANK_LEN)

#define SLEEP_TCSM_CPU_RESMUE_SP   SLEEP_TCSM_SPACE_END - 4


#endif  /*  __TCSM_LAYOUT_H__ */
