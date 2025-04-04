#ifndef _LT9211_H
#define _LT9211_H

/* #define DEBUG */

/******************* MIPI Input Config ********************/
/* #define MIPI_CSI */

#define INPUT_PORTA
//#define INPUT_PORTB

#define Lane_Num 2
/******************* MIPI Input Config ********************/

/******************* Lvds Output Config ********************/
enum LT9211_LVDSPORT_ENUM {
	LVDS_1PORT = 0,
	LVDS_2PORT = 1
};
#define LVDS_PORTNUM LVDS_2PORT

enum LT9211_LVDSMODE_ENUM {
	DE_MODE = 0,
	SYNC_MODE = 1
};
#define LVDS_MODE DE_MODE

enum LT9211_LVDSDATAFORMAT_ENUM {
	VESA = 0,
	JEIDA = 1
};
#define LVDS_DATAFORMAT VESA

enum LT9211_LVDSCOLORDEPTH_ENUM {
	DEPTH_6BIT = 0,
	DEPTH_8BIT = 1
};
#define LVDS_COLORDEPTH DEPTH_8BIT

//#define LVDS_2PORT_SWAP

/******************* Lvds Output Config ********************/

struct video_timing {
	unsigned short hfp;
	unsigned short hs;
	unsigned short hbp;
	unsigned short hact;
	unsigned short htotal;
	unsigned short vfp;
	unsigned short vs;
	unsigned short vbp;
	unsigned short vact;
	unsigned short vtotal;
	unsigned int pclk_khz;
};

#endif
