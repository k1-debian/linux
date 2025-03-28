#ifndef __H264ENC_COMMON_H__
#define __H264ENC_COMMON_H__

#ifdef __KERNEL__
	#include <linux/types.h>
	#include <linux/string.h>
#else

	#include <stdint.h>
	#include <unistd.h>
	#include <string.h>
#endif

#include "osdep.h"
#include "bitstream.h"
#include "set.h"
#include "slice.h"
#include "cabac.h"
#include "nal.h"

#define C_MIN(a,b)                      ((a)<(b) ? (a) : (b))
#define C_MAX(a,b)                      ((a)>(b) ? (a) : (b))
#define C_MIN3(a,b,c)           C_MIN((a),C_MIN((b),(c)))
#define C_MAX3(a,b,c)           C_MAX((a),C_MAX((b),(c)))
#define C_MIN4(a,b,c,d)         C_MIN((a),C_MIN3((b),(c),(d)))
#define C_MAX4(a,b,c,d)         C_MAX((a),C_MAX3((b),(c),(d)))
#define C_XCHG(type,a,b)        do{ type t = a; a = b; b = t; } while(0)
#define C_FIX8(f)                       ((int)(f*(1<<8)+.5))
#define C_ALIGN(x,a)            (((x)+((a)-1))&~((a)-1))
#define ARRAY_ELEMS(a)          ((sizeof(a))/(sizeof(a[0])))

extern int c_clip3(int v, int i_min, int i_max);

enum profile_e {
	PROFILE_BASELINE = 66,
	PROFILE_MAIN    = 77,
	PROFILE_HIGH    = 100,
	PROFILE_HIGH10  = 110,
	PROFILE_HIGH422 = 122,
	PROFILE_HIGH444_PREDICTIVE = 244,
};

enum chroma_format_e {
	CHROMA_400 = 0,
	CHROMA_420 = 1,
	CHROMA_422 = 2,
	CHROMA_444 = 3,
};

#endif
