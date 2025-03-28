#ifndef _JZM_H264_API_C_
#define _JZM_H264_API_C_

#include "jzm_h264_dec.h"
#include <linux/types.h>

__place_k0_data__
static char AryFMT[] = {IS_SKIRT, IS_MIRROR, IS_SKIRT, IS_SKIRT,
                        IS_SKIRT, IS_SKIRT, IS_SKIRT, IS_SKIRT,
                        IS_SKIRT, IS_SKIRT, IS_SKIRT, IS_SKIRT,
                        IS_SKIRT, IS_SKIRT, IS_SKIRT, IS_SKIRT
                       };

__place_k0_data__
static char SubPel[] = {HPEL, QPEL, QPEL, EPEL,
                        QPEL, QPEL, QPEL, QPEL,
                        QPEL, QPEL, QPEL, QPEL,
                        EPEL, HPEL, QPEL, QPEL
                       };

__place_k0_data__
static IntpFMT_t IntpFMT[][16] = {
	{
		/************* MPEG_HPEL ***************/
		{/*H0V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
	},

	{
		/************* MPEG_QPEL ***************/
		{/*H0V0*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V1*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 1},
			{/*intp_srnd*/1, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 1},
			{/*intp_srnd*/1, 1},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V2*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V3*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H1V3*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 1},
			{/*intp_srnd*/1, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H2V3*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H3V3*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 1},
			{/*intp_srnd*/1, 1},
			{/*intp_sbias*/1, 1},
		},
	},

	{
		/************* H264_QPEL ***************/
		{/*H0V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V1*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP6, {NOT_TCS, IS_SCS}, IS_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP6, {NOT_TCS, IS_SCS}, IS_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H0V3*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H1V3*/
			TAP6, {NOT_TCS, IS_SCS}, IS_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H2V3*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H3V3*/
			TAP6, {NOT_TCS, IS_SCS}, IS_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/1, 1},
		},
	},

	{
		/************* H264_EPEL ***************/
		{/*H0V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/0},
		{/*H2V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V0*/0},
		{/*H0V1*/0},
		{/*H1V1*/0},
		{/*H2V1*/0},
		{/*H3V1*/0},
		{/*H0V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V2*/0},
		{/*H2V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V2*/0},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* RV8_TPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 12, 6, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 6, 12, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/0},
		{/*H0V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 12, 6, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 12, 6, -1, 0, 0, 0, 0}, {-1, 12, 6, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 0}, //{0,128}
			{/*intp_sft*/0, 8},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 6, 12, -1, 0, 0, 0, 0}, {-1, 12, 6, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 0}, //{0,128}
			{/*intp_sft*/0, 8},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/0},
		{/*H0V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 6, 12, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 12, 6, -1, 0, 0, 0, 0}, {-1, 6, 12, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 0}, //{0,128}
			{/*intp_sft*/0, 8},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 6, 12, -1, 0, 0, 0, 0}, {-1, 6, 12, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 0}, //{0,128}
			{/*intp_sft*/0, 8},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/0},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* RV9_QPEL ***************/
		{/*H0V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 52, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {1, -5, 52, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 52, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 32},
			{/*intp_sft*/5, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 52, -5, 1, 0, 0}, {1, -5, 52, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 16},
			{/*intp_sft*/6, 5},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 52, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 16},
			{/*intp_sft*/6, 5},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 52, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {1, -5, 20, 52, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 52, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 32},
			{/*intp_sft*/5, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* RV9_CPEL ***************/
		{/*H0V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 7},
			{/*intp_sft*/0, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 4},
			{/*intp_sft*/0, 3},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 7},
			{/*intp_sft*/0, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 4},
			{/*intp_sft*/0, 3},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 1},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 4},
			{/*intp_sft*/0, 3},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 7},
			{/*intp_sft*/0, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 4},
			{/*intp_sft*/0, 3},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 1},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* WMV2_QPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V1*/0},
		{/*H1V1*/0},
		{/*H2V1*/0},
		{/*H3V1*/0},
		{/*H0V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 8},
			{/*intp_sft*/4, 4},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 8},
			{/*intp_sft*/4, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 8},
			{/*intp_sft*/4, 4},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* VC1_QPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/31, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {-4, 53, 18, -3, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 8},
			{/*intp_sft*/6, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {-3, 18, 53, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/7, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-4, 53, 18, -3, 0, 0, 0, 0},},
			{/*intp_rnd*/7, 32},
			{/*intp_sft*/4, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/7, 8},
			{/*intp_sft*/4, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-3, 18, 53, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/7, 32},
			{/*intp_sft*/4, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/31, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {-4, 53, 18, -3, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 8},
			{/*intp_sft*/6, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {-3, 18, 53, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* AVS_QPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, -2, 96, 42, -7, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{0, -7, 42, 96, -2, -1, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, -2, 96, 42, -7, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, IS_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, -5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, -1, 5, 5, -1, 0, 0, 0}, {-1, -2, 96, 42, -7, 0, 0, 0},},
			{/*intp_rnd*/64, 0}, //{0,512}
			{/*intp_sft*/0, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, IS_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, -5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, -2, 96, 42, -7, 0, 0, 0}, {0, -1, 5, 5, -1, 0, 0, 0},},
			{/*intp_rnd*/64, 0}, //{0,512}
			{/*intp_sft*/0, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, 5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, -7, 42, 96, -2, -1, 0, 0}, {0, -1, 5, 5, -1, 0, 0, 0},},
			{/*intp_rnd*/64, 0}, //{0,512}
			{/*intp_sft*/0, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{0, -7, 42, 96, -2, -1, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, IS_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, -5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H2V3*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, -1, 5, 5, -1, 0, 0, 0}, {0, -7, 42, 96, -2, -1, 0, 0},},
			{/*intp_rnd*/64, 0}, //{0,512}
			{/*intp_sft*/0, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, IS_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, -5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/1, 1},
		},
	},

	{
		/************* VP6_QPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {-4, 109, 24, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {-4, 109, 24, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {-4, 109, 24, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {-4, 68, 68, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {-4, 68, 68, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {-4, 68, 68, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {-1, 24, 109, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {-1, 24, 109, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {-1, 24, 109, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* VP8_QPEL ***************/
		{/*H0V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {2, -11, 108, 36, -8, 1, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {2, -11, 108, 36, -8, 1, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {2, -11, 108, 36, -8, 1, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {3, -16, 77, 77, -16, 3, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {3, -16, 77, 77, -16, 3, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {3, -16, 77, 77, -16, 3, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {1, -8, 36, 108, -11, 2, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {1, -8, 36, 108, -11, 2, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {1, -8, 36, 108, -11, 2, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* VP8_EPEL ***************/
		{/*H0V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/0},
		{/*H2V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V0*/0},
		{/*H0V1*/0},
		{/*H1V1*/0},
		{/*H2V1*/0},
		{/*H3V1*/0},
		{/*H0V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V2*/0},
		{/*H2V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 4},
			{/*intp_sft*/3, 3},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V2*/0},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* VP8_BIL ***************/
		{/*H0V0*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V0*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V0*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V1*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V1*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 2},
			{/*intp_sft*/2, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V1*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/1, 2},
			{/*intp_sft*/1, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V1*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 2},
			{/*intp_sft*/2, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V2*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V2*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 1},
			{/*intp_sft*/2, 1},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V2*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/1, 1},
			{/*intp_sft*/1, 1},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V2*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 1},
			{/*intp_sft*/2, 1},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V3*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V3*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 2},
			{/*intp_sft*/2, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V3*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/1, 2},
			{/*intp_sft*/1, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V3*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 2},
			{/*intp_sft*/2, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
	},

	{
		/************* VP8_FPEL ***************/
		{/*H0V0*/0},
		{/*H1V0*/0},
		{/*H2V0*/0},
		{/*H3V0*/0},
		{/*H0V1*/0},
		{/*H1V1*/0},
		{/*H2V1*/0},
		{/*H3V1*/0},
		{/*H0V2*/0},
		{/*H1V2*/0},
		{/*H2V2*/0},
		{/*H3V2*/0},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},
};
__place_k0_data__ uint32_t lps_comb[128] = {
	0xefcfaffe, 0xefcfafff, 0xe2c4a6fe, 0xe2c4a6ff, 0xd7ba9dfe, 0xd7ba9dff, 0xccb195f4, 0xccb195f5,
	0xc2a88de6, 0xc2a88de7, 0xb89f86dc, 0xb89f86dd, 0xae977fd0, 0xae977fd1, 0xa58f79c6, 0xa58f79c7,
	0x9d8873bc, 0x9d8873bd, 0x95816db2, 0x95816db3, 0x8d7a67a8, 0x8d7a67a9, 0x867462a0, 0x867462a1,
	0x7f6e5d98, 0x7f6e5d99, 0x79685890, 0x79685891, 0x73635488, 0x73635489, 0x6d5e4f82, 0x6d5e4f83,
	0x67594b7a, 0x67594b7b, 0x62554774, 0x62554775, 0x5d50446e, 0x5d50446f, 0x584c4068, 0x584c4069,
	0x54483d64, 0x54483d65, 0x4f443a5e, 0x4f443a5f, 0x4b41375a, 0x4b41375b, 0x473e3454, 0x473e3455,
	0x443a3150, 0x443a3151, 0x40372f4c, 0x40372f4d, 0x3d352c48, 0x3d352c49, 0x3a322a44, 0x3a322a45,
	0x372f2840, 0x372f2841, 0x342d263e, 0x342d263f, 0x312a243a, 0x312a243b, 0x2f282238, 0x2f282239,
	0x2c262034, 0x2c262035, 0x2a241e32, 0x2a241e33, 0x28221d2e, 0x28221d2f, 0x26201b2c, 0x26201b2d,
	0x241f1a2a, 0x241f1a2b, 0x221d1928, 0x221d1929, 0x201c1726, 0x201c1727, 0x1e1a1624, 0x1e1a1625,
	0x1d191522, 0x1d191523, 0x1b181420, 0x1b181421, 0x1a16131e, 0x1a16131f, 0x1815121c, 0x1815121d,
	0x1714111a, 0x1714111b, 0x1613101a, 0x1613101b, 0x15120f18, 0x15120f19, 0x14110e16, 0x14110e17,
	0x13100d16, 0x13100d17, 0x120f0d14, 0x120f0d15, 0x110e0c14, 0x110e0c15, 0x100e0b12, 0x100e0b13,
	0xf0d0b12, 0xf0d0b13, 0xe0c0a10, 0xe0c0a11, 0xd0b0a10, 0xd0b0a11, 0xd0b090e, 0xd0b090f, 0xc0a080e,
	0xc0a080f, 0xb0a080c, 0xb0a080d, 0xb09080c, 0xb09080d, 0xa09070c, 0xa09070d, 0xa08070a, 0xa08070b,
	0x908060a, 0x908060b, 0x807060a, 0x807060b, 0x1010102, 0x1010103
};

__place_k0_data__ struct SDE_VLC_STA sde_vlc2_sta[7] = {
	{0, 128, 6},   // vlc_tables_coeff_token_table_0
	{128, 116, 5}, // vlc_tables_coeff_token_table_1
	{256, 104, 6}, // vlc_tables_coeff_token_table_2
	{384,  64, 6}, // vlc_tables_coeff_token_table_3
	{512,  70, 6}, // vlc_tables_chroma_dc_coeff_token_table
	{640,  74, 6}, // vlc_tables_total_zeros_table_0
	{768,  96, 6}, // vlc_tables_run7_table
};
__place_k0_data__ uint16_t sde_vlc2_table[7][128] = {
	{
		// 0
		0xa020, 0x887c, 0x806c, 0x200f, 0x100a, 0x100a, 0x100a, 0x100a,
		0x0805, 0x0805, 0x0805, 0x0805, 0x0805, 0x0805, 0x0805, 0x0805,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // 32
		0xa040, 0x9070, 0x8078, 0x807a, 0x2023, 0x201a, 0x2015, 0x2010,
		0x181f, 0x181f, 0x1816, 0x1816, 0x1811, 0x1811, 0x180c, 0x180c,
		0x101b, 0x101b, 0x101b, 0x101b, 0x1012, 0x1012, 0x1012, 0x1012,
		0x100d, 0x100d, 0x100d, 0x100d, 0x1008, 0x1008, 0x1008, 0x1008, // 64
		0x4001, 0x2035, 0x8060, 0x8062, 0x8064, 0x8066, 0x8068, 0x806a,
		0x203b, 0x2036, 0x2031, 0x2030, 0x2037, 0x2032, 0x202d, 0x202c,
		0x1833, 0x1833, 0x182e, 0x182e, 0x1829, 0x1829, 0x1828, 0x1828,
		0x182f, 0x182f, 0x182a, 0x182a, 0x1825, 0x1825, 0x1824, 0x1824, // 96
		0x0040, 0x0042, 0x0041, 0x003c, 0x0043, 0x003e, 0x003d, 0x0038, // 104
		0x003f, 0x003a, 0x0039, 0x0034, 0x0009, 0x0004, 0x4000, 0x4000, // 112
		0x1020, 0x1026, 0x1021, 0x101c, 0x102b, 0x1022, 0x101d, 0x1018, // 120
		0x0027, 0x001e, 0x0019, 0x0014, 0x0817, 0x080e, 0x0013, 0x0013, // 128
	},
	{
		// 1
		0xa020, 0x8868, 0x806c, 0x806e, 0x8070, 0x8072, 0x2017, 0x2009,
		0x1813, 0x1813, 0x180f, 0x180f, 0x100a, 0x100a, 0x100a, 0x100a,
		0x0805, 0x0805, 0x0805, 0x0805, 0x0805, 0x0805, 0x0805, 0x0805,
		0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800,
		0x9840, 0x9050, 0x8858, 0x885c, 0x8060, 0x8062, 0x8064, 0x8066,
		0x1827, 0x1827, 0x181e, 0x181e, 0x181d, 0x181d, 0x1818, 0x1818,
		0x1014, 0x1014, 0x1014, 0x1014, 0x101a, 0x101a, 0x101a, 0x101a,
		0x1019, 0x1019, 0x1019, 0x1019, 0x1010, 0x1010, 0x1010, 0x1010,
		0x4001, 0x4001, 0x103f, 0x103f, 0x1843, 0x1842, 0x1841, 0x1840,
		0x183d, 0x183c, 0x183e, 0x1839, 0x103a, 0x103a, 0x1038, 0x1038,
		0x103b, 0x1036, 0x1035, 0x1034, 0x1037, 0x1032, 0x1031, 0x1030,
		0x082c, 0x082e, 0x082d, 0x0828, 0x0833, 0x082a, 0x0829, 0x0824,
		0x002f, 0x0026, 0x0025, 0x0020, 0x002b, 0x0022, 0x0021, 0x001c,
		0x0823, 0x0816, 0x0815, 0x080c, 0x001f, 0x0012, 0x0011, 0x0008,
		0x001b, 0x000e, 0x000d, 0x0004,
	},
	{
		// 2
		0x9840, 0x9050, 0x8858, 0x885c, 0x8060, 0x8062, 0x8064, 0x8066,
		0x280c, 0x281e, 0x281d, 0x2808, 0x2827, 0x281a, 0x2819, 0x2804,
		0x2015, 0x2015, 0x2016, 0x2016, 0x2011, 0x2011, 0x2012, 0x2012,
		0x200d, 0x200d, 0x2023, 0x2023, 0x200e, 0x200e, 0x2009, 0x2009,
		0x181f, 0x181f, 0x181f, 0x181f, 0x181b, 0x181b, 0x181b, 0x181b,
		0x1817, 0x1817, 0x1817, 0x1817, 0x1813, 0x1813, 0x1813, 0x1813,
		0x180f, 0x180f, 0x180f, 0x180f, 0x180a, 0x180a, 0x180a, 0x180a,
		0x1805, 0x1805, 0x1805, 0x1805, 0x1800, 0x1800, 0x1800, 0x1800,
		0x4001, 0x1840, 0x1843, 0x1842, 0x1841, 0x183c, 0x183f, 0x183e,
		0x183d, 0x1838, 0x183b, 0x183a, 0x1839, 0x1834, 0x1035, 0x1035,
		0x1030, 0x1036, 0x1031, 0x102c, 0x1037, 0x1032, 0x102d, 0x1028,
		0x0833, 0x082e, 0x0829, 0x0824, 0x082f, 0x082a, 0x0825, 0x0820,
		0x001c, 0x0018, 0x0026, 0x0014, 0x002b, 0x0022, 0x0021, 0x0010,
	},
	{
		// 3
		0x2804, 0x2805, 0x4001, 0x2800, 0x2808, 0x2809, 0x280a, 0x4001,
		0x280c, 0x280d, 0x280e, 0x280f, 0x2810, 0x2811, 0x2812, 0x2813,
		0x2814, 0x2815, 0x2816, 0x2817, 0x2818, 0x2819, 0x281a, 0x281b,
		0x281c, 0x281d, 0x281e, 0x281f, 0x2820, 0x2821, 0x2822, 0x2823,
		0x2824, 0x2825, 0x2826, 0x2827, 0x2828, 0x2829, 0x282a, 0x282b,
		0x282c, 0x282d, 0x282e, 0x282f, 0x2830, 0x2831, 0x2832, 0x2833,
		0x2834, 0x2835, 0x2836, 0x2837, 0x2838, 0x2839, 0x283a, 0x283b,
		0x283c, 0x283d, 0x283e, 0x283f, 0x2840, 0x2841, 0x2842, 0x2843,
	},
	{
		// 4
		0x8840, 0x8044, 0x2810, 0x280c, 0x2808, 0x280f, 0x2809, 0x2804,
		0x100a, 0x100a, 0x100a, 0x100a, 0x100a, 0x100a, 0x100a, 0x100a,
		0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800,
		0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800,
		0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005,
		0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005,
		0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005,
		0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005, 0x0005,
		0x0013, 0x0013, 0x0812, 0x0811, 0x000e, 0x000d,
	},
	{
		0x9040, 0x8048, 0x2808, 0x2807, 0x2006, 0x2006, 0x2005, 0x2005,
		0x1804, 0x1804, 0x1804, 0x1804, 0x1803, 0x1803, 0x1803, 0x1803,
		0x1002, 0x1002, 0x1002, 0x1002, 0x1002, 0x1002, 0x1002, 0x1002,
		0x1001, 0x1001, 0x1001, 0x1001, 0x1001, 0x1001, 0x1001, 0x1001,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x4001, 0x100f, 0x100e, 0x100d, 0x080c, 0x080c, 0x080b, 0x080b,
		0x000a, 0x0009,
	},
	{
		0xa040, 0x2809, 0x2008, 0x2008, 0x1807, 0x1807, 0x1807, 0x1807,
		0x1006, 0x1006, 0x1006, 0x1006, 0x1006, 0x1006, 0x1006, 0x1006,
		0x1005, 0x1005, 0x1005, 0x1005, 0x1005, 0x1005, 0x1005, 0x1005,
		0x1004, 0x1004, 0x1004, 0x1004, 0x1004, 0x1004, 0x1004, 0x1004,
		0x1003, 0x1003, 0x1003, 0x1003, 0x1003, 0x1003, 0x1003, 0x1003,
		0x1002, 0x1002, 0x1002, 0x1002, 0x1002, 0x1002, 0x1002, 0x1002,
		0x1001, 0x1001, 0x1001, 0x1001, 0x1001, 0x1001, 0x1001, 0x1001,
		0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000,
		0x4001, 0x200e, 0x180d, 0x180d, 0x100c, 0x100c, 0x100c, 0x100c,
		0x080b, 0x080b, 0x080b, 0x080b, 0x080b, 0x080b, 0x080b, 0x080b,
		0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a,
		0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a, 0x000a,
	},
};

__place_k0_data__ int8_t cabac_context_init_I[460][2] = {
	/* 0 - 10 */
	{ 20, -15 }, {  2, 54 },  {  3,  74 }, { 20, -15 },
	{  2,  54 }, {  3, 74 },  { -28, 127 }, { -23, 104 },
	{ -6,  53 }, { -1, 54 },  {  7,  51 },

	/* 11 - 23 unsused for I */
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },

	/* 24- 39 */
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },

	/* 40 - 53 */
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },

	/* 54 - 59 */
	{ 0, 0 },    { 0, 0 },    { 0, 0 },      { 0, 0 },
	{ 0, 0 },    { 0, 0 },

	/* 60 - 69 */
	{ 0, 41 },   { 0, 63 },   { 0, 63 },     { 0, 63 },
	{ -9, 83 },  { 4, 86 },   { 0, 97 },     { -7, 72 },
	{ 13, 41 },  { 3, 62 },

	/* 70 -> 87 */
	{ 0, 11 },   { 1, 55 },   { 0, 69 },     { -17, 127 },
	{ -13, 102 }, { 0, 82 },   { -7, 74 },    { -21, 107 },
	{ -27, 127 }, { -31, 127 }, { -24, 127 },  { -18, 95 },
	{ -27, 127 }, { -21, 114 }, { -30, 127 },  { -17, 123 },
	{ -12, 115 }, { -16, 122 },

	/* 88 -> 104 */
	{ -11, 115 }, { -12, 63 }, { -2, 68 },    { -15, 84 },
	{ -13, 104 }, { -3, 70 },  { -8, 93 },    { -10, 90 },
	{ -30, 127 }, { -1, 74 },  { -6, 97 },    { -7, 91 },
	{ -20, 127 }, { -4, 56 },  { -5, 82 },    { -7, 76 },
	{ -22, 125 },

	/* 105 -> 135 */
	{ -7, 93 },  { -11, 87 }, { -3, 77 },    { -5, 71 },
	{ -4, 63 },  { -4, 68 },  { -12, 84 },   { -7, 62 },
	{ -7, 65 },  { 8, 61 },   { 5, 56 },     { -2, 66 },
	{ 1, 64 },   { 0, 61 },   { -2, 78 },    { 1, 50 },
	{ 7, 52 },   { 10, 35 },  { 0, 44 },     { 11, 38 },
	{ 1, 45 },   { 0, 46 },   { 5, 44 },     { 31, 17 },
	{ 1, 51 },   { 7, 50 },   { 28, 19 },    { 16, 33 },
	{ 14, 62 },  { -13, 108 }, { -15, 100 },

	/* 136 -> 165 */
	{ -13, 101 }, { -13, 91 }, { -12, 94 },   { -10, 88 },
	{ -16, 84 }, { -10, 86 }, { -7, 83 },    { -13, 87 },
	{ -19, 94 }, { 1, 70 },   { 0, 72 },     { -5, 74 },
	{ 18, 59 },  { -8, 102 }, { -15, 100 },  { 0, 95 },
	{ -4, 75 },  { 2, 72 },   { -11, 75 },   { -3, 71 },
	{ 15, 46 },  { -13, 69 }, { 0, 62 },     { 0, 65 },
	{ 21, 37 },  { -15, 72 }, { 9, 57 },     { 16, 54 },
	{ 0, 62 },   { 12, 72 },

	/* 166 -> 196 */
	{ 24, 0 },   { 15, 9 },   { 8, 25 },     { 13, 18 },
	{ 15, 9 },   { 13, 19 },  { 10, 37 },    { 12, 18 },
	{ 6, 29 },   { 20, 33 },  { 15, 30 },    { 4, 45 },
	{ 1, 58 },   { 0, 62 },   { 7, 61 },     { 12, 38 },
	{ 11, 45 },  { 15, 39 },  { 11, 42 },    { 13, 44 },
	{ 16, 45 },  { 12, 41 },  { 10, 49 },    { 30, 34 },
	{ 18, 42 },  { 10, 55 },  { 17, 51 },    { 17, 46 },
	{ 0, 89 },   { 26, -19 }, { 22, -17 },

	/* 197 -> 226 */
	{ 26, -17 }, { 30, -25 }, { 28, -20 },   { 33, -23 },
	{ 37, -27 }, { 33, -23 }, { 40, -28 },   { 38, -17 },
	{ 33, -11 }, { 40, -15 }, { 41, -6 },    { 38, 1 },
	{ 41, 17 },  { 30, -6 },  { 27, 3 },     { 26, 22 },
	{ 37, -16 }, { 35, -4 },  { 38, -8 },    { 38, -3 },
	{ 37, 3 },   { 38, 5 },   { 42, 0 },     { 35, 16 },
	{ 39, 22 },  { 14, 48 },  { 27, 37 },    { 21, 60 },
	{ 12, 68 },  { 2, 97 },

	/* 227 -> 251 */
	{ -3, 71 },  { -6, 42 },  { -5, 50 },    { -3, 54 },
	{ -2, 62 },  { 0, 58 },   { 1, 63 },     { -2, 72 },
	{ -1, 74 },  { -9, 91 },  { -5, 67 },    { -5, 27 },
	{ -3, 39 },  { -2, 44 },  { 0, 46 },     { -16, 64 },
	{ -8, 68 },  { -10, 78 }, { -6, 77 },    { -10, 86 },
	{ -12, 92 }, { -15, 55 }, { -10, 60 },   { -6, 62 },
	{ -4, 65 },

	/* 252 -> 275 */
	{ -12, 73 }, { -8, 76 },  { -7, 80 },    { -9, 88 },
	{ -17, 110 }, { -11, 97 }, { -20, 84 },   { -11, 79 },
	{ -6, 73 },  { -4, 74 },  { -13, 86 },   { -13, 96 },
	{ -11, 97 }, { -19, 117 }, { -8, 78 },    { -5, 33 },
	{ -4, 48 },  { -2, 53 },  { -3, 62 },    { -13, 71 },
	{ -10, 79 }, { -12, 86 }, { -13, 90 },   { -14, 97 },

	/* 276 a bit special (not used, bypass is used instead) */
	{ 0, 0 },

	/* 277 -> 307 */
	{ -6, 93 },  { -6, 84 },  { -8, 79 },    { 0, 66 },
	{ -1, 71 },  { 0, 62 },   { -2, 60 },    { -2, 59 },
	{ -5, 75 },  { -3, 62 },  { -4, 58 },    { -9, 66 },
	{ -1, 79 },  { 0, 71 },   { 3, 68 },     { 10, 44 },
	{ -7, 62 },  { 15, 36 },  { 14, 40 },    { 16, 27 },
	{ 12, 29 },  { 1, 44 },   { 20, 36 },    { 18, 32 },
	{ 5, 42 },   { 1, 48 },   { 10, 62 },    { 17, 46 },
	{ 9, 64 },   { -12, 104 }, { -11, 97 },

	/* 308 -> 337 */
	{ -16, 96 }, { -7, 88 },  { -8, 85 },    { -7, 85 },
	{ -9, 85 },  { -13, 88 }, { 4, 66 },     { -3, 77 },
	{ -3, 76 },  { -6, 76 },  { 10, 58 },    { -1, 76 },
	{ -1, 83 },  { -7, 99 },  { -14, 95 },   { 2, 95 },
	{ 0, 76 },   { -5, 74 },  { 0, 70 },     { -11, 75 },
	{ 1, 68 },   { 0, 65 },   { -14, 73 },   { 3, 62 },
	{ 4, 62 },   { -1, 68 },  { -13, 75 },   { 11, 55 },
	{ 5, 64 },   { 12, 70 },

	/* 338 -> 368 */
	{ 15, 6 },   { 6, 19 },   { 7, 16 },     { 12, 14 },
	{ 18, 13 },  { 13, 11 },  { 13, 15 },    { 15, 16 },
	{ 12, 23 },  { 13, 23 },  { 15, 20 },    { 14, 26 },
	{ 14, 44 },  { 17, 40 },  { 17, 47 },    { 24, 17 },
	{ 21, 21 },  { 25, 22 },  { 31, 27 },    { 22, 29 },
	{ 19, 35 },  { 14, 50 },  { 10, 57 },    { 7, 63 },
	{ -2, 77 },  { -4, 82 },  { -3, 94 },    { 9, 69 },
	{ -12, 109 }, { 36, -35 }, { 36, -34 },

	/* 369 -> 398 */
	{ 32, -26 }, { 37, -30 }, { 44, -32 },   { 34, -18 },
	{ 34, -15 }, { 40, -15 }, { 33, -7 },    { 35, -5 },
	{ 33, 0 },   { 38, 2 },   { 33, 13 },    { 23, 35 },
	{ 13, 58 },  { 29, -3 },  { 26, 0 },     { 22, 30 },
	{ 31, -7 },  { 35, -15 }, { 34, -3 },    { 34, 3 },
	{ 36, -1 },  { 34, 5 },   { 32, 11 },    { 35, 5 },
	{ 34, 12 },  { 39, 11 },  { 30, 29 },    { 34, 26 },
	{ 29, 39 },  { 19, 66 },

	/* 399 -> 435 */
	{  31,  21 }, {  31,  31 }, {  25,  50 },
	{ -17, 120 }, { -20, 112 }, { -18, 114 }, { -11,  85 },
	{ -15,  92 }, { -14,  89 }, { -26,  71 }, { -15,  81 },
	{ -14,  80 }, {   0,  68 }, { -14,  70 }, { -24,  56 },
	{ -23,  68 }, { -24,  50 }, { -11,  74 }, {  23, -13 },
	{  26, -13 }, {  40, -15 }, {  49, -14 }, {  44,   3 },
	{  45,   6 }, {  44,  34 }, {  33,  54 }, {  19,  82 },
	{  -3,  75 }, {  -1,  23 }, {   1,  34 }, {   1,  43 },
	{   0,  54 }, {  -2,  55 }, {   0,  61 }, {   1,  64 },
	{   0,  68 }, {  -9,  92 },

	/* 436 -> 459 */
	{ -14, 106 }, { -13,  97 }, { -15,  90 }, { -12,  90 },
	{ -18,  88 }, { -10,  73 }, {  -9,  79 }, { -14,  86 },
	{ -10,  73 }, { -10,  70 }, { -10,  69 }, {  -5,  66 },
	{  -9,  64 }, {  -5,  58 }, {   2,  59 }, {  21, -10 },
	{  24, -11 }, {  28,  -8 }, {  28,  -1 }, {  29,   3 },
	{  29,   9 }, {  35,  20 }, {  29,  36 }, {  14,  67 }
};
__place_k0_data__ int8_t cabac_context_init_PB[3][460][2] = {
	/* i_cabac_init_idc == 0 */
	{
		/* 0 - 10 */
		{  20, -15 }, {   2,  54 }, {   3,  74 }, {  20, -15 },
		{   2,  54 }, {   3,  74 }, { -28, 127 }, { -23, 104 },
		{  -6,  53 }, {  -1,  54 }, {   7,  51 },

		/* 11 - 23 */
		{  23,  33 }, {  23,   2 }, {  21,   0 }, {   1,   9 },
		{   0,  49 }, { -37, 118 }, {   5,  57 }, { -13,  78 },
		{ -11,  65 }, {   1,  62 }, {  12,  49 }, {  -4,  73 },
		{  17,  50 },

		/* 24 - 39 */
		{  18,  64 }, {   9,  43 }, {  29,   0 }, {  26,  67 },
		{  16,  90 }, {   9, 104 }, { -46, 127 }, { -20, 104 },
		{   1,  67 }, { -13,  78 }, { -11,  65 }, {   1,  62 },
		{  -6,  86 }, { -17,  95 }, {  -6,  61 }, {   9,  45 },

		/* 40 - 53 */
		{  -3,  69 }, {  -6,  81 }, { -11,  96 }, {   6,  55 },
		{   7,  67 }, {  -5,  86 }, {   2,  88 }, {   0,  58 },
		{  -3,  76 }, { -10,  94 }, {   5,  54 }, {   4,  69 },
		{  -3,  81 }, {   0,  88 },

		/* 54 - 59 */
		{  -7,  67 }, {  -5,  74 }, {  -4,  74 }, {  -5,  80 },
		{  -7,  72 }, {   1,  58 },

		/* 60 - 69 */
		{   0,  41 }, {   0,  63 }, {   0,  63 }, { 0, 63 },
		{  -9,  83 }, {   4,  86 }, {   0,  97 }, { -7, 72 },
		{  13,  41 }, {   3,  62 },

		/* 70 - 87 */
		{   0,  45 }, {  -4,  78 }, {  -3,  96 }, { -27,  126 },
		{ -28,  98 }, { -25, 101 }, { -23,  67 }, { -28,  82 },
		{ -20,  94 }, { -16,  83 }, { -22, 110 }, { -21,  91 },
		{ -18, 102 }, { -13,  93 }, { -29, 127 }, {  -7,  92 },
		{  -5,  89 }, {  -7,  96 }, { -13, 108 }, {  -3,  46 },
		{  -1,  65 }, {  -1,  57 }, {  -9,  93 }, {  -3,  74 },
		{  -9,  92 }, {  -8,  87 }, { -23, 126 }, {   5,  54 },
		{   6,  60 }, {   6,  59 }, {   6,  69 }, {  -1,  48 },
		{   0,  68 }, {  -4,  69 }, {  -8,  88 },

		/* 105 -> 165 */
		{  -2,  85 }, {  -6,  78 }, {  -1,  75 }, {  -7,  77 },
		{   2,  54 }, {   5,  50 }, {  -3,  68 }, {   1,  50 },
		{   6,  42 }, {  -4,  81 }, {   1,  63 }, {  -4,  70 },
		{   0,  67 }, {   2,  57 }, {  -2,  76 }, {  11,  35 },
		{   4,  64 }, {   1,  61 }, {  11,  35 }, {  18,  25 },
		{  12,  24 }, {  13,  29 }, {  13,  36 }, { -10,  93 },
		{  -7,  73 }, {  -2,  73 }, {  13,  46 }, {   9,  49 },
		{  -7, 100 }, {   9,  53 }, {   2,  53 }, {   5,  53 },
		{  -2,  61 }, {   0,  56 }, {   0,  56 }, { -13,  63 },
		{  -5,  60 }, {  -1,  62 }, {   4,  57 }, {  -6,  69 },
		{   4,  57 }, {  14,  39 }, {   4,  51 }, {  13,  68 },
		{   3,  64 }, {   1,  61 }, {   9,  63 }, {   7,  50 },
		{  16,  39 }, {   5,  44 }, {   4,  52 }, {  11,  48 },
		{  -5,  60 }, {  -1,  59 }, {   0,  59 }, {  22,  33 },
		{   5,  44 }, {  14,  43 }, {  -1,  78 }, {   0,  60 },
		{   9,  69 },

		/* 166 - 226 */
		{  11,  28 }, {   2,  40 }, {   3,  44 }, {   0,  49 },
		{   0,  46 }, {   2,  44 }, {   2,  51 }, {   0,  47 },
		{   4,  39 }, {   2,  62 }, {   6,  46 }, {   0,  54 },
		{   3,  54 }, {   2,  58 }, {   4,  63 }, {   6,  51 },
		{   6,  57 }, {   7,  53 }, {   6,  52 }, {   6,  55 },
		{  11,  45 }, {  14,  36 }, {   8,  53 }, {  -1,  82 },
		{   7,  55 }, {  -3,  78 }, {  15,  46 }, {  22,  31 },
		{  -1,  84 }, {  25,   7 }, {  30,  -7 }, {  28,   3 },
		{  28,   4 }, {  32,   0 }, {  34,  -1 }, {  30,   6 },
		{  30,   6 }, {  32,   9 }, {  31,  19 }, {  26,  27 },
		{  26,  30 }, {  37,  20 }, {  28,  34 }, {  17,  70 },
		{   1,  67 }, {   5,  59 }, {   9,  67 }, {  16,  30 },
		{  18,  32 }, {  18,  35 }, {  22,  29 }, {  24,  31 },
		{  23,  38 }, {  18,  43 }, {  20,  41 }, {  11,  63 },
		{   9,  59 }, {   9,  64 }, {  -1,  94 }, {  -2,  89 },
		{  -9, 108 },

		/* 227 - 275 */
		{  -6,  76 }, {  -2,  44 }, {   0,  45 }, {   0,  52 },
		{  -3,  64 }, {  -2,  59 }, {  -4,  70 }, {  -4,  75 },
		{  -8,  82 }, { -17, 102 }, {  -9,  77 }, {   3,  24 },
		{   0,  42 }, {   0,  48 }, {   0,  55 }, {  -6,  59 },
		{  -7,  71 }, { -12,  83 }, { -11,  87 }, { -30, 119 },
		{   1,  58 }, {  -3,  29 }, {  -1,  36 }, {   1,  38 },
		{   2,  43 }, {  -6,  55 }, {   0,  58 }, {   0,  64 },
		{  -3,  74 }, { -10,  90 }, {   0,  70 }, {  -4,  29 },
		{   5,  31 }, {   7,  42 }, {   1,  59 }, {  -2,  58 },
		{  -3,  72 }, {  -3,  81 }, { -11,  97 }, {   0,  58 },
		{   8,   5 }, {  10,  14 }, {  14,  18 }, {  13,  27 },
		{   2,  40 }, {   0,  58 }, {  -3,  70 }, {  -6,  79 },
		{  -8,  85 },

		/* 276 a bit special (not used, bypass is used instead) */
		{ 0, 0 },

		/* 277 - 337 */
		{ -13, 106 }, { -16, 106 }, { -10,  87 }, { -21, 114 },
		{ -18, 110 }, { -14,  98 }, { -22, 110 }, { -21, 106 },
		{ -18, 103 }, { -21, 107 }, { -23, 108 }, { -26, 112 },
		{ -10,  96 }, { -12,  95 }, {  -5,  91 }, {  -9,  93 },
		{ -22,  94 }, {  -5,  86 }, {   9,  67 }, {  -4,  80 },
		{ -10,  85 }, {  -1,  70 }, {   7,  60 }, {   9,  58 },
		{   5,  61 }, {  12,  50 }, {  15,  50 }, {  18,  49 },
		{  17,  54 }, {  10,  41 }, {   7,  46 }, {  -1,  51 },
		{   7,  49 }, {   8,  52 }, {   9,  41 }, {   6,  47 },
		{   2,  55 }, {  13,  41 }, {  10,  44 }, {   6,  50 },
		{   5,  53 }, {  13,  49 }, {   4,  63 }, {   6,  64 },
		{  -2,  69 }, {  -2,  59 }, {   6,  70 }, {  10,  44 },
		{   9,  31 }, {  12,  43 }, {   3,  53 }, {  14,  34 },
		{  10,  38 }, {  -3,  52 }, {  13,  40 }, {  17,  32 },
		{   7,  44 }, {   7,  38 }, {  13,  50 }, {  10,  57 },
		{  26,  43 },

		/* 338 - 398 */
		{  14,  11 }, {  11,  14 }, {   9,  11 }, {  18,  11 },
		{  21,   9 }, {  23,  -2 }, {  32, -15 }, {  32, -15 },
		{  34, -21 }, {  39, -23 }, {  42, -33 }, {  41, -31 },
		{  46, -28 }, {  38, -12 }, {  21,  29 }, {  45, -24 },
		{  53, -45 }, {  48, -26 }, {  65, -43 }, {  43, -19 },
		{  39, -10 }, {  30,   9 }, {  18,  26 }, {  20,  27 },
		{   0,  57 }, { -14,  82 }, {  -5,  75 }, { -19,  97 },
		{ -35, 125 }, {  27,   0 }, {  28,   0 }, {  31,  -4 },
		{  27,   6 }, {  34,   8 }, {  30,  10 }, {  24,  22 },
		{  33,  19 }, {  22,  32 }, {  26,  31 }, {  21,  41 },
		{  26,  44 }, {  23,  47 }, {  16,  65 }, {  14,  71 },
		{   8,  60 }, {   6,  63 }, {  17,  65 }, {  21,  24 },
		{  23,  20 }, {  26,  23 }, {  27,  32 }, {  28,  23 },
		{  28,  24 }, {  23,  40 }, {  24,  32 }, {  28,  29 },
		{  23,  42 }, {  19,  57 }, {  22,  53 }, {  22,  61 },
		{  11,  86 },

		/* 399 - 435 */
		{  12,  40 }, {  11,  51 }, {  14,  59 },
		{  -4,  79 }, {  -7,  71 }, {  -5,  69 }, {  -9,  70 },
		{  -8,  66 }, { -10,  68 }, { -19,  73 }, { -12,  69 },
		{ -16,  70 }, { -15,  67 }, { -20,  62 }, { -19,  70 },
		{ -16,  66 }, { -22,  65 }, { -20,  63 }, {   9,  -2 },
		{  26,  -9 }, {  33,  -9 }, {  39,  -7 }, {  41,  -2 },
		{  45,   3 }, {  49,   9 }, {  45,  27 }, {  36,  59 },
		{  -6,  66 }, {  -7,  35 }, {  -7,  42 }, {  -8,  45 },
		{  -5,  48 }, { -12,  56 }, {  -6,  60 }, {  -5,  62 },
		{  -8,  66 }, {  -8,  76 },

		/* 436 - 459 */
		{  -5,  85 }, {  -6,  81 }, { -10,  77 }, {  -7,  81 },
		{ -17,  80 }, { -18,  73 }, {  -4,  74 }, { -10,  83 },
		{  -9,  71 }, {  -9,  67 }, {  -1,  61 }, {  -8,  66 },
		{ -14,  66 }, {   0,  59 }, {   2,  59 }, {  21, -13 },
		{  33, -14 }, {  39,  -7 }, {  46,  -2 }, {  51,   2 },
		{  60,   6 }, {  61,  17 }, {  55,  34 }, {  42,  62 },
	},

	/* i_cabac_init_idc == 1 */
	{
		/* 0 - 10 */
		{  20, -15 }, {   2,  54 }, {   3,  74 }, {  20, -15 },
		{   2,  54 }, {   3,  74 }, { -28, 127 }, { -23, 104 },
		{  -6,  53 }, {  -1,  54 }, {   7,  51 },

		/* 11 - 23 */
		{  22,  25 }, {  34,   0 }, {  16,   0 }, {  -2,   9 },
		{   4,  41 }, { -29, 118 }, {   2,  65 }, {  -6,  71 },
		{ -13,  79 }, {   5,  52 }, {   9,  50 }, {  -3,  70 },
		{  10,  54 },

		/* 24 - 39 */
		{  26,  34 }, {  19,  22 }, {  40,   0 }, {  57,   2 },
		{  41,  36 }, {  26,  69 }, { -45, 127 }, { -15, 101 },
		{  -4,  76 }, {  -6,  71 }, { -13,  79 }, {   5,  52 },
		{   6,  69 }, { -13,  90 }, {   0,  52 }, {   8,  43 },

		/* 40 - 53 */
		{  -2,  69 }, {  -5,  82 }, { -10,  96 }, {   2,  59 },
		{   2,  75 }, {  -3,  87 }, {  -3,  100 }, {   1,  56 },
		{  -3,  74 }, {  -6,  85 }, {   0,  59 }, {  -3,  81 },
		{  -7,  86 }, {  -5,  95 },

		/* 54 - 59 */
		{  -1,  66 }, {  -1,  77 }, {   1,  70 }, {  -2,  86 },
		{  -5,  72 }, {   0,  61 },

		/* 60 - 69 */
		{ 0, 41 },   { 0, 63 },   { 0, 63 },     { 0, 63 },
		{ -9, 83 },  { 4, 86 },   { 0, 97 },     { -7, 72 },
		{ 13, 41 },  { 3, 62 },

		/* 70 - 104 */
		{  13,  15 }, {   7,  51 }, {   2,  80 }, { -39, 127 },
		{ -18,  91 }, { -17,  96 }, { -26,  81 }, { -35,  98 },
		{ -24, 102 }, { -23,  97 }, { -27, 119 }, { -24,  99 },
		{ -21, 110 }, { -18, 102 }, { -36, 127 }, {   0,  80 },
		{  -5,  89 }, {  -7,  94 }, {  -4,  92 }, {   0,  39 },
		{   0,  65 }, { -15,  84 }, { -35, 127 }, {  -2,  73 },
		{ -12, 104 }, {  -9,  91 }, { -31, 127 }, {   3,  55 },
		{   7,  56 }, {   7,  55 }, {   8,  61 }, {  -3,  53 },
		{   0,  68 }, {  -7,  74 }, {  -9,  88 },

		/* 105 -> 165 */
		{ -13, 103 }, { -13,  91 }, {  -9,  89 }, { -14,  92 },
		{  -8,  76 }, { -12,  87 }, { -23, 110 }, { -24, 105 },
		{ -10,  78 }, { -20, 112 }, { -17,  99 }, { -78, 127 },
		{ -70, 127 }, { -50, 127 }, { -46, 127 }, {  -4,  66 },
		{  -5,  78 }, {  -4,  71 }, {  -8,  72 }, {   2,  59 },
		{  -1,  55 }, {  -7,  70 }, {  -6,  75 }, {  -8,  89 },
		{ -34, 119 }, {  -3,  75 }, {  32,  20 }, {  30,  22 },
		{ -44, 127 }, {   0,  54 }, {  -5,  61 }, {   0,  58 },
		{  -1,  60 }, {  -3,  61 }, {  -8,  67 }, { -25,  84 },
		{ -14,  74 }, {  -5,  65 }, {   5,  52 }, {   2,  57 },
		{   0,  61 }, {  -9,  69 }, { -11,  70 }, {  18,  55 },
		{  -4,  71 }, {   0,  58 }, {   7,  61 }, {   9,  41 },
		{  18,  25 }, {   9,  32 }, {   5,  43 }, {   9,  47 },
		{   0,  44 }, {   0,  51 }, {   2,  46 }, {  19,  38 },
		{  -4,  66 }, {  15,  38 }, {  12,  42 }, {   9,  34 },
		{   0,  89 },

		/* 166 - 226 */
		{   4,  45 }, {  10,  28 }, {  10,  31 }, {  33, -11 },
		{  52, -43 }, {  18,  15 }, {  28,   0 }, {  35, -22 },
		{  38, -25 }, {  34,   0 }, {  39, -18 }, {  32, -12 },
		{ 102, -94 }, {   0,   0 }, {  56, -15 }, {  33,  -4 },
		{  29,  10 }, {  37,  -5 }, {  51, -29 }, {  39,  -9 },
		{  52, -34 }, {  69, -58 }, {  67, -63 }, {  44,  -5 },
		{  32,   7 }, {  55, -29 }, {  32,   1 }, {   0,   0 },
		{  27,  36 }, {  33, -25 }, {  34, -30 }, {  36, -28 },
		{  38, -28 }, {  38, -27 }, {  34, -18 }, {  35, -16 },
		{  34, -14 }, {  32,  -8 }, {  37,  -6 }, {  35,   0 },
		{  30,  10 }, {  28,  18 }, {  26,  25 }, {  29,  41 },
		{   0,  75 }, {   2,  72 }, {   8,  77 }, {  14,  35 },
		{  18,  31 }, {  17,  35 }, {  21,  30 }, {  17,  45 },
		{  20,  42 }, {  18,  45 }, {  27,  26 }, {  16,  54 },
		{   7,  66 }, {  16,  56 }, {  11,  73 }, {  10,  67 },
		{ -10, 116 },

		/* 227 - 275 */
		{ -23, 112 }, { -15,  71 }, {  -7,  61 }, {   0,  53 },
		{  -5,  66 }, { -11,  77 }, {  -9,  80 }, {  -9,  84 },
		{ -10,  87 }, { -34, 127 }, { -21, 101 }, {  -3,  39 },
		{  -5,  53 }, {  -7,  61 }, { -11,  75 }, { -15,  77 },
		{ -17,  91 }, { -25, 107 }, { -25, 111 }, { -28, 122 },
		{ -11,  76 }, { -10,  44 }, { -10,  52 }, { -10,  57 },
		{  -9,  58 }, { -16,  72 }, {  -7,  69 }, {  -4,  69 },
		{  -5,  74 }, {  -9,  86 }, {   2,  66 }, {  -9,  34 },
		{   1,  32 }, {  11,  31 }, {   5,  52 }, {  -2,  55 },
		{  -2,  67 }, {   0,  73 }, {  -8,  89 }, {   3,  52 },
		{   7,   4 }, {  10,   8 }, {  17,   8 }, {  16,  19 },
		{   3,  37 }, {  -1,  61 }, {  -5,  73 }, {  -1,  70 },
		{  -4,  78 },

		/* 276 a bit special (not used, bypass is used instead) */
		{ 0, 0 },

		/* 277 - 337 */
		{ -21, 126 }, { -23, 124 }, { -20, 110 }, { -26, 126 },
		{ -25, 124 }, { -17, 105 }, { -27, 121 }, { -27, 117 },
		{ -17, 102 }, { -26, 117 }, { -27, 116 }, { -33, 122 },
		{ -10,  95 }, { -14, 100 }, {  -8,  95 }, { -17, 111 },
		{ -28, 114 }, {  -6,  89 }, {  -2,  80 }, {  -4,  82 },
		{  -9,  85 }, {  -8,  81 }, {  -1,  72 }, {   5,  64 },
		{   1,  67 }, {   9,  56 }, {   0,  69 }, {   1,  69 },
		{   7,  69 }, {  -7,  69 }, {  -6,  67 }, { -16,  77 },
		{  -2,  64 }, {   2,  61 }, {  -6,  67 }, {  -3,  64 },
		{   2,  57 }, {  -3,  65 }, {  -3,  66 }, {   0,  62 },
		{   9,  51 }, {  -1,  66 }, {  -2,  71 }, {  -2,  75 },
		{  -1,  70 }, {  -9,  72 }, {  14,  60 }, {  16,  37 },
		{   0,  47 }, {  18,  35 }, {  11,  37 }, {  12,  41 },
		{  10,  41 }, {   2,  48 }, {  12,  41 }, {  13,  41 },
		{   0,  59 }, {   3,  50 }, {  19,  40 }, {   3,  66 },
		{  18,  50 },

		/* 338 - 398 */
		{  19,  -6 }, {  18,  -6 }, {  14,   0 }, {  26, -12 },
		{  31, -16 }, {  33, -25 }, {  33, -22 }, {  37, -28 },
		{  39, -30 }, {  42, -30 }, {  47, -42 }, {  45, -36 },
		{  49, -34 }, {  41, -17 }, {  32,   9 }, {  69, -71 },
		{  63, -63 }, {  66, -64 }, {  77, -74 }, {  54, -39 },
		{  52, -35 }, {  41, -10 }, {  36,   0 }, {  40,  -1 },
		{  30,  14 }, {  28,  26 }, {  23,  37 }, {  12,  55 },
		{  11,  65 }, {  37, -33 }, {  39, -36 }, {  40, -37 },
		{  38, -30 }, {  46, -33 }, {  42, -30 }, {  40, -24 },
		{  49, -29 }, {  38, -12 }, {  40, -10 }, {  38,  -3 },
		{  46,  -5 }, {  31,  20 }, {  29,  30 }, {  25,  44 },
		{  12,  48 }, {  11,  49 }, {  26,  45 }, {  22,  22 },
		{  23,  22 }, {  27,  21 }, {  33,  20 }, {  26,  28 },
		{  30,  24 }, {  27,  34 }, {  18,  42 }, {  25,  39 },
		{  18,  50 }, {  12,  70 }, {  21,  54 }, {  14,  71 },
		{  11,  83 },

		/* 399 - 435 */
		{  25,  32 }, {  21,  49 }, {  21,  54 },
		{  -5,  85 }, {  -6,  81 }, { -10,  77 }, {  -7,  81 },
		{ -17,  80 }, { -18,  73 }, {  -4,  74 }, { -10,  83 },
		{  -9,  71 }, {  -9,  67 }, {  -1,  61 }, {  -8,  66 },
		{ -14,  66 }, {   0,  59 }, {   2,  59 }, {  17, -10 },
		{  32, -13 }, {  42,  -9 }, {  49,  -5 }, {  53,   0 },
		{  64,   3 }, {  68,  10 }, {  66,  27 }, {  47,  57 },
		{  -5,  71 }, {   0,  24 }, {  -1,  36 }, {  -2,  42 },
		{  -2,  52 }, {  -9,  57 }, {  -6,  63 }, {  -4,  65 },
		{  -4,  67 }, {  -7,  82 },

		/* 436 - 459 */
		{  -3,  81 }, {  -3,  76 }, {  -7,  72 }, {  -6,  78 },
		{ -12,  72 }, { -14,  68 }, {  -3,  70 }, {  -6,  76 },
		{  -5,  66 }, {  -5,  62 }, {   0,  57 }, {  -4,  61 },
		{  -9,  60 }, {   1,  54 }, {   2,  58 }, {  17, -10 },
		{  32, -13 }, {  42,  -9 }, {  49,  -5 }, {  53,   0 },
		{  64,   3 }, {  68,  10 }, {  66,  27 }, {  47,  57 },
	},

	/* i_cabac_init_idc == 2 */
	{
		/* 0 - 10 */
		{  20, -15 }, {   2,  54 }, {   3,  74 }, {  20, -15 },
		{   2,  54 }, {   3,  74 }, { -28, 127 }, { -23, 104 },
		{  -6,  53 }, {  -1,  54 }, {   7,  51 },

		/* 11 - 23 */
		{  29,  16 }, {  25,   0 }, {  14,   0 }, { -10,  51 },
		{  -3,  62 }, { -27,  99 }, {  26,  16 }, {  -4,  85 },
		{ -24, 102 }, {   5,  57 }, {   6,  57 }, { -17,  73 },
		{  14,  57 },

		/* 24 - 39 */
		{  20,  40 }, {  20,  10 }, {  29,   0 }, {  54,   0 },
		{  37,  42 }, {  12,  97 }, { -32, 127 }, { -22, 117 },
		{  -2,  74 }, {  -4,  85 }, { -24, 102 }, {   5,  57 },
		{  -6,  93 }, { -14,  88 }, {  -6,  44 }, {   4,  55 },

		/* 40 - 53 */
		{ -11,  89 }, { -15,  103 }, { -21,  116 }, {  19,  57 },
		{  20,  58 }, {   4,  84 }, {   6,  96 }, {   1,  63 },
		{  -5,  85 }, { -13,  106 }, {   5,  63 }, {   6,  75 },
		{  -3,  90 }, {  -1,  101 },

		/* 54 - 59 */
		{   3,  55 }, {  -4,  79 }, {  -2,  75 }, { -12,  97 },
		{  -7,  50 }, {   1,  60 },

		/* 60 - 69 */
		{ 0, 41 },   { 0, 63 },   { 0, 63 },     { 0, 63 },
		{ -9, 83 },  { 4, 86 },   { 0, 97 },     { -7, 72 },
		{ 13, 41 },  { 3, 62 },

		/* 70 - 104 */
		{   7,  34 }, {  -9,  88 }, { -20, 127 }, { -36, 127 },
		{ -17,  91 }, { -14,  95 }, { -25,  84 }, { -25,  86 },
		{ -12,  89 }, { -17,  91 }, { -31, 127 }, { -14,  76 },
		{ -18, 103 }, { -13,  90 }, { -37, 127 }, {  11,  80 },
		{   5,  76 }, {   2,  84 }, {   5,  78 }, {  -6,  55 },
		{   4,  61 }, { -14,  83 }, { -37, 127 }, {  -5,  79 },
		{ -11, 104 }, { -11,  91 }, { -30, 127 }, {   0,  65 },
		{  -2,  79 }, {   0,  72 }, {  -4,  92 }, {  -6,  56 },
		{   3,  68 }, {  -8,  71 }, { -13,  98 },

		/* 105 -> 165 */
		{  -4,  86 }, { -12,  88 }, {  -5,  82 }, {  -3,  72 },
		{  -4,  67 }, {  -8,  72 }, { -16,  89 }, {  -9,  69 },
		{  -1,  59 }, {   5,  66 }, {   4,  57 }, {  -4,  71 },
		{  -2,  71 }, {   2,  58 }, {  -1,  74 }, {  -4,  44 },
		{  -1,  69 }, {   0,  62 }, {  -7,  51 }, {  -4,  47 },
		{  -6,  42 }, {  -3,  41 }, {  -6,  53 }, {   8,  76 },
		{  -9,  78 }, { -11,  83 }, {   9,  52 }, {   0,  67 },
		{  -5,  90 }, {   1,  67 }, { -15,  72 }, {  -5,  75 },
		{  -8,  80 }, { -21,  83 }, { -21,  64 }, { -13,  31 },
		{ -25,  64 }, { -29,  94 }, {   9,  75 }, {  17,  63 },
		{  -8,  74 }, {  -5,  35 }, {  -2,  27 }, {  13,  91 },
		{   3,  65 }, {  -7,  69 }, {   8,  77 }, { -10,  66 },
		{   3,  62 }, {  -3,  68 }, { -20,  81 }, {   0,  30 },
		{   1,   7 }, {  -3,  23 }, { -21,  74 }, {  16,  66 },
		{ -23, 124 }, {  17,  37 }, {  44, -18 }, {  50, -34 },
		{ -22, 127 },

		/* 166 - 226 */
		{   4,  39 }, {   0,  42 }, {   7,  34 }, {  11,  29 },
		{   8,  31 }, {   6,  37 }, {   7,  42 }, {   3,  40 },
		{   8,  33 }, {  13,  43 }, {  13,  36 }, {   4,  47 },
		{   3,  55 }, {   2,  58 }, {   6,  60 }, {   8,  44 },
		{  11,  44 }, {  14,  42 }, {   7,  48 }, {   4,  56 },
		{   4,  52 }, {  13,  37 }, {   9,  49 }, {  19,  58 },
		{  10,  48 }, {  12,  45 }, {   0,  69 }, {  20,  33 },
		{   8,  63 }, {  35, -18 }, {  33, -25 }, {  28,  -3 },
		{  24,  10 }, {  27,   0 }, {  34, -14 }, {  52, -44 },
		{  39, -24 }, {  19,  17 }, {  31,  25 }, {  36,  29 },
		{  24,  33 }, {  34,  15 }, {  30,  20 }, {  22,  73 },
		{  20,  34 }, {  19,  31 }, {  27,  44 }, {  19,  16 },
		{  15,  36 }, {  15,  36 }, {  21,  28 }, {  25,  21 },
		{  30,  20 }, {  31,  12 }, {  27,  16 }, {  24,  42 },
		{   0,  93 }, {  14,  56 }, {  15,  57 }, {  26,  38 },
		{ -24, 127 },

		/* 227 - 275 */
		{ -24, 115 }, { -22,  82 }, {  -9,  62 }, {   0,  53 },
		{   0,  59 }, { -14,  85 }, { -13,  89 }, { -13,  94 },
		{ -11,  92 }, { -29, 127 }, { -21, 100 }, { -14,  57 },
		{ -12,  67 }, { -11,  71 }, { -10,  77 }, { -21,  85 },
		{ -16,  88 }, { -23, 104 }, { -15,  98 }, { -37, 127 },
		{ -10,  82 }, {  -8,  48 }, {  -8,  61 }, {  -8,  66 },
		{  -7,  70 }, { -14,  75 }, { -10,  79 }, {  -9,  83 },
		{ -12,  92 }, { -18, 108 }, {  -4,  79 }, { -22,  69 },
		{ -16,  75 }, {  -2,  58 }, {   1,  58 }, { -13,  78 },
		{  -9,  83 }, {  -4,  81 }, { -13,  99 }, { -13,  81 },
		{  -6,  38 }, { -13,  62 }, {  -6,  58 }, {  -2,  59 },
		{ -16,  73 }, { -10,  76 }, { -13,  86 }, {  -9,  83 },
		{ -10,  87 },

		/* 276 a bit special (not used, bypass is used instead) */
		{ 0, 0 },

		/* 277 - 337 */
		{ -22, 127 }, { -25, 127 }, { -25, 120 }, { -27, 127 },
		{ -19, 114 }, { -23, 117 }, { -25, 118 }, { -26, 117 },
		{ -24, 113 }, { -28, 118 }, { -31, 120 }, { -37, 124 },
		{ -10,  94 }, { -15, 102 }, { -10,  99 }, { -13, 106 },
		{ -50, 127 }, {  -5,  92 }, {  17,  57 }, {  -5,  86 },
		{ -13,  94 }, { -12,  91 }, {  -2,  77 }, {   0,  71 },
		{  -1,  73 }, {   4,  64 }, {  -7,  81 }, {   5,  64 },
		{  15,  57 }, {   1,  67 }, {   0,  68 }, { -10,  67 },
		{   1,  68 }, {   0,  77 }, {   2,  64 }, {   0,  68 },
		{  -5,  78 }, {   7,  55 }, {   5,  59 }, {   2,  65 },
		{  14,  54 }, {  15,  44 }, {   5,  60 }, {   2,  70 },
		{  -2,  76 }, { -18,  86 }, {  12,  70 }, {   5,  64 },
		{ -12,  70 }, {  11,  55 }, {   5,  56 }, {   0,  69 },
		{   2,  65 }, {  -6,  74 }, {   5,  54 }, {   7,  54 },
		{  -6,  76 }, { -11,  82 }, {  -2,  77 }, {  -2,  77 },
		{  25,  42 },

		/* 338 - 398 */
		{  17, -13 }, {  16,  -9 }, {  17, -12 }, {  27, -21 },
		{  37, -30 }, {  41, -40 }, {  42, -41 }, {  48, -47 },
		{  39, -32 }, {  46, -40 }, {  52, -51 }, {  46, -41 },
		{  52, -39 }, {  43, -19 }, {  32,  11 }, {  61, -55 },
		{  56, -46 }, {  62, -50 }, {  81, -67 }, {  45, -20 },
		{  35,  -2 }, {  28,  15 }, {  34,   1 }, {  39,   1 },
		{  30,  17 }, {  20,  38 }, {  18,  45 }, {  15,  54 },
		{   0,  79 }, {  36, -16 }, {  37, -14 }, {  37, -17 },
		{  32,   1 }, {  34,  15 }, {  29,  15 }, {  24,  25 },
		{  34,  22 }, {  31,  16 }, {  35,  18 }, {  31,  28 },
		{  33,  41 }, {  36,  28 }, {  27,  47 }, {  21,  62 },
		{  18,  31 }, {  19,  26 }, {  36,  24 }, {  24,  23 },
		{  27,  16 }, {  24,  30 }, {  31,  29 }, {  22,  41 },
		{  22,  42 }, {  16,  60 }, {  15,  52 }, {  14,  60 },
		{   3,  78 }, { -16, 123 }, {  21,  53 }, {  22,  56 },
		{  25,  61 },

		/* 399 - 435 */
		{  21,  33 }, {  19,  50 }, {  17,  61 },
		{  -3,  78 }, {  -8,  74 }, {  -9,  72 }, { -10,  72 },
		{ -18,  75 }, { -12,  71 }, { -11,  63 }, {  -5,  70 },
		{ -17,  75 }, { -14,  72 }, { -16,  67 }, {  -8,  53 },
		{ -14,  59 }, {  -9,  52 }, { -11,  68 }, {   9,  -2 },
		{  30, -10 }, {  31,  -4 }, {  33,  -1 }, {  33,   7 },
		{  31,  12 }, {  37,  23 }, {  31,  38 }, {  20,  64 },
		{  -9,  71 }, {  -7,  37 }, {  -8,  44 }, { -11,  49 },
		{ -10,  56 }, { -12,  59 }, {  -8,  63 }, {  -9,  67 },
		{  -6,  68 }, { -10,  79 },

		/* 436 - 459 */
		{  -3,  78 }, {  -8,  74 }, {  -9,  72 }, { -10,  72 },
		{ -18,  75 }, { -12,  71 }, { -11,  63 }, {  -5,  70 },
		{ -17,  75 }, { -14,  72 }, { -16,  67 }, {  -8,  53 },
		{ -14,  59 }, {  -9,  52 }, { -11,  68 }, {   9,  -2 },
		{  30, -10 }, {  31,  -4 }, {  33,  -1 }, {  33,   7 },
		{  31,  12 }, {  37,  23 }, {  31,  38 }, {  20,  64 },
	}
};



static inline int jzm_clip2(int a, int min, int max)
{
	if (a < min) {
		return min;
	} else if (a > max) {
		return max;
	} else {
		return a;
	}
}

void jzm_h264_slice_init_vdma(struct JZM_H264 *st_h264)
{
	int i, j;
	unsigned int *tbl_ptr;
	unsigned int qt_ram_addr;
	volatile unsigned int *chn = (volatile unsigned int *)st_h264->des_va;

	/*------------------------------------------------------
	  scheduler
	  ------------------------------------------------------*/
	GEN_VDMA_ACFG(chn, REG_SCH_SCHC, 0, 0x0);
	if (st_h264->slice_type == JZM_H264_I_TYPE) {
		GEN_VDMA_ACFG(chn, REG_SCH_BND, 0, SCH_CH3_HID(HID_DBLK) | SCH_CH2_HID(HID_VMAU) | SCH_DEPTH(DESP_FIFO_WIDTH));
		GEN_VDMA_ACFG(chn, REG_SCH_SCHG0, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHE1, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHE2, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHE3, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHE4, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHC, 0, SCH_CH2_PE | SCH_CH3_PE);
		GEN_VDMA_ACFG(chn, REG_SCH_BND, 0, SCH_CH3_HID(HID_DBLK) | SCH_CH2_HID(HID_VMAU) |
		              SCH_DEPTH(DESP_FIFO_WIDTH) | SCH_BND_G0F2 | SCH_BND_G0F3);
	} else {
		GEN_VDMA_ACFG(chn, REG_SCH_BND, 0, SCH_CH3_HID(HID_DBLK) | SCH_CH2_HID(HID_VMAU) | SCH_CH1_HID(HID_MCE) |
		              SCH_DEPTH(DESP_FIFO_WIDTH));
		GEN_VDMA_ACFG(chn, REG_SCH_SCHG0, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHE1, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHE2, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHE3, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHE4, 0, 0x0);
		GEN_VDMA_ACFG(chn, REG_SCH_SCHC, 0, SCH_CH1_PE | SCH_CH2_PE | SCH_CH3_PE);
		GEN_VDMA_ACFG(chn, REG_SCH_BND, 0, SCH_CH3_HID(HID_DBLK) | SCH_CH2_HID(HID_VMAU) | SCH_CH1_HID(HID_MCE) |
		              SCH_DEPTH(DESP_FIFO_WIDTH) | SCH_BND_G0F1 | SCH_BND_G0F2 | SCH_BND_G0F3);
	}

	/*------------------------------------------------------
	  vmau
	  ------------------------------------------------------*/
	GEN_VDMA_ACFG(chn, REG_VMAU_GBL_RUN, 0, VMAU_RESET);
	GEN_VDMA_ACFG(chn, REG_VMAU_GBL_CTR, 0, 0);
	GEN_VDMA_ACFG(chn, REG_VMAU_VIDEO_TYPE, 0, VMAU_FMT_H264);
	GEN_VDMA_ACFG(chn, REG_VMAU_NCCHN_ADDR, 0, VMAU_DESP_ADDR);
	GEN_VDMA_ACFG(chn, REG_VMAU_DEC_DONE, 0, VPU_BASE + REG_SCH_SCHE2);
	GEN_VDMA_ACFG(chn, REG_VMAU_Y_GS, 0, st_h264->mb_width * 16);
	GEN_VDMA_ACFG(chn, REG_VMAU_GBL_CTR, 0, (VMAU_CTRL_FIFO_M | VMAU_CTRL_TO_DBLK));
	GEN_VDMA_ACFG(chn, REG_VMAU_POS, 0, ((st_h264->start_mb_x & 0xff) | ((st_h264->start_mb_y & 0xff) << 16)));

	qt_ram_addr = REG_VMAU_QT;

	/* scaling_matrix8 Intra Y */
	tbl_ptr = (unsigned int *)&st_h264->scaling_matrix8[0][0];
	for (i = 0 ; i < 16; i++) {
		GEN_VDMA_ACFG(chn, qt_ram_addr, 0, tbl_ptr[i]);
		qt_ram_addr += 4;
	}

	/* scaling_matrix8 Inter Y */
	tbl_ptr = (unsigned int *)&st_h264->scaling_matrix8[3][0];
	for (i = 0 ; i < 16; i++) {
		GEN_VDMA_ACFG(chn, qt_ram_addr, 0, tbl_ptr[i]);
		qt_ram_addr += 4;
	}

	/* scaling_matrix4 Intra Y */
	tbl_ptr = (unsigned int *)&st_h264->scaling_matrix4[0][0];
	for (i = 0 ; i < 4; i++) {
		GEN_VDMA_ACFG(chn, qt_ram_addr, 0, tbl_ptr[i]);
		qt_ram_addr += 4;
	}

	/* scaling_matrix4 Inter Y */
	tbl_ptr = (unsigned int *)&st_h264->scaling_matrix4[3][0];
	for (i = 0 ; i < 4; i++) {
		GEN_VDMA_ACFG(chn, qt_ram_addr, 0, tbl_ptr[i]);
		qt_ram_addr += 4;
	}

	/* scaling_matrix4 Intra Cr */
	tbl_ptr = (unsigned int *)&st_h264->scaling_matrix4[1][0];
	for (i = 0 ; i < 4; i++) {
		GEN_VDMA_ACFG(chn, qt_ram_addr, 0, tbl_ptr[i]);
		qt_ram_addr += 4;
	}

	/* scaling_matrix4 Inter Cr */
	tbl_ptr = (unsigned int *)&st_h264->scaling_matrix4[4][0];
	for (i = 0 ; i < 4; i++) {
		GEN_VDMA_ACFG(chn, qt_ram_addr, 0, tbl_ptr[i]);
		qt_ram_addr += 4;
	}

	/* scaling_matrix4 Intra Cb */
	tbl_ptr = (unsigned int *)&st_h264->scaling_matrix4[2][0];
	for (i = 0 ; i < 4; i++) {
		GEN_VDMA_ACFG(chn, qt_ram_addr, 0, tbl_ptr[i]);
		qt_ram_addr += 4;
	}

	/* scaling_matrix4 Inter Cb */
	tbl_ptr = (unsigned int *)&st_h264->scaling_matrix4[5][0];
	for (i = 0 ; i < 4; i++) {
		GEN_VDMA_ACFG(chn, qt_ram_addr, 0, tbl_ptr[i]);
		qt_ram_addr += 4;
	}

	/*------------------------------------------------------
	  dblk
	  ------------------------------------------------------*/
	GEN_VDMA_ACFG(chn, REG_DBLK_TRIG, 0, DBLK_RESET);
	GEN_VDMA_ACFG(chn, REG_DBLK_DHA, 0, DBLK_DESP_ADDR);
	GEN_VDMA_ACFG(chn, REG_DBLK_GENDA, 0, VPU_BASE + REG_SCH_SCHE3);
	GEN_VDMA_ACFG(chn, REG_DBLK_GSIZE, 0, st_h264->mb_width | (st_h264->mb_height << 16));
	int normal_first_slice = (st_h264->start_mb_x == 0) && (st_h264->start_mb_y == 0);
	GEN_VDMA_ACFG(chn, REG_DBLK_GPOS, 0, ((st_h264->start_mb_x & 0x3ff) |
	                                      ((st_h264->start_mb_y & 0x3ff) << 16) |
	                                      ((!normal_first_slice) << 31))
	             );
	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_YA, 0, st_h264->dec_result_y);
	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_CA, 0, st_h264->dec_result_uv);
	GEN_VDMA_ACFG(chn, REG_DBLK_GP_ENDA, 0, DBLK_GP_ENDF_BASE);
#define DEBLK_VTR_FMT_I 0
#define DEBLK_VTR_FMT_P (1<<3)
#define DEBLK_VTR_FMT_B (2<<3)
#define DEBLK_VTR_BETA_SFT (16)
#define DEBLK_VTR_BETA_MSK (0xff)
#define DEBLK_VTR_ALPHA_SFT (24)
#define DEBLK_VTR_ALPHA_MSK (0xff)
	unsigned int h264_vtr = ((st_h264->slice_type == JZM_H264_I_TYPE) ? DEBLK_VTR_FMT_I :
	                         ((st_h264->slice_type == JZM_H264_P_TYPE) ? DEBLK_VTR_FMT_P : DEBLK_VTR_FMT_B)
	                        ) | DBLK_FMT_H264;
	h264_vtr = h264_vtr | ((st_h264->slice_beta_offset & DEBLK_VTR_BETA_MSK) << DEBLK_VTR_BETA_SFT)
	           | ((st_h264->slice_alpha_c0_offset & DEBLK_VTR_ALPHA_MSK) << DEBLK_VTR_ALPHA_SFT);
	GEN_VDMA_ACFG(chn, REG_DBLK_VTR, 0, h264_vtr);
	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_STR, 0, (st_h264->mb_width * 256) | (st_h264->mb_width * 128) << 16);

	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_Y1A, 0, st_h264->dec_result1_y);
	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_C1A, 0, st_h264->dec_result1_uv);
	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_STR1, 0, (st_h264->frm_y_stride) | (st_h264->frm_c_stride) << 16);
	GEN_VDMA_ACFG(chn, REG_DBLK_CTRL, 0, 0x1 | (st_h264->new_odma_flag) << 6 | (st_h264->new_odma_format) << 7 | (0 << 8) | (1 << 9));

	GEN_VDMA_ACFG(chn, REG_DBLK_TRIG, 0, DBLK_SLICE_RUN);
	//GEN_VDMA_ACFG(chn, REG_DBLK_CTRL, 0, 0x1);
	//write_reg(DBLK_GP_ENDF_BASE, -1);

	/*------------------------------------------------------
	  motion
	  ------------------------------------------------------*/
	//video_motion_init(H264_QPEL, H264_EPEL);
	int intpid  = H264_QPEL;
	int cintpid = H264_EPEL;
	for (i = 0; i < 16; i++) {
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_ILUT + i * 8, 0,
		              MCE_CH1_IINFO(IntpFMT[intpid][i].intp[0],/*intp1*/
		                            IntpFMT[intpid][i].tap,/*tap*/
		                            IntpFMT[intpid][i].intp_pkg[0],/*intp1_pkg*/
		                            IntpFMT[intpid][i].hldgl,/*hldgl*/
		                            IntpFMT[intpid][i].avsdgl,/*avsdgl*/
		                            IntpFMT[intpid][i].intp_dir[0],/*intp0_dir*/
		                            IntpFMT[intpid][i].intp_rnd[0],/*intp0_rnd*/
		                            IntpFMT[intpid][i].intp_sft[0],/*intp0_sft*/
		                            IntpFMT[intpid][i].intp_sintp[0],/*sintp0*/
		                            IntpFMT[intpid][i].intp_srnd[0],/*sintp0_rnd*/
		                            IntpFMT[intpid][i].intp_sbias[0]/*sintp0_bias*/
		                           ));
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_ILUT + i * 8 + 4, 0,
		              MCE_CH1_IINFO(IntpFMT[intpid][i].intp[1],/*intp1*/
		                            0,/*tap*/
		                            IntpFMT[intpid][i].intp_pkg[1],/*intp1_pkg*/
		                            IntpFMT[intpid][i].hldgl,/*hldgl*/
		                            IntpFMT[intpid][i].avsdgl,/*avsdgl*/
		                            IntpFMT[intpid][i].intp_dir[1],/*intp1_dir*/
		                            IntpFMT[intpid][i].intp_rnd[1],/*intp1_rnd*/
		                            IntpFMT[intpid][i].intp_sft[1],/*intp1_sft*/
		                            IntpFMT[intpid][i].intp_sintp[1],/*sintp1*/
		                            IntpFMT[intpid][i].intp_srnd[1],/*sintp1_rnd*/
		                            IntpFMT[intpid][i].intp_sbias[1]/*sintp1_bias*/
		                           ));
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_CLUT + i * 8 + 4, 0,
		              MCE_RLUT_WT(IntpFMT[intpid][i].intp_coef[0][7],/*coef8*/
		                          IntpFMT[intpid][i].intp_coef[0][6],/*coef7*/
		                          IntpFMT[intpid][i].intp_coef[0][5],/*coef6*/
		                          IntpFMT[intpid][i].intp_coef[0][4]/*coef5*/
		                         ));
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_CLUT + i * 8, 0,
		              MCE_RLUT_WT(IntpFMT[intpid][i].intp_coef[0][3],/*coef8*/
		                          IntpFMT[intpid][i].intp_coef[0][2],/*coef7*/
		                          IntpFMT[intpid][i].intp_coef[0][1],/*coef6*/
		                          IntpFMT[intpid][i].intp_coef[0][0]/*coef5*/
		                         ));
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_CLUT + (i + 16) * 8 + 4, 0,
		              MCE_RLUT_WT(IntpFMT[intpid][i].intp_coef[1][7],/*coef8*/
		                          IntpFMT[intpid][i].intp_coef[1][6],/*coef7*/
		                          IntpFMT[intpid][i].intp_coef[1][5],/*coef6*/
		                          IntpFMT[intpid][i].intp_coef[1][4]/*coef5*/
		                         ));
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_CLUT + (i + 16) * 8, 0,
		              MCE_RLUT_WT(IntpFMT[intpid][i].intp_coef[1][3],/*coef8*/
		                          IntpFMT[intpid][i].intp_coef[1][2],/*coef7*/
		                          IntpFMT[intpid][i].intp_coef[1][1],/*coef6*/
		                          IntpFMT[intpid][i].intp_coef[1][0]/*coef5*/
		                         ));
		GEN_VDMA_ACFG(chn, REG_MCE_CH2_ILUT + i * 8, 0,
		              MCE_CH2_IINFO(IntpFMT[cintpid][i].intp[0],
		                            IntpFMT[cintpid][i].intp_dir[0],
		                            IntpFMT[cintpid][i].intp_sft[0],
		                            IntpFMT[cintpid][i].intp_coef[0][0],
		                            IntpFMT[cintpid][i].intp_coef[0][1],
		                            IntpFMT[cintpid][i].intp_rnd[0]));
		GEN_VDMA_ACFG(chn, REG_MCE_CH2_ILUT + i * 8 + 4, 0,
		              MCE_CH2_IINFO(IntpFMT[cintpid][i].intp[1],
		                            IntpFMT[cintpid][i].intp_dir[1],
		                            IntpFMT[cintpid][i].intp_sft[1],
		                            IntpFMT[cintpid][i].intp_coef[1][0],
		                            IntpFMT[cintpid][i].intp_coef[1][1],
		                            IntpFMT[cintpid][i].intp_rnd[1]));
	}
#define STAT_PFE_SFT        2
#define STAT_PFE_MSK        0x1
#define STAT_LKE_SFT        1
#define STAT_LKE_MSK        0x1
#define STAT_TKE_SFT        0
#define STAT_TKE_MSK        0x1
#define MCE_PRI             (0x3 << 9)
	GEN_VDMA_ACFG(chn, REG_MCE_CH1_STAT, 0, ((1 & STAT_PFE_MSK) << STAT_PFE_SFT) |
	              ((1 & STAT_LKE_MSK) << STAT_LKE_SFT) |
	              ((1 & STAT_TKE_MSK) << STAT_TKE_SFT));
	GEN_VDMA_ACFG(chn, REG_MCE_CH2_STAT, 0, ((1 & STAT_PFE_MSK) << STAT_PFE_SFT) |
	              ((1 & STAT_LKE_MSK) << STAT_LKE_SFT) |
	              ((1 & STAT_TKE_MSK) << STAT_TKE_SFT));
	GEN_VDMA_ACFG(chn, (REG_MCE_CTRL), 0, MCE_COMP_AUTO_EXPD | MCE_CH2_EN | MCE_CLKG_EN | MCE_PRI |
	              MCE_OFA_EN | MCE_CACHE_FLUSH | MCE_EN);
	GEN_VDMA_ACFG(chn, (REG_MCE_CH1_BINFO), 0, MCE_BINFO(AryFMT[intpid], 0, 0, 0, SubPel[intpid] - 1));
	GEN_VDMA_ACFG(chn, (REG_MCE_CH2_BINFO), 0, MCE_BINFO(0, 0, 0, 0, 0));

	GEN_VDMA_ACFG(chn, (REG_MCE_CH1_PINFO), 0, 0);
	GEN_VDMA_ACFG(chn, (REG_MCE_CH2_PINFO), 0, 0);

	for (i = 0; i < 16; i++) {
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_RLUT + (i) * 8, 0, MCE_RLUT_WT(0, 0, st_h264->luma_weight[0][i], st_h264->luma_offset[0][i]));
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_RLUT + (i) * 8 + 4, 0, st_h264->mc_ref_y[0][i]);
		GEN_VDMA_ACFG(chn, REG_MCE_CH2_RLUT + (i) * 8, 0, MCE_RLUT_WT(st_h264->chroma_weight[0][i][1], st_h264->chroma_offset[0][i][1],
		              st_h264->chroma_weight[0][i][0], st_h264->chroma_offset[0][i][0]));
		GEN_VDMA_ACFG(chn, REG_MCE_CH2_RLUT + (i) * 8 + 4, 0, st_h264->mc_ref_c[0][i]);
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_RLUT + (16 + i) * 8, 0, MCE_RLUT_WT(0, 0, st_h264->luma_weight[1][i], st_h264->luma_offset[1][i]));
		GEN_VDMA_ACFG(chn, REG_MCE_CH1_RLUT + (16 + i) * 8 + 4, 0, st_h264->mc_ref_y[1][i]);
		GEN_VDMA_ACFG(chn, REG_MCE_CH2_RLUT + (16 + i) * 8, 0, MCE_RLUT_WT(st_h264->chroma_weight[1][i][1], st_h264->chroma_offset[1][i][1],
		              st_h264->chroma_weight[1][i][0], st_h264->chroma_offset[1][i][0]));
		GEN_VDMA_ACFG(chn, REG_MCE_CH2_RLUT + (16 + i) * 8 + 4, 0, st_h264->mc_ref_c[1][i]);
	}

	for (j = 0; j < 16; j++) {
		for (i = 0; i < 16; i += 4) {
			GEN_VDMA_ACFG(chn, (MOTION_IWTA_BASE + j * 16 + i), 0, *((int *)(&st_h264->implicit_weight[j][i])));
		}
	}
	GEN_VDMA_ACFG(chn, REG_MCE_IWTA, 0, MOTION_IWTA_BASE);

	GEN_VDMA_ACFG(chn, (REG_MCE_CH1_WINFO), 0, MCE_WINFO(0, (st_h264->use_weight == IS_WT1),
	              st_h264->use_weight, 1, st_h264->luma_log2_weight_denom, 5, 0, 0));
	GEN_VDMA_ACFG(chn, (REG_MCE_CH1_WTRND), 0, MCE_WTRND(0, 1 << 5));
	GEN_VDMA_ACFG(chn, (REG_MCE_CH2_WINFO1), 0, MCE_WINFO(0, st_h264->use_weight_chroma && (st_h264->use_weight == IS_WT1),
	              st_h264->use_weight, 1, st_h264->chroma_log2_weight_denom, 5, 0, 0));
	GEN_VDMA_ACFG(chn, (REG_MCE_CH2_WINFO2), 0, MCE_WINFO(0, 0, 0, 0, 0, 5, 0, 0));
	GEN_VDMA_ACFG(chn, (REG_MCE_CH2_WTRND), 0, MCE_WTRND(1 << 5, 1 << 5));

	GEN_VDMA_ACFG(chn, REG_MCE_CH1_STRD, 0, MCE_STRD(st_h264->mb_width * 16, 0, DOUT_Y_STRD));
	GEN_VDMA_ACFG(chn, REG_MCE_GEOM, 0, MCE_GEOM(st_h264->mb_height * 16, st_h264->mb_width * 16));
	GEN_VDMA_ACFG(chn, REG_MCE_CH2_STRD, 0, MCE_STRD(st_h264->mb_width * 16, 0, DOUT_C_STRD));
	GEN_VDMA_ACFG(chn, REG_MCE_DSA, 0, VPU_BASE + REG_SCH_SCHE1);
	GEN_VDMA_ACFG(chn, REG_MCE_DDC, 0, MC_DESP_ADDR);

	/*------------------------------------------------------
	  sde
	  ------------------------------------------------------*/
	// multi-slice
#if 0
	int start_mb_num = st_h264->start_mb_x + st_h264->start_mb_y * st_h264->mb_width;
	int slice_start_mx = st_h264->start_mb_x;
	int slice_start_my = st_h264->start_mb_y;
	if (st_h264->slice_num == 0) {
		for (i = 0; i < 32; i++) {
			st_h264->curr_frm_slice_start_mb[i] = INT_MAX;
		}
	}
	int ref_frm_start_mb = 0;
	if ((st_h264->slice_type == JZM_H264_B_TYPE) && (st_h264->slice_num)) {
		for (i = 0; i < 32; i++) {
			if ((start_mb_num >= st_h264->ref_frm_slice_start_mb[i]) &&
			    (start_mb_num < st_h264->ref_frm_slice_start_mb[i + 1])) {
				break;
			}
		}
		ref_frm_start_mb = st_h264->ref_frm_slice_start_mb[i];
	}
	st_h264->curr_frm_slice_start_mb[st_h264->slice_num] = start_mb_num;
#else
	int start_mb_num = st_h264->start_mb_x + st_h264->start_mb_y * st_h264->mb_width;
	int slice_start_mx = st_h264->start_mb_x;
	int slice_start_my = st_h264->start_mb_y;
	int ref_frm_start_mb = start_mb_num;
#endif

	// bs init
	unsigned int bs_addr = st_h264->bs_buffer;
	unsigned int bs_ofst = st_h264->bs_index;
#if 0
	for (i = 0; i < sde_bs_len; i++) {
		((int *)sde_bs_buffer)[i] = ((int *)bs_addr)[i];
	}
	bs_addr = sde_bs_buffer;
#endif

	GEN_VDMA_ACFG(chn, REG_SDE_STAT, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SDE_SL_GEOM, 0, SDE_SL_GEOM(st_h264->mb_height, st_h264->mb_width, slice_start_my, slice_start_mx));
	GEN_VDMA_ACFG(chn, REG_SDE_GL_CTRL, 0, 1);
	GEN_VDMA_ACFG(chn, REG_SDE_CODEC_ID, 0, (1 << 0));
	unsigned int cfg_0 = (((!st_h264->cabac) << 0) +
	                      (((st_h264->slice_type) & 0x7) << 1) +
	                      (((st_h264->field_picture) & 0x1) << 4) +
	                      (((st_h264->transform_8x8_mode) & 0x1) << 5) +
	                      (((st_h264->constrained_intra_pred) & 0x1) << 6) +
	                      (((st_h264->direct_8x8_inference_flag) & 0x1) << 7) +
	                      (((st_h264->direct_spatial_mv_pred) & 0x1) << 8) +
	                      ((1 & 0x1) << 9) + /*dir_max_word_64*/
	                      (((st_h264->x264_build > 33) & 0x1) << 10) +
	                      (((!st_h264->x264_build) & 0x1) << 11) +
	                      ((st_h264->dblk_left_en & 0x1) << 14) +
	                      ((st_h264->dblk_top_en & 0x1) << 15) +
	                      ((st_h264->ref_count_0 & 0xF) << 16) +
	                      ((st_h264->ref_count_1 & 0xF) << 20) +
	                      ((bs_ofst & 0x1F) << 24) +
	                      0);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG0, 0, cfg_0);
	unsigned int cfg_1 = (((st_h264->qscale & 0xFF) << 0) +
	                      (((st_h264->deblocking_filter) & 0x1) << 8) +
	                      0);
	//GEN_VDMA_ACFG(chn, REG_SDE_CFG1, 0, cfg_1 );
	GEN_VDMA_ACFG(chn, REG_SDE_CFG1, 0, (cfg_1 +
	                                     ((st_h264->bs_rbsp_en & 0x1) << 16)));
	GEN_VDMA_ACFG(chn, REG_SDE_CFG2, 0, bs_addr);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG3, 0, TOP_NEI_ADDR);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG4, 0, RESIDUAL_DOUT_ADDR);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG5, 0, VMAU_DESP_ADDR);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG6, 0, DBLK_DESP_ADDR);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG7, 0, DBLK_MV_ADDR);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG8, 0, MC_DESP_ADDR);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG9, 0,  st_h264->ref_frm_ctrl  + ref_frm_start_mb * 2 * 4);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG10, 0, st_h264->ref_frm_mv    + ref_frm_start_mb * 32 * 4);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG11, 0, st_h264->curr_frm_ctrl + start_mb_num * 2 * 4);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG12, 0, st_h264->curr_frm_mv   + start_mb_num * 32 * 4);
	GEN_VDMA_ACFG(chn, REG_SDE_CFG13, 0, (start_mb_num & 0xFFFF) + (((start_mb_num - ref_frm_start_mb) & 0xFFFF) << 16));
	GEN_VDMA_ACFG(chn, REG_SDE_CFG14, 0, st_h264->bs_size_in_bits);

	// ctx table init
	unsigned int *sde_table_base = (unsigned int *)REG_SDE_CTX_TBL;
	if (st_h264->cabac) {
		unsigned char state;
		for (i = 0; i < 460; i++) {
			int pre;
			if (st_h264->slice_type == JZM_H264_I_TYPE) {
				pre = jzm_clip2(((cabac_context_init_I[i][0] * st_h264->qscale) >> 4) + cabac_context_init_I[i][1], 1, 126);
			} else {
				pre = jzm_clip2(((cabac_context_init_PB[st_h264->cabac_init_idc][i][0] * st_h264->qscale) >> 4) + cabac_context_init_PB[st_h264->cabac_init_idc][i][1], 1, 126);
			}
			if (pre <= 63) {
				state = 2 * (63 - pre) + 0;
			} else {
				state = 2 * (pre - 64) + 1;
			}
			GEN_VDMA_ACFG(chn, sde_table_base + i, 0, lps_comb[state]);
		}
		GEN_VDMA_ACFG(chn, sde_table_base + 276, 0, lps_comb[126]);
	} else {
		int tbl;
		for (tbl = 0; tbl < 7; tbl++) {
			unsigned int *hw_base = sde_table_base + (sde_vlc2_sta[tbl].ram_ofst >> 1);
			unsigned int *tbl_base = (unsigned int *)&sde_vlc2_table[tbl][0];
			int size = (sde_vlc2_sta[tbl].size + 1) >> 1;
			for (i = 0; i < size; i++) {
				GEN_VDMA_ACFG(chn, hw_base + i, 0, tbl_base[i]);
			}
		}
	}
	// direct prediction scal table
	for (i = 0; i < 16; i++) {
		GEN_VDMA_ACFG(chn, sde_table_base + 480 + i, 0, st_h264->dir_scale_table[i]);
	}
	// set chroma_qp table
	unsigned int *sde_cqp_tbl = (void *)REG_SDE_CQP_TBL;
	for (i = 0; i < 128; i++) {
		GEN_VDMA_ACFG(chn, sde_cqp_tbl + i, 0, st_h264->chroma_qp_table[i]);
	}

	GEN_VDMA_ACFG(chn, (REG_SDE_SL_CTRL), VDMA_ACFG_TERM, SDE_MB_RUN);
}

#endif // _JZM_H264_API_C_
