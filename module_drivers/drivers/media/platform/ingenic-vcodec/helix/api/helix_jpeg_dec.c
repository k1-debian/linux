#include "helix_jpeg.h"

void JPEGD_SliceInit(_JPEGD_SliceInfo *s)
{
	unsigned int i;
	volatile unsigned int *chn = (volatile unsigned int *)s->des_va;

	GEN_VDMA_ACFG(chn, TCSM_FLUSH, 0, 0x0);

	/* Open clock configuration */
	GEN_VDMA_ACFG(chn, REG_JPGC_GLBI, 0, OPEN_CLOCK);

	/**************************************************
	 Huffman Encode Table configuration
	*************************************************/
	for (i = 0; i < HUFFMIN_LEN; i++) {
		GEN_VDMA_ACFG(chn, REG_JPGC_HUFM + i * 4, 0, *(s->huffmin++));
	}
	for (i = 0; i < HUFFBASE_LEN; i++) {
		GEN_VDMA_ACFG(chn, REG_JPGC_HUFB + i * 4, 0, *(s->huffbase++));
	}
	for (i = 0; i < HUFFSYMB_LEN; i++) {
		GEN_VDMA_ACFG(chn, REG_JPGC_HUFS + i * 4, 0, *(s->huffsymb++));
	}
	/**************************************************
	 Quantization Table configuration
	*************************************************/
	for (i = 0; i < QMEM_LEN; i++) {
		GEN_VDMA_ACFG(chn, REG_JPGC_QMEM + i * 4, 0, *(s->qmem++));
	}

	/**************************************************
	 REGs configuration
	*************************************************/
	GEN_VDMA_ACFG(chn, REG_JPGC_STAT, 0, 0);
	GEN_VDMA_ACFG(chn, REG_JPGC_BSA, 0, (uint32_t)s->bsa);
	GEN_VDMA_ACFG(chn, REG_JPGC_P0A, 0, (uint32_t)s->p0a);
	GEN_VDMA_ACFG(chn, REG_JPGC_P1A, 0, (uint32_t)s->p1a);
	GEN_VDMA_ACFG(chn, REG_JPGC_NMCU, 0, s->nmcu);
	GEN_VDMA_ACFG(chn, REG_JPGC_NRSM, 0, s->nrsm);
	GEN_VDMA_ACFG(chn, REG_JPGC_WIDTH, 0, s->width);
	GEN_VDMA_ACFG(chn, REG_JPGC_P0C, 0, s->pxc[0]); /* component 0 configure information NBLK<<4| QT<<2| HA<<1| HD */
	GEN_VDMA_ACFG(chn, REG_JPGC_P1C, 0, s->pxc[1]);
	GEN_VDMA_ACFG(chn, REG_JPGC_P2C, 0, s->pxc[2]);
	GEN_VDMA_ACFG(chn, REG_JPGC_P3C, 0, s->pxc[3]);

	GEN_VDMA_ACFG(chn, REG_JPGC_GLBI, 0, (YUV420PVH /*P0V|P0H*/ |
	                                      JPGC_NCOL /*NCOL*/ |
	                                      JPGC_DEC/*DE*/ |
	                                      JPGC_EN  /*ENABLE*/));

	GEN_VDMA_ACFG(chn, REG_JPGC_TRIG, VDMA_ACFG_TERM, JPGC_BS_TRIG | JPGC_PP_TRIG | JPGC_CORE_OPEN);

}
