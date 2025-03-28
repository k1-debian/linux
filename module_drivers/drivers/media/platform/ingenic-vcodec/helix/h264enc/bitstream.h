#ifndef __BITSTREAM_H__
#define __BITSTREAM_H__

typedef struct {
	uint8_t *p_start;
	uint8_t *p;
	uint8_t *p_end;

	unsigned long long cur_bits;
	int i_left;
	int i_bits_encoded;
} bs_t;

/* Unions for type-punning.
 * Mn: load or store n bits, aligned, native-endian
 * CPn: copy n bits, aligned, native-endian
 * we don't use memcpy for CPn because memcpy's args aren't assumed to be aligned */
typedef union {
	uint16_t i;
	uint8_t  c[2];
} h264_union16_t;
typedef union {
	uint32_t i;
	uint16_t b[2];
	uint8_t  c[4];
} h264_union32_t;
typedef union {
	uint64_t i;
	uint32_t a[2];
	uint16_t b[4];
	uint8_t c[8];
} h264_union64_t;
typedef struct {
	uint64_t i[2];
} h264_uint128_t;
typedef union {
	h264_uint128_t i;
	uint64_t a[2];
	uint32_t b[4];
	uint16_t c[8];
	uint8_t d[16];
} h264_union128_t;
#define M16(src) (((h264_union16_t*)(src))->i)
#define M32(src) (((h264_union32_t*)(src))->i)
#define M64(src) (((h264_union64_t*)(src))->i)
#define M128(src) (((h264_union128_t*)(src))->i)
#define M128_ZERO ((h264_uint128_t){{0,0}})
#define CP16(dst,src) M16(dst) = M16(src)
#define CP32(dst,src) M32(dst) = M32(src)
#define CP64(dst,src) M64(dst) = M64(src)
#define CP128(dst,src) M128(dst) = M128(src)

/*
    p_data必须按WORD地址对齐。
*/
static inline void bs_init(bs_t *s, void *p_data, int i_data)
{
	s->p = s->p_start = (uint8_t *)p_data;
	s->p_end = (uint8_t *)p_data + i_data;
	s->i_left = WORD_SIZE * 8;
	s->cur_bits = endian_fix32(M32(s->p));
	s->cur_bits >>= 32;
}

static inline int bs_pos(bs_t *s)
{
	return (8 * (s->p - s->p_start) + (WORD_SIZE * 8) - s->i_left);
}

static inline void bs_flush(bs_t *s)
{
	M32(s->p) = endian_fix32(s->cur_bits << (s->i_left & 31));
	s->p += WORD_SIZE - (s->i_left >> 3);
	s->i_left = WORD_SIZE * 8;
}

static inline void bs_realign(bs_t *s)
{
	int offset = ((intptr_t)s->p & 3);
	if (offset) {
		s->p       = (uint8_t *)s->p - offset;
		s->i_left  = (WORD_SIZE - offset) * 8;
		s->cur_bits = endian_fix32(M32(s->p));
		s->cur_bits >>= (4 - offset) * 8;
	}
}

static inline void bs_write(bs_t *s, int i_count, uint32_t i_bits)
{
	if (i_count < s->i_left) {
		s->cur_bits = (s->cur_bits << i_count) | i_bits;
		s->i_left -= i_count;
	} else {
		i_count -= s->i_left;
		s->cur_bits = (s->cur_bits << s->i_left) | (i_bits >> i_count);
		M32(s->p) = endian_fix(s->cur_bits);
		s->p += 4;
		s->cur_bits = i_bits;
		s->i_left = 32 - i_count;
	}
}

/* Special case to eliminate branch in normal bs_write. */
/* Golomb never writes an even-size code, so this is only used in slice headers. */
static inline void bs_write32(bs_t *s, uint32_t i_bits)
{
	bs_write(s, 16, i_bits >> 16);
	bs_write(s, 16, i_bits);
}

static inline void bs_write1(bs_t *s, uint32_t i_bit)
{
	s->cur_bits <<= 1;
	s->cur_bits |= i_bit;
	s->i_left--;
	if (s->i_left == WORD_SIZE * 8 - 32) {
		M32(s->p) = endian_fix32(s->cur_bits);
		s->p += 4;
		s->i_left = WORD_SIZE * 8;
	}
}

static inline void bs_align_0(bs_t *s)
{
	bs_write(s, s->i_left & 7, 0);
	bs_flush(s);
}
static inline void bs_align_1(bs_t *s)
{
	bs_write(s, s->i_left & 7, (1 << (s->i_left & 7)) - 1);
	bs_flush(s);
}
static inline void bs_align_10(bs_t *s)
{
	if (s->i_left & 7) {
		bs_write(s, s->i_left & 7, 1 << ((s->i_left & 7) - 1));
	}
	bs_flush(s);
}

/* golomb functions */

static const uint8_t x264_ue_size_tab[256] = {
	1, 1, 3, 3, 5, 5, 5, 5, 7, 7, 7, 7, 7, 7, 7, 7,
	9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
};

static inline void bs_write_ue_big(bs_t *s, unsigned int val)
{
	int size = 0;
	int tmp = ++val;
	if (tmp >= 0x10000) {
		size = 32;
		tmp >>= 16;
	}
	if (tmp >= 0x100) {
		size += 16;
		tmp >>= 8;
	}
	size += x264_ue_size_tab[tmp];
	bs_write(s, size >> 1, 0);
	bs_write(s, (size >> 1) + 1, val);
}

/* Only works on values under 255. */
static inline void bs_write_ue(bs_t *s, int val)
{
	bs_write(s, x264_ue_size_tab[val + 1], val + 1);
}
static inline void bs_write_se(bs_t *s, int val)
{
	int size = 0;
	/* Faster than (val <= 0 ? -val*2+1 : val*2) */
	/* 4 instructions on x86, 3 on ARM */
	int tmp = 1 - val * 2;
	if (tmp < 0) {
		tmp = val * 2;
	}
	val = tmp;

	if (tmp >= 0x100) {
		size = 16;
		tmp >>= 8;
	}
	size += x264_ue_size_tab[tmp];
	bs_write(s, size, val);
}

static inline void bs_write_te(bs_t *s, int x, int val)
{
	if (x == 1) {
		bs_write1(s, 1 ^ val);
	} else { //if( x > 1 )
		bs_write_ue(s, val);
	}
}

static inline void bs_rbsp_trailing(bs_t *s)
{
	bs_write1(s, 1);
	bs_write(s, s->i_left & 7, 0);
}

static inline int bs_size_ue(unsigned int val)
{
	return x264_ue_size_tab[val + 1];
}

static inline int bs_size_ue_big(unsigned int val)
{
	if (val < 255) {
		return x264_ue_size_tab[val + 1];
	} else {
		return x264_ue_size_tab[(val + 1) >> 8] + 16;
	}
}

static inline int bs_size_se(int val)
{
	int tmp = 1 - val * 2;
	if (tmp < 0) {
		tmp = val * 2;
	}
	if (tmp < 256) {
		return x264_ue_size_tab[tmp];
	} else {
		return x264_ue_size_tab[tmp >> 8] + 16;
	}
}

static inline int bs_size_te(int x, int val)
{
	if (x == 1) {
		return 1;
	} else { //if( x > 1 )
		return x264_ue_size_tab[val + 1];
	}
}

#endif
