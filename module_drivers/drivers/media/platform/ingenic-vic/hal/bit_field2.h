#ifndef _BIT_FIELD2_H_
#define _BIT_FIELD2_H_

/**
 * the max value of bit field "[start, end]"
 */
static inline unsigned long bit_field_max(int start, int end)
{
	// 当 start = 0, end = 31 时
	// ((1ul << (end - start + 1)) 会-Wshift-count-overflow溢出
	// 溢出后在32位的机器上 1 << 32 的值是1, 不是我们期望的0或则0x100000000
	// 而 (1 << 31) << 1 在32的机器上的值是0,所以有效
	return ((1ul << (end - start)) << 1) - 1;
}

/**
 * the mask of bit field
 * mask = bit_field_max(start, end) << start;
 */
static inline unsigned long bit_field_mask(int start, int end)
{
	unsigned long e = (1ul << end);
	unsigned long s = (1ul << start);
	return (e - s) + e;
}

/**
 * check value is valid for the bit field
 * return bit_field_max(start, end) >= val;
 */
static inline int check_bit_field(int start, int end, unsigned long val)
{
	return bit_field_max(start, end) >= val;
}

/**
 * return the bitfield value
 */
static inline unsigned long bit_field_val(int start, int end, unsigned long val)
{
	return val << start;
}

/**
 * set value to the bit field
 * reg[start, end] = val;
 * @attention: this function do not check the value is valid for the bit field or not
 */
static inline unsigned long set_bit_field(unsigned long reg, int start, int end, unsigned long val)
{
	return (reg & ~bit_field_mask(start, end)) | (val << start);
}

/**
 * get value in the bit field
 * return reg[start, end];
 */
static inline unsigned long get_bit_field(unsigned long reg, int start, int end)
{
	return (reg >> start) & bit_field_max(start, end);
}

/**
 * set value to the bit field
 * reg[start, end] = val;
 * @attention: this function do not check the value is valid for the bit field or not
 * reg is volatile
 */
static inline void set_bit_field_v(volatile unsigned long *reg, int start, int end, unsigned long val)
{
	unsigned long mask = bit_field_mask(start, end);
	*reg = (*reg & ~mask) | ((val << start) & mask);
}

/**
 * get value in the bit field
 * return reg[start, end];
 * reg is volatile
 */
static inline unsigned long get_bit_field_v(volatile unsigned long *reg, int start, int end)
{
	return (*reg & bit_field_mask(start, end)) >> start;
}

#endif /* _BIT_FIELD2_H_ */
