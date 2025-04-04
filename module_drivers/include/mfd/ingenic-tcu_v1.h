#ifndef __INGENIC_TCU_H__
#define __INGENIC_TCU_H__

#include <dt-bindings/clock/ingenic-tcu.h>

#define NR_TCU_CHNS TCU_NR_IRQS

enum irq_bit {
	FULL_BIT,
	HALF_BIT,
};
enum tcu_prescale {
	TCU_PRESCALE_1,
	TCU_PRESCALE_4,
	TCU_PRESCALE_16,
	TCU_PRESCALE_64,
	TCU_PRESCALE_256,
	TCU_PRESCALE_1024
};

enum tcu_clksrc {
	TCU_CLKSRC_PCK = 1,
	TCU_CLKSRC_RTC = 2,
	TCU_CLKSRC_EXT = 4
};

enum tcu_irq_type {
	NULL_IRQ_MODE,
	FULL_IRQ_MODE,
	HALF_IRQ_MODE,
	FULL_HALF_IRQ_MODE,
};

struct info_bits {
unsigned int id: CHANNEL_BASE_OFF * 2;
unsigned int mode: CHANNEL_BASE_OFF;
unsigned int func: CHANNEL_BASE_OFF;
unsigned int pwmin: CHANNEL_BASE_OFF;
};

struct ingenic_tcu_chn {
	union {
		unsigned int chn_info;
		struct info_bits cib;
	};
	unsigned int index;
	char virq[2];
	char irq_type;
	char clk_src;

	char clk_div;
	char init_level;
	char shutdown_mode;
	char pwm_bapass_mode;

	char is_pwm;
	char is_count_clear;
	char pwm_in_en;
	char work_sleep;

	int half_num;
	int full_num;
	struct device_node *np;
	void (*enable)(int id);
	void (*disable)(int id);
};

void tcu_start_counter(int id);
void tcu_stop_counter(int id);
void tcu_set_counter(int id, unsigned int val);
int tcu_get_counter(int id);
void tcu_enable_counter(int id);
void tcu_disable_counter(int id);

/**
 * ingenic_tcu_set_full_num - set the number of tcu Timer Data FULL Register (TDFR)
 *
 * @id:     tcu channel id.
 * @full_num:   the number of set TDFR.
 *
 */
void ingenic_tcu_set_period(int id, uint16_t period);
/**
 * ingenic_tcu_set_half_num - set the number of tcu Timer Data FULL Register (TDHR)
 *
 * @id:     tcu channel id.
 * @half_num:   the number of set TDFR.
 *
 */
void ingenic_tcu_set_duty(int id, uint16_t duty);
/**
 * ingenic_tcu_set_prescale - set clk div of TCU prescale.
 * Don‘t call the function  when the channel is running.
 *
 * @id:     tcu channel id.
 * @prescale:   the div of clk value.
 *
 */
void ingenic_tcu_set_prescale(int id, enum tcu_prescale prescale);
/**
 * ingenic_tcu_set_pwm_output_init_level - set an initial output level for PWM output.
 *
 * @id:     tcu channel id.
 * @level:  0 LOW, 1 HIGH.
 *
 */
void ingenic_tcu_set_pwm_output_init_level(int id, int level);
/**
 * ingenic_tcu_set_clksrc - set clk soruce of the timer clock input.
 * Don‘t call the function  when the channel is running.
 *
 * @id:     tcu channel id.
 * @src:    the clk source;0 PCLK, 1 RTC, 2 EXT.
 *
 */
void ingenic_tcu_set_clksrc(int id, enum tcu_clksrc src);
/**
 * ingenic_tcu_channel_to_virq - get virtual irq though to tcu channel id
 * (channel 0 ~ 4, 6 ~ 7 use common hardware irq(TCU2))
 *
 * @tcu_chn: the struct of request cell(TCU channel).
 *
 * Search for a virq in a irq domain and save irq to struct ingenic_tcu_chn
 * ->virq[2].
 */
void ingenic_tcu_channel_to_virq(struct ingenic_tcu_chn *tcu_chn);
/**
 * ingenic_tcu_get_count - get the value of the timer counter (TCNT).
 *
 * @id:     tcu channel id.
 *
 * Returns number of timer counter on sucess,
 * -EINVAL if The value read from counter 1 or 2 is a false value.
 *
 */
int ingenic_tcu_get_count(int id);
/**
 * ingenic_tcu_config - init the current channel struct.
 *
 * @tcu_chn:        tcu channel struct.
 *
 * Returns 0 on sucess,
 * Returns < 0 if init failed.
 *
 */
int ingenic_tcu_config(struct ingenic_tcu_chn *tcu_chn);
/**
 * ingenic_tcu_counter_begin - begin counting up, if channel function is pwm, enable pwm.
 *
 * @tcu_chn:        tcu channel struct.
 *
 */
int ingenic_tcu_counter_begin(struct ingenic_tcu_chn *tcu_chn);
/**
 * ingenic_tcu_counter_stop - stop counting up, if channel function is pwm, disable pwm.
 *
 * @tcu_chn:        tcu channel struct.
 *
 * Explain：
 * TCU count stop, is the next stop after match, not immediately stop.
 * If you want to stop immediately, you can use cell->disable.
 *
 */
void ingenic_tcu_counter_stop(struct ingenic_tcu_chn *tcu_chn);

/**
 * ingenic_watchdog_set_count - setwatchdog timer counter.
 * This function is only used for watchdog.
 *
 * @value:      the number of set watchdog counter.
 *
 */
void ingenic_watchdog_set_count(unsigned int value);
/**
 * ingenic_watchdog_config - init watchdog.
 * This function is only used for watchdog.
 *
 * @tcsr_val:       the number of set watchdog TCSR.
 * @timeout_value:      the number of set watchdog TCNT.
 *
 */
void ingenic_watchdog_config(unsigned int tcsr_val, unsigned int timeout_value);

/**
 * request_cell - request tcu channel cell.
 *
 * @id:     tcu channel id.
 *
 * Returns struct cell on sucess,
 * NULL if The channel busy.
 *
 */
struct mfd_cell *request_cell(int id);
/**
 * free_cell - free tcu channel cell.
 *
 * @id:     tcu channel id.
 *
 */
void free_cell(int id);

#endif /* __INGENIC_TCU_H__ */
