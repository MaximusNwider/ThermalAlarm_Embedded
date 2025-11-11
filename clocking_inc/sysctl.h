#include <stdint.h>
#include <stdbool.h>

/* ===== System clock ===== */

/** Configure main clock to 80 MHz via PLL (16 MHz crystal assumed). */
void     sysctl_clock_pll_80mhz(void);

/** Return the cached system clock (Hz). Set by sysctl_clock_pll_80mhz(). */
uint32_t sysctl_get_sysclk_hz(void);

/** Busy-wait for 'cycles' core cycles (approx). */
void     sysctl_delay_cycles(uint32_t cycles);

/* ===== Peripheral clock gating (enable + wait ready) =====
 * Each 'enable_*' sets the corresponding RCGC bit and polls the matching PR* bit.
 * Index ranges (TM4C123GH6PM typical):
 *  - GPIO ports: 0..5  (A..F)
 *  - UART:       0..7
 *  - SSI:        0..3
 *  - I2C:        0..3
 *  - TIMER:      0..5
 *  - WTIMER:     0..5
 *  - ADC:        0..1
 *  - PWM:        0..1
 *  - CAN:        0..1
 *  - QEI:        0..1
 */

/* GPIO */
void sysctl_enable_gpio(uint8_t port_index);

/* UART / SSI / I2C */
void sysctl_enable_uart(uint8_t uart_index);
void sysctl_enable_ssi (uint8_t ssi_index);
void sysctl_enable_i2c (uint8_t i2c_index);

/* Timers */
void sysctl_enable_timer (uint8_t timer_index);
void sysctl_enable_wtimer(uint8_t wtimer_index);

/* Other blocks */
void sysctl_enable_adc   (uint8_t adc_index);
void sysctl_enable_pwm   (uint8_t pwm_index);
void sysctl_enable_can   (uint8_t can_index);
void sysctl_enable_qei   (uint8_t qei_index);
void sysctl_enable_usb   (void);
void sysctl_enable_hib   (void);
void sysctl_enable_eeprom(void);

/* ===== GPIO APB/AHB bus selection =====
 * When set to AHB for a port, the base address changes to the AHB aperture.
 * Keep your GPIO driver bases consistent with this setting.
 */
void sysctl_gpio_use_ahb(uint8_t port_index, bool use_ahb);

/* ===== PWM clock divider =====
 * Enable/disable PWM clock divide and set PWMDIV (2,4,8,16,32,64).
 * pwm_div must be one of: 2,4,8,16,32,64. If invalid, function picks 2.
 */
void sysctl_pwm_clock_config(bool use_div, uint32_t pwm_div);

