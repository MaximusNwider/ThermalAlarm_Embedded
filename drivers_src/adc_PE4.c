
#include <stdint.h>
#include <stdbool.h>
#include "../drivers_inc/adc_PE4.h"

// ---------- Handy X helper ----------
#ifndef X
#define X(x) (*((volatile uint32_t *)(x)))
#endif

// ---------- System Control base ----------
#define SYSCTL_BASE            0x400FE000UL
#define SYSCTL_RCGCGPIO_R      X(SYSCTL_BASE + 0x608)
#define SYSCTL_PRGPIO_R        X(SYSCTL_BASE + 0xA08)
#define SYSCTL_RCGCADC_R       X(SYSCTL_BASE + 0x638)
#define SYSCTL_PRADC_R         X(SYSCTL_BASE + 0xA38)
#define SYSCTL_SRADC_R         X(SYSCTL_BASE + 0x528)

// ---------- GPIO Port E base (PE4 = AIN9) ----------
#define GPIOE_BASE             0x40024000UL
#define GPIO_PORTE_DIR_R       X(GPIOE_BASE + 0x400)
#define GPIO_PORTE_AFSEL_R     X(GPIOE_BASE + 0x420)
#define GPIO_PORTE_DEN_R       X(GPIOE_BASE + 0x51C)
#define GPIO_PORTE_AMSEL_R     X(GPIOE_BASE + 0x528)

// ---------- ADC0 base & registers ----------
#define ADC0_BASE              0x40038000UL
#define ADC0_ACTSS_R           X(ADC0_BASE + 0x000) // Active Sample Sequencer
#define ADC0_RIS_R             X(ADC0_BASE + 0x004) // Raw Int Status
#define ADC0_IM_R              X(ADC0_BASE + 0x008) // Int Mask
#define ADC0_ISC_R             X(ADC0_BASE + 0x00C) // Int Status & Clear
#define ADC0_EMUX_R            X(ADC0_BASE + 0x014) // Event Multiplexer
#define ADC0_SSPRI_R           X(ADC0_BASE + 0x020) // Sequencer Priority
#define ADC0_PSSI_R            X(ADC0_BASE + 0x028) // Processor Trigger
#define ADC0_SAC_R             X(ADC0_BASE + 0x030) // Sample Averaging Control
#define ADC0_CTL_R             X(ADC0_BASE + 0x038) // ADC Control (DITHER, VREF)
#define ADC0_SSMUX3_R          X(ADC0_BASE + 0x0A0) // MUX for SS3
#define ADC0_SSCTL3_R          X(ADC0_BASE + 0x0A4) // Control for SS3
#define ADC0_SSFIFO3_R         X(ADC0_BASE + 0x0A8) // FIFO for SS3
#define ADC0_SSFSTAT3_R        X(ADC0_BASE + 0x0AC) // FIFO status for SS3
#define ADC0_PC_R              X(ADC0_BASE + 0xFC4) // Peripheral Config (sample rate cap)
#define ADC0_CC_R              X(ADC0_BASE + 0xFC8) // Clock Config (clock source)

// ---------- Bit helpers ----------
#define BIT(n)                 (1UL << (n))

// SYSCTL RCGC bits
#define RCGCGPIO_E             BIT(4)
#define RCGCADC_0              BIT(0)
#define PRGPIO_E               BIT(4)
#define PRADC_0                BIT(0)

// ADC ACTSS bits
#define ACTSS_ASEN3            BIT(3)

// ADC EMUX fields (EM3 at bits [15:12])
#define EMUX_EM3_SHIFT         12
#define EMUX_TRIG_PROCESSOR    0x0UL  // Processor trigger

// ADC RIS / ISC bits
#define ADC_RIS_INR3           BIT(3)
#define ADC_ISC_IN3            BIT(3)

// SSCTL3 control bits
#define SSCTL_IE0              BIT(2)
#define SSCTL_END0             BIT(1)
// (TS0/D0 not used for external pin sampling)

// ADC PC (sample-rate cap) SR field [3:0]
#define ADCPC_SR_125KSPS       0x1
#define ADCPC_SR_250KSPS       0x3
#define ADCPC_SR_500KSPS       0x5
#define ADCPC_SR_1MSPS         0x7

// ADC CC (clock source) CS field [3:0]
#define ADCCC_CS_PLLDIV        0x0  // Default: PLL/25 -> 16 MHz ADC clock
#define ADCCC_CS_PIOSC         0x1  // 16 MHz PIOSC

// ADCCTL (global to all ADCs): DITHER bit (per TI headers)
#define ADC_CTL_DITHER         0x00000040U  // See errata note below

// ---------- Calibration store ----------
typedef struct {
  float m;       // Volts per code
  float b;       // Volts offset
  bool  valid;
} adc_cal_t;

static adc_cal_t s_cal = {0};

// ---------- Local helpers ----------
static inline void _delay_cycles(volatile uint32_t n) {
  while (n--) { __asm volatile("nop"); }
}

// ---------- Public API ----------

/**
 * @brief Initialize ADC0/SS3 to sample PE4 (AIN9).
 * @param use_piosc       If true, clock ADC from 16 MHz PIOSC (ADCCC=PIOSC).
 * @param sr_code         ADCPC sample-rate cap: use ADCPC_SR_* values above.
 * @param avg_power2      Hardware averaging: 0->1x,1->2x,2->4x,3->8x,4->16x,5->32x,6->64x.
 * @param enable_dither   If true, set ADCCTL.DITHER (note some revisions ignore this, see errata).
 */
void adc_pe4_init(bool use_piosc, uint8_t sr_code, uint8_t avg_power2, bool enable_dither)
{
  // --- 1) Enable Port E and wait ready ---
  SYSCTL_RCGCGPIO_R |= RCGCGPIO_E;
  while ((SYSCTL_PRGPIO_R & PRGPIO_E) == 0) { }

  // PE4 as analog input (AIN9)
  GPIO_PORTE_DIR_R   &= ~BIT(4);   // input
  GPIO_PORTE_AFSEL_R |=  BIT(4);   // alt function path
  GPIO_PORTE_DEN_R   &= ~BIT(4);   // disable digital
  GPIO_PORTE_AMSEL_R |=  BIT(4);   // enable analog

  // --- 2) Enable ADC0 clock and (per errata) reset ADC before use ---
  SYSCTL_RCGCADC_R |= RCGCADC_0;
  while ((SYSCTL_PRADC_R & PRADC_0) == 0) { }

  // Silicon errata: first two samples after clock enable may be incorrect.
  // Recommended: reset the ADC module after enabling the clock.
  // Assert then deassert software reset for ADC0:
  SYSCTL_SRADC_R |=  RCGCADC_0;
  SYSCTL_SRADC_R &= ~RCGCADC_0;

  // --- 3) Configure ADC clock source and overall max sample rate ---
  ADC0_CC_R = (use_piosc ? ADCCC_CS_PIOSC : ADCCC_CS_PLLDIV); // module clock = 16 MHz source
  ADC0_PC_R = (ADC0_PC_R & ~0xFUL) | (sr_code & 0xF);         // cap: 125/250/500k or 1Msps

  // --- 4) Configure SS3 to read AIN9 (PE4), processor trigger, one sample ---
  ADC0_ACTSS_R &= ~ACTSS_ASEN3;                                  // disable SS3 during config
  ADC0_EMUX_R   = (ADC0_EMUX_R & ~(0xFUL << EMUX_EM3_SHIFT)) |
                  (EMUX_TRIG_PROCESSOR << EMUX_EM3_SHIFT);       // processor trigger
  ADC0_SSMUX3_R = 9U;                                            // AIN9 = PE4
  ADC0_SSCTL3_R = (SSCTL_IE0 | SSCTL_END0);                      // one sample, end, set flag
  ADC0_SAC_R    = (avg_power2 & 0x7);                            // hardware averaging 1..64x

  if (enable_dither) {
    // Note: per TM4C123x errata, DITHER may not function on some silicon revisions.
    ADC0_CTL_R |= ADC_CTL_DITHER;
  }

  ADC0_ACTSS_R |= ACTSS_ASEN3;                                   // enable SS3

  // --- 5) Flush a couple of samples (errata safety + settle) ---
  (void)adc_pe4_read_u12_blocking();
  (void)adc_pe4_read_u12_blocking();
}

/**
 * @brief Initiate a blocking conversion using processor trigger on SS3 and return 12-bit result.
 */
uint16_t adc_pe4_read_u12_blocking(void)
{
  // Initiate conversion on SS3
  ADC0_PSSI_R = BIT(3);

  // Wait for completion flag
  while ((ADC0_RIS_R & ADC_RIS_INR3) == 0) { }

  // Read FIFO and clear completion
  uint16_t code = (uint16_t)(ADC0_SSFIFO3_R & 0xFFFU);
  ADC0_ISC_R = ADC_ISC_IN3;

  return code;
}

/**
 * @brief Take N samples back-to-back (blocking) and return the integer average.
 *        This compounds hardware averaging (if any) with a simple boxcar average.
 */
uint16_t adc_pe4_read_average(uint32_t n)
{
  uint32_t acc = 0;
  if (n == 0) return 0;

  for (uint32_t i = 0; i < n; ++i) {
    acc += adc_pe4_read_u12_blocking();
  }
  return (uint16_t)(acc / n);
}

/**
 * @brief Simple two-point calibration: provide two known voltage points and measured codes.
 *        Driver computes V = m*code + b and stores internally.
 */
void adc_pe4_set_cal_2pt(float v0_volts, uint16_t code0, float v1_volts, uint16_t code1)
{
  if (code1 == code0) { s_cal.valid = false; return; }
  s_cal.m = (v1_volts - v0_volts) / (float)((int32_t)code1 - (int32_t)code0);
  s_cal.b = v0_volts - s_cal.m * (float)code0;
  s_cal.valid = true;
}

/**
 * @brief Convert raw 12-bit code to volts using either 2-point calibration (if set),
 *        or ideal scaling with supplied vref (VDDA).
 */
float adc_pe4_code_to_volts(uint16_t code, float vref_volts_if_no_cal)
{
  if (s_cal.valid) {
    return s_cal.m * (float)code + s_cal.b;
  }
  // Ideal mapping: 0..4095 codes over 0..VREF (datasheet).
  return (vref_volts_if_no_cal * (float)code) / 4095.0f;
}

/**
 * @brief Optionally reconfigure the ADC clock source and sample rate cap at runtime.
 */
void adc_pe4_set_clock(bool use_piosc, uint8_t sr_code)
{
  ADC0_CC_R = (use_piosc ? ADCCC_CS_PIOSC : ADCCC_CS_PLLDIV);
  ADC0_PC_R = (ADC0_PC_R & ~0xFUL) | (sr_code & 0xF);
}

/**
 * @brief Change hardware averaging (ADCSAC) on the fly. 0..6 => 1..64 samples averaged.
 */
void adc_pe4_set_hw_averaging(uint8_t avg_power2)
{
  ADC0_SAC_R = (avg_power2 & 0x7);
}
