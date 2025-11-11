
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Constants & helper macros ===== */

/* 12-bit SAR range */
#define ADC_PE4_MAX_CODE          4095u

/* Hardware averaging (ADCSAC = 2^n samples) */
#define ADC_PE4_AVG_OFF           0u  /* 1x  */
#define ADC_PE4_AVG_2X            1u  /* 2x  */
#define ADC_PE4_AVG_4X            2u  /* 4x  */
#define ADC_PE4_AVG_8X            3u  /* 8x  */
#define ADC_PE4_AVG_16X           4u  /* 16x */
#define ADC_PE4_AVG_32X           5u  /* 32x */
#define ADC_PE4_AVG_64X           6u  /* 64x */

/* ADC clock source (ADC0_CC.CS) — values per TM4C123 datasheet */
#define ADC_PE4_CLK_PLL           0x0u  /* default: PLL/system-derived 16 MHz */
#define ADC_PE4_CLK_PIOSC         0x1u  /* internal 16 MHz PIOSC */

/* Max sample-rate cap (ADC0_PC.SR) */
#define ADC_PE4_SR_125KSPS        0x0u
#define ADC_PE4_SR_250KSPS        0x1u
#define ADC_PE4_SR_500KSPS        0x2u
#define ADC_PE4_SR_1MSPS          0x3u

/* ===== Initialization & configuration ===== */

/**
 * @brief Initialize ADC0 to sample PE4 (AIN9) via SS3 (single sample, processor-triggered).
 *        Sets PE4 analog path, SS3 MUX=9, SSCTL3=IE0|END0, and default averaging (16x).
 *        Uses default ADC clock source and 125 ksps cap (conservative).
 */
void    adc0_pe4_init(void);

/**
 * @brief Set hardware averaging (ADCSAC) to 2^pow2_n samples (0..6 => 1..64x).
 */
void    adc0_pe4_set_hw_averaging(uint8_t pow2_n);

/**
 * @brief Configure ADC clock source and max sample-rate cap at runtime.
 * @param clock_sel  ADC0_CC.CS (e.g., ADC_PE4_CLK_PLL or ADC_PE4_CLK_PIOSC).
 * @param sr_code    ADC0_PC.SR  (e.g., ADC_PE4_SR_125KSPS .. ADC_PE4_SR_1MSPS).
 */
void    adc0_pe4_set_clock(uint32_t clock_sel, uint32_t sr_code);

/**
 * @brief Set the VREF used by raw→volts conversions (typ. 3.3 V).
 */
void    adc0_pe4_set_vref(float vref_volts);

/* ===== Blocking reads ===== */

/**
 * @brief Take one blocking conversion on SS3 and return 12‑bit raw code (0..4095).
 */
uint16_t adc0_pe4_read(void);

/**
 * @brief Take N back‑to‑back conversions and return the rounded integer average.
 *        Use this to compound hardware averaging with a simple boxcar filter.
 */
uint16_t adc0_pe4_read_n_average(uint32_t n);

/* ===== Calibration & conversions =====
 * Two ways to compute temperature (°C):
 *   - Voltage-based:  T = m_V * Vadc + b_V   (default path; Vadc from vref)
 *   - Raw-based:      T = m_R * raw  + b_R   (optional direct mapping)
 */

/** Select a voltage-based line (default): T = m_V * Vadc + b_V. */
void    adc0_pe4_set_cal_voltage_line(float m_V, float b_V);

/** Compute a voltage-based line from two points: (T0,raw0), (T1,raw1). */
void    adc0_pe4_calibrate_two_point_voltage(float T0_C, uint16_t raw0,
                                             float T1_C, uint16_t raw1);

/** Select a raw-based line: T = m_R * raw + b_R. */
void    adc0_pe4_set_cal_raw_line(float m_R, float b_R);

/** Compute a raw-based line from two points: (T0,raw0), (T1,raw1). */
void    adc0_pe4_calibrate_two_point_raw(float T0_C, uint16_t raw0,
                                         float T1_C, uint16_t raw1);

/** Convert a raw code to volts using current VREF. */
float   adc0_pe4_raw_to_volts(uint16_t raw);

/** Convert a raw code to °C using the active calibration line (raw- or voltage-based). */
float   adc0_pe4_raw_to_celsius(uint16_t raw);

/** Take N samples, average, and return °C using the active calibration line. */
float   adc0_pe4_read_celsius_avg(uint32_t n);

/* ===== Diagnostics ===== */

/**
 * @brief Single-shot self test: triggers SS3, validates RIS/ISC behavior.
 * @return 0 on success; negative on timeout/flag errors.
 */
int     adc0_pe4_self_test_once(void);

#ifdef __cplusplus
}
#endif
