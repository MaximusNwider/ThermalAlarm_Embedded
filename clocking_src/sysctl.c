

#include "sysctl.h"

/* ---------- HWREG helper ---------- */
#ifndef HWREG
#define HWREG(x) (*((volatile uint32_t *)(x)))
#endif

/* ---------- System Control base & key regs ---------- */
#define SYSCTL_BASE          0x400FE000UL
#define SYSCTL_RIS_R         HWREG(SYSCTL_BASE + 0x050)
#define SYSCTL_RCC_R         HWREG(SYSCTL_BASE + 0x060)
#define SYSCTL_GPIOHBCTL_R   HWREG(SYSCTL_BASE + 0x06C)
#define SYSCTL_RCC2_R        HWREG(SYSCTL_BASE + 0x070)

/* Run Mode Clock Gating Control (RCGC*) */
#define SYSCTL_RCGCWD_R      HWREG(SYSCTL_BASE + 0x600)
#define SYSCTL_RCGCTIMER_R   HWREG(SYSCTL_BASE + 0x604)
#define SYSCTL_RCGCGPIO_R    HWREG(SYSCTL_BASE + 0x608)
#define SYSCTL_RCGCDMA_R     HWREG(SYSCTL_BASE + 0x60C)
#define SYSCTL_RCGCHIB_R     HWREG(SYSCTL_BASE + 0x614)
#define SYSCTL_RCGCUART_R    HWREG(SYSCTL_BASE + 0x618)
#define SYSCTL_RCGCSSI_R     HWREG(SYSCTL_BASE + 0x61C)
#define SYSCTL_RCGCI2C_R     HWREG(SYSCTL_BASE + 0x620)
#define SYSCTL_RCGCUSB_R     HWREG(SYSCTL_BASE + 0x628)
#define SYSCTL_RCGCCAN_R     HWREG(SYSCTL_BASE + 0x634)
#define SYSCTL_RCGCADC_R     HWREG(SYSCTL_BASE + 0x638)
#define SYSCTL_RCGCACMP_R    HWREG(SYSCTL_BASE + 0x63C)
#define SYSCTL_RCGCPWM_R     HWREG(SYSCTL_BASE + 0x640)
#define SYSCTL_RCGCQEI_R     HWREG(SYSCTL_BASE + 0x644)
#define SYSCTL_RCGCEEPROM_R  HWREG(SYSCTL_BASE + 0x658)
#define SYSCTL_RCGCWTIMER_R  HWREG(SYSCTL_BASE + 0x65C)

/* Peripheral Ready (PR*) */
#define SYSCTL_PRWD_R        HWREG(SYSCTL_BASE + 0xA00)
#define SYSCTL_PRTIMER_R     HWREG(SYSCTL_BASE + 0xA04)
#define SYSCTL_PRGPIO_R      HWREG(SYSCTL_BASE + 0xA08)
#define SYSCTL_PRDMA_R       HWREG(SYSCTL_BASE + 0xA0C)
#define SYSCTL_PRHIB_R       HWREG(SYSCTL_BASE + 0xA14)
#define SYSCTL_PRUART_R      HWREG(SYSCTL_BASE + 0xA18)
#define SYSCTL_PRSSI_R       HWREG(SYSCTL_BASE + 0xA1C)
#define SYSCTL_PRI2C_R       HWREG(SYSCTL_BASE + 0xA20)
#define SYSCTL_PRUSB_R       HWREG(SYSCTL_BASE + 0xA28)
#define SYSCTL_PRCAN_R       HWREG(SYSCTL_BASE + 0xA34)
#define SYSCTL_PRADC_R       HWREG(SYSCTL_BASE + 0xA38)
#define SYSCTL_PRACMP_R      HWREG(SYSCTL_BASE + 0xA3C)
#define SYSCTL_PRPWM_R       HWREG(SYSCTL_BASE + 0xA40)
#define SYSCTL_PRQEI_R       HWREG(SYSCTL_BASE + 0xA44)
#define SYSCTL_PREEPROM_R    HWREG(SYSCTL_BASE + 0xA58)
#define SYSCTL_PRWTIMER_R    HWREG(SYSCTL_BASE + 0xA5C)

/* ---------- Local clock cache ---------- */
static uint32_t g_sysclk_hz = 16000000UL; /* default PIOSC unless reconfigured */

/* ---------- Small helper ---------- */
static inline void _wait_ready(volatile uint32_t *pr_reg, uint32_t bitmask)
{
  while ( (*pr_reg & bitmask) == 0 ) { /* spin */ }
}

/* ===== System clock: 80 MHz PLL ===== */
void sysctl_clock_pll_80mhz(void)
{
  /* The sequence matches the common TM4C123 configuration:
   *  - Switch to RCC2 control
   *  - Bypass PLL during config
   *  - 16 MHz crystal, PIOSC/MOSC select left default (PIOSC okay)
   *  - Power the PLL, wait lock (RIS.PLLLRIS)
   *  - DIV400 mode + SYSDIV settings for 80 MHz
   *  - Clear BYPASS2 to use PLL
   */
  SYSCTL_RCC2_R |= 0x80000000;      /* USERCC2 */
  SYSCTL_RCC2_R |= 0x00000800;      /* BYPASS2=1 (bypass PLL during config) */

  /* XTAL=16MHz: XTAL[10:6]=0x15 (done via mask/or combo used widely) */
  SYSCTL_RCC_R  &= ~0x000007C0;     /* clear XTAL */
  SYSCTL_RCC_R  |=  0x00000540;     /* set to 16MHz */

  SYSCTL_RCC2_R &= ~0x00000070;     /* OSCSRC2 -> main/PIOSC (default OK) */

  SYSCTL_RCC2_R &= ~0x00002000;     /* PWRDN2=0 (power-up PLL) */
  SYSCTL_RCC2_R |=  0x40000000;     /* DIV400: use 400 MHz PLL */
  /* With DIV400, the final clock is 400 MHz / (2 * (SYSDIV+1))  => set for 80 MHz */
  SYSCTL_RCC_R   = (SYSCTL_RCC_R & ~0x1FC00000) | (4u << 22); /* SYSDIV approx */

  /* Wait for PLL lock */
  while ( (SYSCTL_RIS_R & 0x00000040) == 0 ) { }

  /* Use PLL */
  SYSCTL_RCC2_R &= ~0x00000800;     /* BYPASS2=0 -> use PLL */

  g_sysclk_hz = 80000000UL;
}

uint32_t sysctl_get_sysclk_hz(void)
{
  return g_sysclk_hz;
}

void sysctl_delay_cycles(uint32_t cycles)
{
  while (cycles--) { __asm volatile("nop"); }
}

/* ===== Peripheral enables ===== */
void sysctl_enable_gpio(uint8_t port_index)
{
  if (port_index > 5) return;
  SYSCTL_RCGCGPIO_R |= (1u << port_index);
  _wait_ready(&SYSCTL_PRGPIO_R, (1u << port_index));
}

void sysctl_enable_uart(uint8_t uart_index)
{
  if (uart_index > 7) return;
  SYSCTL_RCGCUART_R |= (1u << uart_index);
  _wait_ready(&SYSCTL_PRUART_R, (1u << uart_index));
}

void sysctl_enable_ssi(uint8_t ssi_index)
{
  if (ssi_index > 3) return;
  SYSCTL_RCGCSSI_R |= (1u << ssi_index);
  _wait_ready(&SYSCTL_PRSSI_R, (1u << ssi_index));
}

void sysctl_enable_i2c(uint8_t i2c_index)
{
  if (i2c_index > 3) return;
  SYSCTL_RCGCI2C_R |= (1u << i2c_index);
  _wait_ready(&SYSCTL_PRI2C_R, (1u << i2c_index));
}

void sysctl_enable_timer(uint8_t timer_index)
{
  if (timer_index > 5) return;
  SYSCTL_RCGCTIMER_R |= (1u << timer_index);
  _wait_ready(&SYSCTL_PRTIMER_R, (1u << timer_index));
}

void sysctl_enable_wtimer(uint8_t wtimer_index)
{
  if (wtimer_index > 5) return;
  SYSCTL_RCGCWTIMER_R |= (1u << wtimer_index);
  _wait_ready(&SYSCTL_PRWTIMER_R, (1u << wtimer_index));
}

void sysctl_enable_adc(uint8_t adc_index)
{
  if (adc_index > 1) return;
  SYSCTL_RCGCADC_R |= (1u << adc_index);
  _wait_ready(&SYSCTL_PRADC_R, (1u << adc_index));
}

void sysctl_enable_pwm(uint8_t pwm_index)
{
  if (pwm_index > 1) return;
  SYSCTL_RCGCPWM_R |= (1u << pwm_index);
  _wait_ready(&SYSCTL_PRPWM_R, (1u << pwm_index));
}

void sysctl_enable_can(uint8_t can_index)
{
  if (can_index > 1) return;
  SYSCTL_RCGCCAN_R |= (1u << can_index);
  _wait_ready(&SYSCTL_PRCAN_R, (1u << can_index));
}

void sysctl_enable_qei(uint8_t qei_index)
{
  if (qei_index > 1) return;
  SYSCTL_RCGCQEI_R |= (1u << qei_index);
  _wait_ready(&SYSCTL_PRQEI_R, (1u << qei_index));
}

void sysctl_enable_usb(void)
{
  SYSCTL_RCGCUSB_R |= 1u;
  _wait_ready(&SYSCTL_PRUSB_R, 1u);
}

void sysctl_enable_hib(void)
{
  SYSCTL_RCGCHIB_R |= 1u;
  _wait_ready(&SYSCTL_PRHIB_R, 1u);
}

void sysctl_enable_eeprom(void)
{
  SYSCTL_RCGCEEPROM_R |= 1u;
  _wait_ready(&SYSCTL_PREEPROM_R, 1u);
}

/* ===== GPIO AHB/APB selection ===== */
void sysctl_gpio_use_ahb(uint8_t port_index, bool use_ahb)
{
  if (port_index > 5) return;
  if (use_ahb)  SYSCTL_GPIOHBCTL_R |=  (1u << port_index);
  else          SYSCTL_GPIOHBCTL_R &= ~(1u << port_index);
}

/* ===== PWM clock divider =====
 * use_div = true  -> enable USEPWMDIV and set divisor to {2,4,8,16,32,64}
 * use_div = false -> disable USEPWMDIV (PWM uses sysclk)
 */
void sysctl_pwm_clock_config(bool use_div, uint32_t pwm_div)
{
  uint32_t r = SYSCTL_RCC_R;

  /* Clear PWMDIV field and USEPWMDIV first */
  r &= ~(0x7u << 17);   /* PWMDIV bits [19:17] */
  r &= ~(1u << 20);     /* USEPWMDIV bit */

  if (use_div) {
    uint32_t code = 0; /* default /2 */
    switch (pwm_div) {
      case 2:  code = 0; break;
      case 4:  code = 1; break;
      case 8:  code = 2; break;
      case 16: code = 3; break;
      case 32: code = 4; break;
      case 64: code = 5; break;
      default: code = 0; break; /* fall back to /2 */
    }
    r |= (1u << 20);           /* USEPWMDIV */
    r |= (code << 17);         /* PWMDIV code */
  }

  SYSCTL_RCC_R = r;
}
