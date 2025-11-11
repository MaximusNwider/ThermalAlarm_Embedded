#include<stdint.h>
#include "../drivers_inc/gpio.h"

#ifndef X
#define X(x) (*((volatile uint32_t *)(x)))
#endif

/* System Control: GPIO run/ready */
#define SYSCTL_BASE         0x400FE000UL
#define SYSCTL_RCGCGPIO_R   X(SYSCTL_BASE + 0x608)
#define SYSCTL_PRGPIO_R     X(SYSCTL_BASE + 0xA08)

/* NVIC registers (enable + priority) */
#define NVIC_EN0_R          X(0xE000E100)
#define NVIC_EN1_R          X(0xE000E104)
#define NVIC_PRI_BASE       0xE000E400UL

/* GPIO base addresses */
static inline uint32_t _gpio_base(gpio_port_t p) {
  static const uint32_t base[6] = {
    0x40004000UL, /* A */
    0x40005000UL, /* B */
    0x40006000UL, /* C */
    0x40007000UL, /* D */
    0x40024000UL, /* E */
    0x40025000UL  /* F */
  };
  return base[(int)p];
}

/* Offsets from GPIO base */
#define GPIO_DATA_3FC   0x3FC
#define GPIO_DIR_OFS    0x400
#define GPIO_IS_OFS     0x404
#define GPIO_IBE_OFS    0x408
#define GPIO_IEV_OFS    0x40C
#define GPIO_IM_OFS     0x410
#define GPIO_RIS_OFS    0x414
#define GPIO_MIS_OFS    0x418
#define GPIO_ICR_OFS    0x41C
#define GPIO_AFSEL_OFS  0x420
#define GPIO_DR2R_OFS   0x500
#define GPIO_DR4R_OFS   0x504
#define GPIO_DR8R_OFS   0x508
#define GPIO_ODR_OFS    0x50C
#define GPIO_PUR_OFS    0x510
#define GPIO_PDR_OFS    0x514
#define GPIO_SLR_OFS    0x518
#define GPIO_DEN_OFS    0x51C
#define GPIO_LOCK_OFS   0x520
#define GPIO_CR_OFS     0x524
#define GPIO_AMSEL_OFS  0x528
#define GPIO_PCTL_OFS   0x52C

#define GPIO_LOCK_KEY   0x4C4F434BUL

/* NVIC IRQ numbers for ports on TM4C123 */
static inline int _gpio_irq_num(gpio_port_t p) {
  switch (p) {
    case GPIO_PORT_A: return 0;
    case GPIO_PORT_B: return 1;
    case GPIO_PORT_C: return 2;
    case GPIO_PORT_D: return 3;
    case GPIO_PORT_E: return 4;
    case GPIO_PORT_F: return 30;
    default: return -1;
  }
}

/* ----------------- Public API ----------------- */

void gpio_port_enable(gpio_port_t port)
{
  SYSCTL_RCGCGPIO_R |= (1u << (uint32_t)port);
  /* wait until peripheral ready */
  while ((SYSCTL_PRGPIO_R & (1u << (uint32_t)port)) == 0) { }
}

/* Unlock/commit for PF0 and PD7 (and optionally PC0–3) */
static void _maybe_unlock_commit(gpio_port_t port, uint8_t mask, bool also_pc_jtag)
{
  uint32_t base = _gpio_base(port);
  uint32_t need = 0;

  if (port == GPIO_PORT_F && (mask & (1u<<0))) need |= (1u<<0); /* PF0 */
  if (port == GPIO_PORT_D && (mask & (1u<<7))) need |= (1u<<7); /* PD7 */
  if (also_pc_jtag && port == GPIO_PORT_C && (mask & 0x0Fu))    need |= (mask & 0x0Fu); /* PC0..3 */

  if (need) {
    X(base + GPIO_LOCK_OFS) = GPIO_LOCK_KEY;
    X(base + GPIO_CR_OFS)  |= need;   /* allow changes on these pins */
    /* (Optionally re-lock by writing any value != KEY; not strictly required) */
  }
}

static void _set_drive_strength(uint32_t base, uint8_t mask, gpio_drive_t drv, bool slew)
{
  /* Clear previous */
  X(base + GPIO_DR2R_OFS) &= ~mask;
  X(base + GPIO_DR4R_OFS) &= ~mask;
  X(base + GPIO_DR8R_OFS) &= ~mask;

  switch (drv) {
    case GPIO_DRIVE_2MA: X(base + GPIO_DR2R_OFS) |= mask; break;
    case GPIO_DRIVE_4MA: X(base + GPIO_DR4R_OFS) |= mask; break;
    case GPIO_DRIVE_8MA: X(base + GPIO_DR8R_OFS) |= mask; break;
    default:             X(base + GPIO_DR2R_OFS) |= mask; break; /* default 2mA */
  }
  if (slew) X(base + GPIO_SLR_OFS) |= mask;
  else      X(base + GPIO_SLR_OFS) &= ~mask;
}

static void _set_pctl(uint32_t base, uint8_t mask, uint8_t nibble)
{
  /* Apply same PCTL nibble to each selected pin */
  uint32_t pctl = X(base + GPIO_PCTL_OFS);
  for (uint8_t pin = 0; pin < 8; ++pin) {
    if (mask & (1u << pin)) {
      uint32_t shift = (uint32_t)pin * 4u;
      pctl &= ~(0xFu << shift);
      pctl |=  ((uint32_t)(nibble & 0xF) << shift);
    }
  }
  X(base + GPIO_PCTL_OFS) = pctl;
}

int gpio_configure(gpio_port_t port, const gpio_pin_cfg_t *cfg)
{
  if (!cfg || cfg->pins == 0) return -1;

  gpio_port_enable(port); /* ensure clock */

  uint32_t base = _gpio_base(port);
  uint8_t  mask = cfg->pins;

  if (cfg->unlock_protected) {
    _maybe_unlock_commit(port, mask, /*also_pc_jtag=*/false);
  }

  /* Direction */
  if (cfg->dir_output) X(base + GPIO_DIR_OFS) |= mask;
  else                 X(base + GPIO_DIR_OFS) &= ~mask;

  /* Alternate function select (AFSEL) + PCTL */
  if (cfg->alt_enable) {
    X(base + GPIO_AFSEL_OFS) |= mask;
    _set_pctl(base, mask, cfg->pctl_nibble);
  } else {
    X(base + GPIO_AFSEL_OFS) &= ~mask;
    /* If not alt, ensure PCTL nibbles are 0 for those pins */
    _set_pctl(base, mask, 0);
  }

  /* Digital / Analog path */
  if (cfg->analog_enable) {
    X(base + GPIO_DEN_OFS)   &= ~mask;
    X(base + GPIO_AMSEL_OFS) |=  mask;
  } else {
    X(base + GPIO_AMSEL_OFS) &= ~mask;
    if (cfg->digital_enable) X(base + GPIO_DEN_OFS) |= mask;
    else                     X(base + GPIO_DEN_OFS) &= ~mask;
  }

  /* Pull-ups / Pull-downs (exclusive) */
  if (cfg->pull_up) {
    X(base + GPIO_PUR_OFS) |= mask;
    X(base + GPIO_PDR_OFS) &= ~mask;
  } else if (cfg->pull_down) {
    X(base + GPIO_PDR_OFS) |= mask;
    X(base + GPIO_PUR_OFS) &= ~mask;
  } else {
    X(base + GPIO_PUR_OFS) &= ~mask;
    X(base + GPIO_PDR_OFS) &= ~mask;
  }

  /* Open-drain (only meaningful for outputs) */
  if (cfg->open_drain) X(base + GPIO_ODR_OFS) |= mask;
  else                 X(base + GPIO_ODR_OFS) &= ~mask;

  /* Drive strength + slew rate */
  _set_drive_strength(base, mask, cfg->drive, cfg->slew_ctl);

  return 0;
}

/* ----------------- I/O helpers ----------------- */
void gpio_write_pin(gpio_port_t port, uint8_t pin, bool high)
{
  uint32_t base = _gpio_base(port);
  uint32_t reg  = X(base + GPIO_DATA_3FC);
  if (high) reg |=  (1u << pin);
  else      reg &= ~(1u << pin);
  X(base + GPIO_DATA_3FC) = reg;
}

bool gpio_read_pin(gpio_port_t port, uint8_t pin)
{
  return (X(_gpio_base(port) + GPIO_DATA_3FC) >> pin) & 1u;
}

void gpio_set_bits(gpio_port_t port, uint8_t mask)
{
  X(_gpio_base(port) + GPIO_DATA_3FC) |= mask;
}

void gpio_clear_bits(gpio_port_t port, uint8_t mask)
{
  X(_gpio_base(port) + GPIO_DATA_3FC) &= ~mask;
}

void gpio_toggle_bits(gpio_port_t port, uint8_t mask)
{
  X(_gpio_base(port) + GPIO_DATA_3FC) ^= mask;
}

void gpio_write_port(gpio_port_t port, uint8_t value)
{
  X(_gpio_base(port) + GPIO_DATA_3FC) = value;
}

uint8_t gpio_read_port(gpio_port_t port)
{
  return (uint8_t)(X(_gpio_base(port) + GPIO_DATA_3FC) & 0xFFu);
}

/* ----------------- Interrupts ----------------- */
void gpio_int_config(gpio_port_t port, uint8_t mask, gpio_int_mode_t mode)
{
  uint32_t base = _gpio_base(port);

  /* Level vs Edge */
  switch (mode) {
    case GPIO_INT_LEVEL_HIGH:
    case GPIO_INT_LEVEL_LOW:
      X(base + GPIO_IS_OFS)  |=  mask; /* level */
      X(base + GPIO_IBE_OFS) &= ~mask; /* no both edges */
      X(base + GPIO_IEV_OFS) = (mode == GPIO_INT_LEVEL_HIGH)
                                   ? (X(base + GPIO_IEV_OFS) |  mask)
                                   : (X(base + GPIO_IEV_OFS) & ~mask);
      break;

    case GPIO_INT_EDGE_BOTH:
      X(base + GPIO_IS_OFS)  &= ~mask; /* edge */
      X(base + GPIO_IBE_OFS) |=  mask; /* both edges */
      break;

    case GPIO_INT_EDGE_RISING:
    case GPIO_INT_EDGE_FALLING:
    default:
      X(base + GPIO_IS_OFS)  &= ~mask; /* edge */
      X(base + GPIO_IBE_OFS) &= ~mask; /* single edge */
      X(base + GPIO_IEV_OFS) = (mode == GPIO_INT_EDGE_RISING)
                                   ? (X(base + GPIO_IEV_OFS) |  mask)
                                   : (X(base + GPIO_IEV_OFS) & ~mask);
      break;
  }

  /* Clear any prior latched edge */
  X(base + GPIO_ICR_OFS) = mask;
}

void gpio_int_enable(gpio_port_t port, uint8_t mask, uint8_t priority)
{
  uint32_t base = _gpio_base(port);
  /* Unmask GPIO interrupt */
  X(base + GPIO_IM_OFS) |= mask;

  int irq = _gpio_irq_num(port);
  if (irq < 0) return;

  /* Set NVIC priority (0..7 into bits [7:5] of the byte slot) */
  uint32_t pri_addr = NVIC_PRI_BASE + (uint32_t)(irq / 4) * 4u;
  uint32_t shift    = (uint32_t)(irq % 4) * 8u + 5u;
  uint32_t pri      = X(pri_addr);
  pri &= ~(0x7u << shift);
  pri |= ((uint32_t)(priority & 0x7u) << shift);
  X(pri_addr) = pri;

  /* Enable NVIC line */
  if (irq < 32) NVIC_EN0_R = (1u << irq);
  else          NVIC_EN1_R = (1u << (irq - 32));
}

void gpio_int_disable(gpio_port_t port, uint8_t mask)
{
  uint32_t base = _gpio_base(port);
  X(base + GPIO_IM_OFS) &= ~mask;
}

void gpio_int_clear(gpio_port_t port, uint8_t mask)
{
  X(_gpio_base(port) + GPIO_ICR_OFS) = mask;
}

uint8_t gpio_int_status(gpio_port_t port, bool masked)
{
  uint32_t base = _gpio_base(port);
  return (uint8_t)((masked ? X(base + GPIO_MIS_OFS)
                           : X(base + GPIO_RIS_OFS)) & 0xFFu);
}
