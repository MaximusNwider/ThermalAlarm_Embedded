

#ifndef __GPIO_H__
#define __GPIO_H__

#include<stdint.h>
#include<stdbool.h>

/* ---------- Ports ---------- */
typedef enum {
  GPIO_PORT_A = 0,
  GPIO_PORT_B = 1,
  GPIO_PORT_C = 2,
  GPIO_PORT_D = 3,
  GPIO_PORT_E = 4,
  GPIO_PORT_F = 5,
} gpio_port_t;

/* ---------- Drive strength (choose exactly one) ---------- */
typedef enum {
  GPIO_DRIVE_2MA = 2,
  GPIO_DRIVE_4MA = 4,
  GPIO_DRIVE_8MA = 8,
} gpio_drive_t;

/* ---------- Interrupt trigger modes ---------- */
typedef enum {
  GPIO_INT_EDGE_RISING,
  GPIO_INT_EDGE_FALLING,
  GPIO_INT_EDGE_BOTH,
  GPIO_INT_LEVEL_HIGH,
  GPIO_INT_LEVEL_LOW
} gpio_int_mode_t;

/* ---------- Pin configuration descriptor ---------- */
typedef struct {
  uint8_t       pins;           /* bitmask: (1<<pin) for pins 0..7 */
  bool          dir_output;     /* true = output, false = input */
  bool          digital_enable; /* true => DEN=1 (ignored if analog_enable=true) */
  bool          analog_enable;  /* true => AMSEL=1, DEN=0 (analog path) */
  bool          alt_enable;     /* true => AFSEL=1; set pctl_nibble accordingly */
  uint8_t       pctl_nibble;    /* 0..15 function code applied to each selected pin */
  bool          pull_up;        /* true => PUR=1 (exclusive with pull_down) */
  bool          pull_down;      /* true => PDR=1 */
  bool          open_drain;     /* true => ODR=1 (only meaningful for outputs) */
  gpio_drive_t  drive;          /* 2/4/8 mA */
  bool          slew_ctl;       /* true => SLR=1 (slew rate enable for 8mA fast edges) */
  bool          unlock_protected; /* true => unlock+commit PF0/PD7 (and PC0–3 if desired) */
} gpio_pin_cfg_t;

/* ---------- Port clocking ---------- */
void gpio_port_enable(gpio_port_t port);

/* ---------- Configure pads & mux ---------- */
int  gpio_configure(gpio_port_t port, const gpio_pin_cfg_t *cfg);

/* ---------- Simple I/O helpers ---------- */
void gpio_write_pin(gpio_port_t port, uint8_t pin /*0..7*/, bool high);
bool gpio_read_pin (gpio_port_t port, uint8_t pin /*0..7*/);
void gpio_set_bits  (gpio_port_t port, uint8_t mask);
void gpio_clear_bits(gpio_port_t port, uint8_t mask);
void gpio_toggle_bits(gpio_port_t port, uint8_t mask);
void gpio_write_port(gpio_port_t port, uint8_t value);
uint8_t gpio_read_port(gpio_port_t port);

/* ---------- Interrupts ---------- */
void gpio_int_config (gpio_port_t port, uint8_t mask, gpio_int_mode_t mode);
void gpio_int_enable (gpio_port_t port, uint8_t mask, uint8_t nvic_priority /*0=highest..7*/);
void gpio_int_disable(gpio_port_t port, uint8_t mask);
void gpio_int_clear  (gpio_port_t port, uint8_t mask);
uint8_t gpio_int_status(gpio_port_t port, bool masked /*true=>MIS, false=>RIS*/);

#endif
