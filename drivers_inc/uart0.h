
//Maximus "Dumbass" Nwider

#ifndef __UART0_H__
#define __UART0_H__

/*
#define GPIO_PORTA_DATA_BITS_R  ((volatile unsigned long *)0x40004000)
#define GPIO_PORTA_DATA_R       (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_IS_R         (*((volatile unsigned long *)0x40004404))
#define GPIO_PORTA_IBE_R        (*((volatile unsigned long *)0x40004408))
#define GPIO_PORTA_IEV_R        (*((volatile unsigned long *)0x4000440C))
#define GPIO_PORTA_IM_R         (*((volatile unsigned long *)0x40004410))
#define GPIO_PORTA_RIS_R        (*((volatile unsigned long *)0x40004414))
#define GPIO_PORTA_MIS_R        (*((volatile unsigned long *)0x40004418))
#define GPIO_PORTA_ICR_R        (*((volatile unsigned long *)0x4000441C))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DR2R_R       (*((volatile unsigned long *)0x40004500))
#define GPIO_PORTA_DR4R_R       (*((volatile unsigned long *)0x40004504))
#define GPIO_PORTA_DR8R_R       (*((volatile unsigned long *)0x40004508))
#define GPIO_PORTA_ODR_R        (*((volatile unsigned long *)0x4000450C))
#define GPIO_PORTA_PUR_R        (*((volatile unsigned long *)0x40004510))
#define GPIO_PORTA_PDR_R        (*((volatile unsigned long *)0x40004514))
#define GPIO_PORTA_SLR_R        (*((volatile unsigned long *)0x40004518))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_LOCK_R       (*((volatile unsigned long *)0x40004520))
#define GPIO_PORTA_CR_R         (*((volatile unsigned long *)0x40004524))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define GPIO_PORTA_ADCCTL_R     (*((volatile unsigned long *)0x40004530))
#define GPIO_PORTA_DMACTL_R     (*((volatile unsigned long *)0x40004534))
#define SYSCTL_DC4_GPIOA        0x00000001  // GPIO Port A Present
#define SYSCTL_SRCR2_GPIOA      0x00000001  // Port A Reset Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // Port A Clock Gating Control
#define SYSCTL_SCGC2_GPIOA      0x00000001  // Port A Clock Gating Control
#define SYSCTL_DCGC2_GPIOA      0x00000001  // Port A Clock Gating Control
*/


#define X(x)( * (volatile uint32_t * )(x))

#define SYSCTL_RCGCUART X       (0x400FE618)
#define SYSCTL_RCGCGPIO X       (0x400FE608)

#define GPIO_PORTA_AFSEL X      (0x40004420)
#define GPIO_PORTA_DEN X        (0x4000451C)
#define GPIO_PORTA_PCTL X       (0x4000452C)
#define UART0_DR X              (0x4000C000)
#define UART0_FR X              (0x4000C018)
#define UART0_IBRD X            (0x4000C024)
#define UART0_FBRD X            (0x4000C028)
#define UART0_LCRH X            (0x4000C02C)
#define UART0_CTL X             (0x4000C030)
#define UART0_CC X              (0x4000CFC8)

void uart0_init_115200();
void uart0_putc(char c);
void uart0_write(const char * s);
void uart0_write_int(int v);

#endif