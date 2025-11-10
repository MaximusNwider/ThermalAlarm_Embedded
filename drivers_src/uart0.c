
//Maximus "Dumbass" Nwider
#include  <stdint.h>
#include  "../drivers_inc/uart0.h"

#define WORD_LEN                    (0x3)
#define S_BIT                       (0x1)
#define D_PORT                      (0x3)
#define TXFE                        (1<<7)
#define RXFF                        (1<<6)
#define TXFF                        (1<<5)
#define RXFE                        (1<<4)
#define BUSY                        (1<<3)

/*

To enable and initialize the UART, the following steps are necessary:
 1. Enable the UART module using the RCGCUART register (see page 344). 
 2. Enable the clock to the appropriate GPIO module via the RCGCGPIO register (see page 340). To find out which GPIO port to enable, refer to Table 23-5 on page 1351.
 3. SettheGPIOAFSELbitsfortheappropriate pins (see page 671). To determine which GPIOs to configure, see Table 23-4 on page 1344.
 4. Configure the GPIO current level and/or slew rate as specified for the mode selected (see page 673 and page 681). 902 June 12, 2014 Texas Instruments-Production DataTiva™ TM4C123GH6PM Microcontroller
 5. ConfigurethePMCnfieldsin the GPIOPCTLregistertoassigntheUARTsignalstotheappropriate pins (see page 688 and Table 23-5 on page 1351).


*/

uint16_t i_brate = 43;
uint16_t f_brate = 26;

void uart0_init_115200() {

  //Clock control to Port A, module 1, for  GPIO and UART

  //UART transmission set in run mode clock gating ctrl
  SYSCTL_RCGCUART |= S_BIT;
  //GPIO ports set in run mode clk gating ctrl
  SYSCTL_RCGCGPIO |= S_BIT;
  //AFSEL enabled for last 2 bits for PA0 and PA1 --> 01 PA0 = RX, PA1 = TX
  GPIO_PORTA_AFSEL |= S_BIT | (1 << 1);
  //DEN enabled for last 2 bits for PA0 and PA1 for UART
  GPIO_PORTA_DEN |= S_BIT | (1 << 1);
  //Kill any bit previously set, and ensure only last 2 bits are set for UART ctrl
  GPIO_PORTA_PCTL |= ~(GPIO_PORTA_PCTL &= (0xFF)) | D_PORT;

  /*
  TheUARTCTLregistershouldnotbechangedwhiletheUARTisenabledorelsetheresults are unpredictable.
  Thefollowingsequenceisrecommendedformakingchangestothe UARTCTLregister.

   1. DisabletheUART. 
   2. Wait for the end of transmissionorreceptionofthecurrentcharacter. 
   3. FlushthetransmitFIFObyclearingbit4(FEN)inthelinecontrolregister(UARTLCRH).
   4. Reprogramthecontrolregister.
   5. EnabletheUART.
  */

  //Ensure UArt module is disabled 
  UART0_CTL &= ~D_PORT;
  //TODO: ensure waiting to end of trans/receipt

  //Modify i_brate and f_brate depending on desired baudrate
  //integer baudrate divisor
  UART0_IBRD = i_brate;
  //fractional baudrate divisor
  UART0_FBRD = f_brate;
  //Flush FIFO, by clearing bit 4, I really dont like this assingment MAXIMUS change this 
  UART0_LCRH = (WORD_LEN << 5) | (S_BIT << 4);
  //enable clock reg, and just use sys clock
  UART0_CC |= (S_BIT >> 1);
  //reactivate uart module, TXE,RXE
  UART0_CTL |= (D_PORT << 8) | S_BIT;

}

void uart0_putc(char c) {

  //First UART must be verified to be active, and allowed to continue only while so

  /*
  Afterreset,
  theTXFF,RXFF,andBUSYbitsare0,
  and TXFEandRXFEbitsare1.
  */

  //Ensure bugless condition
  while ((~UART0_FR) & (TXFF)) {
    //ensure this is safe, no actually do it, in addition to my clear to send condition
    if ((UART0_FR | S_BIT)) {
      //I dont like this direct assignment, validate
      UART0_DR = (c);
    }

  }

}

void uart0_write(const char * s) {

  //MUST keep track of size while calling putc TODO
  while ( * s) {
    uart0_putc( * s++);
  }

}

//todo replace this garbage
void uart0_write_int(int v) {

  char b[16];
  int i = 15;
  b[i--] = 0;
  int n = v;

  if (n == 0) {
    uart0_putc('0');
    return;
  }

  int s = n < 0;
  if (s) n = -n;

  while (n && i >= 0) {
    b[i--] = '0' + (n % 10);
    n /= 10;
  }
  if (s) b[i--] = '-';
  uart0_write( & b[i + 1]);

}








