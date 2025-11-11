#include <stdint.h>
extern int main(void);
extern uint32_t _etext,_sdata,_edata,_sbss,_ebss,_estack;
void Default_Handler(void);
void Reset_Handler(void);
void NMI_Handler(void)__attribute__((weak,alias("Default_Handler")));
void HardFault_Handler(void)__attribute__((weak,alias("Default_Handler")));
void MemManage_Handler(void)__attribute__((weak,alias("Default_Handler")));
void BusFault_Handler(void)__attribute__((weak,alias("Default_Handler")));
void UsageFault_Handler(void)__attribute__((weak,alias("Default_Handler")));
void SVC_Handler(void)__attribute__((weak,alias("Default_Handler")));
void DebugMon_Handler(void)__attribute__((weak,alias("Default_Handler")));
void PendSV_Handler(void)__attribute__((weak,alias("Default_Handler")));
void SysTick_Handler(void)__attribute__((weak,alias("Default_Handler")));
void TIMER0A_Handler(void)__attribute__((weak,alias("Default_Handler")));
__attribute__((section(".isr_vector")))
void(*const gVectors[])(void)={
 (void(*)(void))(&_estack), Reset_Handler, NMI_Handler, HardFault_Handler,
 MemManage_Handler, BusFault_Handler, UsageFault_Handler, 0,0,0,0,
 SVC_Handler, DebugMon_Handler, 0, PendSV_Handler, SysTick_Handler,
 /* IRQs up to TIMER0A (vector 19); simple table keeps default handlers */
 [19]=TIMER0A_Handler
};
void Reset_Handler(void){
  uint32_t *s=&_etext; for(uint32_t *d=&_sdata; d<&_edata; ) *d++=*s++;
  for(uint32_t *d=&_sbss; d<&_ebss; ) *d++=0;
  main(); while(1){}
}
void Default_Handler(void){ while(1){ __asm volatile("nop"); } }
