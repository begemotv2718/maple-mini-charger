#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include "systick_alarm.h"


volatile uint32_t tick_counter=0;
void systick_setup(void){
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(SYSTICK_RELOAD_PERIOD);
	systick_interrupt_enable();
	systick_counter_enable();
}

void sys_tick_handler(void){
   if(tick_counter>0){
      tick_counter--;
   }
}

int is_systick_timeout(void){
  return !(tick_counter);
}

void set_systick_timeout(uint32_t timeout){
  tick_counter=timeout;
}
