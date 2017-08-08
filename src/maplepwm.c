/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "usart_irq.h"
#include "systick_alarm.h"
#include "ds_usart.h"

int _write(int file, char *ptr, int len);


volatile uint32_t accum_measure=0;
volatile uint32_t accum_input=0;
volatile uint32_t count_measure=0;
volatile uint32_t count_input=0;
volatile    int32_t K_p=351;
#define ACCUM_VALUE_MEASURE (1000u)
#define ACCUM_VALUE_INPUT (5u)
#define SET_VALUE (425)
volatile uint16_t chans[4];
#define RING_BUFFER_SIZE 1000
volatile uint16_t adc_values[RING_BUFFER_SIZE];
volatile int16_t delta_values[RING_BUFFER_SIZE];
volatile uint16_t tim_ccr_values[RING_BUFFER_SIZE];
volatile uint16_t ring_buffer_index=0;
volatile uint16_t tim_ccr_value=10;
volatile int ring_write_allowed=1;
volatile int output_allowed=1;
volatile int measure_in_progress=0;


static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable TIM3 clock. */
	rcc_periph_clock_enable(RCC_TIM4);

	/* Enable GPIOC, Alternate Function clocks. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_ADC1);
}

static void gpio_setup(void)
{
	/*
	 * Set GPIO6 and 7 (in GPIO port A) to
	 * 'output alternate function push-pull'.
	 */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_TIM4_CH1 | GPIO_TIM4_CH2);


    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO0|GPIO14|GPIO1);
}

void adc1_2_isr(void){
    int i;
    int16_t curr_ccr2;
    int16_t delta;
    uint16_t average_adc;
    for(i=1;i<=4;i++){
       chans[i-1]=adc_read_injected(ADC1,i);
    }
    // TIM4_CCR2=400;
    if(output_allowed){
      accum_input+=chans[1];
      count_input++;
      if(count_input>=ACCUM_VALUE_INPUT){
        count_input=0;
        curr_ccr2=tim_ccr_value;
        average_adc=accum_input/ACCUM_VALUE_INPUT;
        delta=(K_p*(SET_VALUE-average_adc))>>10;
        curr_ccr2+=delta;
        if(curr_ccr2<0){curr_ccr2=0;}
        if(curr_ccr2>4095){curr_ccr2=4095;}
        accum_input=0;
        gpio_toggle(GPIOB,GPIO0);
        tim_ccr_value=(uint16_t)curr_ccr2;
        TIM4_CCR2=tim_ccr_value;
        if(ring_write_allowed){      
            adc_values[ring_buffer_index]=average_adc;
            delta_values[ring_buffer_index]=delta;
            tim_ccr_values[ring_buffer_index]=curr_ccr2;
            ring_buffer_index++;
            if(ring_buffer_index>=RING_BUFFER_SIZE){
              ring_buffer_index=0;
            }
        }
      }
    }else{
      if(TIM4_CCR2>0){
        TIM4_CCR2=0;
      }
      tim_ccr_value=0;
    }

    if(measure_in_progress){
      accum_measure+=chans[0];
      count_measure++;
      if(count_measure>=ACCUM_VALUE_MEASURE){
        count_measure=0;
        measure_in_progress=0;
      }
    }


  

    ADC_SR(ADC1) &= ~ADC_SR_JEOC; // Clear JEOC flag
}

static void tim_setup(void)
{
	/* Clock division and mode */
	TIM4_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Period */
	TIM4_ARR = 4095;
	/* Prescaler */
	TIM4_PSC = 0;
	TIM4_EGR = TIM_EGR_UG;

	/* ---- */
	/* Output compare 1 mode and preload */
	TIM4_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;

	/* Polarity and state */
	TIM4_CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;
	//TIM4_CCER |= TIM_CCER_CC1E;

	/* Capture compare value */
	TIM4_CCR1 = 0;

	/* ---- */
	/* Output compare 2 mode and preload */
	TIM4_CCMR1 |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;

	/* Polarity and state */
	TIM4_CCER |= TIM_CCER_CC2P | TIM_CCER_CC2E;
	//TIM4_CCER |= TIM_CCER_CC2E;

	/* Capture compare value */
	TIM4_CCR2 = 0;

	/* ---- */
	/* ARR reload enable */
	TIM4_CR1 |= TIM_CR1_ARPE;

	/* Counter enable */
	TIM4_CR1 |= TIM_CR1_CEN;
}

static void adc_tim_setup(void){
	/* Clock division and mode */
	TIM2_CR1 = TIM_CR1_CKD_CK_INT_MUL_4 | TIM_CR1_CMS_EDGE | TIM_CR1_DIR_UP;
	/* Period */
    TIM2_ARR=65531;
    TIM2_PSC=0;
    //TIM2_EGR = TIM2_EGR_UG;
    TIM2_CR2 |=TIM_CR2_MMS_UPDATE;
    TIM2_CR1 |=TIM_CR1_CEN;
}

static void adc_setup(void){


    nvic_set_priority(NVIC_ADC1_2_IRQ, 0);
    nvic_enable_irq(NVIC_ADC1_2_IRQ);

    gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO0);
    gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO1);
    gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO4);
    gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO5);

    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);

    adc_enable_external_trigger_injected(ADC1,ADC_CR2_JEXTSEL_TIM2_TRGO);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_55DOT5CYC);
    uint8_t channels[16];
    channels[0]=ADC_CHANNEL0;
    channels[1]=ADC_CHANNEL1;
    channels[2]=ADC_CHANNEL4;
    channels[3]=ADC_CHANNEL5;
    adc_set_injected_sequence(ADC1, 4, channels);

    adc_power_on(ADC1);
	/* Wait for ADC starting up. */
    int i;
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");
    adc_reset_calibration(ADC1);
    adc_calibration(ADC1);

    //adc_enable_eoc_interrupt(ADC1);
    adc_enable_eoc_interrupt_injected(ADC1);
    gpio_toggle(GPIOB,GPIO1);
}

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_putc(ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}

static void read_string(int len, char *my_buffer){
	uint8_t pos=0;
    uint8_t i;
    usart_putc('\r');
    usart_putc('>');
	//_write(1,"Command> ",sizeof("Command> ")-1);
	char c=usart_getc();
	while(c!=13 && pos<len-1){
		usart_putc(c);
		if(c!=127/*backspace*/){
			my_buffer[pos]=c;
			pos++;
		}else{
			my_buffer[pos]=0;
			if(pos>0) pos--;
			my_buffer[pos]=0;
            usart_putc('\033');
            usart_putc('[');
            usart_putc('2');
            usart_putc('K');
            usart_putc('\r');
//          printf("\033[2K\r");
			usart_putc('>');
            for(i=0;i<pos;i++){
               usart_putc(my_buffer[i]);
            } 
		}
	  c = usart_getc();
	}
    usart_putc('\n');
    usart_putc('\r');
	my_buffer[pos]=0;
	return;
}

static void print_ring_buf(void){
  int index;
  ring_write_allowed=0;
  printf("ADC\t\tdelta\t\ttimer\r\n");
  for(int i=0;i<100;i++){
      index=(ring_buffer_index+RING_BUFFER_SIZE-i)%RING_BUFFER_SIZE;
      printf("%d\t\t%d\t\t%d\r\n",adc_values[index],delta_values[index],tim_ccr_values[index]);
  }
  ring_write_allowed=1;

}

static void print_param(void){
  printf("K_p=%ld\r\n",K_p);
}

uint8_t dallas_data[32];
const uint8_t dallas_rom_code[]={0x28,0x44,0x0C,0x6C,0x07,0x00,0x00,0x6E};

static void dallas_get_rom(void){
  if(dallas_reset()==0){
    printf("No temp measuring device present!\r\n");
    return;
  }
  dallas_data[0]=0x33;
  dallas_send(dallas_data,1);
  dallas_recv(dallas_data,8);
  printf("Rom code: ");
  for(int i=0;i<8;i++){
    printf("%02Xh ",dallas_data[i]);
  }
}

static void dallas_measure_t(void){
  if(dallas_reset()==0){
    printf("No temp measuring device present!\r\n");
    return;
  }
  dallas_data[0]=0xCC;
  dallas_data[1]=0x44;
  dallas_send(dallas_data,2);
}

static void dallas_read_scratch(void){
  if(dallas_reset()==0){
    printf("No temp measuring device present!\r\n");
    return;
  }
  dallas_data[0]=0xCC;
  dallas_data[1]=0xBE;
  dallas_send(dallas_data,2);
  dallas_recv(dallas_data,9);
  printf("Scratchpad values: ");
  for(int i=0;i<9;i++){
    printf("%02Xh ",dallas_data[i]);
  }
  printf("Temperature: %02d.%04d\r\n",(((uint16_t)dallas_data[1])<<4)+(((uint16_t)dallas_data[0])>>4),(uint16_t)625*(dallas_data[0]&0x0f));
}


static void dallas_read_scratch_prefix(void){
  int offset=0;
  if(dallas_reset()==0){
    printf("No temp measuring device present!\r\n");
    return;
  }
  dallas_data[0]=0x55;
  for(uint16_t i=0;i<sizeof(dallas_rom_code)/sizeof(dallas_rom_code[0]);i++){
    offset++;
    dallas_data[offset]=dallas_rom_code[i];
  }
  offset++;
  dallas_data[offset]=0xBE;
  offset++;
  dallas_send(dallas_data,offset);
  dallas_recv(dallas_data,9);
  printf("Scratchpad values: ");
  for(int i=0;i<9;i++){
    printf("%02Xh ",dallas_data[i]);
  }
  printf("Temperature: %02d.%04d\r\n",(((uint16_t)dallas_data[1])<<4)+(((uint16_t)dallas_data[0])>>4),(uint16_t)625*(dallas_data[0]&0x0f));
}
static void do_measure(void){
  if(dallas_reset()!=0){
    dallas_data[0]=0xCC;
    dallas_data[1]=0x44;
    dallas_send(dallas_data,2);
  }

  accum_measure=0;
  measure_in_progress=1;
  while(measure_in_progress);
  printf("Measured voltage: %ld",accum_measure*31/40960);

  if(dallas_reset()!=0){
    int offset=0;
    dallas_data[0]=0x55;
    for(uint16_t i=0;i<sizeof(dallas_rom_code)/sizeof(dallas_rom_code[0]);i++){
      offset++;
      dallas_data[offset]=dallas_rom_code[i];
    }
    offset++;
    dallas_data[offset]=0xBE;
    offset++;
    dallas_send(dallas_data,offset);
    dallas_recv(dallas_data,9);
    printf("Temperature: %02d.%04d\r\n",(((uint16_t)dallas_data[1])<<4)+(((uint16_t)dallas_data[0])>>4),(uint16_t)625*(dallas_data[0]&0x0f));
  }
}

enum State { START, CHARGE, PREPARE_MEASURE, MEASURE, CMDLINE};
enum State state=START;
#define CHARGE_TIME (50000u)
#define RELAX_TIME (3000u)

char cmdline[256];
extern volatile uint32_t tick_counter;

int main(void)
{

	clock_setup();
	gpio_setup();
	tim_setup();
    adc_setup();
    adc_tim_setup();
    usart_setup();
    systick_setup();
    dallas_usart_setup();

    TIM4_CCR1 = 10000;
	TIM4_CCR2 = 32767;

    uint8_t data;
    

    while(1){
      switch(state){
        case START:
          set_systick_timeout(CHARGE_TIME);
          state=CHARGE;
          gpio_set(GPIOB,GPIO14);
          break;
        case CHARGE:
          if(is_systick_timeout()){
            set_systick_timeout(RELAX_TIME);
            gpio_clear(GPIOB,GPIO14);
            output_allowed=0;
            state=PREPARE_MEASURE;
          }
          if(usart_poll_getc(&data)){
            printf("CD:%02X\r\n",(int)data);
            if('c'==data){
              state=CMDLINE;
            }
          }
          break;
        case PREPARE_MEASURE:
          if(is_systick_timeout()){
            state=MEASURE;
          }
          if(usart_poll_getc(&data)){
            printf("PD:%02X\r\n",(int)data);
            if('c'==data){
              state=CMDLINE;
            }
          }
          break;
        case MEASURE:
          if(usart_poll_getc(&data)){
            printf("MD:%02X\r\n",(int)data);
            if('c'==data){
              state=CMDLINE;
            }
          }
          if(CMDLINE!=state){
              do_measure();
              set_systick_timeout(CHARGE_TIME);
              gpio_set(GPIOB,GPIO14);
              output_allowed=1;
              state=CHARGE;
          }

          break;
        case CMDLINE:
          printf("cmd:\n\r");
          read_string(255,cmdline);
          if(strncmp(cmdline,"ring",5)==0){
            print_ring_buf();
          }else if(strncmp(cmdline,"kp",3)==0){
            print_param();
          }else if(strncmp(cmdline,"rom_code",9)==0){
            dallas_get_rom();
          }else if(strncmp(cmdline,"meas_t",7)==0){
            dallas_measure_t();
          }else if(strncmp(cmdline,"scratch",8)==0){
            dallas_read_scratch();
          }else if(strncmp(cmdline,"scr_prefix",11)==0){
            dallas_read_scratch_prefix();
          }else if(strncmp(cmdline,"exit",5)==0){
            set_systick_timeout(100000);
            state=CHARGE;
          }
          printf("Read data %s\n\r",cmdline);
        default:
          break;
      }
    }
      

	return 0;
}
