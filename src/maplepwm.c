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

volatile uint32_t accum=0;
volatile uint32_t count=0;
volatile uint32_t count2=0;
#define ACCUM_VALUE 100
volatile uint16_t chans[4];


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
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      GPIO_TIM4_CH1 | GPIO_TIM4_CH2);


    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO0);
}

void adc1_2_isr(void){
    int i;
    for(i=1;i<=4;i++){
       chans[i-1]=adc_read_injected(ADC1,i);
    }
    accum+=chans[0];
   // TIM4_CCR2=400;
    count++;
    if(count>=ACCUM_VALUE){
      TIM4_CCR1=300;
      TIM4_CCR2=accum/ACCUM_VALUE;
      count=0;
      accum=0;
      count2++;
      if(count2>50000){
        count2=0;
      }
    }


  
    gpio_toggle(GPIOB,GPIO0);

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
    gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO2);
    gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO3);

    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);

    adc_enable_external_trigger_injected(ADC1,ADC_CR2_JEXTSEL_TIM2_TRGO);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_55DOT5CYC);
    uint8_t channels[16];
    channels[0]=ADC_CHANNEL0;
    channels[1]=ADC_CHANNEL1;
    channels[2]=ADC_CHANNEL2;
    channels[3]=ADC_CHANNEL3;
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
}

int main(void)
{
	int i, j0, j1, d0, d1;

	clock_setup();
	gpio_setup();
	tim_setup();
    adc_setup();
    adc_tim_setup();

    TIM4_CCR1 = 10000;
	TIM4_CCR2 = 32767;
    
    while(1);


	return 0;
}
