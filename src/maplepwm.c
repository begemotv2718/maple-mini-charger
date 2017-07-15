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


static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable TIM3 clock. */
	rcc_periph_clock_enable(RCC_TIM4);

	/* Enable GPIOC, Alternate Function clocks. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
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


}

static void tim_setup(void)
{
	/* Clock division and mode */
	TIM4_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Period */
	TIM4_ARR = 65535;
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

int main(void)
{
	int i, j0, j1, d0, d1;

	clock_setup();
	gpio_setup();
	tim_setup();

	j0 = 0;
	d0 = 1;
	j1 = 128;
	d1 = 1;
	while (1) {
        if(d0>0){
		   TIM4_CCR1 = 500;
        }else{
           TIM4_CCR1 = 100;
        }
		j0 += d0;
		if (j0 == 255)
			d0 = -1;
		if (j0 == 0)
			d0 = 1;
		TIM4_CCR2 = 600;
		j1 += d1;
		if (j1 == 255)
			d1 = -1;
		if (j1 == 0)
			d1 = 1;
	}


	return 0;
}
