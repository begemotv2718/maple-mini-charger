#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include "queue.h"
#include "usart_irq.h"

struct Queue UART_TXq, UART_RXq;

volatile int TxReenable =0;
volatile int RxOverflow =0;

int usart_getc(void){
  uint8_t data;
  while(!Dequeue(&UART_RXq,&data));
  return data;
  
}

int usart_poll_getc(uint8_t *data){
  return Dequeue(&UART_RXq,data);
}

void usart_putc(int c){
  while(! Enqueue(&UART_TXq,c));
  if(!TxReenable){
    TxReenable = 1;
	usart_enable_tx_interrupt(USART2);
  }
}

void usart_setup(void)
{
	rcc_periph_clock_enable(RCC_USART2);
	/* Enable the USART2 interrupt. */
	nvic_enable_irq(NVIC_USART2_IRQ);

	/* Setup GPIO pins for USART2 transmit. */
    gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO_USART2_TX);
	//gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);

	/* Setup GPIO pins for USART2 receive. */
    gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT,GPIO_USART2_RX);


	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Enable USART2 Receive interrupt. */
	usart_enable_rx_interrupt(USART2);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

void usart2_isr(void)
{
	static uint8_t ldata = 'A';

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		//gpio_toggle(GPIOD, GPIO12);

		/* Retrieve the data from the peripheral. */
		ldata = usart_recv(USART2);

        if(!Enqueue(&UART_RXq,ldata)){
            RxOverflow = 1;
        }
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {
        if(Dequeue(&UART_TXq,&ldata)){
		   /* Put data into the transmit register. */
		   usart_send(USART2, ldata);
        }else{
          /* Disable the TXE interrupt as we don't need it anymore. */
          usart_disable_tx_interrupt(USART2);
          TxReenable = 0;
        }
    }
	
}

