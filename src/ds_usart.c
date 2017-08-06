#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "ds_usart.h"

/*
#define DS_GPIO GPIOB
#define DS_TX_PIN GPIO10
#define DS_RCC RCC_GPIOB
#define DS_USART USART3
#define DS_USART_RCC RCC_USART3

*/

#define DS_GPIO GPIOA
#define DS_TX_PIN GPIO9
#define DS_RX_PIN GPIO10
#define DS_RCC RCC_GPIOA
#define DS_USART USART1
#define DS_USART_RCC RCC_USART1



uint16_t usart_recv_tc_complete(uint32_t usart){
  while((USART_SR(usart) & USART_SR_TC) ==0);
  usart_wait_recv_ready(usart);
  return usart_recv(usart);
}
  

void dallas_usart_setup(void){
   rcc_periph_clock_enable(DS_USART_RCC);
   rcc_periph_clock_enable(DS_RCC);
   gpio_set_mode(DS_GPIO, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, DS_TX_PIN);
   gpio_set_mode(DS_GPIO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, DS_RX_PIN);
   //gpio_mode_setup(DS_GPIO,GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO9); /*RX*/

   //gpio_set_af(DS_GPIO, GPIO_AF7, GPIO9);
   

   usart_set_baudrate(DS_USART, 9600);
   usart_set_databits(DS_USART, 8);
   usart_set_stopbits(DS_USART, USART_STOPBITS_1);
   usart_set_mode(DS_USART, USART_MODE_TX_RX);
   usart_set_parity(DS_USART, USART_PARITY_NONE);
   usart_set_flow_control(DS_USART, USART_FLOWCONTROL_NONE);

   //usart_half_duplex_mode_enable(DS_USART);//??
   usart_enable(DS_USART);
}


int dallas_reset(void){
  uint8_t response;
  usart_disable(DS_USART);
  usart_set_baudrate(DS_USART,9600);
   usart_set_databits(DS_USART, 8);
   usart_set_stopbits(DS_USART, USART_STOPBITS_1);
   usart_set_mode(DS_USART, USART_MODE_TX_RX);
   usart_set_parity(DS_USART, USART_PARITY_NONE);
   usart_set_flow_control(DS_USART, USART_FLOWCONTROL_NONE);
   usart_enable(DS_USART);
   USART_SR(DS_USART)&=~USART_SR_TC;
  usart_send(DS_USART,0xf0);
  response = usart_recv_tc_complete(DS_USART);
 // for(uint32_t i=0;i<720000;i++)
 //   __asm__("nop");
  usart_disable(DS_USART);
  usart_set_baudrate(DS_USART,115200);
   usart_set_databits(DS_USART, 8);
   usart_set_stopbits(DS_USART, USART_STOPBITS_1);
   usart_set_mode(DS_USART, USART_MODE_TX_RX);
   usart_set_parity(DS_USART, USART_PARITY_NONE);
   usart_set_flow_control(DS_USART, USART_FLOWCONTROL_NONE);
   usart_enable(DS_USART);
  //return response!=0xf0?response:0;
  return (response!=0xf0)?response:0;
}

void dallas_send(uint8_t *command, int len){
  uint8_t cur_byte;
  int bit_pos;
  while(len>0){
    for(bit_pos=0;bit_pos<8;bit_pos++){
      cur_byte = ((*command)&(1<<bit_pos))?0xff:0x00;
      USART_SR(DS_USART)&=~USART_SR_TC;
      usart_send_blocking(DS_USART,cur_byte);
      usart_recv_tc_complete(DS_USART);
    }
    command++;
    len--;
  }
}

void dallas_recv(uint8_t *buffer,int len){
   uint8_t response;
   uint8_t cur_byte;
   int bit_pos;
   while(len>0){
     cur_byte=0;
     for(bit_pos=0;bit_pos<8;bit_pos++){
       USART_SR(DS_USART)&=~USART_SR_TC;
       usart_send(DS_USART,0xff);
       response=usart_recv_tc_complete(DS_USART);
       if(response==0xff){
         cur_byte|=1<<bit_pos;
       }
     }
     *buffer=cur_byte;
     buffer++;
     len--;
   }
}


