//u2u_HAL_pico.h
#ifndef U2U_HAL_H
#define U2U_HAL_H


#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#include "u2u_client_profile.h"


#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_IN uart1
#define UART_OUT uart0

/*From u2u.h*/
uint8_t uart0_character_processor(char ch);
uint8_t uart1_character_processor(char ch);

void uart0_irq_routine(void);
void uart1_irq_routine(void);
uint8_t write_from_uart(char* buffer, size_t len);
uint8_t write_from_uart0(char* buffer, size_t len);
uint8_t write_from_uart1(char* buffer, size_t len);
uint8_t write_from_uart2(char* buffer, size_t len);
void comm_logger(char* buffer, int len, int pdr);
//void inbound_message_logger(char* buffer, int len);
int u2u_uart_setup();

//int message_ready;
int u2u_uart_close(void);

#endif
