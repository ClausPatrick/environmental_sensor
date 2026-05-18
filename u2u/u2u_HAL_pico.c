//u2u_HAL_pico.c
#include "u2u_HAL_pico.h"


/* Forward declarations from u2u.h */
uint8_t u2u_write_character(uint8_t port, char ch);
//uint8_t uart0_character_processor(char ch);
//uint8_t uart1_character_processor(char ch);


//message_ready = 0;

void uart0_irq_routine(void){
    //int r;
    //gpio_put(LED_2, 1);
    uint8_t ch;
    while (uart_is_readable(uart0)){
        ch = uart_getc(uart0);
        //r = uart0_character_processor(ch);
        u2u_write_character(0, ch);
    }
}

void uart1_irq_routine(void){
    //int r;
    //gpio_put(LED_3, 1);
    uint8_t ch;
    while (uart_is_readable(uart1)){
        ch = uart_getc(uart1);
        //r = uart1_character_processor(ch);
        u2u_write_character(1, ch);
    }
}

uint8_t write_from_uart0(char* buffer, size_t len){
    uart_puts(uart0, buffer);
    return 0;
}

uint8_t write_from_uart1(char* buffer, size_t len){
    uart_puts(uart1, buffer);
    return 0;
}

uint8_t write_from_uart2(char* buffer, size_t len){
    (void) buffer;
    (void) len;
    return 0;
}

uint8_t write_from_uart(char* buffer, size_t len){
    int r;
    r = write_from_uart0(buffer, len);
    r = write_from_uart1(buffer, len);
    return r;
}

/* Flag for comm logging pdr:
 * bit 0 [1]: PORT, port value.
 * bit 1 [2]: DIR, {0: inbound, 1: outbound}.
 * bit 2 [4]: RFLAG, {For DIR = 0: 0: self or gen addressed, 1: other and forwarded}.
 *               {For DIR = 1: 0: message response,      1: message forward}.
 * bit 3 [8]: ORG, {0: auto,    1: external}.
 *
 * */


void comm_logger(char* buffer, int len, int pdr){
    return;
}

int u2u_uart_setup(void){
    uart_init(uart0, BAUD_RATE);
    uart_init(uart1, BAUD_RATE);
    gpio_set_function(UART_TX_IN, GPIO_FUNC_UART);
    gpio_set_function(UART_TX_OUT, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_IN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_OUT, GPIO_FUNC_UART);
    uart_set_format(uart0, DATA_BITS, STOP_BITS, PARITY);
    uart_set_format(uart1, DATA_BITS, STOP_BITS, PARITY);
    uart_set_hw_flow(uart0, false, false);
    uart_set_hw_flow(uart1, false, false);
    uart_set_fifo_enabled(uart0, false);
    uart_set_fifo_enabled(uart1, false);
    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_routine);
    irq_set_exclusive_handler(UART1_IRQ, uart1_irq_routine);
    irq_set_enabled(UART0_IRQ, true);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
    uart_set_irq_enables(uart1, true, false);
    return 0;
}

int u2u_uart_close(void){
    int r = 0;
    return r;
}
