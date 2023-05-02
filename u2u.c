#ifndef U2U_C
#define U2U_C

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include <stdio.h>

//#include <stdlib.h>
//#include <stdbool.h>


#include "u2uclientdef.h"
#include "u2u.h"
//const char self_name[] = "BLO";

#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_IN uart1    //to DS9
#define UART_OUT uart0


#define TOPIC_AMOUNT 7
#define msg_HAIL 0
#define msg_HELP 1
#define msg_SET_LCD 2
#define msg_SET_OLED 3
#define msg_GET_SENSOR 4
#define msg_SET_ENCODER 5
#define msg_SET_LED 6


//const char* topic_list[] = { "HAIL", "HELP", "SET LCD", "SET OLED", "GET SENSOR", "SET LED", "SET ENCODER"};


const unsigned char CRC7_POLY = 0x91;

struct    Message{
    int   Index;
    char  Segments[10][32];
    char* Payload;
    int   Port;
    int   intLength;
    int   intCRC;
    char  CRC_buffer[255];
    int   CRC_index;
    int   Topic_number;
};

//{struct Message m0, struct Message m1, struct Message m2, struct Message m3, struct Message m4, struct Message m5, struct Message m6, struct Message m7, struct Message m8, struct Message m9, struct Message m10, struct Message m11, struct Message m12, struct Message m13, struct Message m14, struct Message m15};
struct  Message  m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14, m15;
struct  Message* messages[16] = {&m0, &m1, &m2, &m3, &m4, &m5, &m6, &m7, &m8, &m9, &m10, &m11, &m12, &m13, &m14, &m15}; char    payloads[16][255];

char    message_segment_labels[11][32] = {"#Premessage#", "#Sender#", "#Receiver#",  "#RQS#",  "#Topic#",  "#Chapter#",  "#Payload_Length#",  "#Payload_Data#",  "#Hopcount#",  "#CRC#", "#ENDOFMESSAGE#"};

const char RQS_r[] = "RS";
const char RQS_q[] = "RQ";
const char RQS_i[] = "RI";
const char RQS_n[] = "NA";

uint8_t  message_counter_global = 0;
uint8_t  segment_counter[2] = {0, 0};
uint8_t  segment_length[2]  = {0, 0};
uint8_t  message_counter_port[2] = {0, 0};
uint8_t  segment_max_length[] = {1, 32, 32, 4, 32, 4, 4, 255, 4, 4};


uint8_t  (*parse_function_array[12])(uint8_t, int, char);
uint8_t  (*call_router_functions[8])(uint8_t, int);
uint8_t  (*uart_write_functions[2])(char*);


struct Message_out  m_o0, m_o1, m_o2, m_o3, m_o4, m_o5, m_o6, m_o7, m_o8, m_o9, m_o10, m_o11, m_o12, m_o13, m_o14, m_o15;
struct Message_out* messages_out[16] = {&m_o0, &m_o1, &m_o2, &m_o3, &m_o4, &m_o5, &m_o6, &m_o7, &m_o8, &m_o9, &m_o10, &m_o11, &m_o12, &m_o13, &m_o14, &m_o15};


int ascii_to_int(char* str){
    int len = 0;
    int sum = 0;
    int decs[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    while (str[len] != 0){
        len++;
    }
    for (int c=0; c<len; c++){
        sum = sum + (decs[len-c-1] * (str[c]-48));
    }
    //if (enforce_max && rx_text_len > MESSAGE_MAX_LENGTH){
    //    rx_text_len = MESSAGE_MAX_LENGTH;
    //}
    return sum;
}

void int_to_ascii(char* buf, int integer, int p) { // If p is 0 then p is determined.
        int tens[] = {10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};
        int f = 0; //p denotes 'precision' or number of digits.
        int c = 0;
        int j = 0;
        const int M = 7;
        if (integer==0){ // Going to be zero often, so cutting this short.
            buf[0] = 48;
            buf[1] = '\0';
        }else{
            if (p==0){
                for (j=M; j>0; --j){
                    if (integer >= tens[j]){
                        p++;
                    }else{
                        break;
                    }
                }
            }
            f = M-p;
            for (c=0; c<p; c++) {
                buf[c] = 48 + (integer % tens[f+c] / (tens[f+c+1]));
            }
            buf[c] = '\0';
        }
        return;
}


//void int_to_ascii(char* buf, int i, int p) {
//        int tens[] = {100000, 10000, 1000, 100, 10, 1};
//        int f = 5-p; //p denotes 'precision' or number of digits: log-base10(i).
//        int c=0;
//        for (c; c<p; c++) {
//                buf[c] = 48 + (i % tens[f+c] / (tens[f+c+1]));
//        }
//        buf[c] = '\0';
//        return;
//}

int float_to_ascii(char* buf, float f, int p){
    float tens[] = {1., 10., 100., 1000., 10000., 100000.};
    int c = (int) f;
    float d = f - c;

    int i, j = 0;
    char tempbuf[5];
    int high_part = (int) f;
    int low_part = (int) (tens[p]*(f - high_part));
    bool isleading = 1;
    int_to_ascii(tempbuf, high_part, 3);
    for (i=0; i<3; i++){ // Loop to filter out leading zeros.
        if (tempbuf[i]=='0' && isleading){
        }else{
            isleading = 0;
            buf[j] = tempbuf[i];
            j++;
        }
    }
    buf[j] = '.';
    j++;
    int_to_ascii(tempbuf, low_part, p);
    for (i=0; i<3; i++){
        buf[j] = tempbuf[i];
        j++;
    }
    buf[j] = 0;
}

int len(const char* s){
    int i = 0;
    while (s[i] != 0){
        i++;
    }
    return i;
}

int cmp(char* s1, const char* s2){
    //Return True if strings compare equal, False if not or if either of the two is null.
    int l1 = len(s1);
    int l2 = len(s2);
    int r = 0;
    int i = 0;
    if (l1*l2){    // Asserting both are non-zero.
        if (l1<l2){ i = l1; }
        else{ i = l2; }
        r = 1;
        for (int c=0; c<i; c++){
            r = r * ((int) s1[c] == (int) s2[c]);
        }
    }
    return r;
}

int copy_str(char* buffer, const char* s, int index){
    //Copy over string items into a message.
    int i = 0;
    while (s[i] !=0){
        buffer[index] = s[i];
        i++;
        index++;
    }
    return index;
}


char get_crc(char* buffer, char length){
    char i, j, crc = 0;
    for (i=0; i<length; i++){
        crc ^= buffer[i];
        for (j=0; j<8; j++){
            if (crc & 1){
                crc ^= CRC7_POLY;
            }
            crc >>= 1;
        }
    }
    return crc;
}

inline void colon_parser(uint8_t  message_index, int port){ //Appending NULL char onto last pos in segment.
    messages[message_index]->Segments[segment_counter[port]][segment_length[port]] = 0;
    segment_counter[port]++;
    segment_length[port] = 0;
}

uint8_t premessage_setup(uint8_t  message_index, int port, char ch){ // message_index is ignored and message_counter_global is written into ~counter_port[~].
    if (ch == ':'){
        message_index = message_counter_global;
        message_counter_port[port] = message_index;
        if (message_counter_global==15){
            printf("Message registers overflow\n");
        }
        message_counter_global = (message_counter_global + 1) % 16;
        messages[message_index]->Index = message_index;
        messages[message_index]->Port = port;
        messages[message_index]->Payload = payloads[message_index];
        colon_parser(message_index, port);
        messages[message_index]->CRC_buffer[0] = ch;
        messages[message_index]->CRC_index = 1;
    }
    return 0;
}

uint8_t  write_into_segment(uint8_t message_index, int port, char ch){
    uint8_t return_val = 0;
    messages[message_index]->CRC_buffer[messages[message_index]->CRC_index] = ch;
    messages[message_index]->CRC_index++;
    if (ch == ':'){
        colon_parser(message_index, port);
    }else{
        messages[message_index]->Segments[segment_counter[port]][segment_length[port]] = ch;
        segment_length[port] = (segment_length[port] + 1) % segment_max_length[segment_counter[port]];
        //printf("wis: sml: %d, sc:%d\n", segment_max_length[segment_counter[port]], segment_length[port]);

    }
    return return_val;
}

uint8_t write_into_payload_length(uint8_t message_index, int port, char ch){
    uint8_t return_val = 0;
    messages[message_index]->CRC_buffer[messages[message_index]->CRC_index] = ch;
    messages[message_index]->CRC_index++;
    if (ch == ':'){
        char* pl_length;
        int int_pl_len;
        messages[message_index]->Segments[segment_counter[port]][segment_length[port]] = 0;
        pl_length = messages[message_index]->Segments[segment_counter[port]];
        int_pl_len = ascii_to_int(pl_length);
        messages[message_index]->intLength = int_pl_len;
        colon_parser(message_index, port);
    }else{
        messages[message_index]->Segments[segment_counter[port]][segment_length[port]] = ch;
        segment_length[port] = (segment_length[port] + 1) % 32;
        return_val = 0;
    }
    return return_val;
}

uint8_t write_into_payload_data(uint8_t message_index, int port, char ch){
    messages[message_index]->CRC_buffer[messages[message_index]->CRC_index] = ch;
    messages[message_index]->CRC_index++;
    uint8_t return_val = 0;
    if (segment_length[port] > messages[message_index]->intLength-1){
        messages[message_index]->Payload[segment_length[port]] = 0;
        colon_parser(message_index, port);
    }else{
        messages[message_index]->Payload[segment_length[port]] = ch;
        segment_length[port]++;
    }
    return return_val;
}

//char message_segment_labels[11][32] = {"#Premessage#", "#Sender#", "#Receiver#",  "#RQS#",  "#Topic#",  "#Chapter#",  "#Payload_Length#",  "#Payload_Data#",  "#Hopcount#",  "#CRC#", "#ENDOFMESSAGE#"};
//
uint8_t last_segment(uint8_t message_index, int port, char ch){
    uint8_t return_val = 0;
    int r;
    int crc_val;
    int crc_rx;
    if (ch == ':'){ // Call get_crc here
        messages[message_index]->CRC_buffer[messages[message_index]->CRC_index] = 0;
        messages[message_index]->CRC_index++;
        crc_val = get_crc(messages[message_index]->CRC_buffer, messages[message_index]->CRC_index);
        crc_rx = ascii_to_int(messages[message_index]->Segments[9]);
        //printf("crc checker: l:%d, v: %d, rx: %d, for \t%s\n", messages[message_index]->CRC_index, crc_val, crc_rx, messages[message_index]->CRC_buffer);
        colon_parser(message_index, port);
        segment_counter[port] = 0;
        segment_length[port] = 0;
        message_ready = 1;
//        printf("\n");
//        printf("Message in on port %d: \n", port);
//        printf("Sender: %s\n", messages[message_index]->Segments[1]);
//        printf("Receiver: %s\n", messages[message_index]->Segments[2]);
//        printf("RQS: %s\n", messages[message_index]->Segments[3]);
//        printf("Topic: %s\n", messages[message_index]->Segments[4]);
//        printf("Chapter: %s\n", messages[message_index]->Segments[5]);
//        printf("Length: %s\n", messages[message_index]->Segments[6]);
//        printf("Payload: %s\n", messages[message_index]->Payload);
//        printf("Hops: %s\n", messages[message_index]->Segments[8]);
//        printf("CRC: %s\n", messages[message_index]->Segments[9]);
    }else{
        messages[message_index]->Segments[segment_counter[port]][segment_length[port]] = ch;
        segment_length[port] = (segment_length[port] + 1) % 32;
    }
    return return_val;
}

uint8_t  determine_addressee(uint8_t message_index, int port){ //0: Other, 1: Self, 2: General.
    uint8_t  router_value = 0;
    router_value = router_value + (1 * cmp(messages[message_index]->Segments[2], self_name));
    router_value = router_value + (2 * cmp(messages[message_index]->Segments[2], "GEN"));
    router_value = router_value + (4 * cmp(messages[message_index]->Segments[3], "RI"));
    router_value = router_value + (4 * cmp(messages[message_index]->Segments[3], "RS"));
    //printf("router value: %d\n", router_value);
    return router_value;
}

uint8_t message_topic_checker(uint8_t message_index, int port, char* buffer){
    uint8_t  t, i;
    uint8_t  topic_function_selector= -1;
    for (t=0; t<TOPIC_AMOUNT; t++){
        //printf("topic checker: t: %d, topic: %s against %s\n", t, topic_list[t], messages[message_index]->Segments[4]);
        if (cmp(messages[message_index]->Segments[4], topic_list[t])){
            topic_function_selector = t;
            i = copy_str(buffer, msg_responses[t], 0);
            buffer[i] = 0;
            break;
        }
    }
    return  topic_function_selector;
}


inline uint8_t  append_segment(char* buffer, const char* str, int index){
    int new_index;
    new_index = copy_str(buffer, str, index);
    buffer[new_index] = ':';
    new_index++;
    return new_index;
}


uint8_t  add_crc(char* buffer, int index){
    int crc_val;
    char crc_str[3];
    buffer[index] = 0;
    crc_val = get_crc(buffer, index);
    int_to_ascii(crc_str, crc_val, 3);
    index = copy_str(buffer, crc_str, index);
    //printf("add crc: %d, i: %d, cs: %s\n", crc_val, index, crc_str);
    return index;
}


uint8_t  compose_transmit_message(struct Message_out* message_tx, char* buffer){
    uint8_t return_value = 0;
    int index = 0;
    char response_buffer[255];
    char len_buf[4];
    const char* nill = {"0"};
    buffer[index] = ':'; //SENDER
    index++;
    index = append_segment(buffer, self_name, index);
    //index = append_segment(buffer, general_name, index);
    index = append_segment(buffer, message_tx->Receiver, index);
    index = append_segment(buffer, RQS_i, index);
    index = append_segment(buffer, message_tx->Topic, index);
    index = append_segment(buffer, message_tx->Chapter, index);

    int_to_ascii(len_buf, len(message_tx->Payload), 3);
    index = append_segment(buffer, len_buf, index);

    index = append_segment(buffer, message_tx->Payload, index);
    index = append_segment(buffer, nill, index); //Hopcount is nill
    index = add_crc(buffer, index);
    buffer[index] = ':';
    index++;
    buffer[index] = '\0';
    //printf("Complete transmit message: %s\n", buffer);
    return return_value;
}

uint8_t send_message(struct Message_out* message_tx){
    uint8_t return_value = 0;
    int index = 0;
    char buffer[255];
    return_value = compose_transmit_message(message_tx, buffer);
    write_from_uart0(buffer);
    write_from_uart1(buffer);
    return return_value;
}


//char message_segment_labels[11][32] = {"#Premessage#", "#Sender#", "#Receiver#",  "#RQS#",  "#Topic#",  "#Chapter#",  "#Payload_Length#",  "#Payload_Data#",  "#Hopcount#",  "#CRC#", "#ENDOFMESSAGE#"};

uint8_t  compose_response_message(uint8_t message_index, int port, char* buffer){
    uint8_t return_value = 0;
    int index = 0;
    char response_buffer[255];
    uint8_t  topic_function_selector;
    char len_buf[4];
    char hops_buf[3];
    int hops;
    buffer[index] = ':'; //SENDER
    index++;
    index = append_segment(buffer, self_name, index);
    index = append_segment(buffer, messages[message_index]->Segments[1], index);
    topic_function_selector = message_topic_checker(message_index, port, response_buffer);
    if (topic_function_selector == -1){
        index = append_segment(buffer, RQS_n, index);
    }else{
        index = append_segment(buffer, RQS_r, index);
    }
    messages[message_index]->Topic_number = topic_function_selector;
    index = append_segment(buffer, messages[message_index]->Segments[4], index);
    index = append_segment(buffer, messages[message_index]->Segments[5], index);
    int_to_ascii(len_buf, len(response_buffer), 3);
    index = append_segment(buffer, len_buf, index);
    index = append_segment(buffer, response_buffer, index);
    hops = ascii_to_int(messages[message_index]->Segments[8]);
    hops++;
    int_to_ascii(hops_buf, hops, 3);
    index = append_segment(buffer, hops_buf, index);
    index = add_crc(buffer, index);
    buffer[index] = ':';
    index++;
    buffer[index] = '\0';
    //printf("Complete response message: %s\n", buffer);
    return return_value;
}

uint8_t  compose_forward_message(uint8_t message_index, int port, char* buffer){
    uint8_t return_value = 0;
    int index = 0;
    char response_buffer[255];
    uint8_t  i = 0;
    char hops_buf[3];
    uint8_t  hops;
    buffer[index] = ':';
    index++;
    for (i=1; i<7; i++){
        index = append_segment(buffer, messages[message_index]->Segments[i], index);
    }
    index = append_segment(buffer, messages[message_index]->Payload, index);
    hops_buf[3];
    hops = ascii_to_int(messages[message_index]->Segments[8]);
    hops++;
    int_to_ascii(hops_buf, hops, 3);
    index = append_segment(buffer, hops_buf, index);
    index = add_crc(buffer, index);
    buffer[index] = ':';
    index++;
    buffer[index] = '\0';
    //printf("Complete forward message: %s\n", buffer);
    return return_value;
}

uint8_t write_from_uart0(char* buffer){
    uart_puts(uart0, buffer);
    printf("UART_0 gmc: %d, lmc: %d, m: %s\n", message_counter_global, message_counter_port[0], buffer);
    return 0;
}

uint8_t write_from_uart1(char* buffer){
    uart_puts(uart1, buffer);
    printf("UART_1 gmc: %d, lmc: %d, m: %s\n", message_counter_global, message_counter_port[1], buffer);
    return 0;
}

uint8_t self_call(uint8_t message_index, int port){
    uint8_t return_value = 0;
    char buffer[255];
    return_value = compose_response_message(message_index, port, buffer);
    return_value = (*uart_write_functions[port])(buffer);
    messages_out[message_index]->For_self = 1;
    messages_out[message_index]->For_gen = 0;
    //printf("Complete response  message SC: %s\n", buffer);
    return return_value;
}

uint8_t other_call(uint8_t message_index, int port){
    uint8_t return_value = 0;
    char buffer[255];
    //int port = messages[message_index]->Port;
    port = 1 - port;
    //printf("other call\n");
    return_value = compose_forward_message(message_index, port, buffer);
    return_value = (*uart_write_functions[port])(buffer);
    //printf("Complete forward message OC: %s\n", buffer);
    messages_out[message_index]->For_self = 0;
    messages_out[message_index]->For_gen = 0;
    return return_value;
}

uint8_t no_response_call(uint8_t message_index, int port){
    uint8_t return_value = 0;
   // char buffer[255];
   // //int port = messages[message_index]->Port;
   // return_value = compose_response_message(message_index, port, buffer);
   // return_value = (*uart_write_functions[port])(buffer);
   // return_value = compose_forward_message(message_index, port, buffer);
   // //port = 1 - port;
   // return_value = (*uart_write_functions[1-port])(buffer);
    messages_out[message_index]->For_self = 0;
    messages_out[message_index]->For_gen = 0;
    return return_value;
}



uint8_t general_call(uint8_t message_index, int port){
    uint8_t return_value = 0;
    char buffer[255];
    //int port = messages[message_index]->Port;
    return_value = compose_response_message(message_index, port, buffer);
    return_value = (*uart_write_functions[port])(buffer);
    return_value = compose_forward_message(message_index, port, buffer);
    //port = 1 - port;
    return_value = (*uart_write_functions[1-port])(buffer);
    messages_out[message_index]->For_self = 1;
    messages_out[message_index]->For_gen = 1;
    return return_value;
}

uint8_t format_return_message(uint8_t message_index){
    uint8_t r = 0;
    uint8_t i = 0;
    i = copy_str(messages_out[message_index]->Sender, messages[message_index]->Segments[1], 0);
    messages_out[message_index]->Sender[i] = 0;
    i = copy_str(messages_out[message_index]->Receiver, messages[message_index]->Segments[2], 0);
    messages_out[message_index]->Receiver[i] = 0;
    i = copy_str(messages_out[message_index]->Topic, messages[message_index]->Segments[4], 0);
    messages_out[message_index]->Topic[i] = 0;
    i = copy_str(messages_out[message_index]->Chapter, messages[message_index]->Segments[5], 0);
    messages_out[message_index]->Chapter[i] = 0;
    i = copy_str(messages_out[message_index]->Payload, messages[message_index]->Payload, 0);
    messages_out[message_index]->Payload[i] = 0;
    messages_out[message_index]->Topic_number = messages[message_index]->Topic_number;
    messages_out[message_index]->intLength = messages[message_index]->intLength;

    return r;
}

struct Message_out* message_processor(){
    //printf("mp\n");
    struct Message_out* return_message;
    return_message = NULL;
    if (message_ready){
        message_ready = 0;
        uint8_t m;
        uint8_t r = 1;

        for (m=0; m<message_counter_global; m++){

            int res;
            int port = messages[m]->Port;
            int router_value = determine_addressee(m, port);
            res = (*call_router_functions[router_value])(m, port);
            r = format_return_message(m);
            return_message = messages_out[m];
        }
        message_counter_global = 0;
    }
    return return_message;
}

uint8_t  uart0_character_processor(char ch){
    int port = 0;
    //printf("U0CP: %d, %c, %d, %d, %s\n", message_counter_global, ch, segment_counter[port], segment_length[port], message_segment_labels[segment_counter[port]]);
    int res;
    uint8_t message_index = message_counter_port[port];
    res = (*parse_function_array[segment_counter[port]])(message_index, port, ch);
    return 0;
}

uint8_t  uart1_character_processor(char ch){
    int port = 1;
    //printf("U1CP: %d, %c, %d, %d, %s\n", message_counter_global, ch, segment_counter[port], segment_length[port], message_segment_labels[segment_counter[port]]);
    int res;
    uint8_t  message_index = message_counter_port[port];
    res = (*parse_function_array[segment_counter[port]])(message_index, port, ch);
    return 0;
}


void uart0_irq_routine(void){
    int r;
    //gpio_put(LED_2, 1);
    while (uart_is_readable(uart0)){
        uint8_t ch = uart_getc(uart0);
        r = uart0_character_processor(ch);
    }
}

void uart1_irq_routine(void){
    int r;
    //gpio_put(LED_3, 1);
    while (uart_is_readable(uart1)){
        uint8_t ch = uart_getc(uart1);
        r = uart1_character_processor(ch);
    }
}



void message_setup(){
    int i, res;

    //printf("Message stack being set up\n");
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
    segment_counter[0] = 0;
    segment_length[0] = 0;
    segment_counter[1] = 0;
    segment_length[1] = 0;
    parse_function_array[0]  = &premessage_setup;
    parse_function_array[1]  = &write_into_segment; // Sender
    parse_function_array[2]  = &write_into_segment; // Receiver
    parse_function_array[3]  = &write_into_segment; // RQS
    parse_function_array[4]  = &write_into_segment; // Topic
    parse_function_array[5]  = &write_into_segment; // Chapter
    parse_function_array[6]  = &write_into_payload_length; // Payload Length
    parse_function_array[7]  = &write_into_payload_data; // Payload Data
    parse_function_array[8]  = &write_into_segment; // Hopcount
    parse_function_array[9]  = &last_segment; // CRC etc.

    call_router_functions[0] = &other_call;
    call_router_functions[1] = &self_call;
    call_router_functions[2] = &general_call;
    call_router_functions[3] = &general_call;
    call_router_functions[4] = &other_call;
    call_router_functions[5] = &no_response_call;
    call_router_functions[6] = &other_call;
    call_router_functions[7] = &other_call;
    call_router_functions[8] = &other_call;

    uart_write_functions[0] = &write_from_uart0;
    uart_write_functions[1] = &write_from_uart1;
}

#endif
