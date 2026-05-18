//u2u.h
#ifndef U2U_H
#define U2U_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "u2u_client_profile.h"

#if U2U_PLATFORM_CHANNELS == 1   
//#include "u2u_HAL_lx.h"
#include <stdlib.h>
#include <stdint.h> 
#include <stdbool.h> 
#include "c_logger.h"
#include "u2u_HAL_lx.h"
//#define PORT_COUNT 2
#endif 

#if U2U_PLATFORM_CHANNELS == 2 
#include "pico/stdlib.h"
#include "u2u_HAL_pico.h"
//#define PORT_COUNT 2
#endif 

#if U2U_PLATFORM_CHANNELS == 3
#include "u2u_HAL_esp.h"
#endif 

#define PORT_COUNT 3




#define GENERAL_NAME "GEN"


/* U2U version 5
 * Message is no longer segmentated by ':' rather, length of each segment is explicilty 
 * indicated by the preamble. This preamble consists of as many parts as there are segments
 * and starts with a single capital character  - a unique ID for the segment, followed by digits 
 * indicating the segments length. This allows for a flexible ordering of segments.
 * Example:
 * ::S10R3F2T8C1P18H1Y2MDEEPSPACE9GENRQhail____0This is a payload.085:
 * Where:
 *     Segment      Symbol  Length  Content
 *  0  START        ::      2       -
 *  1  SENDER       S       10      DEEPSPACE9
 *  2  RECEIVER     R       3       GEN
 *  3  R-FLAG       F       2       RQ 
 *  4  TOPIC        T       8       HAIL____ 
 *  5  PAYLOAD      P       18      This is a payload.
 *  6  HOPS         H       1       0
 *  7  CRC          Y       2       85
 *  8  END          :       1       -
 */


#define MAX_MESSAGE_KEEP 8
#define MAX_MESSAGE_SIZE 1024
#define MAX_PAYLOAD_SIZE (MAX_MESSAGE_SIZE - 127)
#define MAX_TOPIC_RESPONSES      17
#define MAX_SEGMENT_COUNT            16

#define MAX_SENDER_SIZE          16
#define MAX_RECEIVER_SIZE        16
#define MAX_RFLAG_SIZE              2
#define MAX_TOPIC_SIZE           8
#define MAX_CHAPTER              9999999
#define MAX_CHAPTER_SIZE         8
#define MAX_HOPS                 9999
#define MAX_HOPS_SIZE            4
#define MAX_CRC_SIZE             3
#define MAX_ID_SIZE              8
#define MAX_K_SIZE               8

#define MAX_PR_MESSAGE_SIZE      6    // 0 to 999999    
#define MAX_PR_SENDER_SIZE       2    // 0 to 99
#define MAX_PR_RECEIVER_SIZE     2    // 0 to 99
#define MAX_PR_RFLAG_SIZE           1    // 0 to 9
#define MAX_PR_TOPIC_SIZE        1    // 0 to 9
#define MAX_PR_CHAPTER_SIZE      1    // 0 to 9
#define MAX_PR_PAYLOAD_SIZE      4    // 0 to 9999
#define MAX_PR_HOPS_SIZE         1    // 0 to 9
#define MAX_PR_CRC_SIZE          1    // 0 to 9
#define MAX_PR_ID_SIZE           1    // 0 to 9
#define MAX_PR_K_SIZE            1    // 0 to 9




enum topic_numbers{
    HAIL_TPNR       = 0, 
    HELP_TPNR       = 1, 
    SET_LCD_TPNR    = 2, 
    SET_OLED_TPNR   = 3, 
    GET_SNSR_TPNR   = 4, 
    SET_ENCR_TPNR   = 5, 
    GET_ENCR_TPNR   = 6, 
    SET_LED_TPNR    = 7, 
    SET_TIME_TPNR   = 8, 
    GET_TIME_TPNR   = 9, 
    SET_DATE_TPNR   = 10, 
    GET_DATE_TPNR   = 11, 
    RSERVD_0_TPNR   = 12, 
    RSERVD_1_TPNR   = 13, 
    RSERVD_2_TPNR   = 14, 
    RSERVD_3_TPNR   = 15, 
    RSERVD_4_TPNR   = 16
};

//
//#define MAX_MESSAGE_KEEP 8
//#define MAX_MESSAGE_PAYLOAD_SIZE 255
//#define MAX_MESSAGE_SIZE 512


//const char* topic_list[] = { "HAIL", "HELP", "SET LCD", "SET OLED", "GET SENSOR", "SET ENCODER", "GET ENCODER", "SET LED", "SET TIME", "GET TIME", "SET DATE", "GET DATE", "RESERVED 0", "RESERVED 1", "RESERVED 2", "RESERVED 3"};



/*General purpose functions*/


struct Message_Segments{
    char   sender[MAX_SENDER_SIZE+1];
    char   receiver[MAX_RECEIVER_SIZE+1];
    char   rflag[MAX_RFLAG_SIZE+1];
    char   topic[MAX_TOPIC_SIZE+1];
    char   chapter[MAX_CHAPTER_SIZE+1];
    char   payload[MAX_PAYLOAD_SIZE+1];
    char   crc[MAX_CRC_SIZE+1];
    char   hops[MAX_HOPS_SIZE+1];
};


struct Message{
    //char    payload[MAX_PAYLOAD_SIZE];
    char    raw[MAX_MESSAGE_SIZE];
    size_t  length;
    size_t  preamble_end;
    size_t  segment_length[MAX_SEGMENT_COUNT];
    int     topic_nr;  
    uint16_t error_code;

    char*   segments[MAX_SEGMENT_COUNT];
    char    preamble_flags[MAX_SEGMENT_COUNT];   // Contains preamble flags (S for sender, etc)
    
    char*   sender;
    char*   receiver;
    char*   rflag;
    char*   topic;
    char*   chapter;
    char*   payload;
    char*   crc;
    char*   hops;
    
    char*   segment_flags;
    uint8_t chapter_int;
    uint8_t crc_calculated;
    uint8_t crc_received;
    uint8_t hops_int;
    uint8_t index;
    uint8_t port;
    uint8_t r_value;
    uint8_t routing_control;
    uint8_t s_value;
    uint8_t segment_count;
    //uint8_t segments_received;
};

struct Message_Queue;


int ascii_to_int(char* str);
int ascii_to_int_i(char* str, int length); // Not reliant on NULL termination;
size_t int_to_ascii(char* buf, int integer, int p);  // If p is 0 then p is determined;
uint32_t ascii_to_hex_i(const char *str, size_t length) ;
size_t hex_to_ascii(char *buf, uint32_t value, size_t width) ;
int float_to_ascii(char* buf, float f, int p);
int len(const char* s);
int cmp_i(const char* s1, const char* s2, int l1, int l2);
int cmp(char* s1, const char* s2);
int copy_str_i(char* buffer, const char* s, int index, int string_len);
int copy_str(char* buffer, const char* s, int index);
uint8_t get_crc(char* buffer, char length);
bool is_digit(char c);
bool is_capital(char c);
bool is_lower(char c);
bool is_alpha(char c);

/* Hash related functions */
void topic_hash_table_init();
uint8_t get_topic_hash(char* buffer, int length);
int topic_to_int_hash(char* buffer, int length);

/* Cyclic queue functions */
void message_queue_init(struct Message_Queue* queue, uint8_t port);
void message_queue_clear(struct Message_Queue* queue);
uint8_t get_queue_max(struct Message_Queue* queue);
uint8_t get_queue_size(struct Message_Queue* queue);
uint8_t get_next_free(struct Message_Queue* queue);
bool queue_full(struct Message_Queue* queue);
bool queue_empty(struct Message_Queue* queue);
int in_queue(struct Message_Queue* queue, uint8_t index);
int out_queue(struct Message_Queue* queue, uint8_t* index);

/* U2U specific functions */
void topic_init();
//int u2u_topic_exchange(int topic_nr, const char* buffer, size_t length);
int u2u_topic_exchange(int topic_nr, char* new_response);
void parser_init();
struct Message* assign_message(uint8_t port);
void message_clear(struct Message* message);
void parser_clear(uint8_t port);
uint8_t character_handler(uint8_t port, char ch);
int8_t get_segment_index(struct Message* message, char flag);
size_t get_segment_length(struct Message* message, char flag);
uint8_t message_parser(struct Message* message, uint8_t port);
size_t add_segment(char* buffer, const char* segment, size_t offset, size_t length);
size_t response_composer(struct Message* message, uint8_t port, char* buffer);
size_t forward_composer(struct Message* message, char* buffer);
uint8_t get_max_ports(uint8_t* ports, uint8_t port);
void write_forward_message(char* buffer, size_t length, uint8_t port);
void write_response_message(char* buffer, size_t length, uint8_t port);

uint8_t general_call(struct Message* message, uint8_t port);
uint8_t self_call(struct Message* message, uint8_t port);
uint8_t other_call(struct Message* message, uint8_t port);
uint8_t drop_call(struct Message* message, uint8_t port);
uint8_t drop_RN_call(struct Message* message, uint8_t port);
uint8_t illegal_call(struct Message* message, uint8_t port);
uint8_t illegal_GEN_call(struct Message* message, uint8_t port);
uint8_t illegal_ROUTING_call(struct Message* message, uint8_t port);
uint8_t message_router(struct Message* message, uint8_t port);
uint8_t character_processor(uint8_t port, char ch);
uint8_t uart0_character_processor(char ch);
uint8_t uart1_character_processor(char ch);
void messages_init();
void u2u_message_setup();
void print_usage_message(int argc, char** argv);
struct Message* get_message(struct Message_Segments* segments);
void set_upc(uint8_t upc);
void u2u_close();
int pipe_message_composer(uint8_t port, char* segments);
int format_message(struct Message* message);
int u2u_send_message(struct Message* message, uint8_t port);
int u2u_send_message_uart0(char* buffer, size_t length);
int u2u_send_message_uart1(char* buffer, size_t length);
int u2u_send_message_uart2(char* buffer, size_t length);
uint8_t u2u_write_character(uint8_t port, char ch);
size_t get_u2u_error_log(uint8_t port, char* buffer);
uint32_t get_u2u_errors(uint8_t port);
void clear_u2u_errors(uint8_t port);


#endif //U2U_H

