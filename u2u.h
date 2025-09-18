
//u2u.h
#ifndef U2U_H
#define U2U_H

//#include "u2u_HAL_pico.h"
#include "u2uclientdef.h"

#if U2U_PLATFORM_CHANNELS == 1   
#include "u2u_HAL_lx.h"
#include <stdlib.h>
#include <stdint.h> 
#include <stdbool.h> 
#include "c_logger.h"
#endif 

#if U2U_PLATFORM_CHANNELS == 2 
#include "pico/stdlib.h"
#include "u2u_HAL_pico.h"
#endif 



#define general_name "GEN"

#define MAX_MESSAGE_KEEP 8
#define MAX_MESSAGE_PAYLOAD_SIZE 255
#define MAX_MESSAGE_SIZE 512


//const char* topic_list[] = { "HAIL", "HELP", "SET LCD", "SET OLED", "GET SENSOR", "SET ENCODER", "GET ENCODER", "SET LED", "SET TIME", "GET TIME", "SET DATE", "GET DATE", "RESERVED 0", "RESERVED 1", "RESERVED 2", "RESERVED 3"};



struct    Message{
    int             ID;
    int             intChapter;
    const char*     Sender;
    const char*     Receiver;
    const char*     RQ;
    const char*     Topic;
    const char*     Chapter;
    const char*     CRC;
    const char*     Hops;
    int             segment_len[12];
    int             intLength;
    int             intCh_rx;

    int             Index;
    int             intRQ;
    char            Segments[12][32];
    char*           Payload;
    uint8_t         Port;
    int             intCRC_rx;
    int             intCRC_cal;
    int             intHops;
    char            CRC_buffer[255];
    int             CRC_index;
    bool            CRC_check;
    int             Topic_number;
    int             Router_val;
    bool            For_self;
    bool            For_gen;
    bool            Cleared;
    bool            Manual_response_waiting;

    //bool  RQ_flags[4]; //[0]: R, [1]: Q, [2]: I, [3]: N.
};


struct Message_Queue {
    int             buffer[MAX_MESSAGE_KEEP];
    int             front;
    int             rear;
    int             count;
} ;


struct U2U_error {
    int             errors[2];
    int             crc_errors[2];
    int             r_failures[2];
    int             tfs[2];
};

//extern const char RQS_r[];
//extern const char RQS_q[];
//extern const char RQS_i[];
//extern const char RQS_n[];

/*Inbound messages are stored in 'messages'*/
extern struct Message* messages[MAX_MESSAGE_KEEP];
//struct Message* messages_out[MAX_MESSAGE_KEEP];


/*General purpose functions*/
int     ascii_to_int(char* str);
int     ascii_to_int_i(char* str, int len); // Not reliant on NULL termination.
void    int_to_ascii(char* buf, int i, int p) ;
int     float_to_ascii(char* buf, float f, int p);
int     len(const char* s);
int     cmp(char* s1, const char* s2);
int     cmp_i(char* s1, const char* s2, int l1, int l2);
int     copy_str(char* buffer, const char* s, int index);
int     copy_str_i(char* buffer, const char* s, int index, int string_len);
char    get_crc(char* buffer, char length);
int     in_queue(struct Message_Queue *queue, int data);
int     out_queue(struct Message_Queue *queue, int *data);


/*Function declarations for matrix parsing:
 * f_[mo][m][seg_i], where:
 *      mo:     <0> if previous ch was not colon
 *      mo:     <1> if previous ch was  colon
 *      m:      <0> if  ch is not colon
 *      m:      <1> if  ch is  colon
 *      seg_i:  <n> Segment index n
 *      x:      Don't care
 */
uint8_t f_000_message_clear(uint8_t message_index, uint8_t port, char ch); // Message clear
uint8_t f_110_two_colons(uint8_t message_index, uint8_t port, char ch); // Two successive colons.
uint8_t f_111_three_colons(uint8_t message_index, uint8_t port, char ch); // Three successives colons for some reason.
uint8_t f_11x(uint8_t message_index, uint8_t port, char ch); // Two successive colon mid message.
uint8_t f_100(uint8_t message_index, uint8_t port, char ch); // Should not be invoked.
uint8_t f_00x(uint8_t message_index, uint8_t port, char ch); // Second+ character written into segments.
uint8_t f_10x(uint8_t message_index, uint8_t port, char ch); // First character written into segments.
uint8_t f_01x(uint8_t message_index, uint8_t port, char ch); // Colon marker for second+ segments.
uint8_t f_016(uint8_t message_index, uint8_t port, char ch); // Segment for payload LENGTH is in.
uint8_t f_xx7(uint8_t message_index, uint8_t port, char ch); // Writing into PAYLOAD.
uint8_t f_x08_penultimate_segment(uint8_t message_index, uint8_t port, char ch); // Writing into penultimate segment.
uint8_t f_018_marking_last_segment(uint8_t message_index, uint8_t port, char ch); // Marking start last segment.
uint8_t f_x09_writing_into_last_segment(uint8_t message_index, uint8_t port, char ch); // Writing into last segment.
uint8_t f_019_message_complete(uint8_t message_index, uint8_t port, char ch); // Last colon. Message is complete.


uint8_t message_clear(uint8_t message_index, uint8_t port);
uint8_t write_into_segment(uint8_t message_index, uint8_t port, char ch);
void log_outbound_message(char* buffer, int mes_len, int pdr);
void log_inbound_message(uint8_t message_index, uint8_t port);
int write_crc_buffer(uint8_t message_index, uint8_t port, char ch);
uint8_t write_into_payload(uint8_t message_index, uint8_t port, char ch);
uint8_t message_start(int _, uint8_t port, char ch); // Variable message_index is not valid here.
struct Message* get_message();

/* Message processing. */
uint8_t determine_addressee(uint8_t message_index, uint8_t port); //0: Other, 1: Self, 2: General;
int     topic_to_int(char* topic);
int     message_topic_checker(uint8_t message_index, uint8_t port, char* buffer);
uint8_t append_segment(char* buffer, const char* str, int index);
uint8_t add_crc(char* buffer, int index);
uint8_t compose_transmit_message(struct Message* message_tx, char* buffer, int* message_length);
uint8_t u2u_send_message_uart0(struct Message* message_tx);
uint8_t u2u_send_message_uart1(struct Message* message_tx);
uint8_t u2u_send_message(struct Message* message_tx);
uint8_t compose_response_message(uint8_t message_index, uint8_t port, char* buffer, int* message_length);
uint8_t compose_forward_message(uint8_t message_index, uint8_t port, char* buffer, int* message_length);
uint8_t process_message(uint8_t message_index, uint8_t port); // Writing into last segment.
//uint8_t write_from_uart0(char* buffer);
//uint8_t write_from_uart1(char* buffer);

/* Message responding. */
uint8_t self_call(uint8_t message_index, uint8_t port);
uint8_t other_call(uint8_t message_index, uint8_t port);
uint8_t no_response_call(uint8_t message_index, uint8_t port);
uint8_t general_call(uint8_t message_index, uint8_t port);

/* Function message_processor() calls to format response */
uint8_t format_return_message(uint8_t message_index);

/* Completion of writing into last segment calls to process message for responses (if any).*/
uint8_t message_processor(uint8_t message_index);
struct  Message* get_message();
uint8_t u2u_topic_exchange(char* custom_payload, uint8_t topic_number);
uint8_t  uart_character_processor(char ch);

/* Inbound message handler entry point. */
uint8_t  uart0_character_processor(char ch);
uint8_t  uart1_character_processor(char ch);
uint8_t  character_processor(uint8_t port, char ch);
void     parser_setup();
uint8_t  u2u_self_test(uint8_t port);
//void uart0_irq_routine(void);
//void uart1_irq_routine(void);

/* Alpha and Omega. */
uint8_t     u2u_message_setup();
uint8_t     u2u_close(void);


#endif

