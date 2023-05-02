#ifndef U2U_H
#define U2U_H

#define general_name "GEN"

volatile bool message_ready;


struct Message_out{
    char Sender[32];
    char Receiver[32];
    char RQ[4];
    char Topic[32];
    char Chapter[4];
    char Payload[255];
    bool For_self;
    bool For_gen;
    int Topic_number;
    int intLength;
};


//struct Message{
//    int   Index;
//    char  Segments[10][32];
//    char* Payload;
//    int   Port;
//    int   intLength;
//    int   intCRC;
//    char  CRC_buffer[255];
//    int   CRC_index;
//};

//{struct Message m0, struct Message m1, struct Message m2, struct Message m3, struct Message m4, struct Message m5, struct Message m6, struct Message m7, struct Message m8, struct Message m9, struct Message m10, struct Message m11, struct Message m12, struct Message m13, struct Message m14, struct Message m15};
//struct Message  m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14, m15;
//struct Message* messages[16] = {&m0, &m1, &m2, &m3, &m4, &m5, &m6, &m7, &m8, &m9, &m10, &m11, &m12, &m13, &m14, &m15};
//char payloads[16][255];
//
//char message_segment_labels[11][32] = {"#Premessage#", "#Sender#", "#Receiver#",  "#RQS#",  "#Topic#",  "#Chapter#",  "#Payload_Length#",  "#Payload_Data#",  "#Hopcount#",  "#CRC#", "#ENDOFMESSAGE#"};
//
//const char RQS_r[] = "RS";
//const char RQS_n[] = "NA";
//
//uint8_t message_counter_global = 0;
//uint8_t  segment_counter[2] = {0, 0};
//uint8_t  segment_length[2]  = {0, 0};
//uint8_t  message_counter_port[2] = {0, 0};
//uint8_t  segment_max_length[] = {1, 32, 32, 4, 32, 4, 4, 255, 4, 4};
//
//
//uint8_t  (*parse_function_array[12])(uint8_t, int, char);
//uint8_t  (*call_router_functions[3])(uint8_t, int);
//uint8_t  (*uart_write_functions[2])(char*);

int ascii_to_int(char* str);
void int_to_ascii(char* buf, int i, int p) ;
int len(const char* s);
int cmp(char* s1, const char* s2);
int copy_str(char* buffer, const char* s, int index);
char get_crc(char* buffer, char length);
inline void colon_parser(uint8_t  message_index, int port); //Appending NULL char onto last pos in segment;
uint8_t premessage_setup(uint8_t message_index, int port, char ch); // message_index is ignored and message_counter_global is written into ~counter_port[~];
uint8_t write_into_segment(uint8_t message_index, int port, char ch);
uint8_t write_into_payload_length(uint8_t message_index, int port, char ch);
uint8_t write_into_payload_data(uint8_t message_index, int port, char ch);
uint8_t last_segment(uint8_t message_index, int port, char ch);
uint8_t determine_addressee(uint8_t message_index, int port); //0: Other, 1: Self, 2: General;
uint8_t message_topic_checker(uint8_t message_index, int port, char* buffer);
inline uint8_t append_segment(char* buffer, const char* str, int index);
uint8_t add_crc(char* buffer, int index);
uint8_t compose_transmit_message(struct Message_out* message_tx, char* buffer);
uint8_t send_message(struct Message_out* message_tx);
uint8_t compose_response_message(uint8_t message_index, int port, char* buffer);
uint8_t compose_forward_message(uint8_t message_index, int port, char* buffer);
uint8_t write_from_uart0(char* buffer);
uint8_t write_from_uart1(char* buffer);
uint8_t self_call(uint8_t message_index, int port);
uint8_t other_call(uint8_t message_index, int port);
uint8_t no_response_call(uint8_t message_index, int port);
uint8_t general_call(uint8_t message_index, int port);
uint8_t format_return_message(uint8_t message_index);
struct Message_out*  message_processor();
uint8_t uart0_character_processor(char ch);
uint8_t uart1_character_processor(char ch);
void uart0_irq_routine(void);
void uart1_irq_routine(void);
void message_setup();
//void testfunc(char* s, int port);

#endif
