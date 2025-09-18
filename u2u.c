//u2u.c
#include "u2u.h"
//#include "u2uclientdef.h"

#define COLONS_TOTAL        0
#define COLONS_SUCCESSIVE   1
#define RESPOND_ON_BAD_CRC  1

#define MAX_TABLE_SIZE 1024
#define MAX_PORTS 2


#define PR_MO 2
#define PR_M  2
#define PR_SG 16


int hash_table[MAX_TABLE_SIZE];
struct Message_Queue Queue[2];
static char log_buffer[1024];   
//const char* test_messages = "/home/pi/c_taal/u2u/tests/test_messages/test_messages.txt";
const char* test_messages = "/home/pi/c_taal/u2u/core/u2u_v3/test_messages.txt";
static const char default_response[] = "U2U testing module.";

/* String constants against which topics are compared */
const char* topic_list[]    = { "HAIL____", "HELP____", "SET_LCD_", "SET_OLED", "GET_SNSR", "SET_ENCR", "GET_ENCR", "SET_LED_", "SET_TIME", "GET_TIME", "SET_DATE", "GET_DATE", "RSERVD_0", "RSERVD_1", "RSERVD_2", "RSERVD_3", "RSERVD_4"};
/* Topic number corresponds to indices of char pointers. These can be overwritten in u2u_topic_exchange().*/
const char* msg_responses[] = {msg_hail_response, msg_help_response, msg_set_lcd_response, msg_set_oled_response, msg_get_sensor_response, msg_set_encoder_response, msg_get_encoder_response, msg_set_led_response, msg_set_time_response, msg_get_time_response, msg_set_date_response, msg_get_date_response, msg_RESERVED_0, msg_RESERVED_1, msg_RESERVED_2, msg_RESERVED_3, msg_RESERVED_4} ;
const char message_segment_labels[12][32] = {"#Premessage#", "#Sender#", "#Receiver#",  "#RQS#",  "#Topic#",  "#Chapter#",  "#Payload_Length#",  "#Payload_Data#",  "#Hopcount#",  "#CRC#", "#ENDOFMESSAGE#"};
//const char* routing_labels = {"other_call", "self_call", "general_call", "general_call", "other_call", "no_response_call", "other_call", "other_call", "other_call"};

//struct  Message mo;
struct  U2U_error errors;
//char    mo_sender[16];
//char    mo_receiver[16];
//char    mo_topic[16];
//char    mo_rqs[4];
//char    mo_payload[255];
//char    mo_chapter[4];

//const unsigned char CRC7_POLY              = 0x91;
const uint8_t    CRC7_POLY              = 0x91;

//static const uint8_t sg_WAIT_INDEX         = 0;
//static const uint8_t sg_PREMESSAGE_INDEX   = 0;
static const uint8_t sg_SENDER_INDEX       = 1;
static const uint8_t sg_RECEIVER_INDEX     = 2;
static const uint8_t sg_RQS_INDEX          = 3;
static const uint8_t sg_TOPIC_INDEX        = 4;
static const uint8_t sg_CHAPTER_INDEX      = 5;
//static const uint8_t sg_LENGTH_INDEX       = 6;
static const uint8_t sg_PAYLOAD_INDEX      = 7;
static const uint8_t sg_HOPCOUNT_INDEX     = 8;
static const uint8_t sg_CRC_INDEX          = 9;


static const char RQS_r[]                  = "RS";
//static const char RQS_q[]                  = "RQ";
static const char RQS_i[]                  = "RI";
static const char RQS_n[]                  = "RN";
static const char RQS_a[]                  = "RA";

int rx_id                                  = 0;
int tx_id                                  = 0;

const uint8_t SEGMENT_MAX_LENGTH[]         = {1, 32, 32, 8, 16, 8, 4, MAX_MESSAGE_PAYLOAD_SIZE, 4, 4};

//uint8_t  message_counter_global            = 0;
int      rx_segment_lengths[2][10]         = { 0 };

int message_index_queue[MAX_MESSAGE_KEEP]  = { -1 };

int ch_mapper[255];

struct Parser {
    int         i;
    uint8_t     message_index;
    char        rx_ch;                  // Actual character received
    char        prx_ch;                 // Actual character received
    int         rx_i;                   // Received CHARACTER index (Total received)
    int         segment_index;          // SEGMENT index
    int         character_index;        // CHARACTER per SEGMENT index (Received as per segment)
    uint8_t     (*pr_mapper[PR_MO][PR_M][PR_SG])(uint8_t message_index, uint8_t port, char ch);
};


int         message_counter_global;
struct      Parser parser[MAX_PORTS];
struct      Message M;
struct      Message* messages[MAX_MESSAGE_KEEP];
struct      Message mi0, mi1, mi2, mi3, mi4, mi5, mi6, mi7;

uint8_t    (*call_router_functions[9])(uint8_t, uint8_t);
uint8_t    (*uart_write_functions[2])(char*, int, int);

char       payloads[MAX_MESSAGE_KEEP][MAX_MESSAGE_PAYLOAD_SIZE];
char       payloads_out[MAX_MESSAGE_KEEP][MAX_MESSAGE_PAYLOAD_SIZE];

// Utilities
int ascii_to_int(char* str){
    int len = 0;
    int sum = 0;
    int decs[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    while (str[len] != 0){ len++; }
    for (int c=0; c<len; c++){
        if (str[c]>=48 && str[c]<58){
            sum = sum + (decs[len-c-1] * (str[c]-48));
        }
    }
    return sum;
}

int ascii_to_int_i(char* str, int len){ // Not reliant on NULL termination.
    //int len = 0;
    int sum = 0;
    int decs[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    //while (str[len] != 0){ len++; }
    for (int c=0; c<len; c++){
        if (str[c]>=48 && str[c]<58){
            sum = sum + (decs[len-c-1] * (str[c]-48));
        }
    }
    return sum;
}

void int_to_ascii(char* buf, int integer, int p) { // If p is 0 then p is determined.
        int tens[] = {10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};
        int f = 0; //p denotes 'precision' or number of digits.
        int c = 0;
        int j = 0;
        const int M = 7;
        if (integer==0){ // Going to be zero often, so cutting this short.
            if (p==0 || p==1){
                buf[0] = 48;
                buf[1] = '\0';
            }else{
                for (c=0; c<p; c++){
                    buf[c] = 48;
                }
                buf[c] = '\0';
            }
        }else{
            if (p==0){
                for (j=M; j>0; --j){
                    if (integer >= tens[j]){ p++; }else{ break; }
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
    return 0;
}

int len(const char* s){
    int i = 0;
    while (s[i] != 0){ i++; }
    return i;
}

int cmp_i(char* s1, const char* s2, int l1, int l2){
    /*Return True if strings compare equal, False if not or if empty. 
     * Not reliant on Null termination.*/
    //int l1 = len(s1);
    //int l2 = len(s2);
    int r = 0;
    int i = 0;
    if (l1==l2){
        if (l1==0){ return r;}
        else{
            r = 1;
            for (int c=0; c<l1; c++){
                r = r * ((int) s1[c] == (int) s2[c]);
            }
        }
    }
    return r;
}

int cmp(char* s1, const char* s2){
    /*Return True if strings compare equal, False if not or if empty. 
     * Assuming null termination.*/
    int l1 = len(s1);
    int l2 = len(s2);
    int r = 0;
    int i = 0;
    if (l1==l2){
        if (l1==0){ return r;}
        else{
            r = 1;
            for (int c=0; c<l1; c++){
                r = r * ((int) s1[c] == (int) s2[c]);
            }
        }
    }
    return r;
}

int copy_str_i(char* buffer, const char* s, int index, int string_len){
    /*Copy over string items into a buffer.
     * Provide index = 0 for simple 1-1 copy, for concatting together items
     * into one buffer, the returned index can be provided for next item.
     * No NULL termination required.*/
    int i = 0;
    //while (s[i] !=0){
    for (i=0; i<string_len; i++){
        buffer[index] = s[i];
        //i++;
        index++;
    }
    return index;
}

int copy_str(char* buffer, const char* s, int index){
    /*Copy over string items into a buffer.
     * Provide index = 0 for simple 1-1 copy, for concatting together items
     * into one buffer, the returned index can be provided for next item.
     * Assuming null termination.*/
    int i = 0;
    while (s[i] !=0){
        buffer[index] = s[i];
        i++;
        index++;
    }
    return index;
}

char get_crc(char* buffer, char length){
    char j, crc = 0;
    int i;
    for (i=0; i<length; i++){
        crc ^= buffer[i];
        for (j=0; j<8; j++){
            if (crc & 1){ crc ^= CRC7_POLY; }
            crc >>= 1;
        }
    }
    return crc;
}

/* Values for coef array are emperically determined and seem to yield low count of duplicates,
 * within the context of the actual strings in TOPIC.
 */

int hash(char* buffer, int length){
    int i, c;
    uint8_t  t;
    int coef[] = {255, 7, 3, 1, 127, 63, 31, 15};
    i = 0;
    for (c=0; c<length; c++){
        i = (i + ((((int) buffer[c])) * coef[c])) % (MAX_TABLE_SIZE - 1);
    }
    return i;
}

int topic_to_int_hash(char* buffer, int length){
    int topic_nr = -1;
    int hash_val = -1;
    int ret_val = 0;
    hash_val = hash(buffer, 8);
    topic_nr = hash_table[hash_val];
    if (topic_nr!=-1 && length==8  && (cmp_i(buffer, topic_list[topic_nr], 8, 8))){
        ret_val = topic_nr;
    }else{
        ret_val = -1;
    }
    return ret_val;
}

void hash_table_setup(){
    int i, r;
    for (i=0; i<MAX_TABLE_SIZE; i++){
        hash_table[i] = -1;
    }

    for (i=0; i<TOPIC_AMOUNT; i++){
        r = hash((char*)topic_list[i], 8);
        hash_table[r] = i;
    }
}

uint8_t  append_segment_i(char* buffer, const char* str, int index, int string_len){
    int new_index;
    new_index = copy_str_i(buffer, str, index, string_len);
    buffer[new_index] = ':';
    new_index++;
    return new_index;
}

uint8_t  append_segment(char* buffer, const char* str, int index){
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
    return index;
}
// END utilities


int in_queue(struct Message_Queue *queue, int data) {
    char q_log[255];
    if (queue->count == MAX_MESSAGE_KEEP) {
        // Overflow condition, wrap around and overwrite oldest data
        queue->front = (queue->front + 1) % MAX_MESSAGE_KEEP;
    } else {
        queue->count++;
    }
    queue->buffer[queue->rear] = data;
    queue->rear = (queue->rear + 1) % MAX_MESSAGE_KEEP;
    return 0;
}

int out_queue(struct Message_Queue *queue, int *data) {
    char q_log[255];
    int return_val = 1;

    if (queue->count == 0) {
        // queue is empty
        //return_val = 1;
        return 1;
    }
    *data = queue->buffer[queue->front];
    queue->front = (queue->front + 1) % MAX_MESSAGE_KEEP;
    queue->count--;
    return_val = 0;
    return return_val;
}


/* Flag for comm logging pdr:
 * bit 0 [1]: PORT, port value.
 * bit 1 [2]: DIR, {0: inbound, 1: outbound}.
 * bit 2 [4]: RFLAG, {For DIR = 0: 0: self or gen addressed, 1: other and forwarded}.
 *               {For DIR = 1: 0: message response,      1: message forward}.
 * bit 3 [8]: ORG, {0: auto,    1: external}.
 *
 * */


void log_outbound_message(char* buffer, int mes_len, int pdr){
    //struct Message* new_message = messages[message_index];
    int i = 0;
    int j = 0;
    pdr = pdr + 2;
    comm_logger(buffer, mes_len, pdr);
}

void log_inbound_message(uint8_t message_index, uint8_t port){
    struct Message* new_message = messages[message_index];
    int i = 0;
    int j = 0;
    int pdr = 0;
    char buffer[1000];
    int message_len = (new_message->intCh_rx)-1; // Trimming NULL.

    pdr = pdr + (port*1);                    // 
    pdr = pdr + ((new_message->For_self)*4); // If for_self = true then for_gen = true also, meaning for_other = false.
    //copy_str(bufferdd
    for (i=0; i<message_len; i++){ // In order for message logging to work all NULLs are overwritten with spaces. This is in case payload contains NULLs.
        char c = new_message->CRC_buffer[i];
        if     (c==0){  buffer[i] = ' ';
        }else{          buffer[i] = c; }
    }
    //for (j=0; j<rx_segment_lengths[port][sg_CRC_INDEX]; j++){
    for (j=0; j<new_message->segment_len[sg_CRC_INDEX]; j++){
        buffer[i+j] = new_message->CRC[j];
    }
    buffer[i+j] = 0;
    comm_logger(buffer, i+j, pdr);
}


int write_crc_buffer(uint8_t message_index, uint8_t port, char ch) {
    struct Message* new_message;
    new_message = messages[message_index];
    new_message->CRC_buffer[new_message->CRC_index] = ch;
    new_message->CRC_index++;
    return 0;
}

uint8_t  write_into_segment(uint8_t  message_index, uint8_t port, char ch) {
    uint8_t return_val = 0;
    struct Message* new_message;
    new_message = messages[message_index];
    int P_seg_index = parser[port].segment_index;
    int P_char_index = parser[port].character_index;

    if (P_char_index <= SEGMENT_MAX_LENGTH[P_seg_index]){
        new_message->Segments[P_seg_index][P_char_index] = ch;
        P_char_index++;
        parser[port].character_index = P_char_index;
        new_message->segment_len[P_seg_index] = P_char_index;

        if (P_seg_index<sg_CRC_INDEX){
            write_crc_buffer(message_index, port, ch);
        }
    }else{ return_val = 1; }
    return return_val;
}

uint8_t write_into_payload(uint8_t message_index, uint8_t port, char ch) {
    uint8_t return_val = 0;
    struct Message* new_message;
    new_message = messages[message_index];
    int P_seg_index = parser[port].segment_index;
    int P_char_index = parser[port].character_index;

    if (P_char_index <= SEGMENT_MAX_LENGTH[P_seg_index] && P_char_index < new_message->intLength){
        new_message->Payload[P_char_index] = ch;
        write_crc_buffer(message_index, port, ch);
        P_char_index++;
        parser[port].character_index = P_char_index;
        new_message->segment_len[P_seg_index] = P_char_index;
    }else{
        write_crc_buffer(message_index, port, ':');
        P_seg_index++;
        parser[port].segment_index = P_seg_index;
        parser[port].character_index = 0;
    }
    return return_val;
}

uint8_t message_clear(uint8_t message_index, uint8_t port){
    uint8_t return_val = 0;
    parser[port].rx_i = 0;
    parser[port].segment_index = 0;
    parser[port].character_index = 0;
    //parser[port].prx_ch = 0;
    return return_val;
}

uint8_t f_000_message_clear(uint8_t message_index, uint8_t port, char ch) {  // Message clear
    uint8_t return_val = 0;
    message_clear(message_index, port);
    return return_val;
}

uint8_t message_start(int _, uint8_t port, char ch) { // Variable message_index is not valid here.
    uint8_t return_val = 0;                                             //bB
    struct Message* new_message;
    //parser[port].message_index = message_counter_global;
    parser[port].message_index = (parser[port].message_index + 1) % MAX_MESSAGE_KEEP;
    uint8_t message_index = parser[port].message_index;
    new_message = messages[message_index];
    parser[port].character_index = 0;
    parser[port].segment_index = 1;
    parser[port].prx_ch = 0;
    new_message->CRC_index = 0;
    write_crc_buffer(message_index, port, ':');
    write_crc_buffer(message_index, port, ':');
    message_counter_global = (message_counter_global + 1) % MAX_MESSAGE_KEEP;
    return return_val;
}

uint8_t f_110_two_colons(uint8_t message_index, uint8_t port, char ch) { // Two successive colons.
    uint8_t return_val = 0;
    message_start(message_index, port, ch);
    return return_val;
}

uint8_t f_111_three_colons(uint8_t message_index, uint8_t port, char ch) { // Three successives colons for some reason.
    uint8_t return_val = 0;
    message_clear(message_index, port);
    message_start(message_index, port, ch);
    return return_val;
}

uint8_t f_11x(uint8_t message_index, uint8_t port, char ch) { // Two successive colon mid message.
    uint8_t return_val = 0;
    message_start(message_index, port, ch);
    return return_val;
}

uint8_t f_100(uint8_t message_index, uint8_t port, char ch) { // Should not be invoked.
    uint8_t return_val = 0;
    return return_val;
}

uint8_t f_00x(uint8_t message_index, uint8_t port, char ch) { // Second+ character written into segments.
    uint8_t return_val = write_into_segment(message_index, port, ch);
    return return_val;
}

uint8_t f_10x(uint8_t message_index, uint8_t port, char ch) { // First character written into segments.
    uint8_t return_val = write_into_segment(message_index, port, ch);
    return return_val;
}

uint8_t f_01x(uint8_t message_index, uint8_t port, char ch) { // Colon marker for second+ segments.
    uint8_t return_val = 0;
    write_crc_buffer(message_index, port, ':');
    parser[port].character_index = 0;
    parser[port].segment_index = parser[port].segment_index + 1;

    return return_val;
}

uint8_t f_016(uint8_t message_index, uint8_t port, char ch) { // Segment for payload LENGTH is in.
    uint8_t return_val = 0;
    struct Message* new_message;
    new_message = messages[message_index];
    int P_seg_index = parser[port].segment_index;
    int P_char_index = parser[port].character_index;
    //int seg_cur_len = new_message->segment_len[P_seg_index];
    int payload_len = ascii_to_int_i(new_message->Segments[P_seg_index], P_char_index);
    new_message->intLength = payload_len;
    new_message->Payload = payloads[message_index];
    write_crc_buffer(message_index, port, ':');

    parser[port].character_index = 0;
    parser[port].segment_index = parser[port].segment_index + 1;
    return return_val;
}


uint8_t f_xx7(uint8_t message_index, uint8_t port, char ch) { // Writing into PAYLOAD.
    uint8_t return_val = write_into_payload(message_index, port, ch);
    return return_val ;
}

uint8_t f_x08_penultimate_segment(uint8_t message_index, uint8_t port, char ch) { // Writing into penultimate segment.
    uint8_t return_val = write_into_segment(message_index, port, ch);
    return return_val;
}

uint8_t f_018_marking_last_segment(uint8_t message_index, uint8_t port, char ch) { // Marking start last segment. CRC should follow.
    uint8_t return_val = 0;
    write_crc_buffer(message_index, port, ':');
    parser[port].character_index = 0;
    parser[port].segment_index = parser[port].segment_index + 1;
    return return_val;
}


uint8_t f_019_message_complete(uint8_t message_index, uint8_t port, char ch) { // Last colon. Message is complete.
    int r;
    uint8_t return_val = 0;
    struct Message* new_message;
    //new_message = messages[message_index];
    //write_crc_buffer(message_index, port, ':'); // Completing CRC buffer.
    //write_crc_buffer(message_index, port, 0); //   Null terminating for printing purposes.

    //parser[port].character_index = 0;
    //parser[port].message_index = (parser[port].message_index + 1) % MAX_MESSAGE_KEEP;
    r = process_message(message_index, port);
    return_val = message_clear(message_index, port);
    //parser[port].prx_ch = 0; // UGLY!
    return return_val;
}

uint8_t f_x09_writing_into_last_segment(uint8_t message_index, uint8_t port, char ch) { // Writing into last segment.
    //parser[port].character_index = parser[port].character_index + 1;
    uint8_t return_val = write_into_segment(message_index, port, ch);
    return return_val;
}


uint8_t process_message(uint8_t message_index, uint8_t port) { // Writing into last segment.
    int r;
    uint8_t return_val = 0;
    struct Message* new_message;
    new_message = messages[message_index];
    new_message->intCh_rx = new_message->CRC_index  + 1;
    //new_message->CRC_buffer[new_message->CRC_index] = 0;
    uint8_t crc_val     =  get_crc(new_message->CRC_buffer, new_message->CRC_index);
    int crc_rx      =  ascii_to_int_i(new_message->Segments[sg_CRC_INDEX], new_message->segment_len[sg_CRC_INDEX]);
    int hops        =  ascii_to_int_i(new_message->Segments[sg_HOPCOUNT_INDEX], new_message->segment_len[sg_CRC_INDEX]);
    int topic_nr    =  topic_to_int_hash(new_message->Segments[sg_TOPIC_INDEX], new_message->segment_len[sg_TOPIC_INDEX]);
    new_message->intCRC_rx      =  crc_rx;
    new_message->intCRC_cal     =  crc_val;
    new_message->intHops        =  hops;
    new_message->Topic_number   =  topic_nr;
    new_message->CRC_check      =  (crc_rx==crc_val);
    new_message->Sender         =  new_message->Segments[sg_SENDER_INDEX];
    new_message->Receiver       =  new_message->Segments[sg_RECEIVER_INDEX];
    new_message->RQ             =  new_message->Segments[sg_RQS_INDEX];
    new_message->Topic          =  new_message->Segments[sg_TOPIC_INDEX];
    new_message->Chapter        =  new_message->Segments[sg_CHAPTER_INDEX];
    new_message->Hops           =  new_message->Segments[sg_HOPCOUNT_INDEX];
    new_message->CRC            =  new_message->Segments[sg_CRC_INDEX];
    new_message->Port           =  port;
    //int i;
    if (new_message->CRC_check==0){
        //errors.crc_errors[port]++;
    }else{
         r = in_queue(&Queue[port], message_index);

         if (U2U_PLATFORM_CHANNELS==1){
             log_inbound_message(message_index, port);
         }

    }
    if (new_message->CRC_check || RESPOND_ON_BAD_CRC){

        return_val = message_processor(message_index);
        r = 0;
    } else {
    }

    return return_val;
}


//
//RQS_r[] = "RS";
//RQS_q[] = "RQ";
//RQS_i[] = "RI";
//RQS_n[] = "RN";
//RQS_a[] = "RA";
//

uint8_t determine_addressee(uint8_t message_index, uint8_t port){ //0: Other, 1: Self, 2: General.
    struct Message* new_message = messages[message_index];
    uint8_t  router_value = 0;
    if (new_message->Segments[sg_RQS_INDEX][0] != 'R'){
        errors.r_failures[port]++;
        router_value = 5;
    }else{ //On R-flag cmp: no need to check 'R' char for each flag each time.
        router_value = router_value + (1 * cmp_i(new_message->Segments[sg_RECEIVER_INDEX], self_name, new_message->segment_len[sg_RECEIVER_INDEX], len(self_name)));
        router_value = router_value + (2 * cmp_i(new_message->Segments[sg_RECEIVER_INDEX], "GEN", new_message->segment_len[sg_RECEIVER_INDEX], len("GEN")));
        router_value = router_value + (4 * cmp_i(new_message->Segments[sg_RQS_INDEX], RQS_i, new_message->segment_len[sg_RQS_INDEX], len(RQS_i)));
        router_value = router_value + (4 * cmp_i(new_message->Segments[sg_RQS_INDEX], RQS_r, new_message->segment_len[sg_RQS_INDEX], len(RQS_i)));
        router_value = router_value + (4 * cmp_i(new_message->Segments[sg_RQS_INDEX], RQS_n, new_message->segment_len[sg_RQS_INDEX], len(RQS_i)));
        router_value = router_value + (4 * cmp_i(new_message->Segments[sg_RQS_INDEX], RQS_a, new_message->segment_len[sg_RQS_INDEX], len(RQS_i)));
    }
    new_message->Router_val = router_value;
    
    return router_value;
}

int message_topic_checker(uint8_t message_index, uint8_t port, char* buffer){
    uint8_t i = 0;
    int topic_function_selector= -1;
    topic_function_selector = messages[message_index]->Topic_number; // Topic_number is result from hash-lookup and is asigned during func process_message.
    //topic_function_selector = topic_to_int_hash(messages[message_index]->Segments[sg_TOPIC_INDEX]);
    if (topic_function_selector>=0){
        i = copy_str(buffer, msg_responses[topic_function_selector], 0);
    }else{
        i = copy_str(buffer, default_response, 0);
    }
    buffer[i] = 0;
    return  topic_function_selector;
}



//const char self_name[] = "BUTTERLING";
//const char self_name_TEST[] = "SELFNAME";


//sg_WAIT_INDEX         = 0;
//sg_PREMESSAGE_INDEX   = 0;
//sg_SENDER_INDEX       = 1;
//sg_RECEIVER_INDEX     = 2;
//sg_RQS_INDEX          = 3;
//sg_TOPIC_INDEX        = 4;
//sg_CHAPTER_INDEX      = 5;
//sg_LENGTH_INDEX       = 6;
//sg_PAYLOAD_INDEX      = 7;
//sg_HOPCOUNT_INDEX     = 8;
//sg_CRC_INDEX          = 9;


// compose_transmit_message function still uses NULL terminated strings for func append_segment as those are communicated from main and are expected to be null terminated. We have no other way of getting the length passed on.
uint8_t compose_transmit_message(struct Message* message_tx, char* buffer, int* message_length){
    uint8_t return_value = 0;
    int index = 0;
    int i = 0;
    //char response_buffer[MAX_MESSAGE_SIZE];
    char len_buf[8];
    char chapter_buf[8];
    const char* nill = {"0"};
//    message_tx->ID = tx_id;
//    tx_id++;
    int_to_ascii(len_buf, len(message_tx->Payload), 3);
    int_to_ascii(chapter_buf, message_tx->intChapter, 3);
    i = copy_str(message_tx->Segments[sg_CHAPTER_INDEX], chapter_buf, 0);
    message_tx->Segments[sg_CHAPTER_INDEX][i] = 0;
    message_tx->Chapter = message_tx->Segments[sg_CHAPTER_INDEX];
    buffer[index] = ':'; //SENDER
    index++;
    buffer[index] = ':'; //SENDER
    index++;
    index = append_segment(buffer, self_name, index);
    index = append_segment(buffer, message_tx->Receiver, index);
    index = append_segment(buffer, message_tx->RQ, index);
    index = append_segment(buffer, message_tx->Topic, index);
    index = append_segment(buffer, message_tx->Chapter, index);
    index = append_segment(buffer, len_buf, index);
    index = append_segment(buffer, message_tx->Payload, index);
    index = append_segment(buffer, nill, index); //Hopcount is nill
    index = add_crc(buffer, index);
    buffer[index] = ':';
    index++;
    buffer[index] = '\0';
    *message_length = index;
    return return_value;
}

uint8_t log_write_from_uart0(char* buffer, int mes_len, int pdr){
    uint8_t return_value = 0;
    pdr = pdr + 0;
    log_outbound_message(buffer, mes_len, pdr);
    write_from_uart0(buffer, mes_len);
    return return_value;
}

uint8_t log_write_from_uart1(char* buffer, int mes_len, int pdr){
    uint8_t return_value = 0;
    pdr = pdr + 1;
    log_outbound_message(buffer, mes_len, pdr);
    write_from_uart1(buffer, mes_len);
    return return_value;
}



uint8_t u2u_send_message_uart0(struct Message* message_tx){
    uint8_t return_value = 0;
    int index = 0;
    char buffer[MAX_MESSAGE_SIZE];
    int mes_len;
    int pdr = 8;
    return_value = compose_transmit_message(message_tx, buffer, &mes_len);
    //write_from_uart0(buffer, mes_len); // Calling on u2u_HAL function.
    log_write_from_uart0(buffer, mes_len, pdr); // Calling on u2u_HAL function.
    return return_value;
}

//Some constants are used here to set the length against a fixed value (3). This is ugly. 
    
uint8_t compose_response_message(uint8_t message_index, uint8_t port, char* buffer, int* message_length){
    struct Message* new_message = messages[message_index];
    uint8_t return_value = 0;
    int index = 0;
    char response_buffer[MAX_MESSAGE_SIZE];
    int  topic_function_selector;
    char len_buf[4];
    char hops_buf[3];
    int hops; 
    buffer[index] = ':'; //RESET
    index++;
    buffer[index] = ':'; //SENDER
    index++;
    index = append_segment_i(buffer, self_name, index, len(self_name));
    index = append_segment_i(buffer, new_message->Segments[sg_SENDER_INDEX], index, new_message->segment_len[sg_SENDER_INDEX]);
    topic_function_selector = message_topic_checker(message_index, port, response_buffer);
    //topic_function_selector = new_message->Topic_number;
    if (new_message->CRC_check==1){
        if (topic_function_selector == -1){
            index = append_segment_i(buffer, RQS_a, index, len(RQS_a));
            //return_value = 1;
        }else{
            index = append_segment_i(buffer, RQS_r, index, len(RQS_r));
        }
    }else{
        index = append_segment_i(buffer, RQS_n, index, len(RQS_n));
    }
    //new_message->Topic_number = topic_function_selector;
    index = append_segment_i(buffer, new_message->Segments[sg_TOPIC_INDEX], index, new_message->segment_len[sg_TOPIC_INDEX]);
    index = append_segment_i(buffer, new_message->Segments[sg_CHAPTER_INDEX], index, new_message->segment_len[sg_CHAPTER_INDEX]);
    int_to_ascii(len_buf, len(response_buffer), 3);
    index = append_segment_i(buffer, len_buf, index, 3);
    index = append_segment_i(buffer, response_buffer, index, len(response_buffer));
    hops = ascii_to_int_i(new_message->Segments[sg_HOPCOUNT_INDEX], new_message->segment_len[sg_HOPCOUNT_INDEX]);
    hops++;
    int_to_ascii(hops_buf, hops, 3);
    index = append_segment_i(buffer, hops_buf, index, 3);
    index = add_crc(buffer, index);
    buffer[index] = ':';
    index++;
    *message_length = index;
    buffer[index] = '\0';
    sprintf(log_buffer, "U2U: %s:: topic_f_s: %d, buffer: <%s>.",__func__, topic_function_selector, buffer);
    return return_value;
}

uint8_t compose_forward_message(uint8_t message_index, uint8_t port, char* buffer, int* message_length){
    struct Message* new_message = messages[message_index];
    uint8_t return_value = 0;
    int index = 0;
    char response_buffer[MAX_MESSAGE_SIZE];
    uint8_t  i = 0;
    char hops_buf[3];
    uint8_t  hops;
    buffer[index] = ':';
    index++;
    buffer[index] = ':';
    index++;
    for (i=sg_SENDER_INDEX; i<sg_PAYLOAD_INDEX; i++){
        index = append_segment_i(buffer, new_message->Segments[i], index, new_message->segment_len[i]);
        //sprintf(log_buffer, "U2U: %s:: buffer: <%s>, seg: <%s>, i: %d, len: %d.",__func__, buffer, new_message->Segments[i], index, new_message->segment_len[i]);
    }
    index = append_segment_i(buffer, new_message->Payload, index, new_message->intLength);
    hops = ascii_to_int_i(new_message->Segments[sg_HOPCOUNT_INDEX], new_message->segment_len[sg_HOPCOUNT_INDEX]);
    hops++;
    int_to_ascii(hops_buf, hops, 3);
    index = append_segment_i(buffer, hops_buf, index, 3);
    index = add_crc(buffer, index);
    buffer[index] = ':';
    index++;
    *message_length = index;
    buffer[index] = '\0';
    sprintf(log_buffer, "U2U: %s:: buffer: <%s>.",__func__, buffer);
    return return_value;
}

 
uint8_t u2u_send_message_uart1(struct Message* message_tx){
    uint8_t return_value = 0;
    //int index = 0;
    char buffer[MAX_MESSAGE_SIZE];
    //int i;
    int pdr = 8;


    int mes_len;
    return_value = compose_transmit_message(message_tx, buffer, &mes_len);
    //write_from_uart1(buffer, mes_len); // Calling on u2u_HAL function.
    log_write_from_uart1(buffer, mes_len, pdr); // Calling on u2u_HAL function.
    sprintf(log_buffer, "U2U: %s:: buffer: <%s>.",__func__, buffer);
    return return_value;
}
 
uint8_t u2u_send_message(struct Message* message_tx){
    uint8_t return_value = 0;
    //int index = 0;
    char buffer[MAX_MESSAGE_SIZE];
    int mes_len;
    int pdr = 8;
    return_value = compose_transmit_message(message_tx, buffer, &mes_len);
    //write_from_uart0(buffer, mes_len); // Calling on u2u_HAL function.
    log_write_from_uart0(buffer, mes_len, pdr); // Calling on u2u_HAL function.
    //write_from_uart1(buffer, mes_len); // Calling on u2u_HAL function.
    log_write_from_uart1(buffer, mes_len, pdr); // Calling on u2u_HAL function.
    return return_value;
}


uint8_t self_call(uint8_t message_index, uint8_t port){
    uint8_t return_value = 0;
    char buffer[MAX_MESSAGE_SIZE];
    int mes_len;
    int pdr = 4;
    return_value = compose_response_message(message_index, port, buffer, &mes_len);
    if (return_value==0){ // For costum topics responses are handled manually (i.e. user).
        return_value = (*uart_write_functions[port])(buffer, mes_len, pdr);
    }
    messages[message_index]->For_self = 1;
    messages[message_index]->For_gen = 0;
    //messages[message_index]->Manual_response_waiting = 1;
    return return_value;
}

uint8_t other_call(uint8_t message_index, uint8_t port){
    uint8_t return_value = 0;
    char buffer[MAX_MESSAGE_SIZE];
    uint8_t other_port = 1 - port;
    int mes_len;
    int pdr = 0;
    //port = 1 - port;
    return_value = compose_forward_message(message_index, other_port, buffer, &mes_len);
    return_value = (*uart_write_functions[other_port])(buffer, mes_len, pdr);
    messages[message_index]->For_self = 0;
    messages[message_index]->For_gen = 0;
    return return_value;
}

uint8_t no_response_call(uint8_t message_index, uint8_t port){
    uint8_t return_value = 0;
    messages[message_index]->For_self = 0;
    messages[message_index]->For_gen = 0;
    return return_value;
}

uint8_t general_call(uint8_t message_index, uint8_t port){
    uint8_t return_value = 0;
    char buffer[MAX_MESSAGE_SIZE];
    uint8_t other_port = 1 - port;
    int mes_len;
    int pdr = 4;
    return_value = compose_response_message(message_index, port, buffer, &mes_len);
    if (return_value==0){ // For costum topics responses are handled manually (i.e. user).
        return_value = (*uart_write_functions[port])(buffer, mes_len, pdr);
    }
    //return_value = (*uart_write_functions[port])(buffer, mes_len, pdr);
    return_value = compose_forward_message(message_index, port, buffer, &mes_len);
    //port = 1 - port;
    return_value = (*uart_write_functions[other_port])(buffer, mes_len, pdr);
    messages[message_index]->For_self = 1;
    messages[message_index]->For_gen = 1;
    return return_value;
}


    ////sprintf(log_buffer, "U2U: format_return_message  message_index: %d.", message_index);
    ////u2u_logger(log_buffer, 4);

uint8_t message_processor(uint8_t message_index){
    uint8_t r = 0;
    int res = 0;
    uint8_t port = messages[message_index]->Port;
    int router_value;

    if (messages[message_index]->CRC_check==0){
        router_value = 1;
    }else{
        router_value = determine_addressee(message_index, port);
    }
    res = (*call_router_functions[router_value])(message_index, port);
    return r;
}


struct Message* get_message(){
    int m_q_index, m;
    //char gm_log_buffer[255];
    static int queue_turn = 0;
    
    queue_turn = 1 - queue_turn;
    m_q_index = out_queue(&Queue[queue_turn], &m);
    if (m_q_index==0){
        return messages[m];
    
    }else{
        return NULL;
    }
}

uint8_t u2u_topic_exchange(char* custom_payload, uint8_t topic_number){
    uint8_t return_val = 0;
    msg_responses[topic_number] = custom_payload;
    return return_val;
}

/*This is called by u2u_HAL on UART rx interupt.*/
uint8_t uart0_character_processor(char ch){
    uint8_t r = 1;
    uint8_t port = 0;
    r = character_processor(port, ch);
    return r;
}

/*This is called by u2u_HAL on UART rx interupt.*/
uint8_t uart1_character_processor(char ch){
    uint8_t r = 1;
    uint8_t port = 1;
    r = character_processor(port, ch);
    return r;
}

uint8_t character_processor(uint8_t port, char ch){
    uint8_t r = 1;
    uint8_t m, mo;
    uint8_t message_index = parser[port].message_index;
    parser[port].prx_ch = parser[port].rx_ch;
    parser[port].rx_ch = ch;
    parser[port].rx_i++;
    m  = ch_mapper[(uint8_t) parser[port].rx_ch];
    mo = ch_mapper[(uint8_t) parser[port].prx_ch];
    //printf("%c,   %d, %d, %2d,     %2d  --- ", ch,  mo,  m, parser[port].segment_index, parser[port].character_index);
    r = parser[port].pr_mapper[mo][m][parser[port].segment_index](message_index, port,  ch);
    //printf("(%5d    ---  %2d, %2d)\n", r, parser[port].segment_index, parser[port].character_index);
    return 0;
}

uint8_t u2u_self_test(uint8_t port){ 
    FILE* fptr; 
    char buffer[512]; 
    uint8_t return_val = 0; 
    int len_m = 0; 
    char ch; 
    if ((fptr = fopen(test_messages, "r")) == NULL){ 
        printf("%s:: Error reading test messages at: %s.\n", __func__, test_messages); 
        return 1; 
    } 

    while(fgets(buffer, sizeof(buffer), fptr) != NULL){ 
        len_m++; 
        int i = 0; 
        ch = '.'; 
        while(ch!='\0'){ 
            ch = buffer[i]; 
            //printf("::%d, <%c>\n", i, ch); // %%MATRIX%%
            return_val = character_processor(port, ch); 
            i++; 
        } 
        //printf("%d, %s", len_m, buffer); // %%MATRIX%%
    } 
    if (feof(fptr)){ 
        //printf("END of file\n."); 
 
    } 
 
    fclose(fptr); 
    return return_val; 
} 



void parser_setup(){
    int i;
    uint8_t x, y, z;
    message_counter_global = 0;
    parser[0].rx_i = 0;
    parser[0].segment_index = 0;
    parser[0].character_index = 0;
    parser[0].message_index = 0;
    parser[1].rx_i = 0;
    parser[1].segment_index = 0;
    parser[1].character_index = 0;
    parser[1].message_index = 0;
    for (i=0; i<255; i++) {
        ch_mapper[i] = 0;
    }
    ch_mapper[58] = 1; // Colon

    for (x=0; x<PR_MO; x++) {
        for (y=0; y<PR_M; y++) {
            for (z=0; z<PR_SG; z++) {
                parser[0].pr_mapper[x][y][z] = &f_000_message_clear;
                parser[1].pr_mapper[x][y][z] = &f_000_message_clear;
            }
        }
    }

    for (x=1; x<12; x++) {
        parser[0].pr_mapper[0][0][x] = &f_00x;
        parser[0].pr_mapper[1][0][x] = &f_10x;
        parser[0].pr_mapper[0][1][x] = &f_01x;
        parser[1].pr_mapper[0][0][x] = &f_00x;
        parser[1].pr_mapper[1][0][x] = &f_10x;
        parser[1].pr_mapper[0][1][x] = &f_01x;
        parser[0].pr_mapper[1][1][x] = &f_11x;
        parser[1].pr_mapper[1][1][x] = &f_11x;
    }

    parser[0].pr_mapper[1][1][0] = &f_110_two_colons;
    parser[1].pr_mapper[1][1][0] = &f_110_two_colons;
    parser[0].pr_mapper[1][1][1] = &f_111_three_colons;
    parser[1].pr_mapper[1][1][1] = &f_111_three_colons;

    parser[0].pr_mapper[0][1][6] = &f_016;
    parser[1].pr_mapper[0][1][6] = &f_016;

    parser[0].pr_mapper[0][0][7] = &f_xx7;
    parser[1].pr_mapper[0][0][7] = &f_xx7;
    parser[0].pr_mapper[0][1][7] = &f_xx7;
    parser[1].pr_mapper[0][1][7] = &f_xx7;
    parser[0].pr_mapper[1][0][7] = &f_xx7;
    parser[1].pr_mapper[1][0][7] = &f_xx7;
    parser[0].pr_mapper[1][1][7] = &f_xx7;
    parser[1].pr_mapper[1][1][7] = &f_xx7;

    parser[0].pr_mapper[0][0][8] = &f_x08_penultimate_segment;
    parser[1].pr_mapper[0][0][8] = &f_x08_penultimate_segment;
    parser[0].pr_mapper[0][1][8] = &f_018_marking_last_segment;
    parser[1].pr_mapper[0][1][8] = &f_018_marking_last_segment;
    parser[0].pr_mapper[1][0][8] = &f_x08_penultimate_segment;
    parser[1].pr_mapper[1][0][8] = &f_x08_penultimate_segment;

    parser[0].pr_mapper[0][0][9] = &f_x09_writing_into_last_segment;
    parser[1].pr_mapper[0][0][9] = &f_x09_writing_into_last_segment;
    parser[0].pr_mapper[1][0][9] = &f_x09_writing_into_last_segment;
    parser[1].pr_mapper[1][0][9] = &f_x09_writing_into_last_segment;

    parser[0].pr_mapper[0][1][9] = &f_019_message_complete;
    parser[1].pr_mapper[0][1][9] = &f_019_message_complete;
}

uint8_t u2u_message_setup(){
    //uint8_t i, res;
    hash_table_setup();
    parser_setup();

    //mo.Sender = mo_sender;
    //mo.Receiver = mo_receiver;
    //mo.Payload = mo_payload;
    Queue[0].front                                 = 0;
    Queue[0].rear                                  = 0;
    Queue[0].count                                 = 0;
    Queue[1].front                                 = 0;
    Queue[1].rear                                  = 0;
    Queue[1].count                                 = 0;
    messages[0]                                 = &mi0;
    messages[1]                                 = &mi1;
    messages[2]                                 = &mi2;
    messages[3]                                 = &mi3;
    messages[4]                                 = &mi4;
    messages[5]                                 = &mi5;
    messages[6]                                 = &mi6;
    messages[7]                                 = &mi7;

    call_router_functions[0]                    = &other_call;
    call_router_functions[1]                    = &self_call;
    call_router_functions[2]                    = &general_call;
    call_router_functions[3]                    = &general_call;
    call_router_functions[4]                    = &other_call;
    call_router_functions[5]                    = &no_response_call;
    call_router_functions[6]                    = &other_call;
    call_router_functions[7]                    = &other_call;
    call_router_functions[8]                    = &other_call;

    uart_write_functions[0]                     = &log_write_from_uart0;
    uart_write_functions[1]                     = &log_write_from_uart1;
    if (u2u_uart_setup()){
        return 1;
    }
    return 0;

}


uint8_t u2u_close(void){
    uint8_t r;
    r = u2u_uart_close();
    return r;
}



