
//u2u.c
#include "u2u.h"
//
#if   U2U_PLATFORM_CHANNELS == 1
uint8_t  MAX_PORTS = 2;
#elif U2U_PLATFORM_CHANNELS == 2
uint8_t  MAX_PORTS = 2;
#elif U2U_PLATFORM_CHANNELS == 3
uint8_t  MAX_PORTS = 3;
#endif

#include <time.h>


#define MAX_TABLE_SIZE 255   // HASH table size
#define MAX_UART_BUFFER_SIZE 1024 * 4 
#define RESPONSE_HOP_ADD_INBOUND 0
#define BLOCK_SIZE 128   // Chunks of bytes each round of message parsign takes on from the UART queue.

uint8_t UPC;   
size_t self_name_len;

/* Large buffers allocated upfront */
static char compose_buffer[PORT_COUNT][MAX_MESSAGE_SIZE];
static char payload_buffer[PORT_COUNT][MAX_PAYLOAD_SIZE];

char* topic_responses[MAX_TOPIC_RESPONSES];


const uint8_t    CRC7_POLY              = 0x91;
int topic_hash_table[MAX_TABLE_SIZE];
const char* topic_list[]    = { "HAIL____", "HELP____", "SET_LCD_", "SET_OLED", "GET_SNSR", "SET_ENCR", "GET_ENCR", "SET_LED_", "SET_TIME", "GET_TIME", "SET_DATE", "GET_DATE", "RSERVD_0", "RSERVD_1", "RSERVD_2", "RSERVD_3", "RSERVD_4"};





// Utilities
int ascii_to_int(char* str){
    int length = 0;
    int sum = 0;
    int decs[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    while (str[length] != 0){ length++; }
    for (int c=0; c<length; c++){
        if (str[c]>='0' && str[c]<='9'){
            sum = sum + (decs[length-c-1] * (str[c]-'0'));
        }
    }
    return sum;
}

int ascii_to_int_i(char* str, int length){ // Not reliant on NULL termination.
    int sum = 0;
    int decs[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    for (int c=0; c<length; c++){
        if (str[c]>='0' && str[c]<='9'){
            sum = sum + (decs[length-c-1] * (str[c]-'0'));
        }
    }
    return sum;
}

/* Convert integer to char array in buf taking p space. Buffer space must be allocated to accomodate p bytes.
 * If the bytes taken by the string representation is larger than the number it fills, it will be padded with 
 * zeroes.
 * Returns the length of the number minus the null character.
 */

size_t int_to_ascii(char* buf, int integer, int p) { // If p is 0 then p is determined.
    int tens[] = {10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};
    int f = 0; //p denotes 'precision' or number of digits.
    int c = 0;
    int j = 0;
    const int M = 7;
    if (integer==0){ // Going to be zero often, so cutting this short.
        if (p==0 || p==1){
            buf[0] = '0';
            buf[1] = '\0';
        }else{
            for (c=0; c<p; c++){
                buf[c] = '0';
            }
            buf[c] = '\0';
        }
        c++;
    }else{
        if (p==0){
            for (j=M; j>0; --j){
                if (integer >= tens[j]){ p++; }else{ break; }
            }
        }
        f = M-p;
        for (c=0; c<p; c++) {
            buf[c] = '0' + (integer % tens[f+c] / (tens[f+c+1]));
        }
        buf[c] = '\0';
    }
    return c;
}


uint32_t ascii_to_hex_i(const char *str, size_t length) {
    uint32_t result = 0;
    for (size_t i = 0; i < length; i++) {
        char ch = str[i];
        uint32_t value;

        if (ch >= '0' && ch <= '9') {
            value = ch - '0';
        } else if (ch >= 'A' && ch <= 'F') {
            value = ch - 'A' + 10;
        } else if (ch >= 'a' && ch <= 'f') {
            value = ch - 'a' + 10;
        } else {
            return -1; // invalid character
        }
        if (result > 0x0FFFFFFF) {
            return -2; // would overflow 32-bit
        }
        result = (result << 4) | value;
    }

    return result;
}


size_t hex_to_ascii(char *buf, uint32_t value, size_t width) {
    char tmp[16];
    size_t i = 0;
    if (value == 0) {
        size_t length = (width > 1) ? width : 1;
        for (size_t j = 0; j < length; j++) {
            buf[j] = '0';
        }
        buf[length] = '\0';
        return length;
    }
    while (value > 0) {
        tmp[i++] = "0123456789ABCDEF"[value & 0xF];
        value >>= 4;
    }

    size_t length = (i > width) ? i : width;
    size_t pad = length - i;
    size_t pos = 0;

    // Add zero padding.
    for (size_t j = 0; j < pad; j++) {
        buf[pos++] = '0';
    }
    // Reverse back into output.
    while (i > 0) {
        buf[pos++] = tmp[--i];
    }

    buf[pos] = '\0';
    return pos;
}

int float_to_ascii(char* buf, float f, int p){
    float tens[] = {1., 10., 100., 1000., 10000., 100000.};
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

int cmp_i(const char* s1, const char* s2, int l1, int l2){
    /*Return True if strings compare equal, False if not or if empty. 
     * Not reliant on Null termination.*/
    int r = 0;
    if (l1==l2){
        if (l1==0){ return r;}
        else{
            r = 1;
            for (int c=0; c<l1; c++){
                r = r * ((uint8_t) s1[c] == (uint8_t) s2[c]);
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

uint8_t get_crc(char* buffer, char length){
    uint8_t j, crc = 0;
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
 * within the context of the actual strings in TOPIC. See Python script hash_function_trials.py
 */

uint8_t get_topic_hash(char* buffer, int length){
    uint8_t i, c;
    uint8_t coef[] = {127, 7, 31, 15, 63, 1, 255, 3};
    i = 0;
    for (c=0; c<length; c++){
        //i = (i + ((((int) buffer[c])) * coef[c])) % (MAX_TABLE_SIZE - 1);
        i = i + (((uint8_t) buffer[c] - 48) * coef[c]);
    }
    i = i & 0x00ff;
    return i;
}

/* Todo: worthwhile checking the actual topic string against the topic in the list if a match is found to avoid
 * hash collisions but since custom topics are not supported this has not been implemented yet. 
 * Function will return -1 if no match.
 */
int topic_to_int_hash(char* buffer, int length){
    int topic_nr = -1;
    uint8_t hash_val;
    int ret_val = 0;
    hash_val = get_topic_hash(buffer, 8);
    topic_nr = topic_hash_table[hash_val];
    if (topic_nr!=-1 && length==8  && (cmp_i(buffer, topic_list[topic_nr], 8, 8))){
        ret_val = topic_nr;
    }else{
        ret_val = -1;
    }
    return ret_val;
}

void topic_hash_table_init(){
    uint8_t i, r;
    for (i=0; i<MAX_TABLE_SIZE; i++){
        topic_hash_table[i] = -1;
    }
    for (i=0; i<MAX_TOPIC_RESPONSES; i++){
        r = get_topic_hash((char*)topic_list[i], 8);
        topic_hash_table[r] = i;
    }
}

//struct Topic_Items{


struct Parser{
    struct Message* message;
    size_t   char_counter;
    char     prm_seg_len_buffer[12];   // Takes chars repr. length of each segment, null term'd.
    uint8_t  prm_seg_len_buffer_count; // Counts prm_seg_len_buffer chars.
    size_t   prm_seg_len_array[32];    // Takes length of each segment.
    uint8_t  prm_seg_len_ix;           // Keeps count of what segment is being worked on.
    char     buffer[MAX_MESSAGE_SIZE];
    int      state;
    char     p_c[2];
};

struct Message_Queue{       
    int     buffer[MAX_MESSAGE_KEEP];
    uint8_t port;           // Only for debugging purposes.
    uint8_t count;
    uint8_t front;
    uint8_t rear;
    uint8_t max;
    bool    full;
    uint8_t free_index;
};

struct Circular_Queue{
    char   buffer[MAX_UART_BUFFER_SIZE];
    volatile uint16_t write_index;
    volatile uint16_t read_index;
};


void circular_queue_init(struct Circular_Queue* queue){
    queue->write_index = 0;
    queue->read_index = 0;
}

void message_queue_clear(struct Message_Queue* queue){
    queue->front = 0;
    queue->rear = 0;
    queue->count = 0;
    queue->full = false;
    queue->free_index = 0;
    for (int i=0; i<queue->max; i++){
        queue->buffer[i] = -1;
    }
}

void message_queue_init(struct Message_Queue* queue, uint8_t port){
    queue->port = port;
    queue->max = MAX_MESSAGE_KEEP;
    message_queue_clear(queue);
}

uint8_t get_queue_max(struct Message_Queue* queue){
    return queue->max;
}

uint8_t get_queue_size(struct Message_Queue* queue){
    //return queue->count;
    size_t size = queue->max;
    if (queue->full){
        size = (queue->max + queue->front - queue->rear);
    }else{
        size = (queue->front - queue->rear);
    }
    return size;
}

uint8_t get_next_free(struct Message_Queue* queue){
    uint8_t message_list_index = queue->free_index;
    if (++queue->free_index >= MAX_MESSAGE_KEEP){
        queue->free_index = 0;
    }
    return message_list_index;
    //return queue->front;
}


bool queue_full(struct Message_Queue* queue){
    return queue->full;
}

bool queue_empty(struct Message_Queue* queue){
    return (!queue->full && (queue->front == queue->rear));
}

int in_queue(struct Message_Queue* queue, uint8_t index){
    queue->buffer[queue->front] = index;
    if (queue->full){
        if (++(queue->rear) == queue->max){
            queue->rear = 0;
        }
    }
    if (++(queue->front) == queue->max){
        queue->front = 0;
    }
    queue->full = (queue->front == queue->rear);
    return 0;
}

/* Returns 0 if queue is not empty and -1 otherwise */
int out_queue(struct Message_Queue* queue, uint8_t* index){
    if (queue_empty(queue)){
        return -1;
    }
    *index = queue->buffer[queue->rear];
    if (++(queue->rear) == queue->max){
        queue->rear = 0;
    }
    queue->full = false;
    return 0;
}

int write_to_circular_queue(struct Circular_Queue* queue, char ch) {
    int ret_val = 0;
    uint16_t temp_write_index = queue->write_index + 1;
    if (temp_write_index >= MAX_UART_BUFFER_SIZE){ // Avoiding MODULO % might be overzealous with powers of 2.
        temp_write_index = 0;
    }
    if (temp_write_index == queue->read_index){  // Full buffer.
       ret_val = 1;
    }else{
        queue->buffer[queue->write_index] = ch;
        if (++queue->write_index==MAX_UART_BUFFER_SIZE){
            queue->write_index = 0;
        }
        ret_val = 0;
    }
    return ret_val;
}

int read_from_circular_queue(struct Circular_Queue* queue, char* ch) {
    int ret_val = 0;
    if (queue->read_index == queue->write_index) { // Empty buffer.
       ret_val = 1;
    }else{
        *ch = queue->buffer[queue->read_index];
        if (++queue->read_index==MAX_UART_BUFFER_SIZE){
            queue->read_index = 0;
        }
        ret_val = 0;
    }
    return ret_val;
}

struct U2U_Errors{
    uint32_t error_count[PORT_COUNT];
    uint32_t drb_uart[PORT_COUNT];
    uint32_t crc_handler[PORT_COUNT]; 
    uint32_t seg_handler[PORT_COUNT];
    uint32_t seg_crc_missing[PORT_COUNT];
    uint32_t seg_rflag_missing[PORT_COUNT];
    uint32_t seg_rflag_fault[PORT_COUNT];
    uint32_t seg_rflag_unknown[PORT_COUNT];
    uint32_t len_handler[PORT_COUNT];
    uint32_t rfl_handler[PORT_COUNT];
    uint32_t rfs_handler[PORT_COUNT];
    uint32_t grs_router[PORT_COUNT];
    uint32_t rcf_router[PORT_COUNT];
    uint32_t srn_router[PORT_COUNT];
};

struct Message message_list[PORT_COUNT][MAX_MESSAGE_KEEP];
struct Parser parser[PORT_COUNT];
struct Message_Queue queue[PORT_COUNT];    // Accomodating for all  ports.
struct Circular_Queue circular_queue[PORT_COUNT];
struct U2U_Errors u2u_errors;

enum message_error_code {
    CRC_HAN_ERR = 1,    // Error on crc. Received does not matched calculated or CRC segment not present.
    SEG_HAN_ERR = 2,    // Some essential segment is missing.
    LEN_HAN_ERR = 4,    // Error on message length: no match on advertised value.
    RFL_HAN_ERR = 8,    // Error on R-flag. First char should have been 'R'.
    RFS_HAN_ERR = 16,    // Error on R-flag. Second char is not {Q, I, S, N}.
    GRS_ROU_ERR = 32,   // Routing error: GEN call with RS flag. Responses should never be GEN.
    GSC_ROU_ERR = 64,   // Routing error: SELF call with self name. Either message spoofed or loop in topology.
    GRN_ROU_ERR = 128   // Routing error: R-flag was negative. Possible peer received corrupted message.
};


char digit_array[128];
char capital_array[128];
char lower_array[128];

uint8_t (*router_function_array[16])(struct Message* message, uint8_t port);
uint8_t (*write_from_uart_func_arr[3])(char* buffer, size_t length);

void topic_init(){
    for (int t=0; t<MAX_TOPIC_RESPONSES; t++){
        //for (size_t c=0; c<MAX_PAYLOAD_SIZE; c++){
            //char ch = topic_default_responses[t][c];
            topic_responses[t] = topic_default_responses[t];
            //if (ch == 0){
            //    break;
            //}
        //}
    }
}

/* Function to change the resulting payload on a response to a topic. 
 * Use parameter 'length' to indicate length of string to be written, 
 * or leave to '0' if NULL term * is present. 
 * If 'length' == 0 AND no NULL term is present, entire max allocated space will potentially be copied.
 * Returns error(-1) if parameter 'topic_nr' exceeds max.
 */
//int u2u_topic_exchange(int topic_nr, const char* buffer, size_t length){
int u2u_topic_exchange(int topic_nr, char* new_response){
    if (topic_nr >= MAX_TOPIC_RESPONSES){
        return -1;
    }
    //size_t size = (length==0) ? length : MAX_PAYLOAD_SIZE;
    //for (size_t c=0; c<size; c++){
    //    char ch = buffer[c];
    topic_responses[topic_nr] = new_response;
    //    if (ch == 0){
    //        break;
    //    }
    //}
    return 0;
}


void parser_init(){
    UPC = U2U_PLATFORM_CHANNELS;
    switch (UPC){
        case 1:
            MAX_PORTS = 2;
            break;
        case 2:
            MAX_PORTS = 2;
            break;
        case 3:
            MAX_PORTS = 3;
            break;
        default:
            break;
    }
    self_name_len = len(SELF_NAME);
    router_function_array[0]  = general_call;
    router_function_array[1]  = self_call;
    router_function_array[2]  = other_call;
    router_function_array[3]  = illegal_ROUTING_call;
    router_function_array[4]  = other_call;
    router_function_array[5]  = drop_call;
    router_function_array[6]  = other_call;
    router_function_array[7]  = illegal_ROUTING_call;
    router_function_array[8]  = illegal_GEN_call;
    router_function_array[9]  = drop_call;
    router_function_array[10] = other_call;
    router_function_array[11] = illegal_ROUTING_call;
    router_function_array[12] = illegal_GEN_call;
    router_function_array[13] = drop_RN_call;
    router_function_array[14] = other_call;
    router_function_array[15] = illegal_ROUTING_call;

    write_from_uart_func_arr[0] = write_from_uart0;
    write_from_uart_func_arr[1] = write_from_uart1;
    write_from_uart_func_arr[2] = write_from_uart2;

    for (int i=0; i<128; i++){
        digit_array[i] = 0;
        capital_array[i] = 0;
        lower_array[i] = 0;
    }
    for (int i='0'; i<='9'; i++){
        digit_array[i] = 1;
    }
    for (int i='A'; i<='Z'; i++){
        capital_array[i] = 1;
    }
    for (int i='a'; i<='z'; i++){
        lower_array[i] = 1;
    }
}


/* Assign messages from message_list wrapping around MAX_MESSAGE_KEEP.
 * This function will be called when:
 * - Parser is initialised.
 * - Message is received, checked, routed and queued. NOT if it is flawed, so it will be overwritten.
 */

struct Message* assign_message(uint8_t port){
    uint8_t message_list_index = get_next_free(&queue[port]);
    struct Message* m = &message_list[port][message_list_index];
    m->index = message_list_index;
    return m;
}


/* To be called only during message_init() and if message is flawed to start anew or when message struct is composed in MAIN. */
void message_clear(struct Message* message){
    message->preamble_end     = 0;
    message->routing_control  = 0;
    message->segment_count    = 0;
    message->crc_received     = 0;
    message->topic_nr         = 0;
    message->hops_int         = 0;
    message->r_value          = 0;
    message->length           = 0;

    message->sender           = NULL;
    message->receiver         = NULL;
    message->rflag            = NULL;
    message->topic            = NULL;
    message->chapter          = NULL;
    message->payload          = NULL;
    message->crc              = NULL;
    message->hops             = NULL;
    message->segment_flags    = NULL;
}

    

/* To be called after each message is completed or message is flawed, to prepare for new message. */ 
void parser_clear(uint8_t port){
    parser[port].state = 0;
    //parser[port].buffer_ix = 0;
    parser[port].p_c[0] = 0;
    parser[port].p_c[1] = 0;
}

bool is_digit(char c){
    return digit_array[(uint8_t) c];
}

bool is_capital(char c){
    return capital_array[(uint8_t) c];
}

bool is_lower(char c){
    return lower_array[(uint8_t) c];
}

bool is_alpha(char c){
    return capital_array[(uint8_t) c] | lower_array[(uint8_t) c];
}



/* character_handler(port, ch)
 * Returns 0 if message is coming in.
 * Returns 1 if message is completely received.
 * Returns 2 if error occured.
 *
 * STATE CASES:
 *     0: Waiting for starting condition.
 *        Upon starting condition start filling buffer with characters (ch).
 *        State completed following start condition.
 *     1: Processing preamble.
 *        Write all characters into buffer.
 *        Keep track of lenght of each segment by converting value to int.
 *        Keep track of segment flags. 
 *        State completed upon completion of preamble segment (Char 'M').
 *     2: Continue writing characters into buffer.
 *        Retain total length of message (ultimate char count).
 *        Validate end condition and report ERROR if this fails.
 *        State completed once char counter equals message length value advertised in message.
 */

uint8_t character_handler(uint8_t port, char ch){   
    parser[port].p_c[0] = parser[port].p_c[1];
    parser[port].p_c[1] = ch;
    uint8_t ret_val = 0;
    switch (parser[port].state){
        case 0:         // Starting condition <:\d>
            if (parser[port].p_c[0] == ':' && is_digit(parser[port].p_c[1])){
                gpio_put(LED_G, 1);
                parser[port].message = assign_message(port);
                parser[port].prm_seg_len_ix = 0;
                parser[port].prm_seg_len_buffer[0] = ch;
                parser[port].prm_seg_len_buffer_count = 1;
                parser[port].message->raw[0] = parser[port].p_c[0];
                parser[port].message->raw[1] = parser[port].p_c[1];
                parser[port].message->error_code = 0;
                parser[port].char_counter = 2;
                parser[port].state = 1;
            }
            break;
        case 1:        // Preamble 
            parser[port].message->raw[parser[port].char_counter++] = ch;
            if (is_digit(parser[port].p_c[1])){     // All digits processed here.
                parser[port].prm_seg_len_buffer[parser[port].prm_seg_len_buffer_count] = ch;
                parser[port].prm_seg_len_buffer_count++;
            }
            if (is_digit(parser[port].p_c[0]) && is_capital(parser[port].p_c[1])){    // Next flag.
                int seg_length = ascii_to_int_i(parser[port].prm_seg_len_buffer, 
                        parser[port].prm_seg_len_buffer_count);
                parser[port].prm_seg_len_array[parser[port].prm_seg_len_ix] = seg_length;
                parser[port].message->preamble_flags[parser[port].prm_seg_len_ix] = ch;
                parser[port].prm_seg_len_ix++;
                parser[port].prm_seg_len_buffer_count = 0;
            }
            if (is_digit(parser[port].p_c[0]) && parser[port].p_c[1] == 'M'){     // End of preamble
                parser[port].message->preamble_end = parser[port].char_counter;
                parser[port].state = 2;
            }
            break;
        case 2:      // Segments
            parser[port].message->raw[parser[port].char_counter++] = ch;
            if (parser[port].char_counter == parser[port].prm_seg_len_array[0]){
                parser[port].message->length = parser[port].char_counter;
                parser[port].message->raw[parser[port].char_counter++] = ch;
                /* Last character must be ':' */
                if (ch == ':'){
                    ret_val = 1;
                } else {
                    /* If last char is not ':' then corruption occured. Either lenght is not as advertised
                     * or parts of it got lost and parser ended up in another message.
                     */
                    parser[port].message->error_code += LEN_HAN_ERR;
                    u2u_errors.len_handler[port]++;
                    u2u_errors.error_count[port]++;
                    ret_val = 2;
                }
            }
            break;

        default:
            break;
    }
    return ret_val;
}


/* Return index for flag. Todo: 'M' must not be used (or the flag needs removing from flag list).*/
int8_t get_segment_index(struct Message* message, char flag){
    int flag_index = -1;
    for (int s=0; s<message->segment_count; s++){
        if (flag==message->preamble_flags[s]){
            flag_index = s;
            break;
        }
    }
    return flag_index;
}

size_t get_segment_length(struct Message* message, char flag){
    int8_t flag_index = get_segment_index(message, flag);
    return message->segment_length[flag_index];
}

/* message_parser(message, port)
 * Setting pointers to raw message string for each segment,
 * Filling message variables with details in parser struct.
 * Calculate crc on received message.
 * Returns 0 if message is parsed, 1 if error has occured.
 */

uint8_t message_parser(struct Message* message, uint8_t port){
    size_t ch_ix = message->preamble_end;
    uint8_t seg_received = parser[port].prm_seg_len_ix - 1;
    uint8_t ret_val = 0;
    message->segment_count = seg_received;
    message->port = port;
    for (int s=0; s<seg_received; s++){
        message->segment_length[s] = parser[port].prm_seg_len_array[s+1];
        message->segments[s] = message->raw + ch_ix;
        ch_ix += parser[port].prm_seg_len_array[s+1];
    }
    int8_t crc_index = get_segment_index(message, 'Y');
    if (crc_index < 0){  // No CRC segment present!
        message->error_code |= SEG_HAN_ERR;
        u2u_errors.seg_crc_missing[port]++;
        u2u_errors.error_count[port]++;
        ret_val = 1;
    }
    uint8_t crc_val_rx = 
        ascii_to_int_i(message->segments[crc_index], get_segment_length(message, 'Y'));
    uint8_t crc_val_cl = get_crc(message->raw, message->length-4);
    message->crc_received = crc_val_rx;
    message->crc_calculated = crc_val_cl;
    int8_t chapter_index = get_segment_index(message, 'C');
    uint8_t chapter_int = 0;
    if (chapter_index >= 0){  // No chapter segement present. Setting it to 0. 
        chapter_int = 
            ascii_to_int_i(message->segments[chapter_index], get_segment_length(message, 'C'));
    }
    message->chapter_int = (uint8_t) chapter_int;
    /* Testing CRC value */
    if (crc_val_rx != crc_val_cl){
        message->error_code += CRC_HAN_ERR;
        u2u_errors.crc_handler[port]++;
        u2u_errors.error_count[port]++;
        ret_val = 1;
    }
    /* Ensuring R-Flag starts with 'R' character */
    int8_t rflag_index = get_segment_index(message, 'F');
    if (rflag_index < 0){
        message->error_code |= SEG_HAN_ERR;
        u2u_errors.seg_rflag_missing[port]++;
        u2u_errors.error_count[port]++;
        ret_val = 1;
    }else if (message->segments[(int8_t) rflag_index][0] != 'R'){
        message->error_code += RFL_HAN_ERR;
        u2u_errors.seg_rflag_fault[port]++;
        u2u_errors.error_count[port]++;
        ret_val = 1;
    }
    int8_t hops_index = get_segment_index(message, 'H');
    if (hops_index >= 0){
        message->hops_int = ascii_to_int_i(message->segments[get_segment_index(message, 'H')], 
                get_segment_length(message, 'H'));
    }
    return ret_val;
}

/* Write 'segment' into 'buffer' at 'offset' for 'length' characters. 
 * To allow for c type const char arrays, normal NULL termination is maintained.
 * Todo: when payload is added, NULL must be ignored and the entire length must be added.
 * */

size_t add_segment(char* buffer, const char* segment, size_t offset, size_t length){
    size_t c = 0;
    for (c=0; c<length; c++){
        if (segment[c] == 0){ break; }
        buffer[c+offset] = segment[c];
    }
    return c;
}

/* Fill in the last segment with the crc value */
size_t add_crc_segment(char* buffer, size_t length){
    uint8_t crc_val = get_crc(buffer, length);
    char temp_buffer[5];
    size_t temp_size = int_to_ascii(temp_buffer, crc_val, 3);
    length = add_segment(buffer, temp_buffer, length, temp_size);
    return length;
}

/* Four char spaces are reserved after the first ':' symbol to advertise the message's length. */
void fill_length_preamble(char* buffer, size_t length){
    char message_length_buffer[5];
    int_to_ascii(message_length_buffer, length + 3 + 1, 4);
    for (int c=0; c<4; c++){
        buffer[c+1] = message_length_buffer[c];
    }
    return;
}

/* Takes port as parameter only to select correct allocated payload_buffer array */
size_t response_composer(struct Message* message, uint8_t port, char* buffer){
    char tmp_buffer[MAX_PR_MESSAGE_SIZE];
    size_t payload_length = 0;
    int8_t topic_index = get_segment_index(message, 'T');
    /* Handling response payload segment. */
    if (topic_index >= 0){
        message->topic_nr = topic_to_int_hash(
                message->segments[topic_index],
                message->segment_length[topic_index]);

        if (message->topic_nr > MAX_TOPIC_RESPONSES){
            message->topic_nr = MAX_TOPIC_RESPONSES;
        }
        payload_length += add_segment(payload_buffer[port], topic_responses[message->topic_nr], 0, MAX_PAYLOAD_SIZE);
    } else{
        char* no_topic_warning = "TOPIC segment not present.";
        message->topic_nr = -1;
        payload_length += add_segment(payload_buffer[port], topic_responses[message->topic_nr],
                0, MAX_PAYLOAD_SIZE);
        payload_length += add_segment(payload_buffer[port], no_topic_warning, payload_length, len(no_topic_warning));
    }

    /* Calculating hop segment. */
    int hop = 0;
    if (RESPONSE_HOP_ADD_INBOUND == 1){
        //hop = ascii_to_int_i(
        //        message->segments[get_segment_index(message, 'H')], 
        //        get_segment_length(message, 'H'));
        hop = message->hops_int;
        if (++hop >= 1000){
            hop = 999;
        }

    }
    char hop_buffer[MAX_HOPS_SIZE];
    size_t hop_length = int_to_ascii(hop_buffer, hop, 0);

    size_t buffer_index = 0;
    buffer[buffer_index++] = ':';
    buffer[buffer_index++] = '0';
    buffer[buffer_index++] = '0';
    buffer[buffer_index++] = '0';
    buffer[buffer_index++] = '0';
    for (int s=0; s<message->segment_count; s++){
        char flag = message->preamble_flags[s];
        buffer[buffer_index++] = flag;
        size_t pr_len = 0;
        switch (flag){
            case 'S':
                pr_len = int_to_ascii(tmp_buffer, self_name_len, 0);
                buffer_index += add_segment(buffer, tmp_buffer, buffer_index, pr_len);
                break;
            case 'R': // Inbound message SENDER becomes response message RECEIVER.
                pr_len = int_to_ascii(tmp_buffer, get_segment_length(message, 'S'), 0);
                buffer_index += add_segment(buffer, tmp_buffer, buffer_index, pr_len);
                break;
            case 'P':
                pr_len = int_to_ascii(tmp_buffer, payload_length, 0);
                buffer_index += add_segment(buffer, tmp_buffer, buffer_index, pr_len);
                break;
            case 'H':
                if (RESPONSE_HOP_ADD_INBOUND == 1){
                    char hop_pream_size_buf[3];
                    pr_len = int_to_ascii(hop_pream_size_buf, hop_length, 0);
                    buffer_index += add_segment(buffer, hop_pream_size_buf, buffer_index, pr_len);
                }else{
                    buffer[buffer_index++] = '1';
                }
                break;
            default:
                pr_len = int_to_ascii(tmp_buffer, get_segment_length(message, flag), 0);
                buffer_index += add_segment(buffer, tmp_buffer, buffer_index, pr_len);
                break;
        }
    }

    buffer[buffer_index++] = 'M';

    /* Building message part. Loop and case switch used to maintain same order as inbound message
     * based on the preamble flags order.  * */
    for (int s=0; s<message->segment_count; s++){
        char flag = message->preamble_flags[s];
        switch (flag){
            case 'S':
                buffer_index += add_segment(
                        buffer, 
                        SELF_NAME, 
                        buffer_index, 
                        self_name_len);
                break;
            case 'R':    // Inbound message SENDER becomes response message RECEIVER.
                buffer_index += add_segment(
                        buffer, 
                        message->segments[get_segment_index(message, 'S')], 
                        buffer_index, 
                        get_segment_length(message, 'S'));
                break;
            case 'F':
                buffer_index += add_segment(
                        buffer, 
                        "RS",
                        buffer_index, 
                        2);
                break;
            case 'T':
                buffer_index += add_segment(
                        buffer, 
                        message->segments[get_segment_index(message, flag)], 
                        buffer_index, 
                        get_segment_length(message, flag));
                break;
            case 'C':
                buffer_index += add_segment(
                        buffer, 
                        message->segments[get_segment_index(message, flag)], 
                        buffer_index, 
                        get_segment_length(message, flag));
                break;
            case 'P':
                buffer_index += add_segment(
                        buffer, 
                        payload_buffer[port], 
                        buffer_index, 
                        payload_length);
                break;
            case 'H':
                if (RESPONSE_HOP_ADD_INBOUND == 1){
                    buffer_index += add_segment(
                            buffer, 
                            hop_buffer, 
                            buffer_index, 
                            hop_length);
                }else{
                    buffer[buffer_index++] = '0';
                }
                break;
            case 'Y':
                break;
            default:
                buffer_index += add_segment(
                        buffer, 
                        message->segments[get_segment_index(message, flag)], 
                        buffer_index, 
                        get_segment_length(message, flag));
                break;
        }
    }
    fill_length_preamble(buffer, buffer_index);
    buffer_index += add_crc_segment(buffer, buffer_index);
    buffer[buffer_index++] = ':';
    buffer[buffer_index] = 0;       // Null termination only for debugging processes. 
                                    // Protocol does not depend on it

    return buffer_index;
}


size_t forward_composer(struct Message* message, char* buffer){
    /* Calculating values related to 'Hop' segment in advance for later. */
    char tmp_buffer[4];
    char hop_buffer[4];
    //int hop = ascii_to_int_i(
    //        message->segments[get_segment_index(message, 'H')], 
    //        get_segment_length(message, 'H'));
    int hop = message->hops_int;
    if (++hop >= 1000){
        hop = 999;
    }

    /* Filling in preamble part. */
    size_t hop_length = int_to_ascii(hop_buffer, hop, 0);
    size_t buffer_index = 0;
    buffer[buffer_index++] = ':';
    buffer[buffer_index++] = '0';
    buffer[buffer_index++] = '0';
    buffer[buffer_index++] = '0';
    buffer[buffer_index++] = '0';
    for (int s=0; s<message->segment_count; s++){
        char flag = message->preamble_flags[s];
        buffer[buffer_index++] = flag;
        if (flag == 'H'){
            char hop_pream_size_buf[3];
            size_t hop_pream_len = int_to_ascii(hop_pream_size_buf, hop_length, 0);
            buffer_index += add_segment(buffer, hop_pream_size_buf, buffer_index, hop_pream_len);
        }else{
            size_t length = int_to_ascii(tmp_buffer, get_segment_length(message, flag), 0);
            buffer_index += add_segment(buffer, tmp_buffer, buffer_index, length);
        }
    }
    buffer[buffer_index++] = 'M';
    
    /* Composing message proper. */
    for (int s=0; s<message->segment_count; s++){
        char flag = message->preamble_flags[s];
        if (flag == 'H'){
            buffer_index += add_segment( buffer, hop_buffer, buffer_index, hop_length);
        }else if (flag == 'Y') {
        }else{
            buffer_index += add_segment(
                    buffer, 
                    message->segments[get_segment_index(message, flag)], 
                    buffer_index, 
                    get_segment_length(message, flag));
        }
    }
        

    fill_length_preamble(buffer, buffer_index);
    buffer_index += add_crc_segment(buffer, buffer_index);
    buffer[buffer_index++] = ':';
    buffer[buffer_index] = '\0';            // Null termination only for debugging processes. 
                                          // Protocol does not depend on it
    return buffer_index;
}


/* Todo: Later implementations will reduce this number to restrict 
 * forwarding to ports where it is known the node does not exists
 * based on some node table.  */
uint8_t get_max_ports(uint8_t* ports, uint8_t port){
    // Not ok for 3 ports.
    ports[0] = (port + 1) % MAX_PORTS;    // To be replaced with PORT_COUNT
    ports[1] = (port + 2) % MAX_PORTS;
    return MAX_PORTS - 1;   
}


/* The parameter 'port' defines message origin port. */
void write_forward_message(char* buffer, size_t length, uint8_t port){
    uint8_t ports[2];
    uint8_t forward_port_count = get_max_ports(ports, port);
    for (uint8_t p=0; p<forward_port_count; p++){
        (*write_from_uart_func_arr[ports[p]])(buffer, length);
    }
}

void write_response_message(char* buffer, size_t length, uint8_t port){
    (*write_from_uart_func_arr[port])(buffer, length);
}

uint8_t general_call(struct Message* message, uint8_t port){
    //char buffer[MAX_MESSAGE_SIZE];
    size_t length = response_composer(message, port, compose_buffer[port]);
    write_response_message(compose_buffer[port], length, port);
    length = forward_composer(message, compose_buffer[port]);
    write_forward_message(compose_buffer[port], length, port);
    return 0;
}

uint8_t self_call(struct Message* message, uint8_t port){
    //char buffer[MAX_MESSAGE_SIZE];
    size_t length = response_composer(message, port, compose_buffer[port]);
    write_response_message(compose_buffer[port], length, port);
    return 0;
}

uint8_t other_call(struct Message* message, uint8_t port){
    //char buffer[MAX_MESSAGE_SIZE];
    size_t length = forward_composer(message, compose_buffer[port]);
    write_forward_message(compose_buffer[port], length, port);
    return 0;
}

uint8_t drop_RN_call(struct Message* message, uint8_t port){
    // do nothing
    u2u_errors.error_count[port]++;
    u2u_errors.srn_router[port]++;
    (void) message;
    return 0;
}

uint8_t drop_call(struct Message* message, uint8_t port){
    // do nothing
    (void) message;
    (void) port;
    return 0;
}

uint8_t illegal_call(struct Message* message, uint8_t port){
    // error code
    (void) message;
    (void) port;
    return 0;
}

uint8_t illegal_GEN_call(struct Message* message, uint8_t port){
    // error code
    u2u_errors.error_count[port]++;
    u2u_errors.grs_router[port]++;
    (void) message;
    return 0;
}

uint8_t illegal_ROUTING_call(struct Message* message, uint8_t port){
    // error code
    u2u_errors.error_count[port]++;
    u2u_errors.rcf_router[port]++;
    (void) message;
    return 0;
}

/* R-value |  S-value
 * --------|---------
 *  RQ  0  |  GEN  0
 *  RI  1  |  SELF 1
 *  RS  2  |  OTH  2
 *  RN  3  |
 *
 * */

uint8_t message_router(struct Message* message, uint8_t port){
    gpio_put(LED_Y, 1);
    char flag = message->segments[get_segment_index(message, 'F')][1];
    switch (flag){
        case 'Q':
            message->r_value = 0;
            break;
        case 'I':
            message->r_value = 1;
            break;
        case 'S':
            message->r_value = 2;
            break;
        case 'N':
            message->r_value = 3;
            break;
        default:
            message->r_value = 4;
            message->error_code += RFS_HAN_ERR;
            u2u_errors.seg_rflag_unknown[port]++;
            u2u_errors.error_count[port]++;
            break;
    }

    if (cmp_i(message->segments[get_segment_index(message, 'R')], 
                GENERAL_NAME, get_segment_length(message, 'R'), 3)){
        message->s_value = 0;
    }else if 
        (cmp_i(message->segments[get_segment_index(message, 'R')], SELF_NAME, 
               get_segment_length(message, 'R'), self_name_len)){
        message->s_value = 1;
    }else{
        message->s_value = 2;
    }
    message->routing_control = 0;
    message->routing_control |= message->r_value << 2;
    message->routing_control |= message->s_value;

    uint8_t r = (*router_function_array[message->routing_control])(message, port);
    gpio_put(LED_Y, 0);
    return r;
}

/* Called for each character. State machine will keep track of progress.
 * Will return 0 for success, >0 otherwise.
 *
 */
uint8_t character_processor(uint8_t port, char ch){
    uint8_t ret_val = 0;
    uint8_t parser_val = 0;
    switch (character_handler(port,  ch)) {
        case 0:
            break;
        case 1: {
            parser_val += message_parser(parser[port].message, port);
            if (parser_val != 0){
                parser_clear(port);
                return 1;
            }
            parser_val += message_router(parser[port].message, port);
            if (parser_val != 0){
                parser_clear(port);
                return 2;
            }
            parser_clear(port);
            in_queue(&queue[port], parser[port].message->index);

            gpio_put(LED_G, 0);
            break;
            }
        case 2:
            ret_val = 2;
            break;
        default:
            break;
    }
    return ret_val;
}

uint8_t uart0_character_processor(char ch){
    uint8_t port = 0;
    return character_processor(port, ch);
}

uint8_t uart1_character_processor(char ch){
    uint8_t port = 1;
    return character_processor(port, ch);
}

uint8_t u2u_write_character(uint8_t port, char ch){
    int write_result = write_to_circular_queue(&circular_queue[port], ch);
    if (write_result != 0){
        u2u_errors.drb_uart[port]++;
        u2u_errors.error_count[port]++;
        return 1;
    }
    return 0;
}


void messages_init(){
    for (int p=0; p<PORT_COUNT; p++){
        for (int i=0; i<MAX_MESSAGE_KEEP; i++){
            message_clear(&message_list[p][i]);
            message_list[p][i].index = i;                    // Redundant? Assign index to message.
            message_list[p][i].port = p;                        // Redundant? Assign port to message.
        }
    }
}

void clear_u2u_errors(uint8_t port){
    u2u_errors.error_count[port]        = 0;
    u2u_errors.drb_uart[port]           = 0;
    u2u_errors.crc_handler[port]        = 0; 
    u2u_errors.seg_handler[port]        = 0;
    u2u_errors.seg_crc_missing[port]    = 0;
    u2u_errors.seg_rflag_missing[port]  = 0;
    u2u_errors.seg_rflag_fault[port]    = 0;
    u2u_errors.seg_rflag_unknown[port]  = 0;
    u2u_errors.len_handler[port]        = 0;
    u2u_errors.rfl_handler[port]        = 0;
    u2u_errors.rfs_handler[port]        = 0;
    u2u_errors.grs_router[port]         = 0;
    u2u_errors.rcf_router[port]         = 0;
    u2u_errors.srn_router[port]         = 0;
}

void u2u_message_setup(){
    //messages_init();
    topic_init();
    parser_init();
    topic_hash_table_init();
    for (uint8_t p=0; p<PORT_COUNT; p++){
        message_queue_init(&queue[p], p);
        clear_u2u_errors(p);

    }
    u2u_uart_setup();

    

}



void print_usage_message(int argc, char** argv){
    (void) argc;
    printf("Usage: %s -p {0,1,2}\n", argv[0]);
}
    
void copy_segment(char* segment, struct Message* message, char segment_flag){
    int8_t index  = get_segment_index(message, segment_flag);
    if (index >= 0){
        size_t length = get_segment_length(message, segment_flag);
        size_t c = 0;
        for (c=0; c<length; c++){
            segment[c] = message->segments[index][c];
        }
        segment[length] = '\0';
    }else{
        segment[0] = '-';
        segment[1] = '\0';
    }
}

#if 0
/* Function to start parsing chain. It requires MAIN to dedicated a while loop to maintain U2U bus connectivity.
 * To avoid race conditions MAIN cannot specify the port from which to receive messages and will have to
 * keep polling and this function will alternate (round robin style) between the available ports. 
 * MAIN can implement a 'for loop' to run through the ports that are available.
 */
#define BLOCK_SIZE 128
struct Message* get_message(struct Message_Segments* segments){
    int cq_ret = 0;
    for (uint8_t p=0; p<PORT_COUNT; p++){
        char ch;
        for (uint16_t i=0; i<BLOCK_SIZE; i++){
            cq_ret = read_from_circular_queue(&circular_queue[p], &ch);
            if (cq_ret == 0){
                character_processor(p, ch);
            }else{
                break;
            }
        }
    }

    static uint8_t port_rr = 0;
    uint8_t message_index = 255;                                                
    int r = out_queue(&queue[port_rr], &message_index);                            
    //uint8_t message_index = from_queue_to_file(port_rr);
    struct Message* message = NULL;
    if (r >= 0){
        message = &message_list[port_rr][message_index];           
        if (segments != NULL){ 
            /* Existence of segment/flag is checked in copy_segment(). */
            copy_segment(segments->sender,   message, 'S');
            copy_segment(segments->receiver, message, 'R');
            copy_segment(segments->rflag,    message, 'F');
            copy_segment(segments->topic,    message, 'T');
            copy_segment(segments->chapter,  message, 'C');
    }
}
#endif

/* Function to start parsing chain. It requires MAIN to dedicated a while loop to maintain U2U bus connectivity.
 * To avoid race conditions MAIN cannot specify the port from which to receive messages and will have to
 * keep polling and this function will alternate (round robin style) between the available ports. 
 * MAIN can implement a 'for loop' to run through the ports that are available.
 */
struct Message* get_message(struct Message_Segments* segments){
    gpio_put(LED_R, 0);
    if (u2u_errors.error_count[0] > 0 || u2u_errors.error_count[1] > 0) {
        gpio_put(LED_R, 1);
    }
    int cq_ret = 0;
    for (uint8_t p=0; p<PORT_COUNT; p++){
        char ch;
        for (uint16_t i=0; i<BLOCK_SIZE; i++){
            cq_ret = read_from_circular_queue(&circular_queue[p], &ch);
            if (cq_ret == 0){
                character_processor(p, ch);
            }else{
                break;
            }
        }
    }

    static uint8_t port_rr = 0;
    uint8_t message_index = 255;                                                
    int r = out_queue(&queue[port_rr], &message_index);                            
    //uint8_t message_index = from_queue_to_file(port_rr);
    struct Message* message = NULL;
    if (r >= 0){
        message = &message_list[port_rr][message_index];           
        if (segments != NULL){
            /* Existence of segment/flag is checked in copy_segment(). */
            copy_segment(segments->sender,   message, 'S');
            copy_segment(segments->receiver, message, 'R');
            copy_segment(segments->rflag,    message, 'F');
            copy_segment(segments->topic,    message, 'T');
            copy_segment(segments->chapter,  message, 'C');
            copy_segment(segments->payload,  message, 'P');
            copy_segment(segments->hops,     message, 'H');
            copy_segment(segments->crc,      message, 'Y');
        }

        int8_t topic_index = get_segment_index(message, 'T');
        /* Handling response payload segment. */
        if (topic_index >= 0){
            message->topic_nr = topic_to_int_hash(
                    message->segments[topic_index],
                    message->segment_length[topic_index]);

        }
    }
    //port_rr = (port_rr + 1) % PORT_COUNT;
    if (++port_rr >= PORT_COUNT){
        port_rr = 0;
    }
    return message;
}

//Port 1. Error count: 1. Routing failure: GEN call receiv1.
size_t copy_error_log(char* buffer, const char* label, uint32_t error_count, size_t offset, size_t length){
    size_t c;
    if (length == 0){
        length = 64;
    }
    for (c=0; c<length; c++){
        if (label[c] == 0){break; }
        buffer[c+offset] = label[c];
    }
    offset += c;
    buffer[offset++] = '(';
    char err_count_buffer[16];
    size_t err_count_len = int_to_ascii(err_count_buffer, error_count, 0);

    for (c=0; c<err_count_len; c++){
        buffer[c+offset] = err_count_buffer[c];
    }
    offset += c;
    buffer[offset++] = ')';
    buffer[offset++] = '.';
    buffer[offset++] = ' ';
    return offset;
}


size_t get_u2u_error_log(uint8_t port, char* buffer){
    size_t index = 0;
    if (u2u_errors.error_count[port] > 0){
        index = copy_error_log(buffer, "Port ", (uint32_t) port, index, 0);
        index = copy_error_log(buffer, "Error count: ", 
                u2u_errors.error_count[port], index, 0);
        if (u2u_errors.drb_uart[port] > 0){
            index = copy_error_log(buffer, "Dropped bytes: ", 
                    u2u_errors.drb_uart[port], index, 0);
        }
        if (u2u_errors.crc_handler[port] > 0){
            index = copy_error_log(buffer, "CRC handler: ", 
                    u2u_errors.crc_handler[port], index, 0);
        }
        if (u2u_errors.seg_handler[port] > 0){
            index = copy_error_log(buffer, "Segments missing: ", 
                    u2u_errors.seg_handler[port], index, 0);
        }
        if (u2u_errors.seg_crc_missing[port] > 0){
            index = copy_error_log(buffer, "CRC segment missing: ", 
                    u2u_errors.seg_crc_missing[port], index, 0);
        }
        if (u2u_errors.seg_rflag_missing[port] > 0){
            index = copy_error_log(buffer, "Rflag segment missing: ", 
                    u2u_errors.seg_rflag_missing[port], index, 0);
        }
        if (u2u_errors.seg_rflag_fault[port] > 0){
            index = copy_error_log(buffer, "Rflag segment fault: ", 
                    u2u_errors.seg_rflag_fault[port], index, 0);
        }
        if (u2u_errors.seg_rflag_unknown[port] > 0){
            index = copy_error_log(buffer, "Rflag segment unknown: ", 
                    u2u_errors.seg_rflag_unknown[port], index, 0);
        }
        if (u2u_errors.len_handler[port] > 0){
            index = copy_error_log(buffer, "Message length error: ", 
                    u2u_errors.len_handler[port], index, 0);
        }
        if (u2u_errors.rfl_handler[port] > 0){
         //   index = copy_error_log(buffer, "Not applicable, error not implemented yet.", 
         //           u2u_errors.rfl_handler[port], index, 0);
        }
        if (u2u_errors.rfs_handler[port] > 0){
         //   index = copy_error_log(buffer, "Not applicable, error not implemented yet.", 
         //           u2u_errors.rfs_handler[port], index, 0);
        }
        if (u2u_errors.grs_router[port] > 0){
            index = copy_error_log(buffer, "Routing failure: GEN and 'RS' or 'RN'.", 
                    u2u_errors.grs_router[port], index, 0);
        }
        if (u2u_errors.rcf_router[port] > 0){
            index = copy_error_log(buffer, "Routing failure: routing control.", 
                    u2u_errors.rcf_router[port], index, 0);
        }
        if (u2u_errors.srn_router[port] > 0){
            index = copy_error_log(buffer, "Message error: Self and RN.", 
                    u2u_errors.srn_router[port], index, 0);
        }
        buffer[index++] = '\0';
    }
    return index;
}


uint32_t get_u2u_errors(uint8_t port){
    return u2u_errors.error_count[port];
}

/* Example on setting a message with minimum segments as follows below. The order in which the
 * segments are placed depends on the struct member 'segment_flags'. 
 * Alternative to member 'topic' (which expects a char pointer) 'topic_int' can be used with an integer.
 * Custom topics are not supported. 
 * */
int format_message(struct Message* message){
    char len_buffer[5];
    size_t temp_size = 0;
    message->length = 0;
    message->raw[message->length++] = ':';
    message->raw[message->length++] = '0';
    message->raw[message->length++] = '0';
    message->raw[message->length++] = '0';
    message->raw[message->length++] = '0';
    bool custom_sender  = false;
    bool custom_hops    = false;
    // Todo: bool custom_chapter = false; // using a static variable to increment each formated message.
    size_t preamble_length = 0;

    //char temp_flags[MAX_SEGMENT_COUNT];
    for (size_t c=0; c<MAX_SEGMENT_COUNT; c++){
        char flag = message->segment_flags[c];
        if (flag == '\0'){
            break;
        }
        if (flag == 'S'){
            custom_sender = true;
        }
        if (flag == 'H'){
            custom_hops = true;
        }
        preamble_length++;
    }
    if (custom_sender == false){
        message->raw[message->length++] = 'S';
        temp_size = int_to_ascii(len_buffer, self_name_len, 0);
        message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
    }
    char chapter[MAX_CHAPTER_SIZE];  // In case chapter_int is used, this is stored here.

    for (size_t c=0; c<preamble_length; c++){
        char flag = message->segment_flags[c];
        switch (flag){
            case 'S':
                message->raw[message->length++] = flag;
                temp_size = int_to_ascii(len_buffer, len(message->sender), 0);
                message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
                break;
            case 'R':
                message->raw[message->length++] = flag;
                temp_size = int_to_ascii(len_buffer, len(message->receiver), 0);
                message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
                break;
            case 'F':
                message->raw[message->length++] = flag;
                temp_size = int_to_ascii(len_buffer, len(message->rflag), 0);
                message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
                break;
            case 'T':
                message->raw[message->length++] = flag;
                temp_size = int_to_ascii(len_buffer, len(message->topic), 0);
                message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
                break;
            case 'C':
                message->raw[message->length++] = flag;
                if (message->chapter == NULL){
                    int_to_ascii(chapter, message->chapter_int, 0);
                    temp_size = int_to_ascii(len_buffer, len(chapter), 0);
                    message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
                }else{
                    temp_size = int_to_ascii(len_buffer, len(message->chapter), 0);
                    message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
                }
                break;
            case 'P':
                message->raw[message->length++] = flag;
                temp_size = int_to_ascii(len_buffer, len(message->payload), 0);
                message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
                break;
            case 'H':
                message->raw[message->length++] = flag;
                temp_size = int_to_ascii(len_buffer, len(message->hops), 0);
                message->length += add_segment(message->raw, len_buffer, message->length, temp_size);
                break;
            default:
                break;
        }

    }
    if (custom_hops == false){
        message->raw[message->length++] = 'H';
        message->raw[message->length++] = '1';
    }
    message->raw[message->length++] = 'Y';
    message->raw[message->length++] = '3';
    message->raw[message->length++] = 'M';

    if (custom_sender == false){
        message->length += add_segment(message->raw, SELF_NAME, message->length, self_name_len);
    }

    for (size_t c=0; c<preamble_length; c++){
        char flag = message->segment_flags[c];
        switch (flag){
            case 'S':
                message->length += add_segment(
                        message->raw, 
                        message->sender, 
                        message->length, 
                        len(message->sender));
                break;
            case 'R':
                message->length += add_segment(
                        message->raw, 
                        message->receiver, 
                        message->length, 
                        len(message->receiver));
                break;
            case 'F':
                message->length += add_segment(
                        message->raw, 
                        message->rflag, 
                        message->length, 
                        len(message->rflag));
                break;
            case 'T':
                message->length += add_segment(
                        message->raw, 
                        message->topic, 
                        message->length, 
                        len(message->topic));
                break;
            case 'C':
                if (message->chapter == NULL){
                    message->length += add_segment(
                            message->raw, 
                            chapter, 
                            message->length, 
                            len(chapter));
                }else{
                    message->length += add_segment(
                            message->raw, 
                            message->chapter, 
                            message->length, 
                            len(message->chapter));
                }
                break;
            case 'P':
                message->length += add_segment(
                        message->raw, 
                        message->payload, 
                        message->length, 
                        len(message->payload));
                break;
            case 'H':
                message->length += add_segment(
                        message->raw, 
                        message->hops, 
                        message->length, 
                        len(message->hops));
                break;
            case 'Y':
                break;
            default:
                break;
        }
    }
    if (custom_hops == false){
        message->raw[message->length++] = '0';
    }

    fill_length_preamble(message->raw, message->length);
    message->length += add_crc_segment(message->raw, message->length);
    message->raw[message->length++] = ':';
    message->raw[message->length] = '\0';
    return 0;
}


int u2u_send_message_uart0(char* buffer, size_t length){
    write_from_uart0(buffer, length);
    return 0;
}
int u2u_send_message_uart1(char* buffer, size_t length){
    write_from_uart1(buffer, length);
    return 0;
}
int u2u_send_message_uart2(char* buffer, size_t length){
    write_from_uart2(buffer, length);
    return 0;
}

int u2u_send_message(struct Message* message, uint8_t port){
    if (message->length == 0){
        return -1;
    }
    if (message->raw[0] != ':' || message->raw[message->length-1] != ':'){
        return -2;
    }
    switch (port){
        case 0:
            u2u_send_message_uart0(message->raw, message->length);
            break;
        case 1:
            u2u_send_message_uart1(message->raw, message->length);
            break;
        case 2:
            u2u_send_message_uart2(message->raw, message->length);
            break;
        default:
            u2u_send_message_uart0(message->raw, message->length);
            u2u_send_message_uart1(message->raw, message->length);
            u2u_send_message_uart2(message->raw, message->length);
            break;
    }
    return 0;
}

/* Tricking the system to believe it has different ports. For testing only. */


void u2u_close(){
    u2u_uart_close();
}





    

/* Message are composed from segments sent through the FIFO pipe. 
 * Aside from the pipe command (here 'Smsg') and port the sequence is expected to be like so:
 * '69 {SRFTCPH} {TEST_SNDR} {TEST_RCVR} {RI} {GET_ENCR} {0} {tzns}{309} '
 * Where 69 is the total length (until \n) then follow the segments:
 * segment:        function:
 * {RFTCPH}        PREAMBLE
 * {TEST_SNDR}     SENDER*
 * {TEST_RCVR}     RECEIVER 
 * {RI}            R-FLAG 
 * {GET_ENCR}      TOPIC 
 * {0}             CHAPTER 
 * {tzns}          PAYLOAD
 * {309}           HOPS*
 * (*)Both SENDER and HOPS segments are optional and should be used only for testing. The composer will fill
 * in these values if absent, for sender: directed by client profile and hops: '0'. In that case the 
 * preamble segment shall contain these flags: {RFTCP}.
 * Todo: Now, '{}' brackets are used to seperate segments which means they cannot be used within the payload
 * and this should be changed such that only the advertised length should be used to determine the payload.
 */

// Entry from send_message_from_pipe()
int pipe_message_composer(uint8_t port, char* segments){
    char temp_buffer[5];
    size_t offset = 0;
    size_t char_counter_per_segment = 0;
    size_t c;

    // Get length of pipe message.
    for (c=offset; c<5; c++){
        if (is_digit(segments[c])){
            temp_buffer[char_counter_per_segment++] = segments[c];
        }else if (segments[c] == '{'){
            offset = c;
            break;
        }
    }
    temp_buffer[char_counter_per_segment] = '\0';
    size_t pipe_segments_length = ascii_to_int_i(temp_buffer, char_counter_per_segment);

    

    // Parse pipe message segments.
    char* segment_pointers[8];
    size_t segment_lengths[8];
    size_t segment_counter = 0;
    char_counter_per_segment = 0;
    for (size_t c=offset; c<pipe_segments_length+offset; c++){
        if (segments[c] == '{'){
            char_counter_per_segment = 0;
            segment_pointers[segment_counter] = segments + c + 1;
        } else if (segments[c] == '}'){
            segment_lengths[segment_counter] = char_counter_per_segment;
            segment_counter++;
        }else{
            char_counter_per_segment++;
        }
    }
    for (size_t i=0; i<segment_counter-1; i++){
    }

    // Build raw message.
    char pmc_buffer[MAX_MESSAGE_SIZE];
    size_t buffer_index = 0;
    pmc_buffer[buffer_index++] = ':';
    pmc_buffer[buffer_index++] = '0';
    pmc_buffer[buffer_index++] = '0';
    pmc_buffer[buffer_index++] = '0';
    pmc_buffer[buffer_index++] = '0';

    // Compose preamble.
    bool custom_sender = false;
    bool custom_hops = false;
    size_t temp_size = 0;
    // Run through flags segment to check if SENDER or HOPS are included.
    for (size_t s=0; s<segment_counter-1; s++){
        char flag = segment_pointers[0][s];
        if (flag == 'S'){
            custom_sender = true;
        }
        if (flag == 'H'){
            custom_hops = true;
        }
    }

    if (custom_sender == false){
        pmc_buffer[buffer_index++] = 'S';
        temp_size = int_to_ascii(temp_buffer, self_name_len, 0);
        buffer_index += add_segment(pmc_buffer, temp_buffer, buffer_index, temp_size);
    }

    for (size_t s=0; s<segment_counter-1; s++){
        char flag = segment_pointers[0][s];
        pmc_buffer[buffer_index++] = flag;
        temp_size = int_to_ascii(temp_buffer, segment_lengths[s+1], 0);
        buffer_index += add_segment(pmc_buffer, temp_buffer, buffer_index, temp_size);
    }
    if (custom_hops == false){
        pmc_buffer[buffer_index++] = 'H';
        pmc_buffer[buffer_index++] = '1';
    }
    pmc_buffer[buffer_index++] = 'Y';
    pmc_buffer[buffer_index++] = '3';
    pmc_buffer[buffer_index++] = 'M';

    // Compose message.
    if (custom_sender == false){
        buffer_index += add_segment(pmc_buffer, SELF_NAME, buffer_index, self_name_len);
    }
    for (size_t s=1; s<segment_counter; s++){
        buffer_index += add_segment(pmc_buffer, segment_pointers[s], buffer_index, segment_lengths[s]);
    }
    if (custom_hops == false){
        pmc_buffer[buffer_index++] = '0';    // HOPS set to 0
    }
    fill_length_preamble(pmc_buffer, buffer_index);

    buffer_index += add_crc_segment(pmc_buffer, buffer_index);
    pmc_buffer[buffer_index++] = ':';
    pmc_buffer[buffer_index] = '\0';

    switch (port){
        case 0:
            u2u_send_message_uart0(pmc_buffer, buffer_index);
            break;
        case 1:
            u2u_send_message_uart1(pmc_buffer, buffer_index);
            break;
        case 2:
            u2u_send_message_uart2(pmc_buffer, buffer_index);
            break;
        default:
            u2u_send_message_uart0(pmc_buffer, buffer_index);
            u2u_send_message_uart1(pmc_buffer, buffer_index);
            u2u_send_message_uart2(pmc_buffer, buffer_index);
            break;
    }
    return 0;
}

