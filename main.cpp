//main.c
//#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"

#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "sgp40_i2c.h"

extern "C" {
#include "sensirion_gas_index_algorithm.h"
}

extern "C" {
#include "u2u.h"
}

#define LED_ONBOARD 25

#define RGB_G_PIN   7
#define RGB_B_PIN   8
#define RGB_R_PIN   9
#define RG0_R_PIN   3
#define RG0_G_PIN   6
#define RG1_R_PIN   1
#define RG1_G_PIN   2

#if 0
#define LED_R 11
#define LED_Y 14
#define LED_G 15
#endif

#define RS          26
#define E           22
#define DB4         21
#define DB5         20
#define DB6         19
#define DB7         18

#define output_nr   14


#define SCL_PIN     16
#define SDA_PIN     17

#define RS_COMMAND  0
#define RS_DATA     1

#define LCD_FS      0x02 //Function set
#define LCD_DCO     0x28 //Display control
#define LCD_CD      0x02 //
#define LCD_DCL     0x01 //Display clear
#define LCD_MH      0x80 //Move home

#define PMSA_ADDR   0x12
#define SGP_ADDR    0x59
#define AM_ADDR     0x5C

#define PORT_0      0
#define PORT_1      1


/* Time keeping related variables: */
volatile bool timer_synched = 0;
static bool tog_led = 0;
volatile static uint8_t  ticks = 0;
volatile static uint32_t secs_to_mn = 0;
volatile static uint16_t count_to_sync = 0;
bool timer_callback(repeating_timer_t *rt);


/* LED related variables: */
uint16_t LED_table[501][3];
const uint8_t outputpins[] = {
    LED_ONBOARD, 
    RG0_R_PIN, 
    RG0_G_PIN, 
    RG1_R_PIN, 
    RG1_G_PIN, 
    E, 
    RS, 
    DB4, 
    DB5, 
    DB6, 
    DB7,
    LED_R,
    LED_Y,
    LED_G};


/* 16*4 DISPLAY related variables: */
#define MAX_CUSTOM_DISPLAY_TEXT 256
#define SCROLL_DELAY 2
const uint8_t lcd_datapins[] = {DB4, DB5, DB6, DB7};
char lcd_request_data[2][MAX_CUSTOM_DISPLAY_TEXT];
volatile bool lrd_active_buffer = 0;
volatile uint8_t lcd_data_size;
const uint8_t lcd_display_layout[] = {0x00, 0x40, 0x10, 0x50};

/* Sensor related variables: */
bool sensor_run_flag = 0;
bool sensor_data_ready_flag = 0;
float sampling_interval = 1.f;
GasIndexAlgorithmParams voc_params;
bool sensor_ticker(repeating_timer_t *rs);
int32_t  global_voc = 0;

/* U2U communication buffers: */
char response_buffer_4[127];
char error_buffer[1024];

#if 0
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
    return false;
}
#endif


/* Require buffer of size 8+1 for NULL to hold timeformat. Format var time_ticks into HH:MM:SS. */
uint8_t get_time_stamp(char* buffer, uint32_t time_ticks){
    char digit[3];
    uint32_t temp;
    uint32_t h = time_ticks / (60*60);
    temp = time_ticks - (h*60*60);
    uint32_t m = temp / 60;
    uint32_t s = temp - (m*60);
    uint8_t index = 0;
    int_to_ascii(digit, h, 2);
    buffer[index++] = digit[0];
    buffer[index++] = digit[1];
    buffer[index++] = ':';
    int_to_ascii(digit, m, 2);
    buffer[index++] = digit[0];
    buffer[index++] = digit[1];
    buffer[index++] = ':';
    int_to_ascii(digit, s, 2);
    buffer[index++] = digit[0];
    buffer[index++] = digit[1];
    buffer[index++] = 0;
    return index;
}

struct SensorData{
    float Temperature;
    float Humidity;
    int32_t Voc;
    int32_t Voc_raw;
    int Particles[15];
    char temp_str[12];
    char hum_str[12];
    char voc_str[12];
    char pm_str[15][15];
    uint8_t Particles_u8[32];
};


struct SensorData sensor_data[2];
volatile bool sd_active_buffer = 0;

void lcd_send(uint8_t *buf, uint8_t len_buf, bool command_or_data){
    //printf("len buf: %d.\n", len_buf);
    uint8_t temp = 0;
    uint8_t d = 0;
    for (uint8_t c=0; c<len_buf; c++){
        temp = buf[c];
        temp = temp >> 4;
        gpio_put(E, 1);
        busy_wait_us(10);
        for (d=0; d<4; d++){
            gpio_put(lcd_datapins[d], ((temp>>d) & 0b00000001));
        }
        gpio_put(RS, command_or_data);
        busy_wait_us(10);
        gpio_put(E, 0);
        busy_wait_us(20);
        
        temp = buf[c]; 
        gpio_put(E, 1);
        busy_wait_us(10);
        for (d=0; d<4; d++){
            gpio_put(lcd_datapins[d], ((temp>>d) & 0b00000001));
        }
        gpio_put(RS, command_or_data);
        busy_wait_us(10);
        gpio_put(E, 0);
        busy_wait_us(20);
        for (d=0; d<4; d++){
            gpio_put(lcd_datapins[d], 0);
        }
    }
}

void lcd_init(){
        //Initialisation commands:
        //-Function set (4b/2L/5x7)
        //-Display control
        //-Display clear
        //-Return home
        uint8_t initialisation_commands[] = {0x03, 0x03, 0x03, 0x02, 0x02, 0x28, 0x0E, 0x01, 0x80};
        //Initialisation phase is sandwiched with delay times that come from 
        //the datasheet. Found that when adding 300 each phase yields more reliable operation.
        const int lcd_initialisation_delay_times[] = {100, 100, 100, 40, 40, 40, 1600, 1600, 0};
        for (uint8_t i=0; i<9; i++){
                lcd_send(&initialisation_commands[i], 1, RS_COMMAND );
                busy_wait_us(300+lcd_initialisation_delay_times[i]);
        }
        //sleep_ms(100);
        busy_wait_us(100000);
        return;
}


/* Parameter buf must be 64 in size */
void lcd_fill_lines(uint8_t *buf){
    for (uint8_t lines=0; lines<4; lines++){
        uint8_t adr = 0b10000000 | lcd_display_layout[lines];
        lcd_send(&adr, 1, RS_COMMAND);
        lcd_send(buf+(16*lines), 16, RS_DATA);
    }
}


#define RED 0
#define GREEN 1
#define BLUE 2

void led_table_setup(){
    /*RGB LED will indicate air quality by colour. 
     * Blue: clear    (0   - 150), 
     * Green: mild    (125 - 300),
     * Red: polluted. (300 - 501). */
    uint16_t i;
    const float M = 65535.0;
    float mr = 0.0;

    for (i=0; i<500; i++){
        LED_table[i][0] = 0;
        LED_table[i][1] = 0;
        LED_table[i][2] = 0;
    }
    for (i=0;   i<150; i++){ LED_table[i][BLUE]  = (uint16_t) M; }
    for (i=300; i<501; i++){ LED_table[i][RED]   = (uint16_t) M; }
    for (i=125; i<300; i++){ LED_table[i][GREEN] = (uint16_t) M; }
    for (i=0; i<100; i++){
        mr = ((float) i+1)/100;
        mr = mr*mr;
        LED_table[250+i][RED]   = (uint16_t)(     (M * mr));
        LED_table[125+i][GREEN] = (uint16_t)(     (M * mr));
        LED_table[300+i][GREEN] = (uint16_t)( M - (M * mr));
        LED_table[150+i][BLUE]  = (uint16_t)( M - (M * mr));
    }
}

void io_setup(){
    uint16_t slice_num;
    for (int c=0; c<output_nr; c++){
        gpio_init(outputpins[c]);
        gpio_set_dir(outputpins[c], GPIO_OUT);
        gpio_put(outputpins[c], 0);
    }

    gpio_put(LED_R, 1);
    gpio_put(LED_Y, 1);
    gpio_put(LED_G, 1);
    gpio_set_function(RGB_R_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RGB_G_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RGB_B_PIN, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    slice_num = pwm_gpio_to_slice_num(RGB_G_PIN);
    pwm_init(slice_num, &config, true);
    slice_num = pwm_gpio_to_slice_num(RGB_R_PIN);
    pwm_init(slice_num, &config, true);
    slice_num = pwm_gpio_to_slice_num(RGB_B_PIN);
    pwm_init(slice_num, &config, true);
    
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); // Pull ups are 4K7 added externally.
    gpio_pull_up(SCL_PIN);
}

static uint16_t _calc_crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    while(len--) {
        crc ^= (uint16_t) *buf++;
        for (unsigned i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;      
            }
        }
    }
    return crc;
}

static uint16_t _combine_bytes(uint8_t msb, uint8_t lsb) {
    return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

int hum_conv(float h){
    return (int) h*(65535.0/100.0);
}

int temp_conv(float t){
    return (int) (t+45.0)*(65535.0/175.0);
}

uint16_t sgp40(int32_t* voc, float t, float p){
    uint16_t DEFAULT_COMPENSATION_RH = 0x8000;  // in ticks as defined by SGP40
    uint16_t DEFAULT_COMPENSATION_T = 0x6666;   // in ticks as defined by SGP40I    
    uint16_t return_val = 0; 
    int32_t voc_index_value = 0;
    uint16_t sraw_voc;
    uint16_t default_rh = 0x8000;
    uint16_t default_t = 0x6666;
    default_rh = (uint16_t) (hum_conv(p));
    default_t = (uint16_t) (temp_conv(t));
    return_val = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
    gpio_put(RG0_R_PIN, 1);
    if (return_val>0){
        printf("--------I2C write error SGP40 (0): %d\n", return_val);
    }else{
        gpio_put(RG0_R_PIN, 0);
    }
    GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index_value);
    *voc = voc_index_value;
    return return_val;
}


uint16_t am2320(float* out_temperature, float* out_humidity){
    uint16_t return_val;
    int i2c_ret_val = 0;
    //int i;
    uint8_t data[8];
    data[0] = 0x03;
    data[1] = 0x00;
    data[2] = 0x04;
    i2c_ret_val = i2c_write_blocking(i2c0, AM_ADDR, data, 3, 0);
    busy_wait_us(1000);
    i2c_ret_val = i2c_write_blocking(i2c0, AM_ADDR, data, 3, 0);
    gpio_put(RG0_R_PIN, 1);
    if (i2c_ret_val!=3){
        return_val = 1;
        printf("--------I2C write error AM2320(1): %d\n", i2c_ret_val);
    }else{
        return_val = 0;
        gpio_put(RG0_R_PIN, 0);
    }
    busy_wait_us(1600);
  
    i2c_ret_val = i2c_read_blocking(i2c0, AM_ADDR, data, 8, 0);
    gpio_put(RG0_R_PIN, 1);
    if (i2c_ret_val!=8){
        printf("--------I2C read error (3): %d\n", i2c_ret_val);
        return_val = 1;
    }else{
        gpio_put(RG0_R_PIN, 0);
        return_val = 0;
    }

    /* Check data[0] and data[1] */
    if (data[0] != 0x03 || data[1] != 0x04){
      return_val = 9;
    }else{
        /* Check CRC */
        uint16_t crcdata = _calc_crc16(data, 6);
        uint16_t crcread = _combine_bytes(data[7], data[6]);
        if (crcdata != crcread) {
            return_val = 10;
        }
      
        uint16_t temp16 = _combine_bytes(data[4], data[5]); 
        uint16_t humi16 = _combine_bytes(data[2], data[3]);   
        if (temp16 & 0x8000)
          temp16 = -(temp16 & 0x7FFF);
      
        *out_temperature = (float)temp16 / 10.0;
        *out_humidity = (float)humi16 / 10.0;
    }
    return return_val;
}

uint16_t pmsa003i(uint8_t* buffer){
    uint16_t return_val = 0;
    int i2c_ret_val;
    i2c_ret_val  = i2c_read_blocking(i2c0, PMSA_ADDR, buffer, 32, 0);
    gpio_put(RG0_R_PIN, 1);
    if (i2c_ret_val!=32){
        printf("--------I2C write error PMSA003i (0): %d\n", i2c_ret_val);
        return_val = 1;
    }else{
        gpio_put(RG0_R_PIN, 0);
        return_val = 0;
    }
  // Check that start byte is correct!
    if (buffer[0] != 0x42) {
        gpio_put(RG0_R_PIN, 1);
        printf("--------I2C write error PMSA003i (1): %d\n", return_val);
        return_val = 2;
    }else{
        gpio_put(RG0_R_PIN, 0);
        return_val = 0;
    }
    // get checksum ready
    return return_val;
}

void sensor_read(struct SensorData* sensor_data){
    if (sensor_run_flag){
        uint16_t return_val, i = 0;
        uint16_t return_vals[3];
        gpio_put(RG0_R_PIN, 0);
        sensor_run_flag = 0;
        return_vals[0] = pmsa003i(sensor_data->Particles_u8);
        return_vals[1] = am2320(&sensor_data->Temperature, &sensor_data->Humidity);
        return_vals[2] = sgp40(&sensor_data->Voc, sensor_data->Temperature, sensor_data->Humidity);
        for (i=0; i<3; i++){
            return_val += return_vals[i];
        }
        if (return_val==0){
            global_voc = sensor_data->Voc;
            sensor_data_ready_flag = 1;
            gpio_put(RG0_G_PIN, 0);
        }else{
            printf("--------I2C error (0): %d\n", return_val);
            sensor_data_ready_flag = 0;
        }
    }
}

int sensor_data_processing(struct SensorData* sensor_data){
    sensor_data_ready_flag = 0;
    uint8_t i;
    uint8_t index;
    char temp_str[12];
    char hum_str[12];
    char voc_str[12];
    char pm_str[15][15];

    // The data comes in endian'd, this solves it so it works on all platforms
    for (i = 0; i < 15; i++) {
        sensor_data->Particles[i] = sensor_data->Particles_u8[2 + i * 2 + 1];
        sensor_data->Particles[i] += (sensor_data->Particles_u8[2 + i * 2] << 8);
    }    

    float_to_ascii(sensor_data->temp_str, sensor_data->Temperature, 1);
    float_to_ascii(sensor_data->hum_str, sensor_data->Humidity, 1);
    int_to_ascii(sensor_data->voc_str, sensor_data->Voc, 0);

    index = 0;
    index = copy_str(response_buffer_4, sensor_data->temp_str, index);
    index = copy_str(response_buffer_4, ", ",  index);
    index = copy_str(response_buffer_4, sensor_data->hum_str, index);
    index = copy_str(response_buffer_4, ", ",  index);
    index = copy_str(response_buffer_4, sensor_data->voc_str, index);
    index = copy_str(response_buffer_4, ", ",  index);

    for (i=0; i<15; i++){
        int_to_ascii(sensor_data->pm_str[i], sensor_data->Particles[i], 0);
        index = copy_str(response_buffer_4, sensor_data->pm_str[i], index);
        index = copy_str(response_buffer_4, ", ",  index);
    }
    response_buffer_4[index] = 0;
    int r = u2u_topic_exchange(4, response_buffer_4);

    return 0;
}


/*Copy over string items into a buffer and cast each character into uint8_t.
 * Provide index = 0 for simple 1-1 copy, for concatting together items
 * into one buffer, the returned index can be provided for next item.
 * Assuming null termination.*/
uint8_t copy_char(uint8_t* buffer, const char* s, uint8_t index){
    uint8_t i = 0;
    while (s[i] !=0){
        buffer[index++] = (uint8_t) s[i++];
    }
    return index;
}


/* Dedicated core to update LCD:
 * +----------------+
 * |T: tt.t  H: hh.h|    Temperature/Humidity
 * |OC: ooo HH:MM:SS|    (V)Olatile Compounds
 * |CUSTOM TEXT DISP|    Custom line 3/0
 * |LAYING SPACE... |    Custom line 4/1
 * +----------------+
 * Lines 3/0 and 4/1 are filled in from lcd_request_data.
 * If the submitted data in lcd_request_data exceeds the 32 character limit the 
 * function will engage in display scrolling: the two lines will have the text 
 * displayed in a sliding fashion with the scroll delay set as SCROLL_DELAY.
 */
void core1(){
    uint8_t  core1_display_buffer[16*4 + 1];
    uint8_t  ret_val;
    bool     lcd_refresh_needed = 0;

    struct SensorData* sensor_datap;
    lcd_init();
    busy_wait_us(200000); // .
    /* Blanking LCD */
    for (uint8_t i=0; i<64; i++){
        core1_display_buffer[i] = (uint8_t) ' ';
    }
    uint8_t index; // MAX: 64
    uint8_t scroll_len = 0;
    static uint8_t scroll_offset = 0;
    uint8_t _lcd_data_size = 0;
    uint8_t scroll_ticker = 0;    // Counts SCROLL_DELAY until refresh if scrolling enabled.
    while (1){
        sensor_datap = &sensor_data[sd_active_buffer];
        busy_wait_us(2000); // .
        index = 0;
        index = copy_char(core1_display_buffer, "T: ", index);
        index = copy_char(core1_display_buffer, sensor_datap->temp_str, index);
        index = copy_char(core1_display_buffer, "  H: ", index);
        index = copy_char(core1_display_buffer, sensor_datap->hum_str, index);
        index = copy_char(core1_display_buffer, "OC: ", index);
        index = copy_char(core1_display_buffer, sensor_datap->voc_str, index);
        core1_display_buffer[index++] = ' ';
        //if (timer_synched){
        char time_stamp_buffer[10];
        ret_val = get_time_stamp(time_stamp_buffer, secs_to_mn);
        index = copy_char(core1_display_buffer, time_stamp_buffer, index);
        if (index < 32){  // If two lines are not completly full, fill up with empty space.
            for (size_t i=index; i<32; i++){
                core1_display_buffer[i] = (uint8_t) ' ';
            }
        }

        if (lcd_data_size > 0){
            lcd_refresh_needed = 1;
            _lcd_data_size = lcd_data_size;
            scroll_offset = 0;
            if (lcd_data_size > 32){
                scroll_len = lcd_data_size - 32;
            } else {
                scroll_len = 0;
            }
            lcd_data_size = 0;
        }

        if (lcd_refresh_needed == 1){
            for (uint8_t c=0; c<32; c++){
                if (c<_lcd_data_size){
                    core1_display_buffer[c+32] = (uint8_t) lcd_request_data[lrd_active_buffer][c+scroll_offset];
                }else{
                    core1_display_buffer[c+32] = (uint8_t) ' ';
                }
            }
            if (scroll_len > 0){
                lcd_refresh_needed = 1;
                if (++scroll_ticker == SCROLL_DELAY){
                    scroll_ticker = 0;
                    if (++scroll_offset >= scroll_len){
                        scroll_offset = 0;
                    }
                }
            }else{
                lcd_refresh_needed = 0;
            }
        }
        lcd_fill_lines(core1_display_buffer);
        busy_wait_us(200000);
    }
}

int main(){
    stdio_usb_init();
    busy_wait_us(100000); // Waiting for stdio_usb_init() to settle.
    io_setup();
    led_table_setup();
    u2u_message_setup();

    lcd_data_size = 0;
    for (size_t i=0; i<MAX_CUSTOM_DISPLAY_TEXT; i++){
        lcd_request_data[0][i] = ' ';
        lcd_request_data[1][i] = ' ';
    }

    GasIndexAlgorithm_init_with_sampling_interval(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC, sampling_interval);
    sensirion_i2c_hal_init();

    repeating_timer_t timer;
    repeating_timer_t sensor_tick;
    add_repeating_timer_ms(200, timer_callback, NULL, &timer);
    add_repeating_timer_ms(1000, sensor_ticker, NULL, &sensor_tick);
    bool mr_old = 0;
    int return_val;
    int index = 0;
    multicore_launch_core1(core1);
    struct Message* new_message = NULL;
    struct Message_Segments segments;
    gpio_put(LED_R, 0);
    gpio_put(LED_Y, 0);
    gpio_put(LED_G, 0);
    while (1){
        if (timer_synched == 0){
            gpio_put(RG1_R_PIN, 1);
        }
        bool sd_swap_buffer_ix = 1 - sd_active_buffer;
        sensor_read(&sensor_data[sd_swap_buffer_ix]);
        if (sensor_data_ready_flag){
             return_val = sensor_data_processing(&sensor_data[sd_swap_buffer_ix]);
        }
        sd_active_buffer = sd_swap_buffer_ix;
        new_message = get_message(&segments);
        if (new_message!=NULL){
             gpio_put(RG1_G_PIN, 1);
             if (new_message->topic_nr == SET_TIME_TPNR || new_message->topic_nr == GET_TIME_TPNR){
                 uint8_t count_digits = 0;
                 for (uint8_t i=0; i<5; i++){    // Checking if response has time info in payload.
                     if (is_digit(segments.payload[i]) == 1){
                         count_digits++;
                     }
                 }
                 if (count_digits == 5){    // If it has then:
                     count_digits = 0;
                     secs_to_mn = ascii_to_int(segments.payload);
                     timer_synched = 1;
                     gpio_put(RG1_R_PIN, 0);
                     count_to_sync = 2000;
                 }
             }
             if (new_message->topic_nr == SET_LCD_TPNR){
                 bool is_null = 0;
                 size_t payload_size = get_segment_length(new_message, 'P');
                 if (payload_size > MAX_CUSTOM_DISPLAY_TEXT){
                     payload_size = MAX_CUSTOM_DISPLAY_TEXT;
                 }
                 if (payload_size > 0){
                     bool lrc_swap_buffer_ix = 1 - lrd_active_buffer;
                     for (size_t c=0; c<payload_size; c++){
                         if (segments.payload[c] == '\0'){
                             is_null = 1;
                         }
                         if (is_null == 0){
                             lcd_request_data[lrc_swap_buffer_ix][c] = segments.payload[c];
                             lcd_data_size++;
                         }else{
                             lcd_request_data[lrc_swap_buffer_ix][c] = ' ';
                         }

                     }
                     lrd_active_buffer = lrc_swap_buffer_ix;
                 }
             }

            if (get_u2u_errors(PORT_0) > 0){
                get_u2u_error_log(PORT_0, error_buffer);
                printf("%s\n", error_buffer);
                clear_u2u_errors(PORT_0);
            }

            if (get_u2u_errors(PORT_1) > 0){
                get_u2u_error_log(PORT_1, error_buffer);
                printf("%s\n", error_buffer);
                clear_u2u_errors(PORT_1);
            }
            new_message = NULL;
        }
    }
    return 0;
}

bool timer_callback(repeating_timer_t *rt){    // Every 200 ms
    volatile static uint8_t sync_checking = 0;
    int r;
    gpio_put(RG1_G_PIN, 0);
    if (++ticks >= 5){
        ticks = 0;
    }
    if (ticks==0){
        if (++secs_to_mn >= (24*60*60)){
            secs_to_mn = 0;
        }
    }

    tog_led = 1-tog_led;
    gpio_put(LED_ONBOARD, tog_led);
    pwm_set_gpio_level(RGB_R_PIN, LED_table[global_voc][RED]);
    pwm_set_gpio_level(RGB_G_PIN, LED_table[global_voc][GREEN]);
    pwm_set_gpio_level(RGB_B_PIN, LED_table[global_voc][BLUE]);
    if (global_voc>425 && global_voc<450){
        pwm_set_gpio_level(RGB_G_PIN, 55535*tog_led);
    }
    if (global_voc>=450){
        pwm_set_gpio_level(RGB_B_PIN, 65535*tog_led);
    }

    if (++sync_checking > 8){
        sync_checking = 0;
        if (timer_synched==0){
            struct Message gettime_msg;
            static int chapter_counter = 0;
            char gettime_receiver[] = "GEN";
            char gettime_rflag[] = "RQ";
            char gettime_topic[] = "GET_TIME";
            char gettime_payload[] = "Please provide timing information.";

            message_clear(&gettime_msg);
            gettime_msg.receiver = gettime_receiver;
            gettime_msg.rflag = gettime_rflag;
            gettime_msg.topic = gettime_topic;
            gettime_msg.payload = gettime_payload;
            gettime_msg.chapter_int = chapter_counter++;
            gettime_msg.segment_flags = "RFTCP";
            format_message(&gettime_msg);
            u2u_send_message(&gettime_msg, 0);
            u2u_send_message(&gettime_msg, 1);
        }
    }

    return true;
}

bool sensor_ticker(repeating_timer_t *rs){     // Every second
    sensor_run_flag = 1;
    gpio_put(RG0_G_PIN, 1);
    if (--count_to_sync == 0){
        gpio_put(RG1_R_PIN, 1);
        timer_synched = 0;
    }
    return true;
}




//main.c
#include <string>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"

#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "sgp40_i2c.h"

extern "C" {
#include "sensirion_gas_index_algorithm.h"
}

extern "C" {
#include "u2u.h"
}

#define LED_ONBOARD 25

#define RGB_G_PIN   7
#define RGB_B_PIN   8
#define RGB_R_PIN   9
#define RG0_R_PIN   3
#define RG0_G_PIN   6
#define RG1_R_PIN   1
#define RG1_G_PIN   2

#if 0
#define LED_R 11
#define LED_Y 14
#define LED_G 15
#endif

#define RS          26
#define E           22
#define DB4         21
#define DB5         20
#define DB6         19
#define DB7         18

#define output_nr   14


#define SCL_PIN     16
#define SDA_PIN     17

#define RS_COMMAND  0
#define RS_DATA     1

#define LCD_FS      0x02 //Function set
#define LCD_DCO     0x28 //Display control
#define LCD_CD      0x02 //
#define LCD_DCL     0x01 //Display clear
#define LCD_MH      0x80 //Move home

#define PMSA_ADDR   0x12
#define SGP_ADDR    0x59
#define AM_ADDR     0x5C

#define PORT_0      0
#define PORT_1      1


/* Time keeping related variables: */
volatile bool timer_synched = 0;
static bool tog_led = 0;
volatile static uint8_t  ticks = 0;
volatile static uint32_t secs_to_mn = 0;
volatile static uint16_t count_to_sync = 0;
bool timer_callback(repeating_timer_t *rt);


/* LED related variables: */
uint16_t LED_table[501][3];
const uint8_t outputpins[] = {
    LED_ONBOARD, 
    RG0_R_PIN, 
    RG0_G_PIN, 
    RG1_R_PIN, 
    RG1_G_PIN, 
    E, 
    RS, 
    DB4, 
    DB5, 
    DB6, 
    DB7,
    LED_R,
    LED_Y,
    LED_G};

/* 16*4 DISPLAY related variables: */
const uint8_t lcd_datapins[] = {DB4, DB5, DB6, DB7};
char lcd_request_data[2][32];
volatile bool lrd_active_buffer = 0;
volatile uint8_t lcd_data_size;
const uint8_t lcd_display_layout[] = {0x00, 0x40, 0x10, 0x50};

/* Sensor related variables: */
bool sensor_run_flag = 0;
bool sensor_data_ready_flag = 0;
float sampling_interval = 1.f;
GasIndexAlgorithmParams voc_params;
bool sensor_ticker(repeating_timer_t *rs);
int32_t  global_voc = 0;

/* U2U communication buffers: */
char response_buffer_4[127];
char error_buffer[1024];

#if 0
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
    return false;
}
#endif


/* Require buffer of size 8+1 for NULL to hold timeformat. Format var time_ticks into HH:MM:SS. */
uint8_t get_time_stamp(char* buffer, uint32_t time_ticks){
    char digit[3];
    uint32_t temp;
    uint32_t h = time_ticks / (60*60);
    temp = time_ticks - (h*60*60);
    uint32_t m = temp / 60;
    uint32_t s = temp - (m*60);
    uint8_t index = 0;
    int_to_ascii(digit, h, 2);
    buffer[index++] = digit[0];
    buffer[index++] = digit[1];
    buffer[index++] = ':';
    int_to_ascii(digit, m, 2);
    buffer[index++] = digit[0];
    buffer[index++] = digit[1];
    buffer[index++] = ':';
    int_to_ascii(digit, s, 2);
    buffer[index++] = digit[0];
    buffer[index++] = digit[1];
    buffer[index++] = 0;
    return index;
}

struct SensorData{
    float Temperature;
    float Humidity;
    int32_t Voc;
    int32_t Voc_raw;
    int Particles[15];
    char temp_str[12];
    char hum_str[12];
    char voc_str[12];
    char pm_str[15][15];
    uint8_t Particles_u8[32];
};


struct SensorData sensor_data[2];
volatile bool sd_active_buffer = 0;

void lcd_send(uint8_t *buf, uint8_t len_buf, bool command_or_data){
    uint8_t temp = 0;
    uint8_t d = 0;
    for (uint8_t c=0; c<len_buf; c++){
        temp = buf[c];
        temp = temp >> 4;
        gpio_put(E, 1);
        busy_wait_us(10);
        for (d=0; d<4; d++){
            gpio_put(lcd_datapins[d], ((temp>>d) & 0b00000001));
        }
        gpio_put(RS, command_or_data);
        busy_wait_us(10);
        gpio_put(E, 0);
        busy_wait_us(20);
        
        temp = buf[c]; 
        gpio_put(E, 1);
        busy_wait_us(10);
        for (d=0; d<4; d++){
            gpio_put(lcd_datapins[d], ((temp>>d) & 0b00000001));
        }
        gpio_put(RS, command_or_data);
        busy_wait_us(10);
        gpio_put(E, 0);
        busy_wait_us(20);
        for (d=0; d<4; d++){
            gpio_put(lcd_datapins[d], 0);
        }
    }
}

void lcd_init(){
        //Initialisation commands:
        //-Function set (4b/2L/5x7)
        //-Display control
        //-Display clear
        //-Return home
        uint8_t initialisation_commands[] = {0x03, 0x03, 0x03, 0x02, 0x02, 0x28, 0x0E, 0x01, 0x80};
        //Initialisation phase is sandwiched with delay times that come from 
        //the datasheet. Found that when adding 300 each phase yields more reliable operation.
        const int lcd_initialisation_delay_times[] = {100, 100, 100, 40, 40, 40, 1600, 1600, 0};
        for (uint8_t i=0; i<9; i++){
                lcd_send(&initialisation_commands[i], 1, RS_COMMAND );
                busy_wait_us(300+lcd_initialisation_delay_times[i]);
        }
        //sleep_ms(100);
        busy_wait_us(100000);
        return;
}


/* Parameter buf must be 64 in size */
void lcd_fill_lines(uint8_t *buf){
    for (uint8_t lines=0; lines<4; lines++){
        uint8_t adr = 0b10000000 | lcd_display_layout[lines];
        lcd_send(&adr, 1, RS_COMMAND);
        lcd_send(buf+(16*lines), 16, RS_DATA);
    }
}


#define RED 0
#define GREEN 1
#define BLUE 2

void led_table_setup(){
    /*RGB LED will indicate air quality by colour. 
     * Blue: clear    (0   - 150), 
     * Green: mild    (125 - 300),
     * Red: polluted. (300 - 501). */
    uint16_t i;
    const float M = 65535.0;
    float mr = 0.0;

    for (i=0; i<500; i++){
        LED_table[i][0] = 0;
        LED_table[i][1] = 0;
        LED_table[i][2] = 0;
    }
    for (i=0;   i<150; i++){ LED_table[i][BLUE]  = (uint16_t) M; }
    for (i=300; i<501; i++){ LED_table[i][RED]   = (uint16_t) M; }
    for (i=125; i<300; i++){ LED_table[i][GREEN] = (uint16_t) M; }
    for (i=0; i<100; i++){
        mr = ((float) i+1)/100;
        mr = mr*mr;
        LED_table[250+i][RED]   = (uint16_t)(     (M * mr));
        LED_table[125+i][GREEN] = (uint16_t)(     (M * mr));
        LED_table[300+i][GREEN] = (uint16_t)( M - (M * mr));
        LED_table[150+i][BLUE]  = (uint16_t)( M - (M * mr));
    }
}

void io_setup(){
    uint16_t slice_num;
    for (int c=0; c<output_nr; c++){
        gpio_init(outputpins[c]);
        gpio_set_dir(outputpins[c], GPIO_OUT);
        gpio_put(outputpins[c], 0);
    }

    gpio_put(LED_R, 1);
    gpio_put(LED_Y, 1);
    gpio_put(LED_G, 1);
    gpio_set_function(RGB_R_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RGB_G_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RGB_B_PIN, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    slice_num = pwm_gpio_to_slice_num(RGB_G_PIN);
    pwm_init(slice_num, &config, true);
    slice_num = pwm_gpio_to_slice_num(RGB_R_PIN);
    pwm_init(slice_num, &config, true);
    slice_num = pwm_gpio_to_slice_num(RGB_B_PIN);
    pwm_init(slice_num, &config, true);
    
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); // Pull ups are 4K7 added externally.
    gpio_pull_up(SCL_PIN);
}

static uint16_t _calc_crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    while(len--) {
        crc ^= (uint16_t) *buf++;
        for (unsigned i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;      
            }
        }
    }
    return crc;
}

static uint16_t _combine_bytes(uint8_t msb, uint8_t lsb) {
    return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

int hum_conv(float h){
    return (int) h*(65535.0/100.0);
}

int temp_conv(float t){
    return (int) (t+45.0)*(65535.0/175.0);
}

uint16_t sgp40(int32_t* voc, float t, float p){
    uint16_t DEFAULT_COMPENSATION_RH = 0x8000;  // in ticks as defined by SGP40
    uint16_t DEFAULT_COMPENSATION_T = 0x6666;   // in ticks as defined by SGP40I    
    uint16_t return_val = 0; 
    int32_t voc_index_value = 0;
    uint16_t sraw_voc;
    uint16_t default_rh = 0x8000;
    uint16_t default_t = 0x6666;
    default_rh = (uint16_t) (hum_conv(p));
    default_t = (uint16_t) (temp_conv(t));
    return_val = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
    gpio_put(RG0_R_PIN, 1);
    if (return_val>0){
        printf("--------I2C write error SGP40 (0): %d\n", return_val);
    }else{
        gpio_put(RG0_R_PIN, 0);
    }
    GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index_value);
    *voc = voc_index_value;
    return return_val;
}


uint16_t am2320(float* out_temperature, float* out_humidity){
    uint16_t return_val;
    int i2c_ret_val = 0;
    //int i;
    uint8_t data[8];
    data[0] = 0x03;
    data[1] = 0x00;
    data[2] = 0x04;
    i2c_ret_val = i2c_write_blocking(i2c0, AM_ADDR, data, 3, 0);
    busy_wait_us(1000);
    i2c_ret_val = i2c_write_blocking(i2c0, AM_ADDR, data, 3, 0);
    gpio_put(RG0_R_PIN, 1);
    if (i2c_ret_val!=3){
        return_val = 1;
        printf("--------I2C write error AM2320(1): %d\n", i2c_ret_val);
    }else{
        return_val = 0;
        gpio_put(RG0_R_PIN, 0);
    }
    busy_wait_us(1600);
  
    i2c_ret_val = i2c_read_blocking(i2c0, AM_ADDR, data, 8, 0);
    gpio_put(RG0_R_PIN, 1);
    if (i2c_ret_val!=8){
        printf("--------I2C read error (3): %d\n", i2c_ret_val);
        return_val = 1;
    }else{
        gpio_put(RG0_R_PIN, 0);
        return_val = 0;
    }

    /* Check data[0] and data[1] */
    if (data[0] != 0x03 || data[1] != 0x04){
      return_val = 9;
    }else{
        /* Check CRC */
        uint16_t crcdata = _calc_crc16(data, 6);
        uint16_t crcread = _combine_bytes(data[7], data[6]);
        if (crcdata != crcread) {
            return_val = 10;
        }
      
        uint16_t temp16 = _combine_bytes(data[4], data[5]); 
        uint16_t humi16 = _combine_bytes(data[2], data[3]);   
        if (temp16 & 0x8000)
          temp16 = -(temp16 & 0x7FFF);
      
        *out_temperature = (float)temp16 / 10.0;
        *out_humidity = (float)humi16 / 10.0;
    }
    return return_val;
}

uint16_t pmsa003i(uint8_t* buffer){
    uint16_t return_val = 0;
    int i2c_ret_val;
    i2c_ret_val  = i2c_read_blocking(i2c0, PMSA_ADDR, buffer, 32, 0);
    gpio_put(RG0_R_PIN, 1);
    if (i2c_ret_val!=32){
        printf("--------I2C write error PMSA003i (0): %d\n", i2c_ret_val);
        return_val = 1;
    }else{
        gpio_put(RG0_R_PIN, 0);
        return_val = 0;
    }
  // Check that start byte is correct!
    if (buffer[0] != 0x42) {
        gpio_put(RG0_R_PIN, 1);
        printf("--------I2C write error PMSA003i (1): %d\n", return_val);
        return_val = 2;
    }else{
        gpio_put(RG0_R_PIN, 0);
        return_val = 0;
    }
    // get checksum ready
    return return_val;
}

void sensor_read(struct SensorData* sensor_data){
    if (sensor_run_flag){
        uint16_t return_val, i = 0;
        uint16_t return_vals[3];
        gpio_put(RG0_R_PIN, 0);
        sensor_run_flag = 0;
        return_vals[0] = pmsa003i(sensor_data->Particles_u8);
        return_vals[1] = am2320(&sensor_data->Temperature, &sensor_data->Humidity);
        return_vals[2] = sgp40(&sensor_data->Voc, sensor_data->Temperature, sensor_data->Humidity);
        for (i=0; i<3; i++){
            return_val += return_vals[i];
        }
        if (return_val==0){
            global_voc = sensor_data->Voc;
            sensor_data_ready_flag = 1;
            gpio_put(RG0_G_PIN, 0);
        }else{
            printf("--------I2C error (0): %d\n", return_val);
            sensor_data_ready_flag = 0;
        }
    }
}

int sensor_data_processing(struct SensorData* sensor_data){
    sensor_data_ready_flag = 0;
    uint8_t i;
    uint8_t index;
    char temp_str[12];
    char hum_str[12];
    char voc_str[12];
    char pm_str[15][15];

    // The data comes in endian'd, this solves it so it works on all platforms
    for (i = 0; i < 15; i++) {
        sensor_data->Particles[i] = sensor_data->Particles_u8[2 + i * 2 + 1];
        sensor_data->Particles[i] += (sensor_data->Particles_u8[2 + i * 2] << 8);
    }    

    float_to_ascii(sensor_data->temp_str, sensor_data->Temperature, 1);
    float_to_ascii(sensor_data->hum_str, sensor_data->Humidity, 1);
    int_to_ascii(sensor_data->voc_str, sensor_data->Voc, 0);


    index = 0;
    index = copy_str(response_buffer_4, sensor_data->temp_str, index);
    index = copy_str(response_buffer_4, ", ",  index);
    index = copy_str(response_buffer_4, sensor_data->hum_str, index);
    index = copy_str(response_buffer_4, ", ",  index);
    index = copy_str(response_buffer_4, sensor_data->voc_str, index);
    index = copy_str(response_buffer_4, ", ",  index);

    for (i=0; i<15; i++){
        int_to_ascii(sensor_data->pm_str[i], sensor_data->Particles[i], 0);
        index = copy_str(response_buffer_4, sensor_data->pm_str[i], index);
        index = copy_str(response_buffer_4, ", ",  index);
    }

    response_buffer_4[index] = 0;

  
    int r = u2u_topic_exchange(4, response_buffer_4);

    return 0;
}


uint8_t copy_char(uint8_t* buffer, const char* s, uint8_t index){
    /*Copy over string items into a buffer and cast each character into uint8_t.
     * Provide index = 0 for simple 1-1 copy, for concatting together items
     * into one buffer, the returned index can be provided for next item.
     * Assuming null termination.*/
    uint8_t i = 0;
    while (s[i] !=0){
        buffer[index++] = (uint8_t) s[i++];
    }
    return index;
}


/* Dedicated core to update LCD:
 * +----------------+
 * |T: tt.t  H: hh.h|    Temperature/Humidity
 * |OC: ooo HH:MM:SS|    (V)Olatile Compounds
 * |                |    Costum line 3/0
 * |                |    Costum line 4/1
 * +----------------+
 * Lines 3/0 and 4/1 are filled in from lcd_request_data.
 */
void core1(){
    uint8_t  core1_display_buffer[16*4 + 1];
    uint8_t  ret_val;
    bool     lcd_refresh_needed = 0;

    struct SensorData* sensor_datap;
    lcd_init();
    busy_wait_us(200000); // .
    /* Blanking LCD */
    for (uint8_t i=0; i<64; i++){
        core1_display_buffer[i] = (uint8_t) ' ';
    }
    uint8_t index; // MAX: 64
    while (1){
        sensor_datap = &sensor_data[sd_active_buffer];
        busy_wait_us(2000); // .
        index = 0;
        index = copy_char(core1_display_buffer, "T: ", index);
        index = copy_char(core1_display_buffer, sensor_datap->temp_str, index);
        index = copy_char(core1_display_buffer, "  H: ", index);
        index = copy_char(core1_display_buffer, sensor_datap->hum_str, index);
        index = copy_char(core1_display_buffer, "OC: ", index);
        index = copy_char(core1_display_buffer, sensor_datap->voc_str, index);
        core1_display_buffer[index++] = ' ';
        //if (timer_synched){
        char time_stamp_buffer[10];
        ret_val = get_time_stamp(time_stamp_buffer, secs_to_mn);
        index = copy_char(core1_display_buffer, time_stamp_buffer, index);
        if (index < 32){  // If two lines are not completly full, fill up with empty space.
            for (size_t i=index; i<32; i++){
                core1_display_buffer[i] = (uint8_t) ' ';
            }
        }

        if (lcd_data_size > 0){
            for (uint8_t c=0; c<32; c++){
                if (c<lcd_data_size){
                    core1_display_buffer[c+32] = (uint8_t) lcd_request_data[lrd_active_buffer][c];
                }else{
                    core1_display_buffer[c+32] = (uint8_t) ' ';
                }
            }
            lcd_data_size = 0;
        }
            
        lcd_fill_lines(core1_display_buffer);
        busy_wait_us(200000);
    }
}

int main(){
    stdio_usb_init();
    busy_wait_us(100000); // Waiting for stdio_usb_init() to settle.
    io_setup();
    led_table_setup();
    u2u_message_setup();

    lcd_data_size = 0;
    for (size_t i=0; i<32; i++){
        lcd_request_data[0][i] = ' ';
        lcd_request_data[1][i] = ' ';
    }

    GasIndexAlgorithm_init_with_sampling_interval(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC, sampling_interval);
    sensirion_i2c_hal_init();

    repeating_timer_t timer;
    repeating_timer_t sensor_tick;
    add_repeating_timer_ms(200, timer_callback, NULL, &timer);
    add_repeating_timer_ms(1000, sensor_ticker, NULL, &sensor_tick);
    bool mr_old = 0;
    int return_val;
    int index = 0;
    multicore_launch_core1(core1);
    struct Message* new_message = NULL;
    struct Message_Segments segments;
    gpio_put(LED_R, 0);
    gpio_put(LED_Y, 0);
    gpio_put(LED_G, 0);
    while (1){
        if (timer_synched == 0){
            gpio_put(RG1_R_PIN, 1);
        }
        bool sd_swap_buffer_ix = 1 - sd_active_buffer;
        sensor_read(&sensor_data[sd_swap_buffer_ix]);
        if (sensor_data_ready_flag){
             return_val = sensor_data_processing(&sensor_data[sd_swap_buffer_ix]);
        }
        sd_active_buffer = sd_swap_buffer_ix;
        new_message = get_message(&segments);
        if (new_message!=NULL){
             gpio_put(RG1_G_PIN, 1);
             //printf("\nMessage \n");
             //printf("sender: '%s'\n", segments.sender);
             //printf("receiver: '%s'\n", segments.receiver);
             //printf("rqs: '%s'\n", segments.rflag);
             //printf("topic: '%s'\n", segments.topic);
             //printf("chapter: '%s'\n", segments.chapter);
             //printf("payload: '%s'\n", segments.payload);
             //printf("hops: '%s'\n", segments.hops);
             //printf("topic nr: %d\n", new_message->topic_nr);
             if (new_message->topic_nr == SET_TIME_TPNR || new_message->topic_nr == GET_TIME_TPNR){
                 uint8_t count_digits = 0;
                 for (uint8_t i=0; i<5; i++){    // Checking if response has time info in payload.
                     if (is_digit(segments.payload[i]) == 1){
                         count_digits++;
                     }
                 }
                 if (count_digits == 5){    // If it has then:
                     count_digits = 0;
                     secs_to_mn = ascii_to_int(segments.payload);
                     timer_synched = 1;
                     gpio_put(RG1_R_PIN, 0);
                     count_to_sync = 2000;
                 }
             }
             if (new_message->topic_nr == SET_LCD_TPNR){
                 bool is_null = 0;
                 bool lrc_swap_buffer_ix = 1 - lrd_active_buffer;
                 for (size_t c=0; c<32; c++){
                     if (segments.payload[c] == '\0'){
                         is_null = 1;
                     }
                     if (is_null == 0){
                         lcd_request_data[lrc_swap_buffer_ix][c] = segments.payload[c];
                         lcd_data_size++;
                     }else{
                         lcd_request_data[lrc_swap_buffer_ix][c] = ' ';
                     }

                 }
                 lrd_active_buffer = lrc_swap_buffer_ix;
             }

            if (get_u2u_errors(PORT_0) > 0){
                get_u2u_error_log(PORT_0, error_buffer);
                printf("%s\n", error_buffer);
                clear_u2u_errors(PORT_0);
            }

            if (get_u2u_errors(PORT_1) > 0){
                get_u2u_error_log(PORT_1, error_buffer);
                printf("%s\n", error_buffer);
                clear_u2u_errors(PORT_1);
            }
            new_message = NULL;
        }
    }
    return 0;
}

bool timer_callback(repeating_timer_t *rt){    // Every 200 ms
    volatile static uint8_t sync_checking = 0;
    int r;
    gpio_put(RG1_G_PIN, 0);
    if (++ticks >= 5){
        ticks = 0;
    }
    if (ticks==0){
        if (++secs_to_mn >= (24*60*60)){
            secs_to_mn = 0;
        }
    }

    tog_led = 1-tog_led;
    gpio_put(LED_ONBOARD, tog_led);
    pwm_set_gpio_level(RGB_R_PIN, LED_table[global_voc][RED]);
    pwm_set_gpio_level(RGB_G_PIN, LED_table[global_voc][GREEN]);
    pwm_set_gpio_level(RGB_B_PIN, LED_table[global_voc][BLUE]);
    if (global_voc>425 && global_voc<450){
        pwm_set_gpio_level(RGB_G_PIN, 55535*tog_led);
    }
    if (global_voc>=450){
        pwm_set_gpio_level(RGB_B_PIN, 65535*tog_led);
    }

    if (++sync_checking > 8){
        sync_checking = 0;
        if (timer_synched==0){
            struct Message gettime_msg;
            static int chapter_counter = 0;
            char gettime_receiver[] = "GEN";
            char gettime_rflag[] = "RQ";
            char gettime_topic[] = "GET_TIME";
            char gettime_payload[] = "Please provide timing information.";

            message_clear(&gettime_msg);
            gettime_msg.receiver = gettime_receiver;
            gettime_msg.rflag = gettime_rflag;
            gettime_msg.topic = gettime_topic;
            gettime_msg.payload = gettime_payload;
            gettime_msg.chapter_int = chapter_counter++;
            gettime_msg.segment_flags = "RFTCP";
            format_message(&gettime_msg);
            u2u_send_message(&gettime_msg, 0);
            u2u_send_message(&gettime_msg, 1);
        }
    }

    return true;
}

bool sensor_ticker(repeating_timer_t *rs){     // Every second
    sensor_run_flag = 1;
    gpio_put(RG0_G_PIN, 1);
    if (--count_to_sync == 0){
        gpio_put(RG1_R_PIN, 1);
        timer_synched = 0;
    }
    return true;
}




