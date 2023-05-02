//main.c
#include <stdio.h>
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
#include "u2u.c"
}

#define LED_ONBOARD 25

#define RGB_G_PIN 7
#define RGB_B_PIN 8
#define RGB_R_PIN 9
#define RG0_R_PIN 3
#define RG0_G_PIN 6
#define RG1_R_PIN 1
#define RG1_G_PIN 2

#define RS 26
#define E 22
#define DB4 21
#define DB5 20
#define DB6 19
#define DB7 18

#define output_nr 11

#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define SCL_PIN 16
#define SDA_PIN 17

#define RS_COMMAND 0
#define RS_DATA 1

#define LCD_FS 0x02 //Function set
#define LCD_DCO 0x28 //Display control
#define LCD_CD 0x02 //
#define LCD_DCL 0x01 //Display clear
#define LCD_MH 0x80 //Move home

#define PMSA_ADDR  0x12
#define SGP_ADDR  0x59
#define AM_ADDR  0x5C


static bool tog_led = 0;
static bool tog_led2 = 0;
static int8_t ticks = 0;
static int seconds = 0;

int global_voc = 0;
int VOC;
int VOC_raw;


int LED_table[500][3];
const uint8_t outputpins[] = {LED_ONBOARD, RG0_R_PIN, RG0_G_PIN, RG1_R_PIN, RG1_G_PIN, E, RS, DB4, DB5, DB6, DB7};
const uint8_t datapins[] = {DB4, DB5, DB6, DB7};
bool timer_callback(repeating_timer_t *rt);
bool sensor_ticker(repeating_timer_t *rs);

uint8_t lcd_buffer[16*4];

const uint8_t lcd_display_layout[] = {0x00, 0x40, 0x10, 0x50};


bool sensor_run_flag = 0;
bool sensor_data_ready_flag = 0;

float sampling_interval = 1.f;
GasIndexAlgorithmParams voc_params;

char response_buffer_4[255];

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
    return false;
}


struct SensorData{
    float Temperature;
    char temp_str[12];
    float Humidity;
    char hum_str[12];
    char voc_str[12];
    char pm_str[15][15];
    int Voc;
    int Voc_raw;
    uint8_t Particles_u8[32];
    int Particles[15];
};


struct SensorData sensor_data;

void lcd_send(uint8_t *buf, uint8_t len_buf, uint8_t command_or_data){
    //printf("len buf: %d.\n", len_buf);
    uint8_t temp = 0;
    uint8_t d = 0;
    for (int c=0; c<len_buf; c++){
        temp = buf[c];
        temp = temp >> 4;
        gpio_put(E, 1);
        sleep_us(10);
        for (d=0; d<4; d++){
            gpio_put(datapins[d], ((temp>>d) & 0b00000001));
        }
        gpio_put(RS, command_or_data);
        sleep_us(10);
        gpio_put(E, 0);
        sleep_us(20);

        temp = buf[c];
        gpio_put(E, 1);
        sleep_us(10);
        for (d=0; d<4; d++){
            gpio_put(datapins[d], ((temp>>d) & 0b00000001));
        }
        gpio_put(RS, command_or_data);
        sleep_us(10);
        gpio_put(E, 0);
        sleep_us(20);
        for (d=0; d<4; d++){
            gpio_put(datapins[d], 0);
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
        //the datasheet. Found that when adding 100 each phase yields more reliable operation.
        const int lcd_initialisation_delay_times[] = {100, 100, 100, 40, 40, 40, 1600, 1600, 0};
        for (int i=0; i<9; i++){
                lcd_send(&initialisation_commands[i], 1, RS_COMMAND );
                sleep_us(300+lcd_initialisation_delay_times[i]);
        }
        sleep_ms(100);
        return;
}


void lcd_fill_lines(uint8_t *buf){
    for (int lines=0; lines<4; lines++){
        uint8_t adr = 0b10000000 | lcd_display_layout[lines];
        lcd_send(&adr, 1, RS_COMMAND);
        lcd_send(buf+(16*lines), 16, RS_DATA);
    }
}



void lcd_print(char* buf){
    int length = len(buf);
    if (length > (16*4)){
        length = (16*4);
    }
    for (int l=0; l<length; l++){
        lcd_buffer[l] = buf[l];
    }
    for (int l=length; l<(16*4); l++){
        lcd_buffer[l] = 32;
    }
    lcd_fill_lines(lcd_buffer);
}

void LED_table_setup(){
    int i, j;
    //int VOC;
    float M = 65535;
    float mr = 0;


    for (i=0; i<500; i++){
        for (j=0; j<3; j++){
            LED_table[i][j] = 0;
        }
    }
    for (i=0; i<100; i++){
        LED_table[i][2] = (int) M;
    }

    for (i=200; i<500; i++){
        LED_table[i][0] = (int) M;
    }

    for (i=125; i<175; i++){
        LED_table[i][1] = (int) M;
    }


    for (i=0; i<50; i++){
        mr = ((float) i+1)/50;
        mr = mr*mr;
        LED_table[150+i][0] = (int)((M * mr));
        LED_table[100+i][1] = (int)((M * mr));
        LED_table[175+i][1] = (int)( M - (M * mr));
        LED_table[100+i][2] = (int)( M - (M * mr));
    }
}



void io_setup(){
        uint16_t slice_num;
        for (int c=0; c<output_nr; c++){
            gpio_init(outputpins[c]);
            gpio_set_dir(outputpins[c], GPIO_OUT);
            gpio_put(outputpins[c], 0);
        }
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

int sgp40(int* voc, float t, float p){
    uint16_t DEFAULT_COMPENSATION_RH = 0x8000;  // in ticks as defined by SGP40
    uint16_t DEFAULT_COMPENSATION_T = 0x6666;   // in ticks as defined by SGP40I
    uint16_t error = 0;
    int32_t voc_index_value = 0;
    uint16_t default_rh = 0x8000;
    uint16_t default_t = 0x6666;
    default_rh = (uint16_t) (hum_conv(p));
    default_t = (uint16_t) (temp_conv(t));
    uint16_t sraw_voc;
    error = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
    GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index_value);
    *voc = voc_index_value;
    return 0;
}


int am2321(float* out_temperature, float* out_humidity){
    int return_val;
    uint8_t data[8];
    data[0] = 0x03;
    data[1] = 0x00;
    data[2] = 0x04;
    i2c_write_blocking(i2c0, AM_ADDR, data, 3, 0);
    if (return_val < 0){
        printf("--------I2C write error (0): %d\n", return_val);
    }
    sleep_us(1000);
    return_val = i2c_write_blocking(i2c0, AM_ADDR, data, 3, 0);
    if (return_val < 0){
        printf("--------I2C write error (1): %d\n", return_val);
    }
    sleep_us(1600);

  /*
   * Read out 8 bytes of data
   * Byte 0: Should be Modbus function code 0x03
   * Byte 1: Should be number of registers to read (0x04)
   * Byte 2: Humidity msb
   * Byte 3: Humidity lsb
   * Byte 4: Temperature msb
   * Byte 5: Temperature lsb
   * Byte 6: CRC lsb byte
   * Byte 7: CRC msb byte
   */

    return_val = i2c_read_blocking(i2c0, AM_ADDR, data, 8, 0);
    if (return_val < 0){
        printf("--------I2C read error (3): %d\n", return_val);
    }
    if (return_val != 8){
        printf("--------I2C read error (4): %d\n", return_val);
    }else{
        return_val = 0;
    }
    int i;

    /* Check data[0] and data[1] */
    if (data[0] != 0x03 || data[1] != 0x04)
      return 9;

    /* Check CRC */
    uint16_t crcdata = _calc_crc16(data, 6);
    uint16_t crcread = _combine_bytes(data[7], data[6]);
    if (crcdata != crcread) {
        return_val = 10;
    }

    uint16_t temp16 = _combine_bytes(data[4], data[5]);
    uint16_t humi16 = _combine_bytes(data[2], data[3]);
    //printf("temp=%u 0x%04x  hum=%u 0x%04x\n", temp16, temp16, humi16, humi16);

    /* Temperature resolution is 16Bit,
     * temperature highest bit (Bit15) is equal to 1 indicates a
     * negative temperature, the temperature highest bit (Bit15)
     * is equal to 0 indicates a positive temperature;
     * temperature in addition to the most significant bit (Bit14 ~ Bit0)
     *  indicates the temperature sensor string value.
     * Temperature sensor value is a string of 10 times the
     * actual temperature value.
     */
    if (temp16 & 0x8000)
      temp16 = -(temp16 & 0x7FFF);

    *out_temperature = (float)temp16 / 10.0;
    *out_humidity = (float)humi16 / 10.0;
    return return_val;
}

int pmsa003i(uint8_t* buffer){
    int return_val;
    return_val = i2c_read_blocking(i2c0, PMSA_ADDR, buffer, 32, 0);
  // Check that start byte is correct!
    if (buffer[0] != 0x42) {
        gpio_put(RG0_R_PIN, 1);
        return_val = 1;
    }

    // get checksum ready

    return return_val;
}

void sensor_read(struct SensorData* sensor_data){
    if (sensor_run_flag){
        sensor_run_flag = 0;
        //pmsa003i(pm_buffer);
        pmsa003i(sensor_data->Particles_u8);
        //am2321(&t, &h);
        am2321(&sensor_data->Temperature, &sensor_data->Humidity);
        //sgp40(&voc, t, h);
        sgp40(&sensor_data->Voc, sensor_data->Temperature, sensor_data->Humidity);
        //global_voc = voc;
        global_voc = sensor_data->Voc;
        sensor_data_ready_flag = 1;
        gpio_put(RG0_G_PIN, 0);
    }
}

int sensor_data_processing(struct SensorData* sensor_data){
    sensor_data_ready_flag = 0;
    int i, sum, return_val, index;
    char temp_str[12];
    char hum_str[12];
    char voc_str[12];
    char th_dis[16*4];
    char pm_str[15][15];

    // The data comes in endian'd, this solves it so it works on all platforms
    for (i = 0; i < 15; i++) {
        sensor_data->Particles[i] = sensor_data->Particles_u8[2 + i * 2 + 1];
        sensor_data->Particles[i] += (sensor_data->Particles_u8[2 + i * 2] << 8);
    }
    //memcpy((void *)pm_data, (void *)sensor_data->Particles_u8, 30);
//    for (i=0; i<30; i++){
//        pm_data[i] = sensor_data->Particles_u8[i];
//    }

//    for (i = 0; i < 30; i++) {
//        sum += sensor_data->Particles_u8[i];
//    }
//    if (sum != pm_data->checksum) {
//        gpio_put(RG0_R_PIN, 1);
//        return_val = 2;
//    }

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
        //printf("i: %d, index: %d, P: %d, ts: %s\n", i, index, sensor_data->Particles[i], pm_str[i]);
    }
    //printf("s: %s\n", response_buffer_4);
    response_buffer_4[index] = 0;

//        for (i=0; i<15; i++){
//            int_to_ascii(str_sensor_data->Particles_u8, sensor_data->Particles_u8[i], 0);
//            printf("%d, %s\n", sensor_data->Particles_u8[i], str_sensor_data->Particles_u8);
//
//        }

//        for (i=0; i<30; i++){
//            sum += sensor_data->Particles_u8[i];
//        }
//        printf("sum: %d\n", sum);

//
//        for (i=0; i<32; i++){
//            printf("pb: %d\n", sensor_data->Particles_u8[i]);
//        }
        //printf("rb: %s\n", response_buffer_4);

    msg_responses[4] = response_buffer_4; //teststring;

    return 0;
}

void core1(){
    int i, sum, return_val, index;
    char th_dis[16*4];
    int t = 0;

    struct SensorData* sensor_datap;
    sensor_datap = &sensor_data;

    lcd_init();
    sleep_ms(1000); // .
    while (1){
        index = 0;
        index = copy_str(th_dis, "T: ", index);
        index = copy_str(th_dis, sensor_datap->temp_str, index);
        index = copy_str(th_dis, "  H: ", index);
        index = copy_str(th_dis, sensor_datap->hum_str, index);
        index = copy_str(th_dis, "VOC: ", index);
        index = copy_str(th_dis, sensor_datap->voc_str, index);

        th_dis[index] = 0;
        lcd_print(th_dis);

        sleep_ms(500);
        t = 1 - t;
        //gpio_put(RG1_R_PIN, t);

    }
}



int main(){
    stdio_usb_init();
    struct Message_out* inbound_message;

    sleep_ms(1000); // Waiting for stdio_usb_init() to settle.
    io_setup();
    LED_table_setup();
    message_setup();

    GasIndexAlgorithm_init_with_sampling_interval(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC, sampling_interval);
    sensirion_i2c_hal_init();

    repeating_timer_t timer;
    repeating_timer_t sensor_tick;
    add_repeating_timer_ms(200, timer_callback, NULL, &timer);
    add_repeating_timer_ms(1000, sensor_ticker, NULL, &sensor_tick);
    bool mr_old = 0;
    int return_val;
    multicore_launch_core1(core1);
    while (1){
       sensor_read(&sensor_data);
       if (sensor_data_ready_flag){
            return_val = sensor_data_processing(&sensor_data);
       }


       if (message_ready==1){
           gpio_put(RG1_G_PIN, 1);
           inbound_message = message_processor();
           //printf("Payload received: %s\n", inbound_message->Payload);
           //printf("Topic index: %d\n", inbound_message->Topic_number);
           //lcd_print(inbound_message->Payload);
           gpio_put(RG1_G_PIN, 0);
        }
    }
    return 0;
}

bool timer_callback(repeating_timer_t *rt){
    int r;
    ticks = (ticks+1) % 4;
    if (ticks==0){
        seconds = (seconds+1)%(24*60*60);
    }

    tog_led = 1-tog_led;
    gpio_put(LED_ONBOARD, tog_led);
    //gpio_put(RG0_G_PIN, 0);
    pwm_set_gpio_level(RGB_R_PIN, LED_table[global_voc][0]);
    pwm_set_gpio_level(RGB_G_PIN, LED_table[global_voc][1]);
    pwm_set_gpio_level(RGB_B_PIN, LED_table[global_voc][2]);
    if (global_voc>250 && global_voc<300){
        pwm_set_gpio_level(RGB_G_PIN, 55535*tog_led);
    }
    if (global_voc>300){
        pwm_set_gpio_level(RGB_B_PIN, 65535*tog_led);
    }
    return true;
}

bool sensor_ticker(repeating_timer_t *rs){
    //tog_led2 = 1-tog_led2;
    sensor_run_flag = 1;
    gpio_put(RG0_G_PIN, 1);
    return true;
}

