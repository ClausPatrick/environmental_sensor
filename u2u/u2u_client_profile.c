//u2uclientprofile.c
#include "u2u_client_profile.h"




#define TP_MSG_HAIL                "Response from topic: 'HAIL____'. "    // 0 
#define TP_MSG_HELP                "Response from topic: 'HELP____'. "    // 1
#define TP_MSG_SET_LCD             "Response from topic: 'SET_LCD_'. "    // 2
#define TP_MSG_OLED                "Response from topic: 'SET_OLED'. "    // 3
#define TP_MSG_GET_SENSOR          "Response from topic: 'GET_SNSR'. "    // 4
#define TP_MSG_SET_ENCODER         "Response from topic: 'SET_ENCR'. "    // 5
#define TP_MSG_GET_ENCODER         "Response from topic: 'GET_ENCR'. "    // 6
#define TP_MSG_SET_LED             "Response from topic: 'SET_LED_'. "    // 7
#define TP_MSG_SET_TIME            "Response from topic: 'SET_TIME'. "    // 8
#define TP_MSG_GET_TIME            "Response from topic: 'GET_TIME'. "    // 9
#define TP_MSG_SET_DATE            "Response from topic: 'SET_DATE'. "    // 10
#define TP_MSG_GET_DATE            "Response from topic: 'GET_DATE'. "    // 11
#define TP_MSG_RESERVED_0          "Response from topic: 'RSERVD_0'. "    // 12
#define TP_MSG_RESERVED_1          "Response from topic: 'RSERVD_1'. "    // 13
#define TP_MSG_RESERVED_2          "Response from topic: 'RSERVD_2'. "    // 14
#define TP_MSG_RESERVED_3          "Response from topic: 'RSERVD_3'. "    // 15
#define TP_MSG_RESERVED_4          "Response from topic: 'RSERVD_4'. "    // 16
#define TP_MSG_UNRECOGISNED        "Unrecognised topic:              "    // 17

const char* topic_default_responses[] = {
                                TP_MSG_HAIL,               
                                TP_MSG_HELP,               
                                TP_MSG_SET_LCD,            
                                TP_MSG_OLED,               
                                TP_MSG_GET_SENSOR,         
                                TP_MSG_SET_ENCODER,        
                                TP_MSG_GET_ENCODER,        
                                TP_MSG_SET_LED,            
                                TP_MSG_SET_TIME,           
                                TP_MSG_GET_TIME,           
                                TP_MSG_SET_DATE,           
                                TP_MSG_GET_DATE,           
                                TP_MSG_RESERVED_0,         
                                TP_MSG_RESERVED_1,         
                                TP_MSG_RESERVED_2,         
                                TP_MSG_RESERVED_3,         
                                TP_MSG_RESERVED_4,         
                                TP_MSG_UNRECOGISNED};
