#ifndef DA_CONFIG_CHECK_H
#define DA_CONFIG_CHECK_H

 #ifdef DA_SRF_H
  #ifndef STD_FACTOR
   #warning "STD_FACTOR not defined, set to 2"
   #define STD_FACTOR 2
  #elif STD_FACTOR <= 0
   #warning "STD_FACTOR is 0 or lower"
  #endif //STD_FACTOR
  #ifndef MIN_STD
   #warning "MIN_STD not defined, set to 10"
   #define MIN_STD 10
  #elif MIN_STD <= 0
   #warning "MIN_STD is 0 or lower. You will not get any values from the sensors. Recommend is 10"
  #endif //MIN_STD
  #ifndef SE_DATA_BUFFER_SIZE
   #warning "SE_DATA_BUFFER_SIZE not defined, set to 10"
   #define SE_DATA_BUFFER_SIZE 10
  #elif SE_DATA_BUFFER_SIZE < 1
   #warning "SE_DATA_BUFFER_SIZE lower than 1, set to 1"
   #undef SE_DATA_BUFFER_SIZE
   #define SE_DATA_BUFFER_SIZE 1
  #endif //SE_DATA_BUFFER_SIZE
 #endif //DA_SRF_H

 #ifdef DA_SLAM_H
  //#warning message("SLAM")
 #endif //DA_SLAM_H

 #ifdef DA_MIDDLE_H
  #ifndef TTY_DEVICE
   #error "TTY_DEVICE not defined!"
  #endif
  #ifndef TTY_DEVICE_SPEED
   #warning "TTY_DEVICE_SPEED not defined; set to B57600"
   #define TTY_DEVICE_SPEED B57600
  #else
   #if TTY_DEVICE_SPEED != B57600 && TTY_DEVICE_SPEED != B115200 && TTY_DEVICE_SPEED != 38400
    #warning "TTY_DEVICE_SPEED unknown; set to B57600"
    #undef TTY_DEVICE_SPEED
    #define TTY_DEVICE_SPEED B57600
   #endif
  #endif //TTY_DEVICE_SPEED
  #ifndef DATA_STREAM_SPEED
   #warning "DATA_STREAM_SPEED not defined, set to 0"
   #define DATA_STREAM_SPEED 0
  #else
   #if DATA_STREAM_SPEED < 0
    #warning "DATA_STREAM_SPEED lower than 0, set to 0"
    #undef DATA_STREAM_SPEED
    #define DATA_STREAM_SPEED 0
   #elif DATA_STREAM_SPEED > 50
    #warning "DATA_STREAM_SPEED higher than 50, set to 50"
    #undef DATA_STREAM_SPEED
    #define DATA_STREAM_SPEED 50
   #endif
  #endif // DATA_STREAM_SPEED
  #ifndef TARGET_SYSTEM
   #warning "TARGET_SYSTEM not defined; set to 1"
   #define TARGET_SYSTEM 1
  #endif
  #ifndef TARGET_COMPONENT
   #warning "TARGET_COMPONENT not defined; set to 1"
   #define TARGET_COMPONENT 1
  #endif
  #ifndef SRF_DEVICE
   #error "SRF_DEVICE not defined!"
  #endif
  #ifndef SE_COUNT
   #error "SE_COUNT not defined"
  #elif SE_COUNT < 1
   #error "SE_COUNT lower than 0"
  #elif (((1000*360)/SE_COUNT))%1000 != 0
   #error "SE_COUNT not divisor of 360"
  #endif //SE_COUNT
  #ifndef SRF_SPEED
   #warning "SRF_SPEED not defined, set to 60"
   #define SRF_SPEED 60
  #endif //SRF_SPEED
  #ifndef SE0_ADDRESS
   #error "SE0_ADDRESS not defined"
  #endif //SE0_ADDRESS
  #ifndef EXT_CTRL
   #warning "EXT_CTRL not defined, set to 0 (disabled)"
   #define EXT_CTRL 0
  #endif
  #ifndef MAX_DISTANCE
   #warning "MAX_DISTANCE ist not defined, set to 600"
  #elif MAX_DISTANCE < 0
   #warning "MAX_DISTANCE is lower than 0, set to 0"
   #undef MAX_DISTANCE
   #define MAX_DISTANCE 0
  #endif
  #ifndef MAX_ROLL
   #warning "MAX_ROLL is not defined, set to 600"
   #define MAX_ROLL 600
  #elif MAX_ROLL < 0
   #warning "MAX ROLL lower than 0, set to 0"
   #undef MAX_ROLL
   #define MAX_ROLL 0
  #endif
  #ifndef MAX_PITCH
   #warning "MAX_PITCH is not defined, set to 600"
   #define MAX_PITCH 600
  #elif MAX_PITCH < 0
   #warning "MAX_PITCH lower than 0, set to 0"
   #undef MAX_PITCH
   #define MAX_PITCH 0
  #endif
  #ifndef CONST_YAW
   #warning "CONST_YAW not defined, set to 20"
   #define CONST_YAW 20
  #endif
  #ifndef HTM_DELAY_TIME
   #warning "HTM_DELAY_TIME not defined, set to 0"
   #define HTM_DELAY_TIME 0
  #endif
  #ifndef HOLD_STILL_ROLL_KP
   #warning "HOLD_STILL_ROLL_KP not defined, set to 0"
   #define HOLD_STILL_ROLL_KP 0
  #endif
  #ifndef HOLD_STILL_ROLL_TV
   #warning "HOLD_STILL_ROLL_TV not defined, set to 0"
   #define HOLD_STILL_ROLL_TV 0
  #endif
  #ifndef HOLD_STILL_ROLL_TN
   #warning "HOLD_STILL_ROLL_TN not defined, set to 0"
   #define HOLD_STILL_ROLL_TN 0
  #endif
  #ifndef HOLD_STILL_PITCH_KP
   #warning "HOLD_STILL_PITCH_KP not defined, set to 0"
   #define HOLD_STILL_PITCH_KP 0
  #endif
  #ifndef HOLD_STILL_PITCH_TV
   #warning "HOLD_STILL_PITCH_TV not defined, set to 0"
   #define HOLD_STILL_PITCH_TV 0
  #endif
  #ifndef HOLD_STILL_PITCH_TN
   #warning "HOLD_STILL_PITCH_TN not defined, set to 0"
   #define HOLD_STILL_PITCH_TN 0
  #endif
  #ifndef HOLD_STILL_MAX_XACC
   #warning "HOLD_STILL_MAX_XACC not defined, set to 50"
   #define HOLD_STILL_MAX_XACC 50
  #elif HOLD_STILL_MAX_XACC < 0
   #warning "HOLD_STILL_MAX_XACC lower than 0, set to 50"
   #undef HOLD_STILL_MAX_XACC
   #define HOLD_STILL_MAX_XACC 50
  #endif
  #ifndef HOLD_STILL_MAX_YACC
   #warning "HOLD_STILL_MAX_YACC not defined, set to 50"
   #define HOLD_STILL_MAX_YACC 50
  #elif HOLD_STILL_MAX_YACC < 0
   #warning "HOLD_STILL_MAX_YACC lower than 0, set to 50"
   #undef HOLD_STILL_MAX_YACC
   #define HOLD_STILL_MAX_YACC 50
  #endif
  #ifndef STILL_FAK_ROLL
   #warning "STILL_FAK_ROLL not defined, set to 5"
   #define STILL_FAK_TOLL 5
  #elif STILL_FAK_ROLL < 0
   #warning "STILL_FAK_ROLL lower than 0, set to 0"
   #undef STILL_FAK_ROLL
   #define STILL_FAK_ROLL 0;
  #endif
  #ifndef STILL_FAK_PITCH
   #warning "STILL_FAK_PITCH not defined, set to 5"
   #define STILL_FAK_TOLL 5
  #elif STILL_FAK_PITCH < 0
   #warning "STILL_FAK_PITCH lower than 0, set to 0"
   #undef STILL_FAK_PITCH
   #define STILL_FAK_PITCH 0;
  #endif
  #ifndef CHECK_STILL_SPEED
   #warning "CHECK_STILL_SPEED not defined, set to 100"
   #define CHECK_STILL_SPEED 1000
  #endif
  #ifndef CHECK_STILL_COUNT
   #warning "CHECK_STILL_COUNT not defined, set to 5"
   #define CHECK_STILL_COUNT 5
  #endif
  #ifndef LOG_SPEED
   #warning "LOG_SPEED not defined, set to 1000"
   #define LOG_SPEED 1000
  #endif
  #ifndef WARNINGS
   #warning "WARNINGS not defined, set to 0"
   #define WARNINGS 0
  #endif
  #ifndef SLAM_RESOLUTION
   #warning "SLAM_RESOLUTION not defined, set to 1"
   #define SLAM_RESOLUTION 1
  #elif SLAM_RESOLUTION < 1
   #warning "SLAM_RESOLUTION lower than 1, set to 1"
   #undef SLAM_RESOLUTION
   #define SLAM_RESOLUTION
  #endif
 #endif //DA_MIDDLE_H
#endif //DA_CONFIG_CHECK_H
