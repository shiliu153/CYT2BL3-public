#include "zf_common_headfile.h"

typedef struct
{
    float Kp, Ki, Kd;
    float error, last_error, pre_last_error;
    float integral;
    float output;
} PID_t;

typedef enum 
{
    MENU_MAIN,
    MENU_RUN_MAIN_PROGRAM,
    MENU_SPEED_DEBUG,
    MENU_PARAM_DEBUG,
    MENU_THRESHOLD_DEBUG,
    MENU_PREVIEW_VARS,
    MENU_SERVO_DEBUG,
    MENU_IMU_DEBUG,
    MENU_TRACKING_PARAM_DEBUG
} menu_state_t;