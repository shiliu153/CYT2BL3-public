#include "zf_common_headfile.h"
#include <math.h>

// ----------------- IMU 四元数姿态解算（陀螺积分） -----------------
// 说明：仅使用陀螺仪进行四元数更新，yaw 为相对角（会漂移）。

#ifndef ATTITUDE_GYRO_BIAS_CALIBRATE
#define ATTITUDE_GYRO_BIAS_CALIBRATE (1)   // 1: 开机静止校准零偏  0: 不校准
#endif

#ifndef ATTITUDE_GYRO_BIAS_SAMPLES
#define ATTITUDE_GYRO_BIAS_SAMPLES   (200) // 校准采样数（每次采样间隔 2ms，约 400ms）
#endif

#ifndef ATTITUDE_USE_ACC_FUSION
#define ATTITUDE_USE_ACC_FUSION      (1)    // 1: Mahony(gyro+acc)  0: 仅陀螺积分
#endif

// Mahony 参数（6轴：gyro+acc）
#ifndef ATTITUDE_MAHONY_KP
#define ATTITUDE_MAHONY_KP           (2.0f)
#endif

#ifndef ATTITUDE_MAHONY_KI
#define ATTITUDE_MAHONY_KI           (0.05f)
#endif

// 加速度计低通（抑制车体振动） 0=不滤波，越大越跟随
#ifndef ATTITUDE_ACC_LPF_ALPHA
#define ATTITUDE_ACC_LPF_ALPHA       (0.20f)
#endif

volatile float g_att_q0 = 1.0f;
volatile float g_att_q1 = 0.0f;
volatile float g_att_q2 = 0.0f;
volatile float g_att_q3 = 0.0f;

static float s_gyro_bias_x_dps = 0.0f;
static float s_gyro_bias_y_dps = 0.0f;
static float s_gyro_bias_z_dps = 0.0f;

static float s_att_int_x = 0.0f;
static float s_att_int_y = 0.0f;
static float s_att_int_z = 0.0f;

static float s_acc_filt_x = 0.0f;
static float s_acc_filt_y = 0.0f;
static float s_acc_filt_z = 0.0f;

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

// ----------------- 电机引脚定义 -----------------
#define MOTOR_L_DIR_PIN         (P00_2)
#define MOTOR_L_PWM_PIN         (TCPWM_CH13_P00_3)
#define MOTOR_R_DIR_PIN         (P18_6)
#define MOTOR_R_PWM_PIN         (TCPWM_CH50_P18_7)

// ----------------- 舵机定义 -----------------
#define SERVO_PWM_PIN               (TCPWM_CH52_P18_5)
#define SERVO_PWM_FREQ              (300)          
uint32 g_servo_min_duty =3833;
uint32 g_servo_max_duty = 4900;
uint32 g_servo_center_duty = 4400;
uint32 g_servo_duty = 4400;

// ----------------- 编码器引脚定义 -----------------
#define ENCODER_L               (TC_CH09_ENCODER)
#define ENCODER_L_A_PIN         (TC_CH09_ENCODER_CH1_P05_0)
#define ENCODER_L_B_PIN         (TC_CH09_ENCODER_CH2_P05_1)
#define ENCODER_R               (TC_CH07_ENCODER)
#define ENCODER_R_A_PIN         (TC_CH07_ENCODER_CH1_P02_0)
#define ENCODER_R_B_PIN         (TC_CH07_ENCODER_CH2_P02_1)

// ----------------- 按键引脚定义 -----------------
#define KEY1                    (P11_0)
#define KEY2                    (P22_0)
#define KEY3                    (P23_3)
#define KEY4                    (P23_4)
#define LED1                    (P23_7)

// ----------------- 循迹控制参数 -----------------
#define SPEED_CONTROL_PERIOD    (10000)
#ifndef STEERING_CONTROL_PERIOD
#define STEERING_CONTROL_PERIOD (SPEED_CONTROL_PERIOD)
#endif

// ----------------- 阿克曼角差速参数 -----------------
#ifndef ACKERMANN_WHEELBASE_MM
#define ACKERMANN_WHEELBASE_MM      (155.0f)   // 轴距
#endif

#ifndef ACKERMANN_TRACK_MM
#define ACKERMANN_TRACK_MM          (115.0f)   // 后轮轮距(左右驱动轮中心距)
#endif

#ifndef ACKERMANN_MAX_STEER_DEG
#define ACKERMANN_MAX_STEER_DEG     (15.0f)    // 舵机打满对应的最大前轮转角(左右对称)
#endif

#ifndef ACKERMANN_SCALE_LIMIT
#define ACKERMANN_SCALE_LIMIT       (0.95f)    // 防止内侧速度变负/过小的保护限幅
#endif

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// duty -> 前轮转角（左正右负）
static float servo_duty_to_steer_rad(uint32 duty)
{
    const float center = (float)g_servo_center_duty;
    const float min_duty = (float)g_servo_min_duty;
    const float max_duty = (float)g_servo_max_duty;
    const float max_angle_rad = (float)ACKERMANN_MAX_STEER_DEG * (3.1415926f / 180.0f);

    if ((float)duty >= center)
    {
        float denom = (max_duty - center);
        if (denom < 1.0f) denom = 1.0f;
        float t = ((float)duty - center) / denom;
        t = clampf(t, 0.0f, 1.0f);
        return -t * max_angle_rad;
    }
    else
    {
        float denom = (center - min_duty);
        if (denom < 1.0f) denom = 1.0f;
        float t = (center - (float)duty) / denom;
        t = clampf(t, 0.0f, 1.0f);
        return t * max_angle_rad;
    }
}
int16   g_max_target_speed = 1000;
#define IPS200_TYPE     (IPS200_TYPE_SPI)
int     g_speed_pid_integral_threshold = 75;
float   g_speed_pid_integral_max = 1000.0f; 



//------------------ 前瞻变量------------------
int g_error_row_1 = 55;
int g_error_row_2 = 60;
int g_error_row_3 = 65;
int g_error_row_4 = 75;

float g_weight_1 = 0.2f;
float g_weight_2 = 0.2f;
float g_weight_3 = 0.3f;
float g_weight_4 = 0.3f;


//------------------ 赛道宽度变量 ------------------
int g_track_width_top = 60;
int g_track_width_bottom = 180;


// ----------------- Flash存储参数定义 -----------------
#define PARAMS_FLASH_MAGIC        (0xABCD1239)
#define PARAMS_FLASH_PAGE_INDEX   (0)
#define PARAMS_LEN                (29)

// ===================================================================================================================
// ================================================= 全局变量定义 ====================================================
// ===================================================================================================================

// -- 菜单状态 --
volatile menu_state_t g_menu_state = MENU_MAIN;
int g_dynamic_threshold = 128;          // 图像二值化阈值
int16 g_base_speed   = 350;              // 编码器脉冲数/10ms
float_t g_max_pwm_duty = 30.0;              // 电机最大PWM占空比比例 (0-100)
float_t g_min_start_duty = 15.0f;        // 电机最小启动占空比 (%)
float g_speed_steer_scale = 0.0f;        // 差速比例
volatile bool g_stop_active = false;       // 停车线触发后的停车保持标志（由 stop_line_ported 驱动）

// 运行模式下大津降采样步长
#ifndef OTSU_DOWNSAMPLE_STEP_RUN
#define OTSU_DOWNSAMPLE_STEP_RUN     (2)
#endif

// 调试模式下大津降采样步长
#ifndef OTSU_DOWNSAMPLE_STEP_DEBUG
#define OTSU_DOWNSAMPLE_STEP_DEBUG   (2)
#endif

// 运行模式下每 N 帧更新一次大津阈值
#ifndef OTSU_UPDATE_INTERVAL_RUN
#define OTSU_UPDATE_INTERVAL_RUN     (10)
#endif

// 调试模式下每 N 帧更新一次大津阈值
#ifndef OTSU_UPDATE_INTERVAL_DEBUG
#define OTSU_UPDATE_INTERVAL_DEBUG   (10)
#endif

// 噪滤波控制
#ifndef IMAGE_FILTER_ENABLE
#define IMAGE_FILTER_ENABLE          (1)
#endif

// 每 N 帧滤波一次
#ifndef IMAGE_FILTER_INTERVAL
#define IMAGE_FILTER_INTERVAL        (3)
#endif

// 仅对指定行范围滤波
#ifndef IMAGE_FILTER_ROW_START
#define IMAGE_FILTER_ROW_START       (40)
#endif

#ifndef IMAGE_FILTER_ROW_END
#define IMAGE_FILTER_ROW_END         (70)
#endif


uint8 g_image_binary[MT9V03X_H][MT9V03X_W]; // 二值化后图像数组
uint8 g_left_line[MT9V03X_H];               // 赛道左边界
uint8 g_right_line[MT9V03X_H];              // 赛道右边界
uint8 g_center_line[MT9V03X_H];             // 赛道中心线

// 八邻域巡线临时缓冲区
static uint8 s_bin_image[MT9V03X_H][MT9V03X_W];
static int16 s_right_border[MT9V03X_H];
static int16 s_left_border[MT9V03X_H];
static uint16 s_points_l[1024][2];
static uint16 s_points_r[1024][2];
static int8 s_dir_l[1024];
static int8 s_dir_r[1024];
static uint8 s_l_dir_sum[9];
static uint8 s_r_dir_sum[9];
static uint16 s_points_l_back[512][2];
static uint16 s_points_r_back[512][2];
static uint16 s_l_back_cnt = 0;
static uint16 s_r_back_cnt = 0;
static uint16 s_trace_l_total = 0;
static uint16 s_trace_r_total = 0;
static uint8 s_trace_valid = 0;

// ----------------- 元素识别/处理 -----------------

static uint8 s_width_begin[MT9V03X_H];
static uint8 s_width_zzh[MT9V03X_H];
static uint8 s_width_inited = 0;
static int s_width_last_top = -1;
static int s_width_last_bottom = -1;

static uint8 s_flag_r_cir = 0;
static uint8 s_flag_l_cir = 0;
static uint8 s_flag_cross = 0;
static uint8 s_flag_others2 = 1;

static uint8 s_cir_stage = 0;
static uint8 s_cir_x = 0;
static uint8 s_cir_y = 0;
static int16 s_encoder_sum = 0;
static int16 s_max_encoder = 1700;
static int16 s_encoder_sum_l4 = 800;
static int16 s_encoder_sum_l7 = 600;
static int16 s_encoder_sum_r4 = 800;
static int16 s_encoder_sum_r7 = 500;

static int16 s_length = 0;
static uint8 s_lose_num = 0;
static uint8 s_nolose_num = 0;
static int16 s_r_lose_num = 0;
static int16 s_l_lose_num = 0;
static uint8 s_r_lose_judge[MT9V03X_H];
static uint8 s_l_lose_judge[MT9V03X_H];

static int s_cross_jud_time = 0;
static int s_cross_jud = 0;

static bool s_flag_str = false;
static bool s_flag_str_str = false;

static uint8 s_flag_stop = 0;
static int s_stop_line_time = 0;
static int16 s_encoder_sum_stop = 0;
static uint8 s_stop_arm = 0;

static uint8 s_flag_bar = 0;
static uint8 s_bar_count_l = 0;
static uint8 s_bar_count_r = 0;

static int16 s_l_line_qulv = 0;
static int16 s_r_line_qulv = 0;

static volatile int16 s_warn_sink_i16 = 0;

static int16 s_center_line_i16[MT9V03X_H];

static int16 s_black_jiexian = 0;

static uint16 s_data_statics_l = 0;
static uint16 s_data_statics_r = 0;


PID_t g_steering_pid;                       // 转向PID控制器 (外环)
PID_t g_speed_pid_l;                        // 左轮速度PID (内环)
PID_t g_speed_pid_r;                        // 右轮速度PID (内环)

float g_final_error = 0;                     // 最终计算的偏差值

volatile int16 g_target_speed_l = 0;                 // 左轮目标速度
volatile int16 g_target_speed_r = 0;                 // 右轮目标速度
int16 g_actual_speed_l = 0;                 // 左轮实际速度
int16 g_actual_speed_r = 0;                 // 右轮实际速度
volatile int32 g_encoder_delta_accum = 0;    // PIT 中断累加的编码器增量
float_t g_motor_out_l = 0;                    // 左电机最终编码器速度
float_t g_motor_out_r = 0;                    // 右电机最终编码器速度
float_t g_motor_out_l_duty = 0;               // 左电机最终输出PWM占空比
float_t g_motor_out_r_duty = 0;             // 右电机最终输出PWM占空比
volatile bool g_track_lost = false;
volatile bool g_run_enable = false;





// ===================================================================================================================
// ================================================== 函数声明 =======================================================
// ===================================================================================================================

static inline int zf_abs_i(int v) { return (v >= 0) ? v : -v; }
static void image_filter_ported(uint8 img[MT9V03X_H][MT9V03X_W]);
static void image_draw_rectan_ported(uint8 img[MT9V03X_H][MT9V03X_W]);
static void width_change_ported(void);
static uint8 width_from_row(int row);
static bool stop_line_ported(int16 encoder_data_1);
static int cross_delay_judge_ported(void);
static void element_test_ported(void);
static uint8 straight_judge_ported(void);
static uint8 straight_judge_2_ported(void);
static void judge_bar_ported(void);
static void element_handle_ported(int16 encoder_data_1);

void motor_init(void);
void pid_init(PID_t *pid, float kp, float ki, float kd);
void steering_pid_calculate(void);
void speed_pid_calculate(PID_t *pid, int16 target_speed, int16 actual_speed);
int calculate_otsu_threshold(const uint8 (*image)[MT9V03X_W]);
static int calculate_otsu_threshold_downsampled(const uint8 (*image)[MT9V03X_W], int step);
void image_process(uint8 threshold);

// 姿态解算
static void attitude_quat_reset(void);
static void attitude_gyro_bias_calibrate(uint16 samples);
static void attitude_quat_update_gyro_dps(float gx_dps, float gy_dps, float gz_dps, float dt_s);
static void attitude_quat_update_mahony_imu6_dps_g(float gx_dps, float gy_dps, float gz_dps,
                                                   float ax_g, float ay_g, float az_g,
                                                   float dt_s);
void calculate_line_error(void);
void steering_control(void);
void steering_control_servo_only(void);
bool check_track_lost(void);
void params_save_to_flash(void);
void params_load_from_flash(void);
void keys_init(void);


uint8 get_key_value(void);
void show_main_menu(void);
void menu_speed_debug_task(void);
void menu_param_debug_task(void);
void menu_threshold_debug_task(void);
void run_main_program_task(void);
void menu_preview_var_task(void);
void menu_servo_debug_task(void);
void menu_imu_debug_task(void);
void menu_tracking_param_debug_task(void);


// ===================================================================================================================
// =================================================== 主函数 ========================================================
// ===================================================================================================================
 int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);
    debug_init();

    // ----------------- 外设初始化 -----------------
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    keys_init();
    motor_init();
    encoder_quad_init(ENCODER_L, ENCODER_L_A_PIN, ENCODER_L_B_PIN);
    encoder_quad_init(ENCODER_R, ENCODER_R_A_PIN, ENCODER_R_B_PIN);
    system_delay_ms(100);

    ips200_init(IPS200_TYPE);
    ips200_show_string(0, 0, "Camera Init...");
    system_delay_ms(300);


    while(mt9v03x_init())
    {
        gpio_toggle_level(LED1);
        ips200_show_string(0, 15, "Init Failed!");
        system_delay_ms(500);
    }
    ips200_clear();
    ips200_show_string(0, 0, "Init Success!");
    system_delay_ms(500);

    imu660rb_init();

    attitude_quat_reset();
#if ATTITUDE_GYRO_BIAS_CALIBRATE
    // 提示：校准期间请保持车体静止
    attitude_gyro_bias_calibrate(ATTITUDE_GYRO_BIAS_SAMPLES);
#endif

    
    g_dynamic_threshold = calculate_otsu_threshold((const uint8 (*)[MT9V03X_W])mt9v03x_image);
    
    // ----------------- PID 初始化 -----------------
    pid_init(&g_steering_pid, 9.5f, 0.00f, 0.1f);
    pid_init(&g_speed_pid_l, 6.0f, 0.3f, 0.0f);
    pid_init(&g_speed_pid_r, 6.0f, 0.3f, 0.0f);


    // ----------------- Flash参数初始化 -----------------
    flash_init();
    params_load_from_flash();
    ips200_clear();
    ips200_show_string(0, 0, "Params Loaded!");
    g_servo_duty = g_servo_center_duty;
    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, g_servo_duty);

    pit_us_init(PIT_CH0, SPEED_CONTROL_PERIOD);
    pit_us_init(PIT_CH1, STEERING_CONTROL_PERIOD);

    while(true)
    {
        switch(g_menu_state)
        {
            case MENU_MAIN:
                show_main_menu();
                break;
            case MENU_RUN_MAIN_PROGRAM:
                run_main_program_task();
                break;
            case MENU_SPEED_DEBUG:
                menu_speed_debug_task();
                break;
            case MENU_PARAM_DEBUG:
                menu_param_debug_task();
                break;
            case MENU_THRESHOLD_DEBUG:
                menu_threshold_debug_task();
                break;
            case MENU_PREVIEW_VARS:
                menu_preview_var_task();             
                break;
           case MENU_SERVO_DEBUG:
                menu_servo_debug_task();
                break;
             case MENU_IMU_DEBUG:
                 menu_imu_debug_task();
                 break;
            case MENU_TRACKING_PARAM_DEBUG:
                menu_tracking_param_debug_task();
                break;
        }
        system_delay_ms(50);
    }
}

static void draw_edges_overlay(void)
{
    const int x_off = 0;
    const int y_off = 0;
    const int thickness = 1;

    for (int row = 0; row < MT9V03X_H; row++)
    {
        uint16 y = (uint16)(y_off + row);
        int l = (int)s_left_border[row];
        int r = (int)s_right_border[row];
        int m = (int)s_center_line_i16[row];

        for (int dx = -thickness; dx <= thickness; dx++)
        {
            int lx = l + dx;
            int rx = r + dx;
            int mx = m + dx;

            if (l != -1 && lx >= 0 && lx < MT9V03X_W)
            {
                ips200_draw_point((uint16)(x_off + lx), y, RGB565_GREEN);
            }
            if (r != 200 && rx >= 0 && rx < MT9V03X_W)
            {
                ips200_draw_point((uint16)(x_off + rx), y, RGB565_RED);
            }
            if (mx >= 0 && mx < MT9V03X_W)
            {
                ips200_draw_point((uint16)(x_off + mx), y, RGB565_BLUE);
            }
        }
    }
}

static void draw_trace_points_overlay(void)
{
    if (!s_trace_valid)
    {
        return;
    }

    for (uint16 i = 0; i < s_trace_l_total && i < 1024; i++)
    {
        int x = (int)s_points_l[i][0];
        int y = (int)s_points_l[i][1];
        if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
        {
            ips200_draw_point((uint16)x, (uint16)y, RGB565_GREEN);
        }
    }

    for (uint16 i = 0; i < s_trace_r_total && i < 1024; i++)
    {
        int x = (int)s_points_r[i][0];
        int y = (int)s_points_r[i][1];
        if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
        {
            ips200_draw_point((uint16)x, (uint16)y, RGB565_RED);
        }
    }

    // 向后爬出来又撤回的点，用不同颜色标记
    for (uint16 i = 0; i < s_l_back_cnt && i < 512; i++)
    {
        int x = (int)s_points_l_back[i][0];
        int y = (int)s_points_l_back[i][1];
        if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
        {
            ips200_draw_point((uint16)x, (uint16)y, RGB565_CYAN);
        }
    }

    for (uint16 i = 0; i < s_r_back_cnt && i < 512; i++)
    {
        int x = (int)s_points_r_back[i][0];
        int y = (int)s_points_r_back[i][1];
        if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
        {
            ips200_draw_point((uint16)x, (uint16)y, RGB565_CYAN);
        }
    }
}


// ===================================================================================================================
// ================================================== 菜单功能函数定义 =================================================
// ===================================================================================================================

/**
 * @brief       按键初始化
 */
void keys_init(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);
}

/**
 * @brief       获取按键值
 * @return      按键值 (1-4), 无按键返回 0
 */
uint8 get_key_value(void)
{
    if(!gpio_get_level(KEY1)) { system_delay_ms(20); while(!gpio_get_level(KEY1)); return 1; }
    if(!gpio_get_level(KEY2)) { system_delay_ms(20); while(!gpio_get_level(KEY2)); return 2; }
    if(!gpio_get_level(KEY3)) { system_delay_ms(20); while(!gpio_get_level(KEY3)); return 3; }
    if(!gpio_get_level(KEY4)) { system_delay_ms(20); while(!gpio_get_level(KEY4)); return 4; }
    return 0;
}

/**
 * @brief       显示主菜单并处理按键
 */
#define MAIN_MENU_ITEM_COUNT 8
void show_main_menu(void)
{
    int sel = 0;
    const char *items[MAIN_MENU_ITEM_COUNT] = {
        "Run Main Program",
        "Speed Debug",
        "Param Debug",
        "Threshold Debug",
        "Preview Vars",
        "Servo Debug",
        "IMU Debug",
        "Tracking Param Debug"
    };

    while(g_menu_state == MENU_MAIN)
    {
        ips200_show_string(0, 0,  "====== Main Menu ======");
        for(int i=0;i<MAIN_MENU_ITEM_COUNT;i++){
            char buf[64];
            sprintf(buf, "%c %s", (i==sel)?'>':' ', items[i]);
            ips200_show_string(0, 15 + i*15, buf);
        }
        ips200_show_string(0, 120, "K2:Up K3:Down K1/K4:Enter");

        uint8 key = get_key_value();
        if(key == 2) {
            sel = (sel - 1 + MAIN_MENU_ITEM_COUNT) % MAIN_MENU_ITEM_COUNT;
        } else if(key == 3) {
            sel = (sel + 1) % MAIN_MENU_ITEM_COUNT;
        } else if(key == 1 || key == 4) {
            switch(sel)
            {
                case 0: g_menu_state = MENU_RUN_MAIN_PROGRAM; break;
                case 1: g_menu_state = MENU_SPEED_DEBUG;      break;
                case 2: g_menu_state = MENU_PARAM_DEBUG;      break;
                case 3: g_menu_state = MENU_THRESHOLD_DEBUG;  break;
                case 4: g_menu_state = MENU_PREVIEW_VARS;    break;
                case 5: g_menu_state = MENU_SERVO_DEBUG;     break;
                case 6: g_menu_state = MENU_IMU_DEBUG;       break;
                case 7: g_menu_state = MENU_TRACKING_PARAM_DEBUG; break;
            }
        }
        system_delay_ms(80);
    }
    ips200_clear();
}

/**
 * @brief       运行主循迹程序任务
 */
void run_main_program_task(void)
{
    ips200_show_string(0, 120, "Running... K1 to exit");
    g_track_lost = false;
    g_run_enable = false;
    g_target_speed_l = 0;
    g_target_speed_r = 0;
    system_delay_ms(1000);
    g_run_enable = true;

    static uint16 s_otsu_frame_cnt = 0;
    while(1)
    {
        if(mt9v03x_finish_flag)
        {
            if (OTSU_UPDATE_INTERVAL_RUN <= 1 || (s_otsu_frame_cnt % OTSU_UPDATE_INTERVAL_RUN) == 0)
            {
                g_dynamic_threshold = calculate_otsu_threshold_downsampled((const uint8 (*)[MT9V03X_W])mt9v03x_image, OTSU_DOWNSAMPLE_STEP_RUN);
            }
            s_otsu_frame_cnt++;
            image_process(g_dynamic_threshold);
            
            calculate_line_error();
        
            g_track_lost = check_track_lost();
            mt9v03x_finish_flag = 0;
        }
        if(g_track_lost)
        {
            g_run_enable = false;
            g_target_speed_l = 0;
            g_target_speed_r = 0;
            pwm_set_duty(MOTOR_L_PWM_PIN, 0);
            pwm_set_duty(MOTOR_R_PWM_PIN, 0);
            
            ips200_show_string(0, 30, "Track Lost! Exiting...");
            gpio_toggle_level(LED1);
            system_delay_ms(500);
            gpio_toggle_level(LED1);
            break;
        }

        if(get_key_value() == 1)
        {
            g_run_enable = false;
            g_target_speed_l = 0;
            g_target_speed_r = 0;
            pwm_set_duty(MOTOR_L_PWM_PIN, 0);
            pwm_set_duty(MOTOR_R_PWM_PIN, 0);
            break;
        }
        system_delay_ms(1);
    }
    g_run_enable = false;
    g_menu_state = MENU_MAIN;
}

void menu_speed_debug_task(void)
{
    int selected_param_index = 0;
    bool edit_mode = false;
    bool test_running = false;
    const int param_count = 6;
    const float kp_step = 0.1f;
    const float ki_step = 0.01f;
    const int16 speed_step = 10;

    ips200_clear();
    ips200_show_string(0, 0, "--- Speed PID Debug ---");
    g_track_lost = false;

    g_target_speed_l = 0;
    g_target_speed_r = 0;
    
    pwm_set_duty(SERVO_PWM_PIN, g_servo_center_duty);

    while(1)
    {
        char text_buff[64];

        sprintf(text_buff, "L:%4d/%4d R:%4d/%4d", (int)g_actual_speed_l, (int)g_target_speed_l, (int)g_actual_speed_r, (int)g_target_speed_r);
        ips200_show_string(0, 15, text_buff);

        sprintf(text_buff, "PWM L:%.1f R:%.1f", (double)g_motor_out_l_duty, (double)g_motor_out_r_duty);
        ips200_show_string(0, 30, text_buff);

        sprintf(text_buff, "%c L Kp: %.4f", (selected_param_index == 0) ? (edit_mode ? '*' : '>') : ' ', (double)g_speed_pid_l.Kp);
        ips200_show_string(0, 45, text_buff);

        sprintf(text_buff, "%c L Ki: %.5f", (selected_param_index == 1) ? (edit_mode ? '*' : '>') : ' ', (double)g_speed_pid_l.Ki);
        ips200_show_string(0, 60, text_buff);

        sprintf(text_buff, "%c R Kp: %.4f", (selected_param_index == 2) ? (edit_mode ? '*' : '>') : ' ', (double)g_speed_pid_r.Kp);
        ips200_show_string(0, 75, text_buff);

        sprintf(text_buff, "%c R Ki: %.5f", (selected_param_index == 3) ? (edit_mode ? '*' : '>') : ' ', (double)g_speed_pid_r.Ki);
        ips200_show_string(0, 90, text_buff);

        sprintf(text_buff, "%c Base Spd: %d", (selected_param_index == 4) ? (edit_mode ? '*' : '>') : ' ', (int)g_base_speed);
        ips200_show_string(0, 105, text_buff);


        sprintf(text_buff, "%c %s", (selected_param_index == 5) ? '>' : ' ', test_running ? "Stop Test" : "Start Test");
        ips200_show_string(0, 150, text_buff);

        uint8 key = get_key_value();

        if (key == 1) {
            params_save_to_flash();
            g_menu_state = MENU_MAIN;
            g_target_speed_l = 0;
            g_target_speed_r = 0;
            pwm_set_duty(MOTOR_L_PWM_PIN, 0);
            pwm_set_duty(MOTOR_R_PWM_PIN, 0);
            ips200_clear();
            break;
        }
        else if (key == 4) {
            if (selected_param_index == 5) {
                test_running = !test_running;
                if (test_running) {
                    system_delay_ms(1000);
                    g_target_speed_l = g_base_speed;
                    g_target_speed_r = g_base_speed;
                } else {
                    g_target_speed_l = 0;
                    g_target_speed_r = 0;
                }
            } else {
                edit_mode = !edit_mode;
            }
        }
        else if (edit_mode) {
            if (key == 2) {
                switch(selected_param_index) {
                    case 0: g_speed_pid_l.Kp += kp_step; break;
                    case 1: g_speed_pid_l.Ki += ki_step; break;
                    case 2: g_speed_pid_r.Kp += kp_step; break;
                    case 3: g_speed_pid_r.Ki += ki_step; break;
                    case 4: g_base_speed += speed_step; if(g_base_speed > 1000) g_base_speed = 1000; break;
                }
            } else if (key == 3) {
                switch(selected_param_index) {
                    case 0: g_speed_pid_l.Kp -= kp_step; if(g_speed_pid_l.Kp < 0) g_speed_pid_l.Kp = 0; break;
                    case 1: g_speed_pid_l.Ki -= ki_step; if(g_speed_pid_l.Ki < 0) g_speed_pid_l.Ki = 0; break;
                    case 2: g_speed_pid_r.Kp -= kp_step; if(g_speed_pid_r.Kp < 0) g_speed_pid_r.Kp = 0; break;
                    case 3: g_speed_pid_r.Ki -= ki_step; if(g_speed_pid_r.Ki < 0) g_speed_pid_r.Ki = 0; break;
                    case 4: g_base_speed -= speed_step; if(g_base_speed < 0) g_base_speed = 0; break;
                }
            }
            if (test_running && selected_param_index == 4) {
                g_target_speed_l = g_base_speed;
                g_target_speed_r = g_base_speed;
            }
        }
        else if (!edit_mode && key == 3)
        {
            selected_param_index = (selected_param_index + 1) % param_count;
        }
        else if (!edit_mode && key == 2)
        {
            selected_param_index = (selected_param_index - 1 + param_count) % param_count;
        }
        system_delay_ms(10);
    }
}

/**
 * @brief       参数调试任务
 */
void menu_param_debug_task(void)
{
    int selected_param_index = 0;
    bool edit_mode = false;
    const int param_count = 6;
    const float p_step = 0.05f;
    const float i_step = 0.001f;
    const float d_step = 0.1f;
    const float duty_step = 1.0f;
    const float scale_step = 0.01f;
    const int base_speed_step = 10;

    ips200_clear();
    ips200_show_string(0, 0, "--- Param Debug ---");

    while(1)
    {
        char text_buff[64];

        sprintf(text_buff, "%c Steer P: %.3f", (selected_param_index == 0) ? (edit_mode ? '*' : '>') : ' ', (double)g_steering_pid.Kp);
        ips200_show_string(0, 15, text_buff);

        sprintf(text_buff, "%c Steer I: %.3f", (selected_param_index == 1) ? (edit_mode ? '*' : '>') : ' ', (double)g_steering_pid.Ki);
        ips200_show_string(0, 30, text_buff);

        sprintf(text_buff, "%c Steer D: %.3f", (selected_param_index == 2) ? (edit_mode ? '*' : '>') : ' ', (double)g_steering_pid.Kd);
        ips200_show_string(0, 45, text_buff);

        sprintf(text_buff, "%c Base Speed: %d", (selected_param_index == 3) ? (edit_mode ? '*' : '>') : ' ', (int)g_base_speed);
        ips200_show_string(0, 60, text_buff);

        sprintf(text_buff, "%c Steer Scale: %.3f", (selected_param_index == 4) ? (edit_mode ? '*' : '>') : ' ', (double)g_speed_steer_scale);
        ips200_show_string(0, 75, text_buff);

        sprintf(text_buff, "%c Max Duty: %.1f", (selected_param_index == 5) ? (edit_mode ? '*' : '>') : ' ', (double)g_max_pwm_duty);
        ips200_show_string(0, 90, text_buff);

        ips200_show_string(0, 150, "K1:Exit K4:Edit K2/3:Inc/Dec");

        uint8 key = get_key_value();
        if(key == 1) {
            params_save_to_flash();
            g_menu_state = MENU_MAIN;
            ips200_clear();
            break;
        }
        else if (key == 4) {
            edit_mode = !edit_mode;
            if(edit_mode) ips200_clear();
        }
        else if (edit_mode) {
            if (key == 2) {
                switch(selected_param_index) {
                    case 0: g_steering_pid.Kp += p_step; break;
                    case 1: g_steering_pid.Ki += i_step; break;
                    case 2: g_steering_pid.Kd += d_step; break;
                    case 3: g_base_speed += base_speed_step; if(g_base_speed > 1000) g_base_speed = 1000; break;
                    case 4: g_speed_steer_scale += scale_step; break;
                    case 5: g_max_pwm_duty += duty_step; if(g_max_pwm_duty > 40.0f) g_max_pwm_duty = 40.0f; break;
                }
            } else if (key == 3) {
                switch(selected_param_index) {
                    case 0: g_steering_pid.Kp -= p_step; if(g_steering_pid.Kp < 0) g_steering_pid.Kp = 0; break;
                    case 1: g_steering_pid.Ki -= i_step; if(g_steering_pid.Ki < 0) g_steering_pid.Ki = 0; break;
                    case 2: g_steering_pid.Kd -= d_step; if(g_steering_pid.Kd < 0) g_steering_pid.Kd = 0; break;
                    case 3: g_base_speed -= base_speed_step; if(g_base_speed < 0) g_base_speed = 0; break;
                    case 4: g_speed_steer_scale -= scale_step; if(g_speed_steer_scale < 0) g_speed_steer_scale = 0; break;
                    case 5: g_max_pwm_duty -= duty_step; if(g_max_pwm_duty < g_min_start_duty) g_max_pwm_duty = g_min_start_duty; break;
                }
            }
        }
        else if (!edit_mode && key == 3)
        {
            selected_param_index = (selected_param_index + 1) % param_count;
        }
        else if (!edit_mode && key == 2)
        {
            selected_param_index = (selected_param_index - 1 + param_count) % param_count;
        }
        system_delay_ms(100);
    }
}

/**
 * @brief       图像阈值调试
 */
void menu_threshold_debug_task(void)
{
    ips200_show_string(0, 120, "Auto Otsu Threshold");
    ips200_show_string(0, 135, "K1: Exit");

    g_target_speed_l = 0;
    g_target_speed_r = 0;

    while(1)
    {
        if (get_key_value() == 1) {
            pwm_set_duty(SERVO_PWM_PIN, g_servo_center_duty);
            g_menu_state = MENU_MAIN;
            ips200_clear();
            break;
        }

        if(mt9v03x_finish_flag)
        {
            static uint16 s_otsu_frame_cnt = 0;
            if (OTSU_UPDATE_INTERVAL_DEBUG <= 1 || (s_otsu_frame_cnt % OTSU_UPDATE_INTERVAL_DEBUG) == 0)
            {
                g_dynamic_threshold = calculate_otsu_threshold_downsampled((const uint8 (*)[MT9V03X_W])mt9v03x_image, OTSU_DOWNSAMPLE_STEP_DEBUG);
            }
            s_otsu_frame_cnt++;
            image_process(g_dynamic_threshold);
            
            calculate_line_error();
            
            
            char text_buff[32];
            ips200_displayimage03x((const uint8 *)g_image_binary, MT9V03X_W, MT9V03X_H);
            draw_trace_points_overlay();
            draw_edges_overlay();
            mt9v03x_finish_flag = 0;

            sprintf(text_buff, "Thresh: %d", g_dynamic_threshold);
            ips200_show_string(0, 160, text_buff);
            
            sprintf(text_buff, "Error: %.2f", (double)g_final_error);
            ips200_show_string(0, 175, text_buff);

            sprintf(text_buff, "Error: %.2f", (double)g_steering_pid.output);
            ips200_show_string(0, 200, text_buff);

        }
        system_delay_ms(10);
    }
}

/**
 * @brief       前瞻变量调试任务
 */
void menu_preview_var_task(void)
{
    int idx = 0;
    bool edit = false;
    const int cnt = 8;
    while(1)
    {
        char buf[64];
        sprintf(buf, "%c Row1: %d", (idx==0)?(edit?'*':'>'):' ', g_error_row_1); ips200_show_string(0, 10, buf);
        sprintf(buf, "%c Row2: %d", (idx==1)?(edit?'*':'>'):' ', g_error_row_2); ips200_show_string(0, 25, buf);
        sprintf(buf, "%c Row3: %d", (idx==2)?(edit?'*':'>'):' ', g_error_row_3); ips200_show_string(0, 40, buf);
        sprintf(buf, "%c Row4: %d", (idx==3)?(edit?'*':'>'):' ', g_error_row_4); ips200_show_string(0, 55, buf);

        sprintf(buf, "%c W1: %.3f", (idx==4)?(edit?'*':'>'):' ', (double)g_weight_1); ips200_show_string(0, 80, buf);
        sprintf(buf, "%c W2: %.3f", (idx==5)?(edit?'*':'>'):' ', (double)g_weight_2); ips200_show_string(0, 95, buf);
        sprintf(buf, "%c W3: %.3f", (idx==6)?(edit?'*':'>'):' ', (double)g_weight_3); ips200_show_string(0, 110, buf);
        sprintf(buf, "%c W4: %.3f", (idx==7)?(edit?'*':'>'):' ', (double)g_weight_4); ips200_show_string(0, 125, buf);

        ips200_show_string(0, 150, "K1:Back K4:Edit K2/3:Inc/Dec");
        uint8 key = get_key_value();
        if(key == 1) {
            params_save_to_flash();
            g_menu_state = MENU_MAIN;
            ips200_clear();
            break;}
        else if(key == 4) edit = !edit;
        else if(edit && key == 2) {
            if(idx < 4) {
                if(idx==0) g_error_row_1 = MIN(MAX(g_error_row_1+1,0), MT9V03X_H-1);
                if(idx==1) g_error_row_2 = MIN(MAX(g_error_row_2+1,0), MT9V03X_H-1);
                if(idx==2) g_error_row_3 = MIN(MAX(g_error_row_3+1,0), MT9V03X_H-1);
                if(idx==3) g_error_row_4 = MIN(MAX(g_error_row_4+1,0), MT9V03X_H-1);
            } else {
                if(idx==4) g_weight_1 += 0.01f;
                if(idx==5) g_weight_2 += 0.01f;
                if(idx==6) g_weight_3 += 0.01f;
                if(idx==7) g_weight_4 += 0.01f;
            }
        }
        else if(edit && key == 3) {
            if(idx < 4) {
                if(idx==0) g_error_row_1 = MAX(MIN(g_error_row_1-1, MT9V03X_H-1), 0);
                if(idx==1) g_error_row_2 = MAX(MIN(g_error_row_2-1, MT9V03X_H-1), 0);
                if(idx==2) g_error_row_3 = MAX(MIN(g_error_row_3-1, MT9V03X_H-1), 0);
                if(idx==3) g_error_row_4 = MAX(MIN(g_error_row_4-1, MT9V03X_H-1), 0);
            } else {
                if(idx==4) g_weight_1 = MAX(g_weight_1-0.01f, 0.0f);
                if(idx==5) g_weight_2 = MAX(g_weight_2-0.01f, 0.0f);
                if(idx==6) g_weight_3 = MAX(g_weight_3-0.01f, 0.0f);
                if(idx==7) g_weight_4 = MAX(g_weight_4-0.01f, 0.0f);
            }
        }
        else if(!edit && key == 3) idx = (idx + 1) % cnt;
        else if(!edit && key == 2) idx = (idx - 1 + cnt) % cnt;

        system_delay_ms(100);
    }
    g_menu_state = MENU_MAIN;
}

/** 
* @brief       舵机调试任务  
**/
void menu_servo_debug_task(void)
{
    int sel = 0;
    const int cnt = 4;
    bool edit = false;
    int temp_duty = (int)g_servo_center_duty;

    pwm_set_duty(SERVO_PWM_PIN, g_servo_center_duty);

    while(1)
    {
        ips200_show_string(0, 0, "--- Servo Debug (Duty) ---");

        char buf[64];
        sprintf(buf, "%c Min Duty : %d", (sel==0)?(edit?'*':'>'):' ', (int)g_servo_min_duty);
        ips200_show_string(0, 15, buf);
        sprintf(buf, "%c Max Duty : %d", (sel==1)?(edit?'*':'>'):' ', (int)g_servo_max_duty);
        ips200_show_string(0, 30, buf);
        sprintf(buf, "%c Center Duty: %d", (sel==2)?(edit?'*':'>'):' ', (int)g_servo_center_duty);
        ips200_show_string(0, 45, buf);
        sprintf(buf, "%c Test Duty  : %d", (sel==3)?(edit?'*':'>'):' ', temp_duty);
        ips200_show_string(0, 60, buf);

        ips200_show_string(0, 120, "K1:Back K4:Edit K2/3:Inc/Dec");

        uint8 key = get_key_value();
        if(key == 1) {
            params_save_to_flash();
            pwm_set_duty(SERVO_PWM_PIN, g_servo_center_duty);
            g_menu_state = MENU_MAIN;
            ips200_clear();
            break;
        }
        else if(key == 4) {
            edit = !edit;
            if(!edit && sel != 3) {
                pwm_set_duty(SERVO_PWM_PIN, g_servo_center_duty);
            }
        }
        else if(edit) {
            int step = 20;
            if(key == 2) {
                switch(sel) {
                    case 0: g_servo_min_duty += step; break;
                    case 1: g_servo_max_duty += step; break;
                    case 2: g_servo_center_duty += step; break;
                    case 3: temp_duty += step; break;
                }
            } else if(key == 3) {
                switch(sel) {
                    case 0: g_servo_min_duty -= step; break;
                    case 1: g_servo_max_duty -= step; break;
                    case 2: g_servo_center_duty -= step; break;
                    case 3: temp_duty -= step; break;
                }
            }
        }
        else if(!edit) {
            if(key == 3) sel = (sel + 1) % cnt;
            else if(key == 2) sel = (sel - 1 + cnt) % cnt;
        }
        
        if(edit) {
            int drive_duty = 0;
            switch(sel) {
                case 0: drive_duty = g_servo_min_duty; break;
                case 1: drive_duty = g_servo_max_duty; break;
                case 2: drive_duty = g_servo_center_duty; break;
                case 3: drive_duty = temp_duty; break;
            }
            pwm_set_duty(SERVO_PWM_PIN, drive_duty);
        }
        
        system_delay_ms(80);
    }
}

/**
 * @brief       IMU660RB 数据调试页面
 */
void menu_imu_debug_task(void)
{
    ips200_clear();
    ips200_show_string(0, 0, "--- IMU660RB Debug ---");

    bool init_ok = false;
    for(int retry = 0; retry < 3; retry++)
    {
        if(imu660rb_init() == 0)
        {
            init_ok = true;
            break;
        }
        ips200_show_string(0, 15, "Init fail, retry...");
        system_delay_ms(200);
    }

    if(!init_ok)
    {
        ips200_show_string(0, 30, "IMU init failed (K1 back)");
        while(get_key_value() != 1) { system_delay_ms(50); }
        ips200_clear();
        g_menu_state = MENU_MAIN;
        return;
    }

    attitude_quat_reset();
#if ATTITUDE_GYRO_BIAS_CALIBRATE
    ips200_show_string(0, 160, "Gyro bias calibrating...");
    attitude_gyro_bias_calibrate(ATTITUDE_GYRO_BIAS_SAMPLES);
    ips200_show_string(0, 160, "                         ");
#endif

    while(1)
    {
        imu660rb_get_acc();
        imu660rb_get_gyro();

        // 四元数更新：调试页以 10ms 刷新
        const float dt_s = 0.01f;
        float gx_dps = imu660rb_gyro_transition(imu660rb_gyro_x);
        float gy_dps = imu660rb_gyro_transition(imu660rb_gyro_y);
        float gz_dps = imu660rb_gyro_transition(imu660rb_gyro_z);

#if ATTITUDE_USE_ACC_FUSION
        float ax_g = imu660rb_acc_transition(imu660rb_acc_x);
        float ay_g = imu660rb_acc_transition(imu660rb_acc_y);
        float az_g = imu660rb_acc_transition(imu660rb_acc_z);
        attitude_quat_update_mahony_imu6_dps_g(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt_s);
#else
        attitude_quat_update_gyro_dps(gx_dps, gy_dps, gz_dps, dt_s);
#endif

        char buf[64];

        sprintf(buf, "Acc raw  X:%6d Y:%6d", imu660rb_acc_x, imu660rb_acc_y);
        ips200_show_string(0, 15, buf);
        sprintf(buf, "Acc raw  Z:%6d", imu660rb_acc_z);
        ips200_show_string(0, 30, buf);

        sprintf(buf, "Acc g    X:%6.3f Y:%6.3f", (double)imu660rb_acc_transition(imu660rb_acc_x), (double)imu660rb_acc_transition(imu660rb_acc_y));
        ips200_show_string(0, 45, buf);
        sprintf(buf, "Acc g    Z:%6.3f", (double)imu660rb_acc_transition(imu660rb_acc_z));
        ips200_show_string(0, 60, buf);

        sprintf(buf, "Gyro raw X:%6d Y:%6d", imu660rb_gyro_x, imu660rb_gyro_y);
        ips200_show_string(0, 85, buf);
        sprintf(buf, "Gyro raw Z:%6d", imu660rb_gyro_z);
        ips200_show_string(0, 100, buf);

        sprintf(buf, "Gyro dps X:%6.2f Y:%6.2f", (double)imu660rb_gyro_transition(imu660rb_gyro_x), (double)imu660rb_gyro_transition(imu660rb_gyro_y));
        ips200_show_string(0, 115, buf);
        sprintf(buf, "Gyro dps Z:%6.2f", (double)imu660rb_gyro_transition(imu660rb_gyro_z));
        ips200_show_string(0, 130, buf);

        // 显示四元数与欧拉角（度）
        float q0 = g_att_q0;
        float q1 = g_att_q1;
        float q2 = g_att_q2;
        float q3 = g_att_q3;

        sprintf(buf, "Quat q0:% .4f q1:% .4f", (double)q0, (double)q1);
        ips200_show_string(0, 145, buf);
        sprintf(buf, "Quat q2:% .4f q3:% .4f", (double)q2, (double)q3);
        ips200_show_string(0, 160, buf);

        // ZYX: yaw-pitch-roll
        float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
        float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
        float roll = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2.0f * (q0*q2 - q3*q1);
        if (sinp > 1.0f) sinp = 1.0f;
        if (sinp < -1.0f) sinp = -1.0f;
        float pitch = asinf(sinp);

        float siny_cosp = 2.0f * (q0*q3 + q1*q2);
        float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
        float yaw = atan2f(siny_cosp, cosy_cosp);

        const float rad2deg = 57.2957795f;
        sprintf(buf, "RPY deg R:%6.1f P:%6.1f", (double)(roll*rad2deg), (double)(pitch*rad2deg));
        ips200_show_string(0, 175, buf);
        sprintf(buf, "RPY deg Y:%6.1f", (double)(yaw*rad2deg));
        ips200_show_string(0, 190, buf);

        ips200_show_string(0, 210, "K1:Back  K4:Reinit");

        uint8 key = get_key_value();
        if(key == 1)
        {
            g_menu_state = MENU_MAIN;
            ips200_clear();
            break;
        }
        else if(key == 4)
        {
            imu660rb_init();
            attitude_quat_reset();
#if ATTITUDE_GYRO_BIAS_CALIBRATE
            ips200_show_string(0, 210, "Gyro bias calibrating...");
            attitude_gyro_bias_calibrate(ATTITUDE_GYRO_BIAS_SAMPLES);
#endif
        }

        system_delay_ms(10);
    }
}

/**
 * @brief       循迹参数调试任务
 */
void menu_tracking_param_debug_task(void)
{
    int sel = 0;
    const int cnt = 5;
    bool edit = false;

    while(1)
    {
        char buf[64];
        ips200_show_string(0, 0, "--- Tracking Param Debug ---");

        sprintf(buf, "%c Max Spd: %d", (sel==0)?(edit?'*':'>'):' ', g_max_target_speed);
        ips200_show_string(0, 15, buf);
        sprintf(buf, "%c Spd I Thres: %d", (sel==1)?(edit?'*':'>'):' ', g_speed_pid_integral_threshold);
        ips200_show_string(0, 30, buf);
        sprintf(buf, "%c Spd I Max: %.2f", (sel==2)?(edit?'*':'>'):' ', (double)g_speed_pid_integral_max);
        ips200_show_string(0, 45, buf);
        
        sprintf(buf, "%c Track Top: %d", (sel==3)?(edit?'*':'>'):' ', g_track_width_top);
        ips200_show_string(0, 60, buf);
        sprintf(buf, "%c Track Bot: %d", (sel==4)?(edit?'*':'>'):' ', g_track_width_bottom);
        ips200_show_string(0, 75, buf);

        ips200_show_string(0, 150, "K1:Back K4:Edit K2/3:Inc/Dec");
        uint8 key = get_key_value();
        if(key == 1) {
            params_save_to_flash();
            g_menu_state = MENU_MAIN;
            break;
        }
        else if(key == 4) {
            edit = !edit;
        }
        else if(edit) {
            if(key == 2) {
                switch(sel) {
                    case 0: g_max_target_speed += 10; break;
                    case 1: g_speed_pid_integral_threshold += 1; break;
                    case 2: g_speed_pid_integral_max += 10.0f; break;
                    case 3: g_track_width_top += 2; break;
                    case 4: g_track_width_bottom += 2; break;
                }
            } else if(key == 3) {
                switch(sel) {
                    case 0: g_max_target_speed -= 10; if(g_max_target_speed < 0) g_max_target_speed = 0; break;
                    case 1: g_speed_pid_integral_threshold -= 1; if(g_speed_pid_integral_threshold < 0) g_speed_pid_integral_threshold = 0; break;
                    case 2: g_speed_pid_integral_max -= 10.0f; if(g_speed_pid_integral_max < 0) g_speed_pid_integral_max = 0; break;
                    case 3: g_track_width_top -= 2; if(g_track_width_top < 0) g_track_width_top = 0; break;
                    case 4: g_track_width_bottom -= 2; if(g_track_width_bottom < 0) g_track_width_bottom = 0; break;
                }
            }
        }
        else if(!edit) {
            if(key == 3) sel = (sel + 1) % cnt;
            else if(key == 2) sel = (sel - 1 + cnt) % cnt;
        }
        system_delay_ms(100);
    }
}





// ===================================================================================================================
// ================================================== 核心功能函数定义 =================================================
// ===================================================================================================================

/**
 * @brief       使用大津法计算图像的阈值(旧版，已废弃不再使用)
 */
int calculate_otsu_threshold(const uint8 (*image)[MT9V03X_W])
{
    uint32 histogram[256] = {0};
    uint32 total_pixels = MT9V03X_W * MT9V03X_H;

    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            histogram[image[i][j]]++;
        }
    }

    uint32 sum = 0;
    for (int i = 0; i < 256; i++)
    {
        sum += i * histogram[i];
    }

    uint32 sum_b = 0;
    uint32 w_b = 0;
    float max_variance = 0;
    int threshold = 0;

    for (int i = 0; i < 256; i++)
    {
        w_b += histogram[i];
        if (w_b == 0)
        {
            continue;
        }

        uint32 w_f = total_pixels - w_b;
        if (w_f == 0)
        {
            break;
        }

        sum_b += i * histogram[i];

        float mu_b = (float)sum_b / w_b;
        float mu_f = (float)(sum - sum_b) / w_f;

        float variance_between = (float)w_b * (float)w_f * (mu_b - mu_f) * (mu_b - mu_f);

        if (variance_between > max_variance)
        {
            max_variance = variance_between;
            threshold = i;
        }
    }

    return threshold;
}


/**
 * @brief 大津法下采样版本(算力优化)
 */
static int calculate_otsu_threshold_downsampled(const uint8 (*image)[MT9V03X_W], int step)
{
    if (step <= 1)
    {
        return calculate_otsu_threshold(image);
    }

    uint32 histogram[256] = {0};
    uint32 sample_h = (uint32)((MT9V03X_H + step - 1) / step);
    uint32 sample_w = (uint32)((MT9V03X_W + step - 1) / step);
    uint32 total_pixels = sample_h * sample_w;

    for (int i = 0; i < MT9V03X_H; i += step)
    {
        for (int j = 0; j < MT9V03X_W; j += step)
        {
            histogram[image[i][j]]++;
        }
    }

    uint32 sum = 0;
    for (int i = 0; i < 256; i++)
    {
        sum += (uint32)i * histogram[i];
    }

    uint32 sum_b = 0;
    uint32 w_b = 0;
    float max_variance = 0;
    int threshold = 0;

    for (int i = 0; i < 256; i++)
    {
        w_b += histogram[i];
        if (w_b == 0)
        {
            continue;
        }

        uint32 w_f = total_pixels - w_b;
        if (w_f == 0)
        {
            break;
        }

        sum_b += (uint32)i * histogram[i];

        float mu_b = (float)sum_b / (float)w_b;
        float mu_f = (float)(sum - sum_b) / (float)w_f;

        float variance_between = (float)w_b * (float)w_f * (mu_b - mu_f) * (mu_b - mu_f);

        if (variance_between > max_variance)
        {
            max_variance = variance_between;
            threshold = i;
        }
    }

    return threshold;
}

void motor_init(void)
{
    gpio_init(MOTOR_L_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR_L_PWM_PIN, 17000, 0);
    pwm_init(MOTOR_R_PWM_PIN, 17000, 0);
}

void pid_init(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0;
    pid->last_error = 0;
    pid->pre_last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

/**
 * @brief  转向PID控制器计算 (外环)
 */
void  steering_pid_calculate(void)
{
    g_steering_pid.error = g_final_error;
        
    g_steering_pid.integral += g_steering_pid.error;
    if(g_steering_pid.integral > g_speed_pid_integral_max)
    {
        g_steering_pid.integral = g_speed_pid_integral_max;
    }
    else if(g_steering_pid.integral < -g_speed_pid_integral_max)
    {
        g_steering_pid.integral = -g_speed_pid_integral_max;
    }

    float gyro_z_dps = imu660rb_gyro_transition(imu660rb_gyro_z);

    static float gyro_z_filt = 0.0f;
    const float gyro_lpf_alpha = 0.10f;
    const float gyro_deadband_dps = 3.0f;
    const float gyro_clamp_dps = 200.0f;

    gyro_z_filt += gyro_lpf_alpha * (gyro_z_dps - gyro_z_filt);

    float gyro_abs = (gyro_z_filt < 0.0f) ? -gyro_z_filt : gyro_z_filt;
    if (gyro_abs < gyro_deadband_dps)
    {
        gyro_z_filt = 0.0f;
    }
    else
    {
        if (gyro_z_filt > gyro_clamp_dps) gyro_z_filt = gyro_clamp_dps;
        if (gyro_z_filt < -gyro_clamp_dps) gyro_z_filt = -gyro_clamp_dps;
    }

    float d_term = -g_steering_pid.Kd * gyro_z_filt;

    g_steering_pid.output = g_steering_pid.Kp * g_steering_pid.error +
                            g_steering_pid.Ki * g_steering_pid.integral +
                            d_term;
                            
    g_steering_pid.last_error = g_steering_pid.error;
}

/**
 * @brief  速度PID控制器计算 (内环)
 */
void speed_pid_calculate(PID_t *pid, int16 target_speed, int16 actual_speed)
{
    pid->error = target_speed - actual_speed;

    float increment = pid->Kp * (pid->error - pid->last_error) + 
                      pid->Ki * pid->error + 
                      pid->Kd * (pid->error - 2 * pid->last_error + pid->pre_last_error);
    
    pid->output = increment;

    pid->pre_last_error = pid->last_error;
    pid->last_error = pid->error;
}


bool get_start_points(int start_row, const uint8 bin_image[][MT9V03X_W], int *l_x, int *l_y, int *r_x, int *r_y)
{
    bool l_found = false;
    bool r_found = false;
    *l_x = 0;
    *r_x = 0;
    *l_y = start_row;
    *r_y = start_row;

    for (int x = MT9V03X_W / 2; x > 1; x--)
    {
        if (bin_image[start_row][x] == 255 && bin_image[start_row][x - 1] == 0)
        {
            *l_x = x;
            l_found = true;
            break;
        }
    }

    for (int x = MT9V03X_W / 2; x < MT9V03X_W - 1; x++)
    {
        if (bin_image[start_row][x] == 255 && bin_image[start_row][x + 1] == 0)
        {
            *r_x = x;
            r_found = true;
            break;
        }
    }

    return l_found && r_found;
}

static inline bool in_bounds(int x, int y)
{
    return ((unsigned)(x - 1) < (unsigned)(MT9V03X_W - 2)) && ((unsigned)(y - 1) < (unsigned)(MT9V03X_H - 2));
}

bool search_eight_neighbor(uint16 break_flag, uint8 bin_image[][MT9V03X_W], int l_start_x, int l_start_y, int r_start_x, int r_start_y, uint16 *l_count, uint16 *r_count, uint8 *highest_row)
{
    static const int8 seeds_l[8][2] = { {0, 1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };     // 顺时针
    static const int8 seeds_r[8][2] = { {0, 1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1} };     // 逆时针

    uint8 search_fields_l[8][2] = { {0} };
    uint8 temp_l[8][2] = { {0} };
    uint8 center_point_l[2] = { (uint8)l_start_x, (uint8)l_start_y };
    uint8 index_l = 0;

    uint8 search_fields_r[8][2] = { {0} };
    uint8 temp_r[8][2] = { {0} };
    uint8 center_point_r[2] = { (uint8)r_start_x, (uint8)r_start_y };
    uint8 index_r = 0;

    uint16 l_data_statics = *l_count;
    uint16 r_data_statics = *r_count;
    *highest_row = (uint8)MT9V03X_H;

    // 回退点仅用于阈值调试界面的轨迹显示（RGB565_CYAN）
    const bool record_back_points = (g_menu_state == MENU_THRESHOLD_DEBUG);
    s_l_back_cnt = 0;
    s_r_back_cnt = 0;

    for (int i = 0; i < 9; i++)
    {
        s_l_dir_sum[i] = 0;
        s_r_dir_sum[i] = 0;
    }

    while (break_flag-- && l_data_statics < 1024 && r_data_statics < 1024)
    {
        for (uint8 i = 0; i < 8; i++)
        {
            search_fields_l[i][0] = (uint8)(center_point_l[0] + seeds_l[i][0]);
            search_fields_l[i][1] = (uint8)(center_point_l[1] + seeds_l[i][1]);
        }

        s_points_l[l_data_statics][0] = center_point_l[0];
        s_points_l[l_data_statics][1] = center_point_l[1];
        s_dir_l[l_data_statics] = -1;
        if (in_bounds(center_point_l[0], center_point_l[1]))
        {
            bin_image[center_point_l[1]][center_point_l[0]] = 31;
        }
        l_data_statics++;

        for (uint8 i = 0; i < 8; i++)
        {
            search_fields_r[i][0] = (uint8)(center_point_r[0] + seeds_r[i][0]);
            search_fields_r[i][1] = (uint8)(center_point_r[1] + seeds_r[i][1]);
        }

        s_points_r[r_data_statics][0] = center_point_r[0];
        s_points_r[r_data_statics][1] = center_point_r[1];
        s_dir_r[r_data_statics] = -1;
        if (in_bounds(center_point_r[0], center_point_r[1]))
        {
            bin_image[center_point_r[1]][center_point_r[0]] = 50;
        }

        index_l = 0;
        const bool l_near_top = (center_point_l[1] < 5);

        for (uint8 i = 0; i < 8; i++)
        {
            int x1 = search_fields_l[i][0];
            int y1 = search_fields_l[i][1];
            int x2 = search_fields_l[(i + 1) & 7][0];
            int y2 = search_fields_l[(i + 1) & 7][1];
            if (!in_bounds(x1, y1) || !in_bounds(x2, y2))
            {
                continue;
            }

            if (bin_image[y1][x1] == 0 && bin_image[y2][x2] == 255)
            {
                temp_l[index_l][0] = (uint8)x1;
                temp_l[index_l][1] = (uint8)y1;
                index_l++;

                if (l_near_top)
                {
                    s_dir_l[l_data_statics - 1] = -1;
                }
                else
                {
                    s_dir_l[l_data_statics - 1] = (int8)i;
                    if (i < 9) s_l_dir_sum[i]++;
                }
            }
        }

        if (index_l)
        {
            center_point_l[0] = temp_l[0][0];
            center_point_l[1] = temp_l[0][1];
            for (uint8 j = 0; j < index_l; j++)
            {
                if (center_point_l[0] > temp_l[j][0])
                {
                    center_point_l[0] = temp_l[j][0];
                    center_point_l[1] = temp_l[j][1];
                }
            }
        }

        bool stuck_r = false;
        bool stuck_l = false;
        if (r_data_statics >= 2)
        {
            stuck_r = (s_points_r[r_data_statics][0] == s_points_r[r_data_statics - 1][0] &&
                       s_points_r[r_data_statics][0] == s_points_r[r_data_statics - 2][0] &&
                       s_points_r[r_data_statics][1] == s_points_r[r_data_statics - 1][1] &&
                       s_points_r[r_data_statics][1] == s_points_r[r_data_statics - 2][1]);
        }
        if (l_data_statics >= 3)
        {
            stuck_l = (s_points_l[l_data_statics - 1][0] == s_points_l[l_data_statics - 2][0] &&
                       s_points_l[l_data_statics - 1][0] == s_points_l[l_data_statics - 3][0] &&
                       s_points_l[l_data_statics - 1][1] == s_points_l[l_data_statics - 2][1] &&
                       s_points_l[l_data_statics - 1][1] == s_points_l[l_data_statics - 3][1]);
        }
        if (stuck_r || stuck_l)
        {
            break;
        }

        if (l_data_statics > 0)
        {
            int dx = zf_abs_i((int)s_points_r[r_data_statics][0] - (int)s_points_l[l_data_statics - 1][0]);
            int dy = zf_abs_i((int)s_points_r[r_data_statics][1] - (int)s_points_l[l_data_statics - 1][1]);
            if (dx < 2 && dy < 2)
            {
                *highest_row = (uint8)((s_points_r[r_data_statics][1] + s_points_l[l_data_statics - 1][1]) >> 1);
                break;
            }
        }
        if (l_data_statics > 0 && s_points_r[r_data_statics][1] < s_points_l[l_data_statics - 1][1])
        {
            continue;
        }

        if (l_data_statics > 0 && s_dir_l[l_data_statics - 1] == 7 &&
            s_points_r[r_data_statics][1] > s_points_l[l_data_statics - 1][1])
        {
            if (record_back_points && s_l_back_cnt < 512)
            {
                s_points_l_back[s_l_back_cnt][0] = s_points_l[l_data_statics - 1][0];
                s_points_l_back[s_l_back_cnt][1] = s_points_l[l_data_statics - 1][1];
                s_l_back_cnt++;
            }
            center_point_l[0] = (uint8)s_points_l[l_data_statics - 1][0];
            center_point_l[1] = (uint8)s_points_l[l_data_statics - 1][1];
            l_data_statics--;
        }

        r_data_statics++;

        index_r = 0;
        const bool r_near_top = (center_point_r[1] < 5);

        for (uint8 i = 0; i < 8; i++)
        {
            int x1 = search_fields_r[i][0];
            int y1 = search_fields_r[i][1];
            int x2 = search_fields_r[(i + 1) & 7][0];
            int y2 = search_fields_r[(i + 1) & 7][1];
            if (!in_bounds(x1, y1) || !in_bounds(x2, y2))
            {
                continue;
            }

            if (bin_image[y1][x1] == 0 && bin_image[y2][x2] == 255)
            {
                temp_r[index_r][0] = (uint8)x1;
                temp_r[index_r][1] = (uint8)y1;
                index_r++;

                if (r_near_top)
                {
                    s_dir_r[r_data_statics - 1] = -1;
                }
                else
                {
                    s_dir_r[r_data_statics - 1] = (int8)i;
                    if (i < 9) s_r_dir_sum[i]++;
                }
            }
        }

        if (index_r)
        {
            center_point_r[0] = temp_r[0][0];
            center_point_r[1] = temp_r[0][1];
            for (uint8 j = 0; j < index_r; j++)
            {
                if (center_point_r[1] > temp_r[j][1])
                {
                    center_point_r[0] = temp_r[j][0];
                    center_point_r[1] = temp_r[j][1];
                }
            }
        }
    }

    *l_count = l_data_statics;
    *r_count = r_data_statics;
    return (l_data_statics > 2 && r_data_statics > 2);
}

void extract_borders(uint16 l_total, uint16 r_total)
{
    for (int i = 0; i < MT9V03X_H; i++)
    {
        s_left_border[i] = -1;
        s_right_border[i] = 200;
    }

    for (uint16 i = 0; i < l_total; i++)
    {
        int row = s_points_l[i][1];
        int col = s_points_l[i][0];
        if (row >= 0 && row < MT9V03X_H && col > s_left_border[row])
        {
            s_left_border[row] = col;
        }
    }

    for (uint16 i = 0; i < r_total; i++)
    {
        int row = s_points_r[i][1];
        int col = s_points_r[i][0];
        if (row >= 0 && row < MT9V03X_H && col < s_right_border[row])
        {
            s_right_border[row] = col;
        }
    }
}

void fill_center_lines(bool generate_binary)
{
    (void)generate_binary;
    int last_l = MT9V03X_W / 4;
    int last_r = MT9V03X_W * 3 / 4;
    for (int row = 0; row < MT9V03X_H; row++)
    {
        int l = s_left_border[row];
        int r = s_right_border[row];

        if (l < 0 && r >= MT9V03X_W)
        {
            l = last_l;
            r = last_r;
        }
        else if (l < 0)
        {
            l = last_l;
        }
        else if (r >= MT9V03X_W)
        {
            r = last_r;
        }

        if (l >= r)
        {
            l = r - 1;
        }

        l = (l * 3 + last_l) >> 2;
        r = (r * 3 + last_r) >> 2;

        last_l = l;
        last_r = r;

        int mid = (l + r) / 2;
        g_left_line[row] = (uint8)l;
        g_right_line[row] = (uint8)r;
        g_center_line[row] = (uint8)mid;
    }
}

void fallback_scanline(uint8 threshold, bool generate_binary)
{
    int last_mid = MT9V03X_W / 2;
    uint8 *p_bottom_row = mt9v03x_image[MT9V03X_H - 1];

    if (p_bottom_row[MT9V03X_W / 2] < threshold)
    {
        if (p_bottom_row[MT9V03X_W / 4] >= threshold)
        {
            last_mid = MT9V03X_W / 4;
        }
        else if (p_bottom_row[MT9V03X_W * 3 / 4] >= threshold)
        {
            last_mid = MT9V03X_W * 3 / 4;
        }
    }

    for (int i = MT9V03X_H - 1; i >= 0; i--)
    {
        uint8 *p_row = mt9v03x_image[i];
        int left_border = 0;
        int right_border = MT9V03X_W - 1;

        for (int j = last_mid; j < MT9V03X_W - 2; j++)
        {
            if (p_row[j] >= threshold && p_row[j + 1] < threshold && p_row[j + 2] < threshold)
            {
                right_border = j;
                break;
            }
        }

        for (int j = last_mid; j >= 2; j--)
        {
            if (p_row[j] >= threshold && p_row[j - 1] < threshold && p_row[j - 2] < threshold)
            {
                left_border = j;
                break;
            }
        }

        int mid = (left_border + right_border) / 2;
        last_mid = mid;

        if (last_mid < 2) last_mid = 2;
        if (last_mid > MT9V03X_W - 3) last_mid = MT9V03X_W - 3;

        g_left_line[i] = (uint8)left_border;
        g_right_line[i] = (uint8)right_border;
        g_center_line[i] = (uint8)mid;
    }
}

/**
 * @brief       图像处理函数（八邻域巡线）
 */
void image_process(uint8 threshold)
{
    bool generate_binary = (g_menu_state == MENU_THRESHOLD_DEBUG);

    if (generate_binary)
    {
        for (int i = 0; i < MT9V03X_H; i++)
        {
            uint8 *src = mt9v03x_image[i];
            uint8 *dst = s_bin_image[i];
            uint8 *dbg = g_image_binary[i];
            for (int j = 0; j < MT9V03X_W; j++)
            {
                uint8 val = (src[j] >= threshold) ? 255 : 0;
                dst[j] = val;
                dbg[j] = val;
            }
        }
    }
    else
    {
        for (int i = 0; i < MT9V03X_H; i++)
        {
            uint8 *src = mt9v03x_image[i];
            uint8 *dst = s_bin_image[i];
            for (int j = 0; j < MT9V03X_W; j++)
            {
                dst[j] = (src[j] >= threshold) ? 255 : 0;
            }
        }
    }
    if (IMAGE_FILTER_ENABLE)
    {
        static uint16 s_filter_frame_cnt = 0;
        if (IMAGE_FILTER_INTERVAL <= 1 || (s_filter_frame_cnt % IMAGE_FILTER_INTERVAL) == 0)
        {
            image_filter_ported(s_bin_image);
        }
        s_filter_frame_cnt++;
    }
    image_draw_rectan_ported(s_bin_image);
    width_change_ported();

    int l_start_x = 0, l_start_y = MT9V03X_H - 2;
    int r_start_x = 0, r_start_y = MT9V03X_H - 2;
    uint16 l_count = 0, r_count = 0;
    uint8 highest_row = 0;

    bool got_start = get_start_points(MT9V03X_H - 2, (const uint8 (*)[MT9V03X_W])s_bin_image, &l_start_x, &l_start_y, &r_start_x, &r_start_y);
    bool ok = false;
    if (got_start)
    {
        ok = search_eight_neighbor(1500, s_bin_image, l_start_x, l_start_y, r_start_x, r_start_y, &l_count, &r_count, &highest_row);
    }

    s_trace_valid = ok ? 1 : 0;
    s_trace_l_total = l_count;
    s_trace_r_total = r_count;

    if (ok)
    {
        extract_borders(l_count, r_count);
    }
    else
    {
        fallback_scanline(threshold, generate_binary);

        for (int row = 0; row < MT9V03X_H; row++)
        {
            s_left_border[row] = (int16)g_left_line[row];
            s_right_border[row] = (int16)g_right_line[row];
        }
    }

    s_data_statics_l = l_count;
    s_data_statics_r = r_count;
    bool meet_happened = (ok && highest_row < (uint8)MT9V03X_H);
    int16 forced_mid_after_meet = (int16)(MT9V03X_W / 2);
    if (meet_happened)
    {
        int ref_mid = MT9V03X_W / 2;
        int start = (int)highest_row;
        if (start < 0) start = 0;
        if (start >= MT9V03X_H) start = MT9V03X_H - 1;

        for (int row = start; row < MT9V03X_H; row++)
        {
            int16 l = s_left_border[row];
            int16 r = s_right_border[row];
            if (l >= 0 && r != 200 && r >= 0 && r < MT9V03X_W && l < r)
            {
                ref_mid = (l + r) / 2;
                break;
            }
            if (l >= 0 && (r == 200 || r >= MT9V03X_W))
            {
                ref_mid = l;
                break;
            }
            if (l < 0 && r != 200 && r >= 0 && r < MT9V03X_W)
            {
                ref_mid = r;
                break;
            }
        }

        if (ref_mid < (MT9V03X_W / 2))
        {
            forced_mid_after_meet = 2;
        }
        else
        {
            forced_mid_after_meet = (int16)(MT9V03X_W - 3);
        }
    }

    int16 last_mid = (int16)(MT9V03X_W / 2);
    for (int row = 0; row < MT9V03X_H; row++)
    {
        int16 l = s_left_border[row];
        int16 r = s_right_border[row];
        if (l < 0 && r == 200)
        {
            if (meet_happened && row < (int)highest_row)
            {
                s_center_line_i16[row] = forced_mid_after_meet;
                last_mid = forced_mid_after_meet;
            }
            else
            {
                s_center_line_i16[row] = last_mid;
            }
        }
        else
        {
            if (l < 0) l = (int16)(last_mid - (int16)(width_from_row(row) / 2));
            if (r == 200) r = (int16)(last_mid + (int16)(width_from_row(row) / 2));
            s_center_line_i16[row] = (l + r) / 2;
            last_mid = s_center_line_i16[row];
        }
    }

    uint32 primask = interrupt_global_disable();
    int32 delta = g_encoder_delta_accum;
    g_encoder_delta_accum = 0;
    interrupt_global_enable(primask);

    if (delta < 0) delta = -delta;
    if (delta > 32767) delta = 32767;
    int16 encoder_data_1 = (int16)delta;
    (void)stop_line_ported(encoder_data_1);
    (void)cross_delay_judge_ported();
    element_test_ported();
    (void)straight_judge_ported();
    (void)straight_judge_2_ported();
    judge_bar_ported();

    if (s_flag_stop || s_flag_bar || s_flag_str || s_flag_str_str)
    {
        s_warn_sink_i16 = (int16)(s_warn_sink_i16 + 1);
    }

    element_handle_ported(encoder_data_1);

    for (int row = 0; row < MT9V03X_H; row++)
    {
        int l = (int)s_left_border[row];
        int r = (int)s_right_border[row];
        int m = (int)s_center_line_i16[row];

        if (l < 0) l = 0;
        if (l >= MT9V03X_W) l = MT9V03X_W - 1;

        if (r == 200) r = MT9V03X_W - 1;
        if (r < 0) r = 0;
        if (r >= MT9V03X_W) r = MT9V03X_W - 1;

        if (m < 0) m = 0;
        if (m >= MT9V03X_W) m = MT9V03X_W - 1;

        g_left_line[row] = (uint8)l;
        g_right_line[row] = (uint8)r;
        g_center_line[row] = (uint8)m;
    }
}




void calculate_line_error(void)
{
    float error_row_1 = g_center_line[g_error_row_1] - (MT9V03X_W / 2);
    float error_row_2 = g_center_line[g_error_row_2] - (MT9V03X_W / 2);
    float error_row_3 = g_center_line[g_error_row_3] - (MT9V03X_W / 2);
    float error_row_4 = g_center_line[g_error_row_4] - (MT9V03X_W / 2);

    g_final_error = g_weight_1 * error_row_1
                  + g_weight_2 * error_row_2
                  + g_weight_3 * error_row_3
                  + g_weight_4 * error_row_4;
}

/**
 * @brief  转向控制函数，计算左右轮的目标速度
 */
void steering_control(void)
{   
    imu660rb_get_gyro();

#if ATTITUDE_USE_ACC_FUSION
    imu660rb_get_acc();
#endif

    // 10ms 周期四元数更新（基于陀螺积分）
    const float dt_s = (float)STEERING_CONTROL_PERIOD / 1000000.0f;
    float gx_dps = imu660rb_gyro_transition(imu660rb_gyro_x);
    float gy_dps = imu660rb_gyro_transition(imu660rb_gyro_y);
    float gz_dps = imu660rb_gyro_transition(imu660rb_gyro_z);

#if ATTITUDE_USE_ACC_FUSION
    float ax_g = imu660rb_acc_transition(imu660rb_acc_x);
    float ay_g = imu660rb_acc_transition(imu660rb_acc_y);
    float az_g = imu660rb_acc_transition(imu660rb_acc_z);
    attitude_quat_update_mahony_imu6_dps_g(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt_s);
#else
    attitude_quat_update_gyro_dps(gx_dps, gy_dps, gz_dps, dt_s);
#endif

    steering_pid_calculate();

    int32_t target_duty = (int32_t)g_servo_center_duty - (int32_t)g_steering_pid.output;
    
    if (target_duty < (int32_t)g_servo_min_duty) target_duty = (int32_t)g_servo_min_duty;
    if (target_duty > (int32_t)g_servo_max_duty) target_duty = (int32_t)g_servo_max_duty;
    
    g_servo_duty = (uint32)target_duty;
    pwm_set_duty(SERVO_PWM_PIN, g_servo_duty);

    float steer_rad = servo_duty_to_steer_rad(g_servo_duty);
    float tan_delta = tanf(steer_rad);
    float geom = (ACKERMANN_TRACK_MM / (2.0f * ACKERMANN_WHEELBASE_MM));
    float scale = geom * tan_delta;
    scale = clampf(scale, -ACKERMANN_SCALE_LIMIT, ACKERMANN_SCALE_LIMIT);

    float base = (float)g_base_speed;
    float v_l = base * (1.0f - scale);
    float v_r = base * (1.0f + scale);

    int16 base_target_speed_l = (int16)v_l;
    int16 base_target_speed_r = (int16)v_r;

    g_target_speed_l = base_target_speed_l;
    g_target_speed_r = base_target_speed_r;

    if (g_target_speed_l < 0) g_target_speed_l = 0;
    else if (g_target_speed_l > g_max_target_speed) g_target_speed_l = g_max_target_speed;
    if (g_target_speed_r < 0) g_target_speed_r = 0;
    else if (g_target_speed_r > g_max_target_speed) g_target_speed_r = g_max_target_speed;
}

static void width_init_if_needed(void)
{
    if (s_width_inited && s_width_last_top == g_track_width_top && s_width_last_bottom == g_track_width_bottom) return;
    for (int y = 0; y < MT9V03X_H; y++)
    {
        float t = (MT9V03X_H <= 1) ? 0.0f : (float)y / (float)(MT9V03X_H - 1);
        float w = (1.0f - t) * (float)g_track_width_top + t * (float)g_track_width_bottom;
        if (w < 10.0f) w = 10.0f;
        if (w > (float)(MT9V03X_W - 4)) w = (float)(MT9V03X_W - 4);
        s_width_begin[y] = (uint8)w;

        int wz = (int)s_width_begin[y] - 10;
        if (wz < 6) wz = 6;
        s_width_zzh[y] = (uint8)wz;
    }
    s_width_last_top = g_track_width_top;
    s_width_last_bottom = g_track_width_bottom;
    s_width_inited = 1;
}

static void width_change_ported(void)
{
    width_init_if_needed();
}

static uint8 width_from_row(int row)
{
    int idx = (MT9V03X_H - 1) - row;
    if (idx < 0) idx = 0;
    if (idx >= MT9V03X_H) idx = MT9V03X_H - 1;
    return s_width_zzh[idx];
}

static float control_kk_ported(float k)
{
    if (k > 5.0f) return 5.0f;
    if (k < 0.1f) return 0.0f;
    return k;
}

static uint8 sum_dir_r_ported(void)
{

    int r120 = MIN(MT9V03X_H - 1, 120);
    int r90  = MIN(MT9V03X_H - 1, 90);
    int r80  = MIN(MT9V03X_H - 1, 80);
    int r60  = MIN(MT9V03X_H - 1, 60);
    int r25  = MIN(MT9V03X_H - 1, 25);
    int r10  = MIN(MT9V03X_H - 1, 10);

    float denom1 = (float)(s_right_border[r120] - s_right_border[r60]);
    float denom2 = (float)(s_right_border[r120] - s_right_border[r10]);
    float denom4 = (float)(s_right_border[r120] - s_right_border[r90]);
    float denom6 = (float)(s_right_border[r80]  - s_right_border[r25]);

    float k1 = (denom1 == 0.0f) ? 0.0f : (float)(r120 - r60) / denom1;
    float k2 = (denom2 == 0.0f) ? 0.0f : (float)(r120 - r10) / denom2;
    float k4 = (denom4 == 0.0f) ? 0.0f : (float)(r120 - r90) / denom4;
    float k6 = (denom6 == 0.0f) ? 0.0f : (float)(r80 - r25) / denom6;

    k1 = control_kk_ported(k1);
    k2 = control_kk_ported(k2);
    k4 = control_kk_ported(k4);
    k6 = control_kk_ported(k6);

    if (k1 >= 2.3f && k1 <= 5.0f && k4 >= 2.3f && k2 >= 2.2f && k2 <= 5.0f && k6 > 1.8f && k6 <= 5.0f)
    {
        return 1;
    }
    return 0;
}

static uint8 sum_dir_l_ported(void)
{
    int r120 = MIN(MT9V03X_H - 1, 120);
    int r90  = MIN(MT9V03X_H - 1, 90);
    int r80  = MIN(MT9V03X_H - 1, 80);
    int r60  = MIN(MT9V03X_H - 1, 60);
    int r25  = MIN(MT9V03X_H - 1, 25);
    int r10  = MIN(MT9V03X_H - 1, 10);

    float denom1 = (float)(s_left_border[r60] - s_left_border[r120]);
    float denom2 = (float)(s_left_border[r10] - s_left_border[r120]);
    float denom4 = (float)(s_left_border[r90] - s_left_border[r120]);
    float denom6 = (float)(s_left_border[r25] - s_left_border[r80]);

    float k1 = (denom1 == 0.0f) ? 0.0f : (float)(r120 - r60) / denom1;
    float k2 = (denom2 == 0.0f) ? 0.0f : (float)(r120 - r10) / denom2;
    float k4 = (denom4 == 0.0f) ? 0.0f : (float)(r120 - r90) / denom4;
    float k6 = (denom6 == 0.0f) ? 0.0f : (float)(80 - 25) / denom6;

    k1 = control_kk_ported(k1);
    k2 = control_kk_ported(k2);
    k4 = control_kk_ported(k4);
    k6 = control_kk_ported(k6);

    if (k1 >= 2.2f && k2 >= 2.2f && k2 <= 5.0f && k4 > 2.2f && k4 <= 5.0f && k6 > 1.9f && k6 <= 5.0f)
    {
        return 1;
    }
    return 0;
}

static uint8 l_l_judge_ported(void)
{
    uint8 sum = 0;
    int right_edge = MT9V03X_W - 4;
    for (int i = 0; i < MT9V03X_H; i++)
    {
        if (s_left_border[i] <= 4 && s_right_border[i] >= right_edge)
        {
            sum++;
        }
    }
    if (sum > 30) return 1;
    if (sum > 10) return 2;
    return 0;
}

static void lose_line_ported(void)
{
    s_r_lose_num = 0;
    s_l_lose_num = 0;
    s_length = 0;

    int right_edge_min = MT9V03X_W - 4;
    int mid_x = MT9V03X_W / 2;

    for (int i = MT9V03X_H - 1; i > 1; i--)
    {
        if (s_left_border[i] < 0 || s_left_border[i] <= 3)
        {
            s_l_lose_judge[i] = 1;
            s_l_lose_num++;
        }
        else
        {
            s_l_lose_judge[i] = 0;
        }
    }

    for (int i = MT9V03X_H - 1; i > 1; i--)
    {
        if (s_right_border[i] == 200 || (s_right_border[i] >= right_edge_min && s_right_border[i] < MT9V03X_W))
        {
            s_r_lose_judge[i] = 1;
            s_r_lose_num++;
        }
        else
        {
            s_r_lose_judge[i] = 0;
        }
    }

    for (int i = MT9V03X_H - 1; i > 0; i--)
    {
        if (s_bin_image[i][mid_x] != 0 && s_bin_image[i - 1][mid_x] != 0)
        {
            s_length++;
        }
    }
}

static void lose_location_test_ported(uint8 type, uint8 startline, uint8 endline)
{
    s_lose_num = 0;
    s_nolose_num = 0;
    if (startline >= MT9V03X_H) startline = (uint8)(MT9V03X_H - 1);
    if (endline >= MT9V03X_H) endline = 0;
    if (startline <= endline)
    {
        s_lose_num = 0;
        s_nolose_num = 0;
        return;
    }

    if (type == 1)
    {
        for (int y = startline; y >= endline; y--)
        {
            s_lose_num += s_l_lose_judge[y];
        }
        s_nolose_num = (uint8)((startline - endline + 1) - s_lose_num);
    }
    else if (type == 2)
    {
        for (int y = startline; y >= endline; y--)
        {
            s_lose_num += s_r_lose_judge[y];
        }
        s_nolose_num = (uint8)((startline - endline + 1) - s_lose_num);
    }
}

static void cir_point_ported(int flag)
{
    if (flag == 1)
    {
        for (int i = (int)s_data_statics_l - 1; i > 1; i--)
        {
            if (s_dir_l[i - 1] == 6 && s_points_l[i][1] > 10)
            {
                s_cir_x = (uint8)s_points_l[i - 1][0];
                s_cir_y = (uint8)s_points_l[i - 1][1];
                break;
            }
        }
    }
    else
    {
        for (int i = (int)s_data_statics_r - 1; i > 1; i--)
        {
            if (s_dir_r[i - 1] == 6 && s_points_r[i][1] > 10)
            {
                s_cir_x = (uint8)s_points_r[i - 1][0];
                s_cir_y = (uint8)s_points_r[i - 1][1];
                break;
            }
        }
    }
}

static bool cir1_judge_ported(void)
{
    lose_location_test_ported(2, 20, 0);
    int length_th = MT9V03X_H - 6;
    if (length_th < 0) length_th = 0;
    if (s_cir_stage == 0 && s_nolose_num > 10 && sum_dir_l_ported() && s_l_lose_num < 40 && s_r_lose_num > 50 && s_length > length_th && s_r_dir_sum[4] > 50 && s_l_dir_sum[4] > 65)
    {
        lose_location_test_ported(1, (uint8)(MT9V03X_H - 1), (uint8)MAX(0, MT9V03X_H - 18));
        if (s_nolose_num > 13)
        {
            s_cir_stage = 1;
            return true;
        }
    }
    return false;
}

static bool cir2_judge_ported(void)
{
    lose_location_test_ported(1, 20, 0);
    int length_th = MT9V03X_H - 6;
    if (length_th < 0) length_th = 0;
    if (s_cir_stage == 0 && s_nolose_num > 10 && sum_dir_r_ported() && s_r_lose_num < 50 && s_l_lose_num > 40 && s_length > length_th && s_l_dir_sum[4] > 50 && s_r_dir_sum[4] > 65)
    {
        lose_location_test_ported(1, (uint8)(MT9V03X_H - 1), (uint8)MAX(0, MT9V03X_H - 18));
        if (s_nolose_num > 13)
        {
            s_cir_stage = 1;
            return true;
        }
    }
    return false;
}

static void l_border_cross_ported(uint16 total_l)
{
    for (int h = 0; h < MT9V03X_H; h++)
    {
        s_left_border[h] = -1;
    }

    if (total_l < 3) return;

    if (s_flag_cross == 1)
    {
        int h2 = 0;
        uint8 count = 0;
        for (int i = (int)total_l - 1; i > 0; i--)
        {
            int h = (int)s_points_l[i][1];
            if (h2 > h)
            {
                continue;
            }
            h2 = 0;

            bool trigger = (s_dir_l[i] == 6 && (i + 1) < (int)total_l && s_dir_l[i + 1] != 6 && (i - 2) > 1 && h > 3);
            if (trigger)
            {
                while (count < 100 && (h + count) < MT9V03X_H)
                {
                    int x = (int)s_points_l[i][0] - (int)((float)count / 4.0f);
                    if (x < 2) x = 2;
                    s_left_border[h + count] = x;
                    count++;
                }
                h2 = h + count - 1;
                count = 0;
            }
            else
            {
                int x = (int)s_points_l[i][0];
                if (x > s_left_border[h]) s_left_border[h] = x;
            }
        }
    }
    else if (s_flag_cross == 2)
    {
        int x1 = 0;
        int x2 = 0;
        for (int i = 1; i < (int)total_l - 1; i++)
        {
            if (s_dir_l[i] == 2 && s_dir_l[i - 1] == 4 && s_dir_l[i + 1] == 2)
            {
                x1 = (int)s_points_l[i][0];
                x2 = i;
                break;
            }
        }
        for (int i = 24; i < MT9V03X_H; i++)
        {
            s_left_border[i] = x1 - (i - x2) / 4 - 10;
        }
    }
}

static void r_border_cross_ported(uint16 total_r)
{
    for (int h = 0; h < MT9V03X_H; h++)
    {
        s_right_border[h] = 200;
    }

    if (total_r < 3) return;

    if (s_flag_cross == 1)
    {
        int h2 = 0;
        uint8 count = 0;
        for (int i = (int)total_r - 1; i > 2; i--)
        {
            int h = (int)s_points_r[i][1];
            if (h2 > h)
            {
                continue;
            }
            h2 = 0;

            bool trigger = (s_dir_r[i - 2] == 6 && s_dir_r[i - 1] == 6 && s_dir_r[i] == 6 && (i + 1) < (int)total_r && s_dir_r[i + 1] != 6 && h > 3);
            if (trigger)
            {
                while (count < 100 && (h + count) < MT9V03X_H)
                {
                    int x = (int)s_points_r[i][0] + (int)((float)count / 4.0f);
                    if (x > (MT9V03X_W - 2)) x = MT9V03X_W - 2;
                    s_right_border[h + count] = x;
                    count++;
                }
                h2 = h + count - 1;
                count = 0;
            }
            else
            {
                int x = (int)s_points_r[i][0];
                if (x < s_right_border[h]) s_right_border[h] = x;
            }
        }
    }
    else if (s_flag_cross == 2)
    {
        int x1 = 0;
        int x2 = 0;
        for (int i = 1; i < (int)total_r - 1; i++)
        {
            if (s_dir_r[i] == 2 && s_dir_r[i + 1] == 2 && (s_dir_r[i - 1] == 4 || s_dir_r[i - 1] == 5) && s_points_r[i][0] > (MT9V03X_W / 2))
            {
                x1 = (int)s_points_r[i][0];
                x2 = i;
                break;
            }
        }
        s_cir_x = (uint8)x1;
        s_cir_y = (uint8)MIN(MT9V03X_H - 1, (int)s_points_r[x2][1]);

        if (x1 < (MT9V03X_W / 2))
        {
            for (int i = 24; i < MT9V03X_H; i++)
            {
                s_right_border[i] = s_left_border[i] + width_from_row(i);
            }
        }
        else
        {
            for (int i = 24; i < MT9V03X_H; i++)
            {
                s_right_border[i] = x1 + (i - x2) / 4;
            }
        }
    }
}

static uint8 cross_judge_ported(void)
{
    uint8 num1 = 0;
    uint8 num2 = 0;
    uint8 num3 = 0;
    uint8 num4 = 0;

    uint8 ll = l_l_judge_ported();
    if (ll == 1 && s_length > 110)
    {
        for (uint16 i = 0; i < s_data_statics_l; i++)
        {
            if (i < s_data_statics_r && s_dir_r[i] == 6) num1++;
            if (s_dir_l[i] == 6) num2++;
        }
        if (num2 > 5 && num1 > 5 && s_r_dir_sum[4] > 20 && s_l_dir_sum[4] > 20)
        {
            return 1;
        }
    }
    if (ll == 2 && s_length > 110)
    {
        for (uint16 i = 0; i < s_data_statics_l; i++)
        {
            if (i < s_data_statics_r && s_dir_r[i] == 2) num4++;
            if (s_dir_l[i] == 2) num3++;
        }
        if (num3 > 5 && num4 > 4 && s_r_dir_sum[4] > 10 && s_l_dir_sum[4] > 10)
        {
            return 2;
        }
    }
    return 0;
}

static uint8 my_sqrt_u8_ported(int16 x)
{
    uint8 ans = 0;
    uint8 p = 0x80;
    while (p != 0)
    {
        ans = (uint8)(ans + p);
        if ((int)ans * (int)ans > x)
        {
            ans = (uint8)(ans - p);
        }
        p = (uint8)(p / 2);
    }
    return ans;
}

static float process_curvity_ported(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3)
{
    int s_abc = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    int16 q1 = (int16)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    uint8 ab = my_sqrt_u8_ported(q1);
    q1 = (int16)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    uint8 bc = my_sqrt_u8_ported(q1);
    q1 = (int16)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    uint8 ac = my_sqrt_u8_ported(q1);
    if ((int)ab * (int)bc * (int)ac == 0)
    {
        return 0.0f;
    }
    return (float)4.0f * (float)s_abc / (float)((int)ab * (int)bc * (int)ac);
}

static bool stop_line_ported(int16 encoder_data_1)
{
    int count_line = 0;
    int row = MIN(MT9V03X_H - 1, 110);
    for (int i = 5; i < MIN(MT9V03X_W - 4, 115); i++)
    {
        if (s_bin_image[row][i] != 0 && s_bin_image[row][i + 1] == 0 && s_bin_image[row][i + 2] == 0 && s_bin_image[row][i + 3] == 0)
        {
            count_line++;
        }
    }

    if (s_encoder_sum_stop < 3000 && s_stop_arm == 0)
    {
        s_encoder_sum_stop += encoder_data_1;
    }
    if (s_encoder_sum_stop > 3000)
    {
        s_stop_arm = 1;
        s_encoder_sum_stop = 0;
    }

    if (count_line >= 6 && s_stop_arm == 1)
    {
        s_flag_stop = 1;
    }
    else
    {
        s_flag_stop = 0;
    }

    if (s_flag_stop == 1 && s_stop_line_time == 0)
    {
        s_stop_line_time = 1;
    }
    if (s_stop_line_time != 0 && s_stop_line_time <= 18)
    {
        s_stop_line_time++;
        if (s_stop_line_time == 18)
        {
        }
    }
    if (s_stop_line_time > 18)
    {
        s_stop_line_time = 0;
    }

    return s_flag_stop ? true : false;
}

static int cross_delay_judge_ported(void)
{
    if (s_cross_jud == 0 && (s_flag_cross == 1 || s_flag_cross == 2)) s_cross_jud = 1;
    if (s_cross_jud == 1 && s_flag_cross == 0 && s_flag_others2 == 1) s_cross_jud = 2;
    if (s_cross_jud == 2 && s_flag_others2 == 1)
    {
        s_cross_jud_time++;
        if (s_cross_jud_time >= 10)
        {
            s_cross_jud = 0;
            s_cross_jud_time = 0;
        }
    }
    if (s_cross_jud == 2 && s_flag_others2 == 0)
    {
        s_cross_jud = 0;
        s_cross_jud_time = 0;
    }
    return s_cross_jud;
}

static void judge_bar_ported(void)
{
    s_flag_bar = 0;
    s_bar_count_l = 0;
    s_bar_count_r = 0;

    if (s_length >= 125 && s_flag_cross != 1 && s_flag_cross != 2 && s_flag_l_cir != 1 && s_flag_r_cir != 1 && s_flag_stop != 1)
    {
        for (int i = 10; i < MIN(MT9V03X_H - 4, 110); i++)
        {
            if (zf_abs_i(s_right_border[i] - s_right_border[i + 3]) > 5) s_bar_count_r++;
            if (zf_abs_i(s_left_border[i] - s_left_border[i + 3]) > 5) s_bar_count_l++;
        }
        if (s_bar_count_l >= 6) s_flag_bar = 1;
        if (s_bar_count_r >= 7) s_flag_bar = 2;
    }
}

static void image_filter_ported(uint8 img[MT9V03X_H][MT9V03X_W])
{
    int row_start = IMAGE_FILTER_ROW_START;
    int row_end = IMAGE_FILTER_ROW_END;
    if (row_start < 1) row_start = 1;
    if (row_end > (MT9V03X_H - 1)) row_end = (MT9V03X_H - 1);
    if (row_end <= row_start) return;

    for (int i = row_start; i < row_end; i++)
    {
        uint8 *row_m1 = img[i - 1];
        uint8 *row_0  = img[i];
        uint8 *row_p1 = img[i + 1];

        for (int j = 1; j < MT9V03X_W - 1; j++)
        {
            int white_cnt = 0;
            white_cnt += (row_m1[j - 1] != 0);
            white_cnt += (row_m1[j]     != 0);
            white_cnt += (row_m1[j + 1] != 0);
            white_cnt += (row_0[j - 1]  != 0);
            white_cnt += (row_0[j + 1]  != 0);
            white_cnt += (row_p1[j - 1] != 0);
            white_cnt += (row_p1[j]     != 0);
            white_cnt += (row_p1[j + 1] != 0);

            uint8 center = row_0[j];
            if (white_cnt >= 6 && center == 0)
            {
                row_0[j] = 255;
            }
            else if (white_cnt <= 2 && center != 0)
            {
                row_0[j] = 0;
            }
        }
    }
}

static void image_draw_rectan_ported(uint8 img[MT9V03X_H][MT9V03X_W])
{
    for (int i = 0; i < MT9V03X_H; i++)
    {
        img[i][0] = 0;
        img[i][MT9V03X_W - 1] = 0;
        if (MT9V03X_W > 3)
        {
            img[i][1] = 0;
            img[i][MT9V03X_W - 2] = 0;
            img[i][2] = 0;
            img[i][MT9V03X_W - 3] = 0;
        }
    }
    for (int j = 0; j < MT9V03X_W; j++)
    {
        img[0][j] = 0;
        img[MT9V03X_H - 1][j] = 0;
    }
}

static void cir_r_handle_ported(int16 encoder_data_1)
{
    switch (s_cir_stage)
    {
        case 1:
            s_encoder_sum += encoder_data_1;
            lose_location_test_ported(2, 100, 20);
            if (s_lose_num > 10)
            {
                lose_location_test_ported(2, 40, 0);
                if (s_nolose_num >= 10)
                {
                    s_cir_stage = 2;
                    s_encoder_sum = 0;
                }
                lose_location_test_ported(2, 30, 0);
                if (s_lose_num >= 10)
                {
                    s_encoder_sum = 0;
                    s_cir_stage = 2;
                }
            }
            if (s_encoder_sum > s_max_encoder)
            {
                s_flag_r_cir = 0;
                s_cir_stage = 0;
                s_encoder_sum = 0;
            }
            break;
        case 2:
            s_encoder_sum += encoder_data_1;
            lose_location_test_ported(2, 60, 30);
            if (s_nolose_num > 15)
            {
                lose_location_test_ported(2, 30, 0);
                if (s_lose_num > 15)
                {
                    s_cir_stage = 3;
                    s_encoder_sum = 0;
                }
            }
            if (s_encoder_sum > s_max_encoder)
            {
                s_flag_r_cir = 0;
                s_cir_stage = 0;
                s_encoder_sum = 0;
            }
            break;
        case 3:
            s_encoder_sum += encoder_data_1;
            lose_location_test_ported(2, 60, 30);
            if (s_lose_num > 15)
            {
                lose_location_test_ported(2, 30, 0);
                if (s_nolose_num > 25)
                {
                    s_encoder_sum = 0;
                    s_cir_stage = 4;
                }
            }
            if (s_encoder_sum > s_max_encoder)
            {
                s_flag_r_cir = 0;
                s_cir_stage = 0;
                s_encoder_sum = 0;
            }
            break;
        case 4:
            s_encoder_sum += encoder_data_1;
            if (s_encoder_sum > s_encoder_sum_r4)
            {
                s_cir_stage = 5;
                s_encoder_sum = 0;
            }
            break;
        case 5:
            s_encoder_sum += encoder_data_1;
            if (s_encoder_sum > 1300)
            {
                s_encoder_sum = 0;
                s_cir_stage = 6;
            }
            break;
        case 6:
            lose_location_test_ported(2, 75, 25);
            if (s_lose_num > 20)
            {
                lose_location_test_ported(1, 75, 25);
                if (s_lose_num > 20)
                {
                    s_encoder_sum = 0;
                    s_cir_stage = 7;
                }
            }
            break;
        case 7:
            s_encoder_sum += encoder_data_1;
            if (s_encoder_sum > s_encoder_sum_r7)
            {
                s_cir_stage = 8;
                s_encoder_sum = 0;
            }
            break;
        case 8:
            lose_location_test_ported(2, 20, 0);
            if (s_nolose_num > 15)
            {
                lose_location_test_ported(1, 60, 0);
                if (s_nolose_num > 50)
                {
                    s_cir_stage = 9;
                }
            }
            break;
        case 9:
            s_encoder_sum += encoder_data_1;
            if (s_encoder_sum > 1000)
            {
                s_flag_r_cir = 0;
                s_cir_stage = 0;
                s_encoder_sum = 0;
            }
            break;
        default:
            break;
    }

    if (s_cir_stage == 1)
    {
        for (int i = 0; i < MT9V03X_H; i++)
        {
            s_left_border[i] = s_left_border[i] + 25;
            s_right_border[i] = s_left_border[i] + width_from_row(i) + 20;
        }
    }
    else if (s_cir_stage == 2 || s_cir_stage == 3)
    {
        cir_point_ported(2);
        for (int i = 0; i < MT9V03X_H; i++)
        {
            s_left_border[i] = s_left_border[i] + 4;
            s_right_border[i] = s_left_border[i] + width_from_row(i) + 25;
        }
    }
    else if (s_cir_stage == 4)
    {
        cir_point_ported(2);
        float count = 0.0f;
        for (int i = s_cir_y; i < MT9V03X_H; i++)
        {
            s_right_border[i] = MT9V03X_W - 5;
            s_left_border[i] = (int16)((float)s_cir_x - count);
            count += 1.0f;
        }
        count = 0.0f;
        for (int i = s_cir_y; i > 10; i--)
        {
            s_right_border[i] = MT9V03X_W - 5;
            s_left_border[i] = (int16)((float)s_cir_x + count);
            count += 1.0f;
        }
    }
    else if (s_cir_stage == 7)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            s_right_border[i] = MT9V03X_W - 5;
            s_left_border[i] = s_right_border[i] - width_from_row(i) - 30;
        }
    }
    else if (s_cir_stage == 9)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            s_left_border[i] = s_left_border[i] + 5;
            s_right_border[i] = s_left_border[i] + width_from_row(i);
        }
    }
}

static void cir_l_handle_ported(int16 encoder_data_1)
{
    switch (s_cir_stage)
    {
        case 1:
            s_encoder_sum += encoder_data_1;
            lose_location_test_ported(1, 50, 20);
            if (s_lose_num > 15)
            {
                lose_location_test_ported(1, 20, 0);
                if (s_nolose_num >= 10)
                {
                    s_cir_stage = 2;
                    s_encoder_sum = 0;
                }
            }
            if (s_encoder_sum > s_max_encoder)
            {
                s_flag_r_cir = 0;
                s_cir_stage = 0;
                s_encoder_sum = 0;
            }
            break;
        case 2:
            s_encoder_sum += encoder_data_1;
            lose_location_test_ported(1, 40, 20);
            if (s_nolose_num > 15)
            {
                lose_location_test_ported(1, 20, 0);
                if (s_lose_num > 15)
                {
                    s_cir_stage = 3;
                    s_encoder_sum = 0;
                }
            }
            if (s_encoder_sum > s_max_encoder)
            {
                s_flag_r_cir = 0;
                s_cir_stage = 0;
                s_encoder_sum = 0;
            }
            break;
        case 3:
            s_encoder_sum += encoder_data_1;
            lose_location_test_ported(1, 70, 30);
            if (s_lose_num > 15)
            {
                lose_location_test_ported(1, 40, 0);
                if (s_nolose_num > 15)
                {
                    s_cir_stage = 4;
                    s_encoder_sum = 0;
                }
            }
            if (s_encoder_sum > s_max_encoder)
            {
                s_flag_r_cir = 0;
                s_cir_stage = 0;
                s_encoder_sum = 0;
            }
            break;
        case 4:
            s_encoder_sum += encoder_data_1;
            if (s_encoder_sum > s_encoder_sum_l4)
            {
                s_cir_stage = 5;
                s_encoder_sum = 0;
            }
            break;
        case 5:
            s_encoder_sum += encoder_data_1;
            if (s_encoder_sum > 500)
            {
                s_encoder_sum = 0;
                s_cir_stage = 6;
            }
            break;
        case 6:
            lose_location_test_ported(1, 89, 5);
            if (s_lose_num > 10)
            {
                lose_location_test_ported(2, 89, 25);
                if (s_lose_num > 10)
                {
                    s_encoder_sum = 0;
                    s_cir_stage = 7;
                }
            }
            break;
        case 7:
            s_encoder_sum += encoder_data_1;
            if (s_encoder_sum > s_encoder_sum_l7)
            {
                s_cir_stage = 8;
                s_encoder_sum = 0;
            }
            break;
        case 8:
            lose_location_test_ported(1, 20, 0);
            if (s_nolose_num > 10)
            {
                lose_location_test_ported(2, 60, 10);
                if (s_nolose_num > 45)
                {
                    s_cir_stage = 9;
                }
            }
            break;
        case 9:
            s_encoder_sum += encoder_data_1;
            if (s_encoder_sum > 1000)
            {
                s_flag_l_cir = 0;
                s_cir_stage = 0;
                s_encoder_sum = 0;
            }
            break;
        default:
            break;
    }

    if (s_cir_stage == 1)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            s_right_border[i] = s_right_border[i] - 1;
            s_left_border[i] = s_right_border[i] - width_from_row(i) - 30;
        }
    }
    else if (s_cir_stage == 2 || s_cir_stage == 3)
    {
        cir_point_ported(1);
        for (int i = 0; i < MT9V03X_H; i++)
        {
            s_right_border[i] = s_right_border[i] - 1;
            s_left_border[i] = s_right_border[i] - width_from_row(i) - 30;
        }
    }
    else if (s_cir_stage == 4)
    {
        for (int i = 5; i < MT9V03X_H - 11; i++)
        {
            s_left_border[i] = 5;
            s_right_border[i] = s_left_border[i] + width_from_row(i) + 15;
        }
    }
    else if (s_cir_stage == 7)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            s_right_border[i] = s_left_border[i] + width_from_row(i) + 5;
        }
    }
    else if (s_cir_stage == 9)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            s_right_border[i] = s_right_border[i] - 1;
            s_left_border[i] = s_right_border[i] - width_from_row(i) - 5;
        }
    }
}

static void element_handle_ported(int16 encoder_data_1)
{
    s_black_jiexian = 0;

    if (s_flag_r_cir == 1)
    {
        cir_r_handle_ported(encoder_data_1);
        for (int i = 0; i < MT9V03X_H; i++)
        {
            s_center_line_i16[i] = (s_left_border[i] + s_right_border[i]) / 2;
        }
        if (s_cir_stage == 7 && MT9V03X_H >= 2)
        {
            s_center_line_i16[MT9V03X_H - 1] = s_center_line_i16[MT9V03X_H - 2];
        }
        return;
    }

    if (s_flag_l_cir == 1)
    {
        cir_l_handle_ported(encoder_data_1);
        for (int i = 0; i < MT9V03X_H; i++)
        {
            s_center_line_i16[i] = (s_left_border[i] + s_right_border[i]) / 2;
        }
        if (s_cir_stage == 7 && MT9V03X_H >= 2)
        {
            s_center_line_i16[MT9V03X_H - 1] = s_center_line_i16[MT9V03X_H - 2];
        }
        return;
    }

    if (s_flag_cross == 1)
    {
        l_border_cross_ported(s_data_statics_l);
        r_border_cross_ported(s_data_statics_r);
        for (int i = 0; i < MT9V03X_H; i++)
        {
            s_center_line_i16[i] = (s_left_border[i] + s_right_border[i]) / 2;
        }
        return;
    }


    for (int i = MT9V03X_H - 1; i >= 1; i--)
    {
        if (s_black_jiexian == 0)
        {
            if (s_left_border[i] == -1 || s_right_border[i] == 200)
            {
                s_black_jiexian = i;
            }
        }
    }
}

static void element_test_ported(void)
{
    int y20 = MAX(0, MT9V03X_H - 20);
    int y35 = MAX(0, MT9V03X_H - 35);
    int y50 = MAX(0, MT9V03X_H - 50);

    uint8 lx1 = (uint8)MAX(0, MIN(MT9V03X_W - 1, s_left_border[y20]));
    uint8 lx2 = (uint8)MAX(0, MIN(MT9V03X_W - 1, s_left_border[y35]));
    uint8 lx3 = (uint8)MAX(0, MIN(MT9V03X_W - 1, s_left_border[y50]));
    uint8 rx1 = (uint8)MAX(0, MIN(MT9V03X_W - 1, s_right_border[y20]));
    uint8 rx2 = (uint8)MAX(0, MIN(MT9V03X_W - 1, s_right_border[y35]));
    uint8 rx3 = (uint8)MAX(0, MIN(MT9V03X_W - 1, s_right_border[y50]));

    s_l_line_qulv = (int16)(1000.0f * process_curvity_ported(lx1, (uint8)y20, lx2, (uint8)y35, lx3, (uint8)y50));
    s_r_line_qulv = (int16)(1000.0f * process_curvity_ported(rx1, (uint8)y20, rx2, (uint8)y35, rx3, (uint8)y50));

    s_warn_sink_i16 = (int16)(s_l_line_qulv + s_r_line_qulv);

    lose_line_ported();

    if (cir1_judge_ported() && s_flag_cross != 1)
    {
        s_flag_r_cir = 1;
        s_flag_cross = 0;
        s_flag_others2 = 0;
    }
    else if (cir2_judge_ported() && s_flag_cross != 1 && s_flag_r_cir != 1)
    {
        s_flag_l_cir = 1;
        s_flag_cross = 0;
        s_flag_others2 = 0;
    }
    else
    {
        uint8 cj = cross_judge_ported();
        if (cj == 1)
        {
            s_flag_cross = 1;
            s_flag_others2 = 0;
        }
        else if (cj == 2)
        {
            s_flag_cross = 0;
            s_flag_others2 = 0;
        }
        else
        {
            s_flag_cross = 0;
            s_flag_others2 = 1;
        }
    }
}

static uint8 straight_judge_ported(void)
{
    if (s_length > 115 && s_flag_l_cir != 1 && s_flag_r_cir != 1)
    {
        for (int i = 13; i < 20; i++)
        {
            int i2 = i + 50;
            if (i2 >= MT9V03X_H) break;
            int distance = zf_abs_i(s_center_line_i16[i] - s_center_line_i16[i2]);
            if (distance > 5)
            {
                s_flag_str = false;
                return 0;
            }
        }
        s_flag_str = true;
        return 1;
    }
    s_flag_str = false;
    return 0;
}

static uint8 straight_judge_2_ported(void)
{
    if (s_length > 115 && s_flag_l_cir != 1 && s_flag_r_cir != 1)
    {
        for (int i = 13; i < 20; i++)
        {
            int i2 = i + 50;
            if (i2 >= MT9V03X_H) break;
            int distance = zf_abs_i(s_center_line_i16[i] - s_center_line_i16[i2]);
            if (distance > 2)
            {
                s_flag_str_str = false;
                return 0;
            }
        }
        s_flag_str_str = true;
        return 1;
    }
    s_flag_str_str = false;
    return 0;
}


/**
 * @brief  仅控制舵机转向的函数，不控制电机
 */
void steering_control_servo_only(void)
{
    imu660rb_get_gyro();
    steering_pid_calculate();
    // 用时间窗做“停住保持”，避免只在识别那一帧置位
    g_stop_active = (s_stop_line_time != 0);

    int32_t target_duty = (int32_t)g_servo_center_duty - (int32_t)g_steering_pid.output;
    
    if (target_duty < (int32_t)g_servo_min_duty) target_duty = (int32_t)g_servo_min_duty;
    if (target_duty > (int32_t)g_servo_max_duty) target_duty = (int32_t)g_servo_max_duty;
        if (g_stop_active)
        {
            g_target_speed_l = 0;
            g_target_speed_r = 0;
        }
    
    g_servo_duty = (uint32)target_duty;
    pwm_set_duty(SERVO_PWM_PIN, g_servo_duty);
}

/**
 * @brief       检查赛道是否丢失
 */
bool check_track_lost(void)
{
    const int rows_to_check = 5;
    const float black_pixel_threshold = 0.80f;

    int black_pixel_count = 0;
    int total_pixels_checked = 0;

    for(int i = MT9V03X_H - 1; i >= MT9V03X_H - rows_to_check; i--)
    {
        uint8 *p_src_row = mt9v03x_image[i];
        
        for(int j = 0; j < MT9V03X_W; j++)
        {
            if(p_src_row[j] < g_dynamic_threshold)
            {
                black_pixel_count++;
            }
        }
        total_pixels_checked += MT9V03X_W;
    }


    float black_ratio = (float)black_pixel_count / (float)total_pixels_checked;

    if(black_ratio > black_pixel_threshold)
    {
        return true;
    }

    return false;
}



// ----------------- IMU 四元数姿态解算（陀螺积分）实现 -----------------
static float inv_sqrtf_safe(float x)
{
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}

static void attitude_quat_normalize(void)
{
    float q0 = g_att_q0;
    float q1 = g_att_q1;
    float q2 = g_att_q2;
    float q3 = g_att_q3;
    float inv = inv_sqrtf_safe(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (inv > 0.0f)
    {
        g_att_q0 = q0 * inv;
        g_att_q1 = q1 * inv;
        g_att_q2 = q2 * inv;
        g_att_q3 = q3 * inv;
    }
    else
    {
        g_att_q0 = 1.0f;
        g_att_q1 = 0.0f;
        g_att_q2 = 0.0f;
        g_att_q3 = 0.0f;
    }
}

static void attitude_quat_reset(void)
{
    g_att_q0 = 1.0f;
    g_att_q1 = 0.0f;
    g_att_q2 = 0.0f;
    g_att_q3 = 0.0f;

    s_att_int_x = 0.0f;
    s_att_int_y = 0.0f;
    s_att_int_z = 0.0f;

    s_acc_filt_x = 0.0f;
    s_acc_filt_y = 0.0f;
    s_acc_filt_z = 0.0f;
}

static void attitude_gyro_bias_calibrate(uint16 samples)
{
    if (samples == 0) return;

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    for (uint16 i = 0; i < samples; i++)
    {
        imu660rb_get_gyro();
        sum_x += (double)imu660rb_gyro_transition(imu660rb_gyro_x);
        sum_y += (double)imu660rb_gyro_transition(imu660rb_gyro_y);
        sum_z += (double)imu660rb_gyro_transition(imu660rb_gyro_z);
        system_delay_ms(2);
    }

    s_gyro_bias_x_dps = (float)(sum_x / (double)samples);
    s_gyro_bias_y_dps = (float)(sum_y / (double)samples);
    s_gyro_bias_z_dps = (float)(sum_z / (double)samples);
}

static void attitude_quat_update_gyro_dps(float gx_dps, float gy_dps, float gz_dps, float dt_s)
{
    if (dt_s <= 0.0f) return;

    gx_dps -= s_gyro_bias_x_dps;
    gy_dps -= s_gyro_bias_y_dps;
    gz_dps -= s_gyro_bias_z_dps;

    const float deg2rad = 3.14159265358979323846f / 180.0f;
    float wx = gx_dps * deg2rad;
    float wy = gy_dps * deg2rad;
    float wz = gz_dps * deg2rad;

    float q0 = g_att_q0;
    float q1 = g_att_q1;
    float q2 = g_att_q2;
    float q3 = g_att_q3;

    float half_dt = 0.5f * dt_s;
    float dq0 = (-q1 * wx - q2 * wy - q3 * wz) * half_dt;
    float dq1 = ( q0 * wx + q2 * wz - q3 * wy) * half_dt;
    float dq2 = ( q0 * wy - q1 * wz + q3 * wx) * half_dt;
    float dq3 = ( q0 * wz + q1 * wy - q2 * wx) * half_dt;

    g_att_q0 = q0 + dq0;
    g_att_q1 = q1 + dq1;
    g_att_q2 = q2 + dq2;
    g_att_q3 = q3 + dq3;

    attitude_quat_normalize();
}

static void attitude_quat_update_mahony_imu6_dps_g(float gx_dps, float gy_dps, float gz_dps,
                                                   float ax_g, float ay_g, float az_g,
                                                   float dt_s)
{
    if (dt_s <= 0.0f) return;

    // 去零偏（来自开机静止校准）
    gx_dps -= s_gyro_bias_x_dps;
    gy_dps -= s_gyro_bias_y_dps;
    gz_dps -= s_gyro_bias_z_dps;

    // 加速度计简单低通，抑制震动
    const float a = (ATTITUDE_ACC_LPF_ALPHA < 0.0f) ? 0.0f : ((ATTITUDE_ACC_LPF_ALPHA > 1.0f) ? 1.0f : ATTITUDE_ACC_LPF_ALPHA);
    if (a > 0.0f)
    {
        s_acc_filt_x += a * (ax_g - s_acc_filt_x);
        s_acc_filt_y += a * (ay_g - s_acc_filt_y);
        s_acc_filt_z += a * (az_g - s_acc_filt_z);
        ax_g = s_acc_filt_x;
        ay_g = s_acc_filt_y;
        az_g = s_acc_filt_z;
    }

    // 归一化加速度向量
    float an2 = ax_g*ax_g + ay_g*ay_g + az_g*az_g;
    if (an2 <= 1e-12f)
    {
        attitude_quat_update_gyro_dps(gx_dps, gy_dps, gz_dps, dt_s);
        return;
    }
    float inv_an = inv_sqrtf_safe(an2);
    ax_g *= inv_an;
    ay_g *= inv_an;
    az_g *= inv_an;

    // 当前四元数
    float q0 = g_att_q0;
    float q1 = g_att_q1;
    float q2 = g_att_q2;
    float q3 = g_att_q3;

    // 估计重力方向（机体系）
    // v = [2(q1q3 - q0q2), 2(q0q1 + q2q3), q0^2 - q1^2 - q2^2 + q3^2]
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = (q0*q0 - q1*q1 - q2*q2 + q3*q3);

    // 误差：e = v × a
    float ex = (vy * az_g - vz * ay_g);
    float ey = (vz * ax_g - vx * az_g);
    float ez = (vx * ay_g - vy * ax_g);

    // 积分项（用于估计陀螺零漂的缓慢变化）
    const float ki = ATTITUDE_MAHONY_KI;
    if (ki > 0.0f)
    {
        s_att_int_x += ki * ex * dt_s;
        s_att_int_y += ki * ey * dt_s;
        s_att_int_z += ki * ez * dt_s;
    }
    else
    {
        s_att_int_x = 0.0f;
        s_att_int_y = 0.0f;
        s_att_int_z = 0.0f;
    }

    // 反馈校正后的角速度（dps）
    const float kp = ATTITUDE_MAHONY_KP;
    gx_dps += kp * ex + s_att_int_x;
    gy_dps += kp * ey + s_att_int_y;
    gz_dps += kp * ez + s_att_int_z;

    // 积分四元数
    const float deg2rad = 3.14159265358979323846f / 180.0f;
    float wx = gx_dps * deg2rad;
    float wy = gy_dps * deg2rad;
    float wz = gz_dps * deg2rad;

    float half_dt = 0.5f * dt_s;
    float dq0 = (-q1 * wx - q2 * wy - q3 * wz) * half_dt;
    float dq1 = ( q0 * wx + q2 * wz - q3 * wy) * half_dt;
    float dq2 = ( q0 * wy - q1 * wz + q3 * wx) * half_dt;
    float dq3 = ( q0 * wz + q1 * wy - q2 * wx) * half_dt;

    g_att_q0 = q0 + dq0;
    g_att_q1 = q1 + dq1;
    g_att_q2 = q2 + dq2;
    g_att_q3 = q3 + dq3;

    attitude_quat_normalize();
}

void params_save_to_flash(void)
{
    flash_buffer_clear(); 
    flash_union_buffer[0].uint32_type = PARAMS_FLASH_MAGIC;
    flash_union_buffer[1].float_type  = g_steering_pid.Kp;
    flash_union_buffer[2].float_type  = g_steering_pid.Ki;
    flash_union_buffer[3].float_type  = g_steering_pid.Kd;
    flash_union_buffer[4].float_type  = g_speed_pid_l.Kp;
    flash_union_buffer[5].float_type  = g_speed_pid_l.Ki;
    flash_union_buffer[6].float_type  = g_speed_pid_r.Kp;
    flash_union_buffer[7].float_type  = g_speed_pid_r.Ki;
    flash_union_buffer[8].int32_type  = (int32_t)g_base_speed;
    flash_union_buffer[9].float_type  = g_max_pwm_duty;
    flash_union_buffer[10].float_type = g_min_start_duty;
    flash_union_buffer[11].float_type = g_speed_steer_scale;
    flash_union_buffer[12].int32_type = (int32_t)g_dynamic_threshold;
    flash_union_buffer[13].int32_type = g_error_row_1;
    flash_union_buffer[14].int32_type = g_error_row_2;
    flash_union_buffer[15].int32_type = g_error_row_3;
    flash_union_buffer[16].int32_type = g_error_row_4;
    flash_union_buffer[17].float_type = g_weight_1;
    flash_union_buffer[18].float_type = g_weight_2;
    flash_union_buffer[19].float_type = g_weight_3;
    flash_union_buffer[20].float_type = g_weight_4;
    flash_union_buffer[21].int32_type = (int32_t)g_servo_min_duty;
    flash_union_buffer[22].int32_type = (int32_t)g_servo_max_duty;
    flash_union_buffer[23].int32_type = (int32_t)g_servo_center_duty;
    flash_union_buffer[24].int32_type = g_max_target_speed;
    flash_union_buffer[25].int32_type = g_speed_pid_integral_threshold;
    flash_union_buffer[26].float_type = g_speed_pid_integral_max;
    flash_union_buffer[27].int32_type = g_track_width_top;
    flash_union_buffer[28].int32_type = g_track_width_bottom;
    flash_write_page_from_buffer(0, PARAMS_FLASH_PAGE_INDEX, PARAMS_LEN);
    ips200_show_string(0, 180, "Parameters Saved!");
    system_delay_ms(500);
}

void params_load_from_flash(void)
{  
    flash_read_page_to_buffer(0, PARAMS_FLASH_PAGE_INDEX, PARAMS_LEN);
    if (flash_union_buffer[0].uint32_type != PARAMS_FLASH_MAGIC)
    {
        return;
    }
    g_steering_pid.Kp     = flash_union_buffer[1].float_type;
    g_steering_pid.Ki     = flash_union_buffer[2].float_type;
    g_steering_pid.Kd     = flash_union_buffer[3].float_type;

    g_speed_pid_l.Kp      = flash_union_buffer[4].float_type;
    g_speed_pid_l.Ki      = flash_union_buffer[5].float_type;

    g_speed_pid_r.Kp      = flash_union_buffer[6].float_type;
    g_speed_pid_r.Ki      = flash_union_buffer[7].float_type;

    g_base_speed          = (int16)flash_union_buffer[8].int32_type;
    g_max_pwm_duty        = flash_union_buffer[9].float_type;
    g_min_start_duty      = flash_union_buffer[10].float_type;
    g_speed_steer_scale   = flash_union_buffer[11].float_type;
    g_dynamic_threshold   = (int)flash_union_buffer[12].int32_type;

    g_error_row_1         = flash_union_buffer[13].int32_type;
    g_error_row_2         = flash_union_buffer[14].int32_type;
    g_error_row_3         = flash_union_buffer[15].int32_type;
    g_error_row_4         = flash_union_buffer[16].int32_type;

    g_weight_1            = flash_union_buffer[17].float_type;
    g_weight_2            = flash_union_buffer[18].float_type;
    g_weight_3            = flash_union_buffer[19].float_type;
    g_weight_4            = flash_union_buffer[20].float_type;

    g_servo_min_duty     = (uint32)flash_union_buffer[21].int32_type;
    g_servo_max_duty     = (uint32)flash_union_buffer[22].int32_type;
    g_servo_center_duty  = (uint32)flash_union_buffer[23].int32_type;


    g_max_target_speed               = flash_union_buffer[24].int32_type;
    g_speed_pid_integral_threshold   = flash_union_buffer[25].int32_type;
    g_speed_pid_integral_max         = flash_union_buffer[26].float_type;


    g_track_width_top                = flash_union_buffer[27].int32_type;
    g_track_width_bottom             = flash_union_buffer[28].int32_type;
}

