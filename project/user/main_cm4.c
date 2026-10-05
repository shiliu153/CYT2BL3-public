#include "zf_common_headfile.h"
#include <math.h>

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
uint32 g_servo_min_duty = 3713;
uint32 g_servo_max_duty = 5000;
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
#define ACKERMANN_WHEELBASE_MM      (210.0f)   // 轴距
#endif

#ifndef ACKERMANN_TRACK_MM
#define ACKERMANN_TRACK_MM          (160.0f)   // 后轮轮距(左右驱动轮中心距)
#endif

#ifndef ACKERMANN_MAX_STEER_DEG
#define ACKERMANN_MAX_STEER_DEG     (15.0f)    // 舵机打满对应的最大前轮转角(左右对称)
#endif

#ifndef ACKERMANN_DIFF_SIGN
#define ACKERMANN_DIFF_SIGN         (-1.0f)
#endif


static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

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
        return t * max_angle_rad;
    }
    else
    {
        float denom = (center - min_duty);
        if (denom < 1.0f) denom = 1.0f;
        float t = (center - (float)duty) / denom;
        t = clampf(t, 0.0f, 1.0f);
        return -t * max_angle_rad;
    }
}
int16   g_max_target_speed = 1000;
#define IPS200_TYPE     (IPS200_TYPE_SPI)
int     g_speed_pid_integral_threshold = 75;
float   g_speed_pid_integral_max = 1000.0f; 



//------------------ 赛道宽度变量 ------------------
int g_track_width_top = 60;
int g_track_width_bottom = 180;


// ----------------- Flash存储参数定义 -----------------
#define PARAMS_FLASH_MAGIC        (0xABCD1240)
#define PARAMS_FLASH_PAGE_INDEX   (0)
#define PARAMS_LEN                (26)

// ===================================================================================================================
// ================================================= 全局变量定义 ====================================================
// ===================================================================================================================

// -- 菜单状态 --
volatile menu_state_t g_menu_state = MENU_MAIN;
int g_dynamic_threshold = 128;          // 图像二值化阈值
int16 g_base_speed   = 350;              // 编码器脉冲数/10ms

// 直线加速策略：仅当陀螺仪角速度足够小才加速（不做转弯减速）
int16 g_straight_speed_add = 80;              // 直线额外加速量（脉冲/10ms）
float g_straight_gyro_thr_dps = 20.0f;        // 直线加速允许的陀螺仪角速度阈值（dps）
float g_straight_img_err_thr = 10.0f;         // 直线加速允许的图像误差阈值（像素）

// 图像误差计算的行范围（写入 Flash 持久化）
int g_img_error_row_start = 10;
int g_img_error_row_end   = 40;

float_t g_max_pwm_duty = 30.0;              // 电机最大PWM占空比比例 (0-100)
float_t g_min_start_duty = 15.0f;        // 电机最小启动占空比 (%)
float g_ackermann_scale_limit = 1.2f;    // 阿克曼差速比例限幅（保护），用于 clamp(scale, -limit, +limit)
volatile bool g_stop_active = false;       // 停车线触发后的停车保持标志（由 stop_line 驱动）

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
#define OTSU_UPDATE_INTERVAL_RUN     (6)
#endif

// 调试模式下每 N 帧更新一次大津阈值
#ifndef OTSU_UPDATE_INTERVAL_DEBUG
#define OTSU_UPDATE_INTERVAL_DEBUG   (6)
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
#define IMAGE_FILTER_ROW_START       (1)
#endif

#ifndef IMAGE_FILTER_ROW_END
#define IMAGE_FILTER_ROW_END         (40)
#endif


uint8 g_image_binary[MT9V03X_H][MT9V03X_W]; // 二值化后图像数组
uint8 g_left_line[MT9V03X_H];               // 赛道左边界
uint8 g_right_line[MT9V03X_H];              // 赛道右边界
uint8 g_center_line[MT9V03X_H];             // 赛道中心线

// 八邻域巡线临时缓冲区
static uint8 bin_image[MT9V03X_H][MT9V03X_W];
static int16 right_border[MT9V03X_H];
static int16 left_border[MT9V03X_H];
static uint16 points_l[1024][2];
static uint16 points_r[1024][2];
static int8 dir_l[1024];
static int8 dir_r[1024];
static uint8 l_dir_sum[9];
static uint8 r_dir_sum[9];
static uint16 points_l_back[512][2];
static uint16 points_r_back[512][2];
static uint16 l_back_cnt = 0;
static uint16 r_back_cnt = 0;
static uint16 trace_l_total = 0;
static uint16 trace_r_total = 0;
static uint8 trace_valid = 0;

// ----------------- 元素识别/处理 -----------------

static uint8 width_begin[MT9V03X_H];
static uint8 width_zzh[MT9V03X_H];
static uint8 width_inited = 0;
static int width_last_top = -1;
static int width_last_bottom = -1;

static uint8 flag_r_cir = 0;
static uint8 flag_l_cir = 0;
static uint8 flag_cross = 0;
static uint8 flag_others2 = 1;

static uint8 cir_stage = 0;
static uint8 cir_x = 0;
static uint8 cir_y = 0;

static int16 length = 0;
static uint8 lose_num = 0;
static uint8 nolose_num = 0;
static int16 r_lose_num = 0;
static int16 l_lose_num = 0;
static uint8 r_lose_judge[MT9V03X_H];
static uint8 l_lose_judge[MT9V03X_H];

static int cross_jud_time = 0;
static int cross_jud = 0;

static bool flag_str = false;
static bool flag_str_str = false;

static uint8 flag_stop = 0;
static int stop_line_time = 0;

static uint8 flag_bar = 0;
static uint8 bar_count_l = 0;
static uint8 bar_count_r = 0;

static int16 l_line_qulv = 0;
static int16 r_line_qulv = 0;

static volatile int16 warn_sink_i16 = 0;

static int16 center_line_i16[MT9V03X_H];

static int16 black_jiexian = 0;

static uint16 data_statics_l = 0;
static uint16 data_statics_r = 0;


PID_t g_steering_pid;                       // 转向PID控制器 (外环)
PID_t g_speed_pid_l;                        // 左轮速度PID (内环)
PID_t g_speed_pid_r;                        // 右轮速度PID (内环)

float g_final_error = 0;                     // 最终计算的偏差值

volatile int16 g_target_speed_l = 0;                 // 左轮目标速度
volatile int16 g_target_speed_r = 0;                 // 右轮目标速度
int16 g_actual_speed_l = 0;                 // 左轮实际速度
int16 g_actual_speed_r = 0;                 // 右轮实际速度
float_t g_motor_out_l = 0;                    // 左电机最终编码器速度
float_t g_motor_out_r = 0;                    // 右电机最终编码器速度
float_t g_motor_out_l_duty = 0;               // 左电机最终输出PWM占空比
float_t g_motor_out_r_duty = 0;             // 右电机最终输出PWM占空比
volatile bool g_track_lost = false;
volatile bool g_run_enable = false;

static volatile uint8 g_threshold_debug_show_image = 1;





// ===================================================================================================================
// ================================================== 函数声明 =======================================================
// ===================================================================================================================

static inline int zf_abs_i(int v) { return (v >= 0) ? v : -v; }

static inline int scale_from_120(int v_at_120)
{
    return (int)((MT9V03X_H * v_at_120 + 60) / 120);
}

static inline int scale_from_180(int v_at_180)
{
    return (int)((MT9V03X_W * v_at_180 + 90) / 180);
}

static inline uint8 clamp_row_u8(int row)
{
    if (row < 0) return 0;
    if (row >= MT9V03X_H) return (uint8)(MT9V03X_H - 1);
    return (uint8)row;
}

// 轻量性能计时（用于FPS显示）
static inline void perf_timer_init_once(void)
{
    static uint8 inited = 0;
    if (inited) return;
    inited = 1;

#if defined(CoreDebug) && defined(DWT) && defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static inline uint32 perf_now_ms(void)
{
    perf_timer_init_once();

#if defined(DWT)
    uint32 cycles_per_ms = (uint32)(SystemCoreClock / 1000U);
    if (cycles_per_ms == 0U) cycles_per_ms = 1U;
    return (uint32)(DWT->CYCCNT / cycles_per_ms);
#else
    return 0;
#endif
}
static void image_filter(uint8 img[MT9V03X_H][MT9V03X_W]);
static void image_draw_rectan(uint8 img[MT9V03X_H][MT9V03X_W]);
static void width_change(void);
static uint8 width_from_row(int row);
static bool stop_line(void);
static int cross_delay_judge(void);
static void element_test(void);
static uint8 straight_judge(void);
static uint8 straight_judge_2(void);
static void judge_bar(void);
static void element_handle(void);
static void width_init_if_needed(void);

void motor_init(void);
void pid_init(PID_t *pid, float kp, float ki, float kd);
void steering_pid_calculate(void);
void speed_pid_calculate(PID_t *pid, int16 target_speed, int16 actual_speed);
int calculate_otsu_threshold(const uint8 (*image)[MT9V03X_W]);
static int calculate_otsu_threshold_downsampled(const uint8 (*image)[MT9V03X_W], int step);
void image_process(uint8 threshold);
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

    
    g_dynamic_threshold = calculate_otsu_threshold((const uint8 (*)[MT9V03X_W])mt9v03x_image);
    
    // ----------------- PID 初始化 -----------------
    pid_init(&g_steering_pid, 15.1f, 0.00f, 1.9f);
    pid_init(&g_speed_pid_l, 16.8f, 2.5f, 0.0f);
    pid_init(&g_speed_pid_r, 16.8f, 2.5f, 0.0f);


    // ----------------- Flash参数初始化 -----------------
    flash_init(); 
    params_load_from_flash();
    ips200_clear();
    ips200_show_string(0, 0, "Params Loaded!");
    g_servo_duty = g_servo_center_duty;
    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, g_servo_duty);

    pit_us_init(PIT_CH0, SPEED_CONTROL_PERIOD);
    pit_us_init(PIT_CH1, 7000);

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
    const int edge_thickness = 2;   // 左右边线加粗
    const int center_thickness = 1; // 中心线保持原宽度

    for (int row = 0; row < MT9V03X_H; row++)
    {
        uint16 y = (uint16)(y_off + row);
        int l = (int)left_border[row];
        int r = (int)right_border[row];
        int m = (int)center_line_i16[row];

        for (int dx = -edge_thickness; dx <= edge_thickness; dx++)
        {
            int lx = l + dx;
            int rx = r + dx;
            if (l != -1 && lx >= 0 && lx < MT9V03X_W)
            {
                ips200_draw_point((uint16)(x_off + lx), y, RGB565_GREEN);
            }
            if (r != 200 && rx >= 0 && rx < MT9V03X_W)
            {
                ips200_draw_point((uint16)(x_off + rx), y, RGB565_RED);
            }
        }

        for (int dx = -center_thickness; dx <= center_thickness; dx++)
        {
            int mx = m + dx;
            if (mx >= 0 && mx < MT9V03X_W)
            {
                ips200_draw_point((uint16)(x_off + mx), y, RGB565_BLUE);
            }
        }
    }
}



/**
 * @brief 绘制边界线
 */
static void draw_trace_points_overlay(void)
{
    if (!trace_valid)
    {
        return;
    }

    for (uint16 i = 0; i < trace_l_total && i < 1024; i++)
    {
        int x = (int)points_l[i][0];
        int y = (int)points_l[i][1];
        if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
        {
            ips200_draw_point((uint16)x, (uint16)y, RGB565_GREEN);
        }
    }

    for (uint16 i = 0; i < trace_r_total && i < 1024; i++)
    {
        int x = (int)points_r[i][0];
        int y = (int)points_r[i][1];
        if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
        {
            ips200_draw_point((uint16)x, (uint16)y, RGB565_RED);
        }
    }

    // 向后爬出来又撤回的点，用不同颜色标记
    for (uint16 i = 0; i < l_back_cnt && i < 512; i++)
    {
        int x = (int)points_l_back[i][0];
        int y = (int)points_l_back[i][1];
        if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H)
        {
            ips200_draw_point((uint16)x, (uint16)y, RGB565_CYAN);
        }
    }

    for (uint16 i = 0; i < r_back_cnt && i < 512; i++)
    {
        int x = (int)points_r_back[i][0];
        int y = (int)points_r_back[i][1];
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

    static uint16 otsu_frame_cnt = 0;
    while(1)
    {
        if(mt9v03x_finish_flag)
        {
            if (OTSU_UPDATE_INTERVAL_RUN <= 1 || (otsu_frame_cnt % OTSU_UPDATE_INTERVAL_RUN) == 0)
            {
                g_dynamic_threshold = calculate_otsu_threshold_downsampled((const uint8 (*)[MT9V03X_W])mt9v03x_image, OTSU_DOWNSAMPLE_STEP_RUN);
            }
            otsu_frame_cnt++;
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
    const int param_count = 9;
    const float p_step = 0.05f;
    const float i_step = 0.001f;
    const float d_step = 0.05f;
    const float duty_step = 1.0f;
    const float scale_step = 0.01f;
    const int base_speed_step = 10;
    const int straight_add_step = 10;
    const float gyro_thr_step = 1.0f;
    const float img_err_thr_step = 0.5f;

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

        sprintf(text_buff, "%c Ack Limit: %.3f", (selected_param_index == 4) ? (edit_mode ? '*' : '>') : ' ', (double)g_ackermann_scale_limit);
        ips200_show_string(0, 75, text_buff);

        sprintf(text_buff, "%c Max Duty: %.1f", (selected_param_index == 5) ? (edit_mode ? '*' : '>') : ' ', (double)g_max_pwm_duty);
        ips200_show_string(0, 90, text_buff);

        sprintf(text_buff, "%c Str Add: %d", (selected_param_index == 6) ? (edit_mode ? '*' : '>') : ' ', (int)g_straight_speed_add);
        ips200_show_string(0, 105, text_buff);

        sprintf(text_buff, "%c GyroThr: %.1f", (selected_param_index == 7) ? (edit_mode ? '*' : '>') : ' ', (double)g_straight_gyro_thr_dps);
        ips200_show_string(0, 120, text_buff);

        sprintf(text_buff, "%c ImgErrThr: %.1f", (selected_param_index == 8) ? (edit_mode ? '*' : '>') : ' ', (double)g_straight_img_err_thr);
        ips200_show_string(0, 135, text_buff);

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
                    case 4: g_ackermann_scale_limit += scale_step; if (g_ackermann_scale_limit > 3.0f) g_ackermann_scale_limit = 3.0f; break;
                    case 5: g_max_pwm_duty += duty_step; if(g_max_pwm_duty > 80.0f) g_max_pwm_duty = 80.0f; break;
                    case 6: g_straight_speed_add += straight_add_step; if (g_straight_speed_add > 600) g_straight_speed_add = 600; break;
                    case 7: g_straight_gyro_thr_dps += gyro_thr_step; if (g_straight_gyro_thr_dps > 200.0f) g_straight_gyro_thr_dps = 200.0f; break;
                    case 8: g_straight_img_err_thr += img_err_thr_step; if (g_straight_img_err_thr > 100.0f) g_straight_img_err_thr = 100.0f; break;
                }
            } else if (key == 3) {
                switch(selected_param_index) {
                    case 0: g_steering_pid.Kp -= p_step; if(g_steering_pid.Kp < 0) g_steering_pid.Kp = 0; break;
                    case 1: g_steering_pid.Ki -= i_step; if(g_steering_pid.Ki < 0) g_steering_pid.Ki = 0; break;
                    case 2: g_steering_pid.Kd -= d_step; if(g_steering_pid.Kd < 0) g_steering_pid.Kd = 0; break;
                    case 3: g_base_speed -= base_speed_step; if(g_base_speed < 0) g_base_speed = 0; break;
                    case 4: g_ackermann_scale_limit -= scale_step; if(g_ackermann_scale_limit < 0) g_ackermann_scale_limit = 0; break;
                    case 5: g_max_pwm_duty -= duty_step; if(g_max_pwm_duty < g_min_start_duty) g_max_pwm_duty = g_min_start_duty; break;
                    case 6: g_straight_speed_add -= straight_add_step; if (g_straight_speed_add < 0) g_straight_speed_add = 0; break;
                    case 7: g_straight_gyro_thr_dps -= gyro_thr_step; if (g_straight_gyro_thr_dps < 0.0f) g_straight_gyro_thr_dps = 0.0f; break;
                    case 8: g_straight_img_err_thr -= img_err_thr_step; if (g_straight_img_err_thr < 0.0f) g_straight_img_err_thr = 0.0f; break;
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
    ips200_show_string(0, 135, "K1: Exit  K4:View");

    g_target_speed_l = 0;
    g_target_speed_r = 0;

    static bool show_image = true;
    static uint32 fps_last_ms = 0;
    static uint32 fps_frame_cnt = 0;
    static uint32 fps_value = 0;
    g_threshold_debug_show_image = show_image ? 1U : 0U;
    ips200_show_string(0, 0, show_image ? "View: IMAGE" : "View: FRAMES");

    while(1)
    {
        uint8 key = get_key_value();
        if (key == 1) {
            pwm_set_duty(SERVO_PWM_PIN, g_servo_center_duty);
            g_menu_state = MENU_MAIN;
            ips200_clear();
            break;
        }

        if (key == 4) {
            // 切换前先清屏，避免残影
            ips200_clear();
            show_image = !show_image;
            g_threshold_debug_show_image = show_image ? 1U : 0U;
            ips200_show_string(0, 120, "Auto Otsu Threshold");
            ips200_show_string(0, 135, "K1: Exit  K4:View");
            ips200_show_string(0, 0, show_image ? "View: IMAGE" : "View: FRAMES");
            system_delay_ms(150);
        }

        if(mt9v03x_finish_flag)
        {
            static uint16 otsu_frame_cnt = 0;
            if (OTSU_UPDATE_INTERVAL_DEBUG <= 1 || (otsu_frame_cnt % OTSU_UPDATE_INTERVAL_DEBUG) == 0)
            {
                g_dynamic_threshold = calculate_otsu_threshold_downsampled((const uint8 (*)[MT9V03X_W])mt9v03x_image, OTSU_DOWNSAMPLE_STEP_DEBUG);
            }
            otsu_frame_cnt++;
            image_process(g_dynamic_threshold);
            
            calculate_line_error();
            
            
            char text_buff[48];
            fps_frame_cnt++;
            uint32 now_ms = perf_now_ms();
            if (fps_last_ms == 0U) fps_last_ms = now_ms;
            uint32 dt = now_ms - fps_last_ms;
            if (dt >= 1000U)
            {
                fps_value = (dt > 0U) ? (uint32)((fps_frame_cnt * 1000U) / dt) : 0U;
                fps_frame_cnt = 0;
                fps_last_ms = now_ms;
            }

            if (show_image)
            {
                ips200_displayimage03x((const uint8 *)g_image_binary, MT9V03X_W, MT9V03X_H);
                draw_trace_points_overlay();
                draw_edges_overlay();
            }
            mt9v03x_finish_flag = 0;
            if (show_image)
            {
                if (stop_line_time != 0 || flag_stop)
                {
                    sprintf(text_buff, "Elem: STOP(%d)", (int)stop_line_time);
                }
                else if (flag_r_cir)
                {
                    sprintf(text_buff, "Elem: R_CIR S%u", (unsigned)cir_stage);
                }
                else if (flag_l_cir)
                {
                    sprintf(text_buff, "Elem: L_CIR S%u", (unsigned)cir_stage);
                }
                else if (flag_cross)
                {
                    sprintf(text_buff, "Elem: CROSS J%d", (int)cross_jud);
                }
                else if (flag_bar == 1)
                {
                    sprintf(text_buff, "Elem: BAR_L");
                }
                else if (flag_bar == 2)
                {
                    sprintf(text_buff, "Elem: BAR_R");
                }
                else if (flag_str_str)
                {
                    sprintf(text_buff, "Elem: STR2");
                }
                else if (flag_str)
                {
                    sprintf(text_buff, "Elem: STR");
                }
                else
                {
                    sprintf(text_buff, "Elem: NONE");
                }
                ips200_show_string(0, 145, text_buff);

                sprintf(text_buff, "Thresh: %d", g_dynamic_threshold);
                ips200_show_string(0, 160, text_buff);

                sprintf(text_buff, "Error: %.2f", (double)g_final_error);
                ips200_show_string(0, 175, text_buff);

                sprintf(text_buff, "SteerOut: %.2f", (double)g_steering_pid.output);
                ips200_show_string(0, 200, text_buff);
            }
            else
            {
                sprintf(text_buff, "FPS: %lu", (unsigned long)fps_value);
                ips200_show_string(0, 15, text_buff);
            }

        }
        system_delay_ms(show_image ? 10 : 1);
    }
}

/**
 * @brief       前瞻变量调试任务
 */
void menu_preview_var_task(void)
{
    int idx = 0;
    bool edit = false;
    const int cnt = 2;
    while(1)
    {
        char buf[64];
        sprintf(buf, "%c ErrRowStart: %d", (idx==0)?(edit?'*':'>'):' ', g_img_error_row_start); ips200_show_string(0, 10, buf);
        sprintf(buf, "%c ErrRowEnd  : %d", (idx==1)?(edit?'*':'>'):' ', g_img_error_row_end); ips200_show_string(0, 25, buf);

        ips200_show_string(0, 150, "K1:Back K4:Edit K2/3:Inc/Dec");
        uint8 key = get_key_value();
        if(key == 1) {
            params_save_to_flash();
            g_menu_state = MENU_MAIN;
            ips200_clear();
            break;}
        else if(key == 4) edit = !edit;
        else if(edit && key == 2) {
            if(idx==0) g_img_error_row_start = MIN(MAX(g_img_error_row_start+1,0), MT9V03X_H-1);
            if(idx==1) g_img_error_row_end   = MIN(MAX(g_img_error_row_end+1,0), MT9V03X_H-1);
        }
        else if(edit && key == 3) {
            if(idx==0) g_img_error_row_start = MAX(MIN(g_img_error_row_start-1, MT9V03X_H-1), 0);
            if(idx==1) g_img_error_row_end   = MAX(MIN(g_img_error_row_end-1, MT9V03X_H-1), 0);
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

    while(1)
    {
        imu660rb_get_acc();
        imu660rb_get_gyro();

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

        ips200_show_string(0, 160, "K1:Back  K4:Reinit");

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

    float d_term = g_steering_pid.Kd * gyro_z_filt;

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

/**
 * @brief 寻找八邻域起始点
 */
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


/**
 * @brief  八邻域搜线
 */
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
    l_back_cnt = 0;
    r_back_cnt = 0;

    for (int i = 0; i < 9; i++)
    {
        l_dir_sum[i] = 0;
        r_dir_sum[i] = 0;
    }

    // 单边找不到候选点时冻结该边，让另一边继续爬线
    bool hold_l = false;
    bool hold_r = false;

    while (break_flag-- && l_data_statics < 1024 && r_data_statics < 1024)
    {
        const bool l_at_top = (center_point_l[1] == 0);
        const bool l_top_freeze = (l_at_top && l_data_statics > 0 && points_l[l_data_statics - 1][1] == 0);
        bool l_frozen = (hold_l || l_top_freeze);
        if (l_frozen && l_data_statics == 0) l_frozen = false;

        const bool r_at_top = (center_point_r[1] == 0);
        const bool r_top_freeze = (r_at_top && r_data_statics > 0 && points_r[r_data_statics - 1][1] == 0);
        bool r_frozen = (hold_r || r_top_freeze);
        uint16 r_curr = r_data_statics;
        if (r_frozen)
        {
            if (r_data_statics > 0)
            {
                r_curr = r_data_statics - 1;
            }
            else
            {
                r_frozen = false;
            }
        }

        for (uint8 i = 0; i < 8; i++)
        {
            search_fields_l[i][0] = (uint8)(center_point_l[0] + seeds_l[i][0]);
            search_fields_l[i][1] = (uint8)(center_point_l[1] + seeds_l[i][1]);
        }

        if (!l_frozen)
        {
            points_l[l_data_statics][0] = center_point_l[0];
            points_l[l_data_statics][1] = center_point_l[1];
            dir_l[l_data_statics] = -1;
            l_data_statics++;
        }

        for (uint8 i = 0; i < 8; i++)
        {
            search_fields_r[i][0] = (uint8)(center_point_r[0] + seeds_r[i][0]);
            search_fields_r[i][1] = (uint8)(center_point_r[1] + seeds_r[i][1]);
        }

        points_r[r_curr][0] = center_point_r[0];
        points_r[r_curr][1] = center_point_r[1];
        dir_r[r_curr] = -1;

        index_l = 0;
        if (!l_at_top && !l_frozen)
        {
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

                if (bin_image[y1][x1] != 255 && bin_image[y2][x2] == 255)
                {
                    temp_l[index_l][0] = (uint8)x1;
                    temp_l[index_l][1] = (uint8)y1;
                    index_l++;

                    if (l_near_top)
                    {
                        if (l_data_statics > 0) dir_l[l_data_statics - 1] = -1;
                    }
                    else
                    {
                        if (l_data_statics > 0) dir_l[l_data_statics - 1] = (int8)i;
                        if (i < 9) l_dir_sum[i]++;
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
        }

        // 左边本轮找不到候选点：冻结左边，允许右边继续
        if (!l_at_top && !l_frozen && index_l == 0)
        {
            hold_l = true;
            l_frozen = true;
        }

        bool stuck_r = false;
        bool stuck_l = false;
        if (!r_frozen && r_data_statics >= 2)
        {
            stuck_r = (points_r[r_curr][0] == points_r[r_data_statics - 1][0] &&
                       points_r[r_curr][0] == points_r[r_data_statics - 2][0] &&
                       points_r[r_curr][1] == points_r[r_data_statics - 1][1] &&
                       points_r[r_curr][1] == points_r[r_data_statics - 2][1]);
        }
        if (!l_frozen && l_data_statics >= 3)
        {
            stuck_l = (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
                       points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
                       points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
                       points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]);
        }
        if (stuck_l) hold_l = true;
        if (stuck_r) hold_r = true;
        if ((stuck_l || l_frozen) && (stuck_r || r_frozen))
        {
            break;
        }

        bool r_above_l = false;
        if (l_data_statics > 0)
        {
            int dx = zf_abs_i((int)points_r[r_curr][0] - (int)points_l[l_data_statics - 1][0]);
            int dy = zf_abs_i((int)points_r[r_curr][1] - (int)points_l[l_data_statics - 1][1]);
            r_above_l = (points_r[r_curr][1] < points_l[l_data_statics - 1][1]);

            // 只有两边都在正常爬线时才触发“提前相遇”退出，避免单边冻结时误触发
            if (!l_frozen && !r_frozen && !l_at_top && !r_at_top && dx < 2 && dy < 2)
            {
                *highest_row = (uint8)((points_r[r_curr][1] + points_l[l_data_statics - 1][1]) >> 1);
                break;
            }
        }

        if (!l_at_top && !r_above_l && l_data_statics > 0 && dir_l[l_data_statics - 1] == 7 &&
            points_r[r_curr][1] > points_l[l_data_statics - 1][1])
        {
            if (record_back_points && l_back_cnt < 512)
            {
                points_l_back[l_back_cnt][0] = points_l[l_data_statics - 1][0];
                points_l_back[l_back_cnt][1] = points_l[l_data_statics - 1][1];
                l_back_cnt++;
            }
            center_point_l[0] = (uint8)points_l[l_data_statics - 1][0];
            center_point_l[1] = (uint8)points_l[l_data_statics - 1][1];
            l_data_statics--;
        }

        if (!r_frozen)
        {
            r_data_statics++;
        }

        index_r = 0;
        if (!r_at_top && !r_frozen)
        {
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

                if (bin_image[y1][x1] != 255 && bin_image[y2][x2] == 255)
                {
                    temp_r[index_r][0] = (uint8)x1;
                    temp_r[index_r][1] = (uint8)y1;
                    index_r++;

                    if (r_near_top)
                    {
                        if (r_data_statics > 0) dir_r[r_data_statics - 1] = -1;
                    }
                    else
                    {
                        if (r_data_statics > 0) dir_r[r_data_statics - 1] = (int8)i;
                        if (i < 9) r_dir_sum[i]++;
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

        // 右边本轮找不到候选点：冻结右边，允许左边继续
        if (!r_at_top && !r_frozen && index_r == 0)
        {
            hold_r = true;
            r_frozen = true;
        }

        if (center_point_l[1] == 0 && center_point_r[1] == 0)
        {
            break;
        }
    }


    *l_count = l_data_statics;
    *r_count = r_data_statics;
    

    return (l_data_statics > 2 && r_data_statics > 2);
}

/**
 * @brief 提取边界信息
 */
void extract_borders(uint16 l_total, uint16 r_total)
{
    for (int i = 0; i < MT9V03X_H; i++)
    {
        left_border[i] = -1;
        right_border[i] = 200;
    }

    for (uint16 i = 0; i < l_total; i++)
    {
        int row = points_l[i][1];
        int col = points_l[i][0];
        if (row >= 0 && row < MT9V03X_H && col > left_border[row])
        {
            left_border[row] = col;
        }
    }

    for (uint16 i = 0; i < r_total; i++)
    {
        int row = points_r[i][1];
        int col = points_r[i][0];
        if (row >= 0 && row < MT9V03X_H && col < right_border[row])
        {
            right_border[row] = col;
        }
    }
}

/**
 * @brief  根据已提取的左右边界线，计算并填充赛道中心线数据。
 */
void fill_center_lines(void)
{
    int last_l = MT9V03X_W / 4;
    int last_r = MT9V03X_W * 3 / 4;
    for (int row = 0; row < MT9V03X_H; row++)
    {
        int l = left_border[row];
        int r = right_border[row];

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


/**
 * @brief       备用扫描线巡线算法
 *
 * @param   threshold         图像二值化阈值
 * @param   generate_binary   是否生成二值化图像（true 生成，false 仅计算/处理）
 *
 * @details
 * 本函数实现了一种基于扫描线的巡线算法，作为主巡线算法失效时的备用方案。
 * 通过遍历图像的每一行（或指定行），根据给定的阈值对图像进行二值化处理，
 * 并在每一行中查找赛道的左右边界点。可选择是否生成二值化图像数据。
 * 该方法通常用于应对八邻域巡线失败、丢线等特殊情况，保证车辆能够继续循迹。
 */
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
    bool generate_binary = (g_menu_state == MENU_THRESHOLD_DEBUG) && (g_threshold_debug_show_image != 0);

    if (generate_binary)
    {
        for (int i = 0; i < MT9V03X_H; i++)
        {
            uint8 *src = mt9v03x_image[i];
            uint8 *dst = bin_image[i];
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
            uint8 *dst = bin_image[i];
            for (int j = 0; j < MT9V03X_W; j++)
            {
                dst[j] = (src[j] >= threshold) ? 255 : 0;
            }
        }
    }

    //图像过滤
    if (IMAGE_FILTER_ENABLE)
    {
        static uint16 filter_frame_cnt = 0;
        if (IMAGE_FILTER_INTERVAL <= 1 || (filter_frame_cnt % IMAGE_FILTER_INTERVAL) == 0)
        {
            image_filter(bin_image);
        }
        filter_frame_cnt++;
    }
    image_draw_rectan(bin_image);  //图像边界锁死
    width_init_if_needed();  //计算赛道丢失差值

    int l_start_x = 0, l_start_y = MT9V03X_H - 2;
    int r_start_x = 0, r_start_y = MT9V03X_H - 2;
    uint16 l_count = 0, r_count = 0;
    uint8 highest_row = 0;

    bool got_start = get_start_points(MT9V03X_H - 30, (const uint8 (*)[MT9V03X_W])bin_image, &l_start_x, &l_start_y, &r_start_x, &r_start_y);
    bool ok = false;
    if (got_start)
    {
        ok = search_eight_neighbor(1500, bin_image, l_start_x, l_start_y, r_start_x, r_start_y, &l_count, &r_count, &highest_row);
    }

    trace_valid = ok ? 1 : 0;
    trace_l_total = l_count;
    trace_r_total = r_count;

    if (ok)
    {
        extract_borders(l_count, r_count); //计算边界
    }
    else
    {
        fallback_scanline(threshold, generate_binary); //如果八邻域图像失效，则退回到该图像处理

        for (int row = 0; row < MT9V03X_H; row++)
        {
            left_border[row] = (int16)g_left_line[row];
            right_border[row] = (int16)g_right_line[row];
        }
    }

    data_statics_l = l_count;   //辅助元素判断
    data_statics_r = r_count;   //辅助元素判断
    bool meet_happened = (ok && highest_row < (uint8)MT9V03X_H); //提前相遇，赛道向一侧便宜
    int16 forced_mid_after_meet = (int16)(MT9V03X_W / 2);
    if (meet_happened) //补线策略
    {
        int ref_mid = MT9V03X_W / 2;
        int start = (int)highest_row;
        if (start < 0) start = 0;
        if (start >= MT9V03X_H) start = MT9V03X_H - 1;

        for (int row = start; row < MT9V03X_H; row++)
        {
            int16 l = left_border[row];
            int16 r = right_border[row];
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
        int16 l = left_border[row];
        int16 r = right_border[row];
        if (l < 0 && r == 200)
        {
            if (meet_happened && row < (int)highest_row)
            {
                center_line_i16[row] = forced_mid_after_meet;
                last_mid = forced_mid_after_meet;
            }
            else
            {
                center_line_i16[row] = last_mid;
            }
        }
        else
        {
            if (l < 0) l = (int16)(last_mid - (int16)(width_from_row(row) / 2));
            if (r == 200) r = (int16)(last_mid + (int16)(width_from_row(row) / 2));
            center_line_i16[row] = (l + r) / 2;
            last_mid = center_line_i16[row];
        }
    }

    // uint32 primask = interrupt_global_disable();
    // g_encoder_delta_accum = 0;
    // interrupt_global_enable(primask);

    // (void)stop_line();
    // (void)cross_delay_judge();
    // element_test();
    // (void)straight_judge();
    // (void)straight_judge_2();
    // judge_bar();

    // if (flag_stop || flag_bar || flag_str || flag_str_str)
    // {
    //     warn_sink_i16 = (int16)(warn_sink_i16 + 1);
    // }

    // element_handle();

    for (int row = 0; row < MT9V03X_H; row++)
    {
        int l = (int)left_border[row];
        int r = (int)right_border[row];
        int m = (int)center_line_i16[row];

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



/**
 * @brief 计算error值，用于控制舵机
 */
void calculate_line_error(void)
{
    // 图像误差：取 10 行（y=10~40），自上而下权重依次减小
    // 采样行：10,13,16,20,23,26,30,33,36,40（线性插值得到）
    int row_start = g_img_error_row_start;
    int row_end = g_img_error_row_end;
    const int sample_n = 10;

    if (row_start < 0) row_start = 0;
    if (row_start >= MT9V03X_H) row_start = MT9V03X_H - 1;
    if (row_end < 0) row_end = 0;
    if (row_end >= MT9V03X_H) row_end = MT9V03X_H - 1;
    if (row_end < row_start)
    {
        int tmp = row_start;
        row_start = row_end;
        row_end = tmp;
    }
    if (row_end == row_start)
    {
        row_end = (row_start + 1 < MT9V03X_H) ? (row_start + 1) : row_start;
    }

    float sum = 0.0f;
    float wsum = 0.0f;
    for (int i = 0; i < sample_n; i++)
    {
        int row = row_start + (row_end - row_start) * i / (sample_n - 1);
        float err = (float)g_center_line[clamp_row_u8(row)] - (float)(MT9V03X_W / 2);
        float w = (float)(sample_n - i);   // 10,9,...,1
        sum += w * err;
        wsum += w;
    }

    g_final_error = (wsum > 0.0f) ? (sum / wsum) : 0.0f;
}

/**
 * @brief  转向控制函数，计算左右轮的目标速度
 */
void steering_control(void)
{   
    imu660rb_get_gyro();
    steering_pid_calculate();

    int32_t target_duty = (int32_t)g_servo_center_duty - (int32_t)g_steering_pid.output;
    
    if (target_duty < (int32_t)g_servo_min_duty) target_duty = (int32_t)g_servo_min_duty;
    if (target_duty > (int32_t)g_servo_max_duty) target_duty = (int32_t)g_servo_max_duty;
    
    g_servo_duty = (uint32)target_duty;
    pwm_set_duty(SERVO_PWM_PIN, g_servo_duty);

    float steer_rad = servo_duty_to_steer_rad(g_servo_duty);
    float tan_delta = tanf(steer_rad);
    float geom = (ACKERMANN_TRACK_MM / (2.0f * ACKERMANN_WHEELBASE_MM));
    float scale = (float)ACKERMANN_DIFF_SIGN * geom * tan_delta;
    scale = clampf(scale, -g_ackermann_scale_limit, g_ackermann_scale_limit);

    // 基准速度：只做直线加速（不做转弯减速），且严格限制陀螺仪角速度小于阈值才加速
    float base = (float)g_base_speed;
    {
        float gyro_dps = imu660rb_gyro_transition(imu660rb_gyro_z);
        float gyro_abs = (gyro_dps < 0.0f) ? -gyro_dps : gyro_dps;
        float img_err_abs = (g_final_error < 0.0f) ? -g_final_error : g_final_error;
        float thr = g_straight_gyro_thr_dps;
        if (thr < 0.0f) thr = 0.0f;
        float img_thr = g_straight_img_err_thr;
        if (img_thr < 0.0f) img_thr = 0.0f;
        // 直线加速额外限制：图像误差必须足够小
        if (gyro_abs < thr && img_err_abs < img_thr)
        {
            base += (float)g_straight_speed_add;
        }
    }

    // ----------------- 阿克曼差速：左右轮目标速度 -----------------
    float v_l = base * (1.0f - scale * g_ackermann_scale_limit);
    float v_r = base * (1.0f + scale * g_ackermann_scale_limit);

    int16 base_target_speed_l = (int16)v_l;
    int16 base_target_speed_r = (int16)v_r;

    g_target_speed_l = base_target_speed_l;
    g_target_speed_r = base_target_speed_r;

    if (g_target_speed_l < 0) g_target_speed_l = 0;
    else if (g_target_speed_l > g_max_target_speed) g_target_speed_l = g_max_target_speed;
    if (g_target_speed_r < 0) g_target_speed_r = 0;
    else if (g_target_speed_r > g_max_target_speed) g_target_speed_r = g_max_target_speed;
}


/**
 * @brief 计算每行应该补线多少，逆运算回去
 */
static void width_init_if_needed(void)
{
    if (width_inited && width_last_top == g_track_width_top && width_last_bottom == g_track_width_bottom) return;
    for (int y = 0; y < MT9V03X_H; y++)
    {
        float t = (MT9V03X_H <= 1) ? 0.0f : (float)y / (float)(MT9V03X_H - 1);
        float w = (1.0f - t) * (float)g_track_width_top + t * (float)g_track_width_bottom;
        if (w < 10.0f) w = 10.0f;
        if (w > (float)(MT9V03X_W - 4)) w = (float)(MT9V03X_W - 4);
        width_begin[y] = (uint8)w;

        int wz = (int)width_begin[y] - 10;
        if (wz < 6) wz = 6;
        width_zzh[y] = (uint8)wz;
    }
    width_last_top = g_track_width_top;
    width_last_bottom = g_track_width_bottom;
    width_inited = 1;
}

static uint8 width_from_row(int row)
{
    int idx = (MT9V03X_H - 1) - row;
    if (idx < 0) idx = 0;
    if (idx >= MT9V03X_H) idx = MT9V03X_H - 1;
    return width_zzh[idx];
}

static float control_kk(float k)
{
    if (k > 5.0f) return 5.0f;
    if (k < 0.1f) return 0.0f;
    return k;
}

static uint8 sum_dir_r(void)
{

    int r120 = MIN(MT9V03X_H - 1, 120);
    int r90  = MIN(MT9V03X_H - 1, 90);
    int r80  = MIN(MT9V03X_H - 1, 80);
    int r60  = MIN(MT9V03X_H - 1, 60);
    int r25  = MIN(MT9V03X_H - 1, 25);
    int r10  = MIN(MT9V03X_H - 1, 10);

    float denom1 = (float)(right_border[r120] - right_border[r60]);
    float denom2 = (float)(right_border[r120] - right_border[r10]);
    float denom4 = (float)(right_border[r120] - right_border[r90]);
    float denom6 = (float)(right_border[r80]  - right_border[r25]);

    float k1 = (denom1 == 0.0f) ? 0.0f : (float)(r120 - r60) / denom1;
    float k2 = (denom2 == 0.0f) ? 0.0f : (float)(r120 - r10) / denom2;
    float k4 = (denom4 == 0.0f) ? 0.0f : (float)(r120 - r90) / denom4;
    float k6 = (denom6 == 0.0f) ? 0.0f : (float)(r80 - r25) / denom6;

    k1 = control_kk(k1);
    k2 = control_kk(k2);
    k4 = control_kk(k4);
    k6 = control_kk(k6);

    if (k1 >= 2.3f && k1 <= 5.0f && k4 >= 2.3f && k2 >= 2.2f && k2 <= 5.0f && k6 > 1.8f && k6 <= 5.0f)
    {
        return 1;
    }
    return 0;
}

static uint8 sum_dir_l(void)
{
    int r120 = MIN(MT9V03X_H - 1, 120);
    int r90  = MIN(MT9V03X_H - 1, 90);
    int r80  = MIN(MT9V03X_H - 1, 80);
    int r60  = MIN(MT9V03X_H - 1, 60);
    int r25  = MIN(MT9V03X_H - 1, 25);
    int r10  = MIN(MT9V03X_H - 1, 10);

    float denom1 = (float)(left_border[r60] - left_border[r120]);
    float denom2 = (float)(left_border[r10] - left_border[r120]);
    float denom4 = (float)(left_border[r90] - left_border[r120]);
    float denom6 = (float)(left_border[r25] - left_border[r80]);

    float k1 = (denom1 == 0.0f) ? 0.0f : (float)(r120 - r60) / denom1;
    float k2 = (denom2 == 0.0f) ? 0.0f : (float)(r120 - r10) / denom2;
    float k4 = (denom4 == 0.0f) ? 0.0f : (float)(r120 - r90) / denom4;
    float k6 = (denom6 == 0.0f) ? 0.0f : (float)(80 - 25) / denom6;

    k1 = control_kk(k1);
    k2 = control_kk(k2);
    k4 = control_kk(k4);
    k6 = control_kk(k6);

    if (k1 >= 2.2f && k2 >= 2.2f && k2 <= 5.0f && k4 > 2.2f && k4 <= 5.0f && k6 > 1.9f && k6 <= 5.0f)
    {
        return 1;
    }
    return 0;
}

static uint8 l_l_judge(void)
{
    uint8 sum = 0;
    int right_edge = MT9V03X_W - 4;
    for (int i = 0; i < MT9V03X_H; i++)
    {
        if (left_border[i] <= 4 && right_border[i] >= right_edge)
        {
            sum++;
        }
    }
    // 行计数阈值按高度比例缩放（120 高度时约等于 20/10）
    int th1 = MT9V03X_H / 6;
    int th2 = MT9V03X_H / 12;
    if (th1 < 10) th1 = 10;
    if (th2 < 5) th2 = 5;
    if (sum > (uint8)th1) return 1;
    if (sum > (uint8)th2) return 2;
    return 0;
}

static void lose_line(void)
{
    r_lose_num = 0;
    l_lose_num = 0;
    length = 0;

    int right_edge_min = MT9V03X_W - 4;
    int mid_x = MT9V03X_W / 2;

    for (int i = MT9V03X_H - 1; i > 1; i--)
    {
        if (left_border[i] < 0 || left_border[i] <= 3)
        {
            l_lose_judge[i] = 1;
            l_lose_num++;
        }
        else
        {
            l_lose_judge[i] = 0;
        }
    }

    for (int i = MT9V03X_H - 1; i > 1; i--)
    {
        if (right_border[i] == 200 || (right_border[i] >= right_edge_min && right_border[i] < MT9V03X_W))
        {
            r_lose_judge[i] = 1;
            r_lose_num++;
        }
        else
        {
            r_lose_judge[i] = 0;
        }
    }
    
    int prev_row_black = 0;
    for (int i = MT9V03X_H - 1; i > 0; i--)
    {
        int cx;
        if (left_border[i] >= 0 && right_border[i] != 200)
        {
            cx = (left_border[i] + right_border[i]) / 2;
        }
        else
        {
            cx = mid_x;
        }

        if (cx < 0) cx = 0;
        else if (cx >= MT9V03X_W) cx = MT9V03X_W - 1;

        int black_cnt = 0;
        for (int dx = -3; dx <= 3; dx++)
        {
            int x = cx + dx;
            if (x >= 0 && x < MT9V03X_W && bin_image[i][x] != 0)
            {
                black_cnt++;
            }
        }

        int row_black = (black_cnt >= 1);
        if (row_black && prev_row_black)
        {
            length++;
        }
        prev_row_black = row_black;
    }
}

static void lose_location_test(uint8 type, uint8 startline, uint8 endline)
{
    lose_num = 0;
    nolose_num = 0;
    if (startline >= MT9V03X_H) startline = (uint8)(MT9V03X_H - 1);
    if (endline >= MT9V03X_H) endline = 0;
    if (startline <= endline)
    {
        lose_num = 0;
        nolose_num = 0;
        return;
    }

    if (type == 1)
    {
        for (int y = startline; y >= endline; y--)
        {
            lose_num += l_lose_judge[y];
        }
        nolose_num = (uint8)((startline - endline + 1) - lose_num);
    }
    else if (type == 2)
    {
        for (int y = startline; y >= endline; y--)
        {
            lose_num += r_lose_judge[y];
        }
        nolose_num = (uint8)((startline - endline + 1) - lose_num);
    }
}

static void cir_point(int flag)
{
    if (flag == 1)
    {
        for (int i = (int)data_statics_l - 1; i > 1; i--)
        {
            if (dir_l[i - 1] == 6 && points_l[i][1] > 10)
            {
                cir_x = (uint8)points_l[i - 1][0];
                cir_y = (uint8)points_l[i - 1][1];
                break;
            }
        }
    }
    else
    {
        for (int i = (int)data_statics_r - 1; i > 1; i--)
        {
            if (dir_r[i - 1] == 6 && points_r[i][1] > 10)
            {
                cir_x = (uint8)points_r[i - 1][0];
                cir_y = (uint8)points_r[i - 1][1];
                break;
            }
        }
    }
}

static bool cir1_judge(void)
{
    lose_location_test(2, clamp_row_u8(scale_from_120(20)), 0);
    // length 门槛用比例阈值（避免入元素姿态不正时 length 上不去导致永远判不出）
    int length_th = (MT9V03X_H * 3) / 4; // 0.75H
    if (length_th < 40) length_th = 40;

    // l/r_lose_num 是按整幅图统计的丢边行数，阈值用高度比例，保持在不同分辨率下一致语义
    int l_lose_max = (MT9V03X_H * 1) / 3;     // 120 高度时约 40
    int r_lose_min = (MT9V03X_H * 5) / 12;    // 120 高度时约 50
    if (l_lose_max < 10) l_lose_max = 10;
    if (r_lose_min < 10) r_lose_min = 10;

    if (cir_stage == 0 && nolose_num > 10 && sum_dir_l() && l_lose_num < l_lose_max && r_lose_num > r_lose_min && length > length_th && r_dir_sum[4] > 50 && l_dir_sum[4] > 65)
    {
        lose_location_test(1, (uint8)(MT9V03X_H - 1), clamp_row_u8(MAX(0, MT9V03X_H - scale_from_120(18))));
        if (nolose_num > 13)
        {
            cir_stage = 1;
            return true;
        }
    }
    return false;
}

static bool cir2_judge(void)
{
    lose_location_test(1, clamp_row_u8(scale_from_120(20)), 0);
    int length_th = (MT9V03X_H * 3) / 4; // 0.75H
    if (length_th < 40) length_th = 40;

    int r_lose_max = (MT9V03X_H * 5) / 12;    // 120 高度时约 50
    int l_lose_min = (MT9V03X_H * 1) / 3;     // 120 高度时约 40
    if (r_lose_max < 10) r_lose_max = 10;
    if (l_lose_min < 10) l_lose_min = 10;

    if (cir_stage == 0 && nolose_num > 10 && sum_dir_r() && r_lose_num < r_lose_max && l_lose_num > l_lose_min && length > length_th && l_dir_sum[4] > 50 && r_dir_sum[4] > 65)
    {
        lose_location_test(1, (uint8)(MT9V03X_H - 1), clamp_row_u8(MAX(0, MT9V03X_H - scale_from_120(18))));
        if (nolose_num > 13)
        {
            cir_stage = 1;
            return true;
        }
    }
    return false;
}

static void l_border_cross(uint16 total_l)
{
    for (int h = 0; h < MT9V03X_H; h++)
    {
        left_border[h] = -1;
    }

    if (total_l < 3) return;

    if (flag_cross == 1)
    {
        int h2 = 0;
        uint8 count = 0;
        for (int i = (int)total_l - 1; i > 0; i--)
        {
            int h = (int)points_l[i][1];
            if (h2 > h)
            {
                continue;
            }
            h2 = 0;

            bool trigger = (dir_l[i] == 6 && (i + 1) < (int)total_l && dir_l[i + 1] != 6 && (i - 2) > 1 && h > 3);
            if (trigger)
            {
                while (count < 100 && (h + count) < MT9V03X_H)
                {
                    int x = (int)points_l[i][0] - (int)((float)count / 4.0f);
                    if (x < 2) x = 2;
                    left_border[h + count] = x;
                    count++;
                }
                h2 = h + count - 1;
                count = 0;
            }
            else
            {
                int x = (int)points_l[i][0];
                if (x > left_border[h]) left_border[h] = x;
            }
        }
    }
    else if (flag_cross == 2)
    {
        int x1 = 0;
        int x2 = 0;
        for (int i = 1; i < (int)total_l - 1; i++)
        {
            if (dir_l[i] == 2 && dir_l[i - 1] == 4 && dir_l[i + 1] == 2)
            {
                x1 = (int)points_l[i][0];
                x2 = i;
                break;
            }
        }
        for (int i = 24; i < MT9V03X_H; i++)
        {
            left_border[i] = x1 - (i - x2) / 4 - 10;
        }
    }
}

static void r_border_cross(uint16 total_r)
{
    for (int h = 0; h < MT9V03X_H; h++)
    {
        right_border[h] = 200;
    }

    if (total_r < 3) return;

    if (flag_cross == 1)
    {
        int h2 = 0;
        uint8 count = 0;
        for (int i = (int)total_r - 1; i > 2; i--)
        {
            int h = (int)points_r[i][1];
            if (h2 > h)
            {
                continue;
            }
            h2 = 0;

            bool trigger = (dir_r[i - 2] == 6 && dir_r[i - 1] == 6 && dir_r[i] == 6 && (i + 1) < (int)total_r && dir_r[i + 1] != 6 && h > 3);
            if (trigger)
            {
                while (count < 100 && (h + count) < MT9V03X_H)
                {
                    int x = (int)points_r[i][0] + (int)((float)count / 4.0f);
                    if (x > (MT9V03X_W - 2)) x = MT9V03X_W - 2;
                    right_border[h + count] = x;
                    count++;
                }
                h2 = h + count - 1;
                count = 0;
            }
            else
            {
                int x = (int)points_r[i][0];
                if (x < right_border[h]) right_border[h] = x;
            }
        }
    }
    else if (flag_cross == 2)
    {
        int x1 = 0;
        int x2 = 0;
        for (int i = 1; i < (int)total_r - 1; i++)
        {
            if (dir_r[i] == 2 && dir_r[i + 1] == 2 && (dir_r[i - 1] == 4 || dir_r[i - 1] == 5) && points_r[i][0] > (MT9V03X_W / 2))
            {
                x1 = (int)points_r[i][0];
                x2 = i;
                break;
            }
        }
        cir_x = (uint8)x1;
        cir_y = (uint8)MIN(MT9V03X_H - 1, (int)points_r[x2][1]);

        if (x1 < (MT9V03X_W / 2))
        {
            for (int i = 24; i < MT9V03X_H; i++)
            {
                right_border[i] = left_border[i] + width_from_row(i);
            }
        }
        else
        {
            for (int i = 24; i < MT9V03X_H; i++)
            {
                right_border[i] = x1 + (i - x2) / 4;
            }
        }
    }
}

static uint8 cross_judge(void)
{
    uint8 num1 = 0;
    uint8 num2 = 0;
    uint8 num3 = 0;
    uint8 num4 = 0;

    uint8 ll = l_l_judge();
    // 十字判定的 length 门槛：按高度比例放宽，避免入元素不正时触发不了
    int length_cross_th = (MT9V03X_H * 3) / 4; // 0.75H
    if (length_cross_th < 40) length_cross_th = 40;

    if (ll == 1 && length > length_cross_th)
    {
        for (uint16 i = 0; i < data_statics_l; i++)
        {
            if (i < data_statics_r && dir_r[i] == 6) num1++;
            if (dir_l[i] == 6) num2++;
        }
        if (num2 > 5 && num1 > 5 && r_dir_sum[4] > 20 && l_dir_sum[4] > 20)
        {
            return 1;
        }
    }
    if (ll == 2 && length > length_cross_th)
    {
        for (uint16 i = 0; i < data_statics_l; i++)
        {
            if (i < data_statics_r && dir_r[i] == 2) num4++;
            if (dir_l[i] == 2) num3++;
        }
        if (num3 > 5 && num4 > 4 && r_dir_sum[4] > 10 && l_dir_sum[4] > 10)
        {
            return 2;
        }
    }
    return 0;
}

static uint8 my_sqrt_u8(int16 x)
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

static float process_curvity(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3)
{
    int abc = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    int16 q1 = (int16)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    uint8 ab = my_sqrt_u8(q1);
    q1 = (int16)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    uint8 bc = my_sqrt_u8(q1);
    q1 = (int16)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    uint8 ac = my_sqrt_u8(q1);
    if ((int)ab * (int)bc * (int)ac == 0)
    {
        return 0.0f;
    }
    return (float)4.0f * (float)abc / (float)((int)ab * (int)bc * (int)ac);
}

static bool stop_line(void)
{
    int count_line = 0;
    int row = scale_from_120(110);
    if (row > (MT9V03X_H - 1)) row = (MT9V03X_H - 1);
    int col_start = scale_from_180(5);
    int col_end = scale_from_180(115);
    if (col_start < 1) col_start = 1;
    if (col_end > (MT9V03X_W - 4)) col_end = (MT9V03X_W - 4);

    for (int i = col_start; i < col_end; i++)
    {
        if (bin_image[row][i] != 0 && bin_image[row][i + 1] == 0 && bin_image[row][i + 2] == 0 && bin_image[row][i + 3] == 0)
        {
            count_line++;
        }
    }

    static uint8 stop_confirm = 0;
    if (stop_line_time == 0)
    {
        if (count_line >= 6)
        {
            if (stop_confirm < 3) stop_confirm++;
        }
        else
        {
            stop_confirm = 0;
        }
        flag_stop = (stop_confirm >= 2) ? 1 : 0;
    }
    else
    {
        flag_stop = 0;
    }

    if (flag_stop == 1 && stop_line_time == 0)
    {
        stop_line_time = 1;
    }
    if (stop_line_time != 0 && stop_line_time <= 18)
    {
        stop_line_time++;
        if (stop_line_time == 18)
        {
        }
    }
    if (stop_line_time > 18)
    {
        stop_line_time = 0;
        stop_confirm = 0;
    }

    return flag_stop ? true : false;
}

static int cross_delay_judge(void)
{
    if (cross_jud == 0 && (flag_cross == 1 || flag_cross == 2)) cross_jud = 1;
    if (cross_jud == 1 && flag_cross == 0 && flag_others2 == 1) cross_jud = 2;
    if (cross_jud == 2 && flag_others2 == 1)
    {
        cross_jud_time++;
        if (cross_jud_time >= 10)
        {
            cross_jud = 0;
            cross_jud_time = 0;
        }
    }
    if (cross_jud == 2 && flag_others2 == 0)
    {
        cross_jud = 0;
        cross_jud_time = 0;
    }
    return cross_jud;
}

static void judge_bar(void)
{
    flag_bar = 0;
    bar_count_l = 0;
    bar_count_r = 0;

    // bar 判定前置门槛：length 过低时容易在丢线/噪声/入弯姿态不正时误判。
    // 这里用高度比例阈值（而非接近满屏的 H-5），让 120 高度也能在入元素不正时触发。
    int length_bar_th = (int)(MT9V03X_H * 3 / 4);   // 0.75H
    if (length_bar_th < 40) length_bar_th = 40;

    if (length >= length_bar_th && flag_cross != 1 && flag_cross != 2 && flag_l_cir != 1 && flag_r_cir != 1 && flag_stop != 1)
    {
        int row_start = scale_from_120(10);
        int row_end = scale_from_120(110);
        if (row_start < 0) row_start = 0;
        if (row_end > (MT9V03X_H - 4)) row_end = (MT9V03X_H - 4);

        for (int i = row_start; i < row_end; i++)
        {
            if (zf_abs_i(right_border[i] - right_border[i + 3]) > 5) bar_count_r++;
            if (zf_abs_i(left_border[i] - left_border[i + 3]) > 5) bar_count_l++;
        }
        if (bar_count_l >= 6) flag_bar = 1;
        if (bar_count_r >= 7) flag_bar = 2;
    }
}



/**
 * @brief 过滤图像噪点，宏定义中可修改过滤范围
 */
static void image_filter(uint8 img[MT9V03X_H][MT9V03X_W])
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
/**
 * @brief 图像左右处绘制黑边界
 * 
 * @details 对二值化图像的边界进行清零处理，也就是把图像的最外圈（四周几行/几列）全部置为黑色（0），防止边缘噪点或误判影响后续巡线和元素识别。
 */
static void image_draw_rectan(uint8 img[MT9V03X_H][MT9V03X_W])
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

static void cir_r_handle(void)
{
    switch (cir_stage)
    {
        case 1:
            lose_location_test(2, 100, 20);
            if (lose_num > 10)
            {
                lose_location_test(2, 40, 0);
                if (nolose_num >= 10)
                {
                    cir_stage = 2;
                }
                lose_location_test(2, 30, 0);
                if (lose_num >= 10)
                {
                    cir_stage = 2;
                }
            }
            break;
        case 2:
            lose_location_test(2, 60, 30);
            if (nolose_num > 15)
            {
                lose_location_test(2, 30, 0);
                if (lose_num > 15)
                {
                    cir_stage = 3;
                }
            }
            break;
        case 3:
            lose_location_test(2, 60, 30);
            if (lose_num > 15)
            {
                lose_location_test(2, 30, 0);
                if (nolose_num > 25)
                {
                    cir_stage = 4;
                }
            }
            break;
        case 4:
            lose_location_test(2, 75, 25);
            if (lose_num > 20)
            {
                lose_location_test(1, 75, 25);
                if (lose_num > 20)
                {
                    cir_stage = 7;
                }
            }
            break;
        case 5:
            lose_location_test(2, 75, 25);
            if (lose_num > 20)
            {
                lose_location_test(1, 75, 25);
                if (lose_num > 20)
                {
                    cir_stage = 7;
                }
            }
            break;
        case 6:
            lose_location_test(2, 75, 25);
            if (lose_num > 20)
            {
                lose_location_test(1, 75, 25);
                if (lose_num > 20)
                {
                    cir_stage = 7;
                }
            }
            break;
        case 7:
            lose_location_test(2, 20, 0);
            if (nolose_num > 15)
            {
                cir_stage = 8;
            }
            break;
        case 8:
            lose_location_test(2, 20, 0);
            if (nolose_num > 15)
            {
                lose_location_test(1, 60, 0);
                if (nolose_num > 50)
                {
                    cir_stage = 9;
                }
            }
            break;
        case 9:
            lose_location_test(2, 60, 0);
            if (nolose_num > 50)
            {
                lose_location_test(1, 60, 0);
                if (nolose_num > 50)
                {
                    flag_r_cir = 0;
                    cir_stage = 0;
                }
            }
            break;
        default:
            break;
    }

    if (cir_stage == 1)
    {
        for (int i = 0; i < MT9V03X_H; i++)
        {
            left_border[i] = left_border[i] + 25;
            right_border[i] = left_border[i] + width_from_row(i) + 20;
        }
    }
    else if (cir_stage == 2 || cir_stage == 3)
    {
        cir_point(2);
        for (int i = 0; i < MT9V03X_H; i++)
        {
            left_border[i] = left_border[i] + 4;
            right_border[i] = left_border[i] + width_from_row(i) + 25;
        }
    }
    else if (cir_stage == 4)
    {
        cir_point(2);
        float count = 0.0f;
        for (int i = cir_y; i < MT9V03X_H; i++)
        {
            right_border[i] = MT9V03X_W - 5;
            left_border[i] = (int16)((float)cir_x - count);
            count += 1.0f;
        }
        count = 0.0f;
        for (int i = cir_y; i > 10; i--)
        {
            right_border[i] = MT9V03X_W - 5;
            left_border[i] = (int16)((float)cir_x + count);
            count += 1.0f;
        }
    }
    else if (cir_stage == 7)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            right_border[i] = MT9V03X_W - 5;
            left_border[i] = right_border[i] - width_from_row(i) - 30;
        }
    }
    else if (cir_stage == 9)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            left_border[i] = left_border[i] + 5;
            right_border[i] = left_border[i] + width_from_row(i);
        }
    }
}

static void cir_l_handle(void)
{
    switch (cir_stage)
    {
        case 1:
            lose_location_test(1, 50, 20);
            if (lose_num > 15)
            {
                lose_location_test(1, 20, 0);
                if (nolose_num >= 10)
                {
                    cir_stage = 2;
                }
            }
            break;
        case 2:
            lose_location_test(1, 40, 20);
            if (nolose_num > 15)
            {
                lose_location_test(1, 20, 0);
                if (lose_num > 15)
                {
                    cir_stage = 3;
                }
            }
            break;
        case 3:
            lose_location_test(1, 70, 30);
            if (lose_num > 15)
            {
                lose_location_test(1, 40, 0);
                if (nolose_num > 15)
                {
                    cir_stage = 4;
                }
            }
            break;
        case 4:
            lose_location_test(1, 89, 5);
            if (lose_num > 10)
            {
                lose_location_test(2, 89, 25);
                if (lose_num > 10)
                {
                    cir_stage = 7;
                }
            }
            break;
        case 5:
            lose_location_test(1, 89, 5);
            if (lose_num > 10)
            {
                lose_location_test(2, 89, 25);
                if (lose_num > 10)
                {
                    cir_stage = 7;
                }
            }
            break;
        case 6:
            lose_location_test(1, 89, 5);
            if (lose_num > 10)
            {
                lose_location_test(2, 89, 25);
                if (lose_num > 10)
                {
                    cir_stage = 7;
                }
            }
            break;
        case 7:
            lose_location_test(1, 20, 0);
            if (nolose_num > 10)
            {
                cir_stage = 8;
            }
            break;
        case 8:
            lose_location_test(1, 20, 0);
            if (nolose_num > 10)
            {
                lose_location_test(2, 60, 10);
                if (nolose_num > 45)
                {
                    cir_stage = 9;
                }
            }
            break;
        case 9:
            lose_location_test(1, 60, 0);
            if (nolose_num > 50)
            {
                lose_location_test(2, 60, 0);
                if (nolose_num > 50)
                {
                    flag_l_cir = 0;
                    cir_stage = 0;
                }
            }
            break;
        default:
            break;
    }

    if (cir_stage == 1)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            right_border[i] = right_border[i] - 1;
            left_border[i] = right_border[i] - width_from_row(i) - 30;
        }
    }
    else if (cir_stage == 2 || cir_stage == 3)
    {
        cir_point(1);
        for (int i = 0; i < MT9V03X_H; i++)
        {
            right_border[i] = right_border[i] - 1;
            left_border[i] = right_border[i] - width_from_row(i) - 30;
        }
    }
    else if (cir_stage == 4)
    {
        for (int i = 5; i < MT9V03X_H - 11; i++)
        {
            left_border[i] = 5;
            right_border[i] = left_border[i] + width_from_row(i) + 15;
        }
    }
    else if (cir_stage == 7)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            right_border[i] = left_border[i] + width_from_row(i) + 5;
        }
    }
    else if (cir_stage == 9)
    {
        for (int i = 0; i < MT9V03X_H - 11; i++)
        {
            right_border[i] = right_border[i] - 1;
            left_border[i] = right_border[i] - width_from_row(i) - 5;
        }
    }
}

static void element_handle(void)
{
    black_jiexian = 0;

    if (flag_r_cir == 1)
    {
        cir_r_handle();
        for (int i = 0; i < MT9V03X_H; i++)
        {
            center_line_i16[i] = (left_border[i] + right_border[i]) / 2;
        }
        if (cir_stage == 7 && MT9V03X_H >= 2)
        {
            center_line_i16[MT9V03X_H - 1] = center_line_i16[MT9V03X_H - 2];
        }
        return;
    }

    if (flag_l_cir == 1)
    {
        cir_l_handle();
        for (int i = 0; i < MT9V03X_H; i++)
        {
            center_line_i16[i] = (left_border[i] + right_border[i]) / 2;
        }
        if (cir_stage == 7 && MT9V03X_H >= 2)
        {
            center_line_i16[MT9V03X_H - 1] = center_line_i16[MT9V03X_H - 2];
        }
        return;
    }

    if (flag_cross == 1)
    {
        l_border_cross(data_statics_l);
        r_border_cross(data_statics_r);
        for (int i = 0; i < MT9V03X_H; i++)
        {
            center_line_i16[i] = (left_border[i] + right_border[i]) / 2;
        }
        return;
    }


    for (int i = MT9V03X_H - 1; i >= 1; i--)
    {
        if (black_jiexian == 0)
        {
            if (left_border[i] == -1 || right_border[i] == 200)
            {
                black_jiexian = i;
            }
        }
    }
}

static void element_test(void)
{
    int y20 = MAX(0, MT9V03X_H - 20);
    int y35 = MAX(0, MT9V03X_H - 35);
    int y50 = MAX(0, MT9V03X_H - 50);

    uint8 lx1 = (uint8)MAX(0, MIN(MT9V03X_W - 1, left_border[y20]));
    uint8 lx2 = (uint8)MAX(0, MIN(MT9V03X_W - 1, left_border[y35]));
    uint8 lx3 = (uint8)MAX(0, MIN(MT9V03X_W - 1, left_border[y50]));
    uint8 rx1 = (uint8)MAX(0, MIN(MT9V03X_W - 1, right_border[y20]));
    uint8 rx2 = (uint8)MAX(0, MIN(MT9V03X_W - 1, right_border[y35]));
    uint8 rx3 = (uint8)MAX(0, MIN(MT9V03X_W - 1, right_border[y50]));

    l_line_qulv = (int16)(1000.0f * process_curvity(lx1, (uint8)y20, lx2, (uint8)y35, lx3, (uint8)y50));
    r_line_qulv = (int16)(1000.0f * process_curvity(rx1, (uint8)y20, rx2, (uint8)y35, rx3, (uint8)y50));

    warn_sink_i16 = (int16)(l_line_qulv + r_line_qulv);

    lose_line();

    if (cir1_judge() && flag_cross != 1)
    {
        flag_r_cir = 1;
        flag_cross = 0;
        flag_others2 = 0;
    }
    else if (cir2_judge() && flag_cross != 1 && flag_r_cir != 1)
    {
        flag_l_cir = 1;
        flag_cross = 0;
        flag_others2 = 0;
    }
    else
    {
        uint8 cj = cross_judge();
        if (cj == 1)
        {
            flag_cross = 1;
            flag_others2 = 0;
        }
        else if (cj == 2)
        {
            flag_cross = 0;
            flag_others2 = 0;
        }
        else
        {
            flag_cross = 0;
            flag_others2 = 1;
        }
    }
}

static uint8 straight_judge(void)
{
    int length_straight_th = (MT9V03X_H * 3) / 4; // 0.75H
    if (length_straight_th < 40) length_straight_th = 40;

    if (length > length_straight_th && flag_l_cir != 1 && flag_r_cir != 1)
    {
        int i_begin = scale_from_120(13);
        int i_end = scale_from_120(20);
        int i_offset = scale_from_120(50);
        if (i_begin < 0) i_begin = 0;
        if (i_end > MT9V03X_H) i_end = MT9V03X_H;
        if (i_offset < 1) i_offset = 1;

        for (int i = i_begin; i < i_end; i++)
        {
            int i2 = i + i_offset;
            if (i2 >= MT9V03X_H) break;
            int distance = zf_abs_i(center_line_i16[i] - center_line_i16[i2]);
            if (distance > 5)
            {
                flag_str = false;
                return 0;
            }
        }
        flag_str = true;
        return 1;
    }
    flag_str = false;
    return 0;
}

static uint8 straight_judge_2(void)
{
    int length_straight_strict_th = (MT9V03X_H * 4) / 5; // 0.8H
    if (length_straight_strict_th < 50) length_straight_strict_th = 50;

    if (length > length_straight_strict_th && flag_l_cir != 1 && flag_r_cir != 1)
    {
        int i_begin = scale_from_120(13);
        int i_end = scale_from_120(20);
        int i_offset = scale_from_120(50);
        if (i_begin < 0) i_begin = 0;
        if (i_end > MT9V03X_H) i_end = MT9V03X_H;
        if (i_offset < 1) i_offset = 1;

        for (int i = i_begin; i < i_end; i++)
        {
            int i2 = i + i_offset;
            if (i2 >= MT9V03X_H) break;
            int distance = zf_abs_i(center_line_i16[i] - center_line_i16[i2]);
            if (distance > 2)
            {
                flag_str_str = false;
                return 0;
            }
        }
        flag_str_str = true;
        return 1;
    }
    flag_str_str = false;
    return 0;
}


/**
 * @brief  仅控制舵机转向的函数，不控制电机
 */
void steering_control_servo_only(void)
{
    imu660rb_get_gyro();
    steering_pid_calculate();
    g_stop_active = (stop_line_time != 0);

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
    const float black_pixel_threshold = 0.82f;

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


/**
 * @brief 将参数保存到flash中永久保存
 */
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
    flash_union_buffer[11].float_type = g_ackermann_scale_limit;
    flash_union_buffer[12].int32_type = (int32_t)g_dynamic_threshold;
    flash_union_buffer[13].int32_type = (int32_t)g_servo_min_duty;
    flash_union_buffer[14].int32_type = (int32_t)g_servo_max_duty;
    flash_union_buffer[15].int32_type = (int32_t)g_servo_center_duty;
    flash_union_buffer[16].int32_type = g_max_target_speed;
    flash_union_buffer[17].int32_type = g_speed_pid_integral_threshold;
    flash_union_buffer[18].float_type = g_speed_pid_integral_max;
    flash_union_buffer[19].int32_type = g_track_width_top;
    flash_union_buffer[20].int32_type = g_track_width_bottom;

    // 直线策略相关
    flash_union_buffer[21].int32_type = (int32_t)g_straight_speed_add;
    flash_union_buffer[22].float_type = g_straight_gyro_thr_dps;
    flash_union_buffer[23].float_type = g_straight_img_err_thr;

    // 图像误差行范围
    flash_union_buffer[24].int32_type = g_img_error_row_start;
    flash_union_buffer[25].int32_type = g_img_error_row_end;

    flash_write_page_from_buffer(0, PARAMS_FLASH_PAGE_INDEX, PARAMS_LEN);
    ips200_show_string(0, 180, "Parameters Saved!");
    system_delay_ms(500);
}


/**
 * @brief 从flash中加载数据到实际参数
 */
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
    g_ackermann_scale_limit = flash_union_buffer[11].float_type;
    if (g_ackermann_scale_limit < 0.0f) g_ackermann_scale_limit = 0.0f;
    if (g_ackermann_scale_limit > 3.0f) g_ackermann_scale_limit = 3.0f;
    g_dynamic_threshold   = (int)flash_union_buffer[12].int32_type;

    g_servo_min_duty     = (uint32)flash_union_buffer[13].int32_type;
    g_servo_max_duty     = (uint32)flash_union_buffer[14].int32_type;
    g_servo_center_duty  = (uint32)flash_union_buffer[15].int32_type;

    g_max_target_speed               = flash_union_buffer[16].int32_type;
    g_speed_pid_integral_threshold   = flash_union_buffer[17].int32_type;
    g_speed_pid_integral_max         = flash_union_buffer[18].float_type;

    g_track_width_top                = flash_union_buffer[19].int32_type;
    g_track_width_bottom             = flash_union_buffer[20].int32_type;

    // 直线策略相关
    g_straight_speed_add    = (int16)flash_union_buffer[21].int32_type;
    g_straight_gyro_thr_dps = flash_union_buffer[22].float_type;
    g_straight_img_err_thr  = flash_union_buffer[23].float_type;

    // 图像误差行范围
    g_img_error_row_start   = flash_union_buffer[24].int32_type;
    g_img_error_row_end     = flash_union_buffer[25].int32_type;
}