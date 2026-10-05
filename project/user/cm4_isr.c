#include "zf_common_headfile.h"

extern volatile int16 g_target_speed_l;
extern volatile int16 g_target_speed_r;
extern int16 g_actual_speed_l;
extern int16 g_actual_speed_r;
extern volatile int32 g_encoder_delta_accum;
extern float_t g_motor_out_l;
extern float_t g_motor_out_r;
extern float_t g_max_pwm_duty;
extern float_t g_motor_out_l_duty;
extern float_t g_motor_out_r_duty;
extern float_t g_min_start_duty;
extern volatile bool g_track_lost;
extern bool g_start_flag;
extern volatile bool g_run_enable;

extern PID_t g_speed_pid_l;
extern PID_t g_speed_pid_r;
extern PID_t g_steering_pid;
extern float g_final_error;
extern int16 g_base_speed;
extern float g_servo_angle;
extern float g_servo_to_angle_gain;
extern float g_speed_steer_scale;
extern volatile bool g_stop_active;

extern volatile menu_state_t g_menu_state;

void speed_pid_calculate(PID_t *pid, int16 target_speed, int16 actual_speed);
void steering_control(void);
void steering_control_servo_only(void);

#define MOTOR_L_PWM_PIN         (TCPWM_CH13_P00_3)
#define MOTOR_R_PWM_PIN         (TCPWM_CH50_P18_7)
#define ENCODER_L               (TC_CH09_ENCODER)
#define ENCODER_R               (TC_CH07_ENCODER)

// **************************** PIT中断函数 ****************************
void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);

    if ((g_menu_state == MENU_RUN_MAIN_PROGRAM && !g_run_enable) ||
        (g_menu_state != MENU_RUN_MAIN_PROGRAM && g_menu_state != MENU_SPEED_DEBUG))
    {
        pwm_set_duty(MOTOR_L_PWM_PIN, 0);
        pwm_set_duty(MOTOR_R_PWM_PIN, 0);
        
        g_motor_out_l = 0;
        g_motor_out_r = 0;

        g_speed_pid_l.output = 0;
        g_speed_pid_l.error = 0;
        g_speed_pid_l.last_error = 0;
        g_speed_pid_l.pre_last_error = 0;

        g_speed_pid_r.output = 0;
        g_speed_pid_r.error = 0;
        g_speed_pid_r.last_error = 0;
        g_speed_pid_r.pre_last_error = 0;
        encoder_clear_count(ENCODER_L);
        encoder_clear_count(ENCODER_R);
        g_encoder_delta_accum = 0;
        return;
    }
    if(g_track_lost)
    {
        g_motor_out_l = 0;
        g_motor_out_r = 0;
        g_motor_out_l_duty = 0;
        g_motor_out_r_duty = 0;
        pwm_set_duty(MOTOR_L_PWM_PIN, 0);
        pwm_set_duty(MOTOR_R_PWM_PIN, 0);

        g_speed_pid_l.output = 0;
        g_speed_pid_l.error = 0;
        g_speed_pid_l.last_error = 0;
        g_speed_pid_l.pre_last_error = 0;

        g_speed_pid_r.output = 0;
        g_speed_pid_r.error = 0;
        g_speed_pid_r.last_error = 0;
        g_speed_pid_r.pre_last_error = 0;
        encoder_clear_count(ENCODER_L);
        encoder_clear_count(ENCODER_R);
        g_encoder_delta_accum = 0;
        return;
    }

    if (g_stop_active)
    {
        g_target_speed_l = 0;
        g_target_speed_r = 0;

        g_motor_out_l = 0;
        g_motor_out_r = 0;
        g_motor_out_l_duty = 0;
        g_motor_out_r_duty = 0;
        pwm_set_duty(MOTOR_L_PWM_PIN, 0);
        pwm_set_duty(MOTOR_R_PWM_PIN, 0);

        g_speed_pid_l.output = 0;
        g_speed_pid_l.error = 0;
        g_speed_pid_l.last_error = 0;
        g_speed_pid_l.pre_last_error = 0;
        g_speed_pid_l.integral = 0;

        g_speed_pid_r.output = 0;
        g_speed_pid_r.error = 0;
        g_speed_pid_r.last_error = 0;
        g_speed_pid_r.pre_last_error = 0;
        g_speed_pid_r.integral = 0;

        encoder_clear_count(ENCODER_L);
        encoder_clear_count(ENCODER_R);
        g_encoder_delta_accum = 0;
        return;
    }


    g_actual_speed_l = -encoder_get_count(ENCODER_L);
    encoder_clear_count(ENCODER_L);
    g_actual_speed_r = encoder_get_count(ENCODER_R);
    encoder_clear_count(ENCODER_R);

    int32 dl = (g_actual_speed_l >= 0) ? g_actual_speed_l : -g_actual_speed_l;
    int32 dr = (g_actual_speed_r >= 0) ? g_actual_speed_r : -g_actual_speed_r;
    g_encoder_delta_accum += (dl + dr) / 2;

    speed_pid_calculate(&g_speed_pid_l, g_target_speed_l, g_actual_speed_l);
    speed_pid_calculate(&g_speed_pid_r, g_target_speed_r, g_actual_speed_r);

    g_motor_out_l += g_speed_pid_l.output;
    g_motor_out_r += g_speed_pid_r.output;

    float max_pwm_val = g_max_pwm_duty * 100.0f;

    if(g_motor_out_l > max_pwm_val)
    {
        g_motor_out_l = max_pwm_val;
    }
    else if(g_motor_out_l < 0)
    {
        g_motor_out_l = 0;
    }

    if(g_motor_out_r > max_pwm_val)
    {
        g_motor_out_r = max_pwm_val;
    }
    else if(g_motor_out_r < 0)
    {
        g_motor_out_r = 0;
    }

    g_motor_out_l_duty = g_motor_out_l / 100.0f;
    g_motor_out_r_duty = g_motor_out_r / 100.0f;

    pwm_set_duty(MOTOR_L_PWM_PIN, (uint32)g_motor_out_l);
    pwm_set_duty(MOTOR_R_PWM_PIN, (uint32)g_motor_out_r);
}

void pit0_ch1_isr()
{
    pit_isr_flag_clear(PIT_CH1);

    if (g_menu_state == MENU_RUN_MAIN_PROGRAM)
    {
        if (!g_run_enable)
        {
            return;
        }
        if (!g_track_lost)
        {
            steering_control();
        }
        return;
    }

    if (g_menu_state == MENU_THRESHOLD_DEBUG)
    {
        steering_control_servo_only();
        return;
    }
}

void pit0_ch2_isr()                     // 定时器通道 2 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH2);
    
}

void pit0_ch10_isr()                    // 定时器通道 10 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH10);
    
}

void pit0_ch11_isr()                    // 定时器通道 11 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // 定时器通道 12 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // 定时器通道 13 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // 定时器通道 14 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // 定时器通道 15 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // 定时器通道 16 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // 定时器通道 17 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // 定时器通道 18 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // 定时器通道 19 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // 定时器通道 20 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // 定时器通道 21 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH21);
    
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {
        
#if DEBUG_UART_USE_INTERRUPT             // 如果开启 debug 串口中断
        debug_interrupr_handler();       // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                   // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
      
    }
    else                                 // 串口0发送中断
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {
        
        
      
      
    }
    else                                // 串口1发送中断
    {
      
        
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {
        
      
        
        
    }
    else                                // 串口2发送中断
    {
        
        
        
       
    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {
        

        
        
    }
    else                                // 串口3发送中断
    {
      
        
        
        
    }
}

void uart4_isr (void)
{
    
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {
        

        
       
    }
    else                                // 串口4发送中断
    {
      
        
        
        
    }
}

void uart5_isr (void)
{
    
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {
        

        
       
    }
    else                                // 串口5发送中断
    {
      
        
        
        
    }
}

void uart6_isr (void)
{
    
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {
        

        
       
    }
    else                                // 串口6发送中断
    {
      
        
        
        
    }
}
// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断服务函数     
{
    
  
  
}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断服务函数     
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断服务函数     
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断服务函数     
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断服务函数     
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断服务函数     
{



}

void gpio_6_exti_isr()                  // 外部 GPIO_6 中断服务函数     
{
    


}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断服务函数     
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断服务函数     
{
    


}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断服务函数     
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断服务函数     
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断服务函数     
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断服务函数     
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断服务函数     
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断服务函数     
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断服务函数     
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断服务函数     
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断服务函数     
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断服务函数     
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断服务函数     
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断服务函数     
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断服务函数     
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断服务函数     
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断服务函数     
{



}