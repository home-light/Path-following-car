#include "stm32f10x.h"
#include "motor.h"
#include "oled.h"
#include "delay.h"
#include "sys.h"
short Show_Speed_A = 0;
short Show_Speed_B = 0;
int cross_timer = 0; // 十字路口直行计时
extern PID_Control pid_A; 
extern PID_Control pid_B;
// 2. 循迹PID参数 (外环)
//float Track_Kp = 25.0f; 
//float Track_Kd = 19.5f; 
//float Last_Track_Error = 0;
//float Base_Speed =23.5f;
float Track_Kp = 1200.0f;  // 调整这个值决定转向灵敏度
float Track_Kd = 1100.0f;   // 调整这个值决定抑制震荡的能力
float Last_Error = 0;
int Base_PWM = 3000;       // 基础速度占空比 (根据电压调整)
int Cross_Timer = 0;       // 十字路口计时器
void TIM4_IRQHandler(void) 
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
			// 1. 获取编码器速度（注意方向，如果反了加负号）
        Show_Speed_A = -(short)TIM_GetCounter(TIM3);
        TIM_SetCounter(TIM3, 0);
        Show_Speed_B = (short)TIM_GetCounter(TIM1);
        TIM_SetCounter(TIM1, 0);
        float error = Get_Sensor_Error();
        int pwm_out_a, pwm_out_b;
// --- 逻辑 1: 十字路口处理 ---
        if (error == 100.0f) {
            Cross_Timer = 20; // 发现十字路口，设定 200ms 的强制直行时间
        }
        if (Cross_Timer > 0) {
            // 十字路口期间屏蔽转向，匀速冲过横线
            pwm_out_a = Base_PWM; 
            pwm_out_b = Base_PWM;
            Cross_Timer--;
        }
        // --- 逻辑 2: 直角弯处理 (原地旋转) ---
        else if (error >= 50.0f || error <= -50.0f) {
            int Turn_Speed = 4800; // 原地转向的力度
            if (error <= -50.0f) { // 左转弯
                pwm_out_a = -Turn_Speed; 
                pwm_out_b = Turn_Speed;
            } 
            else {                 // 右转弯
                pwm_out_a = Turn_Speed;
                pwm_out_b = -Turn_Speed;
            }
        } 
        // --- 逻辑 3: 正常 PID 循迹 ---
        else{
            float PID_val = Track_Kp * error + Track_Kd * (error - Last_Error);
            pwm_out_a = Base_PWM + (int)PID_val; 
            pwm_out_b = Base_PWM - (int)PID_val;
            Last_Error = error;
        }
        // 3. 执行输出
        Load_PWM(pwm_out_a, pwm_out_b);
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}
// --- 系统初始化 ---
void System_All_Init(void) 
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();           
// 初始化延时函数
    OLED_Init();            
// 你的驱动里的初始化
    Motor_Init();           
// 之前配置的 PWM 和 GPIO
    Encoder_Init_TIM3_TIM1(); 
// 之前配置的 TIM3/TIM1 编码器模式

    // 配置 TIM4 为 10ms 控制中断
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_Base;
    TIM_Base.TIM_Period = 999; 
    TIM_Base.TIM_Prescaler =719; 
    TIM_TimeBaseInit(TIM4, &TIM_Base);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_Str;
    NVIC_Str.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_Str.NVIC_IRQChannelPreemptionPriority =0;
    NVIC_Str.NVIC_IRQChannelSubPriority =0;
    NVIC_Str.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_Str);   
    TIM_Cmd(TIM4, ENABLE);
}
int main(void)
 {	
	  u8 sensor_str[9]; // 用于存储8位状态字符串 + 结束符 '\0'
		System_All_Init();
    OLED_Clear();
	 Sensor_Init();
	 	// x, y, num, len, size
		OLED_ShowString(0, 0, (u8*)"Motor Status:");
    OLED_ShowString(0, 2, (u8*)"Spd A:");
    OLED_ShowString(0, 4, (u8*)"Spd B:");
;
  while (1)
  {
				        // 1. 依次读取8个传感器的电平 (假设检测到黑线为1，没检测到为0)
        // 顺序：L4(P14) L3(P13) L2(P12) L1(P9) R1(P8) R2(P5) R3(P4) R4(P3)
        sensor_str[0] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) ? '1' : '0';
        sensor_str[1] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) ? '1' : '0';
        sensor_str[2] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ? '1' : '0';
        sensor_str[3] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)  ? '1' : '0';
        sensor_str[4] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)  ? '1' : '0';
        sensor_str[5] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)  ? '1' : '0';
        sensor_str[6] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)  ? '1' : '0';
        sensor_str[7] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)  ? '1' : '0';
        sensor_str[8] = '\0'; // 字符串结束符
        OLED_ShowString(0, 6, sensor_str);
        OLED_ShowNum(60, 2, (Show_Speed_A >= 0 ? Show_Speed_A : -Show_Speed_A), 3, 16);
        OLED_ShowNum(60, 4, (Show_Speed_B >= 0 ? Show_Speed_B : -Show_Speed_B), 3, 16);       
        // 显示方向标志 (如果是负数显示 '-', 正数显示 ' ')
        OLED_ShowChar(52, 2, (Show_Speed_A >= 0 ? ' ' : '-'));
				delay_ms(10);
  }
 }
void Set_Speed(int speed_a, int speed_b)
{
    pid_A.Target = (float)speed_a;
    pid_B.Target = (float)speed_b;
}