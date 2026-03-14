#include "motor.h"

// 速度环PID初始化 (需要根据实际空载333转/分进行微调)
PID_Control pid_A = {0.0f, 15.0f, 0.8f, 2.0f, 0, 0, 0, 0}; 
PID_Control pid_B = {0.0f, 15.0f, 0.8f, 2.0f, 0, 0, 0, 0};

void Motor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // PWMA(PA0), PWMB(PA1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // AIN1(PA5), AIN2(PA4), BIN1(PA10), BIN2(PA11)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PWM定时器配置 (20KHz)
    TIM_TimeBaseStructure.TIM_Period = 7199; 
    TIM_TimeBaseStructure.TIM_Prescaler = 0; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PWMA
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PWMB
    
    TIM_Cmd(TIM2, ENABLE);
}

// 8路传感器误差计算 (0为白底，1为黑线)
void Sensor_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 释放 JTAG 占用 (PB3, PB4)
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | 
                                 GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_12 | 
                                 GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
// 上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 定义静态变量记忆方向，防止在直角顶点全白时丢失目标
static float Last_Valid_Error = 0;
float Get_Sensor_Error(void) 
{
    // 读取 8 路状态 (1 表示黑线)
    int L3 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14); 
    int L2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
    int L1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
    int M1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);  // 注意：此处对应 main.c 中的读取顺序
    int M2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);
    int R1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
    int R2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
    int R3 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);

    int count = L3 + L2 + L1 + M1 + M2 + R1 + R2 + R3;
    if(L3&&R3) return 100.0f; // 返回十字路口标志位
    // --- 1. 直角弯特征检测 ---
    // 如果最左边三个传感器同时亮，判定为左直角弯
    if (L3 && L2) return -50.0f; 
    // 如果最右边三个传感器同时亮，判定为右直角弯
    if (R3&&R2) return 50.0f;

    // --- 2. 丢线处理 (防止直角转过头或转一半丢失) ---
    if (count == 0)  return 0; 
    // --- 3. 正常加权计算 ---
    float weight_sum = (L3 * -4.0f) + (L2 * -2.5f) + (L1 * -1.0f) + (R1 * 1.0f) + (R2 * 2.5f) + (R3 * 4.0f);
    float current_error = weight_sum / (float)count;
    
    Last_Valid_Error = current_error;
    return current_error;
}
int PID_Speed_Compute(PID_Control *pid, int measured) {
    pid->Error = pid->Target - measured;
    pid->Integral += pid->Error;
    if(pid->Integral > 5000) pid->Integral = 5000; // 抗饱和
    if(pid->Integral < -5000) pid->Integral = -5000;
    
    pid->Output = (int)(pid->Kp * pid->Error + pid->Ki * pid->Integral + pid->Kd * (pid->Error - pid->Last_Error));
    pid->Last_Error = pid->Error;
    
    if(pid->Output > 7100) pid->Output = 7100;
    if(pid->Output < -7100) pid->Output = -7100;
    return pid->Output;
}

void Load_PWM(int pwm_a, int pwm_b) {
	// 限制最大占空比，防止溢出
    if (pwm_a > 7199) pwm_a = 7199;
    if (pwm_a < -7199) pwm_a = -7199;
	 if (pwm_b > 7199) pwm_b = 7199;
    if (pwm_b < -7199) pwm_b = -7199;
    if(pwm_a >= 0) { 
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		}
    else{ 
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		pwm_a = -pwm_a;
		}
    
    if(pwm_b >= 0) {
		GPIO_ResetBits(GPIOA, GPIO_Pin_10); 
		GPIO_SetBits(GPIOA, GPIO_Pin_11); 
		}
    else { 
		GPIO_SetBits(GPIOA, GPIO_Pin_10);
		GPIO_ResetBits(GPIOA, GPIO_Pin_11);
		pwm_b = -pwm_b; 
		}
    
    TIM_SetCompare1(TIM2, pwm_a);
    TIM_SetCompare2(TIM2, pwm_b);
}
/**
 * @brief  初始化编码器模式
 * 电机1编码器: PA6 (TIM3_CH1), PA7 (TIM3_CH2)
 * 电机2编码器: PA8 (TIM1_CH1), PA9 (TIM1_CH2)
 */
void Encoder_Init_TIM3_TIM1(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    // 1. 时钟使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // TIM3 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA, ENABLE); // TIM1 和 GPIOA 时钟

    // 2. GPIO 配置：PA6, PA7, PA8, PA9 均设为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 定时器基础配置 (设置计数重装载值为最大 65535)
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // 4. 配置编码器模式 (使用双相计数，即4倍频)
    // TIM_EncoderMode_TI12 表示在 TI1 和 TI2 的上升沿及下降沿都计数
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 5. 输入捕获滤波器配置 (可选，增加抗干扰能力)
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10; // 滤波值 0-15
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    // 6. 清除计数器并使能定时器
    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM1, 0);
    
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}