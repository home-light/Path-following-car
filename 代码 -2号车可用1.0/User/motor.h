#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"

typedef struct {
    float Target;    
    float Kp, Ki, Kd;
    float Error, Last_Error, Integral;  
    int Output;      
} PID_Control;

extern float Track_Kp, Track_Kd, Last_Track_Error;

void Motor_Init(void);
void Encoder_Init_TIM3_TIM1(void);
void Load_PWM(int pwm_a, int pwm_b);
int PID_Speed_Compute(PID_Control *pid, int measured);
float Get_Sensor_Error(void); 
void Sensor_Init(void);
#endif