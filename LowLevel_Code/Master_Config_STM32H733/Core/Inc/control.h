/*
 * Control.h
 *
 *  Created on: Jan 26, 2022
 *      Author: thansak pongpraket
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

/*
 * Include Driver H7
 */
#include "stm32h7xx.h"
//#include "arm_math.h"

#define delta_t 0.001
#define delta_tPow2 (delta_t * delta_t)
#define delta_tPow3 (delta_tPow2 * delta_t)
#define delta_tPow4 (delta_tPow3 * delta_t)
#define PI               3.14159265358979f

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
	uint16_t f_timer;
	uint8_t DIR_init;
}SteperParameter;

/* KalmanFilter Variable */
typedef struct {
	float Q; 			// Adjustable
	float R; 			// Adjustable
	float x1;			// Estimate Position
	float x2;			// Estimate Velocity
	float p11;
	float p12;
	float p21;
	float p22;
}KalmanParameter;


typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float ITerm;
	float Setpoint;
	float Feedback;
	float Error[2];		//Error[0] -> Error @ t , Error[1] = -> Error @ t-1
	float Output;
}PIDParameter;


typedef struct {
	PIDParameter Pos;
	float PositionSetpoint; 				/* Rad	  Input */
	float PositionFeedback;					/* Position */
	float PositionPIDOutput;
	PIDParameter Vel;
	float VelocitySetpoint; 				/* Rad/s  Input (Command Velocity For Feed Forward) */
	float VelocityFeedback;					/* Velocity Output form State Estimator(Kaman Filter)*/
	float VelocityPIDOutput;
	float Vel_Gfeed;						/* Velocity Feedforward Gain */
	float Output;
} ControlParameter;

void Step_Driver_init(SteperParameter *step, TIM_HandleTypeDef *htim, uint32_t Channel, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t f_timer,uint8_t DIR_init);
void Step_Driver(SteperParameter *step, float f_driver);
void Kalman_init(KalmanParameter *kalman, double Q, double R);
void KalmanFilter(KalmanParameter *kalman ,double theta_k);
void PID_init(PIDParameter *PID, float Kp, float Ki, float Kd);
float PID_Control(PIDParameter *PID,float Setpoint,float Feedback);
void CascadeControl_init(ControlParameter *Control,float PosP,float PosI,float PosD,float VelP,float VelI,float VelD, float GearRatio ,float StepDriver);
void CascadeControl(ControlParameter *Control, KalmanParameter *kalman,	float Pos_Feed, float pos_set, float vel_set);

#endif /* INC_CONTROL_H_ */
