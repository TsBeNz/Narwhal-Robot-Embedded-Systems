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

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
}ServoParameter;

/* KalmanFilter Variable */
typedef struct {
	double Q; 			// Adjustable
	double R; 			// Adjustable
	double x1;			// Estimate Position
	double x2;			// Estimate Velocity
	double p11;
	double p12;
	double p21;
	double p22;
}KalmanParameter;


typedef struct {
	double Kp;
	double Ki;
	double Kd;
	double ITerm;
	double Setpoint;
	double Feedback;
	double Error[2];		//Error[0] -> Error @ t , Error[1] = -> Error @ t-1
	double Output;
}PIDParameter;


typedef struct {
	PIDParameter Pos;
	double PositionSetpoint; 				/* Rad	  Input */
	double PositionFeedback;					/* Position */
	double PositionPIDOutput;
	PIDParameter Vel;
	double VelocitySetpoint; 				/* Rad/s  Input (Command Velocity For Feed Forward) */
	double SumVelocityFeedForward;           /* Rad/s  Input (sum of output position control and velocity feedforward) */
	double VelocityFeedback;					/* Velocity Output form State Estimator(Kaman Filter)*/
	double VelocityPIDOutput;
	double Vel_Gfeed;						/* Velocity Feedforward Gain */
	double Output;
} ControlParameter;

typedef struct {
	double TrajCoef[6]; 			// Adjustable
	double T; 					// Adjustable
}TrajParameter;


void Traj_Coeff_Cal(TrajParameter *Traj, double T, double Pos_Final, double Pos_Now, double Vel_Final, double Vel_Now);
void TrajFollow(TrajParameter *Traj, double traj_t[5], double *Position, double *Velocity);

void Step_Driver_init(SteperParameter *step, TIM_HandleTypeDef *htim, uint32_t Channel, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t f_timer,uint8_t DIR_init);
void Step_Driver(SteperParameter *step, double f_driver);
void Servo_init(ServoParameter *Servo,TIM_HandleTypeDef *htim, uint32_t Channel);
void Servo_Drive(ServoParameter *Servo,int8_t Deg);
void Kalman_init(KalmanParameter *kalman, double Q, double R);
void KalmanFilter(KalmanParameter *kalman ,double theta_k);
double BaseENCRead();
void PID_init(PIDParameter *PID, double Kp, double Ki, double Kd);
double PID_Control(PIDParameter *PID,double Setpoint,double Feedback);
void CascadeControl_init(ControlParameter *Control,double PosP,double PosI,double PosD,double VelP,double VelI,double VelD, double GFeed);
void CascadeControl(ControlParameter *Control, KalmanParameter *kalman,	double Pos_Feed, double pos_set, double vel_set);

#endif /* INC_CONTROL_H_ */
