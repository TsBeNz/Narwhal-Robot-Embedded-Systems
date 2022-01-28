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

#define delta_t 0.001
#define delta_tPow2 (delta_t * delta_t)
#define delta_tPow3 (delta_tPow2 * delta_t)
#define delta_tPow4 (delta_tPow3 * delta_t)


typedef struct {
	uint8_t ControlEnable[2];				/* [0] = Position , [1] = Velocity */

	float TrajectoryCoefficient[6];			/* Trajectory Coefficient */

	int16_t EncoderPosition;				/* Encoder Position */
	float VelocityFixWindow;				/* Velocity (Find by Ds/Dt) */

	float PositionSetpoint; 				/* Rad	  Input */
	float Pos_Kp;							/* Position PID Constant Kp Gain*/
	float Pos_Ki;							/* Position PID Constant Ki Gain*/
	float Pos_Kd;							/* Position PID Constant Kd Gain*/
	float EstimatePosition;					/* Position Output form State Estimator(Kaman Filter) */
	float CurrentPosition;					/* Position */
	float PositionPIDOutput; 				/* PID Position Output */
	float PositionError[2];					/* Position Error [0] = Now Error , [1] = Before Error */
	float PositionITerm;					/* Position I Term Form Position Control*/

	float VelocitySetpoint; 				/* Rad/s  Input (Command Velocity For Feed Forward) */
	float Vel_Kp;							/* Velocity PID Constant Kp Gain*/
	float Vel_Ki;							/* Velocity PID Constant Ki Gain*/
	float Vel_Kd;							/* Velocity PID Constant Kd Gain*/
	float Vel_Gfeed;						/* Velocity Feedforward Gain */
	float EstimateVelocity[2];					/* Velocity Output form State Estimator(Kaman Filter)*/
	float VelocityPIDOutput; 				/* PID Velocity Output */
	float VelocityError[3];					/* Velocity Error [0] = Now Error , [1] = Before Error */
} ControlParameter;

typedef struct {
	float sigma_a; 			// Adjustable
	float sigma_w; 			// Adjustable
	float p11; 				// Adjustable
	float p12;
	float p21;
	float p22; 				// Adjustable
}KalmanParameter;

void control_init(ControlParameter *Control,float PosP,float PosI,float PosD,float Vel_Gain_Feed,float VelP,float VelI,float VelD);
void Kalman_init(KalmanParameter *kalman, float sigma_a, float sigma_w, float p11, float p12, float p21, float p22);
void CascadeControl(ControlParameter *Control,KalmanParameter *kalman,int16_t Encoder_Position_New);
void TrajectoryCof(ControlParameter *Control,float traj_t,float traj_tPow2,float traj_tPow3,float traj_tPow4,float traj_tPow5);
void KalmanFilter(ControlParameter *Control,KalmanParameter *kalman);

#endif /* INC_CONTROL_H_ */
