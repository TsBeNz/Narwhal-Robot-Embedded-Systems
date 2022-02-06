/*
 * control.c
 *
 *  Created on: Jan 26, 2022
 *      Author: thans
 */

#include "control.h"


void control_init(ControlParameter *Control,float PosP,float PosI,float PosD,float Vel_Gain_Feed,float VelP,float VelI,float VelD){
	Control->Pos_Kp = PosP;
	Control->Pos_Ki = PosI;
	Control->Pos_Kd = PosD;
	Control->Vel_Kp = VelP;
	Control->Vel_Ki = VelI;
	Control->Vel_Kd = VelD;
	Control->Vel_Gfeed = Vel_Gain_Feed;

	Control->PositionITerm = 0;
	Control->PositionPIDOutput = 0;
	Control->VelocityPIDOutput = 0;
	Control->FrequencyPIDOutput = 0;
}

void Kalman_init(KalmanParameter *kalman, float sigma_a, float sigma_w,
		float p11, float p12, float p21, float p22) {
	kalman->sigma_a = sigma_a; 			// Adjustable
	kalman->sigma_w = sigma_w; 			// Adjustable
	kalman->p11 = p11; 					// Adjustable
	kalman->p12 = p12;
	kalman->p21 = p21;
	kalman->p22 = p22;					// Adjustable
}

void CascadeControl(ControlParameter *Control,KalmanParameter *kalman,int16_t Encoder_Position_New){
	/* Input */
	int16_t Ds = Encoder_Position_New - Control->EncoderPosition;

	//Unwrapping position
	if (Ds >= 8192) {
		Ds -= 16383;
	} else if (Ds <= -8192) {
		Ds += 16383;
	}

	/*Start Change Unit*/
	Control->CurrentPosition = Encoder_Position_New;
	/*End Change Unit*/

	Control->VelocityFixWindow =  ((float)Ds)/delta_t;

	/*Kalman Filter*/
	KalmanFilter(Control, kalman);

	/* Position */
	Control->PositionError[0] = Control->PositionSetpoint - Control->CurrentPosition;
	Control->PositionITerm += Control->PositionError[0];
	Control->PositionPIDOutput = (Control->PositionError[0] * Control->Pos_Kp)						/* P */
			+ (Control->PositionITerm * Control->Pos_Ki)											/* I */
			+ ((Control->PositionError[0] - Control->PositionError[1])	* Control->Pos_Kd);			/* D */

	/*  Velocity  */
	Control->VelocityError[0] = Control->PositionPIDOutput+ Control->VelocitySetpoint - Control->VelocityFixWindow; /*input of velocity controld*/
	Control->VelocityPIDOutput = (Control->Vel_Kp * (Control->VelocityError[0] - Control->VelocityError[1]))
			+ (Control->Vel_Ki * Control->VelocityError[0])
			+ (Control->Vel_Kd	* (Control->VelocityError[0] - 2 * Control->VelocityError[1] + Control->VelocityError[2]));
	/* Frequency */
	Control->FrequencyPIDOutput = Control->Vel_Gfeed * (Control->PositionPIDOutput+ Control->VelocitySetpoint); //vel ของ feedforward เป็นที่ sum มาหมดหรือแค่ ออกจาก position
	/* Update */
	Control->EncoderPosition = Encoder_Position_New;
	Control->PositionError[1] = Control->PositionError[0];
	Control->VelocityError[2] = Control->VelocityError[1];
	Control->VelocityError[1] = Control->VelocityError[0];
}

void TrajectoryCof(ControlParameter *Control,float traj_t,float traj_tPow2,float traj_tPow3,float traj_tPow4,float traj_tPow5){
	Control->PositionSetpoint = Control->TrajectoryCoefficient[0] + (Control->TrajectoryCoefficient[1] * traj_t) + (Control->TrajectoryCoefficient[3] * traj_tPow3) + (Control->TrajectoryCoefficient[4] * traj_tPow4) + (Control->TrajectoryCoefficient[5] * traj_tPow5);
	Control->VelocitySetpoint = Control->TrajectoryCoefficient[1] + (3 * Control->TrajectoryCoefficient[3] * traj_tPow2) + (4 * Control->TrajectoryCoefficient[4] * traj_tPow3) + (5 * Control->TrajectoryCoefficient[5] * traj_tPow4);
}

void KalmanFilter(ControlParameter *Control,KalmanParameter *kalman){

	/*  Velocity Estimation (Input By Velocity)  */
	float Q = kalman->sigma_a * kalman->sigma_a;
	float R = kalman->sigma_w * kalman->sigma_w;
	Control->EstimatePosition += Control->EstimateVelocity[1] * delta_t;
	Control->EstimateVelocity[0] = 0 + Control->EstimateVelocity[1];
	float ye = Control->VelocityFixWindow - Control->EstimateVelocity[0];  // Input
	kalman->p11 = kalman->p11 + delta_t * kalman->p21 + (Q * delta_tPow4) / 4 + (delta_tPow2 * (kalman->p12 + delta_t * kalman->p22)) / delta_t;
	kalman->p12 = kalman->p12 + delta_t * kalman->p22 + (Q * delta_tPow3) / 2;
	kalman->p21 = (2 * delta_t * kalman->p21 + Q * delta_tPow2 + 2 * kalman->p22 * delta_tPow2)	/ (2 * delta_t);
	kalman->p22 = Q * delta_tPow2 + kalman->p22;
	Control->EstimatePosition += (kalman->p12 * ye) / (R + kalman->p22);
	Control->EstimateVelocity[0] += (kalman->p22 * ye) / (R + kalman->p22);
	kalman->p11 = kalman->p11 - (kalman->p12 * kalman->p21) / (R + kalman->p22);
	kalman->p12 = kalman->p12 - (kalman->p12 * kalman->p22) / (R + kalman->p22);
	kalman->p21 = -kalman->p21 * (kalman->p22 / (R + kalman->p22) - 1);
	kalman->p22 = -kalman->p22 * (kalman->p22 / (R + kalman->p22) - 1);

	/*	Update	*/
	Control->EstimateVelocity[1] = Control->EstimateVelocity[0];
}
