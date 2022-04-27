/*
 * control.c
 *
 *  Created on: Jan 26, 2022
 *      Author: thansak
 */

#include "control.h"



/*Stepper Driver Function*/
/*
 * GPIO for DIR Interface
 * Timer for Step Interface
 *
 *
 *
 * Note
 * __HAL_TIM_SET_COMPARE();		CCR
 * __HAL_TIM_SET_AUTORELOAD();	ARR
 */
void Step_Driver_init(SteperParameter *step, TIM_HandleTypeDef *htim,
		uint32_t Channel, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
		uint32_t f_timer,uint8_t DIR_init) {
	step->htim = htim;
	step->Channel = Channel;
	step->GPIOx = GPIOx;
	step->GPIO_Pin = GPIO_Pin;
	step->f_timer = f_timer;
	step->DIR_init = DIR_init;
	HAL_TIM_PWM_Start(step->htim, step->Channel);
	step->htim->Instance->ARR = 500;
	step->htim->Instance->CCR1 = 0;
}


void Step_Driver(SteperParameter *step, double f_driver) {
	double abs_f_driver = fabs(f_driver);
	uint16_t reg_out;
	if (abs_f_driver <= 7) {
		reg_out = 50000;
		step->htim->Instance->ARR = 50000;
		step->htim->Instance->CCR1 = 0;
	} else if (abs_f_driver < 10) {
		reg_out = 50000;
		step->htim->Instance->ARR = reg_out;
		step->htim->Instance->CCR1 = reg_out >> 1;
	} else {
		reg_out = (uint16_t) (step->f_timer / abs_f_driver);
		step->htim->Instance->ARR = reg_out;
		step->htim->Instance->CCR1 = reg_out >> 1;
	}
	if (f_driver >= 0) {
		HAL_GPIO_WritePin(step->GPIOx, step->GPIO_Pin, step->DIR_init);
	} else {
		HAL_GPIO_WritePin(step->GPIOx, step->GPIO_Pin, step->DIR_init ^ 0x01);
	}
}


/*
 * Servo Drive init
 */
void Servo_init(ServoParameter *Servo,TIM_HandleTypeDef *htim,
		uint32_t Channel){
	Servo->htim = htim;
	Servo->Channel = Channel;
	HAL_TIM_PWM_Start(Servo->htim, Servo->Channel);
	__HAL_TIM_SET_COMPARE(Servo->htim,Servo->Channel,1499);
}

/*
 * Servo Drive Function
 * Deg Range (0 -> 180)
 */
void Servo_Drive(ServoParameter *Servo,uint8_t Deg){
	uint16_t Pulse_in  =  (uint16_t)((Deg * 8.3333333f) +499);
	__HAL_TIM_SET_COMPARE(Servo->htim,Servo->Channel,Pulse_in);
}

void Traj_Coeff_Cal(TrajParameter *Traj, double T, double Pos_Final,
  double Pos_Now, double Vel_Final, double Vel_Now) {
 Traj->T = T;
 double T_P2 = T * T;
 double T_P3 = T_P2 * T;
 double T_P4 = T_P3 * T;
 double T_P5 = T_P4 * T;
 double ds = Pos_Now - Pos_Final;
 double tfv0 = T * Vel_Now;
 double tfv1 = T * Vel_Final;
 Traj->TrajCoef[0] = Pos_Now;
 Traj->TrajCoef[1] = Vel_Now;
 Traj->TrajCoef[3] = -(2 * (5 * ds + 3 * tfv0 + 2*tfv1)) / T_P3;
 Traj->TrajCoef[4] = (15 * ds + 8 * tfv0 + 7*tfv1) / T_P4;
 Traj->TrajCoef[5] = -(3 * (2 * ds + tfv0 + tfv1)) / T_P5;
}

void TrajFollow(TrajParameter *Traj, double traj_t[5], double *Position,
		double *Velocity) {
	*Position = Traj->TrajCoef[0] + (Traj->TrajCoef[1] * traj_t[0])
			+ (Traj->TrajCoef[3] * traj_t[2]) + (Traj->TrajCoef[4] * traj_t[3])
			+ (Traj->TrajCoef[5] * traj_t[4]);
	*Velocity = Traj->TrajCoef[1] + ((3.0 * Traj->TrajCoef[3]) * traj_t[1])
			+ ((4.0 * Traj->TrajCoef[4]) * traj_t[2])
			+ ((5.0 * Traj->TrajCoef[5]) * traj_t[3]);
}


/* KalmanFilter Function */
/*
 * KalmanFilter
 *
 * Q -> Process
 * R -> Sensor
 */
void Kalman_init(KalmanParameter *kalman, double Q, double R) {
	kalman->Q = Q; 			// Adjustable
	kalman->R = R; 			// Adjustable
	kalman->x1 = 0.0;
	kalman->x2 = 0.0;
	kalman->p11 = 0.05;
	kalman->p12 = 0.05;
	kalman->p21 = 0.05;
	kalman->p22 = 0.05;
}


/*
 *	theta_k is Position input
 */
void KalmanFilter(KalmanParameter *kalman ,double theta_k) {
	double b_xx1_tmp;
	double b_xx2_tmp;
	double c_xx1_tmp;
	double c_xx2_tmp;
	double d_xx1_tmp;
	double d_xx2_tmp;
	double e_xx1_tmp;
	double xx1_tmp;
	double xx1_tmp_tmp;
	double xx2_tmp;
	double xx1,xx2,pp11,pp12,pp21,pp22;
	xx1_tmp = 4.0 * delta_t * kalman->p12;
	b_xx1_tmp = 4.0 * delta_t * kalman->p21;
	c_xx1_tmp = kalman->Q * delta_tPow4;
	xx1_tmp_tmp = delta_tPow2;
	d_xx1_tmp = 4.0 * xx1_tmp_tmp * kalman->p22;
	e_xx1_tmp = ((((4.0 * kalman->R + 4.0 * kalman->p11) + xx1_tmp) + b_xx1_tmp) + c_xx1_tmp)
			+ d_xx1_tmp;
	xx1 = ((((((4.0 * kalman->R *kalman->x1+ 4.0 * kalman->p11 * theta_k) + d_xx1_tmp * theta_k)
			+ 4.0 * kalman->R * delta_t * kalman->x2) + xx1_tmp * theta_k) + b_xx1_tmp * theta_k)
			+ c_xx1_tmp * theta_k) / e_xx1_tmp;
	xx2_tmp = kalman->p22 * delta_t;
	b_xx2_tmp = kalman->Q * delta_tPow3;
	c_xx2_tmp = b_xx2_tmp / 2.0 + xx2_tmp;
	d_xx2_tmp = c_xx2_tmp + kalman->p21;
	xx2_tmp = (((kalman->R + kalman->p11) + delta_t * kalman->p21) + c_xx1_tmp / 4.0) + delta_t * (kalman->p12 + xx2_tmp);
	xx2 = kalman->x2 - d_xx2_tmp * ((kalman->x1 - theta_k) + delta_t * kalman->x2) / xx2_tmp;
	pp11 = kalman->R * ((((4.0 * kalman->p11 + xx1_tmp) + b_xx1_tmp) + c_xx1_tmp) + d_xx1_tmp)
			/ e_xx1_tmp;
	xx1_tmp = b_xx2_tmp + 2.0 * kalman->p22 * delta_t;
	pp12 = 2.0 * kalman->R * (xx1_tmp + 2.0 * kalman->p12) / e_xx1_tmp;
	pp21 = 2.0 * kalman->R * (xx1_tmp + 2.0 * kalman->p21) / e_xx1_tmp;
	pp22 = (kalman->p22 + kalman->Q * xx1_tmp_tmp) - (c_xx2_tmp + kalman->p12) * d_xx2_tmp / xx2_tmp;

	/*Update Variable*/
	kalman->x1 = xx1;
	kalman->x2 = xx2;
	kalman->p11 = pp11;
	kalman->p12 = pp12;
	kalman->p21 = pp21;
	kalman->p22 = pp22;
}

double BaseENCRead(){
	return 970.0f;  //fsaldfkjas;dflkjas;dflksjf;asdf
}

void PID_init(PIDParameter *PID, double Kp, double Ki, double Kd) {
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->ITerm = 0;
	PID->Setpoint = 0;
	PID->Feedback = 0;
	PID->Error[0] = 0;
	PID->Error[1] = 0;
	PID->Output = 0;
}

double PID_Control(PIDParameter *PID,double Setpoint,double Feedback){
	PID->Feedback = Feedback; 	// Feedback Input
	PID->Setpoint = Setpoint;	// Setpoint Input
	PID->Error[0] = PID->Setpoint - PID->Feedback;
	PID->ITerm += PID->Error[0];
	PID->Output = ((PID->Kp * PID->Error[0]) + (PID->Ki * PID->ITerm)
			+ (PID->Kd * (PID->Error[0] - PID->Error[1])));
	PID->Error[1] = PID->Error[0]; // Update Error
	return PID->Output;
}


void CascadeControl_init(ControlParameter *Control,double PosP,double PosI,double PosD,double VelP,double VelI,double VelD, double GFeed){
	PID_init(&Control->Pos,PosP,PosI,PosD);
	PID_init(&Control->Vel,VelP,VelI,VelD);
	Control->Vel_Gfeed = GFeed;
}


void CascadeControl(ControlParameter *Control, KalmanParameter *kalman,
		double Pos_Feed, double pos_set, double vel_set) {
	/*Set Setpoint*/
	Control->PositionSetpoint = pos_set;
	Control->VelocitySetpoint = vel_set;

	/*Kalman Filter*/
	KalmanFilter(kalman, Pos_Feed); /*Kalman filter */
	Control->VelocityFeedback = kalman->x2;
	Control->PositionFeedback = kalman->x1;

	/*Position PID Control*/
	Control->PositionPIDOutput = PID_Control(&Control->Pos,
			Control->PositionSetpoint, Control->PositionFeedback);
	/*Feedforward Velocity*/
	Control->SumVelocityFeedForward = Control->PositionPIDOutput + Control->VelocitySetpoint;
	/*Velocity PID Control*/
	Control->VelocityPIDOutput = PID_Control(&Control->Vel,
			Control->SumVelocityFeedForward, Control->VelocityFeedback);
	/*Feedforward Velocity Setpoint*/
	Control->Output = (Control->Vel_Gfeed * Control->SumVelocityFeedForward)
			+ Control->VelocityPIDOutput;
//	Control->Output = Control->VelocityPIDOutput;
}
