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



//void Chess_Tracker(uint8_t Chess_Index, double Chess_Theta, double Chess_Omega){
//	/*
//	 * Offset2Center = robot to chess board center
//	 */
//	double Offset2Center[2] = {300,0};
//	/*
//	 * Output
//	 *
//	 */
//	double ChessPosition[2];
//	double ChessVelocity[2];
//
//	double ChessRadius;
//	double TzOnly = 3;
//    uint8_t X = Chess_Index%8;
//    uint8_t N = Chess_Index/8;
//
//    ChessPose(X, N, Chess_Theta, ChessPosition);
//    FindR(Offset2Center, ChessPosition, &ChessRadius);
//    FindXYSpeed(Chess_Theta, Chess_Omega, ChessRadius, ChessVelocity);
//
//    /*
//	 * Traj Z Axis
//	 */
//    for (int i = 0; i < 4; i++) {
//    		q_in[i] = Control[i].PositionFeedback;
//    	}
//    FPK(q_in, 269.0f, EndEffectorNow);
//	Traj_Coeff_Cal(&Traj[4], TzOnly, Z_Top_Offset,
//			      EndEffectorNow, 0, 0); /* Cal C for Trajz*/
//	Trajz_Flag = 1;
//	if (Chessmove_State == 1 && Trajz_Flag == 1) { /* Let do while in check position state and have Cal Coeff Z*/
//		double traj_t_set[5];
//		double TaskZ_Position = 0;
//		double TaskZ_Velocity = 0;
//		traj_t_set[0] = t;
//		traj_t_set[1] = t * t;
//		traj_t_set[2] = traj_t_set[1] * t;
//		traj_t_set[3] = traj_t_set[2] * t;
//		traj_t_set[4] = traj_t_set[3] * t;
//		TrajFollow(&Traj[4], traj_t_set, TaskZ_Position,
//				   TaskZ_Velocity);
//	}
//	if (t >= Traj[4].T) { /* Trajz finished */
//		Trajz_Flag = 0;
//	}
//	t += 0.005;
//
//    // Transfer to Joint
//	double EndEffectorgoal[3];
//	EndEffectorgoal[0] = ChessPosition[0];
//	EndEffectorgoal[1] = ChessPosition[1];
//	EndEffectorgoal[2] = TaskZ_Position;
//	double gamma[3] = { 1, 1, -1 };
//	IPK(gamma, EndEffectorgoal, SetPoint_Position);
//	d_Task[0] = ChessVelocity[0];
//	d_Task[1] = ChessVelocity[1];
//	d_Task[2] = TaskZ_Velocity;
//	IVK(SetPoint_Position, d_Task, SetPoint_Velocity);
//
//}

//void ChessMove1(uint8_t StartIndex, uint8_t EndIndex){
//	double Z_Top_Offset = 200;
//
//	double EndEffectorNow[3];
//	double EndEffectorAtChess[2];
//	double q_in[5];
//
//	for (int i = 0; i < 4; i++) {
//		q_in[i] = Control[i].PositionFeedback;
//	}
//	ChessPose(StartIndex, Kalman[4].x1, EndEffectorAtChess);
//	FPK(q_in, 269.0f, EndEffectorNow);
//	double DeltaX = EndEffectorNow[0] - EndEffectorAtChess[0];
//	double DeltaY = EndEffectorNow[1] - EndEffectorAtChess[1];
//	double DeltaZ = EndEffectorNow[2] - Z_Top_Offset;
//	double Distance = sqrt(
//			(DeltaX * DeltaX) + (DeltaY * DeltaY) + (DeltaZ * DeltaZ));
//	double T2Move = (Distance / 150.0f) + 1;
//
////	/*
////	 * Traj Z Axis
////	 */
////
////	Traj_Coeff_Cal(&Traj[4], T2Move, Z_Top_Offset,
////			EndEffectorNow, 0, 0); /* Cal C for Trajz*/
////	Trajz_Flag = 1;
////	if (Chessmove_State == 1 && Trajz_Flag == 1) { /* Let do while in check position state and have Cal Coeff Z*/
////		double traj_t_set[5];
////		traj_t_set[0] = t;
////		traj_t_set[1] = t * t;
////		traj_t_set[2] = traj_t_set[1] * t;
////		traj_t_set[3] = traj_t_set[2] * t;
////		traj_t_set[4] = traj_t_set[3] * t;
////		TrajFollow(&Traj[4], traj_t_set, &SetPoint_Position[4], /* SetPoint_Position[4] valueof z axis not related to control */
////								&SetPoint_Velocity[4]);
////	}
////	if (t >= Traj[4].T) { /* Trajz finished */
////		Trajz_Flag = 0;
////	}
////	t += 0.005;
//
//	/*
//	 * Traj 2 Move XY
//	 */
//
//	double EndEffectorTarget[3];
//	EndEffectorTarget[2] = Z_Top_Offset;
//	ChessPose(StartIndex, Kalman[4].x1 + (T2Move * (Kalman[4].x2)) , EndEffectorTarget);
//
//	double gamma[3] = { 1, 1, -1 };
//	double q_inv[4];
//	IPK(gamma, EndEffectorTarget, q_inv);
//	t = 0;
//	for (int i = 0; i < 4; i++) {
//		Traj_Coeff_Cal(&Traj[i], T2Move, q_inv[i],
//				Control[i].PositionFeedback, 0, Control[i].VelocityFeedback);
//	}
//	Traj_Flag = 0x0F;
//	Chessmove_State = 0;
//
//}
//
//uint8_t ChessMove2() {
//	double EndEffectorNow[3];
//	double q_now[5];
//	for (int i = 0; i < 4; i++) {
//		q_now[i] = Control[i].PositionFeedback;
//	}
//	FPK(q_now, 269.0f, EndEffectorNow);
//	uint8_t count_ready = 0;
//	for (int i = 0; i < 4; i++) {
//		if (fabs(EndEffectorNow[i] - EndEffectorTarget[i]) <= 0.01) {
//			count_ready += 1;
//		}
//		if (count_ready == 3) {  	// Finish Task
//			return 1;
//		} else {						// Move Not Finish
//			return 0;
//		}
//	}
//	return 0;
//}
