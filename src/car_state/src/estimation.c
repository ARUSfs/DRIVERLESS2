#include "car_state/estimation.h"

void Estimation_Init(EstimationData *est, SensorData *sensors) {
    // Initialize matrix structures
    arm_mat_init_f32(&est->A, 3, 3, est->A_data);
    arm_mat_init_f32(&est->B, 3, 2, est->B_data);
    arm_mat_init_f32(&est->H_TME, 9, 3, est->H_TME_data);
    arm_mat_init_f32(&est->Q, 3, 3, est->Q_data);
    arm_mat_init_f32(&est->R_GSS, 3, 3, est->R_GSS_data);
    arm_mat_init_f32(&est->R_TME, 9, 9, est->R_TME_data);
    arm_mat_init_f32(&est->x, 3, 1, est->x_data);
    arm_mat_init_f32(&est->P, 3, 3, est->P_data);
    arm_mat_init_f32(&est->u, 2, 1, est->u_data);
    arm_mat_init_f32(&est->x_pred, 3, 1, est->x_pred_data);
    arm_mat_init_f32(&est->P_pred, 3, 3, est->P_pred_data);
    arm_mat_init_f32(&est->z_GSS, 3, 1, est->z_GSS_data);
    arm_mat_init_f32(&est->z_TME, 9, 1, est->z_TME_data);
    arm_mat_init_f32(&est->z_TME_pred, 9, 1, est->z_TME_pred_data);
    arm_mat_init_f32(&est->K_GSS, 3, 3, est->K_GSS_data);
    arm_mat_init_f32(&est->K_TME, 3, 9, est->K_TME_data);
    arm_mat_init_f32(&est->I3, 3, 3, est->I3_data);
    arm_mat_init_f32(&est->z_TME_prev, 9, 1, est->z_TME_prev_data);

    // Initialize matrix data
    for (int i = 0; i < 9; i++) {
        est->A_data[i] = 0.0f;
        est->I3_data[i] = 0.0f;
    }
    for (int i = 0; i < 3; i++) {
        est->A_data[i*3 + i] = 1.0f;
        est->I3_data[i*3 + i] = 1.0f;
    }

    for (int i = 0; i < 6; i++) {
        est->B_data[i] = 0.0f;
    }
    est->B_data[0] = 1.0f;
    est->B_data[3] = CF_ / M_;
    est->B_data[5] = CF_ * LF_ / IZ_;

    for (int i = 0; i < 9; i++) {
        est->Q_data[i] = 0.0f;
    }
    est->Q_data[0] = 0.125f;
    est->Q_data[4] = 0.15f;
    est->Q_data[8] = 0.2f;

    for (int i = 0; i < 9; i++) {
		est->R_GSS_data[i] = 0.0f;
	}
	est->R_GSS_data[0] = 0.075f;
	est->R_GSS_data[4] = 0.025f;
	est->R_GSS_data[8] = 0.015f;

    for (int i = 0; i < 81; i++) {
        est->R_TME_data[i] = 0.0f;
    }
    est->R_TME_data[0]   = 0.1f;
    est->R_TME_data[10]  = 0.1f;
    est->R_TME_data[20]  = 0.1f;
    est->R_TME_data[30]  = 0.1f;
    est->R_TME_data[40]  = 0.1f;
    est->R_TME_data[50]  = 0.1f;
    est->R_TME_data[60]  = 0.125f;
    est->R_TME_data[70]  = 0.125f;
    est->R_TME_data[80]  = 0.002f;

    for (int i = 0; i < 27; i++) {
        est->H_TME_data[i] = 0.0f;
    }
    est->H_TME_data[6] = 1.0f;
    est->H_TME_data[8] = -0.5*TR_;
    est->H_TME_data[9] = 1.0f;
    est->H_TME_data[11] = 0.5*TR_;
    est->H_TME_data[19] = 1.0f;
    est->H_TME_data[20] = -LR_;
    est->H_TME_data[22] = 1.0f;
    est->H_TME_data[23] = -LR_;
    est->H_TME_data[26] = 1.0f;

    est->x_data[0] = 0.0f;
	est->x_data[1] = 0.0f;
	est->x_data[2] = 0.0f;

    for (int i = 0; i < 9; i++) {
        est->P_data[i] = 0.0f;
    }
    est->P_data[0] = 0.1f;
    est->P_data[4] = 0.1f;
    est->P_data[8] = 0.1f;

    for (int i = 0; i < 2; i++) {
        est->u_data[i] = 0.0f;
    }

    for (int i = 0; i < 9; i++) {
        est->valid_idx[i] = 1;
    }

    sensors->acceleration_x = 0.0f;
    sensors->acceleration_y = 0.0f;
    sensors->angular_z      = 0.01f;
    sensors->wheelspeed_fl  = 0.15f;
    sensors->wheelspeed_fr  = 0.15f;
    sensors->wheelspeed_rl  = 0.1f;
    sensors->wheelspeed_rr  = 0.1f;
	sensors->steering_angle = 0.0f;

    est->cont = 0;

	for(int i=0; i<4; i++){
		est->sr[i] = 0.0f;
		est->sa[i] = 0.0f;
	}

	// FZ CALCULATION
	est->fz_params[0] = NSM_F_ * H_CDG_NSM_F_ / TF_;
	est->fz_params[1] = NSM_R_ * H_CDG_NSM_R_ / TR_;
	est->fz_params[2] = SM_F_ * H_RC_F_ / TF_;
	est->fz_params[3] = SM_R_ * H_RC_R_ / TR_;
	est->fz_params[4] = (SM_F_ + SM_R_) * (H_CDG_SM_ - H_RA_) * RS_F_ / TF_;
	est->fz_params[5] = (SM_F_ + SM_R_) * (H_CDG_SM_ - H_RA_) * RS_R_ / TR_;
	est->fz_params[6] = (NSM_F_*H_CDG_NSM_F_ + NSM_R_*H_CDG_NSM_R_)/(LF_+LR_);
	est->fz_params[7] = (SM_F_ + SM_R_) * H_CDG_SM_ / (LF_+LR_);
	est->fz_params[8] = 0.5*M_*G_*LF_/(LF_+LR_);
	est->fz_params[9] = 0.5*R_CDP_;
	est->fz_params[10] = 0.5*H_CDP_/(LF_+LR_);
	est->fz_params[11] = 0.5*M_*G_*LR_/(LF_+LR_);

}


void Estimation_Update(EstimationData *est, SensorData *sensors, float x_out[3], int GSS_OK) {
    // ----------- LINEAR BICYCLE FILTER -----------
    // Update transition matrix
    float vx_prev = est->x_data[0];

    est->A_data[4] = 1 + DT_ * (-(CF_ + CR_) / (M_ * vx_prev + EPS_));
    est->A_data[5] = DT_ * ((LR_ * CR_ - LF_ * CF_) / (M_ * vx_prev + EPS_) - vx_prev);
    est->A_data[7] = DT_ * ((LR_ * CR_ - LF_ * CF_) / (IZ_ * vx_prev + EPS_));
    est->A_data[8] = 1 + DT_ * (-(LF_*LF_*CF_ + LR_*LR_*CR_) / (IZ_ * vx_prev + EPS_));

    // ----- State Prediction: x_pred = A*x + DT * B*u -----
    float Ax_data[3];
    arm_matrix_instance_f32 Ax;
    arm_mat_init_f32(&Ax, 3, 1, Ax_data);
    arm_mat_mult_f32(&est->A, &est->x, &Ax);

    float Bu_data[3];
    arm_matrix_instance_f32 Bu;
    arm_mat_init_f32(&Bu, 3, 1, Bu_data);
    arm_mat_mult_f32(&est->B, &est->u, &Bu);

    for (int i = 0; i < 3; i++) {
    	est->x_pred_data[i] = Ax_data[i] + DT_ * Bu_data[i];
    }

    // ----- Covariance Prediction: P_pred = A*P*A^T + Q -----
    float AP_data[9];
    arm_matrix_instance_f32 AP;
    arm_mat_init_f32(&AP, 3, 3, AP_data);
    arm_mat_mult_f32(&est->A, &est->P, &AP);

    float A_T_data[9];
    arm_matrix_instance_f32 A_T;
    arm_mat_init_f32(&A_T, 3, 3, A_T_data);
    arm_mat_trans_f32(&est->A, &A_T);

    float APA_T_data[9];
    arm_matrix_instance_f32 APA_T;
    arm_mat_init_f32(&APA_T, 3, 3, APA_T_data);
    arm_mat_mult_f32(&AP, &A_T, &APA_T);

    for (int i = 0; i < 9; i++) {
    	est->P_pred_data[i] = APA_T_data[i] + est->Q_data[i];
    }


    // Update sensor data and control vector
    est->ax = sensors->acceleration_x;
    est->ay = sensors->acceleration_y;
    est->delta = sensors->steering_angle;
    est->u_data[0] = est->ax;
    est->u_data[1] = est->delta;

    if(GSS_OK == 1){
    	// ----- Measurement vector -----
		est->z_GSS_data[0] = sensors->speed_x;
		est->z_GSS_data[1] = sensors->speed_y;
		est->z_GSS_data[2] = sensors->angular_z;

		// ----- Measurement Prediction: z_pred = I3 * x_pred = x_pred -----

		// ----- Kalman Gain: K = P_pred * I3^T * (I3 * P_pred * I3^T + R)^(-1) = P_pred * (P_pred + R)^(-1) -----
		// Compute S = P_pred + R
		float S_data[9];
		arm_matrix_instance_f32 S;
		arm_mat_init_f32(&S, 3, 3, S_data);
		for (int i = 0; i < 9; i++) {
			S_data[i] = est->P_pred_data[i] + est->R_GSS_data[i];
		}
		
		// Invert S
		float S_inv_data[9];
		arm_matrix_instance_f32 S_inv;
		arm_mat_init_f32(&S_inv, 3, 3, S_inv_data);
		arm_mat_inverse_f32(&S, &S_inv);

		// Compute K = (P_pred * I3^T) * S_inv = P_pred * S_inv
		arm_mat_mult_f32(&est->P_pred, &S_inv, &est->K_GSS);

		// ----- State Update: x = x_pred + K*(z - z_pred) -----
		float innovation[3];
		for (int i = 0; i < 3; i++) {
			innovation[i] = est->z_GSS_data[i] - est->x_pred_data[i];
		}
		float K_innov[3] = {0.0f, 0.0f, 0.0f};
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				K_innov[i] += est->K_GSS_data[i*3+j] * innovation[j];
			}
			est->x_data[i] = est->x_pred_data[i] + K_innov[i];
		}

		// ----- Covariance Update: P = (I3 - K*I3)*P_pred = (I3 - K)*P_pred -----
		float I_minus_KH_data[9];
		arm_matrix_instance_f32 I_minus_KH;
		arm_mat_init_f32(&I_minus_KH, 3, 3, I_minus_KH_data);
		for (int i = 0; i < 9; i++) {
			I_minus_KH_data[i] = est->I3_data[i] - est->K_GSS_data[i];
		}

		arm_mat_mult_f32(&I_minus_KH, &est->P_pred, &est->P);

    } else {
    	// ----------- TIRE FORCE ESTIMATION -----------
		float F_wx[4], F_wy[4], F_wz[4];
		// float aux_long[4] = { -1.0f, -1.0f, 1.0f, 1.0f };
		// float aux_lat[4]  = { -RS_F_/TF_, RS_F_/TF_, -RS_R_/TR_, RS_R_/TR_ };
		float v = sqrtf(est->x_data[0]*est->x_data[0] + est->x_data[1]*est->x_data[1]);
		
		// for (int i = 0; i < 4; i++) {
		// 	F_wz[i] = 0.25f * (M_*G_ + 0.5f*RHO_*CLA_*v*v + M_*H_CG_*((est->ax * aux_long[i])/(2*(LF_+LR_)) + est->ay * aux_lat[i]));
		// 	if(F_wz[i] < 0)
		// 		F_wz[i] = 0.0001;
		// }

		// Tire load calculation
		float F_L = 0.5*RHO_*CLA_*v*v;
		float F_D = 0.5*RHO_*CDA_*v*v;

		float y_WT_ns_f = est->fz_params[0] * est->ay;
		float y_WT_ns_r = est->fz_params[1] * est->ay;

		float y_WT_s_g_f = est->fz_params[2] * est->ay;
		float y_WT_s_g_r = est->fz_params[3] * est->ay;

		float y_WT_s_e_f = est->fz_params[4] * est->ay;
		float y_WT_s_e_r = est->fz_params[5] * est->ay;

		float x_WT_ns = est->fz_params[6] * est->ax;

		float x_WT_s = est->fz_params[7] * est->ax;

		F_wz[0] = est->fz_params[8] + est->fz_params[9] * F_L - est->fz_params[10] * F_D 
			- y_WT_ns_f - y_WT_s_g_f - y_WT_s_e_f - 0.5*(x_WT_ns+x_WT_s);
		F_wz[1] = est->fz_params[8] + est->fz_params[9] * F_L - est->fz_params[10] * F_D
			+ y_WT_ns_f + y_WT_s_g_f + y_WT_s_e_f - 0.5*(x_WT_ns+x_WT_s);
		F_wz[2] = est->fz_params[11] + (0.5 - est->fz_params[9]) * F_L + est->fz_params[10] * F_D
			- y_WT_ns_r - y_WT_s_g_r - y_WT_s_e_r + 0.5*(x_WT_ns+x_WT_s);
		F_wz[3] = est->fz_params[11] + (0.5 - est->fz_params[9]) * F_L + est->fz_params[10] * F_D
			+ y_WT_ns_r + y_WT_s_g_r + y_WT_s_e_r + 0.5*(x_WT_ns+x_WT_s);	
		
		// TO DO: NEGATIVE LOADS RECALCULATION
		for(int i=0; i<4; i++){
			if(F_wz[i] < 0){
				F_wz[i] = 0.0001;
			}
		}


		float Fz = F_wz[0] + F_wz[1] + F_wz[2] + F_wz[3];
		float k_fz = 1/(Fz + EPS_);
		for (int i = 0; i < 4; i++) {
			F_wx[i] = F_wz[i] * k_fz * (M_*est->ax + 0.5f*RHO_*CDA_*v*v);
			F_wy[i] = F_wz[i] * k_fz * (M_*est->ay);
		}

		// ----------- TIRE INVERSE MODEL -----------
		// TMEasy force parameters
		float dF_x0[4], F_x_M[4], F_x_S[4];
		float dF_y0[4], F_y_M[4], F_y_S[4];

		for (int i = 0; i < 4; i++) {
			dF_x0[i] = DMU_X0_ * F_wz[i];
			F_x_M[i]  = MU_X_M_ * F_wz[i];
			F_x_S[i]  = MU_X_S_ * F_wz[i];

			dF_y0[i] = DMU_Y0_ * F_wz[i];
			F_y_M[i]  = MU_Y_M_ * F_wz[i];
			F_y_S[i]  = MU_Y_S_ * F_wz[i];
		}

		// Slip ratio
		for (int i = 0; i < 4; i++) {
			float solutions[5] = {1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f};

			// First piece
			float a = 1;
			float b = dF_x0[i]*S_X_M_*(1/(F_x_M[i] + EPS_) - 1/(F_wx[i] + EPS_)) - 2;
			float c = 1;

			float sol;
			if(b*b -4 < 0){	// LO DEL MÁXIMO ESTÁ MAL, PUEDE DARSE F_wx<0
				solutions[0] = S_X_M_;
			} else {
				sol = S_X_M_ *  0.5*(-b + sqrtf(b*b - 4));
				if(fabsf(sol) <= S_X_M_){
					solutions[0] = sol;
				}

				sol = S_X_M_ *  0.5*(-b - sqrtf(b*b - 4));
				if(fabsf(sol) <= S_X_M_){
					solutions[1] = sol;
				}
			}

			// Second piece
			a = 2;
			b = -3;
			float d = (F_wx[i] - F_x_M[i]) / (F_x_S[i] - F_x_M[i] + EPS_);

			float p = - b*b / (3*a*a);
			float q = (2*b*b*b + 27*a*a*d) / (27*a*a*a);
			float D = q*q/4 + p*p*p/27;

			if(D < 0){
				float r = sqrtf(-p*p*p/27);
				float theta = acosf(-q/(2*r));

				sol = (S_X_S_ - S_X_M_) * (-b/(3*a) + 2*sqrtf(-p/3)*cosf(theta/3)) + S_X_M_;
				if(S_X_M_ <= fabsf(sol) && fabsf(sol) <= S_X_S_){
					solutions[2] = sol;
				}

				sol = (S_X_S_ - S_X_M_) * (-b/(3*a) + 2*sqrtf(-p/3)*cosf((theta+2*M_PI)/3)) + S_X_M_;
				if(S_X_M_ <= fabsf(sol) && fabsf(sol) <= S_X_S_){
					solutions[3] = sol;
				}

				sol = (S_X_S_ - S_X_M_) * (-b/(3*a) + 2*sqrtf(-p/3)*cosf((theta+4*M_PI)/3)) + S_X_M_;
				if(S_X_M_ <= fabsf(sol) && fabsf(sol) <= S_X_S_){
					solutions[4] = sol;
				}
			} else {
				float alpha = cbrtf(-q/2 + sqrt(D));
				float beta = cbrtf(-q/2 - sqrt(D));

				sol = (S_X_S_ - S_X_M_) * (alpha + beta) + S_X_M_;
				if(S_X_M_ <= fabsf(sol) && fabsf(sol) <= S_X_S_){
					solutions[2] = sol;
				}
			}

			// Choose the solution
			if(fabsf(F_wx[i]) < F_x_S[i]){
				if(fabsf(est->sr[i] - solutions[0]) < fabsf(est->sr[i] - solutions[1])){
					est->sr[i] = solutions[0];
				} else {
					est->sr[i] = solutions[1];
				}
			} else {
				float sr_min_diff = solutions[0];
				float min_diff = fabsf(est->sr[i] - solutions[0]);
				for(int i=1; i<5; i++){
					if(fabsf(solutions[i]) > EPS_ && fabsf(est->sr[i] - solutions[i]) < min_diff){
						sr_min_diff = solutions[i];
						min_diff = fabsf(est->sr[i] - solutions[i]);
					}
				}
				est->sr[i] = sr_min_diff;
			}
		}


		// Slip angle
		for (int i = 0; i < 4; i++) {
			float solutions[5] = {1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f};

			// Fisrt piece
			float a = 1;
			float b = dF_y0[i]*S_Y_M_*(1/(F_y_M[i] + EPS_) - 1/(F_wy[i] + EPS_)) - 2;
			float c = 1;

			float sol;
			if(b*b -4 < 0){ // LO DEL MÁXIMO ESTÁ MAL, PUEDE DARSE F_wy<0
				solutions[0] = F_wy[i] < 0 ? -S_Y_M_ : S_Y_M_;
			} else {
				sol = S_Y_M_ *  0.5*(-b + sqrtf(b*b - 4));
				if(fabsf(sol) <= S_Y_M_){
					solutions[0] = sol;
				}

				sol = S_Y_M_ *  0.5*(-b - sqrtf(b*b - 4));
				if(fabsf(sol) <= S_Y_M_){
					solutions[1] = sol;
				}
			}

			// Second piece
			a = 2;
			b = -3;
			float d = (F_wy[i] - F_y_M[i]) / (F_y_S[i] - F_y_M[i] + EPS_);

			float p = - b*b / (3*a*a);
			float q = (2*b*b*b + 27*a*a*d) / (27*a*a*a);
			float D = q*q/4 + p*p*p/27;

			if(D < 0){
				float r = sqrtf(-p*p*p/27);
				float theta = acosf(-q/(2*r));

				sol = (S_Y_S_ - S_Y_M_) * (-b/(3*a) + 2*sqrtf(-p/3)*cosf(theta/3)) + S_Y_M_;
				if(S_Y_M_ <= fabsf(sol) && fabsf(sol) <= S_Y_S_){
					solutions[2] = sol;
				}

				sol = (S_Y_S_ - S_Y_M_) * (-b/(3*a) + 2*sqrtf(-p/3)*cosf((theta+2*M_PI)/3)) + S_Y_M_;
				if(S_Y_M_ <= fabsf(sol) && fabsf(sol) <= S_Y_S_){
					solutions[3] = sol;
				}

				sol = (S_Y_S_ - S_Y_M_) * (-b/(3*a) + 2*sqrtf(-p/3)*cosf((theta+4*M_PI)/3)) + S_Y_M_;
				if(S_Y_M_ <= fabsf(sol) && fabsf(sol) <= S_Y_S_){
					solutions[4] = sol;
				}
			} else {
				float alpha = cbrtf(-q/2 + sqrt(D));
				float beta = cbrtf(-q/2 - sqrt(D));

				sol = (S_Y_S_ - S_Y_M_) * (alpha + beta) + S_Y_M_;
				if(S_Y_M_ <= fabsf(sol) && fabsf(sol) <= S_Y_S_){
					solutions[2] = sol;
				}
			}

			// Choose the solution
			if(fabsf(F_wy[i]) < F_y_S[i]){
				if(fabsf(est->sa[i] + solutions[0]) < fabsf(est->sa[i] + solutions[1])){
					est->sa[i] = -solutions[0];
				} else {
					est->sa[i] = -solutions[1];
				}
			} else {
				float sa_min_diff = -solutions[0];
				float min_diff = fabsf(est->sa[i] + solutions[0]);
				for(int i=1; i<5; i++){
					if(fabsf(solutions[i]) > EPS_ && fabsf(est->sa[i] + solutions[i]) < min_diff){
						sa_min_diff = -solutions[i];
						min_diff = fabsf(est->sa[i] + solutions[i]);
					}
				}
				est->sa[i] = sa_min_diff;
			}
		}

		// ----------- WHEEL SPEED ESTIMATION -----------
		float wheelspeed[4] = { sensors->wheelspeed_fl, sensors->wheelspeed_fr,
								 sensors->wheelspeed_rl, sensors->wheelspeed_rr };
		float v_wx[4], v_wy[4];
		for (int i = 0; i < 4; i++) {
			v_wx[i] = RDYN_ * wheelspeed[i] / (est->sr[i] + 1.0f);
			v_wy[i] = tanf(est->sa[i]) * v_wx[i];
		}

		// ----- Update observation matrix -----
		float cos_delta = cosf(est->delta);
		float sin_delta = sinf(est->delta);

		est->H_TME_data[0] = cos_delta;
		est->H_TME_data[1] = sin_delta;
		est->H_TME_data[2] = -0.5*TF_*cos_delta + LF_*sin_delta;
		est->H_TME_data[3] = cos_delta;
		est->H_TME_data[4] = sin_delta;
		est->H_TME_data[5] = 0.5*TF_*cos_delta + LF_*sin_delta;
		est->H_TME_data[12] = - sin_delta;
		est->H_TME_data[13] = cos_delta;
		est->H_TME_data[14] = LF_*cos_delta + 0.5*TF_*sin_delta;
		est->H_TME_data[15] = - sin_delta;
		est->H_TME_data[16] = cos_delta;
		est->H_TME_data[17] = LF_*cos_delta - 0.5*TF_*sin_delta;

		// // ----- Sensor Supervisor -----
		// // ESTO LO CAMBIARÍA ELIMINANDO LA VARIABLE z_TME_prev Y SIMPLEMENTE COGIENDO z_TME_data, QUE TODAVÍA NO SE HA MODIFICADO
		// arm_mat_mult_f32(&est->H_TME, &est->x, &est->z_TME_prev);

		// int valid_cont = 1;
		// est->valid_idx[8] = 1;

		// for (int j = 0; j < 4; j++) {
		// 	if(fabs(est->z_TME_prev_data[j] - v_wx[j]) < TOL_VX_) {
		// 		est->valid_idx[j] = 1;
		// 		valid_cont++;
		// 	}
		// }
		// for (int j = 0; j < 4; j++) {
		// 	if(fabs(est->z_TME_prev_data[j+4] - v_wy[j]) < TOL_VY_) {
		// 		est->valid_idx[j+4] = 1;
		// 		valid_cont++;
		// 	}
		// }
		// while(valid_cont < 5) {
		//    for (int j = 0; j < 8; j++) {
		// 		if(est->valid_idx[j] < 1) {
		// 			est->valid_idx[j] = 1;
		// 			valid_cont++;
		// 			break;
		// 		}
		//    }
		// }

		// ----- Measurement vector -----
		est->z_TME_data[0] = v_wx[0];
		est->z_TME_data[1] = v_wx[1];
		est->z_TME_data[2] = v_wx[2];
		est->z_TME_data[3] = v_wx[3];
		est->z_TME_data[4] = v_wy[0];
		est->z_TME_data[5] = v_wy[1];
		est->z_TME_data[6] = v_wy[2];
		est->z_TME_data[7] = v_wy[3];
		est->z_TME_data[8] = sensors->angular_z;

		// ----- Measurement Prediction: z_pred = H * x_pred -----
		arm_mat_mult_f32(&est->H_TME, &est->x_pred, &est->z_TME_pred);

		// ----- Kalman Gain: K = P_pred * H^T * (H * P_pred * H^T + R)^(-1) -----
		// Compute H^T (transpose of H)
		float H_T_data[27];
		arm_matrix_instance_f32 H_T;
		arm_mat_init_f32(&H_T, 3, 9, H_T_data);
		arm_mat_trans_f32(&est->H_TME, &H_T);
		// Compute P_pred * H^T
		float PH_T_data[27];
		arm_matrix_instance_f32 PH_T;
		arm_mat_init_f32(&PH_T, 3, 9, PH_T_data);
		arm_mat_mult_f32(&est->P_pred, &H_T, &PH_T);
		// Compute S_temp = H * P_pred * H^T
		float S_data[81];
		arm_matrix_instance_f32 S;
		arm_mat_init_f32(&S, 9, 9, S_data);
		arm_mat_mult_f32(&est->H_TME, &PH_T, &S);
		// Add R: S = S_temp + R
		for (int i = 0; i < 81; i++) {
			S_data[i] = S_data[i] + est->R_TME_data[i];
		}
		
		// Invert S
		float S_inv_data[81];
		arm_matrix_instance_f32 S_inv;
		arm_mat_init_f32(&S_inv, 9, 9, S_inv_data);
		arm_mat_inverse_f32(&S, &S_inv);

		// Compute K = (P_pred * H^T) * S_inv
		arm_mat_mult_f32(&PH_T, &S_inv, &est->K_TME);

		// ----- State Update: x = x_pred + K*(z - z_pred) -----
		float innovation[9];
		for (int i = 0; i < 9; i++) {
			innovation[i] = est->z_TME_data[i] - est->z_TME_pred_data[i];
		}

		float K_innov[3] = {0.0f, 0.0f, 0.0f};
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 9; j++) {
				// K_innov[i] = K_innov[i] + est->K_TME_data[i*9+j] * innovation[j] * est->valid_idx[j];
				K_innov[i] = K_innov[i] + est->K_TME_data[i*9+j] * innovation[j];
			}
			est->x_data[i] = est->x_pred_data[i] + K_innov[i];
		}

		// ----- Covariance Update: P = (I - K*H)*P_pred -----
		float KH_data[9];
		arm_matrix_instance_f32 KH;
		arm_mat_init_f32(&KH, 3, 3, KH_data);
		arm_mat_mult_f32(&est->K_TME, &est->H_TME, &KH);

		float I_minus_KH_data[9];
		arm_matrix_instance_f32 I_minus_KH;
		arm_mat_init_f32(&I_minus_KH, 3, 3, I_minus_KH_data);
		for (int i = 0; i < 9; i++) {
			I_minus_KH_data[i] = est->I3_data[i] - KH_data[i];
		}

		arm_mat_mult_f32(&I_minus_KH, &est->P_pred, &est->P);
    }

	for(int i=0; i<3; i++){
		x_out[i] = est->x_data[i];
	}

//
//    est->cont = est->cont + 1;
//    if(est->cont >= 100){
//    	est->cont = 0;
//    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
//    }
}
