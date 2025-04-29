#ifndef ESTIMATION_H
#define ESTIMATION_H


#ifdef __cplusplus
extern "C" {
#endif


#include "arm_math.h"
#include "car_state/SensorData.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// #include "stm32f4xx_hal.h"

// Estimation frequency
#define DT_ 0.01f

// Car parameters (ART25) ------- DIFERENCIAS ENTRE DV2 Y VDC-ART25
#define TF_ 1.24f
#define TR_ 1.18f
#define LF_ 0.84315f
#define LR_ 0.68985f
#define M_ 270.0f
#define IZ_ 180.0f
#define G_ 9.81f
#define CF_ 26000.0f
#define CR_ 22000.0f
#define H_CG_ 0.28f
#define RHO_ 1.1f
#define CDA_ 1.f
#define CLA_ 3.5f
#define RDYN_ 0.202f
#define RS_F_ 0.5032f
#define RS_R_ 0.4968f

#define NSM_F_ 25.0f
#define NSM_R_ 25.0f
#define H_CDG_NSM_F_ 0.225f
#define H_CDG_NSM_R_ 0.225f
#define SM_F_ 114.4f
#define SM_R_ 105.6f
#define H_CDG_SM_ 0.3f
#define H_RC_F_ 0.2f
#define H_RC_R_ 0.3f
#define H_RA_ 0.255001076f
#define R_CDP_ 0.5f
#define H_CDP_ 0.2f

// TMEasy parameters
#define DMU_X0_ 45.2889911982648f
#define MU_X_M_ 1.27500000000002f
#define MU_X_S_ 1.18051035833282f
#define S_X_M_ 0.0800000000000222f
#define S_X_S_ 0.948162803350065f

#define DMU_Y0_ 31.7095470405367f
#define MU_Y_M_ 1.50383931756024f
#define MU_Y_S_ 1.49999960896867f
#define S_Y_M_ 0.0622188440683049f
#define S_Y_S_ 0.49999791668207f

// Sensor supervisor parameters
#define TOL_VX_ 2.0833333333333f
#define TOL_VY_ 0.6944444444444f

// Epsilon
#define EPS_ 0.000001f

// Struct to hold estimation data
typedef struct {
    float ax, ay, delta;

    float A_data[9], B_data[6], H_TME_data[27], Q_data[9], R_GSS_data[9], R_TME_data[81];
    float x_data[3], P_data[9], u_data[2], x_pred_data[3], P_pred_data[9];
    float z_GSS_data[3], z_TME_data[9], z_TME_pred_data[9], K_GSS_data[9], K_TME_data[27], I3_data[9], z_TME_prev_data[9];

    arm_matrix_instance_f32 A, B, H_TME, Q, R_GSS, R_TME, x, P, u;
    arm_matrix_instance_f32 x_pred, P_pred, z_GSS, z_TME, z_TME_pred, K_GSS, K_TME, I3, z_TME_prev;

    float sr[4], sa[4];

    int valid_idx[9];

    int cont;

    float fz_params[12];
} EstimationData;



// Estimation functions
void Estimation_Init(EstimationData *est, SensorData *sensors);
void Estimation_Update(EstimationData *est, SensorData *sensors, float x_out[3], int GSS_OK);

#ifdef __cplusplus
}
#endif

#endif
