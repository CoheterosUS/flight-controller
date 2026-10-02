#include <stdio.h>
#include "KalmanLib.h"
#include <stdbool.h>

int main(){

    kalman_ErrorCode error;

    arm_matrix_instance_f32 P, Q;

    float32_t P_data[H_P_A_Q_COLS * H_P_A_Q_COLS];
    float32_t Q_data[H_P_A_Q_COLS * H_P_A_Q_COLS];

    float32_t pos[3] = {1.2f, 3.4f, 5.6f};
    float32_t vel[3] = {0.1f, 0.2f, -0.3f};
    float32_t eul_deg_0[3] = {0.0f, 85.0f, 45.0f};
    float32_t quat[4];

    float32_t p_ref_BAR = 102000;

    float32_t accel_IMU[3] = {30, 1, 2};
    float32_t omega_IMU[3] = {4, 10, 7};
    float32_t z_GPS[6] = {5, 4, 3, 2, 5, -6};
    float32_t z_MAG[3] = {2, 4, 5};
    float32_t z_BAR[1] = {101825};
    float32_t B_e[3] = {1, 2, 4};
    float32_t h = 0.1f;
    float32_t R_GPS_data[36] =  {4, 0, 0, 0, 0, 0,
                                0, 2, 0, 0, 0, 0,
                                0, 0, 3, 0, 0, 0,
                                0, 0, 0, 5, 0, 0,
                                0, 0, 0, 0, 7, 0,
                                0, 0, 0, 0, 0, 8};
    float32_t R_MAG_data[3 * 3];
    float32_t R_BAR_data[1];
    arm_matrix_instance_f32 R_GPS, R_MAG, R_BAR;

    arm_mat_init_f32(&R_GPS, 6, 6, R_GPS_data);

    kalman_init(&P, &Q, &R_GPS, &R_MAG, &R_BAR, &P_data, &Q_data, &R_MAG_data, &R_BAR_data, &eul_deg_0, &quat, h);

    error = kalman_filter(true, true, true, true, &accel_IMU, &omega_IMU, &z_GPS, &z_MAG, &z_BAR, &pos, &vel, &quat, &B_e, &P, &Q, &R_GPS, &R_MAG, &R_BAR, p_ref_BAR, h);
    if (error != SUCCESS) printf("Error %i\n",error);


    printf("Filter P matrix:\n");
    for (int i = 0; i < H_P_A_Q_COLS; i++) {
        for (int j = 0; j < H_P_A_Q_COLS; j++) {

            printf("%f  ",P.pData[j+i*H_P_A_Q_COLS]);

        }
        printf("\n");
    }

    printf("Filter x:\n");
    for (int i = 0; i < 3; i++) {
        printf("%f  ",pos[i]);
    }
    for (int i = 0; i < 3; i++) {
        printf("%f  ",vel[i]);
    }
    for (int i = 0; i < 4; i++) {
        printf("%f  ",quat[i]);
    }
    printf("\n");

}