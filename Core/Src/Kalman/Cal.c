#include "cal.h"

six_point_cal_ErrorCode six_point_cal(arm_matrix_instance_f32 *w, arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *DCM_bIMU, arm_matrix_instance_f32 *M) {

    if (DCM_bIMU->numCols != 3 || DCM_bIMU->numRows != 3) return DCM_bIMU_INVALID_SIZE_SIX_POINT_CAL;
    if (M->numCols != 3 || M->numRows != 3) return M_INVALID_SIZE_SIX_POINT_CAL;

    arm_status arm_error;

    arm_matrix_instance_f32 Y;
    static float32_t Y_data[6*N * 3];
    memset(Y_data, 0, sizeof(Y_data));
    arm_mat_init_f32(&Y, 6*N, 3, Y_data);

    arm_matrix_instance_f32 w_trans;
    static float32_t w_trans_data[4 * 6*N];
    arm_mat_init_f32(&w_trans, 4, 6*N, w_trans_data);

    arm_matrix_instance_f32 w_trans_w;
    float32_t w_trans_w_data[4 * 4];
    arm_mat_init_f32(&w_trans_w, 4, 4, w_trans_w_data);

    arm_matrix_instance_f32 w_trans_w_inv;
    float32_t w_trans_w_inv_data[4 * 4];
    arm_mat_init_f32(&w_trans_w_inv, 4, 4, w_trans_w_inv_data);

    arm_matrix_instance_f32 w_trans_Y;
    float32_t w_trans_Y_data[4 * 3];
    arm_mat_init_f32(&w_trans_Y, 4, 3, w_trans_Y_data);

    int cont = 0;

    for (cont = 0; cont < N*3; cont = cont+3) {

        Y_data[cont] = 9.8f;

    }

    for (cont = N*3; cont < 2*N*3; cont = cont+3) {

        Y_data[cont] = -9.8f;

    }

    for (cont = 2*N*3; cont < 3*N*3; cont = cont+3) {

        Y_data[cont+1] = 9.8f;

    }

    for (cont = 3*N*3; cont < 4*N*3; cont = cont+3) {

        Y_data[cont+1] = -9.8f;

    }

    for (cont = 4*N*3; cont < 5*N*3; cont = cont+3) {

        Y_data[cont+2] = 9.8f;

    }

    for (cont = 5*N*3; cont < 6*N*3; cont = cont+3) {

        Y_data[cont+2] = -9.8f;

    }


    arm_error = arm_mat_trans_f32(w, &w_trans);
    if (arm_error == ARM_MATH_SIZE_MISMATCH) return W_INVALID_SIZE_SIX_POINT_CAL;
    arm_mat_mult_f32(&w_trans, w, &w_trans_w);
    arm_error = arm_mat_inverse_f32(&w_trans_w, &w_trans_w_inv);
    if (arm_error == ARM_MATH_SINGULAR) return SINGULAR_MATRIX_SIX_POINT_CAL;
    arm_mat_mult_f32(&w_trans, &Y, &w_trans_Y);
    arm_error = arm_mat_mult_f32(&w_trans_w_inv, &w_trans_Y, X);
    if (arm_error == ARM_MATH_SIZE_MISMATCH) return X_INVALID_SIZE_SIX_POINT_CAL;

    arm_matrix_instance_f32 M_trans;
    float32_t M_trans_data[3 * 3];
    arm_mat_init_f32(&M_trans, 3, 3, M_trans_data);

    memcpy(M_trans.pData, X->pData, 3*3*sizeof(float32_t));

    arm_mat_trans_f32(&M_trans, M);

    arm_matrix_instance_f32 R;
    float32_t R_data[3 * 3];
    arm_mat_init_f32(&R, 3, 3, R_data);

    float32_t tau[3];
    float32_t tmpA[3];
    float32_t tmpB[3];

    arm_matrix_instance_f32 M_copy;
    float32_t M_copy_data[3 * 3];
    memcpy(M_copy_data, M->pData, sizeof(M_copy_data));
    arm_mat_init_f32(&M_copy, 3, 3, M_copy_data);

    arm_mat_qr_f32(&M_copy, DEFAULT_HOUSEHOLDER_THRESHOLD_F32, &R, DCM_bIMU, tau, tmpA, tmpB);

    // Imponer signos positivos en la diagonal de R y ajustar DCM_bIMU en consecuencia

    float32_t r00 = R_data[0]; 
    float32_t r11 = R_data[4]; 
    float32_t r22 = R_data[8];

    if (r00 < 0.0f) {
        DCM_bIMU->pData[0] = -DCM_bIMU->pData[0]; // Fila 0, Col 0
        DCM_bIMU->pData[3] = -DCM_bIMU->pData[3]; // Fila 1, Col 0
        DCM_bIMU->pData[6] = -DCM_bIMU->pData[6]; // Fila 2, Col 0
    }

    if (r11 < 0.0f) {
        DCM_bIMU->pData[1] = -DCM_bIMU->pData[1]; // Fila 0, Col 1
        DCM_bIMU->pData[4] = -DCM_bIMU->pData[4]; // Fila 1, Col 1
        DCM_bIMU->pData[7] = -DCM_bIMU->pData[7]; // Fila 2, Col 1
    }

    if (r22 < 0.0f) {
        DCM_bIMU->pData[2] = -DCM_bIMU->pData[2]; // Fila 0, Col 2
        DCM_bIMU->pData[5] = -DCM_bIMU->pData[5]; // Fila 1, Col 2
        DCM_bIMU->pData[8] = -DCM_bIMU->pData[8]; // Fila 2, Col 2
    }

    return SUCCESS_SIX_POINT_CAL;
}
