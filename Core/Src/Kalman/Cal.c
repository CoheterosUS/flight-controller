#include "cal.h"

#include "dsp/arm_math.h"

void six_point_cal(arm_matrix_instance_f32 *w, arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A_m, arm_matrix_instance_f32 *M) {

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


    arm_mat_trans_f32(w, &w_trans);
    arm_mat_mult_f32(&w_trans, w, &w_trans_w);
    arm_mat_inverse_f32(&w_trans_w, &w_trans_w_inv);
    arm_mat_mult_f32(&w_trans, &Y, &w_trans_Y);
    arm_mat_mult_f32(&w_trans_w_inv, &w_trans_Y, X);

    arm_matrix_instance_f32 M_trans;
    float32_t M_trans_data[3 * 3];
    arm_mat_init_f32(&M_trans, 3, 3, M_trans_data);

    memcpy(M_trans.pData, X->pData, 3*3*sizeof(float32_t));

    arm_mat_trans_f32(&M_trans, M);

    float32_t norm_M_col[3] = {0};

    for (int i = 0; i < 3; i++) {

        for (int j = 0; j < 3; j++) {

            norm_M_col[i] += M->pData[j*3 + i] * M->pData[j*3 + i];

        }

        norm_M_col[i] = sqrtf(norm_M_col[i]);

    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {

            A_m->pData[j*3 + i] = M->pData[j*3 + i] / norm_M_col[i];

        }
    }

}