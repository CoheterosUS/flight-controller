#include "KalmanLib.h"

static float32_t eye_data[H_P_A_Q_COLS * H_P_A_Q_COLS] = {0};
static arm_matrix_instance_f32 eye = {H_P_A_Q_COLS, H_P_A_Q_COLS, eye_data};


static const float32_t g[3] = {0.0f, 0.0f, 9.81f};

static float32_t DCM_bIMU_data[3 * 3] = {0};
static float32_t DCM_MAGb_data[3 * 3] = {0};
static arm_matrix_instance_f32 DCM_bIMU = {3, 3, DCM_bIMU_data};
static arm_matrix_instance_f32 DCM_MAGb = {3, 3, DCM_MAGb_data};

static float32_t H_GPS_data[H_GPS_ROWS * H_P_A_Q_COLS] = {0};
static arm_matrix_instance_f32 H_GPS = {H_GPS_ROWS, H_P_A_Q_COLS, H_GPS_data};


kalman_ErrorCode kalman_skew(float32_t (*v)[3], arm_matrix_instance_f32 *M) {

    if (M->numCols != 3 || M->numRows != 3) return INVALID_MATRIX_SIZE_SKEW;

    M->pData[0] = 0.0f;         M->pData[1] = -(*v)[2];     M->pData[2] = (*v)[1];
    M->pData[3] = (*v)[2];      M->pData[4] = 0.0f;         M->pData[5] = -(*v)[0];
    M->pData[6] = -(*v)[1];     M->pData[7] = (*v)[0];      M->pData[8] = 0.0f;

    return SUCCESS;

}

void kalman_eul2quat(float32_t (*eul)[3], float32_t (*quat)[4]) {

    float32_t cr = cos((*eul)[0] * 0.5);
    float32_t sr = sin((*eul)[0] * 0.5);
    float32_t cp = cos((*eul)[1] * 0.5);
    float32_t sp = sin((*eul)[1] * 0.5);
    float32_t cy = cos((*eul)[2] * 0.5);
    float32_t sy = sin((*eul)[2] * 0.5);

    (*quat)[0] = cr * cp * cy + sr * sp * sy;
    (*quat)[1] = sr * cp * cy - cr * sp * sy;
    (*quat)[2] = cr * sp * cy + sr * cp * sy;
    (*quat)[3] = cr * cp * sy - sr * sp * cy;

}

kalman_ErrorCode kalman_propagate(arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, float32_t h) {

    if(P->numCols != H_P_A_Q_COLS || P->numRows != H_P_A_Q_COLS) return P_INVALID_SIZE_PROPAGATE;
    if(A->numCols != H_P_A_Q_COLS || A->numRows != H_P_A_Q_COLS) return A_INVALID_SIZE_PROPAGATE;
    if(Q->numCols != H_P_A_Q_COLS || Q->numRows != H_P_A_Q_COLS) return Q_INVALID_SIZE_PROPAGATE;

    
    arm_matrix_instance_f32 Mtemp1;
    arm_matrix_instance_f32 Mtemp2;
    arm_matrix_instance_f32 Mtemp3;

    float32_t Mtemp1_data[A->numRows * A->numCols];
    float32_t Mtemp2_data[A->numRows * A->numCols];
    float32_t Mtemp3_data[A->numRows * A->numCols];

    arm_mat_init_f32(&Mtemp1, A->numRows, A->numCols, Mtemp1_data);
    arm_mat_init_f32(&Mtemp2, A->numRows, A->numCols, Mtemp2_data);
    arm_mat_init_f32(&Mtemp3, A->numRows, A->numCols, Mtemp3_data);

    /* STM = eye(size(A,1)) + A*h + A^2*(h)^2/2 */
    arm_mat_scale_f32(A, h, &Mtemp1);               /* Mtemp1 = A*h */
    arm_mat_mult_f32(A, A, &Mtemp2);                /* Mtemp2 = A^2 */
    arm_mat_scale_f32(&Mtemp2, 0.5f*h*h, &Mtemp3);  /* Mtemp3 = A^2*h^2/2 */
    arm_mat_add_f32(&eye, &Mtemp1, &Mtemp2);        /* Mtemp2 = eye + A*h */
    arm_mat_add_f32(&Mtemp2, &Mtemp3, &Mtemp1);     /* Mtemp1 = STM */

    /* P = STM*P*STM' + Q */
    arm_mat_mult_f32(&Mtemp1, P, &Mtemp2);          /* Mtemp2 = STM*P */
    arm_mat_trans_f32(&Mtemp1, &Mtemp3);            /* Mtemp3 = STM' */
    arm_mat_mult_f32(&Mtemp2, &Mtemp3, &Mtemp1);    /* Mtemp1 = STM*P*STM' */   /* esta operación fuerza a usar 3 matrices de buffer */
    arm_mat_add_f32(&Mtemp1, Q, P);

    return SUCCESS;
    
}

kalman_ErrorCode kalman_update(float32_t *z, arm_matrix_instance_f32 *H,
    arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *R, float32_t *h_x, float32_t *delta_x) {

    if(P->numCols != H_P_A_Q_COLS || P->numRows != H_P_A_Q_COLS) return P_INVALID_SIZE_UPDATE;
    if((R->numCols != H_GPS_ROWS || R->numRows != H_GPS_ROWS) && (R->numCols != H_MAG_ROWS || R->numRows != H_MAG_ROWS) && (R->numCols != H_BAR_ROWS || R->numRows != H_BAR_ROWS)) return R_INVALID_SIZE_UPDATE;
    if((H->numCols != H_P_A_Q_COLS) || (H->numRows != H_GPS_ROWS && H->numRows != H_MAG_ROWS && H->numRows != H_BAR_ROWS)) return H_INVALID_SIZE_UPDATE;


    arm_status arm_error;

    arm_matrix_instance_f32 Mtemp1;
    arm_matrix_instance_f32 PHt;
    arm_matrix_instance_f32 Mtemp2;
    arm_matrix_instance_f32 HPHtPlusR;
    arm_matrix_instance_f32 Mtemp3;
    arm_matrix_instance_f32 P_temp;

    float32_t Mtemp1_data[H->numCols * H->numRows];
    float32_t PHt_data[H->numCols * H->numRows];
    float32_t Mtemp2_data[H->numRows * H->numRows];
    float32_t HPHtPlusR_data[H->numRows * H->numRows];
    float32_t Mtemp3_data[H->numCols * H->numCols];
    float32_t P_temp_data[H->numCols * H->numCols];

    arm_mat_init_f32(&Mtemp1, H->numCols, H->numRows, Mtemp1_data);
    arm_mat_init_f32(&PHt, H->numCols, H->numRows, PHt_data);
    arm_mat_init_f32(&Mtemp2, H->numRows, H->numRows, Mtemp2_data);
    arm_mat_init_f32(&HPHtPlusR, H->numRows, H->numRows, HPHtPlusR_data);
    arm_mat_init_f32(&Mtemp3, H->numCols, H->numCols, Mtemp3_data);
    arm_mat_init_f32(&P_temp, H->numCols, H->numCols, P_temp_data);

    
    float32_t y[H->numRows];

    arm_sub_f32(z, h_x, y, H->numRows);

    arm_mat_trans_f32(H, &Mtemp1);                          /* Mtemp1 = H' */
    arm_mat_mult_f32(P, &Mtemp1, &PHt);                                         /* esta operación fuerza a usar dos matrices H2xH1 */
    arm_mat_mult_f32(H, &PHt, &Mtemp2);                     /* Mtemp2 = H*P*H' */
    arm_mat_add_f32(&Mtemp2, R, &HPHtPlusR);
    arm_error = arm_mat_inverse_f32(&HPHtPlusR, &Mtemp2);   /* Mtemp2 = (H*P*H'+R)^-1 */    /* esta operación fuerza a usar dos matrices H1xH1 */
    if (arm_error == ARM_MATH_SINGULAR) return SINGULAR_MATRIX_UPDATE;
    arm_mat_mult_f32(&PHt, &Mtemp2, &Mtemp1);               /* Mtemp1 = K */
    arm_mat_vec_mult_f32(&Mtemp1, y, delta_x);
    arm_mat_mult_f32(&Mtemp1, H, &Mtemp3);                  /* Mtemp3 = KH */
    arm_mat_sub_f32(&eye, &Mtemp3, &Mtemp3);                /* Mtemp3 = eye - KH */         /* add, sub, scale soportan in-place */
    arm_mat_mult_f32(&Mtemp3, P, &P_temp);                                                  /* esta operación fuerza a usar dos matrices H2xH2 */

    memcpy(P->pData, P_temp_data, P->numCols * P->numRows * sizeof(P->pData[0]));

    return SUCCESS;

}

void kalman_delta_sum(float32_t (*pos)[3], float32_t(*vel)[3], float32_t(*quat)[4], float32_t (*delta_x)[9]) {

    arm_add_f32((float32_t *)pos, (float32_t *)delta_x, (float32_t *)pos, 3);
    arm_add_f32((float32_t *)vel, (float32_t *)delta_x+3, (float32_t *)vel, 3);

    float32_t delta_eul[3];
    memcpy(delta_eul, (*delta_x)+6, 3*sizeof(delta_eul[0]));

    float32_t delta_quat[4];
    kalman_eul2quat(&delta_eul, &delta_quat);

    float32_t quat_post[4];
    arm_quaternion_product_single_f32((float32_t *)quat, delta_quat, quat_post);    /* esta operación obliga a usar tres cuaterniones */
    arm_quaternion_normalize_f32(quat_post, (float32_t *)quat, 1);
    
}

kalman_ErrorCode kalman_filter(bool IMU_available, bool GPS_available, bool MAG_available, bool BAR_available, float32_t (*accel_IMU)[3],
    float32_t (*omega_IMU)[3], float32_t (*z_GPS)[6], float32_t (*z_MAG)[3], float32_t (*z_BAR)[1], float32_t (*pos)[3], float32_t(*vel)[3],
    float32_t(*quat)[4], float32_t (*B_e)[3], arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R_GPS,
    arm_matrix_instance_f32 *R_MAG, arm_matrix_instance_f32 *R_BAR, float32_t p_ref_BAR,float32_t h) {

    if (P->numCols != H_P_A_Q_COLS || P->numRows != H_P_A_Q_COLS) return P_INVALID_SIZE_FILTER;
    if (Q->numCols != H_P_A_Q_COLS || Q->numRows != H_P_A_Q_COLS) return Q_INVALID_SIZE_FILTER;
    if (R_GPS->numCols != H_GPS_ROWS || R_GPS->numRows != H_GPS_ROWS) return R_GPS_INVALID_SIZE_FILTER;
    if (R_MAG->numCols != H_MAG_ROWS || R_MAG->numRows != H_MAG_ROWS) return R_MAG_INVALID_SIZE_FILTER;
    if (R_BAR->numCols != H_BAR_ROWS || R_BAR->numRows != H_BAR_ROWS) return R_BAR_INVALID_SIZE_FILTER;

    kalman_ErrorCode error;

    if (GPS_available) {

        float32_t h_x[H_GPS_ROWS];
        memcpy(h_x, (*pos), 3*sizeof(h_x[0]));
        memcpy(h_x+3, (*vel), 3*sizeof(h_x[0]));

        float32_t delta_x[9];
        error = kalman_update((float32_t *)z_GPS, &H_GPS, P, R_GPS, h_x, delta_x);
        if (error != SUCCESS) return error;
        kalman_delta_sum(pos, vel, quat, &delta_x);

    }
    
    if (MAG_available) {

        arm_matrix_instance_f32 H;
        float32_t H_data[H_MAG_ROWS * H_P_A_Q_COLS] = {0};
        arm_mat_init_f32(&H, H_MAG_ROWS, H_P_A_Q_COLS, H_data);
        
        arm_matrix_instance_f32 Mtemp1, Mtemp2;
        float32_t Mtemp1_data[3 * 3];
        float32_t Mtemp2_data[3 * 3];
        arm_mat_init_f32(&Mtemp1, 3, 3, Mtemp1_data);
        arm_mat_init_f32(&Mtemp2, 3, 3, Mtemp2_data);

        float32_t B_b[3];
        arm_quaternion2rotation_f32((float32_t *)quat, Mtemp1_data, 1);     /* Mtemp1 = DCM_eb */
        arm_mat_trans_f32(&Mtemp1, &Mtemp2);                                /* Mtemp2 = DCM_be */   /* esta operación obliga a usar dos Mtemp */
        arm_mat_vec_mult_f32(&Mtemp2, (float32_t *)B_e, B_b);
        error = kalman_skew(&B_b, &Mtemp1);                                 /* Mtemp1 = skew(B_b) */
        if (error != SUCCESS) return error;
        arm_mat_mult_f32(&DCM_MAGb, &Mtemp1, &Mtemp2);                      /* Mtemp2 = DCM_MAGb * skew(B_b) */

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {

                H_data[j + H_P_A_Q_COLS - 3 + i*H_P_A_Q_COLS] = Mtemp2_data[j + i*3];

            }
        }

        float32_t B_MAG[3];
        arm_mat_vec_mult_f32(&DCM_MAGb, B_b, B_MAG);

        float32_t delta_x[9];
        error = kalman_update((float32_t *)z_MAG, &H, P, R_MAG, B_MAG, delta_x);
        if (error != SUCCESS) return error;
        kalman_delta_sum(pos, vel, quat, &delta_x);

    }

    if (BAR_available) {

        arm_matrix_instance_f32 H;
        float32_t H_data[H_BAR_ROWS * H_P_A_Q_COLS] = {0};
        arm_mat_init_f32(&H, H_BAR_ROWS, H_P_A_Q_COLS, H_data);

        float32_t h_x[H_BAR_ROWS];
        h_x[0] = p_ref_BAR * powf(1.0f + ALPHA_AIR * (*pos)[2] / T_REF_AIR, (g[2] / (R_AIR * ALPHA_AIR)));

        H_data[2] = p_ref_BAR * (g[2] / (R_AIR * T_REF_AIR)) * powf(1.0f + ALPHA_AIR * (*pos)[2] / T_REF_AIR, (g[2] / (R_AIR * ALPHA_AIR) - 1.0f));

        float32_t delta_x[9];
        error = kalman_update((float32_t *)z_BAR, &H, P, R_BAR, h_x, delta_x);
        if (error != SUCCESS) return error;
        kalman_delta_sum(pos, vel, quat, &delta_x);

    }
    
    if (IMU_available) {

        arm_scale_f32((float32_t *)omega_IMU, (float32_t)M_PI/180.0f, (float32_t *)omega_IMU, 3);

        float32_t temp[3];
        arm_mat_vec_mult_f32(&DCM_bIMU, (float32_t *)accel_IMU, temp);          /* temp = accel_b */
        
 
        arm_matrix_instance_f32 A;
        float32_t A_data[9 * 9] = {0};
        arm_mat_init_f32(&A, 9, 9, A_data);

        arm_matrix_instance_f32 Mtemp1, skewAccel_b, minusDCM_ebSkewAccel_b;
        float32_t Mtemp1_data[3 * 3];
        float32_t skewAccel_b_data[3 * 3];
        float32_t minusDCM_ebSkewAccel_b_data[3 * 3];
        arm_mat_init_f32(&Mtemp1, 3, 3, Mtemp1_data);
        arm_mat_init_f32(&skewAccel_b, 3, 3, skewAccel_b_data);
        arm_mat_init_f32(&minusDCM_ebSkewAccel_b, 3, 3, minusDCM_ebSkewAccel_b_data);

        error = kalman_skew(&temp, &skewAccel_b);                               /* temp = accel_b */
        if (error != SUCCESS) return error;
        

        float32_t accel_e[3];

        arm_quaternion2rotation_f32((float32_t *)quat, Mtemp1_data, 1);         /* Mtemp1 = DCM_eb */

        arm_mat_vec_mult_f32(&Mtemp1, temp, accel_e);                           /* temp = accel_b */

        arm_mat_scale_f32(&Mtemp1, -1.0f, &Mtemp1);                             /* Mtemp1 = -DCM_eb */

        arm_mat_mult_f32(&Mtemp1, &skewAccel_b, &minusDCM_ebSkewAccel_b);

        arm_mat_vec_mult_f32(&DCM_bIMU, (float32_t *)omega_IMU, temp);          /* temp = omega_b */
        error = kalman_skew(&temp, &Mtemp1);                                    /* temp = omega_b */ /* Mtemp1 = skew(omega_b) */
        if (error != SUCCESS) return error;

        arm_mat_scale_f32(&Mtemp1, -1.0f, &Mtemp1);                             /* Mtemp1 = -skew(omega_b) */
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {

                if (i == j) {
                    A_data[j+3 + i*H_P_A_Q_COLS] = 1.0f;                                                /* A(1:3,4:6) = eye(3) */
                }

                A_data[j+6 + 3*H_P_A_Q_COLS+(i*H_P_A_Q_COLS)] = minusDCM_ebSkewAccel_b_data[j + i*3];   /* A(4:6,7:9) = -DCM_eb(quat) * skew(accel_b) */
                A_data[j+6 + 6*H_P_A_Q_COLS+(i*H_P_A_Q_COLS)] = Mtemp1_data[j + i*3];                   /* A(7:9,7:9) = -skew(omega_b) */

            }
        }

        error = kalman_propagate(P, &A, Q, h);
        if (error != SUCCESS) return error;

        

        float32_t omega_bSquared, normOmega_b, delta_quat[4];

        arm_dot_prod_f32(temp, temp, 3, &omega_bSquared);
        normOmega_b = sqrt(omega_bSquared);

        if (normOmega_b > EPSILON) {

            delta_quat[0] = cos(normOmega_b * 0.5f*h);
            for (int i = 0; i < 3; i++) {
                delta_quat[i+1] = sin(normOmega_b * 0.5f*h) * temp[i]/normOmega_b;
            }

        } else {

            delta_quat[0] = 1.0f;
            arm_scale_f32(temp, 0.5f*h, delta_quat+1, 3);

        }

        float32_t quat_prod[4];
        arm_quaternion_product_single_f32((float32_t *)quat, delta_quat, quat_prod);
        arm_quaternion_normalize_f32(quat_prod, (float32_t *)quat, 1);



        arm_add_f32(accel_e, g, temp, 3);       /* temp = vel_dot */
        
        for (int i = 0; i < 3; i++) {
            (*vel)[i] += h*temp[i];
            (*pos)[i] += h*(*vel)[i];
        }

    }

    arm_matrix_instance_f32 P_trans;
    float32_t P_trans_data[H_P_A_Q_COLS * H_P_A_Q_COLS];
    arm_mat_init_f32(&P_trans, H_P_A_Q_COLS, H_P_A_Q_COLS, P_trans_data);
    arm_mat_trans_f32(P, &P_trans);     /* esta operación fuerza a usar otra matriz */
    arm_mat_add_f32(P, &P_trans, P);    /* P = P + P' */ /* add, sub, scale soportan in-place */
    arm_mat_scale_f32(P, 0.5f, P);      /* P = 0.5*P */

    return SUCCESS;
    
}

void kalman_init(arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R_GPS,
    arm_matrix_instance_f32 *R_MAG, arm_matrix_instance_f32 *R_BAR, float32_t (*P_data)[H_P_A_Q_COLS * H_P_A_Q_COLS],
    float32_t (*Q_data)[H_P_A_Q_COLS * H_P_A_Q_COLS], float32_t (*R_MAG_data)[3 * 3], float32_t (*R_BAR_data)[1 * 1],
    float32_t (*eul_deg_0)[3], float32_t (*quat)[4], float32_t h) {


    for(int i = 0; i < H_P_A_Q_COLS; i++) {
        for(int j = 0; j < H_P_A_Q_COLS; j++) {
    
            if(i == j) {
                eye_data[j + i*H_P_A_Q_COLS] = 1.0f;
            }
    
        }
    }

    float32_t var_pos_0 = powf(SIGMA_POS_0, 2);
    float32_t var_vel_0 = powf(SIGMA_VEL_0, 2);
    float32_t var_accel = powf(SIGMA_ACCEL, 2);
    float32_t var_MAG = powf(SIGMA_MAG, 2);
    float32_t var_BAR = powf(SIGMA_BAR, 2);

    arm_mat_init_f32(P, H_P_A_Q_COLS, H_P_A_Q_COLS, (float32_t *)P_data);
    arm_mat_init_f32(Q, H_P_A_Q_COLS, H_P_A_Q_COLS, (float32_t *)Q_data);
    arm_mat_init_f32(R_MAG, 3, 3, (float32_t *)R_MAG_data);
    arm_mat_init_f32(R_BAR, 1, 1, (float32_t *)R_BAR_data);

    memset(P_data, 0, H_P_A_Q_COLS*H_P_A_Q_COLS*sizeof(float32_t));
    for (int i = 0; i < 3; i++) {
        (*P_data)[i + i*H_P_A_Q_COLS] = var_pos_0;
        (*P_data)[i+3 + (i+3)*H_P_A_Q_COLS] = var_vel_0;
        (*P_data)[i+6 + (i+6)*H_P_A_Q_COLS] = (float32_t)powf(SIGMA_ACT_DEG_0 * (float32_t)M_PI/180.f, 2);
    }


    memset((*Q_data), 0, H_P_A_Q_COLS*H_P_A_Q_COLS*sizeof((*Q_data)[0]));
    memset((*R_MAG_data), 0, 3*3*sizeof((*R_MAG_data)[0]));
    for (int i = 0; i < 3; i++) {
        (*Q_data)[i + i*H_P_A_Q_COLS] = (float32_t)powf(h, 3) * var_accel / 3.0f;
        (*Q_data)[i+3 + i*H_P_A_Q_COLS] = 0.5f * (float32_t)powf(h, 2) * var_accel;
        (*Q_data)[i + (i+3)*H_P_A_Q_COLS] = 0.5f * (float32_t)powf(h, 2) * var_accel;
        (*Q_data)[i+3 + (i+3)*H_P_A_Q_COLS] = h * var_accel;
        (*Q_data)[i+6 + (i+6)*H_P_A_Q_COLS] = h * (float32_t)powf(SIGMA_GYRO * M_PI/180.0f, 2);

        (*R_MAG_data)[i + i*3] = var_MAG;
    }

    (*R_BAR_data)[0] = var_BAR;


    arm_matrix_instance_f32 DCM_IMUb;
    float32_t DCM_IMUb_data[3 * 3];
    arm_mat_init_f32(&DCM_IMUb, 3, 3, DCM_IMUb_data);

    float32_t eul_bIMU[3] = EUL_bIMU_VALS_DEG;
    float32_t eul_bMAG[3] = EUL_bMAG_VALS_DEG;
    arm_scale_f32(eul_bIMU, (float32_t)M_PI/180.0f, eul_bIMU, 3);
    arm_scale_f32(eul_bMAG, (float32_t)M_PI/180.0f, eul_bMAG, 3);

    float32_t quat_temp[4];
    kalman_eul2quat(&eul_bIMU, &quat_temp);                     /* quat_temp = quat_bIMU */
    arm_quaternion2rotation_f32(quat_temp, DCM_IMUb_data, 1);   /* CMSIS sigue el convenio para cuaterniones v'=qvq*, nosotros v'=q*vq, lo que da lugar a rotaciones inversas */
    arm_mat_trans_f32(&DCM_IMUb, &DCM_bIMU);

    kalman_eul2quat(&eul_bMAG, &quat_temp);                     /* quat_temp = quat_bMAG */
    arm_quaternion2rotation_f32(quat_temp, DCM_MAGb_data, 1);


    float32_t eul_0[3];

    for (int i = 0; i < 3; i++) {
        
        eul_0[i] = (*eul_deg_0)[i] * (float32_t)M_PI/180.0f;

    }

    //float32_t eul_0[3] = {0, 0.5f*(float32_t)M_PI, true_heading_0 * (float32_t)M_PI/180.0f};
    //memset((*vel), 0, 3*sizeof((*vel)[0]));
    kalman_eul2quat(&eul_0, quat);

    
    for (int i = 0; i < H_GPS_ROWS; i++) {
        for (int j = 0; j < H_GPS_ROWS; j++) {

            if (i == j) {
                H_GPS_data[j + i*H_P_A_Q_COLS] = 1.0f;
            }

        }
    }

}