/*  
    Librería de funciones para el filtro de Kalman ESKF de Coheteros, desarrollada por
    José Federico Tenor.
    
    Este filtro fusiona datos de IMU, GPS, magnetómetro y barómetro para determinar la
    posición, velocidad y actitud del vehículo.

    Esta librería utiliza la librería DSP de CMSIS, por lo que requiere un microcontrolador con
    soporte para FPU y la librería DSP de CMSIS. Se recomienda asignar 4 KB o más de memoria de stack para su ejecución.
*/

#ifndef KALMAN_LIB_H_
#define KALMAN_LIB_H_

#include "Libs/dsp/arm_math.h"
#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832
#endif

#define R_AIR 287.05
#define ALPHA_AIR 0.0065

#define H_GPS_ROWS 6        /* también orden de R_GPS */
#define H_MAG_ROWS 3        /* también orden de R_MAG */
#define H_BAR_ROWS 1        /* también orden de R_BAR */
#define H_P_A_Q_COLS 9      /* no. de columnas de H_GPS, H_MAG, P, A y Q (son el mismo número) */
#define EPSILON 1e-10f      /* valor por debajo del cual usamos la expresión lineal de la propagación de la actitud */

#define SIGMA_POS_0 1.0f          /* en m */
#define SIGMA_VEL_0 0.01f         /* en m/s */
#define SIGMA_ACT_DEG_0 3.0f      /* en º */
#define SIGMA_ACCEL 0.01f         /* en m/s2 */
#define SIGMA_GYRO 0.1f           /* en º/s */
#define SIGMA_MAG 28.0f           /* uds. de B_e y z_MAG */
#define SIGMA_BAR 200.0f          /* en Pa */

#define EUL_bIMU_VALS_DEG {0, 0, 0}     /* en º, ángulos de rotación de ejes IMU a body. El vector debe ser {X, Y, Z} con orden de rotación ZYX */
#define EUL_bMAG_VALS_DEG {0, 0, 0}     /* en º, lo mismo pero de ejes MAG a body */

typedef enum kalman_ErrorCode {

    KALMAN_SUCCESS = 0,
    INVALID_MATRIX_SIZE_SKEW = -1,
    P_INVALID_SIZE_PROPAGATE = -2,
    A_INVALID_SIZE_PROPAGATE = -3,
    Q_INVALID_SIZE_PROPAGATE = -4,
    P_INVALID_SIZE_UPDATE = -5,
    R_INVALID_SIZE_UPDATE = -6,
    H_INVALID_SIZE_UPDATE = -7,
    SINGULAR_MATRIX_UPDATE = -8,
    P_INVALID_SIZE_FILTER = -9,
    Q_INVALID_SIZE_FILTER = -10,
    R_GPS_INVALID_SIZE_FILTER = -11,
    R_MAG_INVALID_SIZE_FILTER = -12,
    R_BAR_INVALID_SIZE_FILTER = -13

} kalman_ErrorCode;

/*  Inputs: v (vector)
    Outputs: M (matriz)
    Crea una matriz skew-symmetric a partir de un vector tal que el producto vectorial de
    v y otro vector se convierte en el producto matricial de M por ese otro vector. */
kalman_ErrorCode kalman_skew(float32_t (*v)[3], arm_matrix_instance_f32 *M);

/*  Inputs: eul (vector de ángulos de Euler)
    Outputs: quat (vector coeficientes del cuaternión de actitud)
    El vector de ángulos eul debe estar en radianes y dispuesto así:
    {roll, pitch, yaw}. La conversión asume orden de rotación ZYX. */
void kalman_eul2quat(float32_t (*eul)[3], float32_t (*quat)[4]);

/*  Inputs: P (matriz de covarianza), A (jacobiana df/dx), Q (matriz de ruido de proceso)
    h (paso de integración)
    Outputs: P
    Todas las matrices de esta función deben ser cuadradas y del mismo tamaño. */
kalman_ErrorCode kalman_propagate(arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, float32_t h);

/*  Inputs: z (vector de medida real), H (matriz de sensibilidad), P (matriz de covarianza),
    R (matriz de ruido de medida), h_x (vector de medida esperada)
    Outputs: P, delta_x (vector de corrección)
    z y h_x deben tener el mismo tamaño. P y R deben ser cuadradas. H debe tener tantas
    columnas como P y tantas filas como R. */
kalman_ErrorCode kalman_update(float32_t *z, arm_matrix_instance_f32 *H,
    arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *R, float32_t *h_x, float32_t *delta_x);


/*  FUNCIONES ESPECÍFICAS A NUESTRA APLICACIÓN DEL FILTRO:  */

/*  Inputs: pos (posición), vel (velocidad), quat (cuaternión de actitud), delta_x (vector de corrección)
    Outputs: pos (posición), vel (velocidad), quat (cuaternión de actitud)
    Delta_x es un vector de corrección de 9 componentes en el que las últimas 3 son el error de
    actitud en Euler. Este error de actitud debe cumplir las características que se indican en eul2quat. */
void kalman_delta_sum(float32_t (*pos)[3], float32_t(*vel)[3], float32_t(*quat)[4], float32_t (*delta_x)[9]);

/*  Inputs: IMU_available, GPS_available, MAG_available (true cuando hay medidas nuevas de
    los respectivos sensores y false cuando no), accel_IMU (vector de medidas del acelerómetro),
    omega_IMU (vector de medidas del giroscopio), z_GPS (vector de medidas del GPS en NED, primero
    posición y luego velocidad), z_MAG (vector de medidas del magnetómetro), z_BAR (vector de medidas del barómetro),
    pos (posición), vel (velocidad), quat (cuaternión de actitud), B_e (campo magnético en ejes inerciales),
    P, Q, R_GPS, R_MAG, R_BAR, p_ref_BAR (presión de referencia del barómetro), h (paso de integración)
    Outputs: P, x
    Todas las medidas de los sensores en SI, salvo el giroscopio, que va en º/s. Las unidades de B_e
    y z_MAG son arbitrarias, pero deben ser las mismas para las dos variables. */
kalman_ErrorCode kalman_filter(bool IMU_available, bool GPS_available, bool MAG_available, bool BAR_available, float32_t (*accel_IMU)[3],
    float32_t (*omega_IMU)[3], float32_t (*z_GPS)[6], float32_t (*z_MAG)[3], float32_t (*z_BAR)[1], float32_t (*pos)[3], float32_t(*vel)[3],
    float32_t(*quat)[4], float32_t (*B_e)[3], arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R_GPS,
    arm_matrix_instance_f32 *R_MAG, arm_matrix_instance_f32 *R_BAR, float32_t p_ref_BAR, float32_t t_ref_air, float32_t h);

/*  Inputs: P (solo declarada, sin inicializar), Q (solo declarada, sin inicializar), R_GPS (SÍ INICIALIZADA, con el valor asociado a la posición y velocidad inicial),
    R_MAG (solo declarada, sin inicializar), R_BAR (solo declarada, sin inicializar), P_data (solo declarada, sin inicializar)
    Q_data (solo declarada, sin inicializar), R_MAG_data (solo declarada, sin inicializar), R_BAR_data (solo declarada, sin inicializar),
    eul_deg_0 (solo declarada, SÍ INICIALIZADA. Vector de actitud inicial en grados, siguiendo el resto de convenciones de kalman_eul2quat), quat (solo declarada, sin inicializar), h
    Outputs: P, Q, R_MAG, R_BAR, quat (ya inicializadas todas)
    Esta función debe ir precedida de las declaraciones de las matrices de salida y sus respectivos vectores de valores.
    Su objetivo es inicializar dichas matrices y construir matrices internas de la librería, por lo que debe llamarse antes
    de utilizar el filtro. */
void kalman_init(arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R_GPS,
    arm_matrix_instance_f32 *R_MAG, arm_matrix_instance_f32 *R_BAR, float32_t (*P_data)[H_P_A_Q_COLS * H_P_A_Q_COLS],
    float32_t (*Q_data)[H_P_A_Q_COLS * H_P_A_Q_COLS], float32_t (*R_MAG_data)[3 * 3], float32_t (*R_BAR_data)[1 * 1],
    float32_t (*eul_deg_0)[3], float32_t (*quat)[4], float32_t h);

#endif