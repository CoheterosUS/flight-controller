#ifndef CAL_H
#define CAL_H

#include "dsp/arm_math.h"

#define N 500

/*  N número de medidas por cada orientación, define
    Y matriz de g normalizada, tamaño 6nx3
    w matriz de medidas raw con 1 en la cuarta columna, tamaño 6nx4
     X matriz de calibración, tamaño 4x3
     A_m matriz de rotación IMU a body, tamaño 3x3
     M matriz de calibración sin offset (rotación y escala), tamaño 3x3 */
void six_point_cal(arm_matrix_instance_f32 *w, arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A_m, arm_matrix_instance_f32 *M);

#endif // CAL_H