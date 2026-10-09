/*  
    Librería de calibración por seis puntos para IMU por José Federico Tenor.
    
    Se trata de una implementación del algoritmo de mínimos cuadrados descrito en
    https://www.st.com/resource/en/application_note/an4508-parameters-and-calibration-of-a-lowg-3axis-accelerometer-stmicroelectronics.pdf

    Además de la calibración de escala y offset, esta librería permite obtener la matriz de rotación que lleva las medidas de la IMU a los ejes del vehículo (body).
    Se supone que las primeras N medidas de la IMU corresponden a la orientación body +X, las siguientes N a -X, las siguientes N a +Y, las siguientes N a -Y, las siguientes N a +Z y las últimas N a -Z.
    Adicionalmente, se descompone la matrix completa de calibración (X) en una matriz de calibración sin offset (M), y en una matriz de rotación (DCM_bIMU) mediante QR.
    De este modo, para IMUs en las que la orientación de los ejes de acelerómetro coincide con la de los ejes de giroscopio, se puede aplicar DCM_bIMU al giroscopio para que sus medidas también estén en body.

    Esta librería utiliza la librería DSP de CMSIS.
*/

#ifndef CAL_H
#define CAL_H

#include "dsp/arm_math.h"
#include "Utils/configuration.h"

#define N DEEP_CALIBRATION_SAMPLES

typedef enum six_point_cal_ErrorCode {

    SUCCESS_SIX_POINT_CAL = 0,
    W_INVALID_SIZE_SIX_POINT_CAL = -1,
    SINGULAR_MATRIX_SIX_POINT_CAL = -2,
    X_INVALID_SIZE_SIX_POINT_CAL = -3,
    DCM_bIMU_INVALID_SIZE_SIX_POINT_CAL = -4,
    M_INVALID_SIZE_SIX_POINT_CAL = -5

} six_point_cal_ErrorCode;

/*  N número de medidas por cada orientación, define
    Y matriz de g, tamaño 6nx3
    w (entrada) matriz de medidas raw con 1 en la cuarta columna, tamaño 6nx4
    X (salida) matriz de calibración, tamaño 4x3
    DCM_bIMU (salida) matriz de rotación IMU a body, tamaño 3x3
    M (salida) matriz de calibración sin offset (rotación y escala), tamaño 3x3 */
six_point_cal_ErrorCode six_point_cal(arm_matrix_instance_f32 *w, arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *DCM_bIMU, arm_matrix_instance_f32 *M);

#endif // CAL_H