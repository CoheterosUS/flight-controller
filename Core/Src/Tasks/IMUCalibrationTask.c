#include "Tasks/IMUCalibrationTask.h"
#include "Sensors/Sensors.h"
#include "Utils/shared.h"
#include "Utils/Calculations.h"
#include "fatfs.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static TaskHandle_t IMUCalibrationTaskHandle;

void CreateIMUCalibrationTask(const UBaseType_t Priority, const uint16_t StackSize) {
    xTaskCreate(
        IMUCalibrationTask,
        "IMU_CAL",
        StackSize,
        NULL,
        Priority,
        &IMUCalibrationTaskHandle
    );
}

static void WriteCalibrationFile(float R[3][3]) {
    FRESULT Result = f_mount(&SDFatFS, SDPath, 1);
    if (Result != FR_OK) {
        Buzzer_Beep_Counter(100, 10, 100, false);
        return;
    }

    FIL File;
    Result = f_open(&File, "CAL.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    if (Result != FR_OK) {
        Buzzer_Beep_Counter(100, 10, 100, false);
        f_mount(NULL, SDPath, 1);
        return;
    }

    char Buf[128];
    UINT Written;

    snprintf(Buf, sizeof(Buf), "// IMU Rotation Matrix (paste into configuration.h)\r\n");
    f_write(&File, Buf, strlen(Buf), &Written);

    snprintf(Buf, sizeof(Buf), "#define IMU_ROTATION_ENABLED  1\r\n\r\n");
    f_write(&File, Buf, strlen(Buf), &Written);

    const char *Names[] = {
        "IMU_R00", "IMU_R01", "IMU_R02",
        "IMU_R10", "IMU_R11", "IMU_R12",
        "IMU_R20", "IMU_R21", "IMU_R22"
    };

    for (int Row = 0; Row < 3; Row++) {
        for (int Col = 0; Col < 3; Col++) {
            snprintf(Buf, sizeof(Buf), "#define %s  %+.8ff\r\n", Names[Row * 3 + Col], (double)R[Row][Col]);
            f_write(&File, Buf, strlen(Buf), &Written);
        }
    }

    f_sync(&File);
    f_close(&File);
    f_mount(NULL, SDPath, 1);
}

void IMUCalibrationTask(void *pvParameters) {
    (void)pvParameters;

    Buzzer_Beep_Counter(100, 2, 300, false);

    if (IIM42653_Mode_Performance(IIM42653_HANDLE) != HAL_OK) {
        Buzzer_Beep_Counter(100, 10, 100, false);
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    xTimerStart(TimerIIM42653, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(500));

    for (uint16_t i = 0; i < IMU_CAL_DISCARD_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Buzzer_Beep(50);

    float SumX = 0.0f, SumY = 0.0f, SumZ = 0.0f;

    for (uint16_t i = 0; i < IMU_CAL_SAMPLES; i++) {
        IIM42653_SensorData_t Data;
        IIM42653_Mailbox_Read(&Data);

        SumX += Data.AccelX;
        SumY += Data.AccelY;
        SumZ += Data.AccelZ;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    xTimerStop(TimerIIM42653, portMAX_DELAY);

    float InvN = 1.0f / (float)IMU_CAL_SAMPLES;
    float Gx = SumX * InvN;
    float Gy = SumY * InvN;
    float Gz = SumZ * InvN;

    float Norm = sqrtf(Gx * Gx + Gy * Gy + Gz * Gz);
    if (Norm < 0.1f) {
        Buzzer_Beep_Counter(100, 10, 100, false);
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    Gx /= Norm;
    Gy /= Norm;
    Gz /= Norm;

    float Pitch = atan2f(Gx, -Gy);
    float Roll  = atan2f(Gz, -Gy);

    float CP = cosf(Pitch), SP = sinf(Pitch);
    float CR = cosf(Roll),  SR = sinf(Roll);

    // R = Rx(Roll) * Rz(Pitch)
    float R[3][3] = {
        {  CP,      -SP,       0.0f },
        {  CR * SP,  CR * CP, -SR   },
        {  SR * SP,  SR * CP,  CR   }
    };

    WriteCalibrationFile(R);

    Buzzer_Beep_Counter(200, 3, 300, false);

    for (;;) vTaskDelay(portMAX_DELAY);
}
