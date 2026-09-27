#ifndef HOST_BMP581_H
#define HOST_BMP581_H

#include <stdint.h>

#define BMP581_SENSOR_DATA_SIZE 6

typedef struct {
    float TemperatureC;
    float PressurePa;
    uint32_t SampleId;
} BMP581_SensorData_t;

#endif
