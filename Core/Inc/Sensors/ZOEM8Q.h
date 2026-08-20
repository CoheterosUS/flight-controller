#ifndef ZOEM8Q_H
#define ZOEM8Q_H

#include <stdint.h>

#define ZOEM8Q_PAYLOAD_SIZE 19

typedef struct {
    uint32_t UnixTime;
    uint16_t Milliseconds;
    int32_t Latitude;
    int32_t Longitude;
    int32_t AltitudeMm;
    uint8_t Satellites;
} ZOEM8Q_SensorData_t;

void ZOEM8Q_ParsePayload(const uint8_t *Payload, ZOEM8Q_SensorData_t *Out);

#endif //ZOEM8Q_H
