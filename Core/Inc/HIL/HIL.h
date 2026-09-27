#ifndef HIL_H
#define HIL_H

#include <stdint.h>
#include "Utils/configuration.h"

// COMMAND_HIL_DATA: IMU and magnetometer at IMU_ODR_HZ. COMMAND_HIL_BARO: one barometer sample, sent at the
// real sensor rate (about 50 Hz), so the apogee confirmation time in HIL matches the hardware.
#define HIL_DATA_PAYLOAD_SIZE 36
#define HIL_BARO_PAYLOAD_SIZE 8

typedef enum {
    HIL_ACCEL_X  = 0,
    HIL_ACCEL_Y  = 4,
    HIL_ACCEL_Z  = 8,
    HIL_GYRO_X   = 12,
    HIL_GYRO_Y   = 16,
    HIL_GYRO_Z   = 20,
    HIL_MAG_X    = 24,
    HIL_MAG_Y    = 28,
    HIL_MAG_Z    = 32,
} HILPayloadOffset_t;

typedef enum {
    HIL_BARO_PRESSURE    = 0,
    HIL_BARO_TEMPERATURE = 4,
} HILBaroPayloadOffset_t;

#if HIL_MODE
void HandleHILPacket(const uint8_t *Payload, uint8_t PayloadLength);
void HandleHILBaroPacket(const uint8_t *Payload, uint8_t PayloadLength);
#endif

#endif //HIL_H
