#ifndef STRUCTMANAGER_H
#define STRUCTMANAGER_H

#include "Utils/shared.h"

#pragma pack(push, 1)
// Pos Filtro
// Vel Filtro
// Quat Filtro
// Mat P Filtro
// Barom Alt
typedef struct {
    uint16_t Sync;
    uint32_t Tick;
    float RawAccelX;
    float RawAccelY;
    float RawAccelZ;
    float RawGyroX;
    float RawGyroY;
    float RawGyroZ;
    float RawMagX;
    float RawMagY;
    float RawMagZ;
    float CalAccelX;
    float CalAccelY;
    float CalAccelZ;
    float CalGyroX;
    float CalGyroY;
    float CalGyroZ;
    float PressurePa;
    float TemperatureC;
    int32_t Latitude;
    int32_t Longitude;
    float GPSAltitude;
    uint32_t UnixTime;
    uint16_t Milliseconds;
    uint8_t Satellites;
    float BarometricAltitude;

    // KALMAN
    float PosX;
    float PosY;
    float PosZ;
    float VelX;
    float VelY;
    float VelZ;
    float QuatW;
    float QuatX;
    float QuatY;
    float QuatZ;

    float PDiag[9];

    uint32_t Flags;
    float BatteryVoltage;
    uint8_t State;
    uint8_t RelayState;
    uint8_t LastCommand;
    uint16_t CalStatus;
    uint8_t SyncEnd;
} SDLogRecord_t;

// Alt Filtro Kalman, 2 bytes
// Vel Filtro Kalman, Quitar BaromVel, 2 bytes cada uno Vel
// Quaternion Filtro Kalman (opt), 2 decimales, 1 byte por uno
// Tercer Valor Diagonal Mat P Filtro Kalman, 2 decimales, 2 bytes
typedef struct {
    uint16_t Sync;
    uint32_t Tick;
    int16_t CalAccelX;
    int16_t CalAccelY;
    int16_t CalAccelZ;
    int16_t CalGyroX;
    int16_t CalGyroY;
    int16_t CalGyroZ;
    int16_t PressurePa;
    int8_t TemperatureC;
    int32_t Latitude;
    int32_t Longitude;
    int32_t GPSAltitude;
    uint8_t Satellites;
    int32_t BarometricAltitude;
    int32_t BarometricVelocity;
    uint32_t Flags;
    int16_t BatteryVoltage;
    uint8_t State;
    uint8_t RelayState;
    uint8_t LastCommand;
    uint16_t CalStatus;
    uint8_t SyncEnd;
} TelemetryPacket_t;

typedef struct {
    uint16_t Sync;
    uint32_t Tick;
    float RawAccelX;
    float RawAccelY;
    float RawAccelZ;
    float RawGyroX;
    float RawGyroY;
    float RawGyroZ;
    int16_t PosX;
    int16_t PosY;
    int16_t PosZ;
    int16_t VelX;
    int16_t VelY;
    int16_t VelZ;
    int16_t QuatW;
    int16_t QuatX;
    int16_t QuatY;
    int16_t QuatZ;
    int16_t PDiag2;
    uint16_t PressurePa;
    uint8_t State;
    uint8_t SyncEnd;
} FlashLogRecord_t;
#pragma pack(pop)

_Static_assert(sizeof(FlashLogRecord_t) == 56, "FlashLogRecord_t layout changed");
_Static_assert(sizeof(FlashLogRecord_t) <= (256 / FLASH_RECORDS_PER_PAGE), "FlashLogRecord_t exceeds flash page slot");
_Static_assert(sizeof(TelemetryPacket_t) == 54, "TelemetryPacket_t layout changed");
_Static_assert(sizeof(SDLogRecord_t) == 187, "SDLogRecord_t layout changed");

SDLogRecord_t BuildSDLogRecord(const FlightData_t *FlightData);
TelemetryPacket_t BuildTelemetryPacket(const FlightData_t *FlightData);
FlashLogRecord_t BuildFlashLogRecord(const FlightData_t *FlightData);

#endif //STRUCTMANAGER_H
