#ifndef STRUCTMANAGER_H
#define STRUCTMANAGER_H

#include "Utils/shared.h"

#pragma pack(push, 1)
typedef struct {
    uint16_t Sync;
    uint32_t Tick;
    float AccelX;
    float AccelY;
    float AccelZ;
    float GyroX;
    float GyroY;
    float GyroZ;
    float MagX;
    float MagY;
    float MagZ;
    float PressurePa;
    float TemperatureC;
    int32_t Latitude;
    int32_t Longitude;
    float GPSAltitude;
    uint32_t UnixTime;
    uint16_t Milliseconds;
    uint8_t Satellites;
    uint32_t Flags;
    float BatteryVoltage;
    uint8_t State;
    uint8_t RelayState;
    uint8_t LastCommand;
    uint8_t SyncEnd;
} SDLogRecord_t;

typedef struct {
    uint16_t Sync;
    uint32_t Tick;
    int32_t AccelX;
    int32_t AccelY;
    int32_t AccelZ;
    int32_t GyroX;
    int32_t GyroY;
    int32_t GyroZ;
    int32_t MagX;
    int32_t MagY;
    int32_t MagZ;
    int32_t PressurePa;
    int32_t TemperatureC;
    int32_t Latitude;
    int32_t Longitude;
    int32_t GPSAltitude;
    uint8_t Satellites;
    int32_t BarometricAltitude;
    int32_t BarometricVelocity;
    int32_t VelX;
    int32_t VelY;
    int32_t VelZ;
    uint32_t Flags;
    int32_t BatteryVoltage;
    uint8_t State;
    uint8_t RelayState;
    uint8_t LastCommand;
    uint8_t SyncEnd;
} TelemetryPacket_t;

typedef struct {
    uint16_t Sync;
    uint32_t Tick;
    float AccelX;
    float AccelY;
    float AccelZ;
    float GyroX;
    float GyroY;
    float GyroZ;
    uint8_t State;
    uint8_t SyncEnd;
} FlashLogRecord_t;
#pragma pack(pop)

SDLogRecord_t BuildSDLogRecord(const FlightData_t *FlightData);
TelemetryPacket_t BuildTelemetryPacket(const FlightData_t *FlightData);
FlashLogRecord_t BuildFlashLogRecord(const FlightData_t *FlightData);

#endif //STRUCTMANAGER_H
