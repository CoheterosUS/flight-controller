#include "Managers/StructManager.h"

SDLogRecord_t BuildSDLogRecord(const FlightData_t *FlightData) {
    SDLogRecord_t Record;

    Record.Sync = FlightData->Sync;
    Record.Tick = FlightData->Tick;
    Record.AccelX = FlightData->AccelX;
    Record.AccelY = FlightData->AccelY;
    Record.AccelZ = FlightData->AccelZ;
    Record.GyroX = FlightData->GyroX;
    Record.GyroY = FlightData->GyroY;
    Record.GyroZ = FlightData->GyroZ;
    Record.MagX = FlightData->MagX;
    Record.MagY = FlightData->MagY;
    Record.MagZ = FlightData->MagZ;
    Record.PressurePa = FlightData->PressurePa;
    Record.TemperatureC = FlightData->TemperatureC;
    Record.Latitude = FlightData->Latitude;
    Record.Longitude = FlightData->Longitude;
    Record.GPSAltitude = FlightData->GPSAltitude;
    Record.UnixTime = FlightData->UnixTime;
    Record.Milliseconds = FlightData->Milliseconds;
    Record.Satellites = FlightData->Satellites;
    Record.Flags = FlightData->Flags;
    Record.BatteryVoltage = FlightData->BatteryVoltage;
    Record.State = FlightData->State;
    Record.RelayState = FlightData->RelayState;
    Record.LastCommand = FlightData->LastCommand;
    Record.SyncEnd = FlightData->SyncEnd;

    return Record;
}

TelemetryPacket_t BuildTelemetryPacket(const FlightData_t *FlightData) {
    TelemetryPacket_t Packet;

    Packet.Sync = FlightData->Sync;
    Packet.Tick = FlightData->Tick;
    Packet.AccelX = (int16_t)FlightData->AccelX;
    Packet.AccelY = (int16_t)FlightData->AccelY;
    Packet.AccelZ = (int16_t)FlightData->AccelZ;
    Packet.GyroX = (int16_t)FlightData->GyroX;
    Packet.GyroY = (int16_t)FlightData->GyroY;
    Packet.GyroZ = (int16_t)FlightData->GyroZ;
    Packet.PressurePa = (int16_t)(FlightData->PressurePa / 10.0f);
    Packet.TemperatureC = (int8_t)FlightData->TemperatureC;
    Packet.Latitude = FlightData->Latitude;
    Packet.Longitude = FlightData->Longitude;
    Packet.GPSAltitude = (int32_t)(FlightData->GPSAltitude * 100.0f);
    Packet.Satellites = FlightData->Satellites;
    Packet.BarometricAltitude = (int32_t)(FlightData->BarometricAltitude * 100.0f);
    Packet.BarometricVelocity = (int32_t)(FlightData->BarometricVelocity * 100.0f);
    Packet.Flags = FlightData->Flags;
    Packet.BatteryVoltage = (int16_t)(FlightData->BatteryVoltage * 10.0f);
    Packet.State = FlightData->State;
    Packet.RelayState = FlightData->RelayState;
    Packet.LastCommand = FlightData->LastCommand;
    Packet.SyncEnd = FlightData->SyncEnd;

    return Packet;
}
