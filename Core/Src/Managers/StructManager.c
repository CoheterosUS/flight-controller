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
    Packet.AccelX = (int32_t)(FlightData->AccelX * 100.0f);
    Packet.AccelY = (int32_t)(FlightData->AccelY * 100.0f);
    Packet.AccelZ = (int32_t)(FlightData->AccelZ * 100.0f);
    Packet.GyroX = (int32_t)(FlightData->GyroX * 100.0f);
    Packet.GyroY = (int32_t)(FlightData->GyroY * 100.0f);
    Packet.GyroZ = (int32_t)(FlightData->GyroZ * 100.0f);
    Packet.MagX = (int32_t)(FlightData->MagX * 100.0f);
    Packet.MagY = (int32_t)(FlightData->MagY * 100.0f);
    Packet.MagZ = (int32_t)(FlightData->MagZ * 100.0f);
    Packet.PressurePa = (int32_t)(FlightData->PressurePa * 100.0f);
    Packet.TemperatureC = (int32_t)(FlightData->TemperatureC * 100.0f);
    Packet.Latitude = FlightData->Latitude;
    Packet.Longitude = FlightData->Longitude;
    Packet.GPSAltitude = (int32_t)(FlightData->GPSAltitude * 100.0f);
    Packet.Satellites = FlightData->Satellites;
    Packet.BarometricAltitude = (int32_t)(FlightData->BarometricAltitude * 100.0f);
    Packet.BarometricVelocity = (int32_t)(FlightData->BarometricVelocity * 100.0f);
    Packet.VelX = (int32_t)(FlightData->VelX * 100.0f);
    Packet.VelY = (int32_t)(FlightData->VelY * 100.0f);
    Packet.VelZ = (int32_t)(FlightData->VelZ * 100.0f);
    Packet.Flags = FlightData->Flags;
    Packet.BatteryVoltage = (int32_t)(FlightData->BatteryVoltage * 100.0f);
    Packet.State = FlightData->State;
    Packet.RelayState = FlightData->RelayState;
    Packet.LastCommand = FlightData->LastCommand;
    Packet.SyncEnd = FlightData->SyncEnd;

    return Packet;
}

FlashLogRecord_t BuildFlashLogRecord(const FlightData_t *FlightData) {
    FlashLogRecord_t Record;

    Record.Sync = FlightData->Sync;
    Record.Tick = FlightData->Tick;
    Record.AccelX = FlightData->AccelX;
    Record.AccelY = FlightData->AccelY;
    Record.AccelZ = FlightData->AccelZ;
    Record.GyroX = FlightData->GyroX;
    Record.GyroY = FlightData->GyroY;
    Record.GyroZ = FlightData->GyroZ;
    Record.State = FlightData->State;
    Record.SyncEnd = FlightData->SyncEnd;

    return Record;
}
