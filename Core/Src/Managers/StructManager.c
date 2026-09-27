#include "Managers/StructManager.h"
#include <string.h>

SDLogRecord_t BuildSDLogRecord(const FlightData_t *FlightData) {
    SDLogRecord_t Record;

    Record.Sync = FlightData->Sync;
    Record.Tick = FlightData->Tick;
    Record.RawAccelX = FlightData->RawAccelX;
    Record.RawAccelY = FlightData->RawAccelY;
    Record.RawAccelZ = FlightData->RawAccelZ;
    Record.RawGyroX = FlightData->RawGyroX;
    Record.RawGyroY = FlightData->RawGyroY;
    Record.RawGyroZ = FlightData->RawGyroZ;
    Record.RawMagX = FlightData->RawMagX;
    Record.RawMagY = FlightData->RawMagY;
    Record.RawMagZ = FlightData->RawMagZ;
    Record.CalAccelX = FlightData->CalAccelX;
    Record.CalAccelY = FlightData->CalAccelY;
    Record.CalAccelZ = FlightData->CalAccelZ;
    Record.CalGyroX = FlightData->CalGyroX;
    Record.CalGyroY = FlightData->CalGyroY;
    Record.CalGyroZ = FlightData->CalGyroZ;
    Record.PressurePa = FlightData->PressurePa;
    Record.TemperatureC = FlightData->TemperatureC;
    Record.Latitude = FlightData->Latitude;
    Record.Longitude = FlightData->Longitude;
    Record.GPSAltitude = FlightData->GPSAltitude;
    Record.UnixTime = FlightData->UnixTime;
    Record.Milliseconds = FlightData->Milliseconds;
    Record.Satellites = FlightData->Satellites;
    Record.PosX = FlightData->PosX;
    Record.PosY = FlightData->PosY;
    Record.PosZ = FlightData->PosZ;
    Record.VelX = FlightData->VelX;
    Record.VelY = FlightData->VelY;
    Record.VelZ = FlightData->VelZ;
    Record.QuatW = FlightData->QuatW;
    Record.QuatX = FlightData->QuatX;
    Record.QuatY = FlightData->QuatY;
    Record.QuatZ = FlightData->QuatZ;
    Record.BarometricAltitude = FlightData->BarometricAltitude;
    memcpy(Record.PDiag, FlightData->PDiag, sizeof(Record.PDiag));
    Record.Flags = FlightData->Flags;
    Record.BatteryVoltage = FlightData->BatteryVoltage;
    Record.State = FlightData->State;
    Record.RelayState = FlightData->RelayState;
    Record.LastCommand = FlightData->LastCommand;
    Record.CalStatus = FlightData->CalStatus;
    Record.SyncEnd = FlightData->SyncEnd;
    Record.ApogeeTrigger = FlightData->ApogeeTrigger;

    return Record;
}

TelemetryPacket_t BuildTelemetryPacket(const FlightData_t *FlightData) {
    TelemetryPacket_t Packet;

    Packet.Sync = FlightData->Sync;
    Packet.Tick = FlightData->Tick;
    Packet.CalAccelX = (int16_t)FlightData->CalAccelX;
    Packet.CalAccelY = (int16_t)FlightData->CalAccelY;
    Packet.CalAccelZ = (int16_t)FlightData->CalAccelZ;
    Packet.CalGyroX = (int16_t)FlightData->CalGyroX;
    Packet.CalGyroY = (int16_t)FlightData->CalGyroY;
    Packet.CalGyroZ = (int16_t)FlightData->CalGyroZ;
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
    Packet.CalStatus = FlightData->CalStatus;
    Packet.SyncEnd = FlightData->SyncEnd;

    return Packet;
}

FlashLogRecord_t BuildFlashLogRecord(const FlightData_t *FlightData) {
    FlashLogRecord_t Record;

    Record.Sync = FlightData->Sync;
    Record.Tick = FlightData->Tick;
    Record.RawAccelX = FlightData->RawAccelX;
    Record.RawAccelY = FlightData->RawAccelY;
    Record.RawAccelZ = FlightData->RawAccelZ;
    Record.RawGyroX = FlightData->RawGyroX;
    Record.RawGyroY = FlightData->RawGyroY;
    Record.RawGyroZ = FlightData->RawGyroZ;
    Record.PosX = (int16_t)FlightData->PosX;
    Record.PosY = (int16_t)FlightData->PosY;
    Record.PosZ = (int16_t)FlightData->PosZ;
    Record.VelX = (int16_t)(FlightData->VelX * 10.0f);
    Record.VelY = (int16_t)(FlightData->VelY * 10.0f);
    Record.VelZ = (int16_t)(FlightData->VelZ * 10.0f);
    Record.QuatW = (int16_t)(FlightData->QuatW * 10000.0f);
    Record.QuatX = (int16_t)(FlightData->QuatX * 10000.0f);
    Record.QuatY = (int16_t)(FlightData->QuatY * 10000.0f);
    Record.QuatZ = (int16_t)(FlightData->QuatZ * 10000.0f);
    Record.PDiag2 = (int16_t)(FlightData->PDiag[2] * 10.0f);
    Record.PressurePa = (uint16_t)(FlightData->PressurePa / 10.0f);
    Record.State = FlightData->State;
    Record.SyncEnd = FlightData->SyncEnd;
    Record.ApogeeTrigger = FlightData->ApogeeTrigger;

    return Record;
}
