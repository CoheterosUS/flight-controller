#include "Sensors/ZOEM8Q.h"

static uint32_t ReadU32(const uint8_t *Buf) {
    return (uint32_t)Buf[0]
         | (uint32_t)Buf[1] << 8
         | (uint32_t)Buf[2] << 16
         | (uint32_t)Buf[3] << 24;
}

static uint16_t ReadU16(const uint8_t *Buf) {
    return (uint16_t)Buf[0]
         | (uint16_t)Buf[1] << 8;
}

static int32_t ReadI32(const uint8_t *Buf) {
    return (int32_t)ReadU32(Buf);
}

void ZOEM8Q_ParsePayload(const uint8_t *Payload, ZOEM8Q_SensorData_t *Out) {
    Out->UnixTime = ReadU32(&Payload[0]);
    Out->Milliseconds = ReadU16(&Payload[4]);
    Out->Latitude = ReadI32(&Payload[6]);
    Out->Longitude = ReadI32(&Payload[10]);
    Out->AltitudeMm = ReadI32(&Payload[14]);
    Out->Satellites = Payload[18];
}
