#include "HIL/HIL.h"

#if HIL_MODE

#include <string.h>
#include "Sensors/Sensors.h"

void HandleHILPacket(const uint8_t *Payload, uint8_t PayloadLength) {
    IIM42653_SensorData_t IMUData = {0};
    IIS2MDCTR_SensorData_t MagData = {0};

    if (PayloadLength != HIL_DATA_PAYLOAD_SIZE) return;

    memcpy(&IMUData.AccelX, &Payload[HIL_ACCEL_X], 4);
    memcpy(&IMUData.AccelY, &Payload[HIL_ACCEL_Y], 4);
    memcpy(&IMUData.AccelZ, &Payload[HIL_ACCEL_Z], 4);
    memcpy(&IMUData.GyroX, &Payload[HIL_GYRO_X], 4);
    memcpy(&IMUData.GyroY, &Payload[HIL_GYRO_Y], 4);
    memcpy(&IMUData.GyroZ, &Payload[HIL_GYRO_Z], 4);
    memcpy(&MagData.MagX, &Payload[HIL_MAG_X], 4);
    memcpy(&MagData.MagY, &Payload[HIL_MAG_Y], 4);
    memcpy(&MagData.MagZ, &Payload[HIL_MAG_Z], 4);

    IIM42653_Mailbox_Inject(&IMUData);
    IIS2MDCTR_Mailbox_Inject(&MagData);
}

// One packet is one barometer sample: the mailbox assigns a new sample id per inject.
void HandleHILBaroPacket(const uint8_t *Payload, uint8_t PayloadLength) {
    BMP581_SensorData_t BaroData = {0};

    if (PayloadLength != HIL_BARO_PAYLOAD_SIZE) return;

    memcpy(&BaroData.PressurePa, &Payload[HIL_BARO_PRESSURE], 4);
    memcpy(&BaroData.TemperatureC, &Payload[HIL_BARO_TEMPERATURE], 4);

    BMP581_Mailbox_Inject(&BaroData);
}

#endif
