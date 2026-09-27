#ifndef HOST_SENSORS_H
#define HOST_SENSORS_H

#include "Sensors/BMP581.h"

typedef struct {
    BMP581_SensorData_t Slot[2];
    volatile uint8_t WriteIndex;
} BMP581_Mailbox_t;

void BMP581_Mailbox_Publish(const uint8_t *RXBuffer);
void BMP581_Mailbox_Inject(const BMP581_SensorData_t *Data);
void BMP581_Mailbox_Read(BMP581_SensorData_t *Out);

#endif
