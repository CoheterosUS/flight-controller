#include "Sensors/BMP581.h"
#include "Utils/Calculations.h"
#include "Sensors/Sensors.h"

static BMP581_Mailbox_t BMP581_Mailbox = {0};

void BMP581_Mailbox_Publish(const uint8_t *RXBuffer) {
    uint8_t wi = BMP581_Mailbox.WriteIndex;
    BMP581_Mailbox.Slot[wi].TemperatureC = CalculatePressureTemperature(RXBuffer[2], RXBuffer[1], RXBuffer[0], true);
    BMP581_Mailbox.Slot[wi].PressurePa = CalculatePressureTemperature(RXBuffer[5], RXBuffer[4], RXBuffer[3], false);

    BMP581_Mailbox.Sequence++;
    BMP581_Mailbox.WriteIndex = 1 - wi;
}

void BMP581_Mailbox_Inject(const BMP581_SensorData_t *Data) {
    uint8_t wi = BMP581_Mailbox.WriteIndex;
    BMP581_Mailbox.Slot[wi] = *Data;
    BMP581_Mailbox.Sequence++;
    BMP581_Mailbox.WriteIndex = 1 - wi;
}

void BMP581_Mailbox_Read(BMP581_SensorData_t *Out, uint32_t *Sequence) {
    uint8_t ri = 1 - BMP581_Mailbox.WriteIndex;
    *Out = BMP581_Mailbox.Slot[ri];
    *Sequence = BMP581_Mailbox.Sequence;
}
