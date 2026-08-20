#include "Sensors/ZOEM8Q.h"
#include "Sensors/Sensors.h"

static ZOEM8Q_Mailbox_t ZOEM8Q_Mailbox = {0};

void ZOEM8Q_Mailbox_Inject(const ZOEM8Q_SensorData_t *Data) {
    uint8_t wi = ZOEM8Q_Mailbox.WriteIndex;
    ZOEM8Q_Mailbox.Slot[wi] = *Data;
    ZOEM8Q_Mailbox.WriteIndex = 1 - wi;
}

void ZOEM8Q_Mailbox_Read(ZOEM8Q_SensorData_t *Out) {
    uint8_t ri = 1 - ZOEM8Q_Mailbox.WriteIndex;
    *Out = ZOEM8Q_Mailbox.Slot[ri];
}
