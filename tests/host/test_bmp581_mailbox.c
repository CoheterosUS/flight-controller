#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define PRESSURE_SCALE (1.0f / 64.0f)
#define TEMPERATURE_SCALE (1.0f / 65536.0f)

float CalculatePressureTemperature(uint8_t MSB, uint8_t LSB, uint8_t XLSB, bool Temperature) {
    int32_t RawValue = (int32_t)((MSB << 16) | (LSB << 8) | XLSB);
    int32_t SignValue = (RawValue << 8) >> 8;

    return (float)SignValue * (Temperature ? TEMPERATURE_SCALE : PRESSURE_SCALE);
}

#include "../../Core/Src/Sensors/BMP581/BMP581Mailbox.c"

int main(void) {
    const uint8_t FirstFrame[BMP581_SENSOR_DATA_SIZE] = {1, 2, 3, 4, 5, 6};
    uint8_t ChangedFrame[BMP581_SENSOR_DATA_SIZE] = {1, 2, 3, 4, 5, 7};
    BMP581_SensorData_t Data;

    BMP581_Mailbox_Publish(FirstFrame);
    BMP581_Mailbox_Read(&Data);
    assert(Data.SampleId == 1);
    uint8_t WriteIndex = BMP581_Mailbox.WriteIndex;

    BMP581_Mailbox_Publish(FirstFrame);
    BMP581_Mailbox_Read(&Data);
    assert(Data.SampleId == 1);
    assert(BMP581_Mailbox.WriteIndex == WriteIndex);

    BMP581_Mailbox_Publish(ChangedFrame);
    BMP581_Mailbox_Read(&Data);
    assert(Data.SampleId == 2);

    BMP581_SampleCounter = UINT32_MAX;
    ChangedFrame[5]++;
    BMP581_Mailbox_Publish(ChangedFrame);
    BMP581_Mailbox_Read(&Data);
    assert(Data.SampleId == 1);

    BMP581_Mailbox_Inject(&Data);
    BMP581_Mailbox_Read(&Data);
    assert(Data.SampleId == 2);

    BMP581_Mailbox_Inject(&Data);
    BMP581_Mailbox_Read(&Data);
    assert(Data.SampleId == 3);

    puts("BMP581 mailbox host tests passed");
    return 0;
}
