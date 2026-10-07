#include "Sensors/Sensors.h"
#include "Managers/Managers.h"
#include "stm32h7xx_hal.h"
#include <string.h>

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t SERIAL_TX_BUFFER[2][sizeof(TelemetryPacket_t)];

static uint8_t ActiveTXIndex;
static volatile bool TXBusy;

volatile uint8_t dbg_telem_sent = 0;

void SerialInit(void) {
    ActiveTXIndex = 0;
    TXBusy = false;
}

void SerialSendFlightData(const TelemetryPacket_t *Packet) {
    if (TXBusy) return;

    uint8_t *Buf = SERIAL_TX_BUFFER[ActiveTXIndex];
    memcpy(Buf, Packet, sizeof(TelemetryPacket_t));

    TXBusy = true;
    HAL_UART_Transmit_DMA(USART1_HANDLE, Buf, sizeof(TelemetryPacket_t));
    dbg_telem_sent++;
    ActiveTXIndex ^= 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        TXBusy = false;
    }
}
