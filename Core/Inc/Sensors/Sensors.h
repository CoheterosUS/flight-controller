#ifndef MAILBOXES_H
#define MAILBOXES_H

#include "Sensors/IIS2MDCTR.h"
#include "Sensors/IIM42653.h"
#include "Sensors/BMP581.h"
#include "Sensors/ZOEM8Q.h"
#include "Utils/shared.h"

typedef struct {
    BMP581_SensorData_t Slot[2];
    volatile uint8_t WriteIndex;
} BMP581_Mailbox_t;

typedef struct {
    IIM42653_SensorData_t Slot[2];
    volatile uint8_t WriteIndex;
} IIM42653_Mailbox_t;

typedef struct {
    IIS2MDCTR_SensorData_t Slot[2];
    volatile uint8_t WriteIndex;
} IIS2MDCTR_Mailbox_t;

typedef struct {
    ZOEM8Q_SensorData_t Slot[2];
    volatile uint8_t WriteIndex;
} ZOEM8Q_Mailbox_t;

void Buzzer_Beep(uint32_t Duration);
void Buzzer_Beep_Counter(uint32_t BeepDuration, uint32_t BeepCount, uint32_t WaitDuration, bool UseHAL);

void BMP581_Mailbox_Publish(const uint8_t *RXBuffer);
void BMP581_Mailbox_Inject(const BMP581_SensorData_t *Data);
void BMP581_Mailbox_Read(BMP581_SensorData_t *Out);
void IIM42653_Mailbox_Publish(const uint8_t *RXBuffer);
void IIM42653_Mailbox_Inject(const IIM42653_SensorData_t *Data);
void IIM42653_Mailbox_Read(IIM42653_SensorData_t *Out);
void IIS2MDCTR_Mailbox_Publish(const uint8_t *RXBuffer);
void IIS2MDCTR_Mailbox_Inject(const IIS2MDCTR_SensorData_t *Data);
void IIS2MDCTR_Mailbox_Read(IIS2MDCTR_SensorData_t *Out);
void ZOEM8Q_Mailbox_Inject(const ZOEM8Q_SensorData_t *Data);
void ZOEM8Q_Mailbox_Read(ZOEM8Q_SensorData_t *Out);

HAL_StatusTypeDef BMP581_Mode_Idle(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_Mode_Performance(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef IIM42653_Mode_Idle(SPI_HandleTypeDef *IIM42653_Handle);
HAL_StatusTypeDef IIM42653_Mode_Performance(SPI_HandleTypeDef *IIM42653_Handle);
HAL_StatusTypeDef IIS2MDCTR_Mode_Idle(I2C_HandleTypeDef *IIS2MDCTR_Handle);
HAL_StatusTypeDef IIS2MDCTR_Mode_Performance(I2C_HandleTypeDef *IIS2MDCTR_Handle);

#endif // MAILBOXES_H
