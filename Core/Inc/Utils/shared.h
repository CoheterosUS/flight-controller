#ifndef SHARED_H
#define SHARED_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "configuration.h"
#include "stm32h7xx_hal.h"
#include "timers.h"

#define BIT(n) (1U << (n))
#define BITS_SET(value, mask) (((value) & (mask)) == (mask))
#define BITS_CLEAR(value, mask) (((value) & (mask)) == 0)

typedef enum {
    MSG_DATA = 0x01,
    MSG_COMMAND = 0x02
} MsgType_t;

typedef enum {
    STATE_IDLE,         	// Initialize sensors at low power
    STATE_CALIBRATION,  	// Remote command, maximize sensor output for calibration
    STATE_PRELAUNCH,    	// Initialize SD logging
    STATE_BOOST,            // Acceleration detected
    STATE_COAST,            // Burnout, still ascending
    STATE_ACTIVE_CONTROL,   // Active control
    STATE_APOGEE,           // Apogee detected, drogue parachute deploy
    STATE_MAIN_PARACHUTE,   // Altitude threshold, main parachute deploy
    STATE_LANDED,       	// Landed, turn off except GPS

    STATE_GROUND_ABORT,
    STATE_DESCENT_ABORT,

    STATE_MAX           // Table size
} SystemState_t;

// TODO: Refine
typedef struct {
    float ReferencePressurePa;
    bool ReferencePressurePaValid;
    float GyroBiasX;
    float GyroBiasY;
    float GyroBiasZ;
    bool GyroCalibrationValid;
    volatile bool AltitudeFilterInitialized;
    volatile bool SDLoggingEnabled;
    volatile bool SensorsIdleFinished;
    bool GPSFixValid;
    uint32_t StateEntryTick;
    uint32_t StateEntryTicks[STATE_MAX];
} SystemContext_t;

typedef enum {
    BMP280_MODE_IDLE_FAILED = 1u << 0,
    BMP280_MODE_PERFORMANCE_FAILED = 1u << 1,
    BMP581_MODE_IDLE_FAILED = 1u << 2,
    BMP581_MODE_PERFORMANCE_FAILED = 1u << 3,
    IIM42653_MODE_IDLE_FAILED = 1u << 4,
    IIM42653_MODE_PERFORMANCE_FAILED = 1u << 5,
    IIS2MDCTR_MODE_IDLE_FAILED = 1u << 6,
    IIS2MDCTR_MODE_PERFORMANCE_FAILED = 1u << 7,
    SD_MOUNT_FAILED = 1u << 8,
    SD_OPEN_FAILED = 1u << 9,
    // TODO: Refine
} SystemFaultFlag_t;

typedef uint32_t SystemFaultFlags_t;

#pragma pack(push, 1)
typedef struct {
    uint16_t Sync;
    uint32_t Tick;
    float AccelX;
    float AccelY;
    float AccelZ;
    float GyroX;
    float GyroY;
    float GyroZ;
    float MagX;
    float MagY;
    float MagZ;
    float PressurePa;
    float TemperatureC;
    int32_t Latitude;
    int32_t Longitude;
    float GPSAltitude;
    uint32_t UnixTime;
    uint16_t Milliseconds;
    uint8_t Satellites;
    float BarometricAltitude;
    float BarometricVelocity;
    float GPSVelocity;
    float VelX;
    float VelY;
    float VelZ;
    uint32_t Flags;
    float BatteryVoltage;
    uint8_t State;
    uint8_t RelayState;
    uint8_t LastCommand;
    uint8_t SyncEnd;
} FlightData_t;
#pragma pack(pop)

extern QueueHandle_t SDLoggingQueue;
extern QueueHandle_t CommandQueue;

extern TimerHandle_t TimerIIM42653;
extern TimerHandle_t TimerBMP581;
extern TimerHandle_t TimerIIS2MDCTR;
extern TimerHandle_t TimerBattery;

extern SystemFaultFlags_t SystemFaultFlags;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SD_HandleTypeDef hsd1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

#define BMP581_HANDLE      (&hi2c2)
#define IIM42653_HANDLE    (&hspi2)
#define IIS2MDCTR_HANDLE   (&hi2c1)
#define SD_HANDLE          (&hsd1)
#define TIM2_HANDLE        (&htim2)
#define USART1_HANDLE      (&huart1)
#define ADC1_HANDLE        (&hadc1)

#endif //SHARED_H
