#ifndef FLIGHTDATA_H
#define FLIGHTDATA_H

#include "Utils/shared.h"
#include "Sensors/IIM42653.h"
#include "Sensors/BMP581.h"
#include "Sensors/IIS2MDCTR.h"
#include "Sensors/ZOEM8Q.h"
#include "Protocol/Protocol.h"

FlightData_t GetFlightData(SystemState_t SystemState, SystemContext_t *SystemContext, IIM42653_SensorData_t IIM42653_SensorData, BMP581_SensorData_t BMP581_SensorData, IIS2MDCTR_SensorData_t IIS2MDCTR_SensorData, ZOEM8Q_SensorData_t ZOEM8Q_SensorData, CommandType_t LastCommand);

#endif // FLIGHTDATA_H
