#ifndef CALIBRATIONS_H
#define CALIBRATIONS_H

#include <stdint.h>
#include "shared.h"

void ResetCalibrationContext(SystemContext_t *ctx);
void CalibrateGyroscope(FlightData_t FlightData, SystemContext_t *SystemContext);
void CalibratePressure(FlightData_t FlightData, SystemContext_t *SystemContext);
void CalibrateTemperature(FlightData_t FlightData, SystemContext_t *SystemContext);
void CalibrateAccelBias(FlightData_t FlightData, SystemContext_t *SystemContext);

#endif //CALIBRATIONS_H
