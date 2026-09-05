#ifndef STATEHANDLERS_H
#define STATEHANDLERS_H

#include "Utils/shared.h"

void IdleStateEntry(SystemContext_t *Context);
void CalibrationStateEntry(SystemContext_t *Context);
void PrelaunchStateEntry(SystemContext_t *Context);
void BoostStateEntry(SystemContext_t *Context);
void CoastStateEntry(SystemContext_t *Context);
void ActiveControlStateEntry(SystemContext_t *Context);
void ApogeeStateEntry(SystemContext_t *Context);
void MainParachuteStateEntry(SystemContext_t *Context);
void LandedStateEntry(SystemContext_t *Context);
void GroundAbortStateEntry(SystemContext_t *Context);

SystemState_t IdleStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t CalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t PrelaunchStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t BoostStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t CoastStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t ActiveControlStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t ApogeeStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t MainParachuteStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t LandedStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t GroundAbortStateHandler(SystemContext_t *Context, FlightData_t FlightData);

#endif // STATEHANDLERS_H
