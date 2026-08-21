#ifndef STATEHANDLERS_H
#define STATEHANDLERS_H

#include "Utils/shared.h"

void IdleStateEntry(SystemContext_t *Context);
void CalibrationStateEntry(SystemContext_t *Context);
void PrelaunchStateEntry(SystemContext_t *Context);
void BurnStateEntry(SystemContext_t *Context);
void PassiveBurnoutStateEntry(SystemContext_t *Context);
void ActiveBurnoutStateEntry(SystemContext_t *Context);
void ApogeeStateEntry(SystemContext_t *Context);
void ParachuteStateEntry(SystemContext_t *Context);
void LandedStateEntry(SystemContext_t *Context);
void GroundAbortStateEntry(SystemContext_t *Context);

SystemState_t IdleStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t CalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t PrelaunchStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t BurnStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t PassiveBurnoutStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t ActiveBurnoutStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t ApogeeStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t ParachuteStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t LandedStateHandler(SystemContext_t *Context, FlightData_t FlightData);
SystemState_t GroundAbortStateHandler(SystemContext_t *Context, FlightData_t FlightData);

#endif // STATEHANDLERS_H
