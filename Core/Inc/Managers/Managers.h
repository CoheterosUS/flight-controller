#ifndef MANAGERS_H
#define MANAGERS_H

#include "Protocol/Protocol.h"
#include "Managers/StructManager.h"

void SerialInit(void);
void SerialSendFlightData(const TelemetryWirePacket_t *Packet, SystemState_t State);

void OnStateEntry(const SystemState_t CurrentSystemState, SystemContext_t *SystemContext);
void HandleSensors(SystemContext_t *SystemContext, SystemState_t CurrentSystemState);
SystemState_t HandleCommand(SystemState_t CurrentSystemState, SystemContext_t *SystemContext, CommandType_t CommandType, BaseType_t Received);
SystemState_t HandleState(SystemState_t CurrentSystemState, SystemContext_t *SystemContext, FlightData_t FlightData);

#endif // MANAGERS_H
