#include "States/StateHandlers.h"
#include "Sensors/Sensors.h"
#include "Utils/shared.h"
#include "Utils/Pyro.h"
#include "timers.h"
#include "Protocol/Protocol.h"
#include <Tasks/SensorConfigTask.h>

void StartSensorTimers(void) {
	xTimerStart(TimerIIM42653, 0);
	xTimerStart(TimerBMP581, 0);
	xTimerStart(TimerIIS2MDCTR, 0);
}

void StopSensorTimers(void) {
    xTimerStop(TimerIIM42653, portMAX_DELAY);
    xTimerStop(TimerBMP581, portMAX_DELAY);
    xTimerStop(TimerIIS2MDCTR, portMAX_DELAY);
}

void OnStateEntry(const SystemState_t CurrentSystemState, SystemContext_t *SystemContext) {
    SystemContext->StateEntryTick = xTaskGetTickCount();
    xTaskNotify(SensorConfigTaskHandle, (uint32_t)CurrentSystemState, eSetValueWithOverwrite);

    switch (CurrentSystemState) {
        case STATE_IDLE:
            IdleStateEntry(SystemContext);
            break;
        case STATE_CALIBRATION:
            CalibrationStateEntry(SystemContext);
            break;
        case STATE_PRELAUNCH:
            PrelaunchStateEntry(SystemContext);
            break;
        case STATE_BURN:
            BurnStateEntry(SystemContext);
            break;
        case STATE_PASSIVE_BURNOUT:
            PassiveBurnoutStateEntry(SystemContext);
            break;
        case STATE_ACTIVE_BURNOUT:
            ActiveBurnoutStateEntry(SystemContext);
            break;
        case STATE_APOGEE:
            ApogeeStateEntry(SystemContext);
            break;
        case STATE_PARACHUTE:
            ParachuteStateEntry(SystemContext);
            break;
        case STATE_LANDED:
            LandedStateEntry(SystemContext);
            break;
        case STATE_GROUND_ABORT:
        case STATE_ASCENT_ABORT:
        case STATE_DESCENT_ABORT:
            PyroSafeAll();
            break;
        default:
            break;
    }
}

void HandleSensors(SystemContext_t *SystemContext, SystemState_t CurrentSystemState) {
#if HIL_MODE
	(void)SystemContext;
	(void)CurrentSystemState;
#else
	switch (CurrentSystemState) {
		case STATE_IDLE:
			if (BMP581_Mode_Idle(BMP581_HANDLE) != HAL_OK) {
				SystemFaultFlags |= BMP581_MODE_IDLE_FAILED;
			}
			if (IIM42653_Mode_Idle(IIM42653_HANDLE) != HAL_OK) {
				SystemFaultFlags |= IIM42653_MODE_IDLE_FAILED;
			}
			if (IIS2MDCTR_Mode_Idle(IIS2MDCTR_HANDLE) != HAL_OK) {
				SystemFaultFlags |= IIS2MDCTR_MODE_IDLE_FAILED;
			}

#if AUTO_START_CALIBRATION
			SystemContext->SensorsIdleFinished = true;
#endif
			break;
		case STATE_CALIBRATION:
			if (BMP581_Mode_Performance(BMP581_HANDLE) != HAL_OK) {
				SystemFaultFlags |= BMP581_MODE_PERFORMANCE_FAILED;
			}
			if (IIM42653_Mode_Performance(IIM42653_HANDLE) != HAL_OK) {
				SystemFaultFlags |= IIM42653_MODE_PERFORMANCE_FAILED;
			}
			if (IIS2MDCTR_Mode_Performance(IIS2MDCTR_HANDLE) != HAL_OK) {
				SystemFaultFlags |= IIS2MDCTR_MODE_PERFORMANCE_FAILED;
			}

			StartSensorTimers();
			break;
		default:
			break;
	}
#endif
}

SystemState_t HandleCommand(SystemState_t CurrentSystemState, CommandType_t CommantType, BaseType_t Received) {
    if (Received != pdPASS) {
        return CurrentSystemState;
    }

    switch (CommantType) {
        case COMMAND_RESET:
            return STATE_IDLE;
        case COMMAND_ABORT:
            return STATE_GROUND_ABORT;
        case COMMAND_CALIBRATION:
            if (CurrentSystemState == STATE_IDLE) {
                return STATE_CALIBRATION;
            }
            return CurrentSystemState;
        default:
            return CurrentSystemState;
    }
}

SystemState_t HandleState(SystemState_t CurrentSystemState, SystemContext_t *SystemContext, FlightData_t SensorData) {
	switch (CurrentSystemState) {
		case STATE_IDLE:
			return IdleStateHandler(SystemContext, SensorData);
			break;
		case STATE_CALIBRATION:
			return CalibrationStateHandler(SystemContext, SensorData);
			break;
		case STATE_PRELAUNCH:
			return PrelaunchStateHandler(SystemContext, SensorData);
			break;
		case STATE_BURN:
			return BurnStateHandler(SystemContext, SensorData);
			break;
		case STATE_PASSIVE_BURNOUT:
			return PassiveBurnoutStateHandler(SystemContext, SensorData);
			break;
		case STATE_ACTIVE_BURNOUT:
			return ActiveBurnoutStateHandler(SystemContext, SensorData);
			break;
		case STATE_APOGEE:
			return ApogeeStateHandler(SystemContext, SensorData);
			break;
		case STATE_PARACHUTE:
			return ParachuteStateHandler(SystemContext, SensorData);
			break;
		case STATE_LANDED:
			return LandedStateHandler(SystemContext, SensorData);
			break;
			// TODO: Refine
		case STATE_GROUND_ABORT:
			return STATE_GROUND_ABORT;
			break;
		case STATE_ASCENT_ABORT:
			return STATE_ASCENT_ABORT;
			break;
		case STATE_DESCENT_ABORT:
			return STATE_DESCENT_ABORT;
			break;
		default:
			// Should not be able to reach
			return STATE_IDLE;
			break;
	}
}
