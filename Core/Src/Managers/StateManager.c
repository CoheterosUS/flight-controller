#include "States/StateHandlers.h"
#include "Sensors/Sensors.h"
#include "Managers/Managers.h"
#include "Utils/shared.h"
#include "Utils/Pyro.h"
#include "Utils/DeepCalGesture.h"
#include "Utils/ApogeeDetector.h"
#include "timers.h"
#include <Tasks/SensorConfigTask.h>

static ApogeeDetector_t Detector;

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
    uint32_t Now = xTaskGetTickCount();
    SystemContext->StateEntryTick = Now;
    SystemContext->StateEntryTicks[CurrentSystemState] = Now;
    if (CurrentSystemState == STATE_IDLE) {
        SystemContext->SensorsIdleFinished = false;
    }
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
        case STATE_BOOST:
            BoostStateEntry(SystemContext);
            SystemContext->ApogeeTrigger = APOGEE_TRIGGER_NONE;
            ApogeeDetector_Reset(&Detector, Now);
            break;
        case STATE_COAST:
            CoastStateEntry(SystemContext);
            break;
        case STATE_ACTIVE_CONTROL:
            ActiveControlStateEntry(SystemContext);
            break;
        case STATE_APOGEE:
            ApogeeStateEntry(SystemContext);
            break;
        case STATE_MAIN_PARACHUTE:
            MainParachuteStateEntry(SystemContext);
            break;
        case STATE_LANDED:
            LandedStateEntry(SystemContext);
            break;
        case STATE_GROUND_ABORT:
            GroundAbortStateEntry(SystemContext);
            break;
        case STATE_DESCENT_ABORT:
            DescentAbortStateEntry(SystemContext);
            break;
        case STATE_ASCENT_ABORT:
            AscentAbortStateEntry(SystemContext);
            break;
        case STATE_DEEP_CALIBRATION:
            DeepCalibrationStateEntry(SystemContext);
            break;
        default:
            break;
    }
}

void HandleSensors(SystemContext_t *SystemContext, SystemState_t CurrentSystemState) {
#if HIL_MODE
	switch (CurrentSystemState) {
		case STATE_IDLE:
#if AUTO_START_CALIBRATION
			SystemContext->SensorsIdleFinished = true;
#endif
			break;
		default:
			break;
	}
#else
	switch (CurrentSystemState) {
		case STATE_IDLE:
			if (BMP581_Mode_Idle(BMP581_HANDLE) != HAL_OK) {
				SystemFaultSet(BMP581_MODE_IDLE_FAILED);
			}
			if (IIM42653_Mode_Idle(IIM42653_HANDLE) != HAL_OK) {
				SystemFaultSet(IIM42653_MODE_IDLE_FAILED);
			}
			if (IIS2MDCTR_Mode_Idle(IIS2MDCTR_HANDLE) != HAL_OK) {
				SystemFaultSet(IIS2MDCTR_MODE_IDLE_FAILED);
			}

#if AUTO_START_CALIBRATION
			SystemContext->SensorsIdleFinished = true;
#endif
			break;
		case STATE_CALIBRATION:
		case STATE_DEEP_CALIBRATION:
			if (BMP581_Mode_Performance(BMP581_HANDLE) != HAL_OK) {
				SystemFaultSet(BMP581_MODE_PERFORMANCE_FAILED);
			}
			if (IIM42653_Mode_Performance(IIM42653_HANDLE) != HAL_OK) {
				SystemFaultSet(IIM42653_MODE_PERFORMANCE_FAILED);
			}
			if (IIS2MDCTR_Mode_Performance(IIS2MDCTR_HANDLE) != HAL_OK) {
				SystemFaultSet(IIS2MDCTR_MODE_PERFORMANCE_FAILED);
			}

			StartSensorTimers();
			break;
		case STATE_GROUND_ABORT:
		case STATE_DESCENT_ABORT:
		case STATE_ASCENT_ABORT:
		case STATE_LANDED:
			StopSensorTimers();
			if (BMP581_Mode_Idle(BMP581_HANDLE) != HAL_OK) {
				SystemFaultSet(BMP581_MODE_IDLE_FAILED);
			}
			if (IIM42653_Mode_Idle(IIM42653_HANDLE) != HAL_OK) {
				SystemFaultSet(IIM42653_MODE_IDLE_FAILED);
			}
			if (IIS2MDCTR_Mode_Idle(IIS2MDCTR_HANDLE) != HAL_OK) {
				SystemFaultSet(IIS2MDCTR_MODE_IDLE_FAILED);
			}
			break;
		default:
			break;
	}
#endif
}

SystemState_t HandleCommand(SystemState_t CurrentSystemState, SystemContext_t *SystemContext, CommandType_t CommantType, BaseType_t Received) {
    if (Received != pdPASS) {
        return CurrentSystemState;
    }

    if (CurrentSystemState == STATE_DEEP_CALIBRATION
        && (CommantType == COMMAND_DROGUE
            || CommantType == COMMAND_LANDED
            || CommantType == COMMAND_CALIBRATION)) {
        return CurrentSystemState;
    }

    switch (CommantType) {
        case COMMAND_RESET:
            return STATE_IDLE;
        case COMMAND_GROUND_ABORT:
            return STATE_GROUND_ABORT;
        case COMMAND_DROGUE:
#if !DROGUE_COMMAND_ANY_STATE
            if (CurrentSystemState != STATE_BOOST && CurrentSystemState != STATE_COAST &&
                CurrentSystemState != STATE_ACTIVE_CONTROL) {
                return CurrentSystemState;
            }
#endif
            SystemContext->ApogeeTrigger = APOGEE_TRIGGER_COMMAND;
            return STATE_APOGEE;
        case COMMAND_LANDED:
            return STATE_LANDED;
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
	static DeepCalGesture_t DeepCalibrationGesture;
	SystemState_t NextState;
	float RawAccel[3] = {SensorData.RawAccelX, SensorData.RawAccelY, SensorData.RawAccelZ};
	float RawGyro[3] = {SensorData.RawGyroX, SensorData.RawGyroY, SensorData.RawGyroZ};
	if (CurrentSystemState == STATE_BOOST || CurrentSystemState == STATE_COAST || CurrentSystemState == STATE_ACTIVE_CONTROL) {
		ApogeeInput_t Input = {
			.NowMs = xTaskGetTickCount(),
			.BaroAllowed = CurrentSystemState != STATE_BOOST,
			.BaroValid = SensorData.BaroValid,
			.AltitudeM = SensorData.BarometricAltitude,
			.BaroSampleId = SensorData.BaroSampleId
		};
		ApogeeTrigger_t Trigger = ApogeeDetector_Update(&Detector, &Input);
		if (Trigger != APOGEE_TRIGGER_NONE) {
			SystemContext->ApogeeTrigger = Trigger;
			return STATE_APOGEE;
		}
	}

	switch (CurrentSystemState) {
		case STATE_IDLE:
			NextState = IdleStateHandler(SystemContext, SensorData);
			break;
		case STATE_CALIBRATION:
			NextState = CalibrationStateHandler(SystemContext, SensorData);
			break;
		case STATE_PRELAUNCH:
			NextState = PrelaunchStateHandler(SystemContext, SensorData);
			break;
		case STATE_BOOST:
			NextState = BoostStateHandler(SystemContext, SensorData);
			break;
		case STATE_COAST:
			NextState = CoastStateHandler(SystemContext, SensorData);
			break;
		case STATE_ACTIVE_CONTROL:
			NextState = ActiveControlStateHandler(SystemContext, SensorData);
			break;
		case STATE_APOGEE:
			NextState = ApogeeStateHandler(SystemContext, SensorData);
			break;
		case STATE_MAIN_PARACHUTE:
			NextState = MainParachuteStateHandler(SystemContext, SensorData);
			break;
		case STATE_LANDED:
			NextState = LandedStateHandler(SystemContext, SensorData);
			break;
		case STATE_GROUND_ABORT:
			NextState = GroundAbortStateHandler(SystemContext, SensorData);
			break;
		case STATE_DESCENT_ABORT:
			NextState = DescentAbortStateHandler(SystemContext, SensorData);
			break;
		case STATE_ASCENT_ABORT:
			NextState = AscentAbortStateHandler(SystemContext, SensorData);
			break;
		case STATE_DEEP_CALIBRATION:
			NextState = DeepCalibrationStateHandler(SystemContext, SensorData);
			break;
		default:
			// Should not be able to reach
			NextState = STATE_IDLE;
			break;
	}

	if (CurrentSystemState == STATE_IDLE || CurrentSystemState == STATE_CALIBRATION
		|| CurrentSystemState == STATE_PRELAUNCH) {
		if (NextState == CurrentSystemState
			&& DeepCalGesture_Update(&DeepCalibrationGesture, RawAccel, RawGyro, SensorData.Tick)) {
			return STATE_DEEP_CALIBRATION;
		}
	} else {
		DeepCalGesture_Reset(&DeepCalibrationGesture);
	}

	return NextState;
}
