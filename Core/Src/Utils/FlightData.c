#include "Utils/shared.h"
#include "Utils/FlightData.h"
#include "Sensors/Sensors.h"
#include "Protocol/Protocol.h"
#include "Utils/Battery.h"
#include "Utils/Calculations.h"
#include "Utils/Pyro.h"

#define MM_TO_METERS 0.001f

FlightData_t GetFlightData(SystemState_t SystemState, SystemContext_t *SystemContext, IIM42653_SensorData_t IIM42653_FlightData, BMP581_SensorData_t BMP581_FlightData, uint32_t BaroSequence, IIS2MDCTR_SensorData_t IIS2MDCTR_FlightData, ZOEM8Q_SensorData_t ZOEM8Q_FlightData, CommandType_t LastCommand) {
	FlightData_t FlightData;

	FlightData.Sync = PACKET_HEADER;
	FlightData.Tick = xTaskGetTickCount();

	FlightData.PressurePa = BMP581_FlightData.PressurePa;
	FlightData.TemperatureC = BMP581_FlightData.TemperatureC;

	FlightData.MagX = IIS2MDCTR_FlightData.MagX;
	FlightData.MagY = IIS2MDCTR_FlightData.MagY;
	FlightData.MagZ = IIS2MDCTR_FlightData.MagZ;

	ApplyIMURotation(
		IIM42653_FlightData.AccelX, IIM42653_FlightData.AccelY, IIM42653_FlightData.AccelZ,
		&FlightData.AccelX, &FlightData.AccelY, &FlightData.AccelZ
	);

	ApplyIMURotation(
		CalculateBiasedGyroscope(SystemContext, IIM42653_FlightData.GyroX, SystemContext->GyroBiasX),
		CalculateBiasedGyroscope(SystemContext, IIM42653_FlightData.GyroY, SystemContext->GyroBiasY),
		CalculateBiasedGyroscope(SystemContext, IIM42653_FlightData.GyroZ, SystemContext->GyroBiasZ),
		&FlightData.GyroX, &FlightData.GyroY, &FlightData.GyroZ
	);

	FlightData.Latitude = ZOEM8Q_FlightData.Latitude;
	FlightData.Longitude = ZOEM8Q_FlightData.Longitude;
	FlightData.GPSAltitude = ZOEM8Q_FlightData.AltitudeMm * MM_TO_METERS;
	FlightData.UnixTime = ZOEM8Q_FlightData.UnixTime;
	FlightData.Milliseconds = ZOEM8Q_FlightData.Milliseconds;
	FlightData.Satellites = ZOEM8Q_FlightData.Satellites;

	FlightData.BarometricAltitude = CalculateAltitude(SystemContext, FlightData.PressurePa, FlightData.TemperatureC);
	FlightData.BarometricAltitude = CalculateFilteredAltitude(SystemContext, FlightData.BarometricAltitude, BaroSequence);
	FlightData.BarometricVelocity = CalculateBarometricVerticalVelocity(FlightData.BarometricAltitude, FlightData.Tick, BaroSequence);
	FlightData.GPSVelocity = CalculateGPSVerticalVelocity(FlightData.GPSAltitude, FlightData.Tick);

	if (SystemContext->AccelCalibrationValid) {
		const float *M = SystemContext->AccelM;
		const float *b = SystemContext->AccelBias;
		FlightData.BodyAccelX = M[0]*FlightData.AccelX + M[1]*FlightData.AccelY + M[2]*FlightData.AccelZ + b[0];
		FlightData.BodyAccelY = M[3]*FlightData.AccelX + M[4]*FlightData.AccelY + M[5]*FlightData.AccelZ + b[1];
		FlightData.BodyAccelZ = M[6]*FlightData.AccelX + M[7]*FlightData.AccelY + M[8]*FlightData.AccelZ + b[2];

		const float *A = SystemContext->AccelA_m;
		FlightData.BodyGyroX = A[0]*FlightData.GyroX + A[1]*FlightData.GyroY + A[2]*FlightData.GyroZ;
		FlightData.BodyGyroY = A[3]*FlightData.GyroX + A[4]*FlightData.GyroY + A[5]*FlightData.GyroZ;
		FlightData.BodyGyroZ = A[6]*FlightData.GyroX + A[7]*FlightData.GyroY + A[8]*FlightData.GyroZ;
	} else {
		FlightData.BodyAccelX = FlightData.AccelX;
		FlightData.BodyAccelY = FlightData.AccelY;
		FlightData.BodyAccelZ = FlightData.AccelZ;
		FlightData.BodyGyroX = FlightData.GyroX;
		FlightData.BodyGyroY = FlightData.GyroY;
		FlightData.BodyGyroZ = FlightData.GyroZ;
	}

	FlightData.VelX = 0;
	FlightData.VelY = 0;
	FlightData.VelZ = 0;

	FlightData.Flags = SystemFaultFlags
		| ((uint32_t)SystemContext->DeepCalFacesCaptured << 16)
		| ((uint32_t)(SystemContext->DeepCalCurrentFace + 1) << 22);
	FlightData.BatteryVoltage = BatteryGetVoltage();
	FlightData.State = SystemState;
	FlightData.RelayState = PyroGetState();
	FlightData.LastCommand = (LastCommand < COMMAND_HIL_DATA) ? LastCommand : COMMAND_NONE;
	FlightData.SyncEnd = PACKET_FOOTER;

	return FlightData;
}
