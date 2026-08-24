#include "Utils/shared.h"
#include "Utils/FlightData.h"
#include "Sensors/Sensors.h"
#include "Protocol/Protocol.h"
#include "Utils/Battery.h"
#include "Utils/Calculations.h"
#include "Utils/Pyro.h"

#define MM_TO_METERS 0.001f

FlightData_t GetFlightData(SystemState_t SystemState, SystemContext_t *SystemContext, IIM42653_SensorData_t IIM42653_FlightData, BMP581_SensorData_t BMP581_FlightData, IIS2MDCTR_SensorData_t IIS2MDCTR_FlightData, ZOEM8Q_SensorData_t ZOEM8Q_FlightData, CommandType_t LastCommand) {
	FlightData_t FlightData;

	FlightData.Sync = PACKET_HEADER;
	FlightData.Tick = xTaskGetTickCount();

	FlightData.PressurePa = BMP581_FlightData.PressurePa;
	FlightData.TemperatureC = BMP581_FlightData.TemperatureC;

	FlightData.MagX = IIS2MDCTR_FlightData.MagX;
	FlightData.MagY = IIS2MDCTR_FlightData.MagY;
	FlightData.MagZ = IIS2MDCTR_FlightData.MagZ;

	FlightData.GyroX = CalculateBiasedGyroscope(SystemContext, IIM42653_FlightData.GyroX, SystemContext->GyroBiasX);
	FlightData.GyroY = CalculateBiasedGyroscope(SystemContext, IIM42653_FlightData.GyroY, SystemContext->GyroBiasY);
	FlightData.GyroZ = CalculateBiasedGyroscope(SystemContext, IIM42653_FlightData.GyroZ, SystemContext->GyroBiasZ);

	FlightData.AccelX = IIM42653_FlightData.AccelX;
	FlightData.AccelY = IIM42653_FlightData.AccelY;
	FlightData.AccelZ = IIM42653_FlightData.AccelZ;

	FlightData.Latitude = ZOEM8Q_FlightData.Latitude;
	FlightData.Longitude = ZOEM8Q_FlightData.Longitude;
	FlightData.GPSAltitude = ZOEM8Q_FlightData.AltitudeMm * MM_TO_METERS;
	FlightData.UnixTime = ZOEM8Q_FlightData.UnixTime;
	FlightData.Milliseconds = ZOEM8Q_FlightData.Milliseconds;
	FlightData.Satellites = ZOEM8Q_FlightData.Satellites;

	FlightData.BarometricAltitude = CalculateAltitude(SystemContext, FlightData.PressurePa, FlightData.TemperatureC);
	FlightData.BarometricAltitude = CalculateFilteredAltitude(SystemContext, FlightData.BarometricAltitude);
	FlightData.BarometricVelocity = CalculateVerticalVelocity(FlightData.BarometricAltitude, FlightData.Tick);

	FlightData.VelX = 0;
	FlightData.VelY = 0;
	FlightData.VelZ = 0;

	FlightData.Flags = SystemFaultFlags;
	FlightData.BatteryVoltage = BatteryGetVoltage();
	FlightData.State = SystemState;
	FlightData.RelayState = PyroGetState();
	FlightData.LastCommand = (LastCommand < COMMAND_HIL_DATA) ? LastCommand : COMMAND_NONE;
	FlightData.SyncEnd = PACKET_FOOTER;

	return FlightData;
}
