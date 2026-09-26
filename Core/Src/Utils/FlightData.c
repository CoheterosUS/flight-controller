#include "Utils/shared.h"
#include "Utils/FlightData.h"
#include "Sensors/Sensors.h"
#include "Protocol/Protocol.h"
#include "Utils/Battery.h"
#include "Utils/Calculations.h"
#include "Utils/Pyro.h"
#include "KalmanLib.h"

#define MM_TO_METERS 0.001f
#define KALMAN_DT LOOP_DT

static arm_matrix_instance_f32 KalmanP, KalmanQ, KalmanRGPS, KalmanRMAG, KalmanRBAR;
static float32_t KalmanPData[H_P_A_Q_COLS * H_P_A_Q_COLS];
static float32_t KalmanQData[H_P_A_Q_COLS * H_P_A_Q_COLS];
static float32_t KalmanRGPSData[H_GPS_ROWS * H_GPS_ROWS] = {0};
static float32_t KalmanRMAGData[H_MAG_ROWS * H_MAG_ROWS];
static float32_t KalmanRBARData[H_BAR_ROWS * H_BAR_ROWS];

static float32_t KalmanPos[3] = {0};
static float32_t KalmanVel[3] = {0};
static float32_t KalmanQuat[4] = {0};

static float32_t KalmanBe[3] = {0};
static float32_t KalmanZGPS[6] = {0};
static float32_t KalmanZMAG[3] = {0};

static float KalmanLastPressure = 0.0f;
static float KalmanLastAccelX = 0.0f;

void KalmanFilter_Init(SystemContext_t *SystemContext) {
	float32_t EulDeg0[3] = {KALMAN_INITIAL_ROLL_DEG, KALMAN_INITIAL_PITCH_DEG, KALMAN_INITIAL_YAW_DEG};

	for (int i = 0; i < H_GPS_ROWS; i++) {
		KalmanRGPSData[i + i * H_GPS_ROWS] = 1.0f;
	}
	arm_mat_init_f32(&KalmanRGPS, H_GPS_ROWS, H_GPS_ROWS, KalmanRGPSData);

	kalman_init(&KalmanP, &KalmanQ, &KalmanRGPS, &KalmanRMAG, &KalmanRBAR,
		&KalmanPData, &KalmanQData, &KalmanRMAGData, &KalmanRBARData,
		&EulDeg0, &KalmanQuat, KALMAN_DT);
}

void KalmanFilter_IMU_Cal(float32_t *accelIn, float32_t *gyroIn, float32_t *accelOut, float32_t *gyroOut) {
	accelOut[0] = accelIn[0];
	accelOut[1] = accelIn[1];
	accelOut[2] = accelIn[2];
	gyroOut[0] = gyroIn[0];
	gyroOut[1] = gyroIn[1];
	gyroOut[2] = gyroIn[2];
}

FlightData_t GetFlightData(SystemState_t SystemState, SystemContext_t *SystemContext, IIM42653_SensorData_t IIM42653_FlightData, BMP581_SensorData_t BMP581_FlightData, IIS2MDCTR_SensorData_t IIS2MDCTR_FlightData, ZOEM8Q_SensorData_t ZOEM8Q_FlightData, CommandType_t LastCommand) {
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
	// FlightData.BarometricAltitude = CalculateFilteredAltitude(SystemContext, FlightData.BarometricAltitude);
	FlightData.BarometricVelocity = CalculateBarometricVerticalVelocity(FlightData.BarometricAltitude, FlightData.Tick);
	FlightData.GPSVelocity = CalculateGPSVerticalVelocity(FlightData.GPSAltitude, FlightData.Tick);

	if (SystemContext->KalmanInitialized) {
		bool IMU_Available = (FlightData.AccelX != KalmanLastAccelX);
		bool BAR_Available = (FlightData.PressurePa != KalmanLastPressure);
		KalmanLastAccelX = FlightData.AccelX;
		KalmanLastPressure = FlightData.PressurePa;

		float32_t AccelRaw[3] = {FlightData.AccelX, FlightData.AccelY, FlightData.AccelZ};
		float32_t GyroRaw[3] = {FlightData.GyroX, FlightData.GyroY, FlightData.GyroZ};
		float32_t AccelIMU[3];
		float32_t OmegaIMU[3];
		float32_t ZBAR[1] = {FlightData.PressurePa};

		KalmanFilter_IMU_Cal(AccelRaw, GyroRaw, AccelIMU, OmegaIMU);

		FlightData.CalAccelX = AccelIMU[0];
		FlightData.CalAccelY = AccelIMU[1];
		FlightData.CalAccelZ = AccelIMU[2];
		FlightData.CalGyroX = OmegaIMU[0];
		FlightData.CalGyroY = OmegaIMU[1];
		FlightData.CalGyroZ = OmegaIMU[2];

		kalman_filter(IMU_Available, false, false, BAR_Available,
			&AccelIMU, &OmegaIMU, &KalmanZGPS, &KalmanZMAG, &ZBAR,
			&KalmanPos, &KalmanVel, &KalmanQuat,
			&KalmanBe, &KalmanP, &KalmanQ, &KalmanRGPS, &KalmanRMAG, &KalmanRBAR,
			SystemContext->ReferencePressurePa, CalculateKelvinFromCelsius(SystemContext->ReferenceTemperatureC), KALMAN_DT
    );

		FlightData.PosX = KalmanPos[0];
		FlightData.PosY = KalmanPos[1];
		FlightData.PosZ = KalmanPos[2];
		FlightData.VelX = KalmanVel[0];
		FlightData.VelY = KalmanVel[1];
		FlightData.VelZ = KalmanVel[2];
		FlightData.QuatW = KalmanQuat[0];
		FlightData.QuatX = KalmanQuat[1];
		FlightData.QuatY = KalmanQuat[2];
		FlightData.QuatZ = KalmanQuat[3];
		for (int i = 0; i < 9; i++) {
			FlightData.PDiag[i] = KalmanPData[i + i * H_P_A_Q_COLS];
		}
	} else {
		FlightData.PosX = 0;
		FlightData.PosY = 0;
		FlightData.PosZ = 0;
		FlightData.VelX = 0;
		FlightData.VelY = 0;
		FlightData.VelZ = 0;
		FlightData.QuatW = 0;
		FlightData.QuatX = 0;
		FlightData.QuatY = 0;
		FlightData.QuatZ = 0;
		memset(FlightData.PDiag, 0, sizeof(FlightData.PDiag));
		FlightData.CalAccelX = FlightData.AccelX;
		FlightData.CalAccelY = FlightData.AccelY;
		FlightData.CalAccelZ = FlightData.AccelZ;
		FlightData.CalGyroX = FlightData.GyroX;
		FlightData.CalGyroY = FlightData.GyroY;
		FlightData.CalGyroZ = FlightData.GyroZ;
	}

	FlightData.Flags = SystemFaultFlags;
	FlightData.BatteryVoltage = BatteryGetVoltage();
	FlightData.State = SystemState;
	FlightData.RelayState = PyroGetState();
	FlightData.LastCommand = (LastCommand < COMMAND_HIL_DATA) ? LastCommand : COMMAND_NONE;
	FlightData.SyncEnd = PACKET_FOOTER;

	return FlightData;
}
