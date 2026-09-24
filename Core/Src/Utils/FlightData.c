#include "Utils/shared.h"
#include "Utils/FlightData.h"
#include "Sensors/Sensors.h"
#include "Protocol/Protocol.h"
#include "Utils/Battery.h"
#include "Utils/Calculations.h"
#include "Utils/Pyro.h"
#include "KalmanLib.h"

#define MM_TO_METERS 0.001f
#define KALMAN_DT 0.01f

static arm_matrix_instance_f32 kalman_P, kalman_Q, kalman_R_GPS, kalman_R_MAG, kalman_R_BAR;
static float32_t kalman_P_data[H_P_A_Q_COLS * H_P_A_Q_COLS];
static float32_t kalman_Q_data[H_P_A_Q_COLS * H_P_A_Q_COLS];
static float32_t kalman_R_GPS_data[H_GPS_ROWS * H_GPS_ROWS] = {0};
static float32_t kalman_R_MAG_data[H_MAG_ROWS * H_MAG_ROWS];
static float32_t kalman_R_BAR_data[H_BAR_ROWS * H_BAR_ROWS];

static float32_t kalman_pos[3] = {0};
static float32_t kalman_vel[3] = {0};
static float32_t kalman_quat[4] = {0};

static float32_t kalman_B_e[3] = {0};
static float32_t kalman_z_GPS[6] = {0};
static float32_t kalman_z_MAG[3] = {0};

static float kalman_last_pressure = 0.0f;

void KalmanFilter_Init(SystemContext_t *SystemContext) {
	float32_t eul_deg_0[3] = {0.0f, 90.0f, 0.0f};

	for (int i = 0; i < H_GPS_ROWS; i++) {
		kalman_R_GPS_data[i + i * H_GPS_ROWS] = 1.0f;
	}
	arm_mat_init_f32(&kalman_R_GPS, H_GPS_ROWS, H_GPS_ROWS, kalman_R_GPS_data);

	kalman_init(&kalman_P, &kalman_Q, &kalman_R_GPS, &kalman_R_MAG, &kalman_R_BAR,
		&kalman_P_data, &kalman_Q_data, &kalman_R_MAG_data, &kalman_R_BAR_data,
		&eul_deg_0, &kalman_quat, KALMAN_DT);
}
#define KALMAN_DT 0.01f

static arm_matrix_instance_f32 kalman_P, kalman_Q, kalman_R_GPS, kalman_R_MAG, kalman_R_BAR;
static float32_t kalman_P_data[H_P_A_Q_COLS * H_P_A_Q_COLS];
static float32_t kalman_Q_data[H_P_A_Q_COLS * H_P_A_Q_COLS];
static float32_t kalman_R_GPS_data[H_GPS_ROWS * H_GPS_ROWS] = {0};
static float32_t kalman_R_MAG_data[H_MAG_ROWS * H_MAG_ROWS];
static float32_t kalman_R_BAR_data[H_BAR_ROWS * H_BAR_ROWS];

static float32_t kalman_pos[3] = {0};
static float32_t kalman_vel[3] = {0};
static float32_t kalman_quat[4] = {0};

static float32_t kalman_B_e[3] = {0};
static float32_t kalman_z_GPS[6] = {0};
static float32_t kalman_z_MAG[3] = {0};

static float kalman_last_pressure = 0.0f;

void KalmanFilter_Init(SystemContext_t *SystemContext) {
	float32_t eul_deg_0[3] = {0.0f, 90.0f, 0.0f};

	for (int i = 0; i < H_GPS_ROWS; i++) {
		kalman_R_GPS_data[i + i * H_GPS_ROWS] = 1.0f;
	}
	arm_mat_init_f32(&kalman_R_GPS, H_GPS_ROWS, H_GPS_ROWS, kalman_R_GPS_data);

	kalman_init(&kalman_P, &kalman_Q, &kalman_R_GPS, &kalman_R_MAG, &kalman_R_BAR,
		&kalman_P_data, &kalman_Q_data, &kalman_R_MAG_data, &kalman_R_BAR_data,
		&eul_deg_0, &kalman_quat, KALMAN_DT);
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
	FlightData.BarometricAltitude = CalculateFilteredAltitude(SystemContext, FlightData.BarometricAltitude);
	FlightData.BarometricVelocity = CalculateBarometricVerticalVelocity(FlightData.BarometricAltitude, FlightData.Tick);
	FlightData.GPSVelocity = CalculateGPSVerticalVelocity(FlightData.GPSAltitude, FlightData.Tick);

	if (SystemContext->KalmanInitialized) {
		bool BAR_available = (FlightData.PressurePa != kalman_last_pressure);
		kalman_last_pressure = FlightData.PressurePa;

		float32_t accel_IMU[3] = {FlightData.AccelX, FlightData.AccelY, FlightData.AccelZ};
		float32_t omega_IMU[3] = {FlightData.GyroX, FlightData.GyroY, FlightData.GyroZ};
		float32_t z_BAR[1] = {FlightData.PressurePa};

		kalman_filter(true, false, false, BAR_available,
			&accel_IMU, &omega_IMU, &kalman_z_GPS, &kalman_z_MAG, &z_BAR,
			&kalman_pos, &kalman_vel, &kalman_quat,
			&kalman_B_e, &kalman_P, &kalman_Q, &kalman_R_GPS, &kalman_R_MAG, &kalman_R_BAR,
			SystemContext->ReferencePressurePa, KALMAN_DT);

		FlightData.VelX = kalman_vel[0];
		FlightData.VelY = kalman_vel[1];
		FlightData.VelZ = kalman_vel[2];
	} else {
		FlightData.VelX = 0;
		FlightData.VelY = 0;
		FlightData.VelZ = 0;
	}
	if (SystemContext->KalmanInitialized) {
		bool BAR_available = (FlightData.PressurePa != kalman_last_pressure);
		kalman_last_pressure = FlightData.PressurePa;

		float32_t accel_IMU[3] = {FlightData.AccelX, FlightData.AccelY, FlightData.AccelZ};
		float32_t omega_IMU[3] = {FlightData.GyroX, FlightData.GyroY, FlightData.GyroZ};
		float32_t z_BAR[1] = {FlightData.PressurePa};

		kalman_filter(true, false, false, BAR_available,
			&accel_IMU, &omega_IMU, &kalman_z_GPS, &kalman_z_MAG, &z_BAR,
			&kalman_pos, &kalman_vel, &kalman_quat,
			&kalman_B_e, &kalman_P, &kalman_Q, &kalman_R_GPS, &kalman_R_MAG, &kalman_R_BAR,
			SystemContext->ReferencePressurePa, KALMAN_DT);

		FlightData.VelX = kalman_vel[0];
		FlightData.VelY = kalman_vel[1];
		FlightData.VelZ = kalman_vel[2];
	} else {
		FlightData.VelX = 0;
		FlightData.VelY = 0;
		FlightData.VelZ = 0;
	}

	FlightData.Flags = SystemFaultFlags;
	FlightData.BatteryVoltage = BatteryGetVoltage();
	FlightData.State = SystemState;
	FlightData.RelayState = PyroGetState();
	FlightData.LastCommand = (LastCommand < COMMAND_HIL_DATA) ? LastCommand : COMMAND_NONE;
	FlightData.SyncEnd = PACKET_FOOTER;

	return FlightData;
}
