#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define QUEUE_LENGTH    5

#define PACKET_HEADER      	0xCAFE
#define PACKET_HEADER_LSB  	(PACKET_HEADER & 0xFF)
#define PACKET_HEADER_MSB  	((PACKET_HEADER >> 8) & 0xFF)
#define PACKET_FOOTER		0xBE

#define HIL_MODE                    1
#define EXTERNAL_COMMANDS           1
#define AUTO_START_CALIBRATION		1

#define BUZZER_ENABLED				1
#define PYRO_PULSE_MS				3000

// Altitude configuration
#define ALTITUDE_IIR_FILTER_ALPHA    0.1f

// Barometer Configuration
#define PRESSURE_CALIBRATION_DISCARD_SAMPLES    1000
#define PRESSURE_CALIBRATION_SAMPLES            1000

// IMU Configuration
#define GYRO_CALIBRATION_DISCARD_SAMPLES        1000
#define GYRO_CALIBRATION_SAMPLES                1000

#define IMU_ROTATION_ENABLED              0
#define IMU_ROT_XX  +1.00000000f
#define IMU_ROT_XY  +0.00000000f
#define IMU_ROT_XZ  +0.00000000f
#define IMU_ROT_YX  +0.00000000f
#define IMU_ROT_YY  +1.00000000f
#define IMU_ROT_YZ  +0.00000000f
#define IMU_ROT_ZX  +0.00000000f
#define IMU_ROT_ZY  +0.00000000f
#define IMU_ROT_ZZ  +1.00000000f

// GPS Configuration
#define GPS_FIX_REQUIRED             0
#define GPS_FIX_MIN_SATELLITES       1
#define GPS_ALTITUDE_ASL_BASELINE    90.0f // Baseline to calculate AGL from ASL

// Stack Sizes (words)
#define STACK_SIZE_TELEMETRY            256
#define STACK_SIZE_SENSOR_CONFIG        256
#define STACK_SIZE_STATE_MACHINE        512
#define STACK_SIZE_SD_LOGGING           1024
#define STACK_SIZE_PYRO                 128

// Telemetry Configuration (main loop at 100Hz)
#define TELEMETRY_DIVIDER                   100  // 1Hz in active states
#define TELEMETRY_DIVIDER_IDLE              100  // 1Hz in IDLE

// SD Configuration
#define SD_LOGGING_RECORDS_PER_BUFFER       500

// Flash Configuration
#define FLASH_DUMP_TO_SD                    0
#define FLASH_ERASE_ALL                     0
#define FLASH_LOGGING_DIVIDER               10	// 10Hz with a 10 divider
#define STACK_SIZE_FLASH_LOGGING            512
#define STACK_SIZE_FLASH_MAINTENANCE        1024
#define FLASH_LOGGING_QUEUE_LENGTH          10
#define FLASH_RECORDS_PER_PAGE              8

// Transition Configuration

// Prelaunch to Boost Acceleration Threshold
#define PRELAUNCH_BOOST_ACCEL_Y_THRESHOLD      	-20.0f // IMU Y-axis is inverted, so negative is upwards
#define PRELAUNCH_BOOST_CONSECUTIVE_SAMPLES      5

// Boost to Coast Acceleration Threshold
#define BOOST_COAST_ACCEL_Y_THRESHOLD          -5.0f // IMU Y-axis is inverted, so negative is upwards
#define BOOST_COAST_CONSECUTIVE_SAMPLES        5

// Coast to Active Control Altitude Threshold
#define COAST_ACTIVE_CONTROL_BAROM_ALT_THRESHOLD    2000.0f // Barometric altitude threshold for active control
#define COAST_ACTIVE_CONTROL_CONSECUTIVE_SAMPLES    5

// Active Control to Apogee Barometric Altitude + GPS Altitude + GPS Vertical Velocity
#define ACTIVE_CONTROL_APOGEE_BAROM_ALT_ENABLED			0
#define ACTIVE_CONTROL_APOGEE_BAROM_ALT_THRESHOLD		2900.0f
#define ACTIVE_CONTROL_APOGEE_BAROM_VEL_ENABLED			1
#define ACTIVE_CONTROL_APOGEE_BAROM_VEL_THRESHOLD		0.0f
#define ACTIVE_CONTROL_APOGEE_GPS_ENABLED				0
#define ACTIVE_CONTROL_APOGEE_GPS_ALT_THRESHOLD			2900.0f
#define ACTIVE_CONTROL_APOGEE_GPS_VEL_Y_THRESHOLD		0.0f
#define ACTIVE_CONTROL_APOGEE_DELAY_ENABLED				0
#define ACTIVE_CONTROL_APOGEE_DELAY_MS					10000

// Apogee to Main Parachute
#define APOGEE_MAIN_PARACHUTE_BAROM_ALT_THRESHOLD 	3000.0f // WARN: AGL
#define APOGEE_MAIN_PARACHUTE_GPS_ALT_ENABLED		0
#define APOGEE_MAIN_PARACHUTE_GPS_ALT_THRESHOLD		450.0f // WARN: ASL
#define APOGEE_MAIN_PARACHUTE_DELAY_ENABLED			0
#define APOGEE_MAIN_PARACHUTE_DELAY_MS				30000

// Main Parachute to Landed
#define MAIN_PARACHUTE_LANDED_BAROM_ALT_THRESHOLD		100.0f
#define MAIN_PARACHUTE_LANDED_BAROM_VEL_Y_THRESHOLD		2.0f
#define MAIN_PARACHUTE_LANDED_CONSECUTIVE_SAMPLES		100

#define LANDED_SD_STOP_DELAY_ENABLED            1
#define LANDED_SD_STOP_DELAY_MS                 5000
#define LANDED_FLASH_STOP_DELAY_ENABLED         1
#define LANDED_FLASH_STOP_DELAY_MS              5000

#endif //CONFIGURATION_H
