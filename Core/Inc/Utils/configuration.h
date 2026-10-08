#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define DEFAULT_QUEUE_LENGTH                5
#define FLASH_LOGGING_QUEUE_LENGTH          10

#define PACKET_HEADER      	0xCAFE
#define PACKET_HEADER_LSB  	(PACKET_HEADER & 0xFF)
#define PACKET_HEADER_MSB  	((PACKET_HEADER >> 8) & 0xFF)
#define PACKET_FOOTER		0xBE

#define HIL_MODE                    0
#define EXTERNAL_COMMANDS           1
#define AUTO_START_CALIBRATION		1

#define BUZZER_ENABLED				1

// Pyro Configuration
#define PYRO_PULSE_MS       3000
#define PYROS_ENABLED       0

// Altitude configuration
#define ALTITUDE_IIR_FILTER_ALPHA    0.1f

// Barometer Configuration
#define PRESSURE_CALIBRATION_DISCARD_SAMPLES    1000
#define PRESSURE_CALIBRATION_SAMPLES            1000
#define TEMPERATURE_CALIBRATION_DISCARD_SAMPLES 1000
#define TEMPERATURE_CALIBRATION_SAMPLES         1000

// IMU Configuration
#define GYRO_CALIBRATION_DISCARD_SAMPLES        1000
#define GYRO_CALIBRATION_SAMPLES                1000


// GPS Configuration
#define GPS_FIX_REQUIRED             1
#define GPS_FIX_MIN_SATELLITES       4
#define GPS_ALTITUDE_ASL_BASELINE    90.0f // Baseline to calculate AGL from ASL

// Stack Sizes (words)
#define STACK_SIZE_TELEMETRY            512
#define STACK_SIZE_SENSOR_CONFIG        512
#define STACK_SIZE_STATE_MACHINE        1024
#define STACK_SIZE_SD_LOGGING           1024
#define STACK_SIZE_BUZZER               128
#define STACK_SIZE_PYRO                 256
#define STACK_SIZE_FLASH_LOGGING        768
#define STACK_SIZE_FLASH_MAINTENANCE    1024

// Telemetry Configuration (main loop at 100Hz)

// SD Configuration
#define SD_LOGGING_ENABLED          		1
#define SD_LOGGING_RECORDS_PER_BUFFER       500

// Flash Configuration
#define FLASH_DUMP_TO_SD                    0
#define FLASH_ERASE_ALL                     0
#define FLASH_LOGGING_DIVIDER               10	// 10Hz with a 10 divider
#define FLASH_RECORDS_PER_PAGE              8

// Transition Configuration

// Prelaunch to Boost Acceleration Threshold
#define PRELAUNCH_BOOST_ACCEL_Y_THRESHOLD      	-40.0f // IMU Y-axis is inverted, so negative is upwards
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
#define ACTIVE_CONTROL_APOGEE_BAROM_VEL_THRESHOLD		-10.0f
#define ACTIVE_CONTROL_APOGEE_GPS_ENABLED				0
#define ACTIVE_CONTROL_APOGEE_GPS_ALT_THRESHOLD			2900.0f
#define ACTIVE_CONTROL_APOGEE_GPS_VEL_Y_THRESHOLD		0.0f
#define ACTIVE_CONTROL_APOGEE_DELAY_ENABLED				0
#define ACTIVE_CONTROL_APOGEE_DELAY_MS					10000

// Apogee to Main Parachute
#define APOGEE_MAIN_PARACHUTE_BAROM_ALT_THRESHOLD 	450.0f // WARN: AGL
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

// DEEP CALIBRATION
#define DEEP_CALIBRATION_ENABLED                1
#define DEEP_CALIBRATION_DURATION_MS            10000
#define DEEP_CALIBRATION_TIMEOUT_MS             180000

#define DEEP_CALIBRATION_ACCEL_THRESHOLD        8.0f
#define DEEP_CALIBRATION_CONFIRM_SAMPLES        100
#define DEEP_CALIBRATION_GYRO_MAX_DPS           7.5f
#define DEEP_CALIBRATION_DISCARD_SAMPLES        1000
#define DEEP_CALIBRATION_SAMPLES                500

#define DEEP_CALIBRATION_FACE_COUNT             6
#define DEEP_CALIBRATION_ALL_FACES              ((1u << DEEP_CALIBRATION_FACE_COUNT) - 1u)
#define DEEP_CALIBRATION_AXES                   3

#endif //CONFIGURATION_H
