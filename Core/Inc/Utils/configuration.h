#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define QUEUE_LENGTH    5

#define PACKET_HEADER      	0xCAFE
#define PACKET_HEADER_LSB  	(PACKET_HEADER & 0xFF)
#define PACKET_HEADER_MSB  	((PACKET_HEADER >> 8) & 0xFF)
#define PACKET_FOOTER		0xBE

#define HIL_MODE                    0
#define EXTERNAL_COMMANDS           1
#define AUTO_START_CALIBRATION		1

#define BUZZER_SILENT				1

// Altitude configuration
#define ALTITUDE_IIR_FILTER_ALPHA    0.1f

// Transition Configuration
#define PRELAUNCH_BURN_ACCEL_Y_THRESHOLD      	20.0f
#define PRELAUNCH_BURN_CONSECUTIVE_SAMPLES      5
#define BURN_MIN_VEL_Y_REACHED                  10.0f
#define BURN_APOGEE_VEL_Y_THRESHOLD         	5.0f
#define APOGEE_PARACHUTE_ALTITUDE_THRESHOLD 	500.0f
#define PARACHUTE_MIN_VEL_Y_REACHED             -3.0f
#define PARACHUTE_LANDED_VEL_Y_THRESHOLD    	2.0f

// Barometer Configuration
#define PRESSURE_CALIBRATION_DISCARD_SAMPLES    3000
#define PRESSURE_CALIBRATION_SAMPLES            1000

// Gyroscope Configuration
#define GYRO_CALIBRATION_DISCARD_SAMPLES        3000
#define GYRO_CALIBRATION_SAMPLES                1000

// Stack Sizes (words)
#define STACK_SIZE_TELEMETRY            256
#define STACK_SIZE_SENSOR_CONFIG        256
#define STACK_SIZE_STATE_MACHINE        512
#define STACK_SIZE_SD_LOGGING           1024

// SD Configuration
#define SD_LOGGING_RECORDS_PER_BUFFER       500

#endif //CONFIGURATION_H
