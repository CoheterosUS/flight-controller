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

#define BUZZER_ENABLED				0

// Altitude configuration
#define ALTITUDE_IIR_FILTER_ALPHA    0.1f

// Barometer Configuration
#define PRESSURE_CALIBRATION_DISCARD_SAMPLES    3000
#define PRESSURE_CALIBRATION_SAMPLES            1000

// Gyroscope Configuration
#define GYRO_CALIBRATION_DISCARD_SAMPLES        3000
#define GYRO_CALIBRATION_SAMPLES                1000

// GPS Configuration
#define GPS_FIX_MIN_SATELLITES       1
#define GPS_ALTITUDE_ASL_BASELINE    90.0f // Baseline to calculate AGL from ASL

// Stack Sizes (words)
#define STACK_SIZE_TELEMETRY            256
#define STACK_SIZE_SENSOR_CONFIG        256
#define STACK_SIZE_STATE_MACHINE        512
#define STACK_SIZE_SD_LOGGING           1024

// Telemetry Configuration (Sends on every 10 iterations of main loop)
#define TELEMETRY_DIVIDER                   10

// SD Configuration
#define SD_LOGGING_RECORDS_PER_BUFFER       500

// Transition Configuration

// Prelaunch to Burn Acceleration Threshold
#define PRELAUNCH_BURN_ACCEL_Y_THRESHOLD      	20.0f
#define PRELAUNCH_BURN_CONSECUTIVE_SAMPLES      5

// Burn to Passive Burnout Acceleration Threshold
#define BURN_PASSIVE_BURNOUT_ACCEL_Y_THRESHOLD          5.0f
#define BURN_PASSIVE_BURNOUT_CONSECUTIVE_SAMPLES        5

// Passive Burnout to Active Burnout Automatic

// Active Burnout to Apogee Barometric Altitude + GPS Altitude + GPS Vertical Velocity
#define ACTIVE_BURNOUT_APOGEE_BAROM_ALT_THRESHOLD		2900.0f
#define ACTIVE_BURNOUT_APOGEE_GPS_ALT_THRESHOLD			2900.0f
#define ACTIVE_BURNOUT_APOGEE_GPS_VEL_Y_THRESHOLD		0.0f
#define ACTIVE_BURNOUT_APOGEE_DELAY_ENABLED				1
#define ACTIVE_BURNOUT_APOGEE_DELAY_MS					10000

// Apogee to Parachute
#define APOGEE_PARACHUTE_BAROM_ALT_THRESHOLD 	450.0f
#define APOGEE_PARACHUTE_DELAY_ENABLED			1
#define APOGEE_PARACHUTE_DELAY_MS				30000

// Parachute to Landed
#define PARACHUTE_LANDED_BAROM_ALT_THRESHOLD		100.0f
#define PARACHUTE_LANDED_BAROM_VEL_Y_THRESHOLD		2.0f
#define PARACHUTE_LANDED_CONSECUTIVE_SAMPLES		10

#define LANDED_SD_STOP_DELAY_ENABLED            1
#define LANDED_SD_STOP_DELAY_MS                 5000

#endif //CONFIGURATION_H
