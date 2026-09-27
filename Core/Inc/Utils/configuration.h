#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define QUEUE_LENGTH    5

#define PACKET_HEADER      	0xCAFE
#define PACKET_HEADER_LSB  	(PACKET_HEADER & 0xFF)
#define PACKET_HEADER_MSB  	((PACKET_HEADER >> 8) & 0xFF)
#define PACKET_FOOTER		0xBE

#define SD_LOGGING_ENABLED          0
#define HIL_MODE                    1

// FLIGHT_BUILD must be 1 for any image that goes on a rocket. It refuses to compile with HIL switches on,
// because HIL_MODE 1 never configures or starts the real sensors (no launch detection, no parachutes).
// Pre-flight checklist: FLIGHT_BUILD 1, HIL_MODE 0, HIL_PRESEED_M 0, erase the flash log (old records use another layout).
#define FLIGHT_BUILD                0
#if FLIGHT_BUILD && (HIL_MODE || HIL_PRESEED_M)
#error "FLIGHT_BUILD requires HIL_MODE 0 and HIL_PRESEED_M 0"
#endif

// Manual drogue command (COMMAND_DROGUE): 0 = only accepted in BOOST, COAST, ACTIVE_CONTROL (a stray UART frame
// must not fire the drogue on the pad), 1 = accepted in every state (bench pyro tests).
#define DROGUE_COMMAND_ANY_STATE    0
#if FLIGHT_BUILD && DROGUE_COMMAND_ANY_STATE
#error "FLIGHT_BUILD requires DROGUE_COMMAND_ANY_STATE 0"
#endif

// HIL only (see BRINGUP_AND_HIL.md)
#define HIL_PRESEED_M                       0
#if HIL_PRESEED_M && !HIL_MODE
#error "HIL_PRESEED_M requires HIL_MODE"
#endif
#define EXTERNAL_COMMANDS           1
#define AUTO_START_CALIBRATION		1

#define BUZZER_ENABLED				1
#define PYRO_PULSE_MS				3000

// Main loop rate
#define LOOP_RATE_HZ                250
#define LOOP_PERIOD_MS              (1000 / LOOP_RATE_HZ)
#define LOOP_DT                     (1.0f / LOOP_RATE_HZ)

// Altitude configuration
#define KALMAN_INITIAL_ROLL_DEG      0.0f
#define KALMAN_INITIAL_PITCH_DEG     90.0f
#define KALMAN_INITIAL_YAW_DEG       0.0f

#define ALTITUDE_IIR_FILTER_ALPHA    (10.0f / LOOP_RATE_HZ)

// Barometer Configuration
// Counted in UNIQUE barometer samples (not loop iterations): about 1 s discard and 5 s average at the sensor rate
#define PRESSURE_CALIBRATION_DISCARD_SAMPLES    50
#define PRESSURE_CALIBRATION_SAMPLES            250

// IMU Configuration
// IMU timing (WP-E)
#define IMU_ODR_HZ                          200

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

// Deep calibration gesture (WP-D)
#define DEEP_CAL_HOLD_MS                    10000
#define DEEP_CAL_HOLD_AXIS_RAW              1        // 0=X, 1=Y, 2=Z
#define DEEP_CAL_HOLD_AXIS_SIGN             (+1)     // nose down reads +9.81 on raw Y
#define DEEP_CAL_HOLD_G_BAND_PCT            10
#define DEEP_CAL_HOLD_LATERAL_MAX_G         0.2f
#define DEEP_CAL_HOLD_GYRO_MAX_DPS          2.0f

// Deep calibration tumble (WP-A, WP-D)
#define DEEP_CAL_POSE_SETTLE_MS             10000
#define DEEP_CAL_POSE_SAMPLE_MS             30000
#define DEEP_CAL_POSE_G_BAND_PCT            10
#define DEEP_CAL_POSE_DOMINANT_MIN_G        0.8f
#define DEEP_CAL_POSE_ANGLE_TOL_DEG         20.0f
#define DEEP_CAL_GYRO_MOTION_MAX_DPS        2.0f
#define DEEP_CAL_POSE_MAX_RESTARTS          5
#define DEEP_CAL_TIMEOUT_MS                 (2 * 6 * (DEEP_CAL_POSE_SETTLE_MS + DEEP_CAL_POSE_SAMPLE_MS))
#define DEEP_CAL_NORM_RESIDUAL_MAX          0.02f    // fraction of g, per pose after correction
#define DEEP_CAL_DET_TOL                    0.1f     // |det(Q) - 1|

// Buzzer patterns (WP-C)
#define BUZZER_POSE_ON_MS                   400
#define BUZZER_POSE_OFF_MS                  400
#define BUZZER_DEEPCAL_ENTERED_MS           2000
#define BUZZER_DEEPCAL_OK_COUNT             3
#define BUZZER_DEEPCAL_OK_ON_MS             1200
#define BUZZER_DEEPCAL_OK_OFF_MS            500
#define BUZZER_DEEPCAL_FAIL_MS              4000
#define STACK_SIZE_BUZZER                   256

// Per-boot bias calibration (WP-E)
#define GYRO_CAL_STILL_MAX_DPS              1.0f
#define ACCEL_BIAS_CAL_DISCARD_SAMPLES      1000
#define ACCEL_BIAS_CAL_SAMPLES              1000
#define ACCEL_BIAS_LATERAL_MAX_G            0.1f
#define ACCEL_BIAS_STILL_GYRO_MAX_DPS       2.0f
#define CAL_EXPECTED_NOSE_UP_X              9.81f

// Flash calibration sector (WP-B)
#define FLASH_CAL_SECTOR_ADDRESS            0x003FF000

// GPS Configuration
#define GPS_FIX_REQUIRED             0
#define GPS_FIX_MIN_SATELLITES       1
#define GPS_ALTITUDE_ASL_BASELINE    90.0f // Baseline to calculate AGL from ASL

// Stack Sizes (words)
#define STACK_SIZE_TELEMETRY            512
#define STACK_SIZE_SENSOR_CONFIG        512
#define STACK_SIZE_STATE_MACHINE        1024
#define STACK_SIZE_SD_LOGGING           1024
#define STACK_SIZE_PYRO                 256
#define STACK_SIZE_FLASH_LOGGING        768

// Telemetry Configuration
#define TELEMETRY_DIVIDER                   LOOP_RATE_HZ  // 1Hz in active states
#define TELEMETRY_DIVIDER_IDLE              LOOP_RATE_HZ  // 1Hz in IDLE

// SD Configuration
#define SD_LOGGING_RECORDS_PER_BUFFER       500

// Flash Configuration
#define FLASH_DUMP_TO_SD                    0
#define FLASH_ERASE_ALL                     0
#define FLASH_LOGGING_DIVIDER               (LOOP_RATE_HZ / 10)
#define STACK_SIZE_FLASH_MAINTENANCE        1024
#define FLASH_LOGGING_QUEUE_LENGTH          10
#define FLASH_RECORDS_PER_PAGE              4

// Transition Configuration

// Prelaunch to Boost Acceleration Threshold
#define PRELAUNCH_BOOST_ACCEL_Y_THRESHOLD      	-20.0f // IMU Y-axis is inverted, so negative is upwards
#define PRELAUNCH_BOOST_CONSECUTIVE_SAMPLES      (LOOP_RATE_HZ / 20)

// Boost to Coast Acceleration Threshold
#define BOOST_COAST_ACCEL_Y_THRESHOLD          -5.0f // IMU Y-axis is inverted, so negative is upwards
#define BOOST_COAST_CONSECUTIVE_SAMPLES        (LOOP_RATE_HZ / 20)

// Coast to Active Control Altitude Threshold
// TODO(EuRoC): ACTIVE_CONTROL is a EuRoC feature, set the altitudes from the judges' released rules (see BACKLOG.md)
#define COAST_ACTIVE_CONTROL_BAROM_ALT_THRESHOLD    2000.0f // Barometric altitude threshold for active control
#define COAST_ACTIVE_CONTROL_CONSECUTIVE_SAMPLES    (((BARO_ODR_HZ / 20) < 2) ? 2 : (BARO_ODR_HZ / 20))

// Apogee detection (see APOGEE_DETECTION_PLAN.md). Channel B: barometer drop from peak. Channel D: timer from BOOST entry.
#define BARO_ODR_HZ                         46      // BMP581 typical rate in continuous mode at pressure x32, temperature x2
#define BARO_VELOCITY_WINDOW_MS             1000
#define BARO_GRAVITY_MS2                    9.81f   // Must match the Kalman barometer model g[2]
#define APOGEE_DROP_M                       15.0f
#define APOGEE_CONFIRM_SAMPLES              5       // new valid barometer samples
#define APOGEE_TIMER_MS                     28000   // launch (BOOST entry) to forced drogue. Must exceed the latest plausible apogee time
#define APOGEE_BARO_MAX_SPEED_MPS           400.0f  // slew gate speed
#define APOGEE_BARO_SLEW_MARGIN_M           10.0f   // slew gate constant margin

// Barometer sample plausibility (BMP581 operating range, verify against Datasheets/BMP581.pdf)
#define BARO_VALID_MIN_PA                   30000.0f
#define BARO_VALID_MAX_PA                   125000.0f
#define BARO_VALID_MIN_TEMP_C               (-40.0f)
#define BARO_VALID_MAX_TEMP_C               85.0f

// Apogee to Main Parachute
#define APOGEE_MAIN_PARACHUTE_BAROM_ALT_THRESHOLD 	450.0f // WARN: AGL
#define APOGEE_MAIN_PARACHUTE_GPS_ALT_ENABLED		0
#define APOGEE_MAIN_PARACHUTE_GPS_ALT_THRESHOLD		450.0f // WARN: ASL
#define APOGEE_MAIN_PARACHUTE_DELAY_ENABLED			0
#define APOGEE_MAIN_PARACHUTE_DELAY_MS				30000
// Placeholders to tune with the descent simulation. This channel must never fire while the barometer is healthy.
#define APOGEE_MAIN_BARO_LOSS_MS                    5000
#define APOGEE_MAIN_BARO_LOSS_DELAY_MS              60000

// Main Parachute to Landed
#define MAIN_PARACHUTE_LANDED_BAROM_ALT_THRESHOLD		100.0f
#define MAIN_PARACHUTE_LANDED_BAROM_VEL_Y_THRESHOLD		2.0f
#define MAIN_PARACHUTE_LANDED_CONSECUTIVE_SAMPLES		BARO_ODR_HZ

#define LANDED_SD_STOP_DELAY_ENABLED            1
#define LANDED_SD_STOP_DELAY_MS                 5000
#define LANDED_FLASH_STOP_DELAY_ENABLED         1
#define LANDED_FLASH_STOP_DELAY_MS              5000

#endif //CONFIGURATION_H
