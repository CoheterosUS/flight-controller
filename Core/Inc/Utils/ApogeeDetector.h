#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

// Pure C apogee detector (no HAL, no RTOS, host testable). See APOGEE_DETECTION_PLAN.md.
// Channel B: barometric altitude below (peak - APOGEE_DROP_M) for APOGEE_CONFIRM_SAMPLES new valid samples.
// Channel D: time since launch (BOOST entry) reaches APOGEE_TIMER_MS.
// Config macros come from Utils/configuration.h. For host tests, define APOGEE_DETECTOR_HOST_TEST and
// provide the same macros (with #ifndef defaults in the .c file).

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APOGEE_TRIGGER_NONE = 0,
    APOGEE_TRIGGER_BARO = 1,
    APOGEE_TRIGGER_TIMER = 2,
    APOGEE_TRIGGER_COMMAND = 3   // manual COMMAND_DROGUE, set by the state manager, never by the detector
} ApogeeTrigger_t;

typedef struct {
    uint32_t NowMs;             // current tick in ms
    bool BaroAllowed;           // channel B allowed (true in COAST and ACTIVE_CONTROL, false in BOOST)
    bool BaroValid;             // this iteration's barometer sample passed BaroSampleValid()
    float AltitudeM;            // barometric altitude above the pad, only meaningful when BaroValid
    uint32_t BaroSampleId;      // mailbox sample counter, a new value means a new sample
} ApogeeInput_t;

typedef struct {
    float Peak;                     // highest ACCEPTED altitude since Reset
    float LastAcceptedAltitude;
    uint32_t LastAcceptedTickMs;
    bool HaveAnchor;                // at least one accepted sample
    uint32_t LastSampleId;
    uint8_t DropCount;
    uint32_t LaunchTickMs;
    ApogeeTrigger_t Fired;          // latched once set
    uint32_t RejectedSamples;       // diagnostics: samples rejected by the slew gate
} ApogeeDetector_t;

// Plausibility check of one raw barometer sample. False means the sample must be ignored by every consumer.
// Not valid: non-finite pressure or temperature, pressure outside [BARO_VALID_MIN_PA, BARO_VALID_MAX_PA],
// temperature outside [BARO_VALID_MIN_TEMP_C, BARO_VALID_MAX_TEMP_C], reference pressure not valid
// (non-finite, <= 0, or ReferencePressurePaValid false), or sample id 0 (nothing published yet).
bool BaroSampleValid(float PressurePa, float TemperatureC, float ReferencePressurePa, bool ReferenceValid, uint32_t SampleId);

// Call on BOOST entry (launch). Clears everything and stores the launch tick.
void ApogeeDetector_Reset(ApogeeDetector_t *Detector, uint32_t LaunchTickMs);

// Call every state machine iteration while in BOOST, COAST or ACTIVE_CONTROL.
// Returns APOGEE_TRIGGER_NONE, BARO or TIMER. Once it returns non-NONE it keeps returning the same value.
// Channel B only processes an input when BaroSampleId differs from the last one seen. Rules for a new sample:
//  - not BaroValid: ignored completely (peak, anchor and counter untouched, not a drop, not a confirmation).
//  - slew gate: if |Altitude - LastAcceptedAltitude| > APOGEE_BARO_MAX_SPEED_MPS * dt + APOGEE_BARO_SLEW_MARGIN_M
//    (dt = seconds since the last accepted sample, from NowMs) the sample is rejected like an invalid one
//    (so a single bad spike, up or down, can neither poison the peak nor count as a drop).
//    The gate widens with dt, so it recovers by itself after a real gap.
//  - accepted: Peak = max(Peak, Altitude). If Altitude < Peak - APOGEE_DROP_M the DropCount increments, otherwise
//    it resets to 0. BARO fires when DropCount reaches APOGEE_CONFIRM_SAMPLES and BaroAllowed is true.
//  - while BaroAllowed is false (BOOST) accepted samples still update Peak and anchor but the drop counter stays 0.
// Channel D: TIMER fires when (NowMs - LaunchTickMs) >= APOGEE_TIMER_MS, evaluated every call, independent of the barometer.
// If both are true in the same call, BARO wins (it is the more precise event).
ApogeeTrigger_t ApogeeDetector_Update(ApogeeDetector_t *Detector, const ApogeeInput_t *Input);

#endif // APOGEE_DETECTOR_H
