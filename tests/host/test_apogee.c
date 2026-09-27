#include "Utils/ApogeeDetector.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define DROP_M 15.0f
#define TIMER_MS 28000u
#define CONFIRM_SAMPLES 5u

typedef bool (*TestFunction)(void);

static bool NearlyEqual(float A, float B, float Tolerance)
{
    return fabsf(A - B) <= Tolerance;
}

static ApogeeInput_t Input(uint32_t NowMs, bool BaroAllowed, bool BaroValid,
                           float AltitudeM, uint32_t BaroSampleId)
{
    ApogeeInput_t Result = {
        .NowMs = NowMs,
        .BaroAllowed = BaroAllowed,
        .BaroValid = BaroValid,
        .AltitudeM = AltitudeM,
        .BaroSampleId = BaroSampleId
    };
    return Result;
}

static void AddAccepted(ApogeeDetector_t *Detector, uint32_t *NowMs, uint32_t *SampleId,
                        bool BaroAllowed, float AltitudeM)
{
    ApogeeInput_t Sample = Input(*NowMs, BaroAllowed, true, AltitudeM, ++(*SampleId));
    (void)ApogeeDetector_Update(Detector, &Sample);
    *NowMs += 20u;
}

static void EstablishPeak(ApogeeDetector_t *Detector, uint32_t *NowMs, uint32_t *SampleId,
                          float AltitudeM)
{
    AddAccepted(Detector, NowMs, SampleId, true, AltitudeM);
    AddAccepted(Detector, NowMs, SampleId, true, AltitudeM);
    AddAccepted(Detector, NowMs, SampleId, true, AltitudeM);
}

static bool TestCleanFlight(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeTrigger_t Trigger = APOGEE_TRIGGER_NONE;
    float DetectionAltitude = 0.0f;

    ApogeeDetector_Reset(&Detector, 0u);
    for (unsigned Sample = 0u; Sample < 1500u; Sample++) {
        const float Time = (float)Sample * 0.02f;
        const float Altitude = 1000.0f - 4.8f * (20.0f - Time) * (20.0f - Time)
                             + 0.3f * ((Sample % 3u == 0u) ? 1.0f : -1.0f);
        ApogeeInput_t Current = Input(NowMs, true, true, Altitude, ++SampleId);
        Trigger = ApogeeDetector_Update(&Detector, &Current);
        if (Time < 20.0f && Trigger != APOGEE_TRIGGER_NONE) {
            return false;
        }
        if (Trigger != APOGEE_TRIGGER_NONE) {
            DetectionAltitude = Altitude;
            break;
        }
        NowMs += 20u;
    }

    return Trigger == APOGEE_TRIGGER_BARO
        && DetectionAltitude < Detector.Peak
        && DetectionAltitude > Detector.Peak - DROP_M - 4.0f
        && Detector.DropCount >= CONFIRM_SAMPLES;
}

static bool TestRepeatedSampleId(void)
{
    ApogeeDetector_t Detector;
    ApogeeDetector_Reset(&Detector, 0u);
    ApogeeInput_t Sample = Input(0u, true, true, 100.0f, 1u);
    (void)ApogeeDetector_Update(&Detector, &Sample);
    Sample.NowMs = 20u;
    Sample.AltitudeM = 101.0f;
    (void)ApogeeDetector_Update(&Detector, &Sample);
    return Detector.WindowCount == 1u
        && NearlyEqual(Detector.LastAcceptedAltitude, 100.0f, 0.001f);
}

static bool TestSampleIdWrap(void)
{
    ApogeeDetector_t Detector;
    ApogeeDetector_Reset(&Detector, 0u);
    ApogeeInput_t Sample = Input(0u, true, true, 100.0f, UINT32_MAX);
    (void)ApogeeDetector_Update(&Detector, &Sample);
    Sample.NowMs = 20u;
    Sample.AltitudeM = 101.0f;
    Sample.BaroSampleId = 0u;
    (void)ApogeeDetector_Update(&Detector, &Sample);
    return Detector.WindowCount == 2u && Detector.LastSampleId == 0u;
}

static bool TestConfirmationAndRecovery(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    for (unsigned Sample = 0u; Sample < 3u; Sample++) {
        AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    }
    AddAccepted(&Detector, &NowMs, &SampleId, true, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 100.0f);
    if (Detector.Fired != APOGEE_TRIGGER_NONE) {
        return false;
    }

    ApogeeDetector_Reset(&Detector, 0u);
    NowMs = 0u;
    SampleId = 0u;
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    for (unsigned Sample = 0u; Sample < 5u; Sample++) {
        AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    }
    return Detector.Fired == APOGEE_TRIGGER_BARO;
}

static bool TestMedianOutliers(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 90.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 100.0f);
    if (Detector.Fired != APOGEE_TRIGGER_NONE || !NearlyEqual(Detector.Peak, 100.0f, 0.001f)
            || Detector.RejectedSamples != 0u) {
        return false;
    }

    ApogeeDetector_Reset(&Detector, 0u);
    NowMs = 0u;
    SampleId = 0u;
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 110.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 100.0f);
    return Detector.Fired == APOGEE_TRIGGER_NONE
        && NearlyEqual(Detector.Peak, 100.0f, 0.001f)
        && Detector.RejectedSamples == 0u;
}

static bool TestStepHasOneSampleLag(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    if (Detector.DropCount != 0u) {
        return false;
    }
    AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    return Detector.DropCount == 1u;
}

static bool TestSpikeDownRejected(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 60.0f);
    return Detector.Fired == APOGEE_TRIGGER_NONE
        && Detector.DropCount == 0u
        && Detector.RejectedSamples == 1u
        && NearlyEqual(Detector.Peak, 100.0f, 0.001f);
}

static bool TestBoostThenCoast(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    AddAccepted(&Detector, &NowMs, &SampleId, false, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, false, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, false, 100.0f);
    for (unsigned Sample = 0u; Sample < 6u; Sample++) {
        AddAccepted(&Detector, &NowMs, &SampleId, false, 84.0f);
    }
    if (Detector.Fired != APOGEE_TRIGGER_NONE || Detector.DropCount != 0u) {
        return false;
    }
    for (unsigned Sample = 0u; Sample < 6u; Sample++) {
        AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
        if (Detector.Fired != APOGEE_TRIGGER_NONE && Sample < 4u) {
            return false;
        }
    }
    return Detector.Fired == APOGEE_TRIGGER_BARO;
}

static bool TestInvalidSamplesIgnored(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    for (unsigned Sample = 0u; Sample < 50u; Sample++) {
        const bool Valid = (Sample % 3u) == 0u;
        const float Altitude = (Sample % 3u) == 1u ? NAN : INFINITY;
        ApogeeInput_t Invalid = Input(NowMs, true, Valid, Altitude, ++SampleId);
        (void)ApogeeDetector_Update(&Detector, &Invalid);
        NowMs += 20u;
    }
    if (Detector.Fired != APOGEE_TRIGGER_NONE || Detector.WindowCount != 3u
            || Detector.DropCount != 0u || Detector.RejectedSamples != 0u) {
        return false;
    }
    for (unsigned Sample = 0u; Sample < 7u; Sample++) {
        AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    }
    return Detector.Fired == APOGEE_TRIGGER_BARO;
}

static bool TestSpikeUpDoesNotPoisonPeak(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 600.0f);
    for (unsigned Sample = 0u; Sample < 500u && Detector.Fired == APOGEE_TRIGGER_NONE; Sample++) {
        const float Time = (float)Sample * 0.02f;
        const float Altitude = 100.0f - 4.8f * Time * Time;
        AddAccepted(&Detector, &NowMs, &SampleId, true, Altitude);
    }
    return Detector.Fired == APOGEE_TRIGGER_BARO
        && NearlyEqual(Detector.Peak, 100.0f, 0.001f)
        && Detector.RejectedSamples == 1u;
}

static bool TestZeroAfterHighRejected(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 3000.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 0.0f);
    return Detector.Fired == APOGEE_TRIGGER_NONE
        && Detector.RejectedSamples == 1u
        && NearlyEqual(Detector.Peak, 3000.0f, 0.001f);
}

static bool TestBadFirstSampleDoesNotSetPeak(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    /* First accepted sample is an impossible +500 m (there is no anchor yet for the slew gate). */
    AddAccepted(&Detector, &NowMs, &SampleId, true, 500.0f);
    for (unsigned Sample = 0u; Sample < 1000u && Detector.Fired == APOGEE_TRIGGER_NONE; Sample++) {
        const float Time = (float)Sample * 0.02f;
        const float Altitude = 300.0f - 4.8f * (Time - 8.0f) * (Time - 8.0f);
        AddAccepted(&Detector, &NowMs, &SampleId, true, Altitude);
        if (Detector.Fired != APOGEE_TRIGGER_NONE && Time < 8.0f) {
            return false; /* fired during the climb */
        }
    }
    return Detector.Fired == APOGEE_TRIGGER_BARO
        && NearlyEqual(Detector.Peak, 300.0f, 1.0f)
        && NowMs > 8000u;
}

static bool TestTimerExactAndWrap(void)
{
    ApogeeDetector_t Detector;
    ApogeeInput_t Sample;
    ApogeeDetector_Reset(&Detector, 100u);
    Sample = Input(100u + TIMER_MS - 1u, false, false, 0.0f, 0u);
    if (ApogeeDetector_Update(&Detector, &Sample) != APOGEE_TRIGGER_NONE) {
        return false;
    }
    Sample.NowMs++;
    if (ApogeeDetector_Update(&Detector, &Sample) != APOGEE_TRIGGER_TIMER) {
        return false;
    }

    ApogeeDetector_Reset(&Detector, UINT32_MAX - 100u);
    Sample = Input(UINT32_MAX - 100u, false, false, 0.0f, 0u);
    if (ApogeeDetector_Update(&Detector, &Sample) != APOGEE_TRIGGER_NONE) {
        return false;
    }
    Sample.NowMs = (uint32_t)((UINT32_MAX - 100u) + TIMER_MS - 1u);
    if (ApogeeDetector_Update(&Detector, &Sample) != APOGEE_TRIGGER_NONE) {
        return false;
    }
    Sample.NowMs++;
    return ApogeeDetector_Update(&Detector, &Sample) == APOGEE_TRIGGER_TIMER;
}

static bool TestBaroWinsTimer(void)
{
    ApogeeDetector_t Detector;
    uint32_t SampleId = 0u;
    uint32_t NowMs = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    for (unsigned Sample = 0u; Sample < 4u; Sample++) {
        AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    }
    ApogeeInput_t Final = Input(TIMER_MS, true, true, 84.0f, ++SampleId);
    return ApogeeDetector_Update(&Detector, &Final) == APOGEE_TRIGGER_BARO;
}

static bool TestLatchAndReset(void)
{
    ApogeeDetector_t Detector;
    uint32_t NowMs = 0u;
    uint32_t SampleId = 0u;
    ApogeeDetector_Reset(&Detector, 0u);
    EstablishPeak(&Detector, &NowMs, &SampleId, 100.0f);
    for (unsigned Sample = 0u; Sample < 7u; Sample++) {
        AddAccepted(&Detector, &NowMs, &SampleId, true, 84.0f);
    }
    if (Detector.Fired != APOGEE_TRIGGER_BARO) {
        return false;
    }
    ApogeeInput_t After = Input(NowMs, true, true, 600.0f, ++SampleId);
    if (ApogeeDetector_Update(&Detector, &After) != APOGEE_TRIGGER_BARO) {
        return false;
    }
    ApogeeDetector_Reset(&Detector, 1234u);
    return Detector.WindowCount == 0u
        && Detector.WindowIndex == 0u
        && Detector.Peak == 0.0f
        && Detector.LastAcceptedAltitude == 0.0f
        && Detector.LastAcceptedTickMs == 0u
        && !Detector.HaveAnchor
        && Detector.LastSampleId == 0u
        && Detector.DropCount == 0u
        && Detector.LaunchTickMs == 1234u
        && Detector.Fired == APOGEE_TRIGGER_NONE
        && Detector.RejectedSamples == 0u;
}

static bool TestBaroValidity(void)
{
    const float ValidPressure = 30000.0f;
    const float ValidTemperature = -40.0f;
    const float ValidReference = 100000.0f;
    if (!BaroSampleValid(ValidPressure, ValidTemperature, ValidReference, true, 1u)
            || !BaroSampleValid(125000.0f, 85.0f, ValidReference, true, UINT32_MAX)) {
        return false;
    }
    return !BaroSampleValid(NAN, 20.0f, ValidReference, true, 1u)
        && !BaroSampleValid(INFINITY, 20.0f, ValidReference, true, 1u)
        && !BaroSampleValid(29999.9f, 20.0f, ValidReference, true, 1u)
        && !BaroSampleValid(125000.1f, 20.0f, ValidReference, true, 1u)
        && !BaroSampleValid(50000.0f, NAN, ValidReference, true, 1u)
        && !BaroSampleValid(50000.0f, INFINITY, ValidReference, true, 1u)
        && !BaroSampleValid(50000.0f, -40.1f, ValidReference, true, 1u)
        && !BaroSampleValid(50000.0f, 85.1f, ValidReference, true, 1u)
        && !BaroSampleValid(50000.0f, 20.0f, ValidReference, false, 1u)
        && !BaroSampleValid(50000.0f, 20.0f, 0.0f, true, 1u)
        && !BaroSampleValid(50000.0f, 20.0f, NAN, true, 1u)
        && !BaroSampleValid(50000.0f, 20.0f, ValidReference, true, 0u);
}

static uint32_t RandomState;
static bool HaveGaussian;
static float GaussianValue;

static float UniformRandom(void)
{
    RandomState = RandomState * 1664525u + 1013904223u;
    return ((float)(RandomState >> 8) + 1.0f) / 16777217.0f;
}

static float GaussianRandom(void)
{
    if (HaveGaussian) {
        HaveGaussian = false;
        return GaussianValue;
    }
    const float U1 = UniformRandom();
    const float U2 = UniformRandom();
    const float Radius = sqrtf(-2.0f * logf(U1));
    const float Angle = 6.2831853071795864769f * U2;
    GaussianValue = Radius * sinf(Angle);
    HaveGaussian = true;
    return Radius * cosf(Angle);
}

static bool TestRandomFlights(void)
{
    for (uint32_t Seed = 1u; Seed <= 20u; Seed++) {
        ApogeeDetector_t Detector;
        ApogeeDetector_Reset(&Detector, 0u);
        RandomState = 0xA5A50000u + Seed;
        HaveGaussian = false;
        uint32_t TriggerTime = 0u;
        ApogeeTrigger_t Trigger = APOGEE_TRIGGER_NONE;
        for (uint32_t Sample = 0u; Sample <= 1400u; Sample++) {
            const float Time = (float)Sample * 0.02f;
            const float Difference = 25.0f - Time;
            const float Altitude = 3000.0f - 4.8f * Difference * Difference
                                 + 0.3f * GaussianRandom();
            ApogeeInput_t Current = Input(Sample * 20u, true, true, Altitude, Sample + 1u);
            Trigger = ApogeeDetector_Update(&Detector, &Current);
            if (Time < 25.0f && Trigger != APOGEE_TRIGGER_NONE) {
                return false;
            }
            if (Trigger != APOGEE_TRIGGER_NONE) {
                TriggerTime = Current.NowMs;
                break;
            }
        }
        if (Trigger != APOGEE_TRIGGER_BARO || TriggerTime < 25000u || TriggerTime > 28000u) {
            return false;
        }
    }
    return true;
}

static bool RunCase(const char *Name, TestFunction Function)
{
    const bool Passed = Function();
    printf("%s: %s\n", Passed ? "PASS" : "FAIL", Name);
    return Passed;
}

int main(void)
{
    const struct {
        const char *Name;
        TestFunction Function;
    } Cases[] = {
        {"clean climb and descent confirmation", TestCleanFlight},
        {"repeated sample id is ignored", TestRepeatedSampleId},
        {"sample id wrap is a new sample", TestSampleIdWrap},
        {"four low samples recover and five low samples fire", TestConfirmationAndRecovery},
        {"isolated median outliers do not affect the peak", TestMedianOutliers},
        {"a descent step has one sample of median lag", TestStepHasOneSampleLag},
        {"single downward spike is slew rejected", TestSpikeDownRejected},
        {"boost does not fire, coast starts the drop count", TestBoostThenCoast},
        {"invalid samples are ignored in coast and descent", TestInvalidSamplesIgnored},
        {"upward spike does not poison the peak", TestSpikeUpDoesNotPoisonPeak},
        {"zero altitude after a high sample is rejected", TestZeroAfterHighRejected},
        {"bad first sample cannot set the peak", TestBadFirstSampleDoesNotSetPeak},
        {"timer exact boundary and tick wrap", TestTimerExactAndWrap},
        {"barometer wins when timer is simultaneous", TestBaroWinsTimer},
        {"firing is latched and reset clears state", TestLatchAndReset},
        {"barometer validity truth table", TestBaroValidity},
        {"twenty randomized nominal flights", TestRandomFlights}
    };
    bool AllPassed = true;
    for (unsigned Case = 0u; Case < sizeof(Cases) / sizeof(Cases[0]); Case++) {
        if (!RunCase(Cases[Case].Name, Cases[Case].Function)) {
            AllPassed = false;
        }
    }
    return AllPassed ? 0 : 1;
}
