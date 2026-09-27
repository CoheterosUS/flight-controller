#include "Utils/ApogeeDetector.h"

#include <math.h>

#ifdef APOGEE_DETECTOR_HOST_TEST
#ifndef APOGEE_DROP_M
#define APOGEE_DROP_M 15.0f
#endif
#ifndef APOGEE_CONFIRM_SAMPLES
#define APOGEE_CONFIRM_SAMPLES 5
#endif
#ifndef APOGEE_TIMER_MS
#define APOGEE_TIMER_MS 28000
#endif
#ifndef APOGEE_BARO_MAX_SPEED_MPS
#define APOGEE_BARO_MAX_SPEED_MPS 400.0f
#endif
#ifndef APOGEE_BARO_SLEW_MARGIN_M
#define APOGEE_BARO_SLEW_MARGIN_M 10.0f
#endif
#ifndef BARO_VALID_MIN_PA
#define BARO_VALID_MIN_PA 30000.0f
#endif
#ifndef BARO_VALID_MAX_PA
#define BARO_VALID_MAX_PA 125000.0f
#endif
#ifndef BARO_VALID_MIN_TEMP_C
#define BARO_VALID_MIN_TEMP_C (-40.0f)
#endif
#ifndef BARO_VALID_MAX_TEMP_C
#define BARO_VALID_MAX_TEMP_C 85.0f
#endif
#else
#include "Utils/configuration.h"
#endif

static float Median3(const float Values[3])
{
    float A = Values[0];
    float B = Values[1];
    float C = Values[2];

    if (A > B) {
        const float Temporary = A;
        A = B;
        B = Temporary;
    }
    if (B > C) {
        const float Temporary = B;
        B = C;
        C = Temporary;
    }
    if (A > B) {
        const float Temporary = A;
        A = B;
        B = Temporary;
    }
    return B;
}

bool BaroSampleValid(float PressurePa, float TemperatureC, float ReferencePressurePa,
                     bool ReferenceValid, uint32_t SampleId)
{
    return ReferenceValid
        && SampleId != 0u
        && isfinite(PressurePa)
        && PressurePa >= BARO_VALID_MIN_PA
        && PressurePa <= BARO_VALID_MAX_PA
        && isfinite(TemperatureC)
        && TemperatureC >= BARO_VALID_MIN_TEMP_C
        && TemperatureC <= BARO_VALID_MAX_TEMP_C
        && isfinite(ReferencePressurePa)
        && ReferencePressurePa > 0.0f;
}

void ApogeeDetector_Reset(ApogeeDetector_t *Detector, uint32_t LaunchTickMs)
{
    Detector->Window[0] = 0.0f;
    Detector->Window[1] = 0.0f;
    Detector->Window[2] = 0.0f;
    Detector->WindowCount = 0u;
    Detector->WindowIndex = 0u;
    Detector->Peak = 0.0f;
    Detector->PeakValid = false;
    Detector->LastAcceptedAltitude = 0.0f;
    Detector->LastAcceptedTickMs = 0u;
    Detector->HaveAnchor = false;
    Detector->LastSampleId = 0u;
    Detector->DropCount = 0u;
    Detector->LaunchTickMs = LaunchTickMs;
    Detector->Fired = APOGEE_TRIGGER_NONE;
    Detector->RejectedSamples = 0u;
}

ApogeeTrigger_t ApogeeDetector_Update(ApogeeDetector_t *Detector, const ApogeeInput_t *Input)
{
    if (Detector->Fired != APOGEE_TRIGGER_NONE) {
        return Detector->Fired;
    }

    if (Input != 0 && Input->BaroSampleId != Detector->LastSampleId) {
        Detector->LastSampleId = Input->BaroSampleId;

        if (Input->BaroValid && isfinite(Input->AltitudeM)) {
            bool Accepted = true;
            if (Detector->HaveAnchor) {
                const uint32_t ElapsedMs = (uint32_t)(Input->NowMs - Detector->LastAcceptedTickMs);
                const float ElapsedSeconds = (ElapsedMs == 0u) ? 1.0e-6f : (float)ElapsedMs / 1000.0f;
                const float MaximumChange = APOGEE_BARO_MAX_SPEED_MPS * ElapsedSeconds
                                          + APOGEE_BARO_SLEW_MARGIN_M;
                if (fabsf(Input->AltitudeM - Detector->LastAcceptedAltitude) > MaximumChange) {
                    Accepted = false;
                    Detector->RejectedSamples++;
                }
            }

            if (Accepted) {
                Detector->Window[Detector->WindowIndex] = Input->AltitudeM;
                Detector->WindowIndex = (uint8_t)((Detector->WindowIndex + 1u) % 3u);
                if (Detector->WindowCount < 3u) {
                    Detector->WindowCount++;
                }

                Detector->HaveAnchor = true;
                if (Detector->WindowCount == 3u) {
                    /* The peak is only ever taken from the median, never from a single raw sample,
                       so one bad first sample cannot set an impossible peak. */
                    const float FilteredAltitude = Median3(Detector->Window);
                    if (!Detector->PeakValid || FilteredAltitude > Detector->Peak) {
                        Detector->Peak = FilteredAltitude;
                        Detector->PeakValid = true;
                    }

                    if (Input->BaroAllowed) {
                        if (FilteredAltitude < Detector->Peak - APOGEE_DROP_M) {
                            Detector->DropCount++;
                        } else {
                            Detector->DropCount = 0u;
                        }
                        if (Detector->DropCount >= APOGEE_CONFIRM_SAMPLES) {
                            Detector->Fired = APOGEE_TRIGGER_BARO;
                        }
                    }
                }

                if (!Input->BaroAllowed) {
                    Detector->DropCount = 0u;
                }

                Detector->LastAcceptedAltitude = Input->AltitudeM;
                Detector->LastAcceptedTickMs = Input->NowMs;
            }
        }
    }

    if (Detector->Fired == APOGEE_TRIGGER_NONE && Input != 0
            && (uint32_t)(Input->NowMs - Detector->LaunchTickMs) >= APOGEE_TIMER_MS) {
        Detector->Fired = APOGEE_TRIGGER_TIMER;
    }

    return Detector->Fired;
}
