// Wire layout of the telemetry packet. The flight packet must stay byte-identical to the original
// 52 byte layout the ground software parses. HIL builds add a 54 byte packet with CalStatus.
#include "Managers/StructManager.h"

#include <stddef.h>
#include <stdio.h>

static int Failures;

static void CheckValue(const char *Name, size_t Actual, size_t Expected)
{
    printf("%-40s %3u (expected %3u) %s\n", Name, (unsigned)Actual, (unsigned)Expected,
           Actual == Expected ? "ok" : "FAIL");
    if (Actual != Expected) {
        Failures++;
    }
}

#define CHECK_OFFSET(Type, Field, Expected) \
    CheckValue(#Type "." #Field, offsetof(Type, Field), (Expected))

#define CHECK_COMMON_LAYOUT(Type)                  \
    do {                                           \
        CHECK_OFFSET(Type, Sync, 0);               \
        CHECK_OFFSET(Type, Tick, 2);               \
        CHECK_OFFSET(Type, CalAccelX, 6);          \
        CHECK_OFFSET(Type, CalAccelY, 8);          \
        CHECK_OFFSET(Type, CalAccelZ, 10);         \
        CHECK_OFFSET(Type, CalGyroX, 12);          \
        CHECK_OFFSET(Type, CalGyroY, 14);          \
        CHECK_OFFSET(Type, CalGyroZ, 16);          \
        CHECK_OFFSET(Type, PressurePa, 18);        \
        CHECK_OFFSET(Type, TemperatureC, 20);      \
        CHECK_OFFSET(Type, Latitude, 21);          \
        CHECK_OFFSET(Type, Longitude, 25);         \
        CHECK_OFFSET(Type, GPSAltitude, 29);       \
        CHECK_OFFSET(Type, Satellites, 33);        \
        CHECK_OFFSET(Type, BarometricAltitude, 34);\
        CHECK_OFFSET(Type, BarometricVelocity, 38);\
        CHECK_OFFSET(Type, Flags, 42);             \
        CHECK_OFFSET(Type, BatteryVoltage, 46);    \
        CHECK_OFFSET(Type, State, 48);             \
        CHECK_OFFSET(Type, RelayState, 49);        \
        CHECK_OFFSET(Type, LastCommand, 50);       \
    } while (0)

#ifdef PROBE_HIL_TYPE
// Negative compile probe: run_telemetry_layout_tests.ps1 builds this with -DHIL_MODE=0 and expects
// the build to fail, proving the HIL packet type does not exist in a flight build.
static const size_t ProbeSize = sizeof(TelemetryPacketHil_t);
#endif

int main(void)
{
    printf("HIL_MODE=%d\n", (int)HIL_MODE);

    CheckValue("sizeof(TelemetryPacket_t)", sizeof(TelemetryPacket_t), 52);
    CHECK_COMMON_LAYOUT(TelemetryPacket_t);
    CHECK_OFFSET(TelemetryPacket_t, SyncEnd, 51);

#if HIL_MODE
    CheckValue("sizeof(TelemetryPacketHil_t)", sizeof(TelemetryPacketHil_t), 54);
    CHECK_COMMON_LAYOUT(TelemetryPacketHil_t);
    CHECK_OFFSET(TelemetryPacketHil_t, CalStatus, 51);
    CHECK_OFFSET(TelemetryPacketHil_t, SyncEnd, 53);
    CheckValue("sizeof(TelemetryWirePacket_t)", sizeof(TelemetryWirePacket_t), 54);
#else
    CheckValue("sizeof(TelemetryWirePacket_t)", sizeof(TelemetryWirePacket_t), 52);
#endif

    if (Failures != 0) {
        fprintf(stderr, "telemetry layout tests failed: %d\n", Failures);
        return 1;
    }
    puts("telemetry layout tests passed");
    return 0;
}
