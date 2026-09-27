#include "Tasks/TelemetryTask.h"

#include <stdio.h>

int main(void)
{
    for (unsigned Command = 0u; Command <= 0xffu; Command++) {
        bool Expected = false;
#if EXTERNAL_COMMANDS
        Expected = Command == COMMAND_RESET || Command == COMMAND_GROUND_ABORT ||
                   Command == COMMAND_CALIBRATION;
#if HIL_MODE
        Expected = Expected || Command == COMMAND_DROGUE || Command == COMMAND_LANDED;
#endif
#endif
        if (Telemetry_CommandAllowed((uint8_t)Command) != Expected) {
            fprintf(stderr, "command gate mismatch for 0x%02x\n", Command);
            return 1;
        }
    }
    puts("command gate tests passed");
    return 0;
}
