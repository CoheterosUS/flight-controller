# Buzzer Usage

## API
`Buzzer_Notify(count, duration_ms)` -- defined in Buzzer.c, declared in BuzzerTask.h
- count: number of beeps
- duration_ms: beep duration (0 = default 80ms)
- Gap between beeps = 2x duration

Pre-scheduler only: `Buzzer_Beep_Counter(duration, count, gap, true)` -- direct, no task

## Beep patterns

| Pattern | Meaning | Location |
|---|---|---|
| 5x 200ms (HAL) | Error handler / boot error | main.c |
| 2x 100ms | State machine started | StateMachineTask.c |
| 1x 5ms | State change (debug) | StateMachineTask.c |
| 3x 80ms | Deep cal entry | 13DeepCalibrationStateHandler.c |
| 1-6x 80ms | Deep cal face captured (count = step+1) | 13DeepCalibrationStateHandler.c |
| 1x 80ms | Deep cal timeout, skipped | 2CalibrationStateHandler.c |
| 3x 100ms | Flash erase success | FlashLoggingTask.c |
| 2x 100ms | Flash dump success | FlashLoggingTask.c |
| 5x 100ms | Flash operation failed | FlashLoggingTask.c |
