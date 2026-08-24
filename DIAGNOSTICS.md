# Flight Controller Diagnostics Plan

Proposed additions for runtime observability, sensor health tracking, and post-flight analysis.

---

## 1. Timing & Performance

### Loop Execution Time
Use ARM DWT cycle counter (`DWT->CYCCNT`) to measure `StateMachineTask` iteration time. Current `xTaskGetTickCount()` gives 1ms resolution — cycle counter gives microsecond precision. Shows whether 10ms loop deadline is met and how much margin remains.

### Sensor Read Latency
Timestamp before/after each `Mailbox_Read` call. Detects slow or hung sensors before they cause stale data. Log per-sensor min/max/avg over a rolling window.

### SD Write Duration
Measure `WriteLoggingBuffer()` execution time in `SDWriterTask`. SD cards degrade over time and write latency spikes appear before data loss. Catching a slow card early prevents missed flight data.

---

## 2. System Health

### FreeRTOS Stack High Water Marks
Call `uxTaskGetStackHighWaterMark()` per task periodically. Stack overflow is silent death on embedded — no fault handler, no warning. Tracking remaining headroom per task prevents it. Log for: `STATE_MACHINE_TASK`, `SD_PRODUCER`, `SD_WRITER`, `TELEMETRY_TASK`, `SENSOR_CONFIG`.

### Heap Free Bytes
Call `xPortGetFreeHeapSize()` and `xPortGetMinimumEverFreeHeapSize()`. Catches slow memory leaks. Minimum-ever value shows worst-case allocation pressure across entire flight.

### CPU Load
Use FreeRTOS idle hook counter or enable `configGENERATE_RUN_TIME_STATS` with a high-resolution timer. Shows how much processing margin exists. If CPU load exceeds ~80%, sensor reads and state transitions start missing deadlines.

### Queue Depths
Call `uxQueueMessagesWaiting()` on `SDLoggingQueue` and `CommandQueue` each loop iteration. Current `QUEUE_LENGTH` is 5 — if SD queue stays near capacity, flight data is being silently dropped by `xQueueSend` returning `errQUEUE_FULL`.

---

## 3. Sensor Diagnostics

### Sensor Error Counters
Count HAL return errors per sensor (SPI/I2C transaction failures). Current `SystemFaultFlags` are one-shot flags — they show first failure but not intermittent issues. Counters reveal sensors that fail sporadically under vibration or thermal stress.

Tracked per sensor:
- `BMP581`: I2C NACK count, timeout count
- `IIM42653`: SPI error count, CRC mismatch count
- `IIS2MDCTR`: I2C NACK count, timeout count

### Raw vs Filtered Altitude
Log both raw barometric altitude and IIR-filtered altitude in `FlightData_t`. Currently only filtered value is stored. Post-flight filter tuning (`ALTITUDE_IIR_FILTER_ALPHA`) requires raw data to evaluate different alpha values without reflying.

### Sensor Sample Freshness
Timestamp each mailbox write from sensor callbacks. In `StateMachineTask`, compare mailbox timestamp against current tick. If delta exceeds expected sample period, sensor data is stale — DMA may have stopped or sensor hung. Flag stale reads in diagnostics.

### Sensor Temperature Monitoring
`IIM42653` has an internal temperature sensor. Log it alongside `BMP581` temperature. Divergence between the two indicates thermal gradient across the board, which affects IMU bias stability.

---

## 4. State Machine Observability

### State Transition Log
Ring buffer of `{tick, previous_state, new_state, trigger}` entries. Currently transitions produce a buzzer beep and update `dbg_current_state`, but there is no persistent record. Post-flight analysis needs exact transition times. A 32-entry ring buffer costs ~256 bytes.

### Dropped SD Record Counter
`xQueueSend(SDLoggingQueue, &FlightData, 0)` silently drops data when queue is full. Count `errQUEUE_FULL` returns. `dbg_write_fault` tracks SD write errors but not queue overflow — these are different failure modes with different root causes.

### Command Reception Log
Log received commands with timestamps: `{tick, command_type, source}`. For post-flight verification that ground station commands were received and processed. Small ring buffer, same pattern as state transition log.

---

## 5. Proposed Diagnostics Struct

```c
typedef struct {
    // Timing
    uint32_t LoopExecutionUs;
    uint32_t LoopMaxExecutionUs;
    uint32_t BMP581ReadUs;
    uint32_t IIM42653ReadUs;
    uint32_t IIS2MDCTRReadUs;
    uint32_t SDWriteUs;

    // System health
    uint16_t StackWatermarkStateMachine;
    uint16_t StackWatermarkSDProducer;
    uint16_t StackWatermarkSDWriter;
    uint16_t StackWatermarkTelemetry;
    uint32_t HeapFreeBytes;
    uint32_t HeapMinFreeBytes;
    uint8_t  CPULoadPercent;
    uint8_t  SDQueueDepth;
    uint8_t  CommandQueueDepth;

    // Sensor errors
    uint16_t BMP581ErrorCount;
    uint16_t IIM42653ErrorCount;
    uint16_t IIS2MDCTRErrorCount;

    // Altitude
    float    RawAltitude;

    // Freshness (ticks since last sensor update)
    uint16_t BMP581StaleTicks;
    uint16_t IIM42653StaleTicks;
    uint16_t IIS2MDCTRStaleTicks;

    // Counters
    uint32_t DroppedSDRecords;
    uint32_t SDWriteFaults;

    // State transitions (latest)
    uint32_t LastTransitionTick;
    uint8_t  LastTransitionFrom;
    uint8_t  LastTransitionTo;
} Diagnostics_t;
```

Total size: ~78 bytes. Can be logged alongside `FlightData_t` or at lower frequency (every N iterations) to save SD bandwidth.

---

## 6. Implementation Priority

| Priority | Item | Effort | Impact |
|----------|------|--------|--------|
| 1 | DWT loop timing | Low | Shows deadline margin |
| 2 | Stack watermarks | Low | Prevents silent crashes |
| 3 | Queue depths + drop counter | Low | Detects data loss |
| 4 | Sensor error counters | Low | Catches intermittent failures |
| 5 | Raw altitude logging | Low | Enables post-flight filter tuning |
| 6 | State transition ring buffer | Medium | Full flight state reconstruction |
| 7 | Heap monitoring | Low | Catches leaks |
| 8 | Sensor freshness timestamps | Medium | Detects hung sensors |
| 9 | SD write duration | Low | Catches SD degradation |
| 10 | CPU load measurement | Medium | Requires timer config in CubeMX |

---

## 7. DWT Setup Reference

```c
// Enable DWT cycle counter (call once at startup)
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

// Measure execution time
uint32_t start = DWT->CYCCNT;
// ... work ...
uint32_t cycles = DWT->CYCCNT - start;
uint32_t microseconds = cycles / (SystemCoreClock / 1000000);
```
