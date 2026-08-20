function renderStats(records, skipped) {
    const statsDiv = document.getElementById('stats');
    statsDiv.classList.remove('hidden');
    const n = records.length;
    const pressure = getCol(records, 'PressurePa');
    const temp = getCol(records, 'TemperatureC');
    const alt = getCol(records, 'GPSAltitude');
    const pMin = arrMin(pressure).toFixed(1);
    const pMax = arrMax(pressure).toFixed(1);
    const tAvg = (temp.reduce((a, b) => a + b, 0) / n).toFixed(1);
    const altRange = (arrMax(alt) - arrMin(alt)).toFixed(3);
    const voltage = getCol(records, 'BatteryVoltage').filter(v => v > 0 && v < 100);
    const vMin = voltage.length ? arrMin(voltage).toFixed(1) : 'N/A';
    const vMax = voltage.length ? arrMax(voltage).toFixed(1) : 'N/A';
    const states = records[0] && records[0].State !== undefined
        ? [...new Set(getCol(records, 'State'))].sort((a,b) => a-b).join(', ') : '—';

    const ticks = getCol(records, 'Tick');
    const hasTick = ticks.length > 1 && ticks[0] !== undefined;
    const durationMs = hasTick ? ticks[ticks.length - 1] - ticks[0] : 0;
    const durationStr = hasTick ? (durationMs / 1000).toFixed(1) + ' s' : '—';
    const rateStr = hasTick && durationMs > 0 ? (((n - 1) / durationMs) * 1000).toFixed(0) + ' Hz' : '—';

    statsDiv.innerHTML = `
        <div class="stat-card"><div class="label">Records</div><div class="value">${n}</div></div>
        <div class="stat-card"><div class="label">Duration</div><div class="value">${durationStr}</div></div>
        <div class="stat-card"><div class="label">Sample rate</div><div class="value">${rateStr}</div></div>
        <div class="stat-card"><div class="label">Skipped bytes</div><div class="value">${skipped}</div></div>
        <div class="stat-card"><div class="label">Pressure range</div><div class="value">${pMin}–${pMax}</div></div>
        <div class="stat-card"><div class="label">Avg temp</div><div class="value">${tAvg} °C</div></div>
        <div class="stat-card"><div class="label">Alt range</div><div class="value">${altRange} m</div></div>
        <div class="stat-card"><div class="label">Battery</div><div class="value">${vMin}–${vMax} V</div></div>
        <div class="stat-card"><div class="label">Dropped samples</div><div class="value">${dropIndices.length}</div></div>
        <div class="stat-card"><div class="label">States</div><div class="value">${states}</div></div>
    `;
}
