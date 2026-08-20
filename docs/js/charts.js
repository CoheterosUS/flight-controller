const chartMeta = new Map();

function getVisibleRecords() {
    const start = Math.floor(viewStart * allRecords.length);
    const end = Math.floor(viewEnd * allRecords.length);
    return allRecords.slice(start, Math.max(end, start + 2));
}

function drawChart(canvas, datasets) {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);

    const w = rect.width, h = rect.height;
    const pad = { top: 10, right: 12, bottom: 24, left: 60 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    let allVals = [], maxLen = 0;
    for (const ds of datasets) { allVals = allVals.concat(ds.data); maxLen = Math.max(maxLen, ds.data.length); }

    let yMin = arrMin(allVals), yMax = arrMax(allVals);
    if (yMin === yMax) { yMin -= 1; yMax += 1; }
    const yPad = (yMax - yMin) * 0.05;
    yMin -= yPad; yMax += yPad;

    const style = getComputedStyle(document.documentElement);
    const gridColor = style.getPropertyValue('--chart-grid').trim();
    const gridStrong = style.getPropertyValue('--chart-grid-strong').trim();
    const textDim = style.getPropertyValue('--text-dim').trim();

    ctx.font = '10px monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= 5; i++) {
        const y = pad.top + (plotH / 5) * i;
        const val = yMax - ((yMax - yMin) / 5) * i;
        ctx.strokeStyle = i === 0 || i === 5 ? gridStrong : gridColor;
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(w - pad.right, y); ctx.stroke();
        ctx.fillStyle = textDim;
        ctx.fillText(formatNum(val), pad.left - 6, y);
    }

    const startIdx = Math.floor(viewStart * allRecords.length);
    const endIdx = Math.floor(viewEnd * allRecords.length);
    const visRecs = allRecords.slice(startIdx, Math.max(endIdx, startIdx + 2));
    const hasTick = visRecs.length > 1 && visRecs[0].Tick !== undefined;
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (let i = 0; i <= 6; i++) {
        const x = pad.left + (plotW / 6) * i;
        const sampleIdx = Math.round(((visRecs.length - 1) / 6) * i);
        let label;
        if (hasTick) {
            const ms = visRecs[sampleIdx].Tick - allRecords[0].Tick;
            label = (ms / 1000).toFixed(1) + 's';
        } else {
            label = (startIdx + sampleIdx).toString();
        }
        ctx.fillStyle = textDim;
        ctx.fillText(label, x, h - pad.bottom + 6);
    }

    const dropLocalSet = new Set();
    const dropRegions = [];
    if (dropIndices.length > 0) {
        for (const di of dropIndices) {
            const li = di - startIdx;
            if (li < 1 || li >= maxLen) continue;
            dropLocalSet.add(li);
            const x0 = pad.left + ((li - 1) / (maxLen - 1)) * plotW;
            const x1 = pad.left + (li / (maxLen - 1)) * plotW;
            dropRegions.push({ x0, x1: Math.max(x1, x0 + 4), di });
        }
    }

    if (dropRegions.length > 0) {
        ctx.save();
        ctx.beginPath();
        ctx.rect(pad.left, pad.top, plotW, plotH);
        for (const r of dropRegions) {
            ctx.rect(r.x1, pad.top, 0, plotH);
            ctx.rect(r.x0, pad.top, -(r.x0 - pad.left), plotH);
        }
        const outerPath = new Path2D();
        outerPath.rect(pad.left, pad.top, plotW, plotH);
        for (const r of dropRegions) outerPath.rect(r.x0, pad.top, r.x1 - r.x0, plotH);
        ctx.clip(outerPath, 'evenodd');
    }

    for (const ds of datasets) {
        ctx.strokeStyle = ds.color;
        ctx.lineWidth = 1.2;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        for (let i = 0; i < ds.data.length; i++) {
            const x = pad.left + (i / (maxLen - 1)) * plotW;
            const y = pad.top + (1 - (ds.data[i] - yMin) / (yMax - yMin)) * plotH;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }

    if (dropRegions.length > 0) {
        ctx.restore();
        const redColor = style.getPropertyValue('--red').trim();
        for (const r of dropRegions) {
            const rw = r.x1 - r.x0;
            ctx.save();
            ctx.globalAlpha = 0.18;
            ctx.fillStyle = redColor;
            ctx.fillRect(r.x0, pad.top, rw, plotH);
            ctx.globalAlpha = 0.45;
            ctx.strokeStyle = redColor;
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.rect(r.x0, pad.top, rw, plotH);
            ctx.clip();
            const spacing = 6;
            for (let ly = pad.top; ly < pad.top + plotH + rw; ly += spacing) {
                ctx.moveTo(r.x0, ly);
                ctx.lineTo(r.x0 + rw, ly - rw);
            }
            ctx.stroke();
            ctx.restore();
        }
    }

    chartMeta.set(canvas, { datasets, pad, plotW, plotH, yMin, yMax, maxLen, w: rect.width, h: rect.height, dropRegions });
}

function drawCrosshairOnChart(canvas, idx) {
    const meta = chartMeta.get(canvas);
    if (!meta || idx < 0 || idx >= meta.maxLen) return;
    const dpr = window.devicePixelRatio || 1;
    const ctx = canvas.getContext('2d');
    ctx.save();
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const cx = meta.pad.left + (idx / (meta.maxLen - 1)) * meta.plotW;
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--text-dim').trim();
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(cx, meta.pad.top); ctx.lineTo(cx, meta.pad.top + meta.plotH); ctx.stroke();
    ctx.setLineDash([]);
    for (const ds of meta.datasets) {
        const val = ds.data[idx];
        if (val === undefined) continue;
        const cy = meta.pad.top + (1 - (val - meta.yMin) / (meta.yMax - meta.yMin)) * meta.plotH;
        ctx.fillStyle = ds.color;
        ctx.beginPath(); ctx.arc(cx, cy, 3.5, 0, Math.PI * 2); ctx.fill();
    }
    ctx.restore();
}

function showTooltipOnChart(canvas, idx, frac) {
    const meta = chartMeta.get(canvas);
    const tooltip = canvas.parentElement.querySelector('.chart-tooltip');
    if (!meta || !tooltip || idx < 0 || idx >= meta.maxLen) {
        if (tooltip) tooltip.style.opacity = '0';
        return;
    }

    const cx = meta.pad.left + frac * meta.plotW;
    let inDrop = null;
    if (meta.dropRegions) {
        for (const r of meta.dropRegions) {
            if (cx >= r.x0 && cx <= r.x1) { inDrop = r; break; }
        }
    }

    const startIdx = Math.floor(viewStart * allRecords.length);
    let rows;
    if (inDrop) {
        const prev = allRecords[inDrop.di - 1];
        const curr = allRecords[inDrop.di];
        const gap = curr.Tick - prev.Tick;
        const medianDelta = getMedianDelta();
        const missed = Math.round(gap / medianDelta) - 1;
        rows = '<div class="tt-drop">DROPPED ' + missed + ' sample' + (missed !== 1 ? 's' : '') + ' · gap ' + gap + ' ms</div>';
    } else {
        const rec = allRecords[startIdx + idx];
        const tickStr = rec && rec.Tick !== undefined ? ' · ' + ((rec.Tick - allRecords[0].Tick) / 1000).toFixed(2) + 's' : '';
        rows = '<div class="tt-sample">#' + (startIdx + idx) + tickStr + '</div>';
        const labels = meta.datasets.length === 3 ? ['X', 'Y', 'Z'] : null;
        for (let d = 0; d < meta.datasets.length; d++) {
            const ds = meta.datasets[d];
            const val = ds.data[idx];
            if (val === undefined) continue;
            const label = labels ? labels[d] : '';
            rows += `<div class="tt-row"><span class="tt-dot" style="background:${ds.color}"></span>${label ? label + ': ' : ''}${formatNum(val)}</div>`;
        }
    }
    tooltip.innerHTML = rows;
    tooltip.style.opacity = '1';
    let tx = cx + 12;
    const tw = tooltip.offsetWidth;
    if (tx + tw > meta.w - 4) tx = cx - tw - 12;
    let ty = meta.pad.top + 4;
    tooltip.style.left = tx + 'px';
    tooltip.style.top = ty + 'px';
}

function syncCrosshair(frac) {
    const allCanvases = document.querySelectorAll('#charts canvas[data-key]');
    for (const c of allCanvases) {
        const m = chartMeta.get(c);
        if (!m) continue;
        const idx = Math.round(frac * (m.maxLen - 1));
        drawChart(c, m.datasets);

        const cx = m.pad.left + frac * m.plotW;
        let inDrop = false;
        if (m.dropRegions) {
            for (const r of m.dropRegions) {
                if (cx >= r.x0 && cx <= r.x1) { inDrop = true; break; }
            }
        }
        if (!inDrop) drawCrosshairOnChart(c, idx);
        showTooltipOnChart(c, idx, frac);
    }
}

function clearCrosshair() {
    const allCanvases = document.querySelectorAll('#charts canvas[data-key]');
    for (const c of allCanvases) {
        const m = chartMeta.get(c);
        if (m) drawChart(c, m.datasets);
        const tt = c.parentElement.querySelector('.chart-tooltip');
        if (tt) tt.style.opacity = '0';
    }
}

function drawMinimap() {
    const canvas = document.getElementById('minimapCanvas');
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    const w = rect.width, h = rect.height;

    const accelZ = getCol(allRecords, 'AccelZ');
    if (!accelZ.length) return;
    let yMin = arrMin(accelZ), yMax = arrMax(accelZ);
    if (yMin === yMax) { yMin -= 1; yMax += 1; }

    const style = getComputedStyle(document.documentElement);
    ctx.strokeStyle = style.getPropertyValue('--text-dim').trim();
    ctx.lineWidth = 0.8;
    ctx.globalAlpha = 0.4;
    ctx.beginPath();
    for (let i = 0; i < accelZ.length; i++) {
        const x = (i / (accelZ.length - 1)) * w;
        const y = (1 - (accelZ[i] - yMin) / (yMax - yMin)) * h;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
}

function redrawAllCharts() {
    const visible = getVisibleRecords();
    const style = getComputedStyle(document.documentElement);
    const red = style.getPropertyValue('--red').trim();
    const green = style.getPropertyValue('--green').trim();
    const blue = style.getPropertyValue('--blue').trim();
    const accent = style.getPropertyValue('--accent').trim();
    const purple = style.getPropertyValue('--purple').trim();
    const cyan = style.getPropertyValue('--cyan').trim();
    const yellow = style.getPropertyValue('--yellow').trim();

    const chartDefs = [
        { key: 'accel', datasets: [
            { data: getCol(visible, 'AccelX'), color: red },
            { data: getCol(visible, 'AccelY'), color: green },
            { data: getCol(visible, 'AccelZ'), color: blue },
        ]},
        { key: 'gyro', datasets: [
            { data: getCol(visible, 'GyroX'), color: red },
            { data: getCol(visible, 'GyroY'), color: green },
            { data: getCol(visible, 'GyroZ'), color: blue },
        ]},
        { key: 'mag', datasets: [
            { data: getCol(visible, 'MagX'), color: red },
            { data: getCol(visible, 'MagY'), color: green },
            { data: getCol(visible, 'MagZ'), color: blue },
        ]},
        { key: 'pressure', datasets: [{ data: getCol(visible, 'PressurePa'), color: accent }] },
        { key: 'temp', datasets: [{ data: getCol(visible, 'TemperatureC'), color: cyan }] },
        { key: 'alt', datasets: [{ data: getCol(visible, 'GPSAltitude'), color: purple }] },
        { key: 'voltage', datasets: [{ data: getCol(visible, 'BatteryVoltage'), color: yellow }] },
        { key: 'relay', datasets: [
            { data: visible.map(r => (r.RelayState & 0x01) ? 1 : 0), color: red },
            { data: visible.map(r => (r.RelayState & 0x02) ? 1 : 0), color: blue },
        ]},
    ];

    for (const def of chartDefs) {
        const canvas = document.querySelector(`canvas[data-key="${def.key}"]`);
        if (canvas) drawChart(canvas, def.datasets);
    }

    const stateCanvas = document.querySelector('canvas[data-key="state"]');
    if (stateCanvas) drawStateTimeline(stateCanvas);
}

function renderCharts() {
    const chartsDiv = document.getElementById('charts');
    chartsDiv.innerHTML = '';
    chartsDiv.classList.remove('hidden');

    const chartDefs = [
        { key: 'accel', title: 'Accelerometer', legend: [['X','legend-x'],['Y','legend-y'],['Z','legend-z']] },
        { key: 'gyro', title: 'Gyroscope', legend: [['X','legend-x'],['Y','legend-y'],['Z','legend-z']] },
        { key: 'mag', title: 'Magnetometer', legend: [['X','legend-x'],['Y','legend-y'],['Z','legend-z']] },
        { key: 'pressure', title: 'Pressure' },
        { key: 'temp', title: 'Temperature' },
        { key: 'alt', title: 'GPS Altitude' },
        { key: 'voltage', title: 'Battery Voltage' },
        { key: 'relay', title: 'Relay State', legend: [['Drogue','legend-x'],['Parachute','legend-z']] },
    ];

    if (allRecords[0] && allRecords[0].State !== undefined) {
        const stateContainer = document.createElement('div');
        stateContainer.className = 'chart-container';
        stateContainer.innerHTML = '<div class="chart-title">State Timeline</div>';
        const stateCanvas = document.createElement('canvas');
        stateCanvas.dataset.key = 'state';
        stateCanvas.style.height = '60px';
        stateContainer.appendChild(stateCanvas);
        chartsDiv.appendChild(stateContainer);
    }

    for (const def of chartDefs) {
        const container = document.createElement('div');
        container.className = 'chart-container';
        let legendHTML = '';
        if (def.legend) {
            legendHTML = '<div class="legend">' +
                def.legend.map(([l, cls]) => `<span class="${cls}">${l}</span>`).join('') + '</div>';
        }
        container.innerHTML = `<div class="chart-title">${def.title}${legendHTML}</div>`;
        const canvas = document.createElement('canvas');
        canvas.dataset.key = def.key;
        container.appendChild(canvas);

        const tooltip = document.createElement('div');
        tooltip.className = 'chart-tooltip';
        tooltip.style.opacity = '0';
        container.appendChild(tooltip);

        canvas.addEventListener('mousemove', (e) => {
            const meta = chartMeta.get(canvas);
            if (!meta) return;
            const rect = canvas.getBoundingClientRect();
            const mx = e.clientX - rect.left;
            if (mx < meta.pad.left || mx > meta.w - meta.pad.right) { clearCrosshair(); return; }
            const frac = (mx - meta.pad.left) / meta.plotW;
            if (frac < 0 || frac > 1) { clearCrosshair(); return; }
            syncCrosshair(frac);
        });

        canvas.addEventListener('mouseleave', () => {
            clearCrosshair();
        });

        chartsDiv.appendChild(container);
    }

    requestAnimationFrame(redrawAllCharts);
}
