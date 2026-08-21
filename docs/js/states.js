const STATE_NAMES = [
    'Idle',
    'Calibration',
    'Prelaunch',
    'Burn',
    'Passive Burnout',
    'Active Burnout',
    'Apogee',
    'Parachute',
    'Landed',
    'Ground Abort',
    'Descent Abort',
];

const STATE_COLORS = [
    '#7a7e8c', // Idle — dim gray
    '#5b9cf6', // Calibration — blue
    '#e8c840', // Prelaunch — yellow
    '#e89030', // Burn — orange
    '#d4882a', // Passive Burnout — dark orange
    '#c07020', // Active Burnout — brown-orange
    '#b07af5', // Apogee — purple
    '#4ec9c9', // Parachute — cyan
    '#4ecb71', // Landed — green
    '#e85454', // Ground Abort — red
    '#e85454', // Descent Abort — red
];

function getStateName(id) {
    return STATE_NAMES[id] || ('State ' + id);
}

function getStateColor(id) {
    return STATE_COLORS[id] || '#7a7e8c';
}

function drawStateTimeline(canvas) {
    const visible = getVisibleRecords();
    if (!visible.length || visible[0].State === undefined) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);

    const w = rect.width, h = rect.height;
    const pad = { top: 4, right: 12, bottom: 20, left: 60 };
    const plotW = w - pad.left - pad.right;
    const barH = h - pad.top - pad.bottom;

    const style = getComputedStyle(document.documentElement);
    const textDim = style.getPropertyValue('--text-dim').trim();

    const startIdx = Math.floor(viewStart * allRecords.length);
    const hasTick = visible.length > 1 && visible[0].Tick !== undefined;

    // Draw state segments
    let segStart = 0;
    let segState = visible[0].State;
    for (let i = 1; i <= visible.length; i++) {
        const curState = i < visible.length ? visible[i].State : -1;
        if (curState !== segState || i === visible.length) {
            const x0 = pad.left + (segStart / (visible.length - 1)) * plotW;
            const x1 = pad.left + ((i - 1) / (visible.length - 1)) * plotW;
            const segW = Math.max(x1 - x0, 1);

            ctx.fillStyle = getStateColor(segState);
            ctx.globalAlpha = 0.8;
            ctx.beginPath();
            ctx.roundRect(x0, pad.top, segW, barH, 3);
            ctx.fill();
            ctx.globalAlpha = 1;

            if (segW > 40) {
                ctx.fillStyle = '#fff';
                ctx.font = 'bold 11px -apple-system, BlinkMacSystemFont, sans-serif';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(getStateName(segState), x0 + segW / 2, pad.top + barH / 2);
            }

            segStart = i;
            segState = curState;
        }
    }

    // Time axis
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillStyle = textDim;
    for (let i = 0; i <= 6; i++) {
        const x = pad.left + (plotW / 6) * i;
        const sampleIdx = Math.round(((visible.length - 1) / 6) * i);
        let label;
        if (hasTick) {
            const ms = visible[sampleIdx].Tick - allRecords[0].Tick;
            label = (ms / 1000).toFixed(1) + 's';
        } else {
            label = (startIdx + sampleIdx).toString();
        }
        ctx.fillText(label, x, h - pad.bottom + 4);
    }
}
