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

const COMMAND_NAMES = {
    0x00: 'None',
    0x01: 'Reset',
    0x02: 'Ground Abort',
    0x03: 'Calibration',
    0x04: 'Drogue',
    0x10: 'HIL Data',
    0x20: 'GPS Data',
};

const COMMAND_COLORS = {
    0x00: '#7a7e8c',
    0x01: '#e85454',
    0x02: '#e85454',
    0x03: '#5b9cf6',
    0x04: '#b07af5',
    0x10: '#e8c840',
    0x20: '#4ecb71',
};

function getCommandName(id) {
    return COMMAND_NAMES[id] || ('Cmd 0x' + id.toString(16).toUpperCase());
}

function getCommandColor(id) {
    return COMMAND_COLORS[id] || '#7a7e8c';
}

function drawCommandMarkers(canvas, pad, plotW) {
    const visible = getVisibleRecords();
    if (!visible.length || visible[0].LastCommand === undefined) return;

    const dpr = window.devicePixelRatio || 1;
    const ctx = canvas.getContext('2d');
    ctx.save();
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    for (let i = 0; i < visible.length; i++) {
        const cmd = visible[i].LastCommand;
        if (cmd === 0x00) continue;

        const x = pad.left + (i / (visible.length - 1)) * plotW;
        const color = getCommandColor(cmd);

        const rect = canvas.getBoundingClientRect();
        const fullH = rect.height;

        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.moveTo(x, 14);
        ctx.lineTo(x, fullH - 20);
        ctx.stroke();
        ctx.setLineDash([]);

        const label = getCommandName(cmd);
        ctx.font = 'bold 9px -apple-system, BlinkMacSystemFont, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';

        const textW = ctx.measureText(label).width + 6;
        ctx.fillStyle = color;
        ctx.globalAlpha = 0.9;
        ctx.beginPath();
        ctx.roundRect(x - textW / 2, 0, textW, 14, 2);
        ctx.fill();
        ctx.globalAlpha = 1;

        ctx.fillStyle = '#fff';
        ctx.fillText(label, x, 13);
    }

    ctx.restore();
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
    const pad = { top: 18, right: 12, bottom: 20, left: 60 };
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
