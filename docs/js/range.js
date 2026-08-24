function updateRangeUI() {
    const track = document.getElementById('rangeTrack');
    const tw = track.getBoundingClientRect().width;
    const sel = document.getElementById('rangeSelected');
    const hl = document.getElementById('handleLeft');
    const hr = document.getElementById('handleRight');

    sel.style.left = (viewStart * 100) + '%';
    sel.style.width = ((viewEnd - viewStart) * 100) + '%';
    hl.style.left = (viewStart * tw - 4) + 'px';
    hr.style.left = (viewEnd * tw - 4) + 'px';

    const startIdx = Math.floor(viewStart * allRecords.length);
    const endIdx = Math.floor(viewEnd * allRecords.length);
    document.getElementById('rangeInfo').textContent = `${startIdx} – ${endIdx} (${endIdx - startIdx} samples)`;
}

function initRangeControl() {
    const ctrl = document.getElementById('rangeControl');
    ctrl.classList.remove('hidden');

    const track = document.getElementById('rangeTrack');
    const hl = document.getElementById('handleLeft');
    const hr = document.getElementById('handleRight');
    const sel = document.getElementById('rangeSelected');

    let dragging = null;
    let dragStartX = 0, dragStartLeft = 0, dragStartRight = 0;

    function getPos(e) {
        const rect = track.getBoundingClientRect();
        const x = e.touches ? e.touches[0].clientX : e.clientX;
        return Math.max(0, Math.min(1, (x - rect.left) / rect.width));
    }

    function getClientX(e) {
        return e.touches ? e.touches[0].clientX : e.clientX;
    }

    function onStart(e, type) {
        e.preventDefault();
        dragging = type;
        if (type === 'left') hl.classList.add('active');
        if (type === 'right') hr.classList.add('active');
        if (type === 'middle') {
            dragStartX = getClientX(e);
            dragStartLeft = viewStart;
            dragStartRight = viewEnd;
        }
    }

    hl.addEventListener('mousedown', (e) => onStart(e, 'left'));
    hr.addEventListener('mousedown', (e) => onStart(e, 'right'));
    sel.addEventListener('mousedown', (e) => onStart(e, 'middle'));
    hl.addEventListener('touchstart', (e) => onStart(e, 'left'), { passive: false });
    hr.addEventListener('touchstart', (e) => onStart(e, 'right'), { passive: false });
    sel.addEventListener('touchstart', (e) => onStart(e, 'middle'), { passive: false });

    track.addEventListener('mousedown', (e) => {
        if (e.target === track || e.target.tagName === 'CANVAS') {
            const pos = getPos(e);
            const halfSpan = (viewEnd - viewStart) / 2;
            viewStart = Math.max(0, pos - halfSpan);
            viewEnd = Math.min(1, pos + halfSpan);
            if (viewStart < 0) { viewEnd -= viewStart; viewStart = 0; }
            if (viewEnd > 1) { viewStart -= (viewEnd - 1); viewEnd = 1; }
            updateRangeUI(); redrawAllCharts();
        }
    });

    function onMove(e) {
        if (!dragging) return;
        e.preventDefault();
        const pos = getPos(e);
        if (dragging === 'left') {
            viewStart = Math.min(pos, viewEnd - 0.01);
        } else if (dragging === 'right') {
            viewEnd = Math.max(pos, viewStart + 0.01);
        } else if (dragging === 'middle') {
            const rect = track.getBoundingClientRect();
            const dx = (getClientX(e) - dragStartX) / rect.width;
            const span = dragStartRight - dragStartLeft;
            let newStart = dragStartLeft + dx;
            let newEnd = dragStartRight + dx;
            if (newStart < 0) { newStart = 0; newEnd = span; }
            if (newEnd > 1) { newEnd = 1; newStart = 1 - span; }
            viewStart = newStart; viewEnd = newEnd;
        }
        updateRangeUI(); redrawAllCharts();
    }

    function onEnd() {
        dragging = null;
        hl.classList.remove('active'); hr.classList.remove('active');
    }

    document.addEventListener('mousemove', onMove);
    document.addEventListener('touchmove', onMove, { passive: false });
    document.addEventListener('mouseup', onEnd);
    document.addEventListener('touchend', onEnd);

    drawMinimap();
    updateRangeUI();
}
