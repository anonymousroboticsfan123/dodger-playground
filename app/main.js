import { installFailureReplay } from './failureReplay.js?v=dodger-3';

const unitreeG1Host = document.getElementById('unitreeG1Host');
const failureOverlay = document.getElementById('failureOverlay');
const failureTitle = document.getElementById('failureTitle');
const failureMessage = document.getElementById('failureMessage');

let unitreeG1Case = null;
let lastTime = performance.now();
let accumulator = 0;

function updateUi() {
    if (!unitreeG1Case) return;
    const snapshot = unitreeG1Case.getSnapshot();
    window.__lastSnapshot = snapshot;
    const showOverlay = !!snapshot.failure;
    failureOverlay.hidden = !showOverlay;
    document.body.classList.toggle('simulation-stopped', showOverlay);
    if (showOverlay) {
        failureTitle.textContent = snapshot.failure.reason === 'collision' ? 'Collision'
            : snapshot.failure.reason === 'fall' ? 'Robot fell'
            : snapshot.failure.reason === 'qp infeasible' ? 'QP infeasible' : 'Simulation stopped';
        failureMessage.textContent = snapshot.failure.details;
    }
}

function restartFromFailure() {
    unitreeG1Case?.resetFromFailure();
    accumulator = 0;
    lastTime = performance.now();
    updateUi();
}

installFailureReplay(document.querySelector('.canvas-stack'), failureOverlay, {
    hasFailure: () => Boolean(unitreeG1Case?.failure),
    replay: restartFromFailure
});

unitreeG1Host.addEventListener('contextmenu', (event) => event.preventDefault());
unitreeG1Host.addEventListener('pointerdown', (event) => {
    unitreeG1Case?.handlePointerDown(event);
});
unitreeG1Host.addEventListener('pointermove', (event) => {
    unitreeG1Case?.handlePointerMove(event);
});
unitreeG1Host.addEventListener('pointerup', (event) => {
    unitreeG1Case?.handlePointerUp(event);
});
unitreeG1Host.addEventListener('pointercancel', () => {
    unitreeG1Case?.handlePointerLeave();
});
unitreeG1Host.addEventListener('pointerleave', () => {
    unitreeG1Case?.handlePointerLeave();
});
unitreeG1Host.addEventListener('wheel', (event) => {
    event.preventDefault();
    unitreeG1Case?.handleWheel(event);
}, { passive: false });

window.__playgroundDebug = {
    getCurrentCaseId: () => 'unitree_g1',
    getCurrentCase: () => unitreeG1Case,
    getUnitreeG1Case: () => unitreeG1Case,
    getSnapshot: () => unitreeG1Case?.getSnapshot(),
    forceUpdateUi: updateUi,
    forceRender: () => {
        unitreeG1Case?.render();
        updateUi();
    }
};

async function frame(now) {
    try {
        const deltaSeconds = Math.min(0.1, (now - lastTime) / 1000);
        lastTime = now;
        accumulator += deltaSeconds;
        const playbackStep = unitreeG1Case.getPlaybackStep();
        while (accumulator >= playbackStep) {
            await unitreeG1Case.step(now);
            accumulator -= playbackStep;
        }
        unitreeG1Case.render();
        updateUi();
    } catch (error) {
        console.error('[frame]', error);
        accumulator = 0;
    }
    requestAnimationFrame(frame);
}

async function initialize() {
    try {
        const { DodgerCase } = await import('./cases/dodgerCase.js?v=dodger-5');
        unitreeG1Case = new DodgerCase(unitreeG1Host);
        unitreeG1Case.setActive(true);
        await unitreeG1Case.initialize();
        accumulator = 0;
        lastTime = performance.now();
        updateUi();
        requestAnimationFrame(frame);
    } catch (error) {
        console.error('[unitree-g1]', error);
        const panel = document.createElement('div');
        panel.className = 'unitree-g1-initial-loading error';
        panel.setAttribute('role', 'alert');
        const title = document.createElement('strong');
        title.textContent = 'Unitree G1 failed to load';
        const message = document.createElement('small');
        message.textContent = error.message || 'Unknown loading error.';
        const retry = document.createElement('button');
        retry.type = 'button';
        retry.className = 'unitree-g1-pause-btn';
        retry.textContent = 'Retry';
        retry.addEventListener('click', () => window.location.reload());
        panel.append(title, message, retry);
        unitreeG1Host.replaceChildren(panel);
    }
}

void initialize();
