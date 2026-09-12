import {
    G1_COLLISION_FOOTPRINTS,
    G1_OBSTACLE_DEFAULTS,
    getG1ObstacleCollisionRadius,
    getObstacleLaunchVelocity
} from './obstacles.js?v=dodger-3';
import { getActiveDpcbfConstraintIds } from './dpcbfVisualState.js?v=dodger-3';

export { getActiveDpcbfConstraintIds } from './dpcbfVisualState.js?v=dodger-3';

const MINIMAP_SIZE = 224;
const MAP_HALF_SPAN = 4.5;
const MAP_TRIGGER_RATIO = 0.62;
const MAP_REARM_RATIO = 0.38;
const MAP_SLIDE_OMEGA = 8;
const DPCBF_PALETTE = Object.freeze([
    '#f97316',
    '#38bdf8',
    '#22c55e',
    '#f59e0b',
    '#fb7185',
    '#a78bfa',
    '#2dd4bf',
    '#facc15'
]);
const ACTIVE_CONSTRAINT_COLOR = '#ef4444';

export function getG1DpcbfOverlayColor(index) {
    return DPCBF_PALETTE[index % DPCBF_PALETTE.length];
}

function formatSigned(value, digits = 2) {
    if (!Number.isFinite(value)) return '—';
    return `${value >= 0 ? '+' : ''}${value.toFixed(digits)}`;
}

export class UnitreeG1InteractionOverlay {
    constructor(hostEl, callbacks = {}) {
        this.hostEl = hostEl;
        this.callbacks = callbacks;
        this.mode = 'goal';
        this.drag = null;
        this.preview = null;
        this.dpcbfEnabled = true;
        this.size = MINIMAP_SIZE;
        this.mapCenter = [0, 0];
        this.mapTarget = [0, 0];
        this.mapVelocity = [0, 0];
        this.mapArmed = true;
        this.mapOutsideSince = null;
        this.mapInsideSince = null;
        this.mapLastUpdateMs = null;

        this.root = document.createElement('section');
        this.root.className = 'unitree-g1-minimap-panel';
        this.root.dataset.g1Interactive = '';
        this.root.setAttribute('aria-label', 'BEV map and obstacle tools');
        this.root.innerHTML = `
            <div class="unitree-g1-minimap-head">
                <span><strong>BEV MAP</strong><small data-gat-key hidden title="Connections used by the graph encoder">GAT edges</small></span>
                <button type="button" class="unitree-g1-dpcbf-toggle active"
                    data-g1-dpcbf aria-pressed="true">DPCBF ON</button>
            </div>
            <canvas class="unitree-g1-minimap" width="${MINIMAP_SIZE}"
                height="${MINIMAP_SIZE}" role="img"
                aria-label="BEV map for goals, slow moving obstacles, and DPCBF diagnostics."></canvas>
            <div class="unitree-g1-command-key" aria-label="Joystick command legend">
                <span><i class="nominal"></i><span data-command-label>Nominal</span></span>
                <span><i class="filtered"></i><span data-reference-label>Filtered</span></span>
            </div>
            <div class="unitree-g1-map-tools" role="group" aria-label="Minimap edit mode">
                <button type="button" class="active" data-g1-map-mode="goal"
                    aria-pressed="true">Goal</button>
                <button type="button" data-g1-map-mode="add"
                    aria-pressed="false">Add</button>
                <button type="button" data-g1-map-mode="remove"
                    aria-pressed="false">Remove</button>
                <button type="button" data-g1-reset-scene>Reset</button>
            </div>
            <div class="unitree-g1-safety-readout" aria-live="polite">
                <span>clearance <strong data-g1-clearance>—</strong></span>
                <span>barrier <strong data-g1-barrier>—</strong></span>
                <span>filter <strong data-g1-filter-state>nominal</strong></span>
            </div>
            <div class="unitree-g1-filter-params" data-g1-filter-params>
                α — · kλ — · kμ — · scale —
            </div>
        `;
        this.hostEl.appendChild(this.root);

        this.canvas = this.root.querySelector('canvas');
        this.ctx = this.canvas.getContext('2d');
        this.dpcbfButton = this.root.querySelector('[data-g1-dpcbf]');
        this.clearanceText = this.root.querySelector('[data-g1-clearance]');
        this.barrierText = this.root.querySelector('[data-g1-barrier]');
        this.filterStateText = this.root.querySelector('[data-g1-filter-state]');
        this.filterParamsText = this.root.querySelector('[data-g1-filter-params]');
        this.modeButtons = [...this.root.querySelectorAll('[data-g1-map-mode]')];

        this.dpcbfButton.addEventListener('click', () => {
            this.setDpcbfEnabled(!this.dpcbfEnabled);
            this.callbacks.onDpcbfToggle?.(this.dpcbfEnabled);
        });
        for (const button of this.modeButtons) {
            button.addEventListener('click', () => this.setMode(button.dataset.g1MapMode));
        }
        this.root.querySelector('[data-g1-reset-scene]').addEventListener('click', () => {
            this.callbacks.onReset?.();
        });

        this.canvas.addEventListener('pointerdown', (event) => this._pointerDown(event));
        this.canvas.addEventListener('pointermove', (event) => this._pointerMove(event));
        this.canvas.addEventListener('pointerup', (event) => this._pointerUp(event));
        this.canvas.addEventListener('pointercancel', (event) => this._cancelDrag(event));
        this.canvas.addEventListener('lostpointercapture', () => this.cancelDrag());
        for (const type of ['pointerdown', 'pointermove', 'pointerup', 'wheel', 'contextmenu']) {
            this.root.addEventListener(type, (event) => {
                event.stopPropagation();
                if (type === 'contextmenu') event.preventDefault();
            });
        }
    }

    setMode(mode) {
        if (!['goal', 'add', 'remove'].includes(mode)) return;
        this.cancelDrag();
        this.mode = mode;
        for (const button of this.modeButtons) {
            const active = button.dataset.g1MapMode === mode;
            button.classList.toggle('active', active);
            button.setAttribute('aria-pressed', String(active));
        }
    }

    setDpcbfEnabled(enabled) {
        this.dpcbfEnabled = Boolean(enabled);
        this.dpcbfButton.classList.toggle('active', this.dpcbfEnabled);
        this.dpcbfButton.classList.toggle('off', !this.dpcbfEnabled);
        this.dpcbfButton.textContent = this.dpcbfEnabled ? 'DPCBF ON' : 'DPCBF OFF';
        this.dpcbfButton.setAttribute('aria-pressed', String(this.dpcbfEnabled));
    }

    _pointFromEvent(event) {
        const rect = this.canvas.getBoundingClientRect();
        const px = Math.max(0, Math.min(rect.width, event.clientX - rect.left));
        const py = Math.max(0, Math.min(rect.height, event.clientY - rect.top));
        const center = this.drag?.mapCenter ?? this.mapCenter;
        return [
            center[0] + (px / rect.width * 2 - 1) * MAP_HALF_SPAN,
            center[1] + (1 - py / rect.height * 2) * MAP_HALF_SPAN
        ];
    }

    _pointerDown(event) {
        if (event.button !== 0) return;
        event.preventDefault();
        event.stopPropagation();
        const point = this._pointFromEvent(event);
        this.drag = {
            pointerId: event.pointerId,
            start: point,
            current: point,
            moved: false,
            mapCenter: [...this.mapCenter]
        };
        this.canvas.setPointerCapture?.(event.pointerId);
        if (this.mode === 'add') {
            this.preview = {
                position: point,
                velocity: [0, 0]
            };
        }
    }

    _pointerMove(event) {
        if (!this.drag || this.drag.pointerId !== event.pointerId) return;
        event.preventDefault();
        event.stopPropagation();
        const current = this._pointFromEvent(event);
        this.drag.current = current;
        this.drag.moved = this.drag.moved
            || Math.hypot(
                current[0] - this.drag.start[0],
                current[1] - this.drag.start[1]
            ) > 0.08;
        if (this.mode === 'add') {
            this.preview = {
                position: this.drag.start,
                velocity: getObstacleLaunchVelocity(this.drag.start, current)
            };
        }
    }

    _pointerUp(event) {
        if (!this.drag || this.drag.pointerId !== event.pointerId) return;
        event.preventDefault();
        event.stopPropagation();
        const current = this._pointFromEvent(event);
        const start = this.drag.start;
        this.cancelDrag();
        if (this.mode === 'goal') {
            this.callbacks.onGoal?.(current);
        } else if (this.mode === 'add') {
            this.callbacks.onAddObstacle?.({
                position: start,
                velocity: getObstacleLaunchVelocity(start, current)
            });
        } else if (this.mode === 'remove') {
            this.callbacks.onRemoveObstacle?.(current);
        }
    }

    _cancelDrag(event) {
        if (this.drag?.pointerId === event.pointerId) this.cancelDrag();
    }

    cancelDrag() {
        const pointerId = this.drag?.pointerId;
        this.drag = null;
        this.preview = null;
        if (pointerId != null && this.canvas.hasPointerCapture?.(pointerId)) this.canvas.releasePointerCapture(pointerId);
    }

    _resizeCanvas() {
        const cssSize = Math.max(1, Math.round(this.canvas.clientWidth || MINIMAP_SIZE));
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const target = Math.round(cssSize * dpr);
        if (this.canvas.width !== target || this.canvas.height !== target) {
            this.canvas.width = target;
            this.canvas.height = target;
        }
        this.size = cssSize;
        this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    _toCanvas(point) {
        return [
            ((point[0] - this.mapCenter[0]) / (MAP_HALF_SPAN * 2) + 0.5) * this.size,
            (0.5 - (point[1] - this.mapCenter[1]) / (MAP_HALF_SPAN * 2)) * this.size
        ];
    }

    _drawScreenArrow(
        ctx,
        origin,
        end,
        color,
        width = 1.5,
        dashed = false,
        opacity = 1
    ) {
        const length = Math.hypot(end[0] - origin[0], end[1] - origin[1]);
        if (length < 1e-4) return;
        const angle = Math.atan2(end[1] - origin[1], end[0] - origin[0]);
        const headLength = Math.min(7, Math.max(4, length * 0.32));
        ctx.save();
        ctx.globalAlpha = opacity;
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.lineWidth = width;
        ctx.lineCap = 'round';
        if (dashed) ctx.setLineDash([5, 3]);
        ctx.beginPath();
        ctx.moveTo(origin[0], origin[1]);
        ctx.lineTo(end[0], end[1]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.moveTo(end[0], end[1]);
        ctx.lineTo(
            end[0] - headLength * Math.cos(angle - Math.PI / 8),
            end[1] - headLength * Math.sin(angle - Math.PI / 8)
        );
        ctx.lineTo(
            end[0] - headLength * Math.cos(angle + Math.PI / 8),
            end[1] - headLength * Math.sin(angle + Math.PI / 8)
        );
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }

    _drawWorldArrow(
        ctx,
        start,
        end,
        color,
        width = 1.5,
        dashed = false,
        opacity = 1
    ) {
        this._drawScreenArrow(
            ctx,
            this._toCanvas(start),
            this._toCanvas(end),
            color,
            width,
            dashed,
            opacity
        );
    }

    _drawVelocityArrow(ctx, origin, velocity, color) {
        const speed = Math.hypot(velocity[0], velocity[1]);
        if (speed < 1e-4) return;
        const visualScale = 28 / G1_OBSTACLE_DEFAULTS.speedCap;
        this._drawScreenArrow(ctx, origin, [
            origin[0] + velocity[0] * visualScale,
            origin[1] - velocity[1] * visualScale
        ], color, 1.5);
    }

    _drawPath(ctx, points, color, width = 1.4, dashed = true, opacity = 1) {
        if (!Array.isArray(points) || points.length < 2) return;
        ctx.save();
        ctx.globalAlpha = opacity;
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        if (dashed) ctx.setLineDash([4, 4]);
        ctx.beginPath();
        for (let index = 0; index < points.length; index += 1) {
            const point = Array.isArray(points[index])
                ? points[index]
                : [points[index].x, points[index].y];
            const [x, y] = this._toCanvas(point);
            if (index === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.restore();
    }

    _updateMapCenter(robotPosition) {
        if (this.drag) return;
        const now = performance.now();
        const dt = this.mapLastUpdateMs == null
            ? 0
            : Math.min(0.05, Math.max(0, (now - this.mapLastUpdateMs) / 1000));
        this.mapLastUpdateMs = now;
        const normalizedOffset = Math.max(
            Math.abs(robotPosition[0] - this.mapTarget[0]),
            Math.abs(robotPosition[1] - this.mapTarget[1])
        ) / MAP_HALF_SPAN;

        if (this.mapArmed && normalizedOffset > MAP_TRIGGER_RATIO) {
            this.mapOutsideSince ??= now;
            if (now - this.mapOutsideSince >= 120) {
                this.mapTarget = [robotPosition[0], robotPosition[1]];
                this.mapArmed = false;
                this.mapOutsideSince = null;
            }
        } else {
            this.mapOutsideSince = null;
        }

        const insideTarget = Math.max(
            Math.abs(robotPosition[0] - this.mapTarget[0]),
            Math.abs(robotPosition[1] - this.mapTarget[1])
        ) / MAP_HALF_SPAN < MAP_REARM_RATIO;
        if (!this.mapArmed && insideTarget) {
            this.mapInsideSince ??= now;
            if (now - this.mapInsideSince >= 250) {
                this.mapArmed = true;
                this.mapInsideSince = null;
            }
        } else if (!insideTarget) {
            this.mapInsideSince = null;
        }

        if (dt <= 0) return;
        for (let axis = 0; axis < 2; axis += 1) {
            const displacement = this.mapCenter[axis] - this.mapTarget[axis];
            const spring = this.mapVelocity[axis] + MAP_SLIDE_OMEGA * displacement;
            const decay = Math.exp(-MAP_SLIDE_OMEGA * dt);
            this.mapCenter[axis] = this.mapTarget[axis]
                + (displacement + spring * dt) * decay;
            this.mapVelocity[axis] = (
                this.mapVelocity[axis] - MAP_SLIDE_OMEGA * spring * dt
            ) * decay;
            if (
                Math.abs(this.mapCenter[axis] - this.mapTarget[axis]) < 0.01
                && Math.abs(this.mapVelocity[axis]) < 0.02
            ) {
                this.mapCenter[axis] = this.mapTarget[axis];
                this.mapVelocity[axis] = 0;
            }
        }
    }

    _drawCommandArrow(ctx, robot, command, color, offset, dashed) {
        const yaw = robot.yaw;
        const speed = command?.[0] ?? 0;
        const lateral = command?.[1] ?? 0;
        const yawRate = command?.[2] ?? 0;
        const scale = this.size / (MAP_HALF_SPAN * 2);
        const center = this._toCanvas(robot.position);
        const start = [
            center[0] - Math.sin(yaw) * offset,
            center[1] - Math.cos(yaw) * offset
        ];
        const direction = yaw + yawRate * 0.7 + Math.atan2(lateral, speed);
        const worldLength = 0.32 + Math.hypot(speed, lateral) * 1.55 + Math.abs(yawRate) * 0.12;
        const end = [
            start[0] + Math.cos(direction) * worldLength * scale,
            start[1] - Math.sin(direction) * worldLength * scale
        ];
        this._drawScreenArrow(ctx, start, end, color, dashed ? 2.4 : 3.1, dashed);
    }

    render(snapshot) {
        this._resizeCanvas();
        this._updateMapCenter(snapshot.robot.position);
        const ctx = this.ctx;
        const size = this.size;
        const diagnostics = snapshot.dpcbf ?? {};
        const activeConstraintIds = getActiveDpcbfConstraintIds(
            diagnostics,
            this.dpcbfEnabled
        );
        const scale = size / (MAP_HALF_SPAN * 2);
        ctx.clearRect(0, 0, size, size);

        const gradient = ctx.createLinearGradient(0, 0, 0, size);
        gradient.addColorStop(0, 'rgba(10, 24, 39, 0.97)');
        gradient.addColorStop(1, 'rgba(4, 11, 21, 0.97)');
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, size, size);

        ctx.strokeStyle = 'rgba(125, 211, 252, 0.10)';
        ctx.lineWidth = 1;
        const gridStartX = Math.ceil((this.mapCenter[0] - MAP_HALF_SPAN) / 2) * 2;
        const gridEndX = this.mapCenter[0] + MAP_HALF_SPAN;
        const gridStartY = Math.ceil((this.mapCenter[1] - MAP_HALF_SPAN) / 2) * 2;
        const gridEndY = this.mapCenter[1] + MAP_HALF_SPAN;
        for (let coordinate = gridStartX; coordinate <= gridEndX; coordinate += 2) {
            const [x] = this._toCanvas([coordinate, 0]);
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, size);
            ctx.stroke();
        }
        for (let coordinate = gridStartY; coordinate <= gridEndY; coordinate += 2) {
            const [, y] = this._toCanvas([0, coordinate]);
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(size, y);
            ctx.stroke();
        }
        ctx.strokeStyle = 'rgba(125, 211, 252, 0.28)';
        ctx.strokeRect(0.5, 0.5, size - 1, size - 1);
        const worldTopLeft = this._toCanvas([
            -(snapshot.arenaHalfSize ?? G1_OBSTACLE_DEFAULTS.worldLimit),
            snapshot.arenaHalfSize ?? G1_OBSTACLE_DEFAULTS.worldLimit
        ]);
        const worldBottomRight = this._toCanvas([
            snapshot.arenaHalfSize ?? G1_OBSTACLE_DEFAULTS.worldLimit,
            -(snapshot.arenaHalfSize ?? G1_OBSTACLE_DEFAULTS.worldLimit)
        ]);
        ctx.save();
        ctx.strokeStyle = 'rgba(226, 232, 240, 0.55)';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 4]);
        ctx.strokeRect(
            worldTopLeft[0],
            worldTopLeft[1],
            worldBottomRight[0] - worldTopLeft[0],
            worldBottomRight[1] - worldTopLeft[1]
        );
        ctx.restore();

        this._drawPath(ctx, snapshot.trail, 'rgba(56, 189, 248, 0.56)', 1.2);
        this._drawPath(
            ctx,
            diagnostics.predictedPath,
            this.dpcbfEnabled ? '#facc15' : 'rgba(148, 163, 184, 0.55)',
            1.6
        );
        for (const [index, overlay] of (diagnostics.overlays ?? []).entries()) {
            const color = getG1DpcbfOverlayColor(overlay.colorIndex ?? index);
            if (this.dpcbfEnabled) {
                if (overlay.unsafeFill) {
                    ctx.save();
                    ctx.fillStyle = color;
                    ctx.globalAlpha = .18;
                    ctx.beginPath();
                    overlay.unsafeFill.forEach((point,i) => {
                        const [x,y] = this._toCanvas(point);
                        if (i === 0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
                    });
                    ctx.closePath();
                    ctx.fill();
                    ctx.restore();
                }
                this._drawPath(
                    ctx,
                    overlay.points,
                    color,
                    Math.max(1.2, 0.04 * scale),
                    false,
                    0.9
                );
            }
            const relative = overlay.relativeVelocityArrow;
            if (relative) {
                this._drawWorldArrow(
                    ctx,
                    relative.start,
                    relative.end,
                    color,
                    Math.max(1.1, 0.025 * scale),
                    false,
                    0.92
                );
            }
        }

        const graphEdges = snapshot.graphEdges ?? [];
        this.root.querySelector('[data-gat-key]').hidden = graphEdges.length === 0;
        ctx.save();
        ctx.strokeStyle = '#c084fc'; ctx.lineWidth = 1.15; ctx.globalAlpha = .8; ctx.setLineDash([3,3]);
        for (const edge of graphEdges) {
            ctx.beginPath(); ctx.moveTo(...this._toCanvas(edge.start)); ctx.lineTo(...this._toCanvas(edge.end)); ctx.stroke();
        }
        ctx.restore();

        for (const obstacle of snapshot.obstacles ?? []) {
            const center = this._toCanvas([obstacle.x, obstacle.y]);
            const constraintActive = activeConstraintIds.has(String(obstacle.id));
            const collisionRadius = getG1ObstacleCollisionRadius(obstacle);
            ctx.fillStyle = constraintActive
                ? 'rgba(248, 113, 113, 0.48)'
                : 'rgba(148, 163, 184, 0.38)';
            ctx.strokeStyle = constraintActive
                ? ACTIVE_CONSTRAINT_COLOR
                : '#94a3b8';
            ctx.lineWidth = constraintActive ? 1.8 : 1.2;
            ctx.beginPath();
            ctx.arc(center[0], center[1], Math.max(3, collisionRadius * scale), 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();
            this._drawVelocityArrow(
                ctx,
                center,
                [obstacle.vx, obstacle.vy],
                constraintActive ? ACTIVE_CONSTRAINT_COLOR : '#94a3b8'
            );
        }

        const preview = Object.hasOwn(snapshot, 'obstaclePreview') ? snapshot.obstaclePreview : this.preview;
        if (preview) {
            const center = this._toCanvas(preview.position);
            ctx.fillStyle = preview.valid === false ? 'rgba(248, 113, 113, 0.3)' : 'rgba(125, 211, 252, 0.3)';
            ctx.strokeStyle = preview.valid === false ? '#f87171' : '#7dd3fc';
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.arc(
                center[0],
                center[1],
                (preview.radius ?? G1_COLLISION_FOOTPRINTS.obstacleRadius) * scale,
                0,
                Math.PI * 2
            );
            ctx.fill();
            ctx.stroke();
            this._drawVelocityArrow(ctx, center, preview.velocity, preview.valid === false ? '#f87171' : '#7dd3fc');
        }

        const goal = this._toCanvas(snapshot.goal);
        ctx.fillStyle = 'rgba(74, 222, 128, 0.07)';
        ctx.strokeStyle = 'rgba(74, 222, 128, 0.42)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(goal[0], goal[1], (snapshot.goalRadius ?? 0.9) * scale, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.strokeStyle = '#4ade80';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(goal[0], goal[1], 6, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(goal[0] - 8, goal[1]);
        ctx.lineTo(goal[0] + 8, goal[1]);
        ctx.moveTo(goal[0], goal[1] - 8);
        ctx.lineTo(goal[0], goal[1] + 8);
        ctx.stroke();

        const robot = this._toCanvas(snapshot.robot.position);
        const yaw = snapshot.robot.yaw;
        this._drawCommandArrow(
            ctx,
            snapshot.robot,
            snapshot.nominalCommand,
            '#38bdf8',
            -3.5,
            true
        );
        if (diagnostics.showFilteredCommand !== false && diagnostics.referenceAvailable !== false) this._drawCommandArrow(
            ctx,
            snapshot.robot,
            snapshot.filteredCommand,
            '#facc15',
            3.5,
            false
        );
        ctx.save();
        ctx.translate(robot[0], robot[1]);
        ctx.rotate(-yaw);
        ctx.fillStyle = '#7dd3fc';
        ctx.shadowColor = 'rgba(56, 189, 248, 0.8)';
        ctx.shadowBlur = 8;
        ctx.beginPath();
        ctx.moveTo(9, 0);
        ctx.lineTo(-6, -5);
        ctx.lineTo(-3, 0);
        ctx.lineTo(-6, 5);
        ctx.closePath();
        ctx.fill();
        ctx.restore();

        this.clearanceText.textContent = Number.isFinite(diagnostics.clearance)
            ? `${diagnostics.clearance.toFixed(2)} m`
            : '—';
        this.barrierText.textContent = formatSigned(diagnostics.minBarrier);
        const intervening = this.dpcbfEnabled && diagnostics.intervening;
        this.filterStateText.textContent = diagnostics.qpStatus === 'infeasible' ? 'infeasible' : diagnostics.status === 'policy' ? 'RL' : diagnostics.filterApplied ? 'applied' : !this.dpcbfEnabled
            ? 'bypassed'
            : intervening ? 'active' : 'nominal';
        this.filterStateText.classList.toggle('active', Boolean(intervening));
        if (snapshot.policyId) {
            this.dpcbfButton.textContent = `${diagnostics.visualLabel ?? 'DPCBF'} VIS ${this.dpcbfEnabled ? 'ON' : 'OFF'}`;
            this.root.querySelector('[data-command-label]').textContent = diagnostics.showFilteredCommand === false ? 'Command' : 'Nominal';
            this.root.querySelector('[data-reference-label]').textContent = 'Filtered';
            this.root.querySelector('[data-reference-label]').parentElement.hidden = diagnostics.showFilteredCommand === false;
        }
        const params = diagnostics.params ?? {};
        this.filterParamsText.textContent = diagnostics.parameterLabel ?? [
            `α ${Number(params.alpha ?? 0).toFixed(2)}`,
            `kλ ${Number(params.kLambda ?? 0).toFixed(2)}`,
            `kμ ${Number(params.kMu ?? 0).toFixed(2)}`,
            `scale ${Number(params.safetyScale ?? 0).toFixed(2)}`
        ].join(' · ');
    }

    dispose() {
        this.root.remove();
    }
}
