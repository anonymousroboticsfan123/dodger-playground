import { UnitreeG1Case } from './unitreeG1Case.js?v=dodger-3';
import { DodgerRuntime } from '../dodger/runtime.js?v=dodger-3';
import { NavigationPolicies } from '../dodger/policy.js?v=dodger-3';
import { DODGER_CONFIG as C, NAVIGATION_POLICIES, clamp, integrateCommand, wrapAngle } from '../dodger/config.js?v=dodger-3';
import { createScenario, sampleGoal, advanceObstacles, collisionAt } from '../dodger/scenario.js?v=dodger-3';
import { buildNavigationGraph } from '../dodger/observations.js?v=dodger-3';
import { getG1DpcbfOverlayColor } from '../unitree_g1/interactionOverlay.js?v=dodger-3';
import { solveAnalyticController, analyticDiagnostics, learnedDiagnostics } from '../dodger/baselines.js?v=dodger-3';

export class DodgerCase extends UnitreeG1Case {
    constructor(hostEl) {
        super(hostEl);
        this.policies = new NavigationPolicies();
        this.policyId = 'dodger';
        this.seed = C.defaultSeed;
        this.switching = false;
        this.pendingStep = null;
        this.resetPending = false;
        this._resetScene();
        this.controls = document.createElement('section');
        this.controls.className = 'dodger-controls';
        this.controls.dataset.g1Interactive = '';
        this.controls.setAttribute('aria-label', 'Navigation controller');
        this.controls.innerHTML = `<div class="dodger-controls-heading">NAVIGATION CONTROLLER <span>Reduced order model: 5 state with lateral motion</span></div>
            <div class="dodger-policy-buttons" role="group" aria-label="Navigation controller choice">
            ${NAVIGATION_POLICIES.map((p) => `<button type="button" data-policy="${p.id}" aria-pressed="${p.id === this.policyId}" title="Switch controller and replay the same scene">${p.label}<small>${p.kind==='qp'?'analytic QP · 50 Hz':'learned · 10 Hz'}</small></button>`).join('')}</div>
            <div class="dodger-scene-controls"><span class="dodger-run-info" data-run-info role="status"></span><button type="button" data-new-scene>New scene</button></div>
            <div class="dodger-cone-inspector" data-cone-inspector hidden>
                <label>Inspect cone <select data-cone-choice aria-label="Inspect obstacle cone"><option value="all">All obstacles</option></select></label>
                <div class="dodger-cone-body"><div><div data-cone-details></div>
                <p>Shaded wedge: unsafe relative velocity. Arrow: obstacle velocity − robot velocity. Ground scale: 1 m = 1 m/s.</p></div>
                <figure data-cone-zoom hidden><canvas width="256" height="176" aria-label="Zoomed relative velocity and unsafe collision cone"></canvas><figcaption>Velocity · auto scale</figcaption></figure></div>
            </div>`;

        (hostEl.closest('.canvas-stack') ?? hostEl).appendChild(this.controls);
        for (const button of this.controls.querySelectorAll('[data-policy]')) {
            button.addEventListener('click', () => void this.selectPolicy(button.dataset.policy));
        }
        this.controls.querySelector('[data-new-scene]').addEventListener('click', () => {
            if (this.switching) return;
            this.seed += 1;
            this.clearScene();
        });
        this.controls.querySelector('[data-cone-choice]').addEventListener('change', (event) => {
            this.focusedObstacleId = event.target.value;
            this._renderControls();
        });
        this.controlsObserver = new ResizeObserver(() => {
            document.documentElement.style.setProperty('--dodger-controls-bottom', `${this.controls.offsetHeight + 18}px`);
        });
        this.controlsObserver.observe(this.controls);
        this._renderControls();
    }

    async _initialize() {
        this.loadError = null;
        this.runtime = new DodgerRuntime((status) => this.renderer.setStatus(status));
        const results = await Promise.allSettled([this.runtime.initialize(), this.policies.load(this.policyId)]);
        const failure = results.find((result) => result.status === 'rejected');
        if (failure) { await this.policies.dispose(); throw failure.reason; }
        this.renderer.attachRuntime(this.runtime);
        this.ready = true;
        this._resetScene();
    }

    _resetScene() {
        this.interactionOverlay?.cancelDrag();
        const scenario = createScenario(this.seed);
        this.obstacles = scenario.obstacles;
        this.goal = scenario.goal;
        this.goalYaw = scenario.goalYaw;
        this.random = scenario.random;
        this.nextObstacleId = C.obstacleCount + 1;
        this.runtime?.reset('scene replay');
        this.failure = null;
        this.paused = false;
        this.inCollision = false;
        this.collisionCount = 0;
        this.goalsReached = 0;
        this.qpInfeasibleCount = 0;
        this.qpSolveCount = 0;
        this.focusedObstacleId = 'all';
        this.qpInterventions = 0;
        this.nominalCommand = [0, 0, 0];
        this.filteredCommand = [0, 0, 0];
        this.executedCommand = [0, 0, 0];
        this.policyAction = [0, 0, 0];
        this.previousAction = [0, 0, 0];
        this.lowTick = 0;
        this.goalReachedAt = null;
        this.dpcbfDiagnostics = null;
        this.trail = [[0, 0]];
        this.lastTrailTime = 0;
        this.distance = Math.hypot(...this.goal);
        this.headingError = this.goalYaw;
        this.resetPending = false;
    }

    async selectPolicy(id) {
        if (!this.ready || this.switching || id === this.policyId) return;
        this.switching = true;
        this._renderControls();
        try {
            await this.pendingStep;
            await this.policies.load(id);
            this.policyId = id;
            this._resetScene();
        } catch (error) {
            this._fail('policy error', error.message);
        } finally {
            this.switching = false;
            this._renderControls();
        }
    }

    async step() {
        if (this.pendingStep) return this.pendingStep;
        if (!this.active || !this.ready || this.paused || this.failure || this.switching) return;
        this.pendingStep = this._step();
        try { await this.pendingStep; }
        catch (error) { this._fail('runtime error', error.message); console.error('[dodger]', error); }
        finally {
            this.pendingStep = null;
            if (this.resetPending) this._resetScene();
        }
    }

    async _step() {
        let robot = this.runtime.getRobotState();
        this.distance = Math.hypot(this.goal[0] - robot.position[0], this.goal[1] - robot.position[1]);
        this.headingError = wrapAngle(this.goalYaw - robot.yaw);
        if (this.distance <= C.goalRadius && Math.abs(this.headingError) <= C.goalHeadingTolerance && this.goalReachedAt == null) {
            this.goalReachedAt = this.runtime.time;
            this.goalsReached += 1;
        }
        if (this.goalReachedAt != null && this.runtime.time - this.goalReachedAt >= 0.55) {
            Object.assign(this, sampleGoal(this.random, robot.position, this.obstacles));
            this.goalReachedAt = null;
            this.lowTick = 0;
        }
        const policy = NAVIGATION_POLICIES.find(p => p.id === this.policyId);
        const learned = policy.kind === 'learned';
        if (!learned || this.lowTick % 5 === 0) {
            this.policyAction = await this.policies.infer(this.policyId, {
                robot, goal: this.goal, goalYaw: this.goalYaw,
                command: this.executedCommand, previousAction: this.previousAction, obstacles: this.obstacles
            });
            this.previousAction = [...this.policyAction];
            if (learned) this.executedCommand = integrateCommand(this.executedCommand, this.policyAction, C.highLevelDt);
        }
        const beforeCommand = [...this.executedCommand];
        const method = policy.barrier ?? this.policyId;
        this.nominalCommand = learned ? [...this.executedCommand] : integrateCommand(beforeCommand, this.policyAction, C.controlDt);
        if (learned) {
            this.dpcbfDiagnostics = learnedDiagnostics(method, robot, this.obstacles, beforeCommand, this.policyAction, { visible: this.dpcbfEnabled });
            this.filteredCommand = [...this.executedCommand];
        } else {
            const reference = solveAnalyticController(method, robot, this.obstacles, beforeCommand, this.policyAction);
            this.qpSolveCount += 1;
            this.dpcbfDiagnostics = analyticDiagnostics(method, robot, beforeCommand, this.policyAction, reference, { visible: this.dpcbfEnabled });
            this.filteredCommand = this.dpcbfDiagnostics.referenceCommand;
            if (!reference.feasible) {
                this.qpInfeasibleCount += 1;
                this._fail('qp infeasible', 'The CBF constraints and actuator limits cannot all be satisfied.');
                return;
            }
            this.executedCommand = [...this.filteredCommand];
            if (this.dpcbfDiagnostics.intervening) this.qpInterventions += 1;
        }
        let contact = null;
        const result = await this.runtime.advance(this.executedCommand, (state, dt) => {
            advanceObstacles(this.obstacles, dt);
            contact = collisionAt(state, this.obstacles);
            return Boolean(contact);
        });
        this.lowTick += 1;
        if (result.failure) {
            if (result.failure === 'collision') { this.collisionCount += 1; this.inCollision = true; }
            this._fail(result.failure, result.failure === 'collision'
                ? 'Collision detected.'
                : 'Robot stopped.');
            if (contact) this.failure.obstacleId = contact.obstacle.id;
            return;
        }
        robot = this.runtime.getRobotState();
        if (Math.max(Math.abs(robot.position[0]), Math.abs(robot.position[1])) > C.hardBoundary) {
            this._fail('out of bounds', 'Robot left the arena.');
        }
        if (this.runtime.time - this.lastTrailTime >= 0.08) {
            this.trail.push(robot.position.slice(0, 2));
            if (this.trail.length > 500) this.trail.shift();
            this.lastTrailTime = this.runtime.time;
        }
    }

    _fail(reason, details) { this.interactionOverlay.cancelDrag(); this.failure = { reason, details }; this.paused = true; }
    clearScene() { if (this.pendingStep) this.resetPending = true; else this._resetScene(); }
    resetFromFailure() { if (!this.failure) return false; this.clearScene(); return true; }
    setPaused(value) { if (this.ready && !this.failure) this.paused = Boolean(value); return this.paused; }
    setDpcbfEnabled(enabled) { this.dpcbfEnabled = Boolean(enabled); this.interactionOverlay.setDpcbfEnabled(enabled); }
    setGoal(goal) {
        if (this.failure) return;
        const position = this.runtime?.getRobotState().position ?? [0, 0];
        this.goal = goal.map((v) => clamp(v, -C.arenaHalfSize + 0.4, C.arenaHalfSize - 0.4));
        this.goalYaw = Math.atan2(this.goal[1] - position[1], this.goal[0] - position[0]);
        this.goalReachedAt = null;
    }
    getObstaclePreview(draft) {
        if (!draft || this.failure) return null;
        const radius = 0.25, speed = Math.hypot(...draft.velocity);
        const scale = speed > C.obstacleSpeedRange[1] ? C.obstacleSpeedRange[1] / speed : 1;
        const position = draft.position.map(value => clamp(value, -C.arenaHalfSize + radius, C.arenaHalfSize - radius));
        const robotPosition = this.runtime?.getRobotState().position ?? [0,0];
        return { position, velocity: draft.velocity.map(value => value*scale), radius,
            valid: Math.hypot(position[0]-robotPosition[0],position[1]-robotPosition[1]) >= C.robotRadius + radius + .2 };
    }
    addObstacle(draft) {
        const preview = this.getObstaclePreview(draft);
        if (!preview?.valid) return false;
        const { position, velocity, radius } = preview;
        this.obstacles.push({ id: `dodger-user-${this.nextObstacleId++}`, radius, collisionRadius: radius,
            x: position[0], y: position[1], vx: velocity[0], vy: velocity[1], source: 'user' });
        return true;
    }
    removeObstacle(point) { if (!this.failure) return super.removeObstacle(point); }
    getSnapshot() {
        const snapshot = super.getSnapshot();
        const policy = NAVIGATION_POLICIES.find(p => p.id === this.policyId);
        const learned = policy.kind === 'learned';
        const overlays = snapshot.dpcbf.overlays?.filter(overlay => this.focusedObstacleId === 'all' || String(overlay.obstacleId) === this.focusedObstacleId);

        return { ...snapshot, policyId: this.policyId,
            obstaclePreview: this.getObstaclePreview(this.interactionOverlay.preview),
            graphEdges: policy.graphAttention ? buildNavigationGraph({ robot: snapshot.robot, goal: this.goal, obstacles: this.obstacles }) : [],
            controllerLabel: NAVIGATION_POLICIES.find((p) => p.id === this.policyId).label,
            goalYaw: this.goalYaw, goalRadius: C.goalRadius,
            arenaHalfSize: C.arenaHalfSize,
            executedCommand: [...this.executedCommand],
            goalsReached: this.goalsReached, seed: this.seed,
            qpInfeasibleCount: this.qpInfeasibleCount, qpInterventions: this.qpInterventions,
            diagnostics: { ...snapshot.diagnostics, navigationInferenceMs: this.policies.lastInferenceMs },
            dpcbf: { ...snapshot.dpcbf, overlays,
                constrainedObstacleIds: this.focusedObstacleId === 'all' ? snapshot.dpcbf.constrainedObstacleIds : [this.focusedObstacleId], referenceOnly: false, filterApplied: !learned,
                showFilteredCommand: !learned, referenceAvailable: learned ? false : snapshot.dpcbf.referenceAvailable }
        };
    }
    _renderControls() {
        for (const button of this.controls.querySelectorAll('[data-policy]')) {
            button.disabled = this.switching || !this.ready || !NAVIGATION_POLICIES.find((p) => p.id === button.dataset.policy).available;
            button.setAttribute('aria-pressed', String(button.dataset.policy === this.policyId));
        }
        const status = this.failure ? this.failure.reason : this.paused ? 'paused' : 'running';
        this.controls.querySelector('[data-run-info]').textContent = this.switching ? 'Loading policy…'
            : `${(this.runtime?.time ?? 0).toFixed(1)} s · ${this.goalsReached} goals · ${status}`;
        const inspector = this.controls.querySelector('[data-cone-inspector]');
        inspector.hidden = !['c3bf','c3bf_rl'].includes(this.policyId);
        if (!inspector.hidden) {
            const values = this.dpcbfDiagnostics?.constraints ?? [];
            const choice = this.controls.querySelector('[data-cone-choice]');
            const ids = values.map(v => String(v.obstacleId));
            const key = ids.join(',');
            if (choice.dataset.ids !== key) {
                choice.replaceChildren(new Option('All obstacles', 'all'), ...values.map((v,i) => new Option(`Obstacle ${i+1}`, String(v.obstacleId))));
                choice.dataset.ids = key;
            }
            if (!ids.includes(this.focusedObstacleId)) this.focusedObstacleId = 'all';
            choice.value = this.focusedObstacleId;
            const selected = values.find(v => String(v.obstacleId) === this.focusedObstacleId);
            this.controls.querySelector('[data-cone-details]').textContent = selected
                ? `h ${(Math.abs(selected.h) > 0 && Math.abs(selected.h) < .001 ? selected.h.toExponential(1) : selected.h.toFixed(3))} · relative speed ${Math.hypot(...selected.relativeVelocity).toFixed(2)} m/s · ${selected.h < -1e-8 ? 'inside unsafe cone' : Math.abs(selected.h) <= 1e-8 ? 'on boundary' : 'outside cone'}`
                : `${values.filter(v => v.h < -1e-8).length} / ${values.length} relative velocities inside unsafe cones`;
            this._renderConeZoom(selected);
        }
    }

    _renderConeZoom(selected) {
        const figure = this.controls.querySelector('[data-cone-zoom]');
        figure.hidden = !selected;
        if (!selected) return;
        const overlay = this.dpcbfDiagnostics.overlays.find(o => o.obstacleId === selected.obstacleId);
        const canvas = figure.querySelector('canvas'), ctx = canvas.getContext('2d');
        const origin = [64, 44], radius = 34;
        const color = getG1DpcbfOverlayColor(overlay.colorIndex);
        ctx.setTransform(2,0,0,2,0,0); ctx.clearRect(0,0,128,88);
        ctx.strokeStyle = '#35495d'; ctx.lineWidth = .5;
        ctx.beginPath(); ctx.moveTo(10,44); ctx.lineTo(118,44); ctx.moveTo(64,9); ctx.lineTo(64,79); ctx.stroke();
        ctx.fillStyle = '#87a7be'; ctx.font = '8px monospace'; ctx.fillText('vₓ',113,40); ctx.fillText('vᵧ',68,12);
        const center = overlay.relativeVelocityArrow.start;
        const rays = [overlay.points[0],overlay.points[2]].map(point => {
            const dx = point[0]-center[0], dy = point[1]-center[1], length = Math.hypot(dx,dy);
            return [origin[0]+radius*dx/length,origin[1]-radius*dy/length];
        });
        ctx.beginPath(); ctx.moveTo(...rays[0]); ctx.lineTo(...origin); ctx.lineTo(...rays[1]); ctx.closePath();
        ctx.fillStyle = color; ctx.globalAlpha = .28; ctx.fill(); ctx.globalAlpha = 1; ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.stroke();
        const speed = Math.hypot(...selected.relativeVelocity), velocityScale = radius/Math.max(speed*1.25,.001);
        const end = [origin[0]+selected.relativeVelocity[0]*velocityScale,origin[1]-selected.relativeVelocity[1]*velocityScale];
        ctx.strokeStyle = '#f0f9ff'; ctx.fillStyle = '#f0f9ff'; ctx.lineWidth = 1.7;
        if (speed > 1e-8) {
            const angle = Math.atan2(end[1]-origin[1],end[0]-origin[0]);
            ctx.beginPath(); ctx.moveTo(...origin); ctx.lineTo(...end); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(...end);
            ctx.lineTo(end[0]-6*Math.cos(angle-.4),end[1]-6*Math.sin(angle-.4));
            ctx.lineTo(end[0]-6*Math.cos(angle+.4),end[1]-6*Math.sin(angle+.4)); ctx.closePath(); ctx.fill();
        }
        ctx.beginPath(); ctx.arc(...origin,1.7,0,2*Math.PI); ctx.fill();
    }
    render() { super.render(); this._renderControls(); }
    async dispose() { await this.pendingStep; this.controlsObserver.disconnect(); this.controls.remove(); await this.policies.dispose(); await super.dispose(); }
}
