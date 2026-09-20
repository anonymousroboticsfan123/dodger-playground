import {
    clamp,
    computePointNavigationCommand,
    sampleReachableGoal
} from '../unitree_g1/navigation.js?v=dodger-3';
import { UnitreeG1DpcbfFilter } from '../unitree_g1/dpcbfFilter.js?v=dodger-3';
import { UnitreeG1InteractionOverlay } from '../unitree_g1/interactionOverlay.js?v=dodger-5';
import {
    G1_OBSTACLE_DEFAULTS,
    advanceG1Obstacles,
    canPlaceG1Obstacle,
    createG1Obstacle,
    createReferenceObstacles,
    detectG1CircularCollision,
    findG1ObstacleAt,
    sampleG1Obstacle
} from '../unitree_g1/obstacles.js?v=dodger-3';
import { UnitreeG1Runtime, G1_RUNTIME_CONSTANTS } from '../unitree_g1/runtime.js?v=dodger-3';
import { UnitreeG1ThreeRenderer } from '../unitree_g1/threeRenderer.js?v=dodger-3';

const WORLD_LIMIT = 8;
const GOAL_HOLD_SECONDS = 0.55;
const PREDICTION_DT = 0.1;
const PREDICTION_STEPS = 24;

function forwardSpeed(robot) {
    return robot.worldVelocity[0] * Math.cos(robot.yaw)
        + robot.worldVelocity[1] * Math.sin(robot.yaw);
}

function obstacleInwardRadialSpeed(robotPosition, obstacle) {
    const dx = obstacle.x - robotPosition[0];
    const dy = obstacle.y - robotPosition[1];
    const distance = Math.max(1e-6, Math.hypot(dx, dy));
    return -(dx * obstacle.vx + dy * obstacle.vy) / distance;
}

function buildCommandPrediction(robot, command) {
    const path = [[robot.position[0], robot.position[1]]];
    let x = robot.position[0];
    let y = robot.position[1];
    let yaw = robot.yaw;
    let speed = forwardSpeed(robot);
    for (let step = 0; step < PREDICTION_STEPS; step += 1) {
        const speedDelta = clamp(
            command[0] - speed,
            -1.35 * PREDICTION_DT,
            1.0 * PREDICTION_DT
        );
        speed += speedDelta;
        yaw += command[2] * PREDICTION_DT;
        x += speed * Math.cos(yaw) * PREDICTION_DT;
        y += speed * Math.sin(yaw) * PREDICTION_DT;
        path.push([x, y]);
    }
    return path;
}

export class UnitreeG1Case {
    constructor(hostEl) {
        this.hostEl = hostEl;
        this.interaction = { mode: 'goal' };
        this.active = false;
        this.ready = false;
        this.paused = false;
        this.failure = null;
        this.loadError = null;
        this.initializationPromise = null;
        this.runtime = null;
        this.goal = [2.8, 0];
        this.nominalCommand = [0, 0, 0];
        this.filteredCommand = [0, 0, 0];
        this.distance = 2.8;
        this.headingError = 0;
        this.goalReachedAt = null;
        this.trail = [[0, 0]];
        this.lastTrailTime = 0;
        this.obstacles = createReferenceObstacles();
        this.nextObstacleId = this.obstacles.length + 1;
        this.dpcbfEnabled = true;
        this.dpcbfFilter = new UnitreeG1DpcbfFilter();
        this.dpcbfDiagnostics = null;
        this.collisionCount = 0;
        this.inCollision = false;
        this.renderer = new UnitreeG1ThreeRenderer(
            hostEl,
            (goal) => this.setGoal(goal),
            () => this.togglePaused()
        );
        this.interactionOverlay = new UnitreeG1InteractionOverlay(hostEl, {
            onGoal: (goal) => this.setGoal(goal),
            onAddObstacle: (obstacle) => this.addObstacle(obstacle),
            onRemoveObstacle: (point) => this.removeObstacle(point),
            onDpcbfToggle: (enabled) => this.setDpcbfEnabled(enabled),
            onReset: () => this.clearScene()
        });
    }

    async initialize() {
        if (this.ready) return;
        if (this.initializationPromise) return this.initializationPromise;
        this.initializationPromise = this._initialize();
        try {
            await this.initializationPromise;
        } catch (error) {
            this.initializationPromise = null;
            this.ready = false;
            this.loadError = error;
            await this.runtime?.dispose();
            this.renderer.detachRuntime();
            this.runtime = null;
            this.renderer.setStatus(error.message || 'Loading failed', true);
            throw error;
        }
    }

    async _initialize() {
        this.loadError = null;
        this.runtime = new UnitreeG1Runtime((status) => this.renderer.setStatus(status));
        await this.runtime.initialize();
        this.renderer.setStatus('Preparing renderer…');
        await new Promise((resolve) => requestAnimationFrame(() => resolve()));
        this.renderer.attachRuntime(this.runtime);
        this.ready = true;
        this.renderer.setStatus('Navigating');
    }

    setActive(active) {
        this.active = active;
        this.renderer.setVisible(active);
    }

    getPlaybackStep() {
        return G1_RUNTIME_CONSTANTS.controlDt;
    }

    getModeButtons() {
        return [];
    }

    setMode() {}

    isOverlayVisible() {
        return this.dpcbfEnabled;
    }

    _filterJoystickCommand(nominalCommand, robotState) {
        const result = this.dpcbfFilter.filter({
            nominalCommand,
            robotState,
            obstacles: this.obstacles,
            dt: G1_RUNTIME_CONSTANTS.controlDt
        });
        const constraints = result.diagnostics.constraints ?? [];
        const nearestConstraint = constraints.reduce((nearest, constraint) => (
            !nearest || constraint.physicalClearance < nearest.physicalClearance
                ? constraint
                : nearest
        ), null);
        this.dpcbfDiagnostics = {
            ...result.diagnostics,
            params: result.diagnostics.parameters,
            clearance: result.diagnostics.nearestClearance,
            nearestObstacleId: nearestConstraint?.obstacleId ?? null,
            intervening: result.diagnostics.intervention > 0.012,
            predictedPath: buildCommandPrediction(robotState, result.command)
        };
        return result.command;
    }

    _sampleSafeGoal(robotPosition) {
        for (let attempt = 0; attempt < 40; attempt += 1) {
            const goal = sampleReachableGoal(robotPosition, Math.random, {
                worldLimit: WORLD_LIMIT
            });
            const safe = this.obstacles.every((obstacle) => (
                Math.hypot(goal[0] - obstacle.x, goal[1] - obstacle.y)
                    >= obstacle.radius + 1.05
            ));
            if (safe) return goal;
        }
        return sampleReachableGoal(robotPosition, Math.random, {
            worldLimit: WORLD_LIMIT
        });
    }

    async step() {
        if (!this.active || !this.ready || this.paused || this.failure) return;

        let robot = this.runtime.getRobotState();
        advanceG1Obstacles(this.obstacles, G1_RUNTIME_CONSTANTS.controlDt);
        let navigation = computePointNavigationCommand({
            position: robot.position,
            yaw: robot.yaw,
            goal: this.goal,
            previousCommand: this.nominalCommand,
            dt: G1_RUNTIME_CONSTANTS.controlDt
        });

        if (navigation.reached || this.goalReachedAt != null) {
            if (this.goalReachedAt == null) {
                this.goalReachedAt = this.runtime.time;
            }

            navigation = {
                ...navigation,
                command: [0, 0, 0],
                reached: true
            };
            if (this.runtime.time - this.goalReachedAt >= GOAL_HOLD_SECONDS) {
                this.goal = this._sampleSafeGoal(robot.position);
                this.goalReachedAt = null;
                navigation = computePointNavigationCommand({
                    position: robot.position,
                    yaw: robot.yaw,
                    goal: this.goal,
                    previousCommand: [0, 0, 0],
                    dt: G1_RUNTIME_CONSTANTS.controlDt
                });
            }
        } else {
            this.goalReachedAt = null;
        }

        this.nominalCommand = navigation.command;
        this.filteredCommand = this._filterJoystickCommand(this.nominalCommand, robot);
        this.distance = navigation.distance;
        this.headingError = navigation.headingError;

        const result = await this.runtime.advance(this.filteredCommand);
        robot = this.runtime.getRobotState();
        if (result.reset) {
            this.nominalCommand = [0, 0, 0];
            this.filteredCommand = [0, 0, 0];
            this.trail = [[robot.position[0], robot.position[1]]];
            this.lastTrailTime = 0;
            this.goalReachedAt = null;
            this.dpcbfFilter.reset(robot);
            this.inCollision = false;
            return;
        }

        const contact = detectG1CircularCollision(robot.position, this.obstacles);
        if (contact) {
            const preCollisionCommand = [...this.filteredCommand];
            const preCollisionNominalCommand = [...this.nominalCommand];
            const preCollisionRobotState = {
                position: [...robot.position],
                yaw: robot.yaw,
                yawRate: robot.yawRate ?? null,
                worldVelocity: [...robot.worldVelocity],
                forwardSpeed: forwardSpeed(robot)
            };
            const preCollisionDpcbf = this.dpcbfDiagnostics
                ? {
                    status: this.dpcbfDiagnostics.status,
                    filteredCommand: [
                        ...(this.dpcbfDiagnostics.filteredCommand
                            ?? preCollisionCommand)
                    ],
                    qpFilteredCommand: [
                        ...(this.dpcbfDiagnostics.qpFilteredCommand
                            ?? preCollisionCommand)
                    ],
                    intervention: this.dpcbfDiagnostics.intervention,
                    nearestClearance: this.dpcbfDiagnostics.nearestClearance,
                    minBarrier: this.dpcbfDiagnostics.minBarrier,
                    predictiveSafety: {
                        ...(this.dpcbfDiagnostics.predictiveSafety ?? {})
                    },
                    nearestObstacleId:
                        this.dpcbfDiagnostics.nearestObstacleId
                }
                : null;
            this.collisionCount += 1;
            this.inCollision = true;
            this.failure = {
                reason: 'collision',
                details: 'Collision detected. Click anywhere to restart.',
                obstacleId: contact.obstacle.id,
                centerDistance: contact.distance,
                compactClearance: contact.clearance,
                obstacleState: {
                    position: [contact.obstacle.x, contact.obstacle.y],
                    velocity: [contact.obstacle.vx, contact.obstacle.vy]
                },
                preCollisionCommand,
                preCollisionNominalCommand,
                preCollisionRobotState,
                preCollisionDpcbf
            };
            this.paused = true;
            return;
        }
        this.inCollision = false;

        if (this.runtime.time - this.lastTrailTime >= 0.08) {
            this.trail.push([robot.position[0], robot.position[1]]);
            if (this.trail.length > 500) this.trail.shift();
            this.lastTrailTime = this.runtime.time;
        }
    }

    render() {
        const snapshot = this.getSnapshot();
        this.renderer.render(snapshot);
        this.interactionOverlay.render(snapshot);
    }

    getSnapshot() {
        const robot = this.runtime?.getRobotState() ?? {
            position: [0, 0, 0.755],
            yaw: 0,
            worldVelocity: [0, 0, 0]
        };
        return {
            status: this.loadError ? 'error' : this.ready ? 'ready' : 'loading',
            ready: this.ready,
            paused: this.paused,
            failure: this.failure,
            time: this.runtime?.time ?? 0,
            robot,
            goal: [...this.goal],
            distance: this.distance,
            headingError: this.headingError,
            goalReached: this.goalReachedAt != null,
            nominalCommand: [...this.nominalCommand],
            filteredCommand: [...this.filteredCommand],
            obstacles: this.obstacles,
            dpcbf: {
                enabled: this.dpcbfEnabled,
                status: this.dpcbfEnabled ? 'monitoring' : 'disabled',
                params: this.dpcbfFilter.config,
                clearance: null,
                minBarrier: null,
                nearestObstacleId: null,
                intervening: false,
                predictedPath: []
            },
            collisionCount: this.collisionCount,
            inCollision: this.inCollision,
            diagnostics: this.runtime?.getDiagnostics() ?? {
                inferenceMs: 0,
                physicsMs: 0,
                resetCount: 0,
                observationSize: G1_RUNTIME_CONSTANTS.observationSize,
                actionSize: G1_RUNTIME_CONSTANTS.actionSize
            },
            trail: this.trail,
            ...(this.dpcbfDiagnostics ? {
                dpcbf: {
                    ...this.dpcbfDiagnostics,
                    enabled: this.dpcbfEnabled
                }
            } : {})
        };
    }

    setGoal(goal) {
        this.goal = [
            clamp(goal[0], -WORLD_LIMIT, WORLD_LIMIT),
            clamp(goal[1], -WORLD_LIMIT, WORLD_LIMIT)
        ];
        this.goalReachedAt = null;
    }

    randomizeScene() {
        const robot = this.runtime?.getRobotState().position ?? [0, 0, 0];
        this.obstacles = [];
        for (let index = 0; index < G1_OBSTACLE_DEFAULTS.count; index += 1) {
            const obstacle = sampleG1Obstacle(
                `g1-obstacle-${this.nextObstacleId++}`,
                robot,
                this.goal,
                this.obstacles
            );
            if (obstacle) this.obstacles.push(obstacle);
        }
        this.setGoal(this._sampleSafeGoal(robot));
        this.dpcbfFilter.reset(this.runtime?.getRobotState());
    }

    clearScene() {
        this.runtime?.reset('manual');
        this.failure = null;
        this.paused = false;
        this.goal = [2.8, 0];
        this.obstacles = createReferenceObstacles();
        this.nominalCommand = [0, 0, 0];
        this.filteredCommand = [0, 0, 0];
        this.dpcbfDiagnostics = null;
        this.dpcbfFilter.reset(this.runtime?.getRobotState());
        this.trail = [[0, 0]];
        this.lastTrailTime = 0;
        this.goalReachedAt = null;
        this.collisionCount = 0;
        this.inCollision = false;
    }

    resetFromFailure() {
        if (!this.failure) return false;
        this.runtime?.reset('collision restart');
        let robot = this.runtime?.getRobotState() ?? {
            position: [0, 0, 0.755],
            yaw: 0,
            worldVelocity: [0, 0, 0]
        };

        if (detectG1CircularCollision(robot.position, this.obstacles)) {
            this.obstacles = createReferenceObstacles();
        }
        this.failure = null;
        this.paused = false;
        this.inCollision = false;
        this.nominalCommand = [0, 0, 0];
        this.filteredCommand = [0, 0, 0];
        this.dpcbfDiagnostics = null;
        this.dpcbfFilter.reset(robot);
        this.trail = [[robot.position[0], robot.position[1]]];
        this.lastTrailTime = 0;
        this.goalReachedAt = null;
        const navigation = computePointNavigationCommand({
            position: robot.position,
            yaw: robot.yaw,
            goal: this.goal,
            previousCommand: [0, 0, 0],
            dt: G1_RUNTIME_CONSTANTS.controlDt
        });
        this.distance = navigation.distance;
        this.headingError = navigation.headingError;
        return true;
    }

    setDpcbfEnabled(enabled) {
        this.dpcbfEnabled = Boolean(enabled);
        this.dpcbfFilter.setEnabled(this.dpcbfEnabled);
        this.interactionOverlay.setDpcbfEnabled(this.dpcbfEnabled);
    }

    addObstacle({ position, velocity }) {
        const candidate = createG1Obstacle({
            id: `g1-obstacle-${this.nextObstacleId++}`,
            x: clamp(position[0], -G1_OBSTACLE_DEFAULTS.worldLimit, G1_OBSTACLE_DEFAULTS.worldLimit),
            y: clamp(position[1], -G1_OBSTACLE_DEFAULTS.worldLimit, G1_OBSTACLE_DEFAULTS.worldLimit),
            vx: velocity[0],
            vy: velocity[1],
            source: 'user'
        });
        const robot = this.runtime?.getRobotState() ?? {
            position: [0, 0, 0.755],
            worldVelocity: [0, 0, 0]
        };
        const inwardThreshold = G1_OBSTACLE_DEFAULTS.inwardUserThreatSpeed;
        if (
            obstacleInwardRadialSpeed(robot.position, candidate)
            >= inwardThreshold
        ) {
            const concurrentInward = this.obstacles.filter((obstacle) => (
                obstacle.source === 'user'
                && obstacleInwardRadialSpeed(robot.position, obstacle)
                    >= inwardThreshold
            )).length;
            if (
                concurrentInward
                >= G1_OBSTACLE_DEFAULTS.maxConcurrentInwardUserObstacles
            ) {
                return false;
            }
        }
        if (!canPlaceG1Obstacle(candidate, this.obstacles, robot.position, {
            robotLaunchDistance: G1_OBSTACLE_DEFAULTS.robotLaunchDistance
        })) return false;
        this.obstacles.push(candidate);
        return true;
    }

    removeObstacle(point) {
        const obstacle = findG1ObstacleAt(this.obstacles, point);
        if (!obstacle) return false;
        this.obstacles = this.obstacles.filter((entry) => entry.id !== obstacle.id);
        return true;
    }

    togglePaused() {
        return this.setPaused(!this.paused);
    }

    setPaused(value) {
        const nextPaused = Boolean(value);
        if (!this.ready || this.failure || nextPaused === this.paused) {
            return this.paused;
        }
        this.paused = nextPaused;
        if (this.paused) {
            this.nominalCommand = [0, 0, 0];
            this.filteredCommand = [0, 0, 0];
        }
        return this.paused;
    }

    resetView() {
        this.renderer.resetView();
    }

    handlePointerDown(event) {
        return this.renderer.handlePointerDown(event);
    }

    handlePointerMove(event) {
        return this.renderer.handlePointerMove(event);
    }

    handlePointerUp(event) {
        return this.renderer.handlePointerUp(event);
    }

    handlePointerLeave() {
        return this.renderer.handlePointerLeave();
    }

    handleWheel(event) {
        return this.renderer.handleWheel(event);
    }

    async dispose() {
        this.interactionOverlay.dispose();
        this.renderer.dispose();
        await this.runtime?.dispose();
    }
}
