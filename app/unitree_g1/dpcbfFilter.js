import { solveBoxConstrainedQP } from '../qp.js?v=dodger-3';

const EPS = 1e-6;
const DEFAULT_DT = 0.02;

export const G1_DPCBF_DEFAULTS = Object.freeze({
    enabled: true,
    robotRadius: 0.42,
    safetyScale: 1.55,
    kLambda: 0.14,
    kMu: 0.42,
    alpha: 0.52,
    minForwardSpeed: -0.22,
    maxForwardSpeed: 0.8,
    minMeasuredForwardSpeed: -1.2,
    maxMeasuredForwardSpeed: 1.2,
    minAcceleration: -2.0,
    maxAcceleration: 1.25,
    maxYawRate: 1.0,
    maxLateralSpeed: 0.3,
    activationDistance: 6.2,
    maxObstacles: 24,
    policyLagSeconds: 0.9,
    positionUncertainty: 0.18,
    commandLeadSeconds: 0.42,
    measurementBlend: 0.18,
    finiteDifferenceStep: 1e-4,
    qpWeights: Object.freeze([5, 0.07]),
    slackEnabled: true,
    slackWeight: 10_000,
    interventionTolerance: 1e-4,
    lateralScaleWhileFiltering: 0.25,
    escapeSteeringEnabled: true,
    escapeTimeToContact: 3.2,
    escapeMinimumClosingSpeed: 0.12,
    escapeYawRate: 0.85,
    escapeHoldSeconds: 1.1,
    escapePredictionHorizon: 1.8,
    escapePredictionStep: 0.3,
    predictiveSafetyEnabled: true,
    predictiveHorizon: 3,
    predictiveStep: 0.15,
    predictiveTriggerClearance: 0.45,
    predictiveTargetClearance: 0.3,
    predictiveForwardLagSeconds: 1.5,
    predictiveYawLagSeconds: 1.3,
    predictiveMinAcceleration: -0.35,
    predictiveMaxAcceleration: 0.45,
    predictiveMaxYawAcceleration: 0.55,
    predictiveTurnTrackingTolerance: 0.18,
    predictiveMeasuredSpeedCandidateEnabled: true,
    contactRobotRadius: 0.28,
    contactObstacleRadius: 0.28,
    overlayHalfWidth: 1.5,
    overlayPointCount: 72
});

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function finite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function positive(value, fallback) {
    return Number.isFinite(value) && value > 0 ? value : fallback;
}

function readPair(value, fallback = [0, 0]) {
    return Array.isArray(value) || ArrayBuffer.isView(value)
        ? [finite(value[0], fallback[0]), finite(value[1], fallback[1])]
        : [...fallback];
}

function cloneCommand(command) {
    return [
        finite(command?.[0], 0),
        finite(command?.[1], 0),
        finite(command?.[2], 0)
    ];
}

function resolveObstacleRadii(obstacle) {
    const radii = obstacle.radii ?? obstacle.semiAxes ?? obstacle.halfExtents;
    const pair = readPair(radii, [NaN, NaN]);
    const baseRadius = positive(obstacle.radius, 0.35);
    const radiusX = positive(
        obstacle.radiusX ?? obstacle.rx ?? obstacle.semiMajor ?? pair[0],
        baseRadius
    );
    const radiusY = positive(
        obstacle.radiusY ?? obstacle.ry ?? obstacle.semiMinor ?? pair[1],
        baseRadius
    );
    return [radiusX, radiusY];
}

export function normalizeG1Obstacle(obstacle = {}, index = 0) {
    const position = readPair(obstacle.position, [
        finite(obstacle.x, 0),
        finite(obstacle.y, 0)
    ]);
    const velocity = readPair(obstacle.velocity, [
        finite(obstacle.vx, 0),
        finite(obstacle.vy, 0)
    ]);
    const radii = resolveObstacleRadii(obstacle);
    return {
        id: String(obstacle.id ?? `obstacle_${index}`),
        x: position[0],
        y: position[1],
        vx: velocity[0],
        vy: velocity[1],
        radiusX: radii[0],
        radiusY: radii[1],
        yaw: finite(obstacle.yaw ?? obstacle.heading ?? obstacle.rotation, 0),
        angularVelocity: finite(
            obstacle.angularVelocity ?? obstacle.omega ?? obstacle.yawRate,
            0
        ),
        shape: Math.abs(radii[0] - radii[1]) <= 1e-9 ? 'circle' : 'ellipse'
    };
}

export function reduceG1RobotState(robotState = {}, fallbackForwardSpeed = 0) {
    const position = readPair(robotState.position, [
        finite(robotState.x, 0),
        finite(robotState.y, 0)
    ]);
    const heading = finite(
        robotState.yaw ?? robotState.theta ?? robotState.heading,
        0
    );
    const worldVelocity = readPair(robotState.worldVelocity, [
        finite(robotState.vx, 0),
        finite(robotState.vy, 0)
    ]);
    const measuredForwardSpeed =
        worldVelocity[0] * Math.cos(heading) + worldVelocity[1] * Math.sin(heading);
    const explicitForwardSpeed =
        robotState.forwardSpeed ?? robotState.v;
    return {
        x: position[0],
        y: position[1],
        heading,
        forwardSpeed: finite(
            explicitForwardSpeed,
            Number.isFinite(measuredForwardSpeed)
                ? measuredForwardSpeed
                : fallbackForwardSpeed
        ),
        measuredWorldVelocity: worldVelocity
    };
}

function obstacleSupportRadius(obstacle, bearing) {
    const localBearing = bearing - obstacle.yaw;
    const ux = Math.cos(localBearing);
    const uy = Math.sin(localBearing);

    return Math.hypot(obstacle.radiusX * ux, obstacle.radiusY * uy);
}

export function evaluateG1DpcbfBarrier(
    reducedState,
    obstacleInput,
    options = {}
) {
    const config = { ...G1_DPCBF_DEFAULTS, ...options };
    const obstacle = obstacleInput.radiusX == null
        ? normalizeG1Obstacle(obstacleInput)
        : obstacleInput;
    const dx = obstacle.x - reducedState.x;
    const dy = obstacle.y - reducedState.y;
    const centerDistance = Math.max(Math.hypot(dx, dy), EPS);
    const bearing = Math.atan2(dy, dx);
    const radialX = dx / centerDistance;
    const radialY = dy / centerDistance;
    const tangentX = -radialY;
    const tangentY = radialX;

    const obstacleRadius = obstacleSupportRadius(obstacle, bearing);
    const physicalRadius = obstacleRadius + config.robotRadius;
    const egoVx = reducedState.forwardSpeed * Math.cos(reducedState.heading);
    const egoVy = reducedState.forwardSpeed * Math.sin(reducedState.heading);
    const relativeVx = obstacle.vx - egoVx;
    const relativeVy = obstacle.vy - egoVy;
    const relativeSpeed = Math.max(Math.hypot(relativeVx, relativeVy), EPS);
    const radialVelocity = relativeVx * radialX + relativeVy * radialY;
    const lateralVelocity = relativeVx * tangentX + relativeVy * tangentY;
    const closingSpeed = Math.max(0, -radialVelocity);
    const policyLagBuffer =
        closingSpeed * Math.max(0, finite(config.policyLagSeconds, 0));
    const uncertaintyBuffer = Math.max(
        0,
        finite(config.positionUncertainty, 0)
    );
    const reactionBuffer = policyLagBuffer + uncertaintyBuffer;
    const safetyRadius =
        physicalRadius * config.safetyScale + reactionBuffer;
    const squaredClearance = Math.max(
        centerDistance * centerDistance - safetyRadius * safetyRadius,
        EPS
    );
    const distanceFactor = Math.sqrt(squaredClearance);
    const adaptiveScale =
        Math.sqrt(Math.max(config.safetyScale * config.safetyScale - 1, EPS))
        / Math.max(safetyRadius, EPS);
    const lambdaCoefficient =
        config.kLambda * distanceFactor * adaptiveScale / relativeSpeed;
    const muCoefficient = config.kMu * distanceFactor * adaptiveScale;
    const h =
        radialVelocity
        + lambdaCoefficient * lateralVelocity * lateralVelocity
        + muCoefficient;

    return {
        h,
        centerDistance,
        physicalClearance: centerDistance - physicalRadius,
        safetyClearance: centerDistance - safetyRadius,
        physicalRadius,
        safetyRadius,
        obstacleRadius,
        relativeVelocity: [relativeVx, relativeVy],
        radialVelocity,
        lateralVelocity,
        closingSpeed,
        relativeSpeed,
        policyLagBuffer,
        uncertaintyBuffer,
        reactionBuffer,
        bearing,
        lambdaCoefficient,
        muCoefficient,
        colliding: centerDistance <= physicalRadius,
        insideSafetyRadius: centerDistance <= safetyRadius
    };
}

function advanceDrift(reducedState, obstacle, duration) {
    return {
        state: {
            ...reducedState,
            x: reducedState.x
                + reducedState.forwardSpeed * Math.cos(reducedState.heading) * duration,
            y: reducedState.y
                + reducedState.forwardSpeed * Math.sin(reducedState.heading) * duration
        },
        obstacle: {
            ...obstacle,
            x: obstacle.x + obstacle.vx * duration,
            y: obstacle.y + obstacle.vy * duration,
            yaw: obstacle.yaw + obstacle.angularVelocity * duration
        }
    };
}

function centeredDifference(evaluatePlus, evaluateMinus, step) {
    return (evaluatePlus() - evaluateMinus()) / (2 * step);
}

function createOverlay(reducedState, obstacle, barrier, config) {
    const count = Math.max(3, Math.round(config.overlayPointCount));
    const halfWidth = positive(config.overlayHalfWidth, 1.25);
    const c = Math.cos(barrier.bearing);
    const s = Math.sin(barrier.bearing);
    const points = [];
    for (let index = 0; index < count; index += 1) {
        const lateral = -halfWidth + (2 * halfWidth * index) / (count - 1);
        const radial =
            -barrier.lambdaCoefficient * lateral * lateral
            - barrier.muCoefficient;
        points.push([
            reducedState.x + c * radial - s * lateral,
            reducedState.y + s * radial + c * lateral
        ]);
    }
    return {
        type: 'dpcbf-parabola',
        obstacleId: obstacle.id,
        points,
        relativeVelocityArrow: {
            start: [reducedState.x, reducedState.y],
            end: [
                reducedState.x + barrier.relativeVelocity[0],
                reducedState.y + barrier.relativeVelocity[1]
            ]
        },
        safetyShape: {
            center: [obstacle.x, obstacle.y],
            radii: [
                (obstacle.radiusX + config.robotRadius) * config.safetyScale
                    + barrier.reactionBuffer,
                (obstacle.radiusY + config.robotRadius) * config.safetyScale
                    + barrier.reactionBuffer
            ],
            yaw: obstacle.yaw
        }
    };
}

export function buildG1DpcbfConstraint(
    reducedState,
    obstacleInput,
    options = {}
) {
    const config = { ...G1_DPCBF_DEFAULTS, ...options };
    const obstacle = obstacleInput.radiusX == null
        ? normalizeG1Obstacle(obstacleInput)
        : obstacleInput;
    const step = positive(config.finiteDifferenceStep, 1e-4);
    const barrier = evaluateG1DpcbfBarrier(reducedState, obstacle, config);

    const driftDerivative = centeredDifference(
        () => {
            const next = advanceDrift(reducedState, obstacle, step);
            return evaluateG1DpcbfBarrier(next.state, next.obstacle, config).h;
        },
        () => {
            const previous = advanceDrift(reducedState, obstacle, -step);
            return evaluateG1DpcbfBarrier(previous.state, previous.obstacle, config).h;
        },
        step
    );
    const accelerationGradient = centeredDifference(
        () => evaluateG1DpcbfBarrier({
            ...reducedState,
            forwardSpeed: reducedState.forwardSpeed + step
        }, obstacle, config).h,
        () => evaluateG1DpcbfBarrier({
            ...reducedState,
            forwardSpeed: reducedState.forwardSpeed - step
        }, obstacle, config).h,
        step
    );
    const yawGradient = centeredDifference(
        () => evaluateG1DpcbfBarrier({
            ...reducedState,
            heading: reducedState.heading + step
        }, obstacle, config).h,
        () => evaluateG1DpcbfBarrier({
            ...reducedState,
            heading: reducedState.heading - step
        }, obstacle, config).h,
        step
    );

    return {
        label: `g1_dpcbf_${obstacle.id}`,
        obstacleId: obstacle.id,
        A: [accelerationGradient, yawGradient],
        b: driftDerivative + config.alpha * barrier.h,
        h: barrier.h,
        driftDerivative,
        barrier,
        overlay: createOverlay(reducedState, obstacle, barrier, config)
    };
}

function selectObstacles(reducedState, obstacles, config) {
    return obstacles
        .map((obstacle, index) => normalizeG1Obstacle(obstacle, index))
        .map((obstacle) => ({
            obstacle,
            barrier: evaluateG1DpcbfBarrier(reducedState, obstacle, config)
        }))
        .sort((left, right) => {
            const clearanceDelta =
                left.barrier.safetyClearance - right.barrier.safetyClearance;
            return Math.abs(clearanceDelta) > 1e-12
                ? clearanceDelta
                : left.obstacle.id.localeCompare(right.obstacle.id);
        })
        .filter((entry) => entry.barrier.safetyClearance <= config.activationDistance)
        .slice(0, Math.max(0, Math.floor(config.maxObstacles)))
        .map((entry) => entry.obstacle);
}

function wrapAngle(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function stableDirection(id) {
    let hash = 0;
    for (const character of String(id)) {
        hash = ((hash << 5) - hash + character.charCodeAt(0)) | 0;
    }
    return hash % 2 === 0 ? 1 : -1;
}

function scoreEscapeDirection(
    reducedState,
    obstacles,
    desiredForward,
    direction,
    config
) {
    const step = positive(config.escapePredictionStep, 0.3);
    const horizon = Math.max(step, positive(config.escapePredictionHorizon, 1.8));
    const yawRate = direction * positive(config.escapeYawRate, 0.85);
    let x = reducedState.x;
    let y = reducedState.y;
    let heading = reducedState.heading;
    let speed = reducedState.forwardSpeed;
    const targetSpeed = clamp(
        Math.max(0.28, desiredForward),
        0.28,
        config.maxForwardSpeed
    );
    let score = Infinity;

    for (let time = step; time <= horizon + EPS; time += step) {
        speed += clamp(
            targetSpeed - speed,
            config.minAcceleration * step,
            config.maxAcceleration * step
        );
        heading += yawRate * step;
        x += speed * Math.cos(heading) * step;
        y += speed * Math.sin(heading) * step;
        for (const obstacle of obstacles) {
            const obstacleX = obstacle.x + obstacle.vx * time;
            const obstacleY = obstacle.y + obstacle.vy * time;
            const dx = obstacleX - x;
            const dy = obstacleY - y;
            const bearing = Math.atan2(dy, dx);
            const clearance =
                Math.hypot(dx, dy)
                - obstacleSupportRadius(obstacle, bearing)
                - config.robotRadius;
            score = Math.min(score, clearance);
        }
    }
    return score;
}

function chooseEscapeDirection(
    reducedState,
    constraints,
    selectedObstacles,
    desiredForward,
    config
) {
    if (!config.escapeSteeringEnabled) return null;
    const timeLimit = positive(config.escapeTimeToContact, 3.2);
    const minimumClosingSpeed = positive(
        config.escapeMinimumClosingSpeed,
        0.12
    );
    const threats = constraints
        .filter((constraint) => (
            constraint.barrier.closingSpeed >= minimumClosingSpeed
        ))
        .map((constraint) => ({
            constraint,
            timeToContact:
                Math.max(0, constraint.barrier.physicalClearance)
                / constraint.barrier.closingSpeed
        }))
        .filter((entry) => entry.timeToContact <= timeLimit)
        .sort((left, right) => {
            const timeDelta = left.timeToContact - right.timeToContact;
            return Math.abs(timeDelta) > 1e-9
                ? timeDelta
                : left.constraint.obstacleId.localeCompare(
                    right.constraint.obstacleId
                );
        });
    if (threats.length === 0) return null;

    const leftScore = scoreEscapeDirection(
        reducedState,
        selectedObstacles,
        desiredForward,
        1,
        config
    );
    const rightScore = scoreEscapeDirection(
        reducedState,
        selectedObstacles,
        desiredForward,
        -1,
        config
    );
    let direction;
    if (Math.abs(leftScore - rightScore) > 0.04) {
        direction = leftScore > rightScore ? 1 : -1;
    } else {
        const bearingError = wrapAngle(
            threats[0].constraint.barrier.bearing - reducedState.heading
        );
        direction = Math.abs(bearingError) > 0.04
            ? (bearingError > 0 ? -1 : 1)
            : stableDirection(threats[0].constraint.obstacleId);
    }
    return {
        direction,
        obstacleId: threats[0].constraint.obstacleId,
        timeToContact: threats[0].timeToContact,
        leftScore,
        rightScore
    };
}

function predictCommandClearance(
    reducedState,
    estimatedYawRate,
    obstacles,
    command,
    config
) {
    const step = positive(config.predictiveStep, 0.15);
    const horizon = Math.max(step, positive(config.predictiveHorizon, 3));
    const forwardLag = positive(config.predictiveForwardLagSeconds, 1.5);
    const yawLag = positive(config.predictiveYawLagSeconds, 1.3);
    const minimumAcceleration = finite(
        config.predictiveMinAcceleration,
        -0.35
    );
    const maximumAcceleration = finite(
        config.predictiveMaxAcceleration,
        0.45
    );
    const maximumYawAcceleration = positive(
        config.predictiveMaxYawAcceleration,
        0.55
    );
    const requiredDistance =
        positive(config.contactRobotRadius, 0.28)
        + positive(config.contactObstacleRadius, 0.28);
    let x = reducedState.x;
    let y = reducedState.y;
    let heading = reducedState.heading;
    let speed = reducedState.forwardSpeed;
    let yawRate = finite(estimatedYawRate, 0);
    let minimumClearance = Infinity;

    for (let time = step; time <= horizon + EPS; time += step) {
        speed += clamp(
            (command[0] - speed) / forwardLag,
            minimumAcceleration,
            maximumAcceleration
        ) * step;
        yawRate += clamp(
            (command[2] - yawRate) / yawLag,
            -maximumYawAcceleration,
            maximumYawAcceleration
        ) * step;
        heading += yawRate * step;
        x += speed * Math.cos(heading) * step;
        y += speed * Math.sin(heading) * step;

        for (const obstacle of obstacles) {
            const obstacleX = obstacle.x + obstacle.vx * time;
            const obstacleY = obstacle.y + obstacle.vy * time;
            minimumClearance = Math.min(
                minimumClearance,
                Math.hypot(obstacleX - x, obstacleY - y) - requiredDistance
            );
        }
    }
    return minimumClearance;
}

function uniqueNumbers(values) {
    const output = [];
    for (const value of values) {
        if (!output.some((existing) => Math.abs(existing - value) <= 1e-9)) {
            output.push(value);
        }
    }
    return output;
}

function choosePredictiveSafetyCommand(
    reducedState,
    estimatedYawRate,
    obstacles,
    filteredCommand,
    nominalCommand,
    config
) {
    const baseClearance = predictCommandClearance(
        reducedState,
        estimatedYawRate,
        obstacles,
        filteredCommand,
        config
    );
    const triggerClearance = Math.max(
        0,
        finite(config.predictiveTriggerClearance, 0.45)
    );
    if (
        !config.predictiveSafetyEnabled
        || baseClearance >= triggerClearance
    ) {
        return {
            command: [...filteredCommand],
            active: false,
            baseClearance,
            selectedClearance: baseClearance,
            candidateCount: 1,
            safeCandidateFound: true,
            accelerationGuardActive: false,
            guardedCandidateCount: 0
        };
    }

    const forwardCandidates = uniqueNumbers([
        filteredCommand[0],
        nominalCommand[0],
        ...(config.predictiveMeasuredSpeedCandidateEnabled
            ? [reducedState.forwardSpeed]
            : []),
        config.minForwardSpeed,
        0,
        0.2,
        0.4,
        0.6,
        config.maxForwardSpeed
    ].map((value) => clamp(
        finite(value, 0),
        config.minForwardSpeed,
        config.maxForwardSpeed
    )));
    const yawCandidates = uniqueNumbers([
        filteredCommand[2],
        nominalCommand[2],
        -config.maxYawRate,
        -0.72 * config.maxYawRate,
        -0.4 * config.maxYawRate,
        0,
        0.4 * config.maxYawRate,
        0.72 * config.maxYawRate,
        config.maxYawRate
    ].map((value) => clamp(
        finite(value, 0),
        -config.maxYawRate,
        config.maxYawRate
    )));
    const targetClearance = Math.max(
        0,
        finite(config.predictiveTargetClearance, 0.3)
    );
    let bestSafe = null;
    let bestEmergency = null;
    let candidateCount = 0;
    const currentForward = clamp(
        reducedState.forwardSpeed,
        config.minForwardSpeed,
        config.maxForwardSpeed
    );
    const turnTrackingTolerance = Math.max(
        0,
        finite(config.predictiveTurnTrackingTolerance, 0.18)
    );
    const accelerationGuardActive = baseClearance < targetClearance;
    let guardedCandidateCount = 0;

    for (const forward of forwardCandidates) {
        for (const yawRate of yawCandidates) {
            candidateCount += 1;
            const command = [
                forward,
                filteredCommand[1],
                yawRate
            ];
            const clearance = predictCommandClearance(
                reducedState,
                estimatedYawRate,
                obstacles,
                command,
                config
            );
            const trackingCost =
                0.7 * (command[0] - filteredCommand[0]) ** 2
                + 0.16 * (command[2] - filteredCommand[2]) ** 2
                - 0.08 * command[0];
            const accelerationGuarded =
                accelerationGuardActive
                && Math.abs(yawRate - estimatedYawRate)
                    > turnTrackingTolerance
                && forward > currentForward + 0.01;
            guardedCandidateCount += Number(accelerationGuarded);
            if (
                !accelerationGuarded
                && clearance >= targetClearance
                && (
                    !bestSafe
                    || trackingCost < bestSafe.trackingCost - 1e-12
                )
            ) {
                bestSafe = { command, clearance, trackingCost };
            }
            const emergencyScore = clearance;
            if (
                !accelerationGuarded
                && (
                    !bestEmergency
                    || emergencyScore > bestEmergency.emergencyScore + 1e-12
                    || (
                        Math.abs(emergencyScore - bestEmergency.emergencyScore)
                            <= 1e-12
                        && command[0] < bestEmergency.command[0] - 1e-12
                    )
                )
            ) {
                bestEmergency = { command, clearance, emergencyScore };
            }
        }
    }
    const selected = bestSafe ?? bestEmergency;
    return {
        command: [...selected.command],
        active: true,
        baseClearance,
        selectedClearance: selected.clearance,
        candidateCount,
        safeCandidateFound: Boolean(bestSafe),
        accelerationGuardActive,
        guardedCandidateCount
    };
}

function emptyDiagnostics(config, nominalCommand, command, reducedState, status) {
    return {
        enabled: Boolean(config.enabled),
        status,
        nominalCommand: [...nominalCommand],
        filteredCommand: [...command],
        reducedState: { ...reducedState },
        parameters: {
            kLambda: config.kLambda,
            kMu: config.kMu,
            alpha: config.alpha,
            safetyScale: config.safetyScale,
            robotRadius: config.robotRadius,
            policyLagSeconds: config.policyLagSeconds,
            positionUncertainty: config.positionUncertainty,
            commandLeadSeconds: config.commandLeadSeconds
        },
        nominalControl: [0, nominalCommand[2]],
        qpReferenceControl: [0, nominalCommand[2]],
        filteredControl: [0, command[2]],
        appliedCommandControl: [0, command[2]],
        escapeSteering: {
            active: false,
            direction: 0,
            obstacleId: null,
            timeToContact: null,
            leftScore: null,
            rightScore: null
        },
        predictiveSafety: {
            active: false,
            baseClearance: null,
            selectedClearance: null,
            candidateCount: 0,
            safeCandidateFound: false,
            accelerationGuardActive: false,
            guardedCandidateCount: 0,
            contactRadius:
                config.contactRobotRadius + config.contactObstacleRadius
        },
        constraints: [],
        overlays: [],
        constrainedObstacleIds: [],
        bindingObstacleIds: [],
        violatedObstacleIds: [],
        activeObstacleIds: [],
        minBarrier: null,
        nearestClearance: null,
        minimumConstraintMargin: null,
        intervention: Math.hypot(
            command[0] - nominalCommand[0],
            command[1] - nominalCommand[1],
            command[2] - nominalCommand[2]
        ),
        slack: 0,
        feasible: true,
        lateralCommandMode: 'pass-through'
    };
}

export class UnitreeG1DpcbfFilter {
    constructor(options = {}) {
        this.config = {
            ...G1_DPCBF_DEFAULTS,
            ...options,
            qpWeights: [...(options.qpWeights ?? G1_DPCBF_DEFAULTS.qpWeights)]
        };
        this.estimatedForwardSpeed = null;
        this.estimatedYawRate = 0;
        this.lastDiagnostics = null;
        this.escapeDirection = 0;
        this.escapeHoldRemaining = 0;
        this.escapeObstacleId = null;
    }

    setEnabled(enabled) {
        const next = Boolean(enabled);
        if (next !== Boolean(this.config.enabled)) {
            this.estimatedForwardSpeed = null;
            this.estimatedYawRate = 0;
            this.escapeDirection = 0;
            this.escapeHoldRemaining = 0;
            this.escapeObstacleId = null;
        }
        this.config.enabled = next;
    }

    setParameters(parameters = {}) {
        const supported = [
            'kLambda',
            'kMu',
            'alpha',
            'safetyScale',
            'robotRadius',
            'activationDistance',
            'policyLagSeconds',
            'positionUncertainty',
            'commandLeadSeconds',
            'escapeTimeToContact',
            'escapeMinimumClosingSpeed',
            'escapeYawRate',
            'escapeHoldSeconds',
            'escapePredictionHorizon',
            'predictiveHorizon',
            'predictiveStep',
            'predictiveTriggerClearance',
            'predictiveTargetClearance',
            'predictiveForwardLagSeconds',
            'predictiveYawLagSeconds',
            'predictiveTurnTrackingTolerance'
        ];
        for (const key of supported) {
            if (Number.isFinite(parameters[key])) {
                this.config[key] = parameters[key];
            }
        }
    }

    reset(robotState = null) {
        this.estimatedForwardSpeed = robotState
            ? reduceG1RobotState(robotState).forwardSpeed
            : null;
        this.estimatedYawRate = finite(
            robotState?.yawRate ?? robotState?.omega,
            0
        );
        this.lastDiagnostics = null;
        this.escapeDirection = 0;
        this.escapeHoldRemaining = 0;
        this.escapeObstacleId = null;
    }

    filter({
        nominalCommand,
        robotState,
        obstacles = [],
        dt = DEFAULT_DT
    }) {
        const nominal = cloneCommand(nominalCommand);
        const timestep = positive(dt, DEFAULT_DT);
        const measuredState = reduceG1RobotState(robotState);

        if (!this.config.enabled) {
            this.estimatedForwardSpeed = nominal[0];
            this.estimatedYawRate = nominal[2];
            const diagnostics = emptyDiagnostics(
                this.config,
                nominal,
                nominal,
                { ...measuredState, forwardSpeed: nominal[0] },
                'disabled'
            );
            this.lastDiagnostics = diagnostics;
            return { command: [...nominal], diagnostics };
        }

        const measuredYawRate =
            robotState?.yawRate ?? robotState?.omega;
        if (Number.isFinite(measuredYawRate)) {

            this.estimatedYawRate = measuredYawRate;
        }
        if (this.estimatedForwardSpeed == null) {
            this.estimatedForwardSpeed = measuredState.forwardSpeed;
        } else {
            const blend = clamp(
                finite(this.config.measurementBlend, 0.18),
                0,
                1
            );
            this.estimatedForwardSpeed +=
                (measuredState.forwardSpeed - this.estimatedForwardSpeed) * blend;
        }
        const conservativeForwardSpeed = nominal[0] < 0
            ? Math.min(this.estimatedForwardSpeed, measuredState.forwardSpeed)
            : Math.max(this.estimatedForwardSpeed, measuredState.forwardSpeed);
        const reducedState = {
            ...measuredState,
            forwardSpeed: clamp(
                conservativeForwardSpeed,
                this.config.minMeasuredForwardSpeed,
                this.config.maxMeasuredForwardSpeed
            )
        };
        const selectedObstacles = selectObstacles(
            reducedState,
            obstacles,
            this.config
        );

        if (selectedObstacles.length === 0) {
            this.estimatedForwardSpeed = clamp(
                nominal[0],
                this.config.minForwardSpeed,
                this.config.maxForwardSpeed
            );
            const command = [...nominal];
            const yawLag = positive(
                this.config.predictiveYawLagSeconds,
                1.3
            );
            const maxYawAcceleration = positive(
                this.config.predictiveMaxYawAcceleration,
                0.55
            );
            this.estimatedYawRate += clamp(
                (command[2] - this.estimatedYawRate) / yawLag,
                -maxYawAcceleration,
                maxYawAcceleration
            ) * timestep;
            const diagnostics = emptyDiagnostics(
                this.config,
                nominal,
                command,
                reducedState,
                'clear'
            );
            this.lastDiagnostics = diagnostics;
            return { command, diagnostics };
        }

        const constraints = selectedObstacles.map((obstacle) =>
            buildG1DpcbfConstraint(reducedState, obstacle, this.config)
        );
        const desiredForward = clamp(
            nominal[0],
            this.config.minForwardSpeed,
            this.config.maxForwardSpeed
        );
        const nominalAcceleration = clamp(
            (desiredForward - reducedState.forwardSpeed) / timestep,
            this.config.minAcceleration,
            this.config.maxAcceleration
        );
        const nominalYawRate = clamp(
            nominal[2],
            -this.config.maxYawRate,
            this.config.maxYawRate
        );
        const escape = chooseEscapeDirection(
            reducedState,
            constraints,
            selectedObstacles,
            desiredForward,
            this.config
        );
        if (escape) {
            if (
                this.escapeHoldRemaining <= 0
                || this.escapeObstacleId !== escape.obstacleId
            ) {
                this.escapeDirection = escape.direction;
                this.escapeObstacleId = escape.obstacleId;
            }
            this.escapeHoldRemaining = positive(
                this.config.escapeHoldSeconds,
                1.1
            );
        } else {
            this.escapeHoldRemaining = Math.max(
                0,
                this.escapeHoldRemaining - timestep
            );
            if (this.escapeHoldRemaining <= 0) {
                this.escapeDirection = 0;
                this.escapeObstacleId = null;
            }
        }
        const qpYawReference = this.escapeDirection === 0
            ? nominalYawRate
            : clamp(
                this.escapeDirection
                    * positive(this.config.escapeYawRate, 0.85),
                -this.config.maxYawRate,
                this.config.maxYawRate
            );
        const solve = solveBoxConstrainedQP(
            [nominalAcceleration, qpYawReference],
            constraints.map((constraint) => ({
                A: constraint.A,
                b: constraint.b,
                label: constraint.label
            })),
            {
                min: [this.config.minAcceleration, -this.config.maxYawRate],
                max: [this.config.maxAcceleration, this.config.maxYawRate]
            },
            {
                weights: this.config.qpWeights,
                slack: {
                    enabled: this.config.slackEnabled,
                    weight: this.config.slackWeight
                }
            }
        );
        const filteredAcceleration = solve.u[0];
        const filteredYawRate = solve.u[1];
        const accelerationIntervention =
            Math.abs(filteredAcceleration - nominalAcceleration)
            > this.config.interventionTolerance;
        const commandLeadSeconds = Math.max(
            timestep,
            finite(this.config.commandLeadSeconds, timestep)
        );
        const filteredForward = clamp(
            accelerationIntervention
                ? reducedState.forwardSpeed
                    + filteredAcceleration * commandLeadSeconds
                : nominal[0],
            this.config.minForwardSpeed,
            this.config.maxForwardSpeed
        );
        const qpIntervention = Math.hypot(
            filteredForward - nominal[0],
            filteredYawRate - nominal[2]
        );
        const qpFiltering =
            qpIntervention > this.config.interventionTolerance;
        const lateralLimit = Math.min(
            this.config.maxLateralSpeed,
            Math.abs(nominal[1])
        );
        const filteredLateral = clamp(
            nominal[1],
            -lateralLimit,
            lateralLimit
        ) * (qpFiltering ? this.config.lateralScaleWhileFiltering : 1);
        const qpCommand = [
            filteredForward,
            filteredLateral,
            filteredYawRate
        ];
        const predictiveSafety = choosePredictiveSafetyCommand(
            reducedState,
            this.estimatedYawRate,
            selectedObstacles,
            qpCommand,
            nominal,
            this.config
        );
        const command = [
            clamp(
                predictiveSafety.command[0],
                this.config.minForwardSpeed,
                this.config.maxForwardSpeed
            ),
            clamp(
                predictiveSafety.command[1],
                -this.config.maxLateralSpeed,
                this.config.maxLateralSpeed
            ),
            clamp(
                predictiveSafety.command[2],
                -this.config.maxYawRate,
                this.config.maxYawRate
            )
        ];
        const rawIntervention = Math.hypot(
            command[0] - nominal[0],
            command[1] - nominal[1],
            command[2] - nominal[2]
        );
        const filtering =
            rawIntervention > this.config.interventionTolerance;
        const appliedAcceleration = predictiveSafety.active
            ? clamp(
                (command[0] - reducedState.forwardSpeed)
                    / positive(this.config.predictiveForwardLagSeconds, 1.5),
                finite(this.config.predictiveMinAcceleration, -0.35),
                finite(this.config.predictiveMaxAcceleration, 0.45)
            )
            : filteredAcceleration;
        this.estimatedForwardSpeed = clamp(
            reducedState.forwardSpeed + appliedAcceleration * timestep,
            this.config.minForwardSpeed,
            this.config.maxForwardSpeed
        );
        const yawLag = positive(
            this.config.predictiveYawLagSeconds,
            1.3
        );
        const maxYawAcceleration = positive(
            this.config.predictiveMaxYawAcceleration,
            0.55
        );
        this.estimatedYawRate += clamp(
            (command[2] - this.estimatedYawRate) / yawLag,
            -maxYawAcceleration,
            maxYawAcceleration
        ) * timestep;

        const bindingLabels = new Set(solve.activeSet);
        const constraintDiagnostics = constraints.map((constraint) => {
            const rawMargin =
                constraint.A[0] * filteredAcceleration
                + constraint.A[1] * filteredYawRate
                + constraint.b;
            const binding = bindingLabels.has(constraint.label);
            return {
                obstacleId: constraint.obstacleId,
                h: constraint.h,
                margin: rawMargin,
                A: [...constraint.A],
                b: constraint.b,
                driftDerivative: constraint.driftDerivative,
                physicalClearance: constraint.barrier.physicalClearance,
                safetyClearance: constraint.barrier.safetyClearance,
                physicalRadius: constraint.barrier.physicalRadius,
                safetyRadius: constraint.barrier.safetyRadius,
                relativeVelocity: [...constraint.barrier.relativeVelocity],
                radialVelocity: constraint.barrier.radialVelocity,
                lateralVelocity: constraint.barrier.lateralVelocity,
                closingSpeed: constraint.barrier.closingSpeed,
                reactionBuffer: constraint.barrier.reactionBuffer,
                colliding: constraint.barrier.colliding,
                insideSafetyRadius: constraint.barrier.insideSafetyRadius,
                binding,
                violated: rawMargin < 0,

                active: binding
            };
        });
        const constrainedObstacleIds = constraints
            .map((constraint) => constraint.obstacleId);
        const bindingObstacleIds = constraintDiagnostics
            .filter((constraint) => constraint.binding)
            .map((constraint) => constraint.obstacleId);
        const violatedObstacleIds = constraintDiagnostics
            .filter((constraint) => constraint.violated)
            .map((constraint) => constraint.obstacleId);
        const collision = constraintDiagnostics.some((constraint) => constraint.colliding);
        const infeasible = !solve.feasible || solve.slack > 1e-3;
        const status = collision
            ? 'collision'
            : infeasible
                ? 'infeasible'
                : filtering
                    ? 'filtering'
                    : 'monitoring';
        const diagnostics = {
            enabled: true,
            status,
            nominalCommand: [...nominal],
            filteredCommand: [...command],
            reducedState: { ...reducedState },
            parameters: {
                kLambda: this.config.kLambda,
                kMu: this.config.kMu,
                alpha: this.config.alpha,
                safetyScale: this.config.safetyScale,
                robotRadius: this.config.robotRadius,
                policyLagSeconds: this.config.policyLagSeconds,
                positionUncertainty: this.config.positionUncertainty,
                commandLeadSeconds: this.config.commandLeadSeconds
            },
            nominalControl: [nominalAcceleration, nominalYawRate],
            qpReferenceControl: [nominalAcceleration, qpYawReference],
            filteredControl: [filteredAcceleration, filteredYawRate],
            qpFilteredCommand: [...qpCommand],
            appliedCommandControl: [appliedAcceleration, command[2]],
            escapeSteering: {
                active: this.escapeDirection !== 0,
                direction: this.escapeDirection,
                obstacleId: this.escapeObstacleId,
                timeToContact: escape?.timeToContact ?? null,
                leftScore: escape?.leftScore ?? null,
                rightScore: escape?.rightScore ?? null
            },
            predictiveSafety: {
                active: predictiveSafety.active,
                baseClearance: predictiveSafety.baseClearance,
                selectedClearance: predictiveSafety.selectedClearance,
                candidateCount: predictiveSafety.candidateCount,
                safeCandidateFound: predictiveSafety.safeCandidateFound,
                accelerationGuardActive:
                    predictiveSafety.accelerationGuardActive,
                guardedCandidateCount:
                    predictiveSafety.guardedCandidateCount,
                contactRadius:
                    this.config.contactRobotRadius
                    + this.config.contactObstacleRadius
            },
            constraints: constraintDiagnostics,
            overlays: constraints.map((constraint) => constraint.overlay),
            constrainedObstacleIds,
            bindingObstacleIds,
            violatedObstacleIds,

            activeObstacleIds: bindingObstacleIds,
            minBarrier: Math.min(...constraints.map((constraint) => constraint.h)),
            nearestClearance: Math.min(
                ...constraintDiagnostics.map((constraint) => constraint.physicalClearance)
            ),
            minimumConstraintMargin: Math.min(
                ...constraintDiagnostics.map((constraint) => constraint.margin)
            ),
            intervention: Math.hypot(
                command[0] - nominal[0],
                command[1] - nominal[1],
                command[2] - nominal[2]
            ),
            slack: solve.slack,
            feasible: solve.feasible && solve.slack <= 1e-3,
            qpStatus: solve.status,
            activeSet: [...solve.activeSet],
            lateralCommandMode: filtering ? 'attenuated' : 'pass-through'
        };
        this.lastDiagnostics = diagnostics;
        return { command, diagnostics };
    }

    getDiagnostics() {
        return this.lastDiagnostics;
    }
}

export function filterG1JoystickCommand(input, options = {}) {
    return new UnitreeG1DpcbfFilter(options).filter(input);
}
