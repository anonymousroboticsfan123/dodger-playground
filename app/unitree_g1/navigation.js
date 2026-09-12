const DEFAULTS = Object.freeze({
    reachThreshold: 0.9,
    distanceGain: 0.75,
    headingGain: 1.8,
    maxForwardSpeed: 0.8,
    maxYawRate: 1.0,
    maxForwardAcceleration: 1.25,
    maxYawAcceleration: 3.5,
    turnInPlaceAngle: 1.05
});

export function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

export function wrapAngle(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function moveToward(value, target, maxDelta) {
    const delta = target - value;
    if (Math.abs(delta) <= maxDelta) return target;
    return value + Math.sign(delta) * maxDelta;
}

export function computePointNavigationCommand({
    position,
    yaw,
    goal,
    previousCommand = [0, 0, 0],
    dt = 0.02,
    config = {}
}) {
    const options = { ...DEFAULTS, ...config };
    const dx = goal[0] - position[0];
    const dy = goal[1] - position[1];
    const distance = Math.hypot(dx, dy);
    const desiredHeading = distance > 1e-9 ? Math.atan2(dy, dx) : yaw;
    const headingError = wrapAngle(desiredHeading - yaw);
    const reached = distance <= options.reachThreshold;

    let targetForward = reached
        ? 0
        : clamp(options.distanceGain * distance, 0, options.maxForwardSpeed);
    const alignment = Math.max(0, Math.cos(headingError));
    targetForward *= alignment * alignment;
    if (Math.abs(headingError) >= options.turnInPlaceAngle) {
        targetForward = 0;
    }

    const targetYawRate = reached
        ? 0
        : clamp(options.headingGain * headingError, -options.maxYawRate, options.maxYawRate);
    const forward = moveToward(
        previousCommand[0] ?? 0,
        targetForward,
        options.maxForwardAcceleration * dt
    );
    const yawRate = moveToward(
        previousCommand[2] ?? 0,
        targetYawRate,
        options.maxYawAcceleration * dt
    );

    return {
        command: [forward, 0, yawRate],
        desiredHeading,
        distance,
        headingError,
        reached
    };
}

export function sampleReachableGoal(
    origin,
    random = Math.random,
    { minRadius = 2.5, maxRadius = 4.5, worldLimit = 8 } = {}
) {
    const angle = random() * Math.PI * 2;
    const radius = minRadius + (maxRadius - minRadius) * random();
    return [
        clamp(origin[0] + radius * Math.cos(angle), -worldLimit, worldLimit),
        clamp(origin[1] + radius * Math.sin(angle), -worldLimit, worldLimit)
    ];
}

export const POINT_NAVIGATION_DEFAULTS = DEFAULTS;
