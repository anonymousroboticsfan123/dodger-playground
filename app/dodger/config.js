

export const DODGER_CONFIG = Object.freeze({
    highLevelDt: 0.1,
    controlDt: 0.02,
    robotRadius: 0.30,
    goalRadius: 0.35,
    goalHeadingTolerance: 10 * Math.PI / 180,
    arenaHalfSize: 5,
    hardBoundary: 10,
    obstacleCount: 20,
    obstacleRadiusRange: [0.2, 0.3],
    obstacleSpeedRange: [0, 0.6],
    initialClearance: 1,
    detectionRadius: 5,
    maxObstacles: 10,
    actionMin: [-4, -2, -1],
    actionMax: [4, 2, 1],
    commandMin: [-1, -1, -1],
    commandMax: [2, 1, 1],
    safetyScale: 1.05,
    epsV: 0.05,
    epsD: 0.05,
    kLambda: 0.5,
    kMu: 1,
    alpha: 2,
    distanceAlpha1: 2,
    distanceAlpha2: 2,
    slackWeight: 1e6,
    qpMaxIterations: 40,
    qpTolerance: 1e-5,
    qpRegularization: 1e-6,
    defaultSeed: 42
});

export const NAVIGATION_POLICIES = Object.freeze([
    { id: 'distance_cbf', label: 'Distance CBF', kind: 'qp', available: true },
    { id: 'c3bf', label: 'C3BF-QP', kind: 'qp', available: true },
    { id: 'dpcbf', label: 'DPCBF-QP', kind: 'qp', available: true },
    { id: 'distance_rl', label: 'Distance-RL', kind: 'learned', graphAttention: true, barrier: 'distance_cbf', available: true,
        encoder: '../../assets/dodger/distance_rl/gat_encoder.onnx',
        head: '../../assets/dodger/distance_rl/policy_head.onnx' },
    { id: 'c3bf_rl', label: 'C3BF-RL', kind: 'learned', graphAttention: true, barrier: 'c3bf', available: true,
        encoder: '../../assets/dodger/c3bf_rl/gat_encoder.onnx',
        head: '../../assets/dodger/c3bf_rl/policy_head.onnx' },
    { id: 'dodger', label: 'DODGER', kind: 'learned', graphAttention: true, barrier: 'dpcbf', available: true,
        encoder: '../../assets/dodger/gat_encoder.onnx',
        head: '../../assets/dodger/policy_head.onnx' }
]);

export const clamp = (value, low, high) => Math.min(high, Math.max(low, value));
export const wrapAngle = (angle) => Math.atan2(Math.sin(angle), Math.cos(angle));

export function bodyVelocity(robot) {
    const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw);
    return [
        c * robot.worldVelocity[0] + s * robot.worldVelocity[1],
        -s * robot.worldVelocity[0] + c * robot.worldVelocity[1]
    ];
}

export function clipAction(action, config = DODGER_CONFIG) {
    return action.map((value, i) => clamp(value, config.actionMin[i], config.actionMax[i]));
}

export function integrateCommand(command, action, dt, config = DODGER_CONFIG) {
    return [command[0] + action[0] * dt, command[1] + action[1] * dt, action[2]]
        .map((value, i) => clamp(value, config.commandMin[i], config.commandMax[i]));
}
