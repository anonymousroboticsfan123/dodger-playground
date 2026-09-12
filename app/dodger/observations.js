import { DODGER_CONFIG, bodyVelocity, wrapAngle } from './config.js?v=dodger-3';

export function selectObstacles(robot, obstacles, config = DODGER_CONFIG) {
    return obstacles.map((obstacle, index) => {
        const px = obstacle.x - robot.position[0], py = obstacle.y - robot.position[1];
        const vx = obstacle.vx - robot.worldVelocity[0], vy = obstacle.vy - robot.worldVelocity[1];
        const distance = Math.hypot(px, py);
        const alignment = -(px * vx + py * vy) / Math.max(distance * Math.hypot(vx, vy), 1e-6);
        return { obstacle, index, distance, score: -alignment + 1e-4 * distance };
    }).filter((item) => item.distance <= config.detectionRadius)
        .sort((a, b) => a.score - b.score || a.index - b.index)
        .slice(0, config.maxObstacles).map((item) => item.obstacle);
}

export function buildNavigationObservation({ robot, goal, goalYaw, command, previousAction, obstacles }, config = DODGER_CONFIG) {
    const selected = selectObstacles(robot, obstacles, config);
    const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw);
    const toBody = (x, y) => [c * x + s * y, -s * x + c * y];
    const velocity = bodyVelocity(robot);
    const goalLocal = toBody(goal[0] - robot.position[0], goal[1] - robot.position[1]);
    const nodes = [1, 0, 0, 0, 0, config.robotRadius, 0, 0,
        0, 0, 1, ...goalLocal, config.goalRadius, -velocity[0], -velocity[1]];
    for (const obstacle of selected) {
        nodes.push(0, 1, 0,
            ...toBody(obstacle.x - robot.position[0], obstacle.y - robot.position[1]),
            obstacle.radius,
            ...toBody(obstacle.vx - robot.worldVelocity[0], obstacle.vy - robot.worldVelocity[1]));
    }
    const headingError = wrapAngle(goalYaw - robot.yaw);
    const normalized = previousAction.map((value, i) => {
        const center = (config.actionMin[i] + config.actionMax[i]) / 2;
        const halfRange = (config.actionMax[i] - config.actionMin[i]) / 2;
        return Math.max(-1, Math.min(1, (value - center) / halfRange));
    });

    const localState = new Float32Array([
        ...goalLocal, Math.sin(headingError), Math.cos(headingError),
        ...velocity, robot.yawRate, ...command, ...normalized
    ]);
    return { nodes: new Float32Array(nodes), localState, selected, nodeCount: selected.length + 2 };
}

export function pdNavigationAction(robot, goal, goalYaw, config = DODGER_CONFIG) {
    const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw);
    const dx = goal[0] - robot.position[0], dy = goal[1] - robot.position[1];
    const [vf, vl] = bodyVelocity(robot);
    const distance = Math.hypot(dx, dy);
    const goalHeading = distance > 1 ? Math.atan2(dy, dx) : goalYaw;
    const desiredVf = Math.max(config.commandMin[0], Math.min(config.commandMax[0], 1.2 * (c * dx + s * dy)));
    const desiredVl = Math.max(config.commandMin[1], Math.min(config.commandMax[1], 1.2 * (-s * dx + c * dy)));
    return [2 * (desiredVf - vf), 2 * (desiredVl - vl), 1.8 * wrapAngle(goalHeading - robot.yaw)];
}

export function buildNavigationGraph({ robot, goal, obstacles }, config = DODGER_CONFIG) {
    const start = robot.position.slice(0,2);
    return [{ nodeId: 'goal', start, end: [...goal] }, ...selectObstacles(robot,obstacles,config)
        .map(obstacle => ({ nodeId: obstacle.id, start, end: [obstacle.x,obstacle.y] }))];
}
