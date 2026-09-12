import { DODGER_CONFIG, clamp } from './config.js?v=dodger-3';
import { evaluateBarrier } from './safety.js?v=dodger-3';

export function seededRandom(seed) {
    let state = seed >>> 0;
    return () => {
        state += 0x6D2B79F5;
        let t = state;
        t = Math.imul(t ^ t >>> 15, t | 1);
        t ^= t + Math.imul(t ^ t >>> 7, t | 61);
        return ((t ^ t >>> 14) >>> 0) / 4294967296;
    };
}

export function sampleGoal(random, robotPosition, obstacles, config = DODGER_CONFIG) {
    for (let attempt = 0; attempt < 100; attempt += 1) {
        const goal = [0, 1].map(() => (2 * random() - 1) * (config.arenaHalfSize - 0.6));
        const distance = Math.hypot(goal[0] - robotPosition[0], goal[1] - robotPosition[1]);
        if (distance < 2 || distance > 6) continue;
        if (obstacles.some((o) => Math.hypot(goal[0] - o.x, goal[1] - o.y) < config.goalRadius + o.radius + 0.2)) continue;
        return { goal, goalYaw: (2 * random() - 1) * Math.PI };
    }
    return { goal: [clamp(-robotPosition[0], -4, 4), clamp(-robotPosition[1] + 3, -4, 4)], goalYaw: 0 };
}

export function createScenario(seed = DODGER_CONFIG.defaultSeed, config = DODGER_CONFIG) {
    const random = seededRandom(seed);
    const robot = { position: [0, 0, 0.785], yaw: 0, yawRate: 0, worldVelocity: [0, 0, 0] };
    const obstacles = Array.from({ length: config.obstacleCount }, (_, index) => {
        const radius = config.obstacleRadiusRange[0] + random() * (config.obstacleRadiusRange[1] - config.obstacleRadiusRange[0]);
        const speed = config.obstacleSpeedRange[0] + random() * (config.obstacleSpeedRange[1] - config.obstacleSpeedRange[0]);
        const angle = (2 * random() - 1) * Math.PI;
        const obstacle = { id: `dodger-obstacle-${index + 1}`, radius, collisionRadius: radius,
            x: 0, y: 0, vx: speed * Math.cos(angle), vy: speed * Math.sin(angle), source: 'reference' };
        let placed = false;
        for (let attempt = 0; attempt < 1000; attempt += 1) {
            obstacle.x = (2 * random() - 1) * (config.arenaHalfSize - radius);
            obstacle.y = (2 * random() - 1) * (config.arenaHalfSize - radius);
            const value = evaluateBarrier(robot, obstacle, [0, 0, 0], config);
            if (value.physicalClearance >= config.initialClearance && value.h >= 0) { placed = true; break; }
        }
        if (!placed) throw new Error('Unable to sample a valid navigation scene.');
        return obstacle;
    });
    return { obstacles, ...sampleGoal(random, robot.position, obstacles, config), random };
}

export function advanceObstacles(obstacles, dt, config = DODGER_CONFIG) {
    for (const obstacle of obstacles) {
        const limit = config.arenaHalfSize - obstacle.radius;
        for (const [position, velocity] of [['x', 'vx'], ['y', 'vy']]) {
            obstacle[position] += obstacle[velocity] * dt;
            if (obstacle[position] < -limit || obstacle[position] > limit) {
                obstacle[position] = clamp(obstacle[position], -limit, limit);
                obstacle[velocity] *= -1;
            }
        }
    }
}

export function collisionAt(robot, obstacles, config = DODGER_CONFIG) {
    let contact = null;
    for (const obstacle of obstacles) {
        const clearance = Math.hypot(obstacle.x - robot.position[0], obstacle.y - robot.position[1])
            - config.robotRadius - obstacle.radius;
        if (clearance <= 0 && (!contact || clearance < contact.clearance)) contact = { obstacle, clearance };
    }
    return contact;
}
