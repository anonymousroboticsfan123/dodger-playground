import { clamp } from './navigation.js?v=dodger-3';

export const G1_OBSTACLE_DEFAULTS = Object.freeze({
    count: 24,
    radius: 0.42,
    robotRadius: 0.42,
    worldLimit: 7.45,

    speedCap: 0.64,
    referenceSpeedFloor: 0.58,
    randomSpeedFloor: 0.5,
    launchVelocityScale: 0.32,
    staticDragThreshold: 0.22,

    placementMargin: 0.92,

    robotLaunchDistance: 2.5,
    maxConcurrentInwardUserObstacles: 1,
    inwardUserThreatSpeed: 0.15
});

export const G1_COLLISION_FOOTPRINTS = Object.freeze({
    robotRadius: 0.28,
    obstacleRadius: 0.28
});

function clampMagnitude(vector, maximum) {
    const magnitude = Math.hypot(vector[0], vector[1]);
    if (magnitude <= maximum || magnitude < 1e-9) {
        return [vector[0] || 0, vector[1] || 0];
    }
    const scale = maximum / magnitude;
    return [(vector[0] * scale) || 0, (vector[1] * scale) || 0];
}

export function createG1Obstacle({
    id,
    x,
    y,
    vx = 0,
    vy = 0,
    radius = G1_OBSTACLE_DEFAULTS.radius,
    source = 'reference'
}) {
    const velocity = clampMagnitude([vx, vy], G1_OBSTACLE_DEFAULTS.speedCap);
    return {
        id,
        x,
        y,
        vx: velocity[0],
        vy: velocity[1],
        radius,
        source
    };
}

export function createReferenceObstacles() {

    const ringCount = 3;
    const actorsPerRing = G1_OBSTACLE_DEFAULTS.count / ringCount;
    const ringRadii = [3.75, 5.2, 6.65];
    const ringPhase = [Math.PI / 8, 0, Math.PI / 8];
    const speedRange =
        G1_OBSTACLE_DEFAULTS.speedCap
        - G1_OBSTACLE_DEFAULTS.referenceSpeedFloor;
    return Array.from({ length: G1_OBSTACLE_DEFAULTS.count }, (_, index) => {
        const ringIndex = Math.floor(index / actorsPerRing);
        const ringPosition = index % actorsPerRing;
        const angle =
            ringPhase[ringIndex]
            + (ringPosition * Math.PI * 2) / actorsPerRing;
        const speedStep = ((index * 5 + ringIndex * 2) % 7) / 6;
        const speed =
            G1_OBSTACLE_DEFAULTS.referenceSpeedFloor
            + speedRange * speedStep;
        const travelDirection = ringIndex % 2 === 0 ? 1 : -1;
        const headingVariation = ((ringPosition % 3) - 1) * 0.16;
        const travelHeading =
            angle + travelDirection * Math.PI / 2 + headingVariation;
        return createG1Obstacle({
            id: `g1-obstacle-${index + 1}`,
            x: ringRadii[ringIndex] * Math.cos(angle),
            y: ringRadii[ringIndex] * Math.sin(angle),
            vx: speed * Math.cos(travelHeading),
            vy: speed * Math.sin(travelHeading)
        });
    });
}

export function advanceG1Obstacles(
    obstacles,
    dt,
    { worldLimit = G1_OBSTACLE_DEFAULTS.worldLimit } = {}
) {
    for (const obstacle of obstacles) {
        obstacle.x += obstacle.vx * dt;
        obstacle.y += obstacle.vy * dt;

        const limit = worldLimit - obstacle.radius;
        if (obstacle.x < -limit) {
            obstacle.x = -limit;
            obstacle.vx = Math.abs(obstacle.vx);
        } else if (obstacle.x > limit) {
            obstacle.x = limit;
            obstacle.vx = -Math.abs(obstacle.vx);
        }
        if (obstacle.y < -limit) {
            obstacle.y = -limit;
            obstacle.vy = Math.abs(obstacle.vy);
        } else if (obstacle.y > limit) {
            obstacle.y = limit;
            obstacle.vy = -Math.abs(obstacle.vy);
        }
    }
    return obstacles;
}

export function getObstacleLaunchVelocity(start, current) {
    const dx = current[0] - start[0];
    const dy = current[1] - start[1];
    if (Math.hypot(dx, dy) < G1_OBSTACLE_DEFAULTS.staticDragThreshold) {
        return [0, 0];
    }
    return clampMagnitude(
        [
            -dx * G1_OBSTACLE_DEFAULTS.launchVelocityScale,
            -dy * G1_OBSTACLE_DEFAULTS.launchVelocityScale
        ],
        G1_OBSTACLE_DEFAULTS.speedCap
    );
}

export function canPlaceG1Obstacle(
    candidate,
    obstacles,
    robotPosition,
    {
        worldLimit = G1_OBSTACLE_DEFAULTS.worldLimit,
        robotRadius = G1_OBSTACLE_DEFAULTS.robotRadius,
        placementMargin = G1_OBSTACLE_DEFAULTS.placementMargin,
        robotLaunchDistance = null
    } = {}
) {
    const coordinateLimit = worldLimit - candidate.radius;
    if (
        Math.abs(candidate.x) > coordinateLimit
        || Math.abs(candidate.y) > coordinateLimit
    ) {
        return false;
    }
    const minimumRobotDistance = Number.isFinite(robotLaunchDistance)
        ? Math.max(0, robotLaunchDistance)
        : candidate.radius + robotRadius + placementMargin;
    if (
        Math.hypot(candidate.x - robotPosition[0], candidate.y - robotPosition[1])
        < minimumRobotDistance
    ) {
        return false;
    }
    return !obstacles.some((obstacle) => (
        obstacle.id !== candidate.id
        && Math.hypot(candidate.x - obstacle.x, candidate.y - obstacle.y)
            < candidate.radius + obstacle.radius + placementMargin
    ));
}

export function findG1ObstacleAt(obstacles, point, extraRadius = 0.18) {
    let nearest = null;
    let nearestDistance = Infinity;
    for (const obstacle of obstacles) {
        const distance = Math.hypot(obstacle.x - point[0], obstacle.y - point[1]);
        if (distance <= obstacle.radius + extraRadius && distance < nearestDistance) {
            nearest = obstacle;
            nearestDistance = distance;
        }
    }
    return nearest;
}

export function computeG1ObstacleClearance(
    robotPosition,
    obstacle,
    robotRadius = G1_OBSTACLE_DEFAULTS.robotRadius
) {
    return Math.hypot(
        obstacle.x - robotPosition[0],
        obstacle.y - robotPosition[1]
    ) - obstacle.radius - robotRadius;
}

export function getG1ObstacleCollisionRadius(
    obstacle = {},
    maximumRadius = G1_COLLISION_FOOTPRINTS.obstacleRadius
) {
    if (Number.isFinite(obstacle.collisionRadius)) {
        return Math.max(0, obstacle.collisionRadius);
    }
    const modelRadius = Number.isFinite(obstacle.radius)
        ? Math.max(0, obstacle.radius)
        : G1_COLLISION_FOOTPRINTS.obstacleRadius;
    return Math.min(modelRadius, Math.max(0, maximumRadius));
}

export function detectG1CircularCollision(
    robotPosition,
    obstacles,
    {
        robotRadius = G1_COLLISION_FOOTPRINTS.robotRadius,
        obstacleRadius = G1_COLLISION_FOOTPRINTS.obstacleRadius
    } = {}
) {
    let contact = null;
    for (const obstacle of obstacles) {
        const obstacleFootprint = getG1ObstacleCollisionRadius(
            obstacle,
            obstacleRadius
        );
        const requiredDistance = robotRadius + obstacleFootprint;
        const distance = Math.hypot(
            obstacle.x - robotPosition[0],
            obstacle.y - robotPosition[1]
        );
        const clearance = distance - requiredDistance;
        if (
            clearance <= 0
            && (!contact || clearance < contact.clearance)
        ) {
            contact = {
                obstacle,
                distance,
                requiredDistance,
                clearance
            };
        }
    }
    return contact;
}

export function getNearestG1Obstacle(
    robotPosition,
    obstacles,
    robotRadius = G1_OBSTACLE_DEFAULTS.robotRadius
) {
    let nearest = null;
    for (const obstacle of obstacles) {
        const clearance = computeG1ObstacleClearance(
            robotPosition,
            obstacle,
            robotRadius
        );
        if (!nearest || clearance < nearest.clearance) {
            nearest = { obstacle, clearance };
        }
    }
    return nearest;
}

export function sampleG1Obstacle(
    id,
    robotPosition,
    goal,
    obstacles,
    random = Math.random
) {
    const limit = G1_OBSTACLE_DEFAULTS.worldLimit - G1_OBSTACLE_DEFAULTS.radius;
    for (let attempt = 0; attempt < 100; attempt += 1) {
        const speed =
            G1_OBSTACLE_DEFAULTS.randomSpeedFloor
            + random() * (
                G1_OBSTACLE_DEFAULTS.speedCap
                - G1_OBSTACLE_DEFAULTS.randomSpeedFloor
            );
        const heading = random() * Math.PI * 2;
        const candidate = createG1Obstacle({
            id,
            x: clamp((random() * 2 - 1) * limit, -limit, limit),
            y: clamp((random() * 2 - 1) * limit, -limit, limit),
            vx: speed * Math.cos(heading),
            vy: speed * Math.sin(heading),
            source: 'random'
        });
        const clearOfGoal = Math.hypot(candidate.x - goal[0], candidate.y - goal[1])
            >= candidate.radius + 0.85;
        if (
            clearOfGoal
            && canPlaceG1Obstacle(candidate, obstacles, robotPosition)
        ) {
            return candidate;
        }
    }
    return null;
}
