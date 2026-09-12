import { DODGER_CONFIG, bodyVelocity, clamp, integrateCommand } from './config.js?v=dodger-3';
import { selectObstacles } from './observations.js?v=dodger-3';
import { solveInequalityQP } from './qp.js?v=dodger-3';

const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);

export function evaluateBarrier(robot, obstacle, action = [0, 0, 0], config = DODGER_CONFIG) {
    const px = obstacle.x - robot.position[0], py = obstacle.y - robot.position[1];
    const vx = obstacle.vx - robot.worldVelocity[0], vy = obstacle.vy - robot.worldVelocity[1];
    const pSquared = Math.max(px * px + py * py, 1e-8);
    const bearing = Math.atan2(py, px);
    const cLos = Math.cos(bearing), sLos = Math.sin(bearing);
    const x = cLos * vx + sLos * vy, y = -sLos * vx + cLos * vy;
    const speed = Math.sqrt(vx * vx + vy * vy + config.epsV ** 2);
    const safeRadius = (config.robotRadius + obstacle.radius) * config.safetyScale;
    const smoothClearance = Math.sqrt(Math.max(pSquared - safeRadius ** 2 + config.epsD ** 2, 1e-8));
    const distance = smoothClearance - config.epsD;
    const adaptive = Math.sqrt(config.safetyScale ** 2 - 1) / safeRadius;
    const lambda = config.kLambda * distance / speed;
    const h = x + adaptive * (lambda * y * y + config.kMu * distance);
    const positionTerm = config.kLambda * y * y / speed + config.kMu;
    const dhDx = py * y / pSquared - adaptive * (2 * lambda * x * y * py / pSquared + px * positionTerm / smoothClearance);
    const dhDy = -px * y / pSquared + adaptive * (2 * lambda * x * y * px / pSquared - py * positionTerm / smoothClearance);
    const relativeHeading = robot.yaw - bearing;
    const c = Math.cos(relativeHeading), s = Math.sin(relativeHeading);
    const [vf, vl] = bodyVelocity(robot);
    const v1 = vf * s + vl * c, v2 = vf * c - vl * s;
    const commonX = config.kLambda * distance * x * y * y / speed ** 3;
    const commonY = config.kLambda * distance * (2 * y / speed - y ** 3 / speed ** 3);
    const lgH = [
        -c + adaptive * (c * commonX - s * commonY),
        s - adaptive * (s * commonX + c * commonY),
        v1 - adaptive * (v1 * commonX + v2 * commonY)
    ];
    const lfH = -dhDx * vx - dhDy * vy;
    return { h, lfH, lgH, condition: lfH + dot(lgH, action) + config.alpha * h,
        bearing, lambdaCoefficient: adaptive * lambda, muCoefficient: adaptive * config.kMu * distance,
        relativeVelocity: [vx, vy], physicalClearance: Math.hypot(px, py) - config.robotRadius - obstacle.radius,
        safetyRadius: safeRadius };
}

export function filterAction(robot, obstacles, command, policyAction, dt = DODGER_CONFIG.controlDt, config = DODGER_CONFIG) {
    const selected = selectObstacles(robot, obstacles, config);
    const values = selected.map((obstacle) => evaluateBarrier(robot, obstacle, policyAction, config));
    const count = selected.length, n = 3 + count;
    const invSqrtWeight = 1 / Math.sqrt(config.slackWeight);
    const q = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => Number(i === j)));
    const p = [...policyAction.map((v) => -v), ...new Array(count).fill(0)];
    const g = [], h = [];
    for (let i = 0; i < count; i += 1) {
        const row = [...values[i].lgH.map((v) => -v), ...new Array(count).fill(0)];
        row[3 + i] = -invSqrtWeight;
        g.push(row);
        h.push(values[i].lfH + config.alpha * values[i].h);
    }
    for (let i = 0; i < count; i += 1) {
        const row = new Array(n).fill(0); row[3 + i] = -1;
        g.push(row); h.push(0);
    }
    const lower = [(config.commandMin[0] - command[0]) / dt, (config.commandMin[1] - command[1]) / dt, config.commandMin[2]];
    const upper = [(config.commandMax[0] - command[0]) / dt, (config.commandMax[1] - command[1]) / dt, config.commandMax[2]];
    for (let i = 0; i < 3; i += 1) {
        for (const sign of [1, -1]) {
            const row = new Array(n).fill(0); row[i] = sign;
            g.push(row); h.push(sign === 1 ? upper[i] : -lower[i]);
        }
    }
    for (let i = 0; i < g.length; i += 1) {
        const scale = Math.max(Math.hypot(...g[i]), 1);
        g[i] = g[i].map((v) => v / scale); h[i] /= scale;
    }
    const result = solveInequalityQP(q, p, g, h, {
        maxIterations: config.qpMaxIterations, tolerance: config.qpTolerance,
        regularization: config.qpRegularization
    });
    const failed = !Number.isFinite(result.residual) || !result.primal.every(Number.isFinite);
    const safeAction = failed ? [...policyAction] : result.primal.slice(0, 3).map((v, i) => clamp(v, lower[i], upper[i]));
    const slack = values.map((value, i) => Math.max(0,
        failed ? 0 : result.primal[3 + i] * invSqrtWeight,
        -(value.lfH + dot(value.lgH, safeAction) + config.alpha * value.h)));
    return { safeAction, selected, values, slack, failed, residual: result.residual, iterations: result.iterations };
}

export function dpcbfOverlay(robot, obstacle, value) {
    const c = Math.cos(value.bearing), s = Math.sin(value.bearing);
    const points = Array.from({ length: 61 }, (_, index) => {
        const lateral = -1.25 + 2.5 * index / 60;
        const radial = -value.lambdaCoefficient * lateral * lateral - value.muCoefficient;
        return [robot.position[0] + c * radial - s * lateral, robot.position[1] + s * radial + c * lateral];
    });
    return { type: 'dpcbf-parabola', obstacleId: obstacle.id, points,
        relativeVelocityArrow: { start: robot.position.slice(0, 2), end: [
            robot.position[0] + value.relativeVelocity[0], robot.position[1] + value.relativeVelocity[1]] },
        safetyShape: { center: [obstacle.x, obstacle.y], radii: [value.safetyRadius, value.safetyRadius], yaw: 0 }
    };
}

export function safetyDiagnostics(robot, command, action, result, { visible = true, filterApplied = false } = {}) {
    const reference = integrateCommand(command, result.safeAction, DODGER_CONFIG.controlDt);
    const constraints = result.values.map((value, i) => ({ ...value,
        obstacleId: result.selected[i].id,
        nominalMargin: value.condition,
        filteredMargin: value.lfH + dot(value.lgH, result.safeAction) + DODGER_CONFIG.alpha * value.h + result.slack[i],
        slack: result.slack[i]
    }));
    const min = (items) => items.length ? Math.min(...items) : null;
    return { enabled: visible, referenceOnly: true, filterApplied,
        status: result.failed ? 'numerical fallback' : filterApplied ? 'filtering' : 'monitoring',
        params: DODGER_CONFIG,
        clearance: min(result.values.map((value) => value.physicalClearance)),
        minBarrier: min(result.values.map((value) => value.h)),
        constrainedObstacleIds: result.selected.map((obstacle) => obstacle.id),
        constraints,
        overlays: result.values.map((value, i) => dpcbfOverlay(robot, result.selected[i], value)),
        intervention: Math.hypot(...action.map((v, i) => v - result.safeAction[i])),
        intervening: filterApplied && Math.hypot(...action.map((v, i) => v - result.safeAction[i])) > 0.012,
        safeAction: [...result.safeAction], referenceCommand: reference,
        qpResidual: result.residual, qpFailed: result.failed,
        predictedPath: []
    };
}
