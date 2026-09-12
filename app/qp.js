import { EPS, dotN } from './math.js?v=dodger-3';

function isInsideHalfPlane(point, halfPlane, tolerance = 1e-8) {
    return dotN(halfPlane.normal, point) + halfPlane.offset >= -tolerance;
}

function collectMargins(point, halfPlanes) {
    return halfPlanes.map((halfPlane) => ({
        label: halfPlane.label,
        margin: dotN(halfPlane.normal, point) + halfPlane.offset
    }));
}

function solveLinearSystem(matrix, rhs) {
    const size = rhs.length;
    const augmented = matrix.map((row, rowIndex) => [...row, rhs[rowIndex]]);

    for (let pivotIndex = 0; pivotIndex < size; pivotIndex += 1) {
        let pivotRow = pivotIndex;
        let bestMagnitude = Math.abs(augmented[pivotIndex][pivotIndex]);
        for (let row = pivotIndex + 1; row < size; row += 1) {
            const magnitude = Math.abs(augmented[row][pivotIndex]);
            if (magnitude > bestMagnitude) {
                bestMagnitude = magnitude;
                pivotRow = row;
            }
        }
        if (bestMagnitude < 1e-10) {
            return null;
        }
        if (pivotRow !== pivotIndex) {
            [augmented[pivotIndex], augmented[pivotRow]] = [augmented[pivotRow], augmented[pivotIndex]];
        }

        const pivot = augmented[pivotIndex][pivotIndex];
        for (let column = pivotIndex; column <= size; column += 1) {
            augmented[pivotIndex][column] /= pivot;
        }

        for (let row = 0; row < size; row += 1) {
            if (row === pivotIndex) {
                continue;
            }
            const factor = augmented[row][pivotIndex];
            if (Math.abs(factor) < 1e-12) {
                continue;
            }
            for (let column = pivotIndex; column <= size; column += 1) {
                augmented[row][column] -= factor * augmented[pivotIndex][column];
            }
        }
    }

    return augmented.map((row) => row[size]);
}

function buildCombinations(count, targetSize, startIndex = 0, prefix = [], output = []) {
    if (prefix.length === targetSize) {
        output.push([...prefix]);
        return output;
    }
    for (let index = startIndex; index < count; index += 1) {
        prefix.push(index);
        buildCombinations(count, targetSize, index + 1, prefix, output);
        prefix.pop();
    }
    return output;
}

function evaluateObjective(point, reference, weights) {
    let value = 0;
    for (let index = 0; index < point.length; index += 1) {
        const delta = point[index] - reference[index];
        value += 0.5 * weights[index] * delta * delta;
    }
    return value;
}

function solveEqualityConstrainedQP(reference, weights, activeConstraints) {
    const dimension = reference.length;
    const activeCount = activeConstraints.length;

    if (activeCount === 0) {
        return {
            point: [...reference],
            lambdas: []
        };
    }

    const size = dimension + activeCount;
    const matrix = Array.from({ length: size }, () => Array(size).fill(0));
    const rhs = Array(size).fill(0);

    for (let index = 0; index < dimension; index += 1) {
        matrix[index][index] = weights[index];
        rhs[index] = weights[index] * reference[index];
    }

    activeConstraints.forEach((constraint, constraintIndex) => {
        const row = dimension + constraintIndex;
        for (let col = 0; col < dimension; col += 1) {
            matrix[col][row] = -constraint.normal[col];
            matrix[row][col] = constraint.normal[col];
        }
        rhs[row] = -constraint.offset;
    });

    const solution = solveLinearSystem(matrix, rhs);
    if (!solution) {
        return null;
    }

    return {
        point: solution.slice(0, dimension),
        lambdas: solution.slice(dimension)
    };
}

function createBoundHalfPlanes(bounds, dimension) {
    const halfPlanes = [];
    for (let index = 0; index < dimension; index += 1) {
        const normalMin = Array.from({ length: dimension }, (_, coord) => (coord === index ? 1 : 0));
        const normalMax = Array.from({ length: dimension }, (_, coord) => (coord === index ? -1 : 0));
        halfPlanes.push({
            label: `u${index}_min`,
            normal: normalMin,
            offset: -bounds.min[index]
        });
        halfPlanes.push({
            label: `u${index}_max`,
            normal: normalMax,
            offset: bounds.max[index]
        });
    }
    return halfPlanes;
}

function solveActiveSetQP(reference, weights, halfPlanes) {
    const dimension = reference.length;
    let best = null;

    for (let activeSize = 0; activeSize <= dimension; activeSize += 1) {
        const combinations = activeSize === 0 ? [[]] : buildCombinations(halfPlanes.length, activeSize);
        for (const combination of combinations) {
            const activeConstraints = combination.map((index) => halfPlanes[index]);
            const candidate = solveEqualityConstrainedQP(reference, weights, activeConstraints);
            if (!candidate) {
                continue;
            }
            if (candidate.lambdas.some((lambda) => lambda < -1e-7)) {
                continue;
            }
            if (!halfPlanes.every((halfPlane) => isInsideHalfPlane(candidate.point, halfPlane, 1e-7))) {
                continue;
            }

            const objective = evaluateObjective(candidate.point, reference, weights);
            if (!best || objective < best.objective) {
                best = {
                    point: candidate.point,
                    objective,
                    margins: collectMargins(candidate.point, halfPlanes)
                };
            }
        }
    }

    return best;
}

function clampPointToBounds(point, bounds) {
    return point.map((value, index) => Math.min(bounds.max[index], Math.max(bounds.min[index], value)));
}

function solveWithoutSlack(uRef, constraints, bounds, weights) {
    const dimension = uRef.length;
    const halfPlanes = [
        ...createBoundHalfPlanes(bounds, dimension),
        ...constraints.map((constraint, index) => ({
            label: constraint.label ?? `cbf_${index}`,
            normal: [...constraint.A],
            offset: constraint.b
        }))
    ];

    const best = solveActiveSetQP(uRef, weights, halfPlanes);
    if (!best) {
        const fallback = clampPointToBounds(uRef, bounds);
        const margins = collectMargins(fallback, halfPlanes);
        return {
            u: fallback,
            status: 'infeasible',
            feasible: false,
            minMargin: Math.min(...margins.map((item) => item.margin)),
            activeSet: [],
            slack: 0
        };
    }

    const minMargin = Math.min(...best.margins.map((item) => item.margin));
    const activeSet = best.margins
        .filter((item) => Math.abs(item.margin) <= 1e-6)
        .map((item) => item.label);

    return {
        u: best.point,
        status: 'optimal',
        feasible: true,
        minMargin,
        activeSet,
        slack: 0
    };
}

function solveWithSharedSlack(uRef, constraints, bounds, weights, slackWeight) {
    const controlDimension = uRef.length;
    const dimension = controlDimension + 1;
    const reference = [...uRef, 0];
    const qpWeights = [...weights, slackWeight];
    const halfPlanes = [
        ...createBoundHalfPlanes(bounds, controlDimension).map((halfPlane) => ({
            ...halfPlane,
            normal: [...halfPlane.normal, 0]
        })),
        {
            label: 'slack_min',
            normal: Array.from({ length: dimension }, (_, index) => (index === dimension - 1 ? 1 : 0)),
            offset: 0
        },
        ...constraints.map((constraint, index) => ({
            label: constraint.label ?? `cbf_${index}`,
            normal: [...constraint.A, 1],
            offset: constraint.b
        }))
    ];

    let best = solveActiveSetQP(reference, qpWeights, halfPlanes);
    if (!best) {
        const fallbackControl = clampPointToBounds(uRef, bounds);
        const slack = Math.max(
            0,
            ...constraints.map((constraint) => -(dotN(constraint.A, fallbackControl) + constraint.b))
        );
        const point = [...fallbackControl, slack];
        best = {
            point,
            margins: collectMargins(point, halfPlanes)
        };
    }

    const minMargin = Math.min(...best.margins.map((item) => item.margin));
    const activeSet = best.margins
        .filter((item) => Math.abs(item.margin) <= 1e-6)
        .map((item) => item.label);

    return {
        u: best.point.slice(0, controlDimension),
        slack: best.point[controlDimension],
        status: 'optimal',
        feasible: true,
        minMargin,
        activeSet
    };
}

export function solveBoxConstrainedQP(uRef, constraints, bounds, options = {}) {
    const weights = options.weights ?? Array.from({ length: uRef.length }, () => 1);
    const slackOptions = options.slack ?? {};
    if (slackOptions.enabled) {
        return solveWithSharedSlack(
            uRef,
            constraints,
            bounds,
            weights,
            slackOptions.weight ?? 100
        );
    }
    return solveWithoutSlack(uRef, constraints, bounds, weights);
}
