export const EPS = 1e-6;

export function zeros(count) {
    return Array.from({ length: count }, () => 0);
}

export function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
}

export function lerp(a, b, t) {
    return a + (b - a) * t;
}

export function sqr(value) {
    return value * value;
}

export function dotN(a, b) {
    let sum = 0;
    const count = Math.min(a.length, b.length);
    for (let index = 0; index < count; index += 1) {
        sum += a[index] * b[index];
    }
    return sum;
}

export function dot(a, b) {
    return dotN(a, b);
}

export function normN(vector) {
    return Math.sqrt(dotN(vector, vector));
}

export function norm(vector) {
    return normN(vector);
}

export function distance(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
}

export function distance3(a, b) {
    return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

export function add(a, b) {
    return a.map((value, index) => value + b[index]);
}

export function subtract(a, b) {
    return a.map((value, index) => value - b[index]);
}

export function scale(vector, scalar) {
    return vector.map((value) => value * scalar);
}

export function add3(a, b) {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function subtract3(a, b) {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function scale3(vector, scalar) {
    return [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar];
}

export function dot3(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function norm3(vector) {
    return Math.sqrt(dot3(vector, vector));
}

export function cross3(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

export function normalize(vector) {
    const magnitude = normN(vector);
    if (magnitude < EPS) {
        return vector.map(() => 0);
    }
    return vector.map((value) => value / magnitude);
}

export function normalize3(vector) {
    const magnitude = norm3(vector);
    if (magnitude < EPS) {
        return [0, 0, 0];
    }
    return [vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude];
}

export function clampMagnitude(vector, maxMagnitude) {
    const magnitude = normN(vector);
    if (magnitude <= maxMagnitude || magnitude < EPS) {
        return [...vector];
    }
    const scaleFactor = maxMagnitude / magnitude;
    return vector.map((value) => value * scaleFactor);
}

export function rotate(vector, angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]];
}

export function angleNormalize(angle) {
    return ((((angle + Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;
}

export function radToDeg(radians) {
    return (radians * 180) / Math.PI;
}

export function projectPointToSegment(point, start, end) {
    const segment = [end[0] - start[0], end[1] - start[1]];
    const lengthSq = dot(segment, segment);
    if (lengthSq < EPS) {
        return [...start];
    }
    const t = clamp(dot([point[0] - start[0], point[1] - start[1]], segment) / lengthSq, 0, 1);
    return [start[0] + segment[0] * t, start[1] + segment[1] * t];
}

export function linspace(start, end, count) {
    if (count <= 1) {
        return [start];
    }
    const values = [];
    const step = (end - start) / (count - 1);
    for (let index = 0; index < count; index += 1) {
        values.push(start + index * step);
    }
    return values;
}

export function minBy(items, selector) {
    let bestItem = null;
    let bestValue = Infinity;
    for (const item of items) {
        const value = selector(item);
        if (value < bestValue) {
            bestValue = value;
            bestItem = item;
        }
    }
    return bestItem;
}
