

const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const norm = (x) => Math.sqrt(dot(x, x));

function cholesky(matrix) {
    const n = matrix.length;
    const l = Array.from({ length: n }, () => new Float64Array(n));
    for (let i = 0; i < n; i += 1) {
        for (let j = 0; j <= i; j += 1) {
            let value = matrix[i][j];
            for (let k = 0; k < j; k += 1) value -= l[i][k] * l[j][k];
            if (i === j) {
                if (!(value > 0) || !Number.isFinite(value)) throw new Error('Non-positive QP factorization.');
                l[i][j] = Math.sqrt(value);
            } else {
                l[i][j] = value / l[j][j];
            }
        }
    }
    return l;
}

function solveFactor(l, rhs) {
    const result = Float64Array.from(rhs);
    for (let i = 0; i < result.length; i += 1) {
        for (let j = 0; j < i; j += 1) result[i] -= l[i][j] * result[j];
        result[i] /= l[i][i];
    }
    for (let i = result.length - 1; i >= 0; i -= 1) {
        for (let j = i + 1; j < result.length; j += 1) result[i] -= l[j][i] * result[j];
        result[i] /= l[i][i];
    }
    return result;
}

function stepLength(value, delta) {
    let alpha = 1;
    for (let i = 0; i < value.length; i += 1) {
        if (delta[i] < 0) alpha = Math.min(alpha, -value[i] / delta[i]);
    }
    return alpha;
}

export function solveInequalityQP(qInput, p, g, h, {
    maxIterations = 40, tolerance = 1e-5, regularization = 1e-6
} = {}) {
    const n = p.length, m = h.length;
    const q = qInput.map((row, i) => row.map((value, j) =>
        0.5 * (value + qInput[j][i]) + (i === j ? regularization : 0)));
    let best = new Array(n).fill(0), bestResidual = Infinity, iterations = 0;
    try {
        const qFactor = cholesky(q);
        if (!m) return { primal: Array.from(solveFactor(qFactor, p.map((x) => -x))), residual: 0, iterations: 0 };
        const inverseQGt = g.map((row) => solveFactor(qFactor, row));
        const base = g.map((row) => inverseQGt.map((column) => dot(row, column)));
        const transposeMultiply = (v) => Array.from({ length: n }, (_, j) =>
            g.reduce((sum, row, i) => sum + row[j] * v[i], 0));

        function kkt(d, rx, rs, rz) {
            const qRx = solveFactor(qFactor, rx);
            const schur = base.map((row, i) => row.map((value, j) => value + (i === j ? 1 / d[i] : 0)));
            const rhs = g.map((row, i) => -(dot(row, qRx) + rs[i] / d[i] - rz[i]));
            const dz = solveFactor(cholesky(schur), rhs);
            const gtz = transposeMultiply(dz);
            const dx = solveFactor(qFactor, rx.map((value, i) => -value - gtz[i]));
            const ds = rs.map((value, i) => (-value - dz[i]) / d[i]);
            return [Array.from(dx), Array.from(ds), Array.from(dz)];
        }

        let [x, s, z] = kkt(new Array(m).fill(1), p, new Array(m).fill(0), h.map((v) => -v));
        const minimumS = Math.min(...s), minimumZ = Math.min(...z);
        if (minimumS < 0) s = s.map((v) => v - minimumS + 1);
        if (minimumZ < 0) z = z.map((v) => v - minimumZ + 1);
        for (let iteration = 0; iteration < maxIterations; iteration += 1) {
            const gtz = transposeMultiply(z);
            const rx = q.map((row, i) => gtz[i] + dot(row, x) + p[i]);
            const rz = g.map((row, i) => dot(row, x) + s[i] - h[i]);
            const mu = Math.abs(dot(s, z) / m);
            const residual = norm(rx) + norm(rz) + m * mu;
            if (!Number.isFinite(residual)) break;
            if (residual < bestResidual) { best = [...x]; bestResidual = residual; }
            if (bestResidual <= tolerance) break;
            const d = z.map((v, i) => v / Math.max(s[i], 1e-12));
            const [dxAff, dsAff, dzAff] = kkt(d, rx, z, rz);
            const affineAlpha = Math.min(stepLength(z, dzAff), stepLength(s, dsAff));
            const sAff = s.map((v, i) => v + affineAlpha * dsAff[i]);
            const zAff = z.map((v, i) => v + affineAlpha * dzAff[i]);
            const sigma = Math.max(0, Math.min(1, (dot(sAff, zAff) / Math.max(dot(s, z), 1e-12)) ** 3));
            const rs = s.map((v, i) => (-mu * sigma + dsAff[i] * dzAff[i]) / Math.max(v, 1e-12));
            const [dxCor, dsCor, dzCor] = kkt(d, new Array(n).fill(0), rs, new Array(m).fill(0));
            const dx = dxAff.map((v, i) => v + dxCor[i]);
            const ds = dsAff.map((v, i) => v + dsCor[i]);
            const dz = dzAff.map((v, i) => v + dzCor[i]);
            const alpha = Math.min(1, 0.999 * Math.min(stepLength(z, dz), stepLength(s, ds)));
            x = x.map((v, i) => v + alpha * dx[i]);
            s = s.map((v, i) => Math.max(v + alpha * ds[i], 1e-12));
            z = z.map((v, i) => Math.max(v + alpha * dz[i], 1e-12));
            iterations += 1;
        }
    } catch {

        return { primal: best, residual: Infinity, iterations };
    }
    return { primal: best, residual: bestResidual, iterations };
}
