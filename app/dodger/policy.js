import * as ort from '../../vendor/onnxruntime/ort.wasm.bundle.min.mjs';
import { NAVIGATION_POLICIES, clipAction } from './config.js?v=dodger-3';
import { buildNavigationObservation, pdNavigationAction } from './observations.js?v=dodger-3';

export class NavigationPolicies {
    constructor() {
        this.sessions = new Map();
        this.loading = new Map();
        this.lastInferenceMs = 0;
    }

    async load(id) {
        const policy = NAVIGATION_POLICIES.find((entry) => entry.id === id);
        if (!policy?.available) throw new Error(`${policy?.label ?? id} weights have not been supplied.`);
        if (policy.kind === 'qp' || this.sessions.has(id)) return;
        if (this.loading.has(id)) return this.loading.get(id);
        const promise = this._load(policy);
        this.loading.set(id, promise);
        try { await promise; } finally { this.loading.delete(id); }
    }

    async _load(policy) {
        ort.env.wasm.numThreads = 1;
        const options = { executionProviders: ['wasm'], graphOptimizationLevel: 'all' };
        const results = await Promise.allSettled([
            ort.InferenceSession.create(new URL(policy.encoder, import.meta.url).href, options),
            ort.InferenceSession.create(new URL(policy.head, import.meta.url).href, options)
        ]);
        if (results.some((result) => result.status === 'rejected')) {
            for (const result of results) if (result.status === 'fulfilled') await result.value.release();
            throw results.find((result) => result.status === 'rejected').reason;
        }
        const [encoder, head] = results.map((result) => result.value);
        if (encoder.inputNames.join(',') !== 'nodes' || encoder.outputNames.join(',') !== 'robot_embedding'
            || head.inputNames.join(',') !== 'robot_embedding,local_state' || head.outputNames.join(',') !== 'policy_action') {
            await encoder.release();
            await head.release();
            throw new Error(`Unexpected ${policy.label} ONNX input/output contract.`);
        }
        this.sessions.set(policy.id, { encoder, head });
    }

    async infer(id, state) {
        const start = performance.now();
        if (NAVIGATION_POLICIES.find((p) => p.id === id)?.kind === 'qp') {
            this.lastInferenceMs = performance.now() - start;
            return clipAction(pdNavigationAction(state.robot, state.goal, state.goalYaw));
        }
        const sessions = this.sessions.get(id);
        if (!sessions) throw new Error(`Navigation policy ${id} is not ready.`);
        const observation = buildNavigationObservation(state);
        const nodes = new ort.Tensor('float32', observation.nodes, [1, observation.nodeCount, 8]);
        const local = new ort.Tensor('float32', observation.localState, [1, 13]);
        let encoded, outputs;
        try {
            encoded = await sessions.encoder.run({ nodes });
            outputs = await sessions.head.run({ robot_embedding: encoded.robot_embedding, local_state: local });
            const action = Array.from(outputs.policy_action.data);
            if (action.length !== 3 || !action.every(Number.isFinite)) {
                throw new Error(`${id} returned an invalid navigation action.`);
            }
            this.lastInferenceMs = performance.now() - start;
            return clipAction(action);
        } finally {
            nodes.dispose();
            local.dispose();
            for (const tensor of Object.values(encoded ?? {})) tensor.dispose();
            for (const tensor of Object.values(outputs ?? {})) tensor.dispose();
        }
    }

    async dispose() {
        for (const { encoder, head } of this.sessions.values()) {
            await encoder.release();
            await head.release();
        }
        this.sessions.clear();
    }
}
