import loadMujoco from '../../vendor/mujoco/mujoco.js';
import * as ort from '../../vendor/onnxruntime/ort.wasm.bundle.min.mjs';
import { fetchTarGz } from './tar.js?v=dodger-3';

const MODEL_BUNDLE_URL = new URL(
    '../../assets/unitree_g1/unitree_g1_assets.tar.gz',
    import.meta.url
);
const POLICY_URL = new URL('../../assets/unitree_g1/g1_policy.onnx', import.meta.url);
const RESET_POLICY_ACTION = new Float32Array([
    -0.70525014, 0.029958919, 0.05401574, 0.8372449, -0.46138704,
    -0.12496665, 0.5283374, -0.7365668, 0.751675, -0.86020285,
    -0.47170067, 0.15094712, 0.14496037, -0.24608624, -0.1825026,
    0.19930375, 0.22935632, -0.13189697, -0.2720494, -0.18651192,
    0.3947176, 0.40895975, 0.08557126, -0.20396715, -0.12869106,
    -0.08224003, 0.22798409, -0.031084783, 0.5606341
]);

export const G1_RUNTIME_CONSTANTS = Object.freeze({
    simulationDt: 0.002,
    controlDt: 0.02,
    substeps: 10,
    actionScale: 0.5,
    observationSize: 103,
    actionSize: 29,
    gaitFrequency: 1.5,
    resetKeyframe: 1
});

function wrapPhase(value) {
    const twoPi = 2 * Math.PI;
    return ((value + Math.PI) % twoPi + twoPi) % twoPi - Math.PI;
}

function nextBrowserFrame() {
    return new Promise((resolve) => {
        if (typeof requestAnimationFrame === 'function') {
            requestAnimationFrame(() => resolve());
        } else {
            setTimeout(resolve, 0);
        }
    });
}

function quaternionYaw(qpos) {
    const w = qpos[3];
    const x = qpos[4];
    const y = qpos[5];
    const z = qpos[6];
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function quaternionYawRate(qpos, qvel) {
    const w = qpos[3];
    const x = qpos[4];
    const y = qpos[5];
    const z = qpos[6];
    const sinRollCosPitch = 2 * (w * x + y * z);
    const cosRollCosPitch = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinRollCosPitch, cosRollCosPitch);
    const sinPitch = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
    const cosPitch = Math.sqrt(1 - sinPitch * sinPitch);
    if (cosPitch < 1e-3) {

        return Number.isFinite(qvel[5]) ? qvel[5] : 0;
    }

    return (
        Math.sin(roll) * qvel[4] + Math.cos(roll) * qvel[5]
    ) / cosPitch;
}

function assertDimensions(model) {
    const expected = { nq: 36, nv: 35, nu: G1_RUNTIME_CONSTANTS.actionSize };
    for (const [key, value] of Object.entries(expected)) {
        if (model[key] !== value) {
            throw new Error(`Unexpected G1 ${key}: expected ${value}, received ${model[key]}.`);
        }
    }
}

function getNamedId(owner, kind, name) {
    const accessor = owner[kind](name);
    if (!accessor) throw new Error(`Missing MuJoCo ${kind}: ${name}`);
    try {
        return accessor.id;
    } finally {
        accessor.delete();
    }
}

export class UnitreeG1Runtime {
    constructor(onStatus = () => {}) {
        this.onStatus = onStatus;
        this.mujoco = null;
        this.model = null;
        this.data = null;
        this.session = null;
        this.ready = false;
        this.initializationPromise = null;
        this.lastAction = new Float32Array(G1_RUNTIME_CONSTANTS.actionSize);
        this.defaultAngles = new Float32Array(G1_RUNTIME_CONSTANTS.actionSize);
        this.phase = new Float64Array([0, Math.PI]);
        this.physicsCounter = 0;
        this.resetCount = 0;
        this.lastInferenceMs = 0;
        this.lastPhysicsMs = 0;
        this.lastResetReason = null;
        this.parityMaxError = null;
        this.sensorRanges = null;
        this.imuSiteId = -1;
    }

    async initialize() {
        if (this.ready) return;
        if (this.initializationPromise) return this.initializationPromise;
        this.initializationPromise = this._initialize();
        try {
            await this.initializationPromise;
        } catch (error) {
            this.initializationPromise = null;
            await this.dispose();
            throw error;
        }
    }

    async _initialize() {
        this.onStatus('Loading G1 physics, model, and policy…');
        ort.env.wasm.numThreads = 1;
        ort.env.wasm.proxy = false;
        ort.env.logLevel = 'warning';

        const physicsPromise = loadMujoco();
        const modelFilesPromise = fetchTarGz(MODEL_BUNDLE_URL);
        const policyPromise = ort.InferenceSession.create(POLICY_URL.href, {
            executionProviders: ['wasm'],
            graphOptimizationLevel: 'all'
        });
        const loadResults = await Promise.allSettled([
            physicsPromise,
            modelFilesPromise,
            policyPromise
        ]);
        const failedLoad = loadResults.find((result) => result.status === 'rejected');
        if (failedLoad) {
            const policyResult = loadResults[2];
            if (policyResult.status === 'fulfilled') {
                await policyResult.value.release();
            }
            throw failedLoad.reason;
        }
        const [mujoco, modelFiles, session] = loadResults.map((result) => result.value);
        this.mujoco = mujoco;
        this.session = session;

        this.onStatus('Compiling the Unitree G1 model…');
        await nextBrowserFrame();
        const sceneBytes = modelFiles.get('scene.xml');
        if (!sceneBytes) throw new Error('The G1 model bundle is missing scene.xml.');

        const vfs = new mujoco.MjVFS();
        let model = null;
        try {
            for (const [name, bytes] of modelFiles) {
                vfs.addBuffer(name, bytes);
            }
            model = mujoco.from_xml_string(new TextDecoder().decode(sceneBytes), vfs);
        } finally {
            vfs.delete();
        }
        if (!model) throw new Error('MuJoCo could not compile the Unitree G1 model.');
        this.model = model;
        assertDimensions(model);
        this.data = new mujoco.MjData(model);
        this.model.opt.timestep = G1_RUNTIME_CONSTANTS.simulationDt;
        this._cacheObservationAddresses();
        this.reset('initial');

        const inputName = this.session.inputNames?.[0];
        const outputName = this.session.outputNames?.includes('continuous_actions')
            ? 'continuous_actions'
            : this.session.outputNames?.[0];
        if (inputName !== 'obs' || !outputName) {
            throw new Error('The G1 policy has an unexpected input/output signature.');
        }
        this.policyInputName = inputName;
        this.policyOutputName = outputName;
        this.onStatus('Checking policy and simulator parity…');
        await this._validateResetParity();
        this.ready = true;
        this.onStatus('Ready');
    }

    _cacheObservationAddresses() {
        const sensorRange = (name) => {
            const id = getNamedId(this.model, 'sensor', name);
            return {
                address: this.model.sensor_adr[id],
                dimension: this.model.sensor_dim[id]
            };
        };
        this.sensorRanges = {
            localLinearVelocity: sensorRange('local_linvel_pelvis'),
            gyro: sensorRange('gyro_pelvis')
        };
        this.imuSiteId = getNamedId(this.model, 'site', 'imu_in_pelvis');
    }

    _sensorValues(range) {
        return this.data.sensordata.subarray(
            range.address,
            range.address + range.dimension
        );
    }

    buildObservation(command) {
        const observation = new Float32Array(G1_RUNTIME_CONSTANTS.observationSize);
        let offset = 0;

        observation.set(this._sensorValues(this.sensorRanges.localLinearVelocity), offset);
        offset += 3;
        observation.set(this._sensorValues(this.sensorRanges.gyro), offset);
        offset += 3;

        const imuMatrixOffset = this.imuSiteId * 9;
        observation[offset++] = -this.data.site_xmat[imuMatrixOffset + 6];
        observation[offset++] = -this.data.site_xmat[imuMatrixOffset + 7];
        observation[offset++] = -this.data.site_xmat[imuMatrixOffset + 8];

        observation[offset++] = command[0];
        observation[offset++] = command[1];
        observation[offset++] = command[2];

        for (let index = 0; index < G1_RUNTIME_CONSTANTS.actionSize; index += 1) {
            observation[offset++] = this.data.qpos[7 + index] - this.defaultAngles[index];
        }
        for (let index = 0; index < G1_RUNTIME_CONSTANTS.actionSize; index += 1) {
            observation[offset++] = this.data.qvel[6 + index];
        }
        observation.set(this.lastAction, offset);
        offset += G1_RUNTIME_CONSTANTS.actionSize;
        observation[offset++] = Math.cos(this.phase[0]);
        observation[offset++] = Math.cos(this.phase[1]);
        observation[offset++] = Math.sin(this.phase[0]);
        observation[offset++] = Math.sin(this.phase[1]);

        if (offset !== G1_RUNTIME_CONSTANTS.observationSize) {
            throw new Error(`G1 observation length mismatch: ${offset}.`);
        }
        return observation;
    }

    async _updatePolicy(command) {
        const observation = this.buildObservation(command);
        const input = new ort.Tensor(
            'float32',
            observation,
            [1, G1_RUNTIME_CONSTANTS.observationSize]
        );
        let outputs = null;
        try {
            const start = performance.now();
            outputs = await this.session.run({ [this.policyInputName]: input });
            this.lastInferenceMs = performance.now() - start;
            const output = outputs[this.policyOutputName];
            const action = output?.data;
            if (!action || action.length !== G1_RUNTIME_CONSTANTS.actionSize) {
                throw new Error('The G1 policy did not return 29 joint actions.');
            }

            for (let index = 0; index < action.length; index += 1) {
                if (!Number.isFinite(action[index])) {
                    throw new Error(`The G1 policy returned a non-finite action at index ${index}.`);
                }
                this.lastAction[index] = action[index];
                this.data.ctrl[index] = this.defaultAngles[index]
                    + G1_RUNTIME_CONSTANTS.actionScale * action[index];
            }

            const phaseStep = 2 * Math.PI
                * G1_RUNTIME_CONSTANTS.gaitFrequency
                * G1_RUNTIME_CONSTANTS.controlDt;
            this.phase[0] = wrapPhase(this.phase[0] + phaseStep);
            this.phase[1] = wrapPhase(this.phase[1] + phaseStep);
        } finally {
            input.dispose?.();
            if (outputs) {
                for (const tensor of Object.values(outputs)) tensor.dispose?.();
            }
        }
    }

    async _validateResetParity() {
        const observation = this.buildObservation([0, 0, 0]);
        const input = new ort.Tensor(
            'float32',
            observation,
            [1, G1_RUNTIME_CONSTANTS.observationSize]
        );
        let outputs = null;
        try {
            outputs = await this.session.run({ [this.policyInputName]: input });
            const output = outputs[this.policyOutputName];
            if (!output?.data || output.data.length !== RESET_POLICY_ACTION.length) {
                throw new Error('The G1 reset parity check returned an invalid action.');
            }
            let maxError = 0;
            for (let index = 0; index < RESET_POLICY_ACTION.length; index += 1) {
                maxError = Math.max(
                    maxError,
                    Math.abs(output.data[index] - RESET_POLICY_ACTION[index])
                );
            }
            this.parityMaxError = maxError;
            if (maxError > 1e-4) {
                throw new Error(`G1 policy parity check failed (max error ${maxError.toExponential(2)}).`);
            }
        } finally {
            input.dispose?.();
            if (outputs) {
                for (const tensor of Object.values(outputs)) tensor.dispose?.();
            }
        }
    }

    async advance(command) {
        if (!this.ready) return { reset: false };
        const start = performance.now();
        let inferenceTime = 0;

        for (let index = 0; index < G1_RUNTIME_CONSTANTS.substeps; index += 1) {
            this.mujoco.mj_step1(this.model, this.data);

            this.physicsCounter += 1;
            if (this.physicsCounter % G1_RUNTIME_CONSTANTS.substeps === 0) {
                await this._updatePolicy(command);
                inferenceTime += this.lastInferenceMs;
            }
            this.mujoco.mj_step2(this.model, this.data);
        }
        this.lastPhysicsMs = Math.max(0, performance.now() - start - inferenceTime);

        const state = this.getRobotState();
        const finite = state.position.every(Number.isFinite)
            && Number.isFinite(state.yaw)
            && Number.isFinite(state.yawRate)
            && state.worldVelocity.every(Number.isFinite)
            && Number.isFinite(this.data.time);
        if (!finite || state.position[2] < 0.35) {
            this.reset(finite ? 'fall' : 'non-finite state');
            return { reset: true, reason: this.lastResetReason };
        }
        return { reset: false };
    }

    reset(reason = 'manual') {
        if (!this.model || !this.data || !this.mujoco) return;
        this.mujoco.mj_resetDataKeyframe(
            this.model,
            this.data,
            G1_RUNTIME_CONSTANTS.resetKeyframe
        );
        this.mujoco.mj_forward(this.model, this.data);
        this.defaultAngles.set(this.data.qpos.subarray(7, 36));
        this.lastAction.fill(0);
        this.phase[0] = 0;
        this.phase[1] = Math.PI;
        this.physicsCounter = 0;
        this.lastResetReason = reason;
        if (reason !== 'initial') this.resetCount += 1;
    }

    getRobotState() {
        if (!this.data) {
            return {
                position: [0, 0, 0.755],
                yaw: 0,
                yawRate: 0,
                worldVelocity: [0, 0, 0]
            };
        }
        const measuredYawRate = quaternionYawRate(
            this.data.qpos,
            this.data.qvel
        );
        return {
            position: [this.data.qpos[0], this.data.qpos[1], this.data.qpos[2]],
            yaw: quaternionYaw(this.data.qpos),

            yawRate: Number.isFinite(measuredYawRate)
                ? measuredYawRate
                : 0,
            worldVelocity: [this.data.qvel[0], this.data.qvel[1], this.data.qvel[2]]
        };
    }

    get time() {
        return this.data?.time ?? 0;
    }

    getDiagnostics() {
        return {
            inferenceMs: this.lastInferenceMs,
            physicsMs: this.lastPhysicsMs,
            resetCount: this.resetCount,
            observationSize: G1_RUNTIME_CONSTANTS.observationSize,
            actionSize: G1_RUNTIME_CONSTANTS.actionSize,
            parityMaxError: this.parityMaxError
        };
    }

    async dispose() {
        this.ready = false;
        await this.session?.release?.();
        this.data?.delete();
        this.model?.delete();
        this.session = null;
        this.data = null;
        this.model = null;
    }
}
