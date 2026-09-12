import loadMujoco from '../../vendor/mujoco/mujoco.js';
import * as ort from '../../vendor/onnxruntime/ort.wasm.bundle.min.mjs';
import { UnitreeG1Runtime } from '../unitree_g1/runtime.js?v=dodger-3';
import { fetchTarGz } from '../unitree_g1/tar.js?v=dodger-3';

const MODEL_URL = new URL('../../assets/unitree_g1/unitree_g1_assets.tar.gz', import.meta.url);
const POLICY_URL = new URL('../../assets/dodger/locomotion.onnx', import.meta.url);
const CONFIG_URL = new URL('../../assets/dodger/locomotion-config.json', import.meta.url);
const PARITY_URL = new URL('../../assets/dodger/locomotion-parity.json', import.meta.url);

async function readJson(url) {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`Could not load ${url.pathname}: HTTP ${response.status}.`);
    return response.json();
}

export function buildLocomotionObservation({ gyro, gravity, command, phaseTime, jointPosition, jointVelocity, lastAction }, config) {
    const phase = 2 * Math.PI * phaseTime / config.gaitPeriod;
    const moving = Math.hypot(...command) >= 0.1;
    return new Float32Array([
        ...gyro, ...gravity, ...command,
        moving ? Math.sin(phase) : 0, moving ? Math.cos(phase) : 0,
        ...Array.from(jointPosition, (value, i) => value - config.defaultJointPosition[i]),
        ...jointVelocity, ...lastAction
    ]);
}

export class DodgerRuntime extends UnitreeG1Runtime {
    constructor(onStatus) {
        super(onStatus);
        this.phaseTime = 0;
        this.config = null;
    }

    async _initialize() {
        ort.env.wasm.numThreads = 1;
        this.onStatus('Loading DODGER locomotion and G1 physics…');
        const results = await Promise.allSettled([
            loadMujoco(), fetchTarGz(MODEL_URL),
            ort.InferenceSession.create(POLICY_URL.href, { executionProviders: ['wasm'], graphOptimizationLevel: 'all' }),
            readJson(CONFIG_URL), readJson(PARITY_URL)
        ]);
        if (results.some((result) => result.status === 'rejected')) {
            if (results[2].status === 'fulfilled') await results[2].value.release();
            throw results.find((result) => result.status === 'rejected').reason;
        }
        const [mujoco, files, session, config, parity] = results.map((result) => result.value);
        this.mujoco = mujoco; this.session = session; this.config = config;
        this.onStatus('Compiling G1 with deployment joint gains…');
        const vfs = new mujoco.MjVFS();
        try {
            for (const [name, bytes] of files) vfs.addBuffer(name, bytes);
            this.model = mujoco.from_xml_string(new TextDecoder().decode(files.get('scene.xml')), vfs);
        } finally { vfs.delete(); }
        if (!this.model || this.model.nq !== 36 || this.model.nv !== 35 || this.model.nu !== 29) {
            throw new Error('Unexpected Unitree G1 model dimensions.');
        }
        this.model.opt.timestep = config.simulationDt;
        for (let i = 0; i < 29; i += 1) {
            const actuator = this.model.actuator(config.jointNames[i]);
            if (!actuator) throw new Error(`Missing G1 actuator ${config.jointNames[i]}.`);
            try { if (actuator.id !== i) throw new Error('Locomotion actuator order mismatch.'); }
            finally { actuator.delete(); }
            this.model.actuator_gainprm[i * 10] = config.stiffness[i];
            this.model.actuator_biasprm[i * 10 + 1] = -config.stiffness[i];
            this.model.actuator_biasprm[i * 10 + 2] = -config.damping[i];
            this.model.dof_damping[i + 6] = 0;
        }
        this.data = new mujoco.MjData(this.model);
        this._cacheObservationAddresses();
        if (session.inputNames.join(',') !== 'obs' || session.outputNames.join(',') !== 'actions') {
            throw new Error('Unexpected 98-to-29 locomotion policy contract.');
        }
        this.policyInputName = 'obs'; this.policyOutputName = 'actions';
        this.reset('initial');
        this.onStatus('Checking locomotion ONNX parity…');
        await this.validateParity(parity);
        this.ready = true;
    }

    buildObservation(command) {
        const offset = this.imuSiteId * 9;
        return buildLocomotionObservation({
            gyro: this._sensorValues(this.sensorRanges.gyro),
            gravity: [-this.data.site_xmat[offset + 6], -this.data.site_xmat[offset + 7], -this.data.site_xmat[offset + 8]],
            command, phaseTime: this.phaseTime,
            jointPosition: this.data.qpos.subarray(7, 36),
            jointVelocity: this.data.qvel.subarray(6, 35),
            lastAction: this.lastAction
        }, this.config);
    }

    async validateParity(samples) {
        let maxError = 0;
        for (const sample of samples) {
            const input = new ort.Tensor('float32', Float32Array.from(sample.observation), [1, 98]);
            let outputs;
            try {
                outputs = await this.session.run({ obs: input });
                const values = outputs.actions.data;
                for (let i = 0; i < 29; i += 1) maxError = Math.max(maxError, Math.abs(values[i] - sample.action[i]));
            } finally {
                input.dispose();
                for (const output of Object.values(outputs ?? {})) output.dispose();
            }
        }
        this.parityMaxError = maxError;
        if (!Number.isFinite(maxError) || maxError > 2e-4) throw new Error(`Locomotion parity failed: ${maxError}.`);
    }

    async _updatePolicy(command) {
        const input = new ort.Tensor('float32', this.buildObservation(command), [1, 98]);
        let outputs;
        try {
            const start = performance.now();
            outputs = await this.session.run({ obs: input });
            this.lastInferenceMs = performance.now() - start;
            const action = outputs.actions?.data;
            if (!action || action.length !== 29 || !action.every(Number.isFinite)) throw new Error('Invalid locomotion action.');
            this.lastAction.set(action);
            for (let i = 0; i < 29; i += 1) {
                const target = this.config.defaultJointPosition[i] + this.config.actionScale[i] * action[i];
                this.data.ctrl[i] = Math.max(this.config.actionLimits[i][0], Math.min(this.config.actionLimits[i][1], target));
            }
            this.phaseTime += this.config.controlDt;
        } finally {
            input.dispose();
            for (const output of Object.values(outputs ?? {})) output.dispose();
        }
    }

    async advance(command, onPhysicsStep = null) {
        if (!this.ready) return { reset: false };
        const start = performance.now();

        await this._updatePolicy(command);
        for (let i = 0; i < 10; i += 1) {
            this.mujoco.mj_step(this.model, this.data);
            this.physicsCounter += 1;
            const robot = this.getRobotState();
            const finite = [...robot.position, robot.yaw, robot.yawRate, ...robot.worldVelocity, this.time].every(Number.isFinite);
            if (!finite) return { failure: 'non-finite state' };
            if (robot.position[2] < 0.45) return { failure: 'fall' };
            if (onPhysicsStep?.(robot, this.config.simulationDt)) return { failure: 'collision' };
        }
        this.lastPhysicsMs = Math.max(0, performance.now() - start - this.lastInferenceMs);
        return { reset: false };
    }

    reset(reason = 'manual') {
        if (!this.model || !this.data || !this.config) return;
        this.mujoco.mj_resetData(this.model, this.data);
        this.data.qpos.fill(0);
        this.data.qpos[2] = this.config.initialHeight;
        this.data.qpos[3] = 1;
        this.data.qpos.set(this.config.defaultJointPosition, 7);
        this.data.ctrl.set(this.config.defaultJointPosition);
        this.defaultAngles.set(this.config.defaultJointPosition);
        this.lastAction.fill(0);
        this.phaseTime = 0;
        this.physicsCounter = 0;
        this.lastResetReason = reason;
        if (reason !== 'initial') this.resetCount += 1;
        this.mujoco.mj_forward(this.model, this.data);
    }

    getDiagnostics() {
        return { ...super.getDiagnostics(), observationSize: 98, actionSize: 29 };
    }

    getRobotState() {
        const robot = super.getRobotState();

        if (this.data && this.sensorRanges) robot.yawRate = this._sensorValues(this.sensorRanges.gyro)[2];
        return robot;
    }
}
