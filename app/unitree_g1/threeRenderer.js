import * as THREE from '../../vendor/three.module.js';
import { UnitreeG1SafetyVisuals } from './safetyVisuals.js?v=dodger-3';

const GEOM = Object.freeze({
    PLANE: 0,
    SPHERE: 2,
    CAPSULE: 3,
    ELLIPSOID: 4,
    CYLINDER: 5,
    BOX: 6,
    MESH: 7
});
const TRAIL_CAPACITY = 500;
const TELEMETRY_CAPACITY = 180;
const TELEMETRY_WIDTH = 252;
const TELEMETRY_HEIGHT = 86;
const CAMERA_PRESETS = Object.freeze({
    orbit: Object.freeze({ azimuth: -1.2, elevation: 0.34, radius: 4.4 }),
    track: Object.freeze({ azimuth: 0, elevation: 0.26, radius: 3.4 })
});

function disposeMaterial(material) {
    if (Array.isArray(material)) material.forEach((entry) => entry.dispose());
    else material?.dispose();
}

function applyStyles(element, styles) {
    Object.assign(element.style, styles);
}

function copyCameraPreset(name) {
    return { ...CAMERA_PRESETS[name] };
}

export class UnitreeG1ThreeRenderer {
    constructor(hostEl, onGoalSelected, onPauseToggle = () => {}) {
        this.hostEl = hostEl;
        this.onGoalSelected = onGoalSelected;
        this.onPauseToggle = onPauseToggle;
        this.runtime = null;
        this.visible = true;
        this.robotObjects = [];
        this.geometryCache = new Map();
        this.drag = null;
        this.cameraMode = 'orbit';
        this.cameraViews = {
            orbit: copyCameraPreset('orbit'),
            track: copyCameraPreset('track')
        };
        this.orbit = this.cameraViews.orbit;
        this.target = new THREE.Vector3(0, 0, 0.82);
        this.desiredTarget = this.target.clone();
        this.worldTarget = this.target.clone();
        this.trackOffset = new THREE.Vector3();
        this.nextTarget = new THREE.Vector3();
        this.lastRobotPosition = new THREE.Vector3(0, 0, 0.755);
        this.lastRobotYaw = 0;
        this.trackHeading = 0;
        this.panRight = new THREE.Vector3();
        this.panForward = new THREE.Vector3();
        this.raycaster = new THREE.Raycaster();
        this.pointer = new THREE.Vector2();
        this.groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        this.telemetryCommand = new Float32Array(TELEMETRY_CAPACITY);
        this.telemetryMeasured = new Float32Array(TELEMETRY_CAPACITY);
        this.telemetryCursor = 0;
        this.telemetryCount = 0;
        this.lastTelemetryTime = -Infinity;

        this.viewport = document.createElement('div');
        this.viewport.className = 'unitree-g1-viewport';
        this.hostEl.replaceChildren(this.viewport);

        this.renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: false,
            powerPreference: 'high-performance'
        });
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.05;
        this.renderer.setClearColor(0x08111d);
        this.renderer.domElement.setAttribute('role', 'img');
        this.renderer.domElement.setAttribute(
            'aria-label',
            'Interactive Unitree G1 humanoid dynamics simulation. Click to set a goal, drag to orbit, and use the camera controls to follow the robot.'
        );
        this.renderer.domElement.setAttribute('aria-describedby', 'unitree-g1-help');
        this.viewport.appendChild(this.renderer.domElement);

        this.scene = new THREE.Scene();
        this.scene.fog = new THREE.FogExp2(0x08111d, 0.022);
        this.camera = new THREE.PerspectiveCamera(48, 1, 0.04, 100);
        this.camera.up.set(0, 0, 1);

        this.scene.add(new THREE.HemisphereLight(0xc7e8ff, 0x182237, 2.1));
        const keyLight = new THREE.DirectionalLight(0xffffff, 2.5);
        keyLight.position.set(-4, -3, 8);
        this.scene.add(keyLight);
        const rimLight = new THREE.DirectionalLight(0x38bdf8, 1.2);
        rimLight.position.set(5, 4, 3);
        this.scene.add(rimLight);

        const floor = new THREE.Mesh(
            new THREE.PlaneGeometry(24, 24),
            new THREE.MeshStandardMaterial({
                color: 0x0b1725,
                roughness: 0.92,
                metalness: 0.02
            })
        );
        floor.position.z = -0.006;
        this.scene.add(floor);

        const grid = new THREE.GridHelper(24, 48, 0x38bdf8, 0x1b3045);
        grid.rotation.x = Math.PI / 2;
        grid.position.z = 0;
        grid.material.transparent = true;
        grid.material.opacity = 0.42;
        this.scene.add(grid);

        this.goalMarker = new THREE.Group();
        const goalRing = new THREE.Mesh(
            new THREE.TorusGeometry(0.3, 0.035, 10, 40),
            new THREE.MeshStandardMaterial({
                color: 0x4ade80,
                emissive: 0x14532d,
                emissiveIntensity: 1.3,
                roughness: 0.35
            })
        );
        this.goalMarker.add(goalRing);
        const goalStem = new THREE.Mesh(
            new THREE.CylinderGeometry(0.016, 0.016, 0.42, 10),
            new THREE.MeshBasicMaterial({ color: 0x86efac })
        );
        goalStem.rotation.x = Math.PI / 2;
        goalStem.position.z = 0.21;
        this.goalMarker.add(goalStem);
        const goalDot = new THREE.Mesh(
            new THREE.SphereGeometry(0.065, 14, 10),
            new THREE.MeshBasicMaterial({ color: 0xbbf7d0 })
        );
        goalDot.position.z = 0.43;
        this.goalMarker.add(goalDot);
        this.goalHeadingArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.05), 0.6, 0x86efac, 0.15, 0.09);
        this.goalHeadingArrow.visible = false;
        this.goalMarker.add(this.goalHeadingArrow);
        this.scene.add(this.goalMarker);

        this.goalArrivalZone = new THREE.Group();
        const arrivalDisc = new THREE.Mesh(
            new THREE.CircleGeometry(0.9, 64),
            new THREE.MeshBasicMaterial({
                color: 0x4ade80,
                transparent: true,
                opacity: 0.045,
                side: THREE.DoubleSide,
                depthWrite: false
            })
        );
        const arrivalRing = new THREE.Mesh(
            new THREE.RingGeometry(0.87, 0.9, 64),
            new THREE.MeshBasicMaterial({
                color: 0x4ade80,
                transparent: true,
                opacity: 0.48,
                side: THREE.DoubleSide,
                depthWrite: false
            })
        );
        this.goalArrivalZone.add(arrivalDisc, arrivalRing);
        this.goalArrivalZone.position.z = 0.012;
        this.scene.add(this.goalArrivalZone);

        this.routePositions = new Float32Array(6);
        this.routeDistances = new Float32Array(2);
        const routeGeometry = new THREE.BufferGeometry();
        this.routePositionAttribute = new THREE.BufferAttribute(this.routePositions, 3);
        this.routePositionAttribute.setUsage(THREE.DynamicDrawUsage);
        this.routeDistanceAttribute = new THREE.BufferAttribute(this.routeDistances, 1);
        this.routeDistanceAttribute.setUsage(THREE.DynamicDrawUsage);
        routeGeometry.setAttribute('position', this.routePositionAttribute);
        routeGeometry.setAttribute('lineDistance', this.routeDistanceAttribute);
        this.routeLine = new THREE.Line(
            routeGeometry,
            new THREE.LineDashedMaterial({
                color: 0x4ade80,
                transparent: true,
                opacity: 0.56,
                dashSize: 0.12,
                gapSize: 0.08
            })
        );
        this.routeLine.frustumCulled = false;
        this.scene.add(this.routeLine);

        this.trailPositions = new Float32Array(TRAIL_CAPACITY * 3);
        const trailGeometry = new THREE.BufferGeometry();
        this.trailPositionAttribute = new THREE.BufferAttribute(this.trailPositions, 3);
        this.trailPositionAttribute.setUsage(THREE.DynamicDrawUsage);
        trailGeometry.setAttribute('position', this.trailPositionAttribute);
        trailGeometry.setDrawRange(0, 0);
        this.trailLine = new THREE.Line(
            trailGeometry,
            new THREE.LineBasicMaterial({
                color: 0x38bdf8,
                transparent: true,
                opacity: 0.68
            })
        );
        this.trailLine.frustumCulled = false;
        this.scene.add(this.trailLine);

        this.safetyVisuals = new UnitreeG1SafetyVisuals(this.scene);
        this._buildHud();
        this._positionCamera();
    }

    _buildHud() {
        this.hud = document.createElement('div');
        this.hud.className = 'unitree-g1-hud';
        this.hud.innerHTML = `
            <div class="unitree-g1-status" role="status" aria-live="polite" aria-atomic="true">
                <span class="unitree-g1-status-dot"></span>
                <strong data-g1-status>UNITREE G1 · LOADING</strong>
            </div>
            <div class="unitree-g1-metrics">
                <span>Goal <strong data-g1-distance>—</strong></span>
                <span>Command <strong data-g1-command>—</strong></span>
                <span>Policy <strong data-g1-policy>—</strong></span>
            </div>
            <button class="unitree-g1-pause-btn" type="button"
                data-g1-interactive data-g1-pause aria-pressed="false"
                aria-label="Pause Unitree G1 simulation" disabled>Pause</button>
        `;
        this.viewport.appendChild(this.hud);
        this.statusText = this.hud.querySelector('[data-g1-status]');
        this.distanceText = this.hud.querySelector('[data-g1-distance]');
        this.commandText = this.hud.querySelector('[data-g1-command]');
        this.policyText = this.hud.querySelector('[data-g1-policy]');
        this.pauseButton = this.hud.querySelector('[data-g1-pause]');
        this.pauseButton.addEventListener('click', (event) => {
            event.stopPropagation();
            this.onPauseToggle();
        });
        this._buildCameraControls();
        this._buildTelemetry();

        this.help = document.createElement('div');
        this.help.className = 'unitree-g1-help';
        this.help.id = 'unitree-g1-help';
        this.help.textContent = 'Click to set goal · Drag to orbit · Right-drag to pan · Use the BEV map to launch obstacles';
        this.viewport.appendChild(this.help);
    }

    _buildCameraControls() {
        this.cameraControls = document.createElement('div');
        this.cameraControls.dataset.g1Interactive = 'true';
        this.cameraControls.setAttribute('role', 'group');
        this.cameraControls.setAttribute('aria-label', 'Camera view');
        applyStyles(this.cameraControls, {
            display: 'flex',
            alignItems: 'center',
            gap: '5px',
            padding: '5px 6px',
            border: '1px solid rgba(125, 211, 252, 0.18)',
            borderRadius: '12px',
            color: '#8ca3bd',
            background: 'rgba(5, 13, 25, 0.74)',
            boxShadow: '0 14px 40px rgba(0, 0, 0, 0.24)',
            backdropFilter: 'blur(14px)',
            pointerEvents: 'auto',
            font: '500 10px "IBM Plex Mono", monospace'
        });

        const label = document.createElement('span');
        label.textContent = 'CAMERA';
        applyStyles(label, {
            padding: '0 4px 0 3px',
            letterSpacing: '0.06em'
        });
        this.cameraControls.appendChild(label);

        this.cameraButtons = new Map();
        for (const [mode, text] of [['orbit', 'Orbit'], ['track', 'Track']]) {
            const button = document.createElement('button');
            button.type = 'button';
            button.textContent = text;
            button.dataset.cameraMode = mode;
            button.setAttribute('aria-label', mode === 'track'
                ? 'Track the humanoid from a third-person camera'
                : 'Use a free orbit camera');
            applyStyles(button, {
                minWidth: '52px',
                padding: '5px 9px',
                border: '1px solid rgba(125, 211, 252, 0.2)',
                borderRadius: '8px',
                color: '#b9cee2',
                background: 'rgba(15, 31, 48, 0.72)',
                cursor: 'pointer',
                font: 'inherit'
            });
            button.addEventListener('click', () => this.setCameraMode(mode));
            this.cameraButtons.set(mode, button);
            this.cameraControls.appendChild(button);
        }

        for (const eventName of ['pointerdown', 'pointermove', 'pointerup', 'wheel']) {
            this.cameraControls.addEventListener(eventName, (event) => event.stopPropagation());
        }
        this.hud.appendChild(this.cameraControls);
        this._updateCameraButtons();
    }

    _buildTelemetry() {
        this.telemetryPanel = document.createElement('section');
        this.telemetryPanel.setAttribute('aria-label', 'Live dynamics telemetry');
        applyStyles(this.telemetryPanel, {
            width: `${TELEMETRY_WIDTH}px`,
            padding: '8px 9px 7px',
            border: '1px solid rgba(125, 211, 252, 0.18)',
            borderRadius: '13px',
            color: '#8ca3bd',
            background: 'rgba(5, 13, 25, 0.78)',
            boxShadow: '0 14px 40px rgba(0, 0, 0, 0.24)',
            backdropFilter: 'blur(14px)'
        });

        const header = document.createElement('div');
        applyStyles(header, {
            display: 'flex',
            justifyContent: 'space-between',
            gap: '10px',
            marginBottom: '4px',
            font: '600 9px "IBM Plex Mono", monospace',
            letterSpacing: '0.05em'
        });
        header.innerHTML = `
            <span style="color:#dbeafe">LIVE DYNAMICS</span>
            <span><i style="display:inline-block;width:6px;height:6px;border-radius:50%;background:#4ade80;box-shadow:0 0 8px rgba(74,222,128,.8);margin-right:4px"></i>MUJOCO · 500 HZ</span>
        `;
        this.telemetryPanel.appendChild(header);

        this.telemetryCanvas = document.createElement('canvas');
        this.telemetryCanvas.setAttribute('role', 'img');
        this.telemetryCanvas.setAttribute(
            'aria-label',
            'Live plot of commanded forward velocity and forward body velocity measured by the simulator'
        );
        applyStyles(this.telemetryCanvas, {
            position: 'static',
            inset: 'auto',
            display: 'block',
            width: `${TELEMETRY_WIDTH}px`,
            height: `${TELEMETRY_HEIGHT}px`,
            touchAction: 'auto'
        });
        this.telemetryContext = this.telemetryCanvas.getContext('2d');
        this.telemetryPanel.appendChild(this.telemetryCanvas);

        const values = document.createElement('div');
        applyStyles(values, {
            display: 'flex',
            justifyContent: 'space-between',
            gap: '8px',
            marginTop: '3px',
            color: '#8ca3bd',
            font: '500 9px "IBM Plex Mono", monospace'
        });
        values.innerHTML = `
            <span><i style="color:#38bdf8;font-style:normal">●</i> command <strong data-g1-plot-command style="color:#dbeafe;font-weight:500">0.00 m/s</strong></span>
            <span><i style="color:#4ade80;font-style:normal">●</i> body <strong data-g1-plot-measured style="color:#dbeafe;font-weight:500">0.00 m/s</strong></span>
        `;
        this.telemetryPanel.appendChild(values);
        this.telemetryCommandText = values.querySelector('[data-g1-plot-command]');
        this.telemetryMeasuredText = values.querySelector('[data-g1-plot-measured]');
        this.hud.appendChild(this.telemetryPanel);
        this._drawTelemetry();
    }

    setStatus(message, error = false) {
        this.statusText.textContent = `UNITREE G1 · ${message.toUpperCase()}`;
        this.hud.classList.toggle('error', error);
    }

    attachRuntime(runtime) {
        this._clearRobotObjects();
        this.runtime = runtime;
        const robot = runtime.getRobotState?.();
        if (robot) {
            this.lastRobotPosition.set(
                robot.position[0],
                robot.position[1],
                robot.position[2]
            );
            this.lastRobotYaw = robot.yaw;
            this.trackHeading = robot.yaw;
            this.worldTarget.set(
                robot.position[0],
                robot.position[1],
                Math.max(0.72, robot.position[2] + 0.08)
            );
            this.target.copy(this.worldTarget);
            this.desiredTarget.copy(this.worldTarget);
        }
        this._resetTelemetry();
        this._buildRobotObjects();
    }

    detachRuntime() {
        this._clearRobotObjects();
        for (const geometry of this.geometryCache.values()) geometry.dispose();
        this.geometryCache.clear();
        this.runtime = null;
    }

    _clearRobotObjects() {
        for (const { object } of this.robotObjects) {
            disposeMaterial(object.material);
            this.scene.remove(object);
        }
        this.robotObjects.length = 0;
    }

    _buildRobotObjects() {
        const { model } = this.runtime;
        const seenMeshes = new Set();
        for (let geomId = 0; geomId < model.ngeom; geomId += 1) {
            const type = model.geom_type[geomId];
            const group = model.geom_group[geomId];
            if (type === GEOM.PLANE || group >= 3) continue;

            if (type === GEOM.MESH) {
                const duplicateKey = `${model.geom_bodyid[geomId]}:${model.geom_dataid[geomId]}`;
                if (seenMeshes.has(duplicateKey)) continue;
                seenMeshes.add(duplicateKey);
            }

            const geometry = this._getGeometry(type, geomId);
            if (!geometry) continue;
            const rgbaOffset = geomId * 4;
            const opacity = model.geom_rgba[rgbaOffset + 3];
            const material = new THREE.MeshStandardMaterial({
                color: new THREE.Color(
                    model.geom_rgba[rgbaOffset],
                    model.geom_rgba[rgbaOffset + 1],
                    model.geom_rgba[rgbaOffset + 2]
                ),
                metalness: type === GEOM.MESH ? 0.48 : 0.18,
                roughness: type === GEOM.MESH ? 0.34 : 0.58,
                transparent: opacity < 0.999,
                opacity
            });
            const object = new THREE.Mesh(geometry, material);
            object.matrixAutoUpdate = false;
            object.frustumCulled = false;
            this.robotObjects.push({ geomId, object });
            this.scene.add(object);
        }
    }

    _getGeometry(type, geomId) {
        const { model } = this.runtime;
        const sizeOffset = geomId * 3;
        const sx = model.geom_size[sizeOffset];
        const sy = model.geom_size[sizeOffset + 1];
        const sz = model.geom_size[sizeOffset + 2];

        if (type === GEOM.MESH) {
            const meshId = model.geom_dataid[geomId];
            const key = `mesh:${meshId}`;
            if (this.geometryCache.has(key)) return this.geometryCache.get(key);

            const vertexAddress = model.mesh_vertadr[meshId] * 3;
            const vertexCount = model.mesh_vertnum[meshId];
            const faceAddress = model.mesh_faceadr[meshId] * 3;
            const faceCount = model.mesh_facenum[meshId];
            const positions = new Float32Array(
                model.mesh_vert.subarray(vertexAddress, vertexAddress + vertexCount * 3)
            );
            const indices = new Uint32Array(
                model.mesh_face.subarray(faceAddress, faceAddress + faceCount * 3)
            );
            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            geometry.setIndex(new THREE.BufferAttribute(indices, 1));
            geometry.computeVertexNormals();
            geometry.computeBoundingSphere();
            this.geometryCache.set(key, geometry);
            return geometry;
        }

        const key = `${type}:${sx}:${sy}:${sz}`;
        if (this.geometryCache.has(key)) return this.geometryCache.get(key);
        let geometry = null;
        if (type === GEOM.SPHERE) {
            geometry = new THREE.SphereGeometry(sx, 20, 14);
        } else if (type === GEOM.CAPSULE) {
            geometry = new THREE.CapsuleGeometry(sx, 2 * sy, 8, 16);
            geometry.rotateX(Math.PI / 2);
        } else if (type === GEOM.ELLIPSOID) {
            geometry = new THREE.SphereGeometry(1, 20, 14);
            geometry.scale(sx, sy, sz);
        } else if (type === GEOM.CYLINDER) {
            geometry = new THREE.CylinderGeometry(sx, sx, 2 * sy, 20);
            geometry.rotateX(Math.PI / 2);
        } else if (type === GEOM.BOX) {
            geometry = new THREE.BoxGeometry(2 * sx, 2 * sy, 2 * sz);
        }
        if (geometry) this.geometryCache.set(key, geometry);
        return geometry;
    }

    _updateRobotTransforms() {
        const matrices = this.runtime.data.geom_xmat;
        const positions = this.runtime.data.geom_xpos;
        for (const { geomId, object } of this.robotObjects) {
            const matrixOffset = geomId * 9;
            const positionOffset = geomId * 3;
            object.matrix.set(
                matrices[matrixOffset],
                matrices[matrixOffset + 1],
                matrices[matrixOffset + 2],
                positions[positionOffset],
                matrices[matrixOffset + 3],
                matrices[matrixOffset + 4],
                matrices[matrixOffset + 5],
                positions[positionOffset + 1],
                matrices[matrixOffset + 6],
                matrices[matrixOffset + 7],
                matrices[matrixOffset + 8],
                positions[positionOffset + 2],
                0, 0, 0, 1
            );
            object.matrixWorldNeedsUpdate = true;
        }
    }

    _updateLines(snapshot) {
        const robot = snapshot.robot.position;
        const goal = snapshot.goal;
        this.routePositions[0] = robot[0];
        this.routePositions[1] = robot[1];
        this.routePositions[2] = 0.035;
        this.routePositions[3] = goal[0];
        this.routePositions[4] = goal[1];
        this.routePositions[5] = 0.035;
        this.routeDistances[0] = 0;
        this.routeDistances[1] = Math.hypot(
            goal[0] - robot[0],
            goal[1] - robot[1]
        );
        this.routePositionAttribute.needsUpdate = true;
        this.routeDistanceAttribute.needsUpdate = true;

        const trail = snapshot.trail ?? [];
        const trailCount = Math.min(trail.length, TRAIL_CAPACITY);
        const trailStart = trail.length - trailCount;
        for (let index = 0; index < trailCount; index += 1) {
            const point = trail[trailStart + index];
            this.trailPositions[index * 3] = point[0];
            this.trailPositions[index * 3 + 1] = point[1];
            this.trailPositions[index * 3 + 2] = 0.025;
        }
        this.trailPositionAttribute.needsUpdate = true;
        this.trailLine.geometry.setDrawRange(0, trailCount);
    }

    _resize() {
        const width = Math.max(1, this.hostEl.clientWidth);
        const height = Math.max(1, this.hostEl.clientHeight);
        const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
        const needsResize = this.renderer.domElement.width !== Math.floor(width * pixelRatio)
            || this.renderer.domElement.height !== Math.floor(height * pixelRatio);
        if (!needsResize) return;
        this.renderer.setPixelRatio(pixelRatio);
        this.renderer.setSize(width, height, false);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
    }

    _effectiveAzimuth() {
        return this.cameraMode === 'track'
            ? this.trackHeading + Math.PI + this.orbit.azimuth
            : this.orbit.azimuth;
    }

    _positionCamera() {
        const horizontal = this.orbit.radius * Math.cos(this.orbit.elevation);
        const azimuth = this._effectiveAzimuth();
        this.camera.position.set(
            this.target.x + horizontal * Math.cos(azimuth),
            this.target.y + horizontal * Math.sin(azimuth),
            this.target.z + this.orbit.radius * Math.sin(this.orbit.elevation)
        );
        this.camera.lookAt(this.target);
    }

    _updateCameraTarget(snapshot) {
        const robot = snapshot.robot;
        const previousRobotX = this.lastRobotPosition.x;
        const previousRobotY = this.lastRobotPosition.y;
        this.lastRobotPosition.set(robot.position[0], robot.position[1], robot.position[2]);
        this.lastRobotYaw = robot.yaw;

        if (this.cameraMode === 'track') {
            const headingDelta = Math.atan2(
                Math.sin(robot.yaw - this.trackHeading),
                Math.cos(robot.yaw - this.trackHeading)
            );
            this.trackHeading += headingDelta * 0.14;
            this.desiredTarget.set(
                robot.position[0] + this.trackOffset.x,
                robot.position[1] + this.trackOffset.y,
                Math.max(0.72, robot.position[2] + 0.08)
            );
        } else {

            this.worldTarget.x += robot.position[0] - previousRobotX;
            this.worldTarget.y += robot.position[1] - previousRobotY;
            this.worldTarget.z = Math.max(0.72, robot.position[2] + 0.08);
            this.desiredTarget.copy(this.worldTarget);
        }
        this.target.lerp(this.desiredTarget, this.cameraMode === 'track' ? 0.16 : 0.24);
    }

    _orbitCamera(dx, dy) {
        this.orbit.azimuth -= dx * 0.006;
        this.orbit.elevation = THREE.MathUtils.clamp(
            this.orbit.elevation + dy * 0.004,
            0.08,
            1.18
        );
    }

    _panCamera(dx, dy) {
        const azimuth = this._effectiveAzimuth();
        const scale = this.orbit.radius * 0.0014;
        this.panRight.set(-Math.sin(azimuth), Math.cos(azimuth), 0);
        this.panForward.set(-Math.cos(azimuth), -Math.sin(azimuth), 0);
        this.panRight.multiplyScalar(-dx * scale);
        this.panForward.multiplyScalar(dy * scale);
        this.panRight.add(this.panForward);
        if (this.cameraMode === 'track') {
            this.trackOffset.add(this.panRight);
        } else {
            this.worldTarget.add(this.panRight);
        }
        this.target.add(this.panRight);
    }

    setCameraMode(mode) {
        if (!Object.hasOwn(this.cameraViews, mode)) return;
        if (mode === this.cameraMode) {
            this._updateCameraButtons();
            return;
        }
        if (mode === 'orbit') {
            this.worldTarget.copy(this.target);
        } else {
            this.trackHeading = this.lastRobotYaw;
            this.trackOffset.set(0, 0, 0);
        }
        this.cameraMode = mode;
        this.orbit = this.cameraViews[mode];
        this._updateCameraButtons();
        this._positionCamera();
    }

    _updateCameraButtons() {
        if (!this.cameraButtons) return;
        for (const [mode, button] of this.cameraButtons) {
            const active = mode === this.cameraMode;
            button.setAttribute('aria-pressed', String(active));
            button.style.color = active ? '#e0f2fe' : '#9db2c8';
            button.style.borderColor = active
                ? 'rgba(56, 189, 248, 0.58)'
                : 'rgba(125, 211, 252, 0.2)';
            button.style.background = active
                ? 'rgba(14, 116, 144, 0.42)'
                : 'rgba(15, 31, 48, 0.72)';
        }
    }

    _resetTelemetry() {
        this.telemetryCommand.fill(0);
        this.telemetryMeasured.fill(0);
        this.telemetryCursor = 0;
        this.telemetryCount = 0;
        this.lastTelemetryTime = -Infinity;
        this._drawTelemetry();
    }

    _sampleTelemetry(snapshot) {
        const time = Number(snapshot.time);
        if (!Number.isFinite(time)) return;
        if (time + 1e-6 < this.lastTelemetryTime) this._resetTelemetry();
        if (time <= this.lastTelemetryTime + 1e-6) return;

        const yaw = snapshot.robot.yaw;
        const worldVelocity = snapshot.robot.worldVelocity ?? [0, 0, 0];
        const measured = worldVelocity[0] * Math.cos(yaw)
            + worldVelocity[1] * Math.sin(yaw);
        const command = (snapshot.executedCommand ?? snapshot.filteredCommand)?.[0] ?? 0;
        this.telemetryCommand[this.telemetryCursor] = Number.isFinite(command) ? command : 0;
        this.telemetryMeasured[this.telemetryCursor] = Number.isFinite(measured) ? measured : 0;
        this.telemetryCursor = (this.telemetryCursor + 1) % TELEMETRY_CAPACITY;
        this.telemetryCount = Math.min(TELEMETRY_CAPACITY, this.telemetryCount + 1);
        this.lastTelemetryTime = time;
        this.telemetryCommandText.textContent = `${command.toFixed(2)} m/s`;
        this.telemetryMeasuredText.textContent = `${measured.toFixed(2)} m/s`;
    }

    _updatePauseButton(snapshot) {
        if (!this.pauseButton) return;
        const paused = Boolean(snapshot.paused);
        const disabled = !snapshot.ready || Boolean(snapshot.failure);
        this.pauseButton.textContent = paused ? 'Resume' : 'Pause';
        this.pauseButton.setAttribute('aria-pressed', String(paused));
        this.pauseButton.setAttribute(
            'aria-label',
            paused
                ? 'Resume Unitree G1 simulation'
                : 'Pause Unitree G1 simulation'
        );
        this.pauseButton.disabled = disabled;
        this.pauseButton.classList.toggle('active', paused && !disabled);
    }

    _drawTelemetrySeries(context, source, color, chart, yMin, yMax) {
        if (this.telemetryCount < 2) return;
        const range = Math.max(1e-6, yMax - yMin);
        context.beginPath();
        for (let point = 0; point < this.telemetryCount; point += 1) {
            const sourceIndex = (
                this.telemetryCursor - this.telemetryCount + point + TELEMETRY_CAPACITY
            ) % TELEMETRY_CAPACITY;
            const x = chart.left
                + (point / Math.max(1, TELEMETRY_CAPACITY - 1)) * chart.width;
            const normalized = (source[sourceIndex] - yMin) / range;
            const y = chart.bottom - normalized * chart.height;
            if (point === 0) context.moveTo(x, y);
            else context.lineTo(x, y);
        }
        context.strokeStyle = color;
        context.lineWidth = 1.5;
        context.lineJoin = 'round';
        context.lineCap = 'round';
        context.stroke();
    }

    _drawTelemetry() {
        const context = this.telemetryContext;
        if (!context) return;
        const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
        const width = Math.floor(TELEMETRY_WIDTH * pixelRatio);
        const height = Math.floor(TELEMETRY_HEIGHT * pixelRatio);
        if (this.telemetryCanvas.width !== width || this.telemetryCanvas.height !== height) {
            this.telemetryCanvas.width = width;
            this.telemetryCanvas.height = height;
        }
        context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
        context.clearRect(0, 0, TELEMETRY_WIDTH, TELEMETRY_HEIGHT);

        const chart = {
            left: 28,
            right: TELEMETRY_WIDTH - 6,
            top: 7,
            bottom: TELEMETRY_HEIGHT - 14
        };
        chart.width = chart.right - chart.left;
        chart.height = chart.bottom - chart.top;

        let yMin = -0.15;
        let yMax = 0.95;
        for (let point = 0; point < this.telemetryCount; point += 1) {
            const sourceIndex = (
                this.telemetryCursor - this.telemetryCount + point + TELEMETRY_CAPACITY
            ) % TELEMETRY_CAPACITY;
            yMin = Math.min(
                yMin,
                this.telemetryCommand[sourceIndex],
                this.telemetryMeasured[sourceIndex]
            );
            yMax = Math.max(
                yMax,
                this.telemetryCommand[sourceIndex],
                this.telemetryMeasured[sourceIndex]
            );
        }
        const padding = Math.max(0.03, (yMax - yMin) * 0.06);
        yMin -= padding;
        yMax += padding;

        context.fillStyle = 'rgba(9, 24, 39, 0.76)';
        context.fillRect(chart.left, chart.top, chart.width, chart.height);
        context.strokeStyle = 'rgba(125, 211, 252, 0.11)';
        context.lineWidth = 1;
        for (let line = 0; line <= 2; line += 1) {
            const y = chart.top + (line / 2) * chart.height;
            context.beginPath();
            context.moveTo(chart.left, y);
            context.lineTo(chart.right, y);
            context.stroke();
        }
        if (yMin < 0 && yMax > 0) {
            const zeroY = chart.bottom - ((0 - yMin) / (yMax - yMin)) * chart.height;
            context.strokeStyle = 'rgba(219, 234, 254, 0.24)';
            context.beginPath();
            context.moveTo(chart.left, zeroY);
            context.lineTo(chart.right, zeroY);
            context.stroke();
        }

        context.fillStyle = '#7890a8';
        context.font = '8px "IBM Plex Mono", monospace';
        context.textAlign = 'right';
        context.textBaseline = 'middle';
        context.fillText(yMax.toFixed(1), chart.left - 4, chart.top + 2);
        context.fillText(yMin.toFixed(1), chart.left - 4, chart.bottom - 2);
        context.textAlign = 'left';
        context.textBaseline = 'alphabetic';
        context.fillText('3.6 s history', chart.left, TELEMETRY_HEIGHT - 2);

        this._drawTelemetrySeries(
            context,
            this.telemetryCommand,
            '#38bdf8',
            chart,
            yMin,
            yMax
        );
        this._drawTelemetrySeries(
            context,
            this.telemetryMeasured,
            '#4ade80',
            chart,
            yMin,
            yMax
        );
    }

    render(snapshot) {
        if (!this.visible) return;
        this._resize();
        this._updatePauseButton(snapshot);
        if (this.runtime?.ready) {
            this._updateRobotTransforms();
            this.goalMarker.position.set(snapshot.goal[0], snapshot.goal[1], 0.02);
            this.goalArrivalZone.position.set(snapshot.goal[0], snapshot.goal[1], 0.012);
            this.goalArrivalZone.scale.setScalar((snapshot.goalRadius ?? 0.9) / 0.9);
            this.goalHeadingArrow.visible = Number.isFinite(snapshot.goalYaw);
            if (this.goalHeadingArrow.visible) this.goalHeadingArrow.setDirection(new THREE.Vector3(Math.cos(snapshot.goalYaw), Math.sin(snapshot.goalYaw), 0));
            const pulse = 1 + 0.08 * Math.sin((snapshot.time ?? 0) * 5);
            this.goalMarker.scale.setScalar(pulse);
            this._updateLines(snapshot);
            this.safetyVisuals.update(snapshot);
            this._updateCameraTarget(snapshot);
            this._positionCamera();
            this._sampleTelemetry(snapshot);
            this._drawTelemetry();

            this.distanceText.textContent = `${snapshot.distance.toFixed(2)} m`;
            this.commandText.textContent = (snapshot.executedCommand ?? snapshot.filteredCommand)
                .map((value) => value.toFixed(2)).join(', ');
            this.policyText.textContent = `${snapshot.diagnostics.inferenceMs.toFixed(1)} ms`;
            const label = snapshot.failure?.reason === 'collision'
                ? 'Collision'
                : snapshot.paused
                    ? 'Paused'
                    : snapshot.inCollision
                        ? 'Reduced-model contact'
                    : snapshot.dpcbf?.intervening
                        ? `${snapshot.dpcbf.visualLabel ?? 'DPCBF'} filtering`
                : snapshot.goalReached
                    ? 'Goal reached'
                    : 'Navigating';
            this.setStatus(snapshot.controllerLabel ? `${snapshot.controllerLabel} · ${label}` : label);
        }
        this.renderer.render(this.scene, this.camera);
    }

    _eventPoint(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        return {
            x: ((event.clientX - rect.left) / rect.width) * 2 - 1,
            y: -((event.clientY - rect.top) / rect.height) * 2 + 1
        };
    }

    _isInterfaceEvent(event) {
        return Boolean(event.target?.closest?.('[data-g1-interactive]'));
    }

    handlePointerDown(event) {
        if (this._isInterfaceEvent(event)) return false;
        this.drag = {
            button: event.button,
            x: event.clientX,
            y: event.clientY,
            lastX: event.clientX,
            lastY: event.clientY,
            moved: false
        };
        this.hostEl.setPointerCapture?.(event.pointerId);
        this.renderer.domElement.style.cursor = 'grabbing';
        return true;
    }

    handlePointerMove(event) {
        if (!this.drag) return false;
        const dx = event.clientX - this.drag.lastX;
        const dy = event.clientY - this.drag.lastY;
        if (Math.hypot(event.clientX - this.drag.x, event.clientY - this.drag.y) > 5) {
            this.drag.moved = true;
        }
        if (this.drag.moved) {
            if (this.drag.button === 2) this._panCamera(dx, dy);
            else if (this.drag.button === 0 || this.drag.button === 1) {
                this._orbitCamera(dx, dy);
            }
            this._positionCamera();
        }
        this.drag.lastX = event.clientX;
        this.drag.lastY = event.clientY;
        return true;
    }

    handlePointerUp(event) {
        if (!this.drag) return false;
        const drag = this.drag;
        this.drag = null;
        this.renderer.domElement.style.cursor = 'grab';
        this.hostEl.releasePointerCapture?.(event.pointerId);
        if (drag.button === 0 && !drag.moved && this.runtime?.ready) {
            const point = this._eventPoint(event);
            this.pointer.set(point.x, point.y);
            this.raycaster.setFromCamera(this.pointer, this.camera);
            const hit = new THREE.Vector3();
            if (this.raycaster.ray.intersectPlane(this.groundPlane, hit)) {
                this.onGoalSelected([hit.x, hit.y]);
            }
        }
        return true;
    }

    handlePointerLeave() {
        this.drag = null;
        this.renderer.domElement.style.cursor = 'grab';
    }

    handleWheel(event) {
        if (this._isInterfaceEvent(event)) return false;
        this.orbit.radius = THREE.MathUtils.clamp(
            this.orbit.radius * Math.exp(event.deltaY * 0.001),
            2.1,
            10
        );
        this._positionCamera();
        return true;
    }

    resetView() {
        this.cameraViews.orbit = copyCameraPreset('orbit');
        this.cameraViews.track = copyCameraPreset('track');
        this.cameraMode = 'orbit';
        this.orbit = this.cameraViews.orbit;
        this.trackOffset.set(0, 0, 0);
        this.trackHeading = this.lastRobotYaw;
        this.worldTarget.set(
            this.lastRobotPosition.x,
            this.lastRobotPosition.y,
            Math.max(0.72, this.lastRobotPosition.z + 0.08)
        );
        this.target.copy(this.worldTarget);
        this.desiredTarget.copy(this.worldTarget);
        this._updateCameraButtons();
        this._positionCamera();
    }

    setVisible(visible) {
        this.visible = visible;
        this.hostEl.hidden = !visible;
    }

    dispose() {
        this.detachRuntime();
        this.safetyVisuals.dispose();
        this.routeLine.geometry.dispose();
        this.trailLine.geometry.dispose();
        this.renderer.dispose();
        this.hostEl.replaceChildren();
    }
}
