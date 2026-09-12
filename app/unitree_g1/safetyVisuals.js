import * as THREE from '../../vendor/three.module.js';
import { GLTFLoader } from '../../vendor/GLTFLoader.js';
import {
    G1_OBSTACLE_DEFAULTS,
    getG1ObstacleCollisionRadius
} from './obstacles.js?v=dodger-3';
import { getActiveDpcbfConstraintIds } from './dpcbfVisualState.js?v=dodger-3';

const MAX_OVERLAY_POINTS = 72;
const COMMAND_ARROW_TORSO_OFFSET = 0.22;
const COMMAND_ARROW_Z_SEPARATION = 0.075;
const OBSTACLE_MODEL_HEIGHT = 1.05;
const OBSTACLE_MODEL_URL = new URL(
    '../../assets/unitree_g1/robot_expressive.glb',
    import.meta.url
).href;
const DPCBF_PALETTE = Object.freeze([
    0xf97316,
    0x38bdf8,
    0x22c55e,
    0xf59e0b,
    0xfb7185,
    0xa78bfa,
    0x2dd4bf,
    0xfacc15
]);
const ACTIVE_CONSTRAINT_COLOR = 0xef4444;

function disposeObject(root) {
    root.traverse((object) => {
        object.geometry?.dispose?.();
        if (Array.isArray(object.material)) {
            object.material.forEach((material) => material.dispose?.());
        } else {
            object.material?.dispose?.();
        }
    });
}

function parallelTraverse(source, clone, callback) {
    callback(source, clone);
    for (let index = 0; index < source.children.length; index += 1) {
        parallelTraverse(source.children[index], clone.children[index], callback);
    }
}

function cloneSkinnedModel(source) {
    const sourceLookup = new Map();
    const cloneLookup = new Map();
    const clone = source.clone();
    parallelTraverse(source, clone, (sourceNode, clonedNode) => {
        sourceLookup.set(clonedNode, sourceNode);
        cloneLookup.set(sourceNode, clonedNode);
    });
    clone.traverse((node) => {
        if (!node.isSkinnedMesh) return;
        const sourceMesh = sourceLookup.get(node);
        node.skeleton = sourceMesh.skeleton.clone();
        node.bindMatrix.copy(sourceMesh.bindMatrix);
        node.skeleton.bones = sourceMesh.skeleton.bones.map(
            (bone) => cloneLookup.get(bone)
        );
        node.bind(node.skeleton, node.bindMatrix);
    });
    return clone;
}

let obstacleModelPromise = null;

function loadObstacleModel() {
    obstacleModelPromise ??= new Promise((resolve, reject) => {
        new GLTFLoader().load(
            OBSTACLE_MODEL_URL,
            (gltf) => resolve({
                scene: gltf.scene,
                animations: gltf.animations
            }),
            undefined,
            reject
        );
    });
    return obstacleModelPromise;
}

function createNormalizedObstacleModel(template) {
    const model = cloneSkinnedModel(template.scene);
    const yUpToZUp = new THREE.Group();
    yUpToZUp.rotation.x = Math.PI / 2;
    yUpToZUp.add(model);

    const aligned = new THREE.Group();
    aligned.rotation.z = Math.PI / 2;
    aligned.add(yUpToZUp);
    aligned.updateMatrixWorld(true);

    const bounds = new THREE.Box3().setFromObject(aligned);
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    bounds.getSize(size);
    bounds.getCenter(center);
    const scale = OBSTACLE_MODEL_HEIGHT / Math.max(size.z, 1e-6);
    aligned.scale.setScalar(scale);
    aligned.position.set(
        -center.x * scale,
        -center.y * scale,
        -bounds.min.z * scale + 0.006
    );
    model.traverse((object) => {
        if (!object.isMesh) return;
        object.frustumCulled = false;
    });
    return { root: aligned, animationRoot: model };
}

function createFlatRibbon(color) {
    const positions = new Float32Array(MAX_OVERLAY_POINTS * 2 * 3);
    const indices = new Uint16Array((MAX_OVERLAY_POINTS - 1) * 6);
    for (let index = 0; index < MAX_OVERLAY_POINTS - 1; index += 1) {
        const vertex = index * 2;
        const offset = index * 6;
        indices.set([
            vertex, vertex + 1, vertex + 2,
            vertex + 1, vertex + 3, vertex + 2
        ], offset);
    }
    const geometry = new THREE.BufferGeometry();
    const attribute = new THREE.BufferAttribute(positions, 3);
    attribute.setUsage(THREE.DynamicDrawUsage);
    geometry.setAttribute('position', attribute);
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    geometry.setDrawRange(0, 0);
    const material = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.9,
        side: THREE.DoubleSide,
        depthTest: false,
        depthWrite: false
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.frustumCulled = false;
    mesh.renderOrder = 20;
    return { mesh, positions, attribute };
}

function updateFlatRibbon(entry, points, width = 0.04, z = 0.028) {
    const count = Math.min(points?.length ?? 0, MAX_OVERLAY_POINTS);
    if (count < 2) {
        entry.mesh.visible = false;
        entry.mesh.geometry.setDrawRange(0, 0);
        return;
    }
    const halfWidth = width / 2;
    for (let index = 0; index < count; index += 1) {
        const previous = points[Math.max(0, index - 1)];
        const next = points[Math.min(count - 1, index + 1)];
        const dx = next[0] - previous[0];
        const dy = next[1] - previous[1];
        const magnitude = Math.max(Math.hypot(dx, dy), 1e-6);
        const nx = -dy / magnitude * halfWidth;
        const ny = dx / magnitude * halfWidth;
        const offset = index * 6;
        entry.positions[offset] = points[index][0] + nx;
        entry.positions[offset + 1] = points[index][1] + ny;
        entry.positions[offset + 2] = z;
        entry.positions[offset + 3] = points[index][0] - nx;
        entry.positions[offset + 4] = points[index][1] - ny;
        entry.positions[offset + 5] = z;
    }
    entry.attribute.needsUpdate = true;
    entry.mesh.geometry.setDrawRange(0, (count - 1) * 6);
    entry.mesh.visible = true;
}

function createGroundArrow(color, opacity = 0.92) {
    const material = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity,
        side: THREE.DoubleSide,
        depthTest: false,
        depthWrite: false
    });
    const shaft = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), material.clone());
    const headGeometry = new THREE.BufferGeometry();
    headGeometry.setAttribute('position', new THREE.BufferAttribute(
        new Float32Array([
            0, 0, 0,
            -1, 0.5, 0,
            -1, -0.5, 0
        ]),
        3
    ));
    const head = new THREE.Mesh(headGeometry, material);
    const root = new THREE.Group();
    root.add(shaft, head);
    root.visible = false;
    root.renderOrder = 22;
    shaft.renderOrder = 22;
    head.renderOrder = 22;
    return { root, shaft, head };
}

function setArrowColor(arrow, color, opacity) {
    arrow.shaft.material.color.setHex(color);
    arrow.head.material.color.setHex(color);
    arrow.shaft.material.opacity = opacity;
    arrow.head.material.opacity = opacity;
}

function updateGroundArrow(
    arrow,
    start,
    end,
    {
        color,
        opacity = 0.92,
        width = 0.025,
        headLength = 0.129,
        headWidth = 0.107,
        z = 0.09
    }
) {
    const dx = end[0] - start[0];
    const dy = end[1] - start[1];
    const length = Math.hypot(dx, dy);
    if (length < 0.025) {
        arrow.root.visible = false;
        return;
    }
    const shaftLength = Math.max(0.01, length - headLength * 0.76);
    arrow.root.position.set(start[0], start[1], z);
    arrow.root.rotation.z = Math.atan2(dy, dx);
    arrow.shaft.position.set(shaftLength / 2, 0, 0);
    arrow.shaft.scale.set(shaftLength, width, 0.012);
    arrow.head.position.set(length, 0, 0.002);
    arrow.head.scale.set(headLength, headWidth, 1);
    setArrowColor(arrow, color, opacity);
    arrow.root.visible = true;
}

function createPredictionLine() {
    const positions = new Float32Array(MAX_OVERLAY_POINTS * 3);
    const geometry = new THREE.BufferGeometry();
    const attribute = new THREE.BufferAttribute(positions, 3);
    attribute.setUsage(THREE.DynamicDrawUsage);
    geometry.setAttribute('position', attribute);
    geometry.setDrawRange(0, 0);
    const line = new THREE.Line(
        geometry,
        new THREE.LineDashedMaterial({
            color: 0xfacc15,
            transparent: true,
            opacity: 0.58,
            dashSize: 0.11,
            gapSize: 0.075,
            depthTest: false,
            depthWrite: false
        })
    );
    line.frustumCulled = false;
    line.renderOrder = 18;
    return { line, positions, attribute };
}

export class UnitreeG1SafetyVisuals {
    constructor(
        scene,
        {
            loadModel = typeof window !== 'undefined',
            modelTemplate = null
        } = {}
    ) {
        this.scene = scene;
        this.assets = new Map();
        this.previewAsset = null;
        this.boundaries = new Map();
        this.relativeArrows = new Map();
        this.coneFills = new Map();
        this.obstacleTemplate = modelTemplate;
        this.disposed = false;

        this.nominalArrow = createGroundArrow(0x38bdf8, 0.9);
        this.filteredArrow = createGroundArrow(0xfacc15, 0.98);
        this.nominalArrow.root.name = 'g1-command-nominal';
        this.filteredArrow.root.name = 'g1-command-filtered';
        this.scene.add(this.nominalArrow.root, this.filteredArrow.root);

        const prediction = createPredictionLine();
        this.predictionLine = prediction.line;
        this.predictionPositions = prediction.positions;
        this.predictionAttribute = prediction.attribute;
        this.scene.add(this.predictionLine);

        if (loadModel && !this.obstacleTemplate) {
            this.obstacleLoadPromise = loadObstacleModel()
                .then((template) => {
                    if (this.disposed) return;
                    this.obstacleTemplate = template;
                    for (const asset of this.assets.values()) {
                        this._attachObstacleModel(asset);
                    }
                    if (this.previewAsset) this._attachObstacleModel(this.previewAsset);
                })
                .catch((error) => {
                    if (!this.disposed) {
                        console.warn('Unable to load the G1 obstacle model.', error);
                    }
                });
        }
    }

    _createAsset(obstacle, isPreview = false) {
        const body = new THREE.Group();
        body.name = isPreview ? 'g1-obstacle-preview' : 'g1-obstacle-robot';
        body.userData.phaseOffset = [...String(obstacle.id)]
            .reduce((sum, character) => sum + character.charCodeAt(0), 0) * 0.07;
        body.userData.heading = Math.hypot(obstacle.vx, obstacle.vy) > 1e-4
            ? Math.atan2(obstacle.vy, obstacle.vx)
            : 0;
        this.scene.add(body);

        const collisionRadius = getG1ObstacleCollisionRadius(obstacle);
        const collisionRing = new THREE.Mesh(
            new THREE.RingGeometry(
                Math.max(0.02, collisionRadius - 0.014),
                collisionRadius + 0.014,
                64
            ),
            new THREE.MeshBasicMaterial({
                color: 0x94a3b8,
                transparent: true,
                opacity: 0.72,
                side: THREE.DoubleSide,
                depthTest: false,
                depthWrite: false
            })
        );
        collisionRing.name = 'g1-obstacle-collision-ring';
        collisionRing.position.z = 0.022;
        collisionRing.renderOrder = 16;
        this.scene.add(collisionRing);

        const velocityArrow = createGroundArrow(0x94a3b8, 0.9);
        this.scene.add(velocityArrow.root);
        const asset = {
            isPreview,
            previewMaterials: new Set(),
            body,
            collisionRing,
            velocityArrow,
            model: null,
            animationRoot: null,
            mixer: null,
            idleAction: null,
            walkingAction: null,
            lastTime: null
        };
        this._attachObstacleModel(asset);
        return asset;
    }

    _attachObstacleModel(asset) {
        if (!this.obstacleTemplate || asset.model || this.disposed) return;
        const { root, animationRoot } = createNormalizedObstacleModel(
            this.obstacleTemplate
        );
        root.name = asset.isPreview ? 'g1-obstacle-preview-model' : 'g1-obstacle-robot-model';
        if (asset.isPreview) {
            const clones = new Map();
            const translucent = material => {
                if (!clones.has(material)) {
                    const clone = material.clone();
                    clone.transparent = true;
                    clone.opacity = .32;
                    clone.depthWrite = false;
                    clones.set(material, clone);
                    asset.previewMaterials.add(clone);
                }
                return clones.get(material);
            };
            root.traverse(object => {
                if (!object.isMesh) return;
                object.material = Array.isArray(object.material) ? object.material.map(translucent) : translucent(object.material);
            });
        }
        asset.body.add(root);
        asset.model = root;
        asset.animationRoot = animationRoot;
        asset.mixer = new THREE.AnimationMixer(animationRoot);

        const clips = new Map(
            this.obstacleTemplate.animations.map((clip) => [clip.name, clip])
        );
        const idleClip = clips.get('Idle');
        const walkingClip = clips.get('Walking');
        if (idleClip) {
            asset.idleAction = asset.mixer.clipAction(idleClip);
            asset.idleAction.play();
        }
        if (walkingClip) {
            asset.walkingAction = asset.mixer.clipAction(walkingClip);
            asset.walkingAction.play();
            asset.walkingAction.time =
                asset.body.userData.phaseOffset % walkingClip.duration;
        }
    }

    _removeAsset(asset) {
        asset.mixer?.stopAllAction();
        if (asset.animationRoot) asset.mixer?.uncacheRoot(asset.animationRoot);
        this.scene.remove(asset.body);
        asset.body.clear();
        for (const material of asset.previewMaterials) material.dispose();
        asset.previewMaterials.clear();
        for (const object of [asset.collisionRing, asset.velocityArrow.root]) {
            this.scene.remove(object);
            disposeObject(object);
        }
    }

    _updateAsset(asset, obstacle, snapshot, constraintActive) {
        const time = snapshot.time ?? 0;
        const dt = asset.lastTime == null ? 0 : Math.max(0, Math.min(0.1, time - asset.lastTime));
        asset.lastTime = time;
        const speed = Math.hypot(obstacle.vx, obstacle.vy);
        if (speed > 1e-4) {
            const targetHeading = Math.atan2(obstacle.vy, obstacle.vx);
            const delta = Math.atan2(
                Math.sin(targetHeading - asset.body.userData.heading),
                Math.cos(targetHeading - asset.body.userData.heading)
            );
            const blend = 1 - Math.exp(-6 * dt);
            asset.body.userData.heading += delta * blend;
        }
        asset.body.position.set(obstacle.x, obstacle.y, 0);
        asset.body.rotation.z = asset.body.userData.heading;

        const walkStrength = THREE.MathUtils.clamp(
            speed / G1_OBSTACLE_DEFAULTS.speedCap,
            0,
            1
        );
        asset.idleAction?.setEffectiveWeight(1 - walkStrength);
        asset.walkingAction?.setEffectiveWeight(walkStrength);
        asset.walkingAction?.setEffectiveTimeScale(0.55 + walkStrength * 0.75);
        asset.mixer?.update(dt);

        asset.collisionRing.position.set(obstacle.x, obstacle.y, 0.022);
        asset.collisionRing.material.color.setHex(
            constraintActive ? ACTIVE_CONSTRAINT_COLOR : 0x94a3b8
        );
        asset.collisionRing.material.opacity = constraintActive ? 0.92 : 0.68;
        updateGroundArrow(
            asset.velocityArrow,
            [obstacle.x, obstacle.y],
            [obstacle.x + obstacle.vx, obstacle.y + obstacle.vy],
            {
                color: constraintActive ? ACTIVE_CONSTRAINT_COLOR : 0x94a3b8,
                width: 0.03,
                z: 0.075
            }
        );
    }

    _updatePreview(preview, snapshot) {
        if (!preview) {
            if (this.previewAsset) {
                this.previewAsset.body.visible = false;
                this.previewAsset.collisionRing.visible = false;
                this.previewAsset.velocityArrow.root.visible = false;
            }
            return;
        }
        const obstacle = { id: 'preview', x: preview.position[0], y: preview.position[1],
            vx: preview.velocity[0], vy: preview.velocity[1], radius: preview.radius ?? .25 };
        this.previewAsset ??= this._createAsset(obstacle, true);
        const asset = this.previewAsset;

        asset.body.userData.heading = Math.atan2(obstacle.vy, obstacle.vx);
        asset.body.visible = true;
        asset.collisionRing.visible = true;
        this._updateAsset(asset, obstacle, snapshot, false);
        const color = preview.valid === false ? 0xf87171 : 0x7dd3fc;
        asset.collisionRing.material.color.setHex(color);
        asset.collisionRing.material.opacity = .8;
        updateGroundArrow(asset.velocityArrow, preview.position,
            [obstacle.x+obstacle.vx,obstacle.y+obstacle.vy],
            { color, width: .04, headLength: .13, headWidth: .12, z: .08 });
    }

    _updatePrediction(points, enabled) {
        const count = enabled
            ? Math.min(points?.length ?? 0, MAX_OVERLAY_POINTS)
            : 0;
        if (count < 2) {
            this.predictionLine.visible = false;
            this.predictionLine.geometry.setDrawRange(0, 0);
            return;
        }
        for (let index = 0; index < count; index += 1) {
            const point = Array.isArray(points[index])
                ? points[index]
                : [points[index].x, points[index].y];
            this.predictionPositions[index * 3] = point[0];
            this.predictionPositions[index * 3 + 1] = point[1];
            this.predictionPositions[index * 3 + 2] = 0.048;
        }
        this.predictionAttribute.needsUpdate = true;
        this.predictionLine.geometry.setDrawRange(0, count);
        this.predictionLine.computeLineDistances();
        this.predictionLine.visible = true;
    }

    _updateOverlays(overlays, enabled) {
        const liveIds = new Set();
        for (const [index, overlay] of (overlays ?? []).entries()) {
            liveIds.add(overlay.obstacleId);
            const color = DPCBF_PALETTE[(overlay.colorIndex ?? index) % DPCBF_PALETTE.length];
            let boundary = this.boundaries.get(overlay.obstacleId);
            if (!boundary) {
                boundary = createFlatRibbon(color);
                boundary.mesh.name = `g1-dpcbf-boundary-${overlay.obstacleId}`;
                this.boundaries.set(overlay.obstacleId, boundary);
                this.scene.add(boundary.mesh);
            }
            boundary.mesh.material.color.setHex(color);
            boundary.mesh.material.opacity = 0.9;
            updateFlatRibbon(boundary, enabled ? overlay.points : [], 0.04, 0.028);

            let fill = this.coneFills.get(overlay.obstacleId);
            if (overlay.unsafeFill && !fill) {
                const geometry = new THREE.BufferGeometry();
                geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(9), 3));
                fill = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ color, transparent: true, opacity: .14,
                    side: THREE.DoubleSide, depthTest: false, depthWrite: false }));
                fill.name = `g1-unsafe-cone-${overlay.obstacleId}`;
                fill.renderOrder = 19;
                fill.frustumCulled = false;
                this.coneFills.set(overlay.obstacleId, fill);
                this.scene.add(fill);
            }
            if (fill) {
                fill.visible = Boolean(enabled && overlay.unsafeFill);
                if (overlay.unsafeFill) {
                    const positions = fill.geometry.attributes.position;
                    overlay.unsafeFill.forEach((point,i) => positions.setXYZ(i, point[0], point[1], .026));
                    positions.needsUpdate = true;
                    fill.material.color.setHex(color);
                }
            }
            let arrow = this.relativeArrows.get(overlay.obstacleId);
            if (!arrow) {
                arrow = createGroundArrow(color, 0.92);
                arrow.root.name = `g1-relative-velocity-${overlay.obstacleId}`;
                this.relativeArrows.set(overlay.obstacleId, arrow);
                this.scene.add(arrow.root);
            }
            const relative = overlay.relativeVelocityArrow;
            if (relative) {
                updateGroundArrow(arrow, relative.start, relative.end, {
                    color,
                    opacity: 0.92,
                    width: overlay.type === 'collision-cone' ? 0.025 : 0.012,
                    headLength: 0.105,
                    headWidth: overlay.type === 'collision-cone' ? 0.085 : 0.052,
                    z: 0.105
                });
            } else {
                arrow.root.visible = false;
            }
        }

        for (const [id, boundary] of this.boundaries) {
            if (!liveIds.has(id)) {
                this.scene.remove(boundary.mesh);
                disposeObject(boundary.mesh);
                this.boundaries.delete(id);
            }
        }
        for (const [id, fill] of this.coneFills) {
            if (!liveIds.has(id)) {
                this.scene.remove(fill);
                disposeObject(fill);
                this.coneFills.delete(id);
            }
        }
        for (const [id, arrow] of this.relativeArrows) {
            if (!liveIds.has(id)) {
                this.scene.remove(arrow.root);
                disposeObject(arrow.root);
                this.relativeArrows.delete(id);
            }
        }
    }

    _updateCommandArrows(snapshot) {
        const robot = snapshot.robot;
        const yaw = robot.yaw;
        const origin = [robot.position[0], robot.position[1]];
        const torsoZ = robot.position[2] + COMMAND_ARROW_TORSO_OFFSET;
        const update = (arrow, command, color, z, width) => {
            const speed = command?.[0] ?? 0;
            const lateral = command?.[1] ?? 0;
            const yawRate = command?.[2] ?? 0;
            const direction = yaw + yawRate * 0.7 + Math.atan2(lateral, speed);
            const length = 0.34 + Math.hypot(speed, lateral) * 1.5 + Math.abs(yawRate) * 0.14;
            updateGroundArrow(
                arrow,
                origin,
                [
                    origin[0] + Math.cos(direction) * length,
                    origin[1] + Math.sin(direction) * length
                ],
                {
                    color,
                    opacity: 0.98,
                    width,
                    headLength: 0.16,
                    headWidth: 0.14,
                    z
                }
            );
        };
        update(this.nominalArrow, snapshot.nominalCommand, 0x38bdf8, torsoZ, 0.045);
        update(
            this.filteredArrow,
            snapshot.filteredCommand,
            0xfacc15,
            torsoZ + COMMAND_ARROW_Z_SEPARATION,
            0.06
        );
        this.filteredArrow.root.visible = snapshot.dpcbf?.showFilteredCommand !== false && snapshot.dpcbf?.referenceAvailable !== false;
    }

    update(snapshot) {
        const obstacles = snapshot.obstacles ?? [];
        const dpcbf = snapshot.dpcbf ?? {};
        const activeConstraintIds = getActiveDpcbfConstraintIds(dpcbf);
        const liveIds = new Set(obstacles.map((obstacle) => obstacle.id));
        for (const [id, asset] of this.assets) {
            if (!liveIds.has(id)) {
                this._removeAsset(asset);
                this.assets.delete(id);
            }
        }
        for (const obstacle of obstacles) {
            let asset = this.assets.get(obstacle.id);
            if (!asset) {
                asset = this._createAsset(obstacle);
                this.assets.set(obstacle.id, asset);
            }
            this._updateAsset(
                asset,
                obstacle,
                snapshot,
                activeConstraintIds.has(String(obstacle.id))
            );
        }

        this._updatePreview(snapshot.obstaclePreview, snapshot);
        this._updateCommandArrows(snapshot);
        this._updatePrediction(dpcbf.predictedPath, dpcbf.enabled !== false);
        this._updateOverlays(
            dpcbf.overlays,
            dpcbf.enabled !== false
        );
    }

    dispose() {
        this.disposed = true;
        for (const asset of this.assets.values()) this._removeAsset(asset);
        this.assets.clear();
        if (this.previewAsset) this._removeAsset(this.previewAsset);
        this.previewAsset = null;
        for (const boundary of this.boundaries.values()) {
            this.scene.remove(boundary.mesh);
            disposeObject(boundary.mesh);
        }
        this.boundaries.clear();
        for (const fill of this.coneFills.values()) {
            this.scene.remove(fill);
            disposeObject(fill);
        }
        this.coneFills.clear();
        for (const arrow of this.relativeArrows.values()) {
            this.scene.remove(arrow.root);
            disposeObject(arrow.root);
        }
        this.relativeArrows.clear();
        for (const object of [
            this.nominalArrow.root,
            this.filteredArrow.root,
            this.predictionLine
        ]) {
            this.scene.remove(object);
            disposeObject(object);
        }
    }
}
