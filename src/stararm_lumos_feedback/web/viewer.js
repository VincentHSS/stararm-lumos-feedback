import * as THREE from "./vendor/three.module.min.js";

// Coordinates are URDF metres/radians, with Z up. Solid approximations are
// explicitly schematic; actual URDF visuals, when available, replace them.
const V = (a = [0, 0, 0]) => new THREE.Vector3(...a);
const finiteSix = q => Array.isArray(q) && q.length === 6 && q.every(Number.isFinite);
const metal = color => new THREE.MeshStandardMaterial({ color, metalness: .58, roughness: .34 });
const Y = V([0, 1, 0]);

function origin(object, value = {}) {
  object.position.copy(V(value.xyz));
  const [r = 0, p = 0, y = 0] = value.rpy ?? [];
  object.quaternion.setFromEuler(new THREE.Euler(r, p, y, "ZYX"));
}

function cylinderBetween(parent, start, end, radius, material) {
  const delta = end.clone().sub(start), length = delta.length();
  if (length < .002) return null;
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 24), material);
  mesh.position.copy(start).add(end).multiplyScalar(.5);
  mesh.quaternion.setFromUnitVectors(Y, delta.normalize());
  mesh.castShadow = mesh.receiveShadow = true;
  parent.add(mesh);
  return mesh;
}

function box(parent, size, at, material) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
  mesh.position.copy(V(at));
  mesh.castShadow = mesh.receiveShadow = true;
  parent.add(mesh);
  return mesh;
}

function stlGeometry(buffer) {
  const view = new DataView(buffer);
  const count = buffer.byteLength >= 84 ? view.getUint32(80, true) : 0;
  let vertices;
  if (count > 0 && 84 + count * 50 <= buffer.byteLength) {
    if (count > 600000) throw new Error("Mesh triangle limit exceeded");
    vertices = new Float32Array(count * 9);
    for (let i = 0; i < count; i++) {
      const start = 84 + i * 50 + 12;
      for (let j = 0; j < 9; j++) vertices[i * 9 + j] = view.getFloat32(start + j * 4, true);
    }
  } else {
    const text = new TextDecoder().decode(buffer), values = [];
    const pattern = /vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)/g;
    for (const match of text.matchAll(pattern)) {
      values.push(Number(match[1]), Number(match[2]), Number(match[3]));
      if (values.length > 600000 * 9) throw new Error("Mesh triangle limit exceeded");
    }
    if (!values.length || values.length % 9) throw new Error("Invalid STL mesh");
    vertices = new Float32Array(values);
  }
  if (!vertices.every(Number.isFinite)) throw new Error("Non-finite mesh coordinates");
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
  geometry.computeVertexNormals();
  geometry.computeBoundingBox();
  return geometry;
}

export class LumosViewer {
  constructor(canvas, model) {
    if (!canvas || !model || !Array.isArray(model.joints)) throw new Error("Viewer model/canvas missing");
    this.canvas = canvas;
    this.model = model;
    this.links = new Map();
    this.joints = new Map();
    this.labels = [];
    this.cpuLabels = [];
    this.cpuGeometryCache = new WeakMap();
    this.cpuObjectFaces = new WeakMap();
    this.cpuMaterialColors = new WeakMap();
    this.controlsBound = new WeakSet();
    this.meshLoaded = 0;
    this.meshFailed = 0;
    this.meshErrors = [];
    this.realMeshModel = model.geometry_source === "urdf_meshes";
    this.meshExpected = (model.links ?? []).reduce((total, link) => total + (link.visuals ?? []).filter(visual => visual.geometry?.type === "mesh").length, 0);
    this.interacted = false;
    this.azimuth = .95;
    this.elevation = .42;
    this.target = new THREE.Vector3(0, 0, .3);
    this.distance = 1.4;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a1823);
    this.camera = new THREE.PerspectiveCamera(38, 1, .005, 50);
    this.camera.up.set(0, 0, 1);
    this.robot = new THREE.Group();
    this.scene.add(this.robot);
    this.materials = {
      teal: metal(0x32c7b5), dark: metal(0x29444f), silver: metal(0xbdcbd1),
      black: metal(0x182b34), accent: metal(0x7be5d6), pad: metal(0x536872),
    };
    this.available = false;
    try {
      this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false, powerPreference: "low-power" });
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      this.renderer.outputColorSpace = THREE.SRGBColorSpace;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.renderer.toneMappingExposure = 1.35;
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      this.available = true;
    } catch (error) {
      this.error = error;
      this.renderer = null;
    }
    canvas.addEventListener("webglcontextlost", event => {
      event.preventDefault();
      this.useSoftwareRenderer("context_lost");
    });
    if (!this.renderer) this.useSoftwareRenderer("webgl_unavailable");
    this.buildEnvironment();
    this.buildRobot();
    this.update(finiteSix(model.preview_q) ? model.preview_q : [.25, 1.05, -1.65, .6, 0, 0], .6);
    this.fit();
    this.controls();
    this.resize();
    if (typeof ResizeObserver !== "undefined") {
      this.observer = new ResizeObserver(() => this.resize());
      this.observer.observe(this.canvas);
    }
    this.initialized = true;
    this.canvas.dataset.viewer = this.available ? "three" : "cpu";
    this.ready = Promise.all(this.loading).then(() => {
      this.meshReady = true;
      this.updateTargetPose();
      if (!this.interacted) this.fit();
      this.render();
      this.notifyStatus();
      return this;
    });
  }

  get geometrySource() {
    if (this.realMeshModel) {
      if (this.meshFailed) return this.meshLoaded ? "incomplete_meshes" : "mesh_error";
      if (!this.meshExpected) return "mesh_error";
      return this.meshLoaded === this.meshExpected ? "urdf_meshes" : "loading_meshes";
    }
    if (!this.meshLoaded) return "urdf_schematic";
    return this.meshFailed ? "mixed" : "urdf_meshes";
  }

  notifyStatus() {
    this.canvas.dispatchEvent(new CustomEvent("lumos-viewer-status", { detail: {
      webgl_available: this.available, geometry_source: this.geometrySource,
      renderer: this.available ? "webgl" : "cpu", fallback_reason: this.fallbackReason ?? null,
      mesh_loaded_count: this.meshLoaded, mesh_failed_count: this.meshFailed,
      mesh_expected_count: this.meshExpected, mesh_errors: this.meshErrors,
    } }));
  }

  buildEnvironment() {
    this.scene.add(new THREE.HemisphereLight(0xd9ffff, 0x324551, 2.8));
    const key = new THREE.DirectionalLight(0xffffff, 3.4);
    key.position.set(1.4, -1.3, 2.4);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    Object.assign(key.shadow.camera, { left: -1, right: 1, top: 1, bottom: -1, near: .1, far: 6 });
    key.shadow.bias = -.0003;
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x74dace, 1.5);
    rim.position.set(-1.5, 1.5, 1);
    this.scene.add(rim);
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(4, 4), new THREE.MeshStandardMaterial({
      color: 0x102732, roughness: .92, metalness: .1,
    }));
    floor.position.z = -.045;
    floor.receiveShadow = true;
    this.scene.add(floor);
    const grid = new THREE.GridHelper(2, 40, 0x42616a, 0x24434d);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -.044;
    grid.material.transparent = true;
    grid.material.opacity = .65;
    this.scene.add(grid);
    const axes = new THREE.AxesHelper(.1);
    axes.position.set(.11, -.11, -.042);
    this.scene.add(axes);
  }

  buildRobot() {
    this.loading = [];
    const sourceLinks = this.model.links ?? [];
    const names = new Set(sourceLinks.map(link => link.name));
    for (const joint of this.model.joints) { names.add(joint.parent); names.add(joint.child); }
    for (const name of names) this.links.set(name, new THREE.Group());
    const order = this.model.joint_order ?? this.model.joints.filter(j => j.type !== "fixed").slice(0, 6).map(j => j.name);
    this.order = order;
    for (const joint of this.model.joints) {
      const parent = this.links.get(joint.parent), child = this.links.get(joint.child);
      if (!parent || !child || parent === child) continue;
      const base = new THREE.Group(), motion = new THREE.Group();
      origin(base, joint.origin);
      base.add(motion);
      motion.add(child);
      parent.add(base);
      const axis = V(joint.axis ?? [0, 0, 1]);
      if (!axis.lengthSq()) axis.set(0, 0, 1);
      axis.normalize();
      this.joints.set(joint.name, { ...joint, base, motion, axis });
      const index = order.indexOf(joint.name);
      if (index >= 0) {
        if (!this.realMeshModel) {
          const motor = new THREE.Mesh(new THREE.CylinderGeometry(.043 - index * .003, .043 - index * .003, .046, 32), this.materials.dark);
          motor.quaternion.setFromUnitVectors(Y, axis);
          motor.castShadow = true;
          child.add(motor);
          const cap = new THREE.Mesh(new THREE.CylinderGeometry(.035 - index * .0025, .035 - index * .0025, .05, 32), this.materials.silver);
          cap.quaternion.copy(motor.quaternion);
          child.add(cap);
          const ring = new THREE.Mesh(new THREE.TorusGeometry(.036 - index * .0025, .0032, 8, 40), this.materials.teal);
          ring.quaternion.setFromUnitVectors(V([0, 0, 1]), axis);
          ring.position.copy(axis).multiplyScalar(.027);
          child.add(ring);
        }
        this.addLabel(child, `J${index + 1}`, [.025, -.035, .04]);
      }
    }
    const rootName = this.model.root ?? sourceLinks[0]?.name ?? [...names][0];
    const root = this.links.get(rootName);
    if (root) this.robot.add(root);
    for (const [name, group] of this.links) {
      const schematic = this.realMeshModel ? null : new THREE.Group();
      if (schematic) group.add(schematic);
      const children = this.model.joints.filter(j => j.parent === name);
      for (const child of schematic ? children : []) {
        const endpoint = V(child.origin?.xyz);
        const length = endpoint.length();
        if (length < .008) continue;
        const radius = Math.min(.035, Math.max(.018, length * .09));
        cylinderBetween(schematic, V(), endpoint, radius, this.materials.teal);
        cylinderBetween(schematic, endpoint.clone().multiplyScalar(.15), endpoint.clone().multiplyScalar(.85), radius * 1.06, this.materials.silver);
        cylinderBetween(schematic, endpoint.clone().multiplyScalar(.22), endpoint.clone().multiplyScalar(.78), radius * 1.08, this.materials.teal);
      }
      if (schematic && name === rootName) {
        cylinderBetween(schematic, V([0, 0, -.04]), V([0, 0, .01]), .085, this.materials.dark);
        cylinderBetween(schematic, V([0, 0, -.012]), V([0, 0, .012]), .07, this.materials.silver);
      }
      const link = sourceLinks.find(link => link.name === name);
      const visuals = link?.visuals ?? [];
      if (visuals.length) {
        const jobs = visuals.map(visual => this.addVisual(group, visual, name));
        const loaded = Promise.all(jobs).then(results => {
          if (schematic && results.every(Boolean)) schematic.visible = false;
        });
        this.loading.push(loaded);
      }
    }
    const final = this.joints.get(order[5]);
    this.toolParent = final ? this.links.get(final.child) : root;
    if (this.realMeshModel) {
      if (this.toolParent) this.addLabel(this.toolParent, this.model.gripper_label ?? "Gripper", [.085, 0, .055]);
    } else this.buildGripper();
  }

  async addVisual(parent, visual, name) {
    const geometry = visual.geometry ?? {}, holder = new THREE.Group();
    origin(holder, visual.origin);
    let material = /base/i.test(name) ? this.materials.dark : this.materials.silver;
    const rgba = visual.color;
    if (Array.isArray(rgba) && rgba.length >= 3 && rgba.every(Number.isFinite)) {
      const channels = rgba.map(value => Math.max(0, Math.min(1, value)));
      material = metal(new THREE.Color().setRGB(channels[0], channels[1], channels[2], THREE.SRGBColorSpace));
      material.opacity = channels[3] ?? 1;
      material.transparent = material.opacity < 1;
      material.depthWrite = !material.transparent;
    }
    try {
      let shape;
      if (geometry.type === "mesh") {
        const url = new URL(geometry.url, window.location.href);
        if (url.origin !== window.location.origin) throw new Error("Mesh URL must be local");
        const response = await fetch(url);
        if (!response.ok) throw new Error("Mesh unavailable");
        shape = stlGeometry(await response.arrayBuffer());
        holder.scale.copy(V(geometry.scale ?? [1, 1, 1]));
        if (![holder.scale.x, holder.scale.y, holder.scale.z].every(value => Number.isFinite(value) && value > 0)) throw new Error("Invalid mesh scale");
        if (shape.boundingBox.getSize(V()).multiply(holder.scale).length() > 5) throw new Error("Mesh metre scale inconsistent");
        shape.userData.importedMesh = true;
        this.meshLoaded++;
      } else if (geometry.type === "box") {
        shape = new THREE.BoxGeometry(...geometry.size);
      } else if (geometry.type === "sphere") {
        shape = new THREE.SphereGeometry(geometry.radius, 24, 16);
      } else if (geometry.type === "cylinder") {
        shape = new THREE.CylinderGeometry(geometry.radius, geometry.radius, geometry.length, 32);
        shape.rotateX(Math.PI / 2); // URDF cylinders have their axis along local Z.
        shape.userData.cpuPreserveGeometry = true;
      } else return false;
      const mesh = new THREE.Mesh(shape, material);
      mesh.castShadow = mesh.receiveShadow = true;
      holder.add(mesh);
      parent.add(holder);
      this.render();
      this.notifyStatus();
      return true;
    } catch (error) {
      this.meshFailed++;
      this.meshErrors.push({ link: name, reason: error.message || "Mesh unavailable" });
      this.notifyStatus();
      return false;
    }
  }

  buildGripper() {
    if (!this.toolParent) return;
    const group = new THREE.Group();
    this.toolParent.add(group);
    this.tool = group;
    const tip = Number.isFinite(this.model.tip_offset_m) ? this.model.tip_offset_m : .17334;
    // TypeLJ's TCP is along link6 +X; finger opening is along local Y.
    cylinderBetween(group, V(), V([.042, 0, 0]), .028, this.materials.silver);
    box(group, [.074, .059, .045], [.069, 0, 0], this.materials.teal);
    box(group, [.017, .105, .03], [.108, 0, 0], this.materials.dark);
    this.fingers = [-1, 1].map(sign => {
      const finger = new THREE.Group();
      const length = Math.max(.025, tip - .112);
      box(finger, [length, .01, .019], [.112 + length / 2, 0, 0], this.materials.silver);
      box(finger, [.014, .011, .021], [tip - .008, 0, 0], this.materials.pad);
      box(finger, [.023, .023, .026], [.113, 0, 0], this.materials.black);
      group.add(finger);
      return { sign, group: finger };
    });
    const tcp = new THREE.Mesh(new THREE.SphereGeometry(.004, 12, 8), new THREE.MeshBasicMaterial({ color: 0xf7cc76 }));
    tcp.position.x = tip;
    group.add(tcp);
    this.addLabel(group, "TypeLJ", [.085, 0, .055]);
  }

  addLabel(parent, text, position) {
    this.cpuLabels.push({ parent, text, position: V(position) });
    if (!this.available) return;
    const canvas = document.createElement("canvas");
    canvas.width = 160; canvas.height = 64;
    const context = canvas.getContext("2d");
    if (!context) return;
    context.fillStyle = "rgba(8,24,32,.86)";
    context.beginPath();
    context.roundRect(2, 5, 156, 54, 15);
    context.fill();
    context.strokeStyle = "rgba(112,227,211,.65)";
    context.lineWidth = 2;
    context.stroke();
    context.fillStyle = "#d9fff8";
    context.font = "600 32px system-ui, sans-serif";
    context.textAlign = "center"; context.textBaseline = "middle";
    context.fillText(text, 80, 33);
    const texture = new THREE.CanvasTexture(canvas);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true }));
    sprite.position.copy(V(position));
    sprite.scale.set(.09, .036, 1);
    sprite.renderOrder = 10;
    parent.add(sprite);
    this.labels.push(sprite);
  }

  // Target coordinates are already URDF radians, just like q. Reuse the exact
  // loaded visual geometry with an independent transform tree and no labels.
  buildTargetRobot() {
    if (this.targetRobot || !this.meshReady) return;
    const material = new THREE.MeshBasicMaterial({
      color: 0x38d8ee, opacity: .32, transparent: true, depthWrite: false,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1,
    });
    const clones = new Map();
    const copy = source => {
      if (source.isSprite) return null;
      const object = source.clone(false);
      clones.set(source, object);
      if (object.isMesh) {
        object.material = material;
        object.castShadow = object.receiveShadow = false;
        object.renderOrder = 1;
      }
      for (const child of source.children) {
        const cloned = copy(child);
        if (cloned) object.add(cloned);
      }
      return object;
    };
    this.targetRobot = copy(this.robot);
    this.targetRobot.name = "calibrated-target";
    this.targetRobot.visible = false;
    this.targetJoints = new Map([...this.joints].map(([name, joint]) => [
      name, { ...joint, motion: clones.get(joint.motion) },
    ]));
    this.targetFingers = this.fingers?.map(finger => ({ ...finger, group: clones.get(finger.group) }));
    this.scene.add(this.targetRobot);
  }

  applyPose(joints, fingers, q, fraction) {
    this.order.forEach((name, index) => {
      const joint = joints.get(name);
      if (!joint) return;
      if (joint.type === "prismatic") joint.motion.position.copy(joint.axis).multiplyScalar(q[index]);
      else joint.motion.quaternion.setFromAxisAngle(joint.axis, q[index]);
    });
    const width = Number.isFinite(this.model.gripper_width_m) ? this.model.gripper_width_m : .08;
    fingers?.forEach(finger => { finger.group.position.y = finger.sign * (fraction * width / 2 + .005); });
    for (const mapping of this.model.gripper_joints ?? []) {
      const joint = joints.get(mapping.name);
      if (!joint || !Number.isFinite(mapping.closed) || !Number.isFinite(mapping.open)) continue;
      const position = mapping.closed + (mapping.open - mapping.closed) * fraction;
      if (joint.type === "prismatic") joint.motion.position.copy(joint.axis).multiplyScalar(position);
      else joint.motion.quaternion.setFromAxisAngle(joint.axis, position);
    }
  }

  updateTargetPose() {
    if (this.lastTargetQ) this.buildTargetRobot();
    if (!this.targetRobot) return;
    this.targetRobot.visible = Boolean(this.lastTargetQ);
    if (this.lastTargetQ) this.applyPose(this.targetJoints, this.targetFingers, this.lastTargetQ, this.lastOpening);
    this.targetRobot.updateMatrixWorld(true);
  }

  update(q, opening, targetUrdfRad6 = null) {
    if (!finiteSix(q)) q = finiteSix(this.model.preview_q) ? this.model.preview_q : [0, 0, 0, 0, 0, 0];
    const fraction = Number.isFinite(opening) ? Math.max(0, Math.min(1, opening)) : .6;
    const target = finiteSix(targetUrdfRad6) ? targetUrdfRad6 : null;
    const targetUnchanged = target === null ? !this.lastTargetQ : this.lastTargetQ && target.every((value, i) => value === this.lastTargetQ[i]);
    if (this.lastQ && q.every((value, i) => value === this.lastQ[i]) && fraction === this.lastOpening && targetUnchanged) return;
    this.lastQ = [...q];
    this.lastTargetQ = target ? [...target] : null;
    this.lastOpening = fraction;
    this.applyPose(this.joints, this.fingers, q, fraction);
    this.updateTargetPose();
    this.robot.updateMatrixWorld(true);
    this.render();
  }

  fit() {
    this.robot.updateMatrixWorld(true);
    const bounds = new THREE.Box3().setFromObject(this.robot);
    if (this.targetRobot?.visible) bounds.union(new THREE.Box3().setFromObject(this.targetRobot));
    if (bounds.isEmpty()) { this.target.set(0, 0, .25); this.fitDistance = 1.3; }
    else {
      bounds.getCenter(this.target);
      const span = Math.max(bounds.getSize(V()).length(), .45);
      this.fitDistance = Math.max(.8, span / (2 * Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2))) * 1.25);
    }
    this.distance = this.fitDistance;
    this.cameraPosition();
  }

  cameraPosition() {
    const horizontal = this.distance * Math.cos(this.elevation);
    this.camera.position.set(
      this.target.x + horizontal * Math.cos(this.azimuth),
      this.target.y + horizontal * Math.sin(this.azimuth),
      this.target.z + this.distance * Math.sin(this.elevation),
    );
    this.camera.lookAt(this.target);
    this.camera.updateMatrixWorld();
  }

  reset() {
    this.interacted = false;
    this.azimuth = .95; this.elevation = .42;
    this.fit(); this.render();
  }

  controls() {
    if (this.controlsBound.has(this.canvas)) return;
    this.controlsBound.add(this.canvas);
    this.canvas.style.touchAction = "none";
    let drag = null;
    this.canvas.addEventListener("pointerdown", event => {
      drag = { x: event.clientX, y: event.clientY, id: event.pointerId };
      this.canvas.setPointerCapture?.(event.pointerId);
    });
    this.canvas.addEventListener("pointermove", event => {
      if (!drag || drag.id !== event.pointerId) return;
      this.interacted = true;
      this.azimuth -= (event.clientX - drag.x) * .008;
      this.elevation = Math.max(.06, Math.min(1.42, this.elevation + (event.clientY - drag.y) * .006));
      drag.x = event.clientX; drag.y = event.clientY;
      this.cameraPosition(); this.render();
    });
    const release = () => { drag = null; };
    this.canvas.addEventListener("pointerup", release);
    this.canvas.addEventListener("pointercancel", release);
    this.canvas.addEventListener("wheel", event => {
      event.preventDefault(); this.interacted = true;
      this.distance = Math.max(.25, Math.min((this.fitDistance || 1.4) * 4, this.distance * Math.exp(event.deltaY * .001)));
      this.cameraPosition(); this.render();
    }, { passive: false });
    this.canvas.addEventListener("dblclick", () => this.reset());
  }

  resize() {
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 900);
    const height = Math.max(1, this.canvas.clientHeight || Math.round(width * .6));
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    if (this.renderer) this.renderer.setSize(width, height, false);
    else {
      const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
      const pixelsX = Math.round(width * ratio), pixelsY = Math.round(height * ratio);
      if (this.canvas.width !== pixelsX) this.canvas.width = pixelsX;
      if (this.canvas.height !== pixelsY) this.canvas.height = pixelsY;
    }
    this.render();
  }

  render() {
    if (!this.camera || !this.robot || this.framePending) return;
    this.framePending = true;
    const draw = () => {
      this.framePending = false;
      this.cameraPosition();
      if (this.renderer) {
        try { this.renderer.render(this.scene, this.camera); }
        catch (error) {
          this.error = error;
          this.useSoftwareRenderer("webgl_render_failed");
          this.renderFallback();
        }
      } else this.renderFallback();
    };
    if (typeof requestAnimationFrame === "function") requestAnimationFrame(draw);
    else draw();
  }

  useSoftwareRenderer(reason = "requested") {
    if (this.cpuContext) return;
    const previousRenderer = this.renderer;
    this.renderer = null;
    this.available = false;
    this.fallbackReason = reason;
    try { previousRenderer?.dispose(); } catch (_) { /* The GPU context can already be gone. */ }
    let context = null;
    try { context = this.canvas.getContext("2d", { alpha: false }); } catch (_) { /* WebGL owns it. */ }
    if (!context) {
      // A canvas that ever acquired WebGL cannot acquire a 2D context, even
      // after context loss. Preserve its public ID/layout while replacing only
      // the backing surface; the app looks up the canvas by ID.
      const previous = this.canvas, replacement = previous.cloneNode(false);
      previous.replaceWith(replacement);
      this.canvas = replacement;
      context = replacement.getContext("2d", { alpha: false });
      this.observer?.disconnect();
      this.observer?.observe(replacement);
    }
    this.cpuContext = context;
    this.canvas.dataset.viewer = "cpu";
    if (this.initialized) {
      this.controls();
      this.resize();
      this.notifyStatus();
    }
  }

  cpuGeometry(original) {
    if (this.cpuGeometryCache.has(original)) return this.cpuGeometryCache.get(original);
    let geometry = original;
    const p = original.parameters ?? {};
    // Complete low-poly surfaces, rather than dropping individual faces and
    // leaving holes. World transforms and metre dimensions remain identical.
    if (original.type === "CylinderGeometry") {
      geometry = new THREE.CylinderGeometry(p.radiusTop, p.radiusBottom, p.height, 12, 1, p.openEnded, p.thetaStart, p.thetaLength);
      // A URDF cylinder visual may already have a baked axis rotation. Use the
      // original geometry in that case; schematic cylinders are unmodified.
      if (original.userData?.cpuPreserveGeometry) geometry = original;
    } else if (original.type === "TorusGeometry") {
      geometry = new THREE.TorusGeometry(p.radius, p.tube, 5, 18, p.arc);
    } else if (original.type === "SphereGeometry") {
      geometry = new THREE.SphereGeometry(p.radius, 12, 8, p.phiStart, p.phiLength, p.thetaStart, p.thetaLength);
    }
    // Imported STL surfaces remain exact in CPU mode, regardless of triangle
    // count. A bounding-box proxy would erase the actual robot's contour.
    const positions = geometry.attributes.position.array;
    const indices = geometry.index?.array ?? Uint32Array.from({ length: positions.length / 3 }, (_, i) => i);
    const normals = new Float32Array(indices.length);
    for (let i = 0; i < indices.length; i += 3) {
      const a = indices[i] * 3, b = indices[i + 1] * 3, c = indices[i + 2] * 3;
      const abx = positions[b] - positions[a], aby = positions[b + 1] - positions[a + 1], abz = positions[b + 2] - positions[a + 2];
      const acx = positions[c] - positions[a], acy = positions[c + 1] - positions[a + 1], acz = positions[c + 2] - positions[a + 2];
      const nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
      const length = Math.hypot(nx, ny, nz) || 1;
      normals[i] = nx / length; normals[i + 1] = ny / length; normals[i + 2] = nz / length;
    }
    const cached = { positions, indices, normals, projected: new Float64Array(positions.length), geometry };
    this.cpuGeometryCache.set(original, cached);
    return cached;
  }

  cpuTriangles(width, height) {
    const result = [], viewProjection = new THREE.Matrix4().multiplyMatrices(this.camera.projectionMatrix, this.camera.matrixWorldInverse);
    this.robot.updateMatrixWorld(true);
    const light = V([1.4, -1.3, 2.4]).normalize();
    const collect = object => {
      if (!object.isMesh || !object.geometry?.attributes?.position) return;
      const cached = this.cpuGeometry(object.geometry);
      const { positions, projected, indices, normals } = cached;
      // Current and target meshes share geometry, but each projected face must
      // retain its own coordinates/material until the painter pass completes.
      if (!this.cpuObjectFaces.has(object)) this.cpuObjectFaces.set(object, Array.from({ length: indices.length / 3 }, () => ({})));
      const faces = this.cpuObjectFaces.get(object);
      const transform = new THREE.Matrix4().multiplyMatrices(viewProjection, object.matrixWorld).elements;
      const normalMatrix = new THREE.Matrix3().getNormalMatrix(object.matrixWorld).elements;
      for (let i = 0; i < positions.length; i += 3) {
        const x = positions[i], y = positions[i + 1], z = positions[i + 2];
        const w = transform[3] * x + transform[7] * y + transform[11] * z + transform[15];
        if (w <= .005) { projected[i + 2] = Infinity; continue; }
        projected[i] = ((transform[0] * x + transform[4] * y + transform[8] * z + transform[12]) / w + 1) * width / 2;
        projected[i + 1] = (1 - (transform[1] * x + transform[5] * y + transform[9] * z + transform[13]) / w) * height / 2;
        projected[i + 2] = (transform[2] * x + transform[6] * y + transform[10] * z + transform[14]) / w;
      }
      const material = Array.isArray(object.material) ? object.material[0] : object.material;
      if (!this.cpuMaterialColors.has(material)) {
        const color = (material.color ?? new THREE.Color(0xb8cad2)).clone().convertLinearToSRGB();
        const rgb = [color.r * 255, color.g * 255, color.b * 255];
        const palette = Array.from({ length: 97 }, (_, level) => `rgb(${rgb.map(channel => Math.min(255, Math.round(channel * level / 96))).join(",")})`);
        this.cpuMaterialColors.set(material, palette);
      }
      const palette = this.cpuMaterialColors.get(material);
      for (let i = 0; i < indices.length; i += 3) {
        const a = indices[i] * 3, b = indices[i + 1] * 3, c = indices[i + 2] * 3;
        const za = projected[a + 2], zb = projected[b + 2], zc = projected[c + 2];
        if (!(za >= -1 && za <= 1 && zb >= -1 && zb <= 1 && zc >= -1 && zc <= 1)) continue;
        const ax = projected[a], ay = projected[a + 1], bx = projected[b], by = projected[b + 1], cx = projected[c], cy = projected[c + 1];
        const area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (area >= -.04) continue; // Perspective front faces are clockwise in canvas Y-down coordinates.
        const nx = normals[i], ny = normals[i + 1], nz = normals[i + 2];
        let x = normalMatrix[0] * nx + normalMatrix[3] * ny + normalMatrix[6] * nz;
        let y = normalMatrix[1] * nx + normalMatrix[4] * ny + normalMatrix[7] * nz;
        let z = normalMatrix[2] * nx + normalMatrix[5] * ny + normalMatrix[8] * nz;
        const length = Math.hypot(x, y, z) || 1;
        x /= length; y /= length; z /= length;
        const shade = .45 + .45 * Math.max(0, x * light.x + y * light.y + z * light.z) + .12 * Math.max(0, z);
        const face = faces[i / 3];
        face.depth = (za + zb + zc) / 3;
        face.ax = ax; face.ay = ay; face.bx = bx; face.by = by; face.cx = cx; face.cy = cy;
        face.color = palette[Math.max(0, Math.min(96, Math.round(shade * 96)))];
        face.alpha = material.opacity ?? 1;
        result.push(face);
      }
    };
    this.robot.traverseVisible(collect);
    if (this.targetRobot?.visible) {
      this.targetRobot.updateMatrixWorld(true);
      this.targetRobot.traverseVisible(collect);
    }
    result.sort((a, b) => b.depth - a.depth);
    return result;
  }

  renderFallback() {
    const context = this.cpuContext;
    if (!context || typeof context.clearRect !== "function") return;
    this.cameraPosition();
    const width = this.canvas.width, height = this.canvas.height;
    const triangles = this.cpuTriangles(width, height);
    const background = context.createLinearGradient(0, 0, 0, height);
    background.addColorStop(0, "#122936"); background.addColorStop(1, "#07141e");
    context.fillStyle = background; context.fillRect(0, 0, width, height);
    const project = point => {
      const value = point.clone().project(this.camera);
      return value.z >= -1 && value.z <= 1 ? [(value.x + 1) * width / 2, (1 - value.y) * height / 2] : null;
    };
    const line = (start, end) => {
      const a = project(start), b = project(end);
      if (!a || !b) return;
      context.beginPath(); context.moveTo(...a); context.lineTo(...b); context.stroke();
    };
    context.lineWidth = 1;
    for (let i = -20; i <= 20; i++) {
      context.strokeStyle = i % 5 === 0 ? "#2c4d59" : "#1b3844";
      line(V([i * .05, -1, -.043]), V([i * .05, 1, -.043]));
      line(V([-1, i * .05, -.043]), V([1, i * .05, -.043]));
    }
    context.lineJoin = "round";
    context.lineWidth = .65;
    for (const face of triangles) {
      context.globalAlpha = face.alpha;
      context.fillStyle = context.strokeStyle = face.color;
      context.beginPath(); context.moveTo(face.ax, face.ay); context.lineTo(face.bx, face.by); context.lineTo(face.cx, face.cy); context.closePath();
      context.fill(); context.stroke();
    }
    context.globalAlpha = 1;
    const font = Math.max(14, Math.min(23, width * .018));
    context.font = `600 ${font}px system-ui, sans-serif`;
    context.textAlign = "center"; context.textBaseline = "middle";
    const placedLabels = [];
    for (const label of this.cpuLabels) {
      const position = project(label.parent.localToWorld(label.position.clone()));
      if (!position) continue;
      const [anchorX, anchorY] = position, textWidth = context.measureText(label.text).width;
      const halfWidth = textWidth / 2 + 10, halfHeight = font * .7;
      let x = anchorX, y = anchorY, chosen = false;
      for (const row of [0, -1, 1, -2, 2, -3, 3]) {
        for (const column of [0, 1, -1, 2, -2]) {
          const candidateX = anchorX + column * (halfWidth * 2 + 12);
          const candidateY = anchorY + row * (halfHeight * 2 + 7);
          if (candidateX - halfWidth < 6 || candidateX + halfWidth > width - 6 ||
              candidateY - halfHeight < height * .14 || candidateY + halfHeight > height - 20) continue;
          if (placedLabels.some(box => Math.abs(box.x - candidateX) < box.halfWidth + halfWidth + 5 &&
              Math.abs(box.y - candidateY) < box.halfHeight + halfHeight + 5)) continue;
          x = candidateX; y = candidateY; chosen = true; break;
        }
        if (chosen) break;
      }
      placedLabels.push({x, y, halfWidth, halfHeight});
      if (Math.hypot(x - anchorX, y - anchorY) > 3) {
        context.strokeStyle = "rgba(115,224,207,.45)"; context.lineWidth = 1;
        context.beginPath(); context.moveTo(anchorX, anchorY); context.lineTo(x, y); context.stroke();
      }
      context.fillStyle = "rgba(7,23,31,.9)";
      context.strokeStyle = "rgba(115,224,207,.65)";
      context.lineWidth = 1;
      context.beginPath();
      if (context.roundRect) context.roundRect(x - textWidth / 2 - 10, y - font * .7, textWidth + 20, font * 1.4, 7);
      else context.rect(x - textWidth / 2 - 10, y - font * .7, textWidth + 20, font * 1.4);
      context.fill(); context.stroke(); context.fillStyle = "#d8fff5"; context.fillText(label.text, x, y);
    }
    this.cpuTriangleCount = triangles.length;
  }
}

window.LumosViewer = LumosViewer;
window.dispatchEvent(new Event("lumos-viewer-ready"));
