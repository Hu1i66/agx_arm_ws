<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from "vue";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { Box, Crosshair, Layers3 } from "lucide-vue-next";
import { useWorkbench } from "../stores/workbench";

const store = useWorkbench();
const host = ref<HTMLElement | null>(null);
const modelState = ref<"loading" | "ready" | "fallback" | "error">("loading");
let renderer: THREE.WebGLRenderer | null = null;
let camera: THREE.PerspectiveCamera | null = null;
let controls: OrbitControls | null = null;
let scene: THREE.Scene | null = null;
let modelRoot: THREE.Group | null = null;
let fallbackRoot: THREE.Group | null = null;
let detectionRoot: THREE.Group | null = null;
let trail: THREE.Line | null = null;
let observer: ResizeObserver | null = null;
let frameId = 0;
let hidden = false;
let detectionSignature = "";
let lastDetectionSync = 0;
const links = new Map<string, THREE.Object3D>();
const desired = new Map<string, { position: THREE.Vector3; quaternion: THREE.Quaternion }>();
const detectionMarkers = new Map<string, {
  group: THREE.Group;
  desired: THREE.Vector3;
  selected: boolean;
}>();
const trailPoints: THREE.Vector3[] = [];
const linkNames = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper_base", "gripper_link1", "gripper_link2", "tcp_link"];

const telemetry = computed(() => store.snapshot.robot || {});
const joints = computed<number[]>(() => telemetry.value.joints_rad || []);
const tcp = computed<number[]>(() => telemetry.value.tcp_xyzw || []);
const target = computed(() => store.currentTarget?.base_position_m || null);
const visibleDetections = computed(() => {
  let count = 0;
  for (const camera of ["d455", "dabai"] as const) {
    const data = store.detections[camera];
    if (!data || Number(store.clock) - Number(data.header_stamp || 0) > 1.5) continue;
    count += (data.objects || []).filter((item) => !!item.base_position_m).length;
  }
  return count;
});

function poseFrom(value: number[]) {
  const baseRotation = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
  return {
    position: new THREE.Vector3(value[0], value[2], -value[1]),
    quaternion: baseRotation.multiply(new THREE.Quaternion(value[3], value[4], value[5], value[6]).normalize()),
  };
}

function buildFallback() {
  fallbackRoot = new THREE.Group();
  fallbackRoot.name = "pose-fallback";
  const material = new THREE.MeshStandardMaterial({ color: 0x1a2738, roughness: 0.58, metalness: 0.18 });
  const jointMaterial = new THREE.MeshStandardMaterial({ color: 0x1769ff, roughness: 0.42 });
  const sphere = new THREE.SphereGeometry(0.022, 16, 12);
  for (const name of linkNames) {
    if (name === "base_link") continue;
    const node = new THREE.Group();
    const pivot = new THREE.Mesh(sphere, jointMaterial);
    node.add(pivot);
    node.name = name;
    fallbackRoot.add(node);
    links.set(name, node);
  }
  const lineGeometry = new THREE.BufferGeometry();
  lineGeometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(8 * 3), 3));
  lineGeometry.setDrawRange(0, 0);
  const line = new THREE.Line(lineGeometry, new THREE.LineBasicMaterial({ color: 0x2c3c53, linewidth: 3 }));
  line.name = "skeleton-line";
  fallbackRoot.add(line);
  const base = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.18, 0.06), material);
  base.position.set(0, 0.03, 0);
  base.name = "base_link";
  fallbackRoot.add(base);
  links.set("base_link", base);
  modelRoot?.add(fallbackRoot);
  modelState.value = "fallback";
}

async function loadModel() {
  try {
    const response = await fetch("/models/model_manifest.json", { cache: "force-cache" });
    if (!response.ok) throw new Error("model manifest missing");
    const manifest = await response.json();
    const loader = new GLTFLoader();
    const entries = Object.entries(manifest.links || {}) as [string, any][];
    if (!entries.length) throw new Error("model manifest has no links");
    await Promise.all(entries.map(async ([name, meta]) => {
      const group = new THREE.Group();
      group.name = name;
      try {
        const gltf = await loader.loadAsync(meta.url);
        const object = gltf.scene;
        // Assimp writes one Z-up conversion on the scene and one on each mesh.
        // Leave one net -90° conversion: that is the axis convention used by the
        // URDF joint chain after the server-side TF is mapped into Three.js.
        object.rotation.x += Math.PI / 2;
        object.traverse((child) => {
          if (child instanceof THREE.Mesh) {
            child.castShadow = false;
            child.receiveShadow = false;
            const source = child.material as THREE.Material | THREE.Material[];
            const materials = Array.isArray(source) ? source : [source];
            for (const material of materials) {
              if (material instanceof THREE.MeshStandardMaterial) {
                material.color.multiplyScalar(0.82);
                material.roughness = Math.max(0.45, material.roughness);
                material.metalness = Math.min(0.35, material.metalness);
              }
            }
          }
        });
        if (meta.scale && Number(meta.scale) !== 1) object.scale.setScalar(Number(meta.scale));
        group.add(object);
      } catch {
        const placeholder = new THREE.Mesh(
          new THREE.SphereGeometry(0.018, 12, 8),
          new THREE.MeshStandardMaterial({ color: 0x697a90 }),
        );
        group.add(placeholder);
      }
      modelRoot?.add(group);
      links.set(name, group);
    }));
    modelState.value = "ready";
  } catch {
    buildFallback();
  }
}

function labelTexture(text: string, selected: boolean, camera: "d455" | "dabai") {
  const canvas = document.createElement("canvas");
  canvas.width = 512;
  canvas.height = 96;
  const context = canvas.getContext("2d");
  if (context) {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.fillStyle = selected ? "rgba(23,105,255,.94)" : camera === "d455" ? "rgba(16,55,105,.86)" : "rgba(6,82,74,.86)";
    context.beginPath();
    context.roundRect(4, 8, 504, 78, 14);
    context.fill();
    context.fillStyle = "#fff";
    context.font = "600 34px 'PingFang SC','Microsoft YaHei',sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(text, 256, 48, 480);
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function createDetectionMarker(camera: "d455" | "dabai", object: any, selected: boolean) {
  const group = new THREE.Group();
  const diameter = Math.max(0.02, Math.min(Number(object.estimated_diameter_m || 0.05), 0.15)) * 0.65;
  const radius = Math.max(0.012, diameter / 2);
  const color = camera === "d455" ? 0x147dff : 0x13bfa0;
  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 22, 16),
    new THREE.MeshStandardMaterial({
      color: selected ? 0x1769ff : color,
      roughness: 0.32,
      metalness: 0.08,
      transparent: true,
      opacity: selected ? 0.9 : 0.72,
    }),
  );
  const halo = new THREE.Mesh(
    new THREE.RingGeometry(radius * (selected ? 1.65 : 1.3), radius * (selected ? 1.9 : 1.48), 36),
    new THREE.MeshBasicMaterial({
      color: selected ? 0x1769ff : color,
      transparent: true,
      opacity: selected ? 0.55 : 0.28,
      side: THREE.DoubleSide,
      depthWrite: false,
    }),
  );
  halo.rotation.x = -Math.PI / 2;
  halo.position.y = -radius * 0.8;
  if (selected) halo.name = "selected-halo";
  const label = new THREE.Sprite(new THREE.SpriteMaterial({
    map: labelTexture(`T${object.track_id ?? "?"}  ${object.object_name || object.raw_object_name || "object"}`, selected, camera),
    transparent: true,
    depthTest: false,
    depthWrite: false,
  }));
  label.scale.set(0.22, 0.041, 1);
  label.position.y = radius + 0.032;
  label.renderOrder = 10;
  group.add(sphere, halo, label);
  group.userData.camera = camera;
  group.userData.trackId = object.track_id;
  return group;
}

function syncDetectionMarkers() {
  if (!detectionRoot) return;
  const rows: { key: string; camera: "d455" | "dabai"; object: any; selected: boolean }[] = [];
  for (const camera of ["d455", "dabai"] as const) {
    const data = store.detections[camera];
    if (!data || Number(store.clock) - Number(data.header_stamp || 0) > 1.5) continue;
    for (const object of data.objects || []) {
      if (!object.base_position_m) continue;
      const selected = camera === store.selectedCamera && Number(object.track_id) === Number(store.selectedTrack);
      const key = `${camera}:${object.track_id ?? `${object.object_name}:${object.base_position_m.x}:${object.base_position_m.y}`}`;
      rows.push({ key, camera, object, selected });
    }
  }
  const signature = rows.map(({ key, object, selected }) =>
    `${key}:${object.object_name || object.raw_object_name || ""}:${selected ? 1 : 0}`).join("|");
  if (signature !== detectionSignature) {
    detectionSignature = signature;
    for (const marker of detectionMarkers.values()) {
      marker.group.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          child.geometry?.dispose();
          const materials = Array.isArray(child.material) ? child.material : [child.material];
          materials.forEach((material) => material?.dispose());
        } else if (child instanceof THREE.Sprite) {
          child.material.map?.dispose();
          child.material.dispose();
        }
      });
    }
    detectionMarkers.clear();
    detectionRoot.clear();
    for (const { key, camera, object, selected } of rows) {
      const group = createDetectionMarker(camera, object, selected);
      const desired = new THREE.Vector3(object.base_position_m.x, object.base_position_m.z + 0.02, -object.base_position_m.y);
      group.position.copy(desired);
      detectionRoot.add(group);
      detectionMarkers.set(key, { group, desired, selected });
    }
  }
  for (const { key, object } of rows) {
    const marker = detectionMarkers.get(key);
    if (marker) marker.desired.set(object.base_position_m.x, object.base_position_m.z + 0.02, -object.base_position_m.y);
  }
}

function updateScene() {
  const data = telemetry.value.links || {};
  for (const [name, value] of Object.entries(data)) {
    if (Array.isArray(value) && value.length >= 7) desired.set(name, poseFrom(value as number[]));
  }
  const tcpPose = data.tcp_link || data.link6;
  if (Array.isArray(tcpPose) && tcpPose.length >= 7) {
    trailPoints.push(new THREE.Vector3(tcpPose[0], tcpPose[2], -tcpPose[1]));
    if (trailPoints.length > 80) trailPoints.shift();
    if (trail) {
      const attribute = trail.geometry.getAttribute("position") as THREE.BufferAttribute;
      trailPoints.forEach((position, index) => attribute.setXYZ(index, position.x, position.y, position.z));
      attribute.needsUpdate = true;
      trail.geometry.setDrawRange(0, trailPoints.length);
    }
  }
  if (performance.now() - lastDetectionSync > 100) {
    lastDetectionSync = performance.now();
    syncDetectionMarkers();
  }
  for (const marker of detectionMarkers.values()) {
    marker.group.position.lerp(marker.desired, 0.32);
    if (marker.selected) {
      const halo = marker.group.getObjectByName("selected-halo");
      if (halo) halo.scale.setScalar(1 + Math.sin(performance.now() / 300) * 0.12);
    }
  }
}

function animate() {
  frameId = requestAnimationFrame(animate);
  if (hidden) return;
  updateScene();
  for (const [name, object] of links) {
    const goal = desired.get(name);
    if (!goal) continue;
    object.position.lerp(goal.position, 0.34);
    object.quaternion.slerp(goal.quaternion, 0.34);
  }
  if (fallbackRoot) {
    const positions: THREE.Vector3[] = [];
    for (const name of ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "tcp_link"]) {
      const object = links.get(name);
      if (object) positions.push(object.position.clone());
    }
    const line = fallbackRoot.getObjectByName("skeleton-line") as THREE.Line | undefined;
    if (line && positions.length > 1) {
      const attribute = line.geometry.getAttribute("position") as THREE.BufferAttribute;
      positions.slice(0, 8).forEach((position, index) => attribute.setXYZ(index, position.x, position.y, position.z));
      attribute.needsUpdate = true;
      line.geometry.setDrawRange(0, Math.min(8, positions.length));
    }
  }
  if (trail && trail.material instanceof THREE.LineBasicMaterial) trail.material.opacity = 0.22;
  controls?.update();
  if (renderer && camera) renderer.render(scene!, camera);
}

function resize() {
  if (!host.value || !renderer || !camera) return;
  const width = Math.max(1, host.value.clientWidth);
  const height = Math.max(1, host.value.clientHeight);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

onMounted(() => {
  if (!host.value) return;
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf4f6f9);
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "high-performance" });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  host.value.appendChild(renderer.domElement);
  camera = new THREE.PerspectiveCamera(42, 1, 0.01, 30);
  camera.position.set(1.35, 1.08, 1.12);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.32, 0.08);
  controls.enableDamping = true;
  controls.dampingFactor = 0.075;
  controls.minDistance = 0.55;
  controls.maxDistance = 4;
  controls.maxPolarAngle = Math.PI * 0.52;
  const hemi = new THREE.HemisphereLight(0xffffff, 0xcbd5e3, 2.1);
  const key = new THREE.DirectionalLight(0xffffff, 2.4);
  key.position.set(2.2, 3.0, 2.4);
  const fill = new THREE.DirectionalLight(0x8fb8ff, 1.1);
  fill.position.set(-2, 1.2, -1.5);
  scene.add(hemi, key, fill);
  const grid = new THREE.GridHelper(4, 32, 0xc1c9d4, 0xdce2ea);
  grid.position.y = -0.005;
  scene.add(grid);
  modelRoot = new THREE.Group();
  scene.add(modelRoot);
  detectionRoot = new THREE.Group();
  detectionRoot.name = "detected-objects";
  scene.add(detectionRoot);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(80 * 3), 3));
  geometry.setDrawRange(0, 0);
  trail = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x1769ff, transparent: true, opacity: 0.22 }));
  modelRoot.add(trail);
  loadModel();
  observer = new ResizeObserver(resize);
  observer.observe(host.value);
  document.addEventListener("visibilitychange", visibility);
  resize();
  animate();
});

function visibility() {
  hidden = document.visibilityState === "hidden";
}

onBeforeUnmount(() => {
  cancelAnimationFrame(frameId);
  observer?.disconnect();
  document.removeEventListener("visibilitychange", visibility);
  controls?.dispose();
  renderer?.dispose();
  scene?.traverse((object) => {
    if (object instanceof THREE.Mesh) {
      object.geometry?.dispose();
      const material = object.material;
      if (Array.isArray(material)) material.forEach((item) => item.dispose());
      else material?.dispose();
    }
  });
  renderer?.domElement.remove();
});
</script>

<template>
  <article class="robot-panel panel">
    <header class="robot-head">
      <div class="camera-title">
        <span class="icon-tile"><Box :size="19" /></span>
        <div><h3>机械臂实时视图</h3><p>base_link · 服务器实际 FK</p></div>
      </div>
      <div class="robot-meta">
        <span class="detected-count">检测物体 {{ visibleDetections }}</span>
        <div class="model-state" :class="modelState">
          <Layers3 :size="14" />{{ modelState === "ready" ? "URDF 模型" : modelState === "fallback" ? "FK 骨架" : "模型加载" }}
        </div>
      </div>
    </header>
    <div ref="host" class="robot-canvas">
      <div v-if="!telemetry.connected" class="model-offline">机械臂反馈未连接 · 保留最后姿态</div>
      <div v-if="visibleDetections" class="target-caption">
        <Crosshair :size="13" />
        {{ target ? `已选 T${store.selectedTrack}` : `检测 ${visibleDetections} 个物体` }}
      </div>
    </div>
    <footer class="robot-foot">
      <div class="joint-line">
        <span v-for="(joint, index) in joints.slice(0, 6)" :key="index">
          J{{ index + 1 }} {{ Number(joint * 180 / Math.PI).toFixed(1) }}°
        </span>
      </div>
      <div class="tcp-line">
        <Crosshair :size="14" />
        TCP
        <strong v-if="tcp.length">{{ tcp.slice(0, 3).map((value) => Number(value).toFixed(3)).join(" / ") }} m</strong>
        <strong v-else>等待反馈</strong>
      </div>
    </footer>
  </article>
</template>
