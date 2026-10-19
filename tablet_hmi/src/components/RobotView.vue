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
let targetMarker: THREE.Group | null = null;
let trail: THREE.Line | null = null;
let observer: ResizeObserver | null = null;
let frameId = 0;
let hidden = false;
const links = new Map<string, THREE.Object3D>();
const desired = new Map<string, { position: THREE.Vector3; quaternion: THREE.Quaternion }>();
const trailPoints: THREE.Vector3[] = [];
const linkNames = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper_base", "gripper_link1", "gripper_link2", "tcp_link"];

const telemetry = computed(() => store.snapshot.robot || {});
const joints = computed<number[]>(() => telemetry.value.joints_rad || []);
const tcp = computed<number[]>(() => telemetry.value.tcp_xyzw || []);
const target = computed(() => store.currentTarget?.base_position_m || null);

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
        // Cancel that double rotation so the mesh stays in the URDF link frame.
        object.rotation.x += Math.PI;
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

function buildTarget() {
  targetMarker = new THREE.Group();
  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(0.025, 20, 14),
    new THREE.MeshBasicMaterial({ color: 0x1769ff, transparent: true, opacity: 0.82 }),
  );
  const halo = new THREE.Mesh(
    new THREE.RingGeometry(0.04, 0.047, 32),
    new THREE.MeshBasicMaterial({ color: 0x1769ff, transparent: true, opacity: 0.32, side: THREE.DoubleSide }),
  );
  halo.rotation.x = Math.PI / 2;
  targetMarker.add(sphere, halo);
  targetMarker.visible = false;
  targetMarker.name = "selected-target";
  modelRoot?.add(targetMarker);
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
  if (targetMarker) {
    const position = target.value;
    targetMarker.visible = !!position;
    if (position) targetMarker.position.set(position.x, position.z, -position.y);
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
  if (targetMarker?.visible) {
    const halo = targetMarker.children[1];
    if (halo) halo.scale.setScalar(1 + Math.sin(performance.now() / 360) * 0.08);
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
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(80 * 3), 3));
  geometry.setDrawRange(0, 0);
  trail = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x1769ff, transparent: true, opacity: 0.22 }));
  modelRoot.add(trail);
  buildTarget();
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
      <div class="model-state" :class="modelState">
        <Layers3 :size="14" />{{ modelState === "ready" ? "URDF 模型" : modelState === "fallback" ? "FK 骨架" : "模型加载" }}
      </div>
    </header>
    <div ref="host" class="robot-canvas">
      <div v-if="!telemetry.connected" class="model-offline">机械臂反馈未连接 · 保留最后姿态</div>
      <div v-if="target" class="target-caption"><Crosshair :size="13" /> 已选目标</div>
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
