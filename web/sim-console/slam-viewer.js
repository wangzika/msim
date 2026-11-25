import * as THREE from 'three';
import {OrbitControls} from '/vendor/three/OrbitControls.js';
import {ColladaLoader} from '/vendor/three/loaders/ColladaLoader.js';
import {STLLoader} from '/vendor/three/loaders/STLLoader.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const palette = {lio: 0x49d6e9, loose: 0xf6a86d, truth: 0xe7edf4, gps: 0x65bfff, route: 0xf2d26f, return: 0x94e49c, planned: 0xd9a5ff, goalBoundary: 0xd9a5ff};

function finitePoint(point) {return point && point.length >= 3 && point.slice(0, 3).every(Number.isFinite);}

export class SlamViewer {
  constructor(container, {sceneMode = false, onChange = () => {}} = {}) {
    this.container = container; this.sceneMode = sceneMode; this.onChange = onChange;
    this.scene = new THREE.Scene(); this.scene.background = new THREE.Color('#0b1420');
    this.renderer = new THREE.WebGLRenderer({antialias: true, preserveDrawingBuffer: true, alpha: false});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.domElement.setAttribute('aria-label', sceneMode ? 'Gazebo ENU 场景三维视图' : 'SLAM 三维点云视图');
    this.renderer.domElement.tabIndex = 0;
    container.prepend(this.renderer.domElement);
    this.camera = new THREE.PerspectiveCamera(48, 1, .05, 20000);
    this.camera.position.set(25, -35, 28);
    this.createControls();
    this.grid = new THREE.GridHelper(200, 40, '#34475a', '#202e3f');
    this.grid.rotation.x = Math.PI / 2; this.scene.add(this.grid);
    this.axes = new THREE.AxesHelper(4); this.axes.renderOrder = 5; this.scene.add(this.axes);
    this.layers = {}; this.lines = {}; this.raw = {}; this.boxGroup = new THREE.Group();
    this.scene.add(this.boxGroup);
    this.modelGroup=new THREE.Group();this.scene.add(this.modelGroup);this.modelGeneration=0;
    if (sceneMode) {
      this.scene.add(new THREE.HemisphereLight(0xe4efff,0x7e7563,2));
      const sun=new THREE.DirectionalLight(0xffffff,2.2);sun.position.set(-300,-200,600);this.scene.add(sun);
    }
    for (const key of ['map', 'current']) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
      geometry.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
      const material = new THREE.PointsMaterial({size: key === 'map' ? 2 : 2.6,
        sizeAttenuation: false, vertexColors: true, opacity: key === 'map' ? .9 : 1, transparent: true});
      this.layers[key] = new THREE.Points(geometry, material); this.scene.add(this.layers[key]);
    }
    this.robot = new THREE.Group();
    const body = new THREE.Mesh(new THREE.OctahedronGeometry(.55), new THREE.MeshBasicMaterial({color: '#f6fafc'}));
    this.robot.add(body);
    const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 2.2, 0x49d6e9, .6, .4);
    this.robot.add(arrow, new THREE.AxesHelper(1.1));this.robot.visible = false; this.scene.add(this.robot);
    this.goalMarker=new THREE.Mesh(new THREE.SphereGeometry(.8,12,8),new THREE.MeshBasicMaterial({color:0xd9a5ff,depthTest:false}));
    this.goalMarker.visible=false;this.goalMarker.renderOrder=9;this.scene.add(this.goalMarker);
    if (sceneMode) {
      this.poseLabel=document.createElement('div');this.poseLabel.className='aircraft-label';
      this.poseLabel.textContent='无人机真值';this.poseLabel.hidden=true;container.append(this.poseLabel);
    }
    this.pointSize = 2; this.budget = 90000; this.colorMode = 'height'; this.follow = false;
    this.fitted = false; this.viewMode = '3d'; this.pose = null; this.bounds = null;this.sceneExtent='local';
    this.fps = 0; this.frames = 0; this.frameAt = performance.now();
    new ResizeObserver(() => this.resize()).observe(container);
    container.addEventListener('dblclick', () => this.fit());
    container.addEventListener('pointerdown', event => {
      if (event.button === 2 || event.shiftKey) {this.follow = false; this.onChange();}
    });
    this.resize(); this.animate();
  }

  createControls() {
    if (this.controls) this.controls.dispose();
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true; this.controls.dampingFactor = .12;
    this.controls.screenSpacePanning = true; this.controls.minDistance = .2;
    this.controls.maxDistance = 10000; this.controls.enableRotate = this.viewMode !== 'top';
    this.controls.addEventListener('start', () => {this.userInteracted = true;});
  }

  resize() {
    const width = this.container.clientWidth, height = this.container.clientHeight;
    if (!width || !height) return;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    if (this.camera.isOrthographicCamera) this.updateOrtho();
    this.camera.updateProjectionMatrix();
  }

  updateOrtho() {
    const aspect = this.container.clientWidth / Math.max(1, this.container.clientHeight);
    const half = this.orthoSpan || 30;
    this.camera.left = -half * aspect; this.camera.right = half * aspect;
    this.camera.top = half; this.camera.bottom = -half;
  }

  setView(mode) {
    const target = this.controls.target.clone();
    this.viewMode = mode; this.follow = false;
    if (mode === 'top') {
      this.camera = new THREE.OrthographicCamera(-50, 50, 50, -50, .05, 20000);
      this.camera.up.set(0, 1, 0);
    } else {
      this.camera = new THREE.PerspectiveCamera(48, 1, .05, 20000);
      this.camera.up.set(0, 0, 1);
    }
    this.camera.position.copy(target).add(new THREE.Vector3(25, -35, 28));
    this.createControls(); this.controls.target.copy(target); this.resize();
    if (this.sceneMode && this.modelBounds) this.fitSceneContext(this.sceneExtent==='all');else this.fit();
    this.onChange();
  }

  pointBounds(points) {
    if (!points.length) return null;
    const samples = [[], [], []]; const step = Math.max(1, Math.floor(points.length / 3 / 3000));
    for (let i = 0; i < points.length; i += 3 * step) for (let j = 0; j < 3; j++) samples[j].push(points[i + j]);
    const low = [], high = [];
    for (let j = 0; j < 3; j++) {
      samples[j].sort((a, b) => a - b);
      low[j] = samples[j][Math.floor(samples[j].length * .01)];
      high[j] = samples[j][Math.min(samples[j].length - 1, Math.floor(samples[j].length * .99))];
    }
    return new THREE.Box3(new THREE.Vector3(...low), new THREE.Vector3(...high));
  }

  setCloud(key, positions) {
    this.raw[key] = positions;
    const total = positions.length / 3, maximum = key === 'map' ? this.budget : 12000;
    const count = Math.min(total, maximum); const sampled = new Float32Array(count * 3);
    for (let i = 0; i < count; i++) {
      const at = Math.floor(i * total / count) * 3;
      sampled.set(positions.subarray(at, at + 3), i * 3);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(sampled, 3));
    this.layers[key].geometry.dispose(); this.layers[key].geometry = geometry;
    this.recolor(key);
    if (key === 'map' || !this.raw.map?.length) this.bounds = this.pointBounds(sampled);
    if (positions.length && !this.fitted) this.fit();
    else if (key === 'map' && this.bounds && !this.userInteracted && !this.follow &&
        this.bounds.getBoundingSphere(new THREE.Sphere()).radius > (this.fittedRadius || 5) * 1.45) this.fit();
  }

  recolor(key) {
    const points = this.layers[key].geometry.getAttribute('position').array;
    const colors = new Float32Array(points.length), color = new THREE.Color();
    const bounds = this.pointBounds(points), low = bounds?.min.z || 0;
    const span = Math.max(.5, (bounds?.max.z || 1) - low);
    for (let i = 0; i < points.length; i += 3) {
      if (key === 'current') color.set('#ff876d');
      else if (this.colorMode === 'solid') color.set('#85bace');
      else {
        const value = Math.max(0, Math.min(1, (points[i + 2] - low) / span));
        color.setHSL(.64 - value * .61, .82, .59);
      }
      colors.set([color.r, color.g, color.b], i);
    }
    this.layers[key].geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  }

  setPointSize(value) {
    this.pointSize = value;
    this.layers.map.material.size = value; this.layers.current.material.size = value + .8;
  }
  setFollow(value) {
    this.follow = value;
    if (value && this.pose) {
      const offset = this.camera.position.clone().sub(this.controls.target);
      offset.setLength(Math.max(12, Math.min(35, offset.length())));
      this.controls.target.copy(this.pose); this.camera.position.copy(this.pose).add(offset);
      if (this.camera.isOrthographicCamera) {this.orthoSpan = 16;this.camera.zoom = 1;this.updateOrtho();this.camera.updateProjectionMatrix();}
      this.controls.update();
    }
    this.onChange();
  }
  setBudget(value) {this.budget = value; if (this.raw.map) this.setCloud('map', this.raw.map);}
  setColorMode(value) {this.colorMode = value; this.recolor('map');}
  setVisible(key, value) {
    if (key === 'robot') this.robotEnabled = value;
    const object = this.layers[key] || this.lines[key] || this[key];
    if (object) object.visible = value;
  }

  setGoal(point, radius=0) {
    this.goalMarker.visible=finitePoint(point);
    if(this.goalMarker.visible)this.goalMarker.position.set(...point.slice(0,3));
    const ring=finitePoint(point) && radius>0 ? Array.from({length:97},(_,i)=>[point[0]+radius*Math.cos(i*Math.PI/48),point[1]+radius*Math.sin(i*Math.PI/48),point[2]]) : [];
    this.setTrack('goalBoundary',ring);
  }

  setTrack(key, values) {
    const points = values.filter(finitePoint);
    if (!this.lines[key]) {
      this.lines[key] = new THREE.Line(new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({color: palette[key] || 0xffffff, depthTest: false, transparent: true, opacity: .95}));
      this.lines[key].renderOrder = 8;this.scene.add(this.lines[key]);
    }
    const geometry = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(...p)));
    this.lines[key].geometry.dispose(); this.lines[key].geometry = geometry;
  }

  setPose(pose) {
    if (!finitePoint(pose?.position)) return;
    this.pose = new THREE.Vector3(...pose.position);
    this.robot.position.copy(this.pose);
    if (pose.orientation?.length === 4 && pose.orientation.every(Number.isFinite))
      this.robot.quaternion.fromArray(pose.orientation).normalize();
    this.robot.visible = this.robotEnabled !== false;
  }

  setBoxes(shapes) {
    while (this.boxGroup.children.length) {
      const child = this.boxGroup.children[0]; child.geometry.dispose(); child.material.dispose(); this.boxGroup.remove(child);
    }
    for (const shape of shapes) {
      const geometry = new THREE.BoxGeometry(...shape.size);
      const material = new THREE.MeshBasicMaterial({color: '#42748a', transparent: true, opacity: .32, depthWrite: false});
      const mesh = new THREE.Mesh(geometry, material);mesh.position.set(...shape.center);mesh.rotation.z = shape.yaw;
      this.boxGroup.add(mesh);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geometry), new THREE.LineBasicMaterial({color: '#6a95a7', transparent: true, opacity: .55}));
      edges.position.copy(mesh.position); edges.rotation.copy(mesh.rotation); this.boxGroup.add(edges);
    }
  }

  clearModels() {
    this.modelGeneration++;
    this.modelGroup.traverse(object=>{
      object.geometry?.dispose();
      for (const material of Array.isArray(object.material) ? object.material : object.material ? [object.material] : []) {
        for (const value of Object.values(material)) if (value?.isTexture) value.dispose();
        material.dispose();
      }
    });
    this.modelGroup.clear();this.modelBounds=null;
  }

  async loadSceneModels(description) {
    this.clearModels();const generation=this.modelGeneration;
    const manager=new THREE.LoadingManager();
    manager.setURLModifier(value=>{
      if (value.startsWith('data:image/')) return value;
      const url=new URL(value,location.href);
      if (url.origin!==location.origin || !url.pathname.startsWith('/api/scene-assets/')) throw new Error('场景资源必须来自本机登记资源');
      return url.href;
    });
    const objects=await Promise.all(description.objects.map(async item=>{
      let object;
      const material=()=>new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(...item.color.slice(0,3),THREE.SRGBColorSpace),roughness:.9,side:THREE.DoubleSide});
      if (item.kind==='dae') {
        const result=await new ColladaLoader(manager).loadAsync(item.url);object=result.scene;
        // The official loader converts Z-up COLLADA into Y-up. Gazebo ENU is
        // Z-up, so undo that conversion while retaining the asset's metre scale.
        object.rotation.x=item.up_axis==='Z_UP' ? 0 : Math.PI/2;
      } else if (item.kind==='stl') object=new THREE.Mesh(await new STLLoader(manager).loadAsync(item.url),material());
      else if (item.kind==='box') object=new THREE.Mesh(new THREE.BoxGeometry(...item.size),material());
      else if (item.kind==='cylinder') {object=new THREE.Mesh(new THREE.CylinderGeometry(item.radius,item.radius,item.length,24),material());object.rotation.x=Math.PI/2;}
      else if (item.kind==='sphere') object=new THREE.Mesh(new THREE.SphereGeometry(item.radius,20,12),material());
      else if (item.kind==='plane') object=new THREE.Mesh(new THREE.PlaneGeometry(...item.size),material());
      else return null;
      if (item.scale) object.scale.multiply(new THREE.Vector3(...item.scale));
      const group=new THREE.Group();group.matrixAutoUpdate=false;group.matrix.fromArray(item.matrix);group.name=item.name;group.add(object);
      object.traverse(child=>{if(child.isMesh)child.castShadow=false;});
      return group;
    }));
    if (generation!==this.modelGeneration) return;
    this.setBoxes([]);
    for (const object of objects) if (object) this.modelGroup.add(object);
    this.modelGroup.updateMatrixWorld(true);this.modelBounds=new THREE.Box3().setFromObject(this.modelGroup);
    this.grid.visible=false;
    if (!this.userInteracted) this.fitSceneContext(this.sceneExtent==='all');
    return {objects:this.modelGroup.children.length,bounds:this.modelBounds};
  }

  fitSceneContext(all=false) {
    this.sceneExtent=all ? 'all' : 'local';
    if (!this.sceneMode || !this.modelBounds || this.modelBounds.isEmpty()) {this.fit();return;}
    this.sceneFocusBounds=all ? this.modelBounds.clone() : new THREE.Box3().setFromCenterAndSize(this.pose || this.modelBounds.getCenter(new THREE.Vector3()),new THREE.Vector3(180,180,70));
    this.fit();this.sceneFocusBounds=null;
  }

  fitBounds() {
    if (this.sceneFocusBounds) return this.sceneFocusBounds.clone();
    const box = this.bounds?.clone() || new THREE.Box3();
    for (const line of Object.values(this.lines)) {
      const attribute = line.geometry.getAttribute('position');
      if (line.visible && attribute?.count) box.union(new THREE.Box3().setFromBufferAttribute(attribute));
    }
    if (box.isEmpty() && this.boxGroup.children.length) box.setFromObject(this.boxGroup);
    if (box.isEmpty() && this.pose) box.setFromCenterAndSize(this.pose, new THREE.Vector3(20, 20, 12));
    if (box.isEmpty()) return null;
    return box;
  }

  fit() {
    if (!this.container.clientWidth || !this.container.clientHeight) return;
    const box = this.fitBounds(); if (!box) return;
    const center = box.getCenter(new THREE.Vector3());
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const radius = Math.max(5, sphere.radius);
    this.fittedRadius = radius;
    const aspect = this.container.clientWidth / Math.max(1, this.container.clientHeight);
    const vertical = THREE.MathUtils.degToRad(48);
    const directions = {top: [0, 0, 1], front: [0, -1, .015], side: [1, 0, .015], '3d': [1, -1.2, .82]};
    const direction = new THREE.Vector3(...directions[this.viewMode]).normalize();
    const right = new THREE.Vector3().crossVectors(this.camera.up, direction).normalize();
    const up = new THREE.Vector3().crossVectors(direction, right).normalize();
    let distance = 10;
    for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) {
      const relative = new THREE.Vector3(x, y, z).sub(center);
      distance = Math.max(distance, Math.abs(relative.dot(up)) / Math.tan(vertical / 2) + relative.dot(direction),
        Math.abs(relative.dot(right)) / (Math.tan(vertical / 2) * aspect) + relative.dot(direction));
    }
    this.camera.position.copy(center).add(direction.multiplyScalar(distance * 1.08));
    if (this.camera.isOrthographicCamera) {
      const size = box.getSize(new THREE.Vector3());
      this.orthoSpan = Math.max(5, size.y / 2, size.x / 2 / aspect) * 1.1;
      this.camera.zoom = 1; this.updateOrtho(); this.camera.updateProjectionMatrix();
    }
    this.controls.target.copy(center); this.controls.update();this.fitted = true;this.follow = false;
    this.grid.position.set(Math.round(center.x / 5) * 5, Math.round(center.y / 5) * 5, 0);
    this.onChange();
  }

  reset() {
    this.clearModels();this.setBoxes([]);
    this.sceneExtent='local';
    this.fitted = false; this.bounds = null; this.pose = null; this.robot.visible = false;
    this.setGoal(null);
    this.userInteracted = false;this.follow = false;
    for (const key of Object.keys(this.raw)) this.setCloud(key, new Float32Array());
    for (const key of Object.keys(this.lines)) this.setTrack(key, []);
  }

  exportPng() {
    this.renderer.render(this.scene, this.camera);
    return new Promise((resolve,reject) => this.renderer.domElement.toBlob(blob =>
      blob ? resolve(blob) : reject(new Error('PNG 编码失败')), 'image/png'));
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    if (!this.container.clientWidth || !this.container.clientHeight || document.hidden) return;
    if (this.follow && this.pose) {
      const delta = this.pose.clone().sub(this.controls.target).multiplyScalar(.14);
      this.camera.position.add(delta); this.controls.target.add(delta);
    }
    this.controls.update();this.renderer.render(this.scene, this.camera);
    if (this.poseLabel) {
      const point=this.pose?.clone().project(this.camera);
      this.poseLabel.hidden=!point || Math.abs(point.x)>1 || Math.abs(point.y)>1 || point.z>1 || point.z< -1;
      if (point) this.poseLabel.style.transform=`translate(${(point.x+1)*this.container.clientWidth/2+10}px,${(1-point.y)*this.container.clientHeight/2-25}px)`;
    }
    this.frames++;
    const now = performance.now();
    if (now - this.frameAt > 1000) {this.fps = Math.round(this.frames * 1000 / (now - this.frameAt)); this.frameAt = now; this.frames = 0;}
  }
}
