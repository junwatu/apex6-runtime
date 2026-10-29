# APEX-6 — web runtime (three.js)

A dependency-free ES module that drives the APEX-6 six-axis robot in any browser
scene graph. It sets the model's joint pivots directly (there is no skeleton) and
adds a gripper, pose helpers and cable routing so the flexible hoses follow the
arm.

```
apex6-runtime.js   the runtime (Apex6Map, Apex6Robot, Apex6Cables)
index.html         a runnable demo (three.js from a CDN, no build step)
package.json       ESM metadata (optional `npm install` of this folder)
```

`apex6-runtime.js` imports **nothing** — it carries its own vec3/quaternion/4×4
maths and only ever touches plain `position` / `quaternion` / `scale` /
`parent` / `children` fields, which every `THREE.Object3D` provides. It also runs
head-less in Node, so the same code can be verified without a browser.

## Quick start

The demo needs the model beside it. From the bundle root:

```bash
python3 -m http.server 8000        # any static server; file:// will not work
# open http://localhost:8000/web/index.html
```

The page reads the model from `../model/apex6_robot.glb` and the map from
`../model/apex6_joint_map.json`. Override with query parameters if you keep them
elsewhere:

```
web/index.html?model=/assets/apex6_robot.glb&map=/assets/apex6_joint_map.json
```

## Use it in your own scene

```html
<script type="importmap">
{ "imports": {
  "three": "https://unpkg.com/three@0.169.0/build/three.module.js",
  "three/addons/": "https://unpkg.com/three@0.169.0/examples/jsm/"
} }
</script>

<script type="module">
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { Apex6Robot, Apex6Cables } from './apex6-runtime.js';

const gltf = await new GLTFLoader().loadAsync('/assets/apex6_robot.glb');
const map  = await fetch('/assets/apex6_joint_map.json').then(r => r.json());

scene.add(gltf.scene);

const robot = new Apex6Robot({ root: gltf.scene, map });
robot.prepare();                          // call once, at the zero pose

const cables = new Apex6Cables(robot);    // hoses follow the arm
cables.prepare();

robot.setJoints([0, -20, 70, 0, 55, 0]);
robot.setGripper(0.040);                  // 40 mm pad-face gap

renderer.setAnimationLoop(() => {
  robot.update(dt);                       // rate-limited toward the command
  cables.update();
  renderer.render(scene, camera);
});
</script>
```

> `prepare()` solves the importer's Blender→engine conversion from the known
> rest positions in the map, so the runtime works on **any** importer — glTF
> imports with a `(x, z, −y)` conversion (det +1) and mirrored imports
> (det −1, joint senses inverted) are both handled. Call it while the model is
> at its zero pose (the exported file's own pose).

### `Apex6Robot`

| member | meaning |
|---|---|
| `new Apex6Robot({ root, map })` | `root` = loaded glTF scene, `map` = parsed `apex6_joint_map.json` |
| `prepare()` → `bool` | find the rig, solve the conversion, calibrate axes |
| `setJoint(i, deg)` / `setJoints([deg…])` | command joints 0..5 in degrees, **clamped to the measured limits** |
| `getJoint(i)` / `getJoints()` / `getAppliedJoints()` | commanded / applied angles |
| `goZero()` / `goHome()` | move to the zero / home pose |
| `setGripper(m)` / `openGripper()` / `closeGripper()` / `getGripper()` | pad-face gap in metres (0.002 … 0.074) |
| `update(dt, speedScale = 1)` | rate-limited step toward the command; `speedScale = 0` snaps |
| `applyNow()` / `applyCommandNow()` | push applied / commanded state onto the transforms |
| `tcpWorld()` | TCP position as `{x, y, z}` (or fill a `THREE.Vector3` via `tcpWorld(vec)`) |
| `atTarget(tol)` | every joint and the gripper have reached their command |
| `selfTest()` | replay the map's FK poses → `{ maxError, worst, tolerance, passes }` |
| `convertBlender({x,y,z})` | map a point from the model's authoring space into the rig frame |
| `conversionNote` / `conversionDet` / `conversionResidual` | the solved conversion, determinant and rest-fit residual (m) |
| `map` (`Apex6Map`) | `limitMin(i)`, `limitMax(i)`, `homeDeg()`, `maxSpeed(i)`, `fingerHalf(m)`, `openingMin/Max`, `joints`, `cables`, … |

### `Apex6Cables`

Rebuilds each `Cable_*` tube every frame from the same spine the model was
authored with:

```
p(t) = pa + (pb − pa)·t + away · slack·sin(π·t)
slack = slackBase · min(1, 0.55 + 0.6·|pb − pa|)
```

Attachment points come from the joint-parented `*_StrainRelief` / `*_Clamp`
objects; radius, slack and bow come from the map. Geometry is written back
through the host's own geometry API (`BufferGeometry.setAttribute`) using the
constructors the loader already created, so no `three` import is needed here and
the existing material is kept.

| option | default | meaning |
|---|---|---|
| `routing` | `true` | rebuild each frame; set `false` to freeze at the baked pose |
| `spineSamples` | `24` | rings along each cable |
| `ringSegments` | `14` | vertices around each cable |

```js
cables.prepare();          // after robot.prepare()
cables.update();           // each frame
cables.routing = false;    // stop routing
```

## Verify it

```js
const r = robot.selfTest();
console.log(r.maxError, r.passes);
```

`selfTest()` replays the poses carried in `apex6_joint_map.json` and compares
the live `CTRL_TCP` against the model's own forward-kinematics result mapped
through the solved conversion. Measured against the shipped model:
**5.0e-7 m** worst error over the five poses (tolerance 1 mm), on both the
pre-converted (det +1) and mirrored (det −1) import conventions; cable end caps
land on their attachments to **2e-16 m** and the spine bulge matches the
formula to **1.2e-9 m**.

## License

See `../LICENSE`.
