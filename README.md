---
license: mit
library_name: apex6-runtime
tags:
  - robotics
  - 3d-model
  - six-axis
  - gltf
  - fbx
  - unity
  - threejs
  - godot
---

# APEX-6 — runtime bundle

**APEX-6** is an original six-axis industrial robot: a 1.77 m reach-class arm
modelled down to its bolts, modelled with clear panel gaps, seated markings and
armoured cable hoses. This bundle contains the **model** and **runtime control
kits** for three engines, plus the documentation needed to load and drive it.

> **This is a runtime/asset bundle.** It ships the model and the code that drives
> it. It does **not** include the authoring project or the generators used to
> build the model.

```
apex6-runtime/
  README.md            this file
  LICENSE              terms of use
  model/               the 3D model + the runtime contract
    apex6_robot.glb        glTF 2.0 binary (preferred: metres, Y-up, pivot hierarchy)
    apex6_robot_unity.fbx  FBX declared in metres  (Unity: use this one)
    apex6_robot.fbx        FBX declared in cm     (DCCs that want raw units)
    apex6_robot.urdf       6-revolute-joint kinematics (collision cylinders only)
    apex6_joint_map.json   joints, limits, rest points, gripper, cables, FK self-test
  unity/               Unity package (UPM)      — see unity/README.md
  web/                 three.js / ES-module runtime — see web/README.md
  godot/               Godot 4 add-on            — see godot/README.md
  docs/
    MODEL.md             the asset: units, hierarchy, node names, limits
    CONTROLS.md          control semantics shared by all three runtimes
```

## Which file for which engine

| engine | model | runtime |
|---|---|---|
| **Unity** (6000.0+) | `model/apex6_robot_unity.fbx` (or the `.glb` with [glTFast](https://docs.unity3d.com/Packages/com.unity.cloud.gltfast@latest)) | `unity/` — plain C#, UPM package `com.apex6.robot` |
| **web / three.js** | `model/apex6_robot.glb` | `web/apex6-runtime.js` — dependency-free ES module |
| **Godot 4** | `model/apex6_robot.glb` (imports natively) | `godot/addons/apex6/` — GDScript add-on |
| Blender / Maya / other DCC | `model/apex6_robot.fbx` or the `.glb` | — |
| ROS / physics engines | `model/apex6_robot.urdf` | — (collision only, no visuals) |

The model is a **named pivot hierarchy with no skeleton and no animation clips**.
Every runtime drives it the same way: set the six joint pivots directly, plus the
two gripper jaws and the cable tubes.

## Quick start

**Unity**

1. Copy `unity/` into `Packages/` (or *Package Manager → Add package from disk*).
2. Drop `model/apex6_robot_unity.fbx` into `Assets/`.
3. Add the **Apex-6 Controller** component to the imported root — it finds the
   pivots itself. Add **Apex-6 Cables** to make the hoses follow the arm.
4. `robot.SetJoints(new float[] { 0, -20, 70, 0, 55, 0 });`

**web / three.js** — copy `model/` and `web/` somewhere a static server can
serve, open `web/index.html`, or import the module:

```js
import { Apex6Robot, Apex6Cables } from './apex6-runtime.js';
const robot = new Apex6Robot({ root: gltf.scene, map });
robot.prepare();
robot.setJoints([0, -20, 70, 0, 55, 0]);
```

**Godot 4** — copy `godot/addons/apex6` into `addons/`, enable the plug-in, put
the model in the project:

```gdscript
robot = Apex6Controller.new()
robot.rig_root = model
add_child(robot)
robot.prepare()
robot.set_joints([0, -20, 70, 0, 55, 0])
```

Each runtime works on **any** importer: it solves the import's Blender→engine
conversion from the known rest positions in `apex6_joint_map.json` instead of
assuming an axis convention.

## The model at a glance

| | |
|---|---|
| joints | 6 revolute — J1 Z, J2 Y, J3 Y, J4 Z, J5 Y, J6 Z (authoring frame) |
| reach / height | 1.390 m / 1.770 m |
| base | Ø 0.400 m |
| measured limits (collision-screened) | J2 −70°…66°, J3 −120°…132°, J5 −138°…138°; J1/J4/J6 per the map |
| gripper | parallel jaw, 0.002 … 0.074 m pad-face gap |
| mesh | 201 meshes, 86,790 triangles, 13 PBR materials, 0 image maps (procedural) |
| glTF | 216 nodes, 10 `CTRL_*` pivots, 112 primitives |
| units | metres, Y-up (glTF/FBX-unity route) |

Full detail, node names and the FK reference table: **[docs/MODEL.md](docs/MODEL.md)**.
Control semantics shared by the runtimes: **[docs/CONTROLS.md](docs/CONTROLS.md)**.

## Verified

Every runtime is checked independently against the model's own forward
kinematics (`fk_position64`), not against itself:

| runtime | how | result |
|---|---|---|
| Unity kit (C#) | joint-map maths replayed over the exported glTF, 302 poses | **4.2e-7 m** |
| Unity live import | Editor 6000.6.7f1, FBX **and** GLB imported | 10/10 pivots, **5/5 FK poses ≤ 0.003 mm** |
| web runtime | head-less Node over the shipped `.glb`, both import conventions | **5.0e-7 m** |
| Godot runtime | head-less Godot 4.7.2 over the shipped `.glb` | **2.9e-5 m** (float32 floor) |
| cable routing | rebuilt tubes vs their attachment transforms / the authored spine | endpoint ≤ **0.0006 mm**, bulge ≤ **0.0007 mm** |

Re-run the self test on your own import from any runtime (see each README) — it
compares the live tool centre point against the expected pose and reports the
worst error.

## License

See [LICENSE](LICENSE).
