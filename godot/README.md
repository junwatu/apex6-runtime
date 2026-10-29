# APEX-6 — Godot 4 runtime

An add-on that drives the APEX-6 six-axis robot in **Godot 4**. It sets the
model's joint pivots directly (there is no skeleton and no `AnimationPlayer`) and
adds a gripper, pose helpers and cable routing so the flexible hoses follow the
arm.

```
addons/apex6/
  plugin.cfg              add-on manifest (Project → Project Settings → Plugins)
  apex6_controller.gd     Apex6Controller — FK, gripper, limits, rate limiting
  apex6_cables.gd         Apex6Cables — rebuilds the hose tubes each frame
examples/
  apex6_demo.gd           a runnable demo (model + controller + panel in code)
```

Pure GDScript, **no dependencies**, no physics.

## Install

1. Copy the `addons/apex6` folder into your project's `addons/` directory.
2. Copy `model/apex6_robot.glb` and `model/apex6_joint_map.json` into the project
   (e.g. at `res://apex6_robot.glb` and `res://apex6_joint_map.json`).
3. *Project → Project Settings → Plugins* → enable **APEX-6 Runtime**.

Godot imports the `.glb` natively — no extra importer is needed. To try the
demo, attach `examples/apex6_demo.gd` to a `Node3D` in an empty scene and press
Play.

## Drive the arm

```gdscript
extends Node3D

var robot: Apex6Controller

func _ready() -> void:
    # your imported model, loaded however you like
    var model: Node3D = (load("res://apex6_robot.glb") as PackedScene).instantiate()
    add_child(model)

    robot = Apex6Controller.new()
    robot.rig_root = model
    robot.map_path = "res://apex6_joint_map.json"
    add_child(robot)
    robot.prepare()                       # solves the import calibration

    var cables := Apex6Cables.new()
    cables.controller = robot
    cables.rig_root = model
    add_child(cables)
    cables.prepare()

    robot.set_joints([0, -20, 70, 0, 55, 0])
    robot.set_gripper(0.040)              # 40 mm pad-face gap
    robot.apply_command_now()
    cables.update_tubes()

func _process(delta: float) -> void:
    robot.update(delta)                   # rate-limited toward the command
    cables.update_tubes()
```

`prepare()` solves the importer's Blender→engine conversion from the known rest
positions in `apex6_joint_map.json`, so the same code works whatever the
importer did with the axis convention. Call it while the model is at its zero
pose.

### `Apex6Controller`

| member | meaning |
|---|---|
| `rig_root` / `map_path` | the model root / the JSON contract (both exported) |
| `prepare()` → `bool` | find the rig, solve the conversion, calibrate axes |
| `set_joint(i, deg)` / `set_joints([deg…])` | command joints 0..5 in degrees, **clamped to the measured limits** |
| `get_joint(i)` / `get_joints()` / `get_applied_joints()` | commanded / applied angles |
| `go_zero()` / `go_home()` | move to the zero / home pose |
| `set_gripper(m)` / `open_gripper()` / `close_gripper()` / `gripper()` | pad-face gap in metres (0.002 … 0.074) |
| `update(dt, speed_scale = 1.0)` | rate-limited step toward the command; `speed_scale = 0` snaps |
| `apply_now()` / `apply_command_now()` | push applied / commanded state onto the transforms |
| `tcp_world()` → `Vector3` | world position of `CTRL_TCP` |
| `at_target(tol)` | every joint and the gripper have reached their command |
| `self_test()` → `Dictionary` | `{ max_error, worst, tolerance, passes }` for the shipped FK poses |
| `convert_blender(v)` → `Vector3` | map a point from the model's authoring space into the rig frame |
| `conversion_note` / `conversion_det` / `conversion_residual` | the solved conversion, determinant and rest-fit residual |
| `map` | the parsed contract (`joints`, `cables`, `gripper`, `self_test`, …) |
| `limit_min(i)` / `limit_max(i)` / `finger_half(m)` / `opening_min()` / `opening_max()` | limits and gripper helpers |

The controller writes the joint pivots as
`basis = P⁻¹ · Rot(axis, θ) · P · rest`, where `P` is the parent's world basis —
exact for any parent frame, including a mirrored import.

### `Apex6Cables`

Rebuilds each `Cable_*` tube every frame from the same spine the model was
authored with:

```
p(t) = pa + (pb − pa)·t + away · slack·sin(PI·t)
slack = slackBase · min(1, 0.55 + 0.6·|pb − pa|)
```

Attachment points come from the joint-parented `*_StrainRelief` / `*_Clamp`
objects; radius, slack and bow come from the map.

| property | default | meaning |
|---|---|---|
| `routing` | `true` | rebuild each frame; `false` freezes the tubes |
| `spine_samples` | `24` | rings along each cable |
| `ring_segments` | `14` | vertices around each cable |
| `controller` / `rig_root` | — | the controller (for the solved conversion) and the model root |

Call `prepare()` after the controller is prepared, then `update_tubes()` from
`_process` (the component already does this when `routing` is on).

## Verify it

```gdscript
var st := robot.self_test()
print(st.max_error, st.passes)
```

`self_test()` replays the poses carried in `apex6_joint_map.json` and compares
the live `CTRL_TCP` against the model's own forward-kinematics result mapped
through the solved conversion. Measured against the shipped model in a head-less
Godot **4.7.2** run: **2.9e-5 m** worst error over the five poses (tolerance
1 mm), cable end caps on their attachments to the float32 limit and the spine
bulge matching the formula exactly. (The residual is Godot's `Vector3`
single-precision floor, not a wiring error.)

## License

See `../LICENSE`.
