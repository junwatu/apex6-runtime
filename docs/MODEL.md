# APEX-6 — the model

Reference for the asset itself: units, hierarchy, node names and the numbers the
runtimes read from `apex6_joint_map.json`.

## Files

| file | format | notes |
|---|---|---|
| `apex6_robot.glb` | glTF 2.0 binary | metres, Y-up. 216 nodes, 112 mesh primitives, 13 materials, no external images. Uses `KHR_materials_clearcoat` and `KHR_materials_anisotropy`. |
| `apex6_robot_unity.fbx` | FBX | same geometry; declares its unit as **metres** (`UnitScaleFactor = 100`). Use this in Unity. |
| `apex6_robot.fbx` | FBX | same geometry; declares its unit as **centimetres** (`UnitScaleFactor = 1`). Use in DCCs that want raw units, or set Scale Factor 100 on import. |
| `apex6_robot.urdf` | URDF | 7 links / 6 revolute joints. **Collision cylinders only — no visual meshes.** For ROS and physics/URDF importers. |
| `apex6_joint_map.json` | JSON | the runtime contract: joints, limits, rest points, gripper, cable routes, FK self-test poses. |

No textures are shipped: the materials are procedural PBR (base colour,
metallic, roughness, clearcoat). The only UV-mapped pieces are the eight
`Mark_*` decal glyphs.

## Units and scale

* 1 unit = **1 metre** in the `.glb` and in `apex6_robot_unity.fbx`.
* `apex6_robot.fbx` is in centimetres — that is a declaration only; the geometry
  bytes are identical.
* The arm is **1.770 m** tall at rest and reaches **1.390 m**.

## Hierarchy

The model is a named pivot hierarchy. There is **no skeleton and no animation
clip** — a runtime drives it by setting these transforms:

```
CTRL_J1_Waist                 (base yaw)
 └ CTRL_J2_Shoulder           (shoulder pitch)
    └ CTRL_J3_Elbow           (elbow pitch)
       └ CTRL_J4_WristRoll    (wrist roll)
          └ CTRL_J5_WristPitch(wrist pitch)
             └ CTRL_J6_ToolRoll(tool roll)
                ├ CTRL_Gripper
                │   ├ CTRL_FingerL
                │   └ CTRL_FingerR
                └ CTRL_TCP         (tool centre point)
```

Plus, inside the model:

* `Cable_BaseToWaist`, `Cable_WaistToArm`, `Cable_ArmToForearm`,
  `Cable_ForearmToWrist`, `Cable_WristToGripper` — the flexible hose tubes.
* `Cable_*_StrainRelief` and `Cable_*_Clamp` — the hose end fittings (parented
  to the joints; a runtime reads the attachment points from these).
* `Mark_*` — painted markings; _meshes_ (castings, bolts, panels, ribs…) whose
  names start with the link they belong to (`Base_*`, `Waist_*`, `UpperArm_*`,
  `Forearm_*`, `Wrist*`, `ToolRoll_*`, `Gripper_*`, `Finger_*`).
* Workcell props from the demo scene also ship in the `.glb`
  (`Workcell_Floor`, `Pickup_Pedestal_*`, `Place_Platform_*`,
  `Training_Part*`, `Safety_Pillar*`). Delete them if you only want the arm.

At the rest (zero) pose every `CTRL_*` node sits exactly at its kinematic joint,
with the importer's own orientation convention:

| node | position (authoring frame, m) |
|---|---|
| `CTRL_J1_Waist` | (0, 0, 0) |
| `CTRL_J2_Shoulder` | (0.07, 0, 0.45) |
| `CTRL_J3_Elbow` | (0.07, 0, 1.05) |
| `CTRL_J4_WristRoll` | (0.07, 0, 1.49) |
| `CTRL_J5_WristPitch` | (0.07, 0, 1.49) |
| `CTRL_J6_ToolRoll` | (0.07, 0, 1.61) |
| `CTRL_Gripper` | (0.07, 0, 1.61) |
| `CTRL_TCP` | (0.07, 0, 1.77) |
| `CTRL_FingerL` | (0.07, −0.061, 1.61) |
| `CTRL_FingerR` | (0.07, +0.061, 1.61) |

The tool axis runs along each wrist pivot's local **+Y**; `CTRL_TCP` is
**0.160 m** along it from `CTRL_J6_ToolRoll`.

## Joints

Authoring-frame axes (the runtimes derive the imported frame themselves):

| # | pivot | axis | min° | max° | home° | max speed °/s | max accel °/s² |
|---|---|---|---|---|---|---|---|
| J1 | `CTRL_J1_Waist` | Z | −170 | 170 | 0 | 120.3 | 240.6 |
| J2 | `CTRL_J2_Shoulder` | Y | −70 | 66 | −20 | 106.0 | 206.3 |
| J3 | `CTRL_J3_Elbow` | Y | −120 | 132 | 70 | 117.5 | 229.2 |
| J4 | `CTRL_J4_WristRoll` | Z | −185 | 185 | 0 | 183.3 | 366.7 |
| J5 | `CTRL_J5_WristPitch` | Y | −138 | 138 | 55 | 174.8 | 349.5 |
| J6 | `CTRL_J6_ToolRoll` | Z | −350 | 350 | 0 | 206.3 | 412.5 |

The limits are the **measured, collision-screened travel** of the modelled
geometry, not nominal catalogue values — J2, J3 and J5 in particular are trimmed
so the arm cannot fold into itself.

## Gripper

A parallel two-jaw gripper with a **2 mm to 74 mm** clear gap between the pad
faces. The gap maps to each jaw's offset from the tool axis as
`opening/2 + 0.024 m`. The jaws slide along the axis between their rest
positions; the runtimes derive that axis rather than assuming one.

## Cables

Five flexible hoses, routed between a strain-relief and a clamp on adjacent
links. Runtime cable components rebuild each tube each frame from

```
p(t) = pa + (pb − pa)·t + away·slack·sin(π·t)
slack = slackBase·min(1, 0.55 + 0.6·|pb − pa|)
```

| tube | radius (m) | slack (m) |
|---|---|---|
| `Cable_BaseToWaist` | 0.0125 | 0.028 |
| `Cable_WaistToArm` | 0.0105 | 0.030 |
| `Cable_ArmToForearm` | 0.0095 | 0.034 |
| `Cable_ForearmToWrist` | 0.0085 | 0.026 |
| `Cable_WristToGripper` | 0.0075 | 0.022 |

The bend direction (`bow`) for each route is carried in `apex6_joint_map.json`.

## Forward-kinematics self-test

`apex6_joint_map.json` carries five reference poses with the exact tool-centre
position the model's own solver produces, so **any** runtime can prove itself on
the real asset. Positions are in the authoring frame, metres, tolerance 1 mm:

| joints (deg) | TCP x | TCP y | TCP z |
|---|---|---|---|
| [0, 0, 0, 0, 0, 0] | 0.070000 | 0.000000 | 1.770000 |
| [0, −20, 70, 0, 55, 0] | 0.472307 | 0.000000 | 1.224173 |
| [0, −45, 90, 0, 30, 0] | 0.227322 | 0.000000 | 1.257860 |
| [90, 20, 40, 60, −90, 120] | 0.242487 | 0.586263 | 1.355059 |
| [−135, −60, 120, −150, 100, −200] | 0.065195 | 0.260177 | 1.152499 |

Each runtime exposes this as a one-call self test (`selfTest()` /
`SelfTest()` / `self_test()`); the shipped results are in each README.

## Mesh budget

| | |
|---|---|
| mesh objects | 201 (180 robot + 21 workcell) |
| triangles | 86,790 placed |
| vertices | 43,577 |
| materials | 13 |
| image maps | 0 |

The model is intentionally **untextured and unwrapped** apart from the eight
`Mark_*` decal glyphs (each with its own `UVMap`). Materials are procedural PBR.
If you need a baked real-time/AR asset, that is a separate step on top of this
geometry.
