# APEX-6 — Unity runtime

Runtime control kit for the **APEX-6** six-axis industrial robot model. It drives
the exported rig by setting its joint pivots directly — there is **no Armature
and no Animator** — and adds a gripper, pose presets, optional inverse
kinematics, an in-game control panel, and cable routing so the flexible hoses
follow the arm.

This package contains **code only**. The model is shipped beside it in the
bundle's `model/` folder (see *Import the model*).

```
Runtime/            Apex6Controller, Apex6JointMap, Apex6Cables, Apex6IK,
                    Apex6ControlPanel, Apex6Demo, Apex6SelfTest   (Apex6.Runtime.dll)
Editor/             Apex6EditorCheck — one-click import validation   (Apex6.Editor.dll)
apex6_joint_map.json  the same joint/gripper/cable data, machine-readable
```

Plain C#, **no external packages**, no ROS, no physics.

---

## Install

**From a folder** — copy the whole `unity/` folder into your project's
`Packages/` directory (or use *Package Manager → Add package from disk* and
select `package.json`). **From git** — *Package Manager → Add package from git
URL* with the repository URL and the sub-path to this folder.

Requires Unity **6000.0** or newer (verified on 6000.6).

## Import the model

Two equivalent routes; pick one.

| route | file | notes |
|---|---|---|
| FBX | `model/apex6_robot_unity.fbx` | needs no extra package; units declared as metres |
| glTF | `model/apex6_robot.glb` | needs the [glTFast](https://docs.unity3d.com/Packages/com.unity.cloud.gltfast@latest) package (`com.unity.cloud.gltfast`); metres by definition |

Drop the file into `Assets/`, select it and set **Rig → Animation Type = None**
(there is no armature). Leave the scale at its default.

> Do **not** use `model/apex6_robot.fbx` — that variant declares its unit as
> centimetres and imports ~100× too small. If you must use it, set
> **Scale Factor = 100**.

You get a named pivot hierarchy:

```
CTRL_J1_Waist
 └ CTRL_J2_Shoulder
    └ CTRL_J3_Elbow
       └ CTRL_J4_WristRoll
          └ CTRL_J5_WristPitch
             └ CTRL_J6_ToolRoll
                ├ (J6 meshes)
                ├ CTRL_Gripper
                │   ├ CTRL_FingerL   (palm + finger meshes)
                │   └ CTRL_FingerR
                └ CTRL_TCP           (tool centre point)
```

## Control it

1. Drag the imported model root into the scene.
2. Add the **`Apex-6 Controller`** component to that root.
3. Leave **Auto Find Rig** on — it locates the ten `CTRL_*` transforms by name.

```csharp
using Apex6;
using UnityEngine;

public class MyCell : MonoBehaviour
{
    public Apex6Controller robot;

    void Start()
    {
        robot.GoHome();
        robot.SetJoint(1, -30f);                       // J2, degrees, clamped
        robot.SetJoints(new float[] { 0, -30, 75, 0, 40, 90 });
        robot.SetGripper(0.040f);                      // 40 mm pad-face gap
        Debug.Log(robot.TcpPosition);                  // world TCP
    }
}
```

The controller does **not** assume a fixed axis vector. On `Prepare()` it solves
the importer's Blender→engine conversion from the known rest positions carried
in `apex6_joint_map.json`, then expresses each joint axis in its parent frame.
That is why the **same component works on both the FBX and the GLB** without you
editing the import: the FBX bakes the Z-up → Y-up conversion into
`CTRL_J1_Waist.localRotation` (a −90° X) with child locals still in Blender
coordinates, while the glTF arrives fully pre-converted.

### API (`Apex6Controller`)

| member | meaning |
|---|---|
| `SetJoint(i, deg)` / `SetJoints(float[])` | command joints 0..5 in degrees, **clamped to the measured limits** |
| `SetJointRadians(i, rad)` | same, radians |
| `GetJoint(i)` / `GetJoints()` | commanded angles (degrees) |
| `GetAppliedJoints()` | angles actually on the transforms (useful while smoothing) |
| `GoHome()` / `GoZero()` | move to the home pose / all zeros |
| `SetGripper(m)` / `OpenGripper()` / `CloseGripper()` / `GetGripper()` | parallel-jaw pad-face gap |
| `AtTarget()` | every joint and the gripper have reached their command |
| `Prepare()` | find the rig + solve the import calibration (once, on first `Update`) |
| `ApplyNow()` | re-apply the rate-limited state to the transforms |
| `ApplyCommandNow()` | push the commanded pose onto the transforms this same frame (bypasses the rate limiter) |
| `TcpPosition` / `TcpRotation` | world pose of `CTRL_TCP` |
| `AxisLocal(i)` | the solved rotation axis for joint `i` in its parent frame |
| `ConvertBlender(v)` | map a Blender (Z-up) vector into the imported rig's frame |
| `ConversionResidual` / `ConversionNote` | rest-position fit error (m) / the solved conversion matrix as text |
| `speedScale` (0..1), `speedMultiplier` | rate limiting in `LateUpdate`; `speedScale = 0` snaps instantly |

### Joints, axes and limits

| # | pivot | reference axis | limits (deg) | home (deg) | max speed (deg/s) |
|---|---|---|---|---|---|
| J1 | `CTRL_J1_Waist` | `(0, 1, 0)` | −170 … 170 | 0 | 120.3 |
| J2 | `CTRL_J2_Shoulder` | `(0, 0, −1)` | −70 … 66 | −20 | 106.0 |
| J3 | `CTRL_J3_Elbow` | `(0, 0, −1)` | −120 … 132 | 70 | 117.5 |
| J4 | `CTRL_J4_WristRoll` | `(0, 1, 0)` | −185 … 185 | 0 | 183.3 |
| J5 | `CTRL_J5_WristPitch` | `(0, 0, −1)` | −138 … 138 | 55 | 174.8 |
| J6 | `CTRL_J6_ToolRoll` | `(0, 1, 0)` | −350 … 350 | 0 | 206.3 |

The reference axis is the exported convention; the **solved** axis is read from
the rig at runtime and exposed as `robot.AxisLocal(i)`. Both produce the same
motion — the acceptance test is that the TCP matches the project's own forward
kinematics (see *Verify*, below).

The two gripper jaws slide along the axis between their imported rest positions;
the pad-face gap maps to `opening/2 + 0.024 m`. Stroke is 2 mm (closed) to 74 mm
(open). The tool axis points along each pivot's local **+Y** (`CTRL_TCP` sits
0.160 m along it from J6).

### Cables follow the arm (`Apex6Cables`)

The flexible hose tubes are baked as static meshes at the export pose, so on
their own they would sit still while the arm moves. **`Apex6Cables`** rebuilds
each one every frame from the same spine the model was authored with:

```
p(t) = pa + (pb - pa)*t + away * slack*sin(pi*t)
slack = slackBase * min(1, 0.55 + 0.6*|pb - pa|)
```

Add it to the same GameObject as the `Apex6Controller` (or **Add Component →
APEX-6 → Apex-6 Cables**). The attachment points come from the joint-parented
`*_StrainRelief` / `*_Clamp` meshes (which already follow the arm); the radius,
slack and bow direction come from `apex6_joint_map.json`. No *Read/Write* import
setting is needed.

| field | default | meaning |
|---|---|---|
| `routing` | on | rebuild every frame; off = freeze at the baked pose |
| `spineSamples` | 24 | rings along each cable |
| `ringSegments` | 14 | vertices around each cable |
| `logCalibration` | off | print the per-cable radius / slack / bow once on `Start` |

> These are routed, not simulated: each cable is rebuilt from its two attachment
> frames plus a fixed slack, exactly as in the original renders.

### Inverse kinematics (optional)

`Apex6IK` is a small cyclic-coordinate-descent solver, no packages:

* **Position** — set `target`; the TCP is driven to that Transform, honouring
  every joint limit.
* **Orientation** — set `aimAt`; J4/J5/J6 point the tool axis at it.

It is a *kinematic* solver (position only, no physics, no torque).

### Runtime control panel (`Apex6ControlPanel`)

An on-screen panel for driving the arm by hand at runtime. Plain **IMGUI**, so
it needs no Canvas, no prefab and no font asset — add it to the same GameObject
as the controller and press Play.

```
APEX-6  control
├ Joints        six sliders (limit-clamped) + typed value + per-joint "0"
├ Gripper       pad-face gap slider (mm) + Open / Close
├ Pose presets  Zero, Home, Reach, Fold, Inspect, Wide
├ Motion        "Instant", speed scale, optional demo sweep
├ Cables        toggle cable routing on/off
├ Inverse kin.  optional: create/drag an IK target and Solve now
├ Readout       live TCP position + rotation, joint lag, gripper gap, scale
└ Home / Zero / Hide
```

Where the project still enables the legacy Input API there is also a **Tab**
hotkey to toggle it; on *Input System Package (New) only* the hotkey compiles
out and the Hide/Show tab is used instead.

---

## Verify it on your import

Add **`Apex-6 Self Test`** to the imported root, set `root`, then use the
component's context menu (⚙) → **Run Self Test** and read the Console. Or use
the menu **APEX-6 → Run Import Validation (FBX / GLB)**, which also writes a
report. It reports:

1. **conversion** — the Blender→engine matrix actually applied (solved from the
   known rest positions), its determinant, and the rest-position fit residual;
2. **scale** — the live zero-pose TCP against the expected 1.770 m tool height;
3. **forward kinematics** — five posed TCPs against the project's own
   `fk_position64` (the real correctness test), with a worst-case error;
4. **joint response** — the TCP displacement from a +2° move on each joint;
5. **gripper** — the finger gap at open and closed.

## Alternative: URDF

`model/apex6_robot.urdf` describes the same chain for the Unity
[URDF Importer](https://github.com/Unity-Technologies/URDF-Importer) (or ROS):
6 revolute joints with the correct axes and limits, and joint points
J1 (0, 0, 0), J2 (0.07, 0, 0.45), J3 (0.07, 0, 1.05), J4/J5 (0.07, 0, 1.49),
J6 (0.07, 0, 1.61).

> The URDF carries **collision cylinders only — no visual meshes**. It is a
> kinematics/collision description. Use the FBX/GLB for visuals and the URDF
> when you want real joint drives.

## Measured results

* Joint-map maths vs the project's own `fk_position64`: **4.2e-07 m** over 302
  poses.
* Live Editor (Unity **6000.6.7f1**), both the FBX and the GLB imported: **10/10
  pivots**, **5/5 FK poses within 0.003 mm**, zero-pose TCP
  `(−0.070, 1.770, 0.000) m`, gripper 122.0 → 50.0 mm.
* Cable routing: rebuilt tubes land on their attachment transforms to
  **≤ 0.0006 mm** at three poses; spine bulge within **0.0007 mm** of the
  authored math.

## License

See `LICENSE.md`.
