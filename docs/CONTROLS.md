# APEX-6 — control semantics

The three runtimes (Unity, web, Godot) are the same algorithm in three
languages, so they behave identically. This page is the shared contract; each
engine's README lists its own type names and idioms.

## 1. What "driving the robot" means

There is no skeleton and no animation clip. A runtime writes the local transform
of ten nodes (`CTRL_J1..J6`, `CTRL_Gripper`, `CTRL_FingerL/R`, `CTRL_TCP`) and
the model follows. Everything else — the castings, panels, bolts, markings and
hose fittings — is parented under those pivots and rides along.

## 2. Joints

Index `0..5` = J1…J6. Commands are in **degrees**, and are always **clamped to
the measured limits** in `apex6_joint_map.json` — you cannot command a pose the
arm is not allowed to hold. Readings come back in degrees.

| index | joint | pivot |
|---|---|---|
| 0 | J1 | `CTRL_J1_Waist` |
| 1 | J2 | `CTRL_J2_Shoulder` |
| 2 | J3 | `CTRL_J3_Elbow` |
| 3 | J4 | `CTRL_J4_WristRoll` |
| 4 | J5 | `CTRL_J5_WristPitch` |
| 5 | J6 | `CTRL_J6_ToolRoll` |

Positive angles follow the authoring-frame axis (J1/J4/J6 about Z, J2/J3/J5
about Y, right-handed). Limits, homes and rated speeds are in
[MODEL.md](MODEL.md#joints).

## 3. Two states: commanded and applied

Every runtime keeps a **command** and an **applied** state:

* `setJoint/set_joint`… updates the *command* only.
* `update(dt, speedScale)` steps the *applied* state toward the command at the
  joint's rated speed (a simple, frame-rate independent rate limiter).
  `speedScale = 0` (or the "Instant" toggle) snaps immediately.
* `applyCommandNow()` / `apply_command_now()` sets the applied state equal to
  the command and writes the transforms **in the same frame** — use this when
  you must pose the arm deterministically (the self tests and IK solvers do).
* `getAppliedJoints()` / `get_applied_joints()` reads what is actually on the
  transforms; `AtTarget()` / `at_target()` tells you when a move is finished.

> The rate limiter is a convenience, not a dynamics simulation: there is no
> torque, no inertia and no collision response. Build those on top if you need
> them (the URDF is the physics-ready description).

## 4. Gripper

The gripper is a single scalar: the **clear gap between the two pad faces**, in
metres, from `opening_min` (0.002, closed) to `opening_max` (0.074, open). Each
jaw is offset from the tool axis by `gap/2 + 0.024 m` and slides along the axis
that joins the two jaws' rest positions.

| action | Unity | web | Godot |
|---|---|---|---|
| set gap (m) | `SetGripper(m)` | `setGripper(m)` | `set_gripper(m)` |
| open / close | `OpenGripper()` / `CloseGripper()` | `openGripper()` / `closeGripper()` | `open_gripper()` / `close_gripper()` |
| read | `GetGripper()` | `getGripper()` | `gripper()` |

## 5. Poses

`goZero()` moves to all-zero joints; `goHome()` moves to the configured home
pose (J2 −20°, J3 70°, J5 55°). The control panels also offer `Reach`, `Fold`,
`Inspect` and `Wide` presets built from the same limits.

## 6. Tool centre point

`TcpPosition` / `tcpWorld()` / `tcp_world()` gives the world position of
`CTRL_TCP`. The tool axis is each wrist pivot's local **+Y**; the TCP is 0.160 m
along it from J6.

## 7. Cables

Five hose tubes follow the arm. A cable component rebuilds each tube each frame
from its two attachment frames (read from the joint-parented
`*_StrainRelief` / `*_Clamp` objects) plus the per-route radius, slack and bend
direction in `apex6_joint_map.json`, using the same spine the model was authored
with ([MODEL.md](MODEL.md#cables)). Each component has a `routing` toggle to
freeze the tubes.

This is **animation-driven routing, not a physics rope** — the hoses stay
attached and bow predictably, exactly like the reference animation.

## 8. Why the runtimes "solve" the import

The model is authored Z-up; glTF and FBX importers each place the Y-up
conversion differently, and a mirrored import reverses every joint's sense.
Rather than hard-code one convention, each runtime:

1. reads the ten known rest positions from `apex6_joint_map.json`;
2. solves the linear map from authoring space to the imported frame by least
   squares (`conversion` / `ConversionNote` / `conversion_note`, with the
   residual exposed);
3. expresses each joint axis in its parent frame and applies
   `R_local = P⁻¹·Rot(axis, θ)·P`, which is exact for any parent frame —
   including a mirrored one.

That is why `apex6_joint_map.json` must ship next to the model, and why
`prepare()` must be called while the model is at its zero pose.

## 9. Self test

Each runtime can replay the five reference poses from the map and compare the
live TCP against the expected positions, reporting the worst error and whether
it is inside the 1 mm tolerance. Run it on your own import to confirm the
conversion solved correctly:

| engine | call |
|---|---|
| Unity | `Apex6SelfTest` component / **APEX-6 → Run Import Validation** |
| web | `robot.selfTest()` |
| Godot | `robot.self_test()` |
