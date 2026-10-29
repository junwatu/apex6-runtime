@tool
class_name Apex6Controller
extends Node3D
## APEX-6 six-axis robot controller (Godot 4).
##
## Drives the exported rig by writing the joint pivots every frame. The model is
## a named pivot hierarchy with no skeleton/AnimationPlayer:
##
##     CTRL_J1_Waist -> CTRL_J2_Shoulder -> ... -> CTRL_J6_ToolRoll
##                                              -> CTRL_Gripper -> CTRL_FingerL/R
##                                              -> CTRL_TCP
##
## Nothing here is hard-coded to one importer. glTF and FBX importers place the
## Z-up -> Y-up conversion in different places, so the controller *solves* the
## conversion from the known rest positions in `apex6_joint_map.json`, then
## expresses each joint axis in its parent frame. That is the same method used
## by the Unity kit (map-vs-FK 4.2e-7 m) and the web runtime (5.0e-7 m).
##
## Usage (from a script):
##
##     var robot := Apex6Controller.new()
##     robot.rig_root = load("res://apex6_robot.glb").instantiate()
##     add_child(robot.rig_root)
##     add_child(robot)
##     robot.prepare()
##     robot.set_joints([0, -20, 70, 0, 55, 0])
##     robot.apply_command_now()
##     print(robot.tcp_world())
##
## ...or just drop `Apex6Controller` in a scene, assign `rig_root`, and it
## prepares itself on `_ready()`.

const DEG := PI / 180.0

## The loaded model root (the instantiated .glb scene).
@export var rig_root: Node3D
## The generated runtime contract. Defaults to the file shipped beside the model.
@export_file("*.json") var map_path: String = "res://apex6_joint_map.json"
## Prepare automatically in `_ready()`.
@export var auto_prepare: bool = true
## Print the solved conversion + rest-fit residual once, on prepare.
@export var log_calibration: bool = true

var map: Dictionary = {}
var ready_flag := false

var joints: Array[Node3D] = []
var gripper_root: Node3D
var finger_l: Node3D
var finger_r: Node3D
var tcp: Node3D

## Solved Blender(Z-up) -> imported frame linear map, as a Basis.
var conversion: Basis = Basis()
var conversion_det := 1.0
var conversion_residual := 0.0
var conversion_note := ""

var _parent_basis: Array = []   # Basis, per joint (parent world basis at rest)
var _world_axis: Array = []     # Vector3, per joint (axis in the rig's world frame)
var _rest_basis: Array = []     # Basis,  per joint (local basis at the zero pose)

var _command: PackedFloat64Array = PackedFloat64Array()
var _applied: PackedFloat64Array = PackedFloat64Array()
var opening := 0.074
var _opening_applied := 0.074

var _finger_base: Array = []      # Vector3 local positions at the zero pose
var _finger_axis := Vector3(0, 0, 1)
var _finger_half_at_import := 0.0


func _ready() -> void:
	if not Engine.is_editor_hint() and auto_prepare:
		if rig_root != null and not ready_flag:
			prepare()


# ---------------------------------------------------------------- setup


func prepare() -> bool:
	if rig_root == null:
		push_error("Apex6Controller: rig_root is not set")
		return false
	if not _load_map():
		return false
	_find_by_role()
	if joints.size() < 6 or joints[0] == null or joints[5] == null:
		push_error("Apex6Controller: could not find CTRL_J1..J6 in the rig")
		return false
	_calibrate()
	_cache_fingers()
	var n := joints.size()
	_command.resize(n)
	_applied.resize(n)
	for i in n:
		_command[i] = 0.0
		_applied[i] = 0.0
	opening = opening_max()
	_opening_applied = opening
	apply_now()
	ready_flag = true
	if log_calibration:
		print("Apex6Controller: %s" % conversion_note)
	return true


func _load_map() -> bool:
	var path := map_path
	if path == "":
		path = "res://apex6_joint_map.json"
	if not FileAccess.file_exists(path):
		push_error("Apex6Controller: map not found: %s" % path)
		return false
	var text := FileAccess.get_file_as_string(path)
	var parsed: Variant = JSON.parse_string(text)
	if typeof(parsed) != TYPE_DICTIONARY:
		push_error("Apex6Controller: map is not valid JSON")
		return false
	map = parsed
	return true


func _find_by_role() -> void:
	joints.clear()
	for j in map.get("joints", []):
		joints.append(_find_deep(rig_root, String(j["name"])))
	var g: Dictionary = map.get("gripper", {})
	gripper_root = _find_deep(rig_root, String(g.get("root", "")))
	var fingers: Dictionary = g.get("fingers", {})
	finger_l = _find_deep(rig_root, String(fingers.get("L", "")))
	finger_r = _find_deep(rig_root, String(fingers.get("R", "")))
	tcp = _find_deep(rig_root, String(map.get("tcp", {}).get("name", "CTRL_TCP")))


func _find_deep(root: Node, target: String) -> Node3D:
	if target == "":
		return null
	if root == null:
		return null
	if String(root.name) == target:
		return root as Node3D
	for c in root.get_children():
		var hit := _find_deep(c, target)
		if hit != null:
			return hit
	return null


func _calibrate() -> void:
	var A := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
	var B := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
	var used := 0
	var rest_points: Array = map.get("rest_points", [])
	for p in rest_points:
		var n := _find_deep(rig_root, String(p["name"]))
		if n == null:
			continue
		var b := Vector3(p["blender"][0], p["blender"][1], p["blender"][2])
		var u := n.global_transform.origin
		A.x += b * b.x
		A.y += b * b.y
		A.z += b * b.z
		B.x += u * b.x
		B.y += u * b.y
		B.z += u * b.z
		used += 1

	if used >= 4:
		conversion = B * A.inverse()
	else:
		conversion = Basis(Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0))
	conversion_det = conversion.determinant()

	conversion_residual = 0.0
	for p in rest_points:
		var n := _find_deep(rig_root, String(p["name"]))
		if n == null:
			continue
		var b := Vector3(p["blender"][0], p["blender"][1], p["blender"][2])
		conversion_residual = maxf(conversion_residual, (conversion * b - n.global_transform.origin).length())

	var mirrored := conversion_det < 0.0
	conversion_note = (
		"C=[(%+.4f,%+.4f,%+.4f)(%+.4f,%+.4f,%+.4f)(%+.4f,%+.4f,%+.4f)] det=%+.4f%s residual=%.4f mm"
		% [
			conversion.x.x, conversion.x.y, conversion.x.z,
			conversion.y.x, conversion.y.y, conversion.y.z,
			conversion.z.x, conversion.z.y, conversion.z.z,
			conversion_det,
			"  (mirrored frame -> joint senses inverted)" if mirrored else "",
			conversion_residual * 1000.0,
		])

	_parent_basis.clear()
	_world_axis.clear()
	_rest_basis.clear()
	var jdefs: Array = map.get("joints", [])
	for i in joints.size():
		var j := joints[i]
		var axb: Array = jdefs[i]["blender_axis_vec"]
		# Joint axis in the rig's world frame. A det<0 world map reverses each
		# joint's sense (Rot(a,t) == Rot(-a,-t)), so the axis is negated before
		# it is carried into the parent frame.
		var world := (conversion * Vector3(axb[0], axb[1], axb[2])).normalized()
		if mirrored:
			world = -world
		var parent_basis := Basis()
		if j != null and j.get_parent() is Node3D:
			parent_basis = (j.get_parent() as Node3D).global_transform.basis
		_parent_basis.append(parent_basis)
		_world_axis.append(world)
		_rest_basis.append(j.transform.basis if j != null else Basis())


func convert_blender(p: Vector3) -> Vector3:
	return conversion * p


# ---------------------------------------------------------------- control


func joint_count() -> int:
	return joints.size()


func set_joint(i: int, deg: float) -> void:
	if i < 0 or i >= _command.size():
		return
	_command[i] = clampf(deg, limit_min(i), limit_max(i))


func set_joints(deg: Array) -> void:
	for i in mini(_command.size(), deg.size()):
		if deg[i] != null and not is_nan(float(deg[i])):
			set_joint(i, float(deg[i]))


func get_joint(i: int) -> float:
	return _command[i] if i >= 0 and i < _command.size() else 0.0


func get_joints() -> Array:
	return Array(_command)


func get_applied_joints() -> Array:
	return Array(_applied)


func go_zero() -> void:
	set_joints(_zeros())


func go_home() -> void:
	var home: Array = []
	for j in map.get("joints", []):
		home.append(j.get("home_deg", 0.0))
	set_joints(home)


func set_gripper(opening_meters: float) -> void:
	opening = clampf(opening_meters, opening_min(), opening_max())


func open_gripper() -> void:
	set_gripper(opening_max())


func close_gripper() -> void:
	set_gripper(opening_min())


func gripper() -> float:
	return opening


func at_target(tol := 0.01) -> bool:
	for i in _applied.size():
		if absf(_applied[i] - _command[i]) > tol:
			return false
	return absf(_opening_applied - opening) < 1e-4


## Rate-limited step toward the command; call from `_process(delta)`.
func update(dt: float, speed_scale := 1.0) -> void:
	if dt <= 0.0:
		return
	var moved := false
	var jdefs: Array = map.get("joints", [])
	for i in _applied.size():
		var target: float
		if speed_scale <= 0.0:
			target = _command[i]
		else:
			var vmax: float = jdefs[i].get("max_speed_deg_s", 90.0)
			target = _move_towards(_applied[i], _command[i], vmax * speed_scale * dt)
		if absf(target - _applied[i]) > 1e-9:
			_applied[i] = target
			moved = true
	if speed_scale > 0.0 and absf(_opening_applied - opening) > 1e-9:
		_opening_applied = _move_towards(_opening_applied, opening, 0.15 * dt)
		moved = true
	if moved:
		apply_now()


## Push the applied state onto the pivots immediately.
func apply_now() -> void:
	for i in _applied.size():
		var j := joints[i] if i < joints.size() else null
		if j == null:
			continue
		var parent_basis: Basis = _parent_basis[i]
		var rad := _applied[i] * DEG
		# R_local = P^-1 * R_world * P (exact for any parent, incl. mirrored)
		var r_local := parent_basis.inverse() * Basis(_world_axis[i], rad) * parent_basis
		var t := j.transform
		t.basis = r_local * _rest_basis[i]
		j.transform = t
	_apply_fingers(_opening_applied)


## Set the command and apply it within this same frame (no smoothing).
func apply_command_now() -> void:
	for i in _command.size():
		_applied[i] = _command[i]
	_opening_applied = opening
	apply_now()


# ---------------------------------------------------------------- queries


func tcp_world() -> Vector3:
	return tcp.global_transform.origin if tcp != null else Vector3.ZERO


## Replay the map's self-test poses and compare with the generated expectation.
## Returns { "max_error": m, "worst": i, "tolerance": m, "passes": bool }.
func self_test() -> Dictionary:
	var st: Dictionary = map.get("self_test", {})
	var poses: Array = st.get("poses_deg", [])
	var expected: Array = st.get("tcp_blender", [])
	var tolerance: float = st.get("tolerance_m", 0.001)
	var saved := _applied.duplicate()
	var max_error := 0.0
	var worst := -1
	for k in poses.size():
		set_joints(poses[k])
		apply_command_now()
		var got := tcp_world()
		var e: Array = expected[k]
		var exp := convert_blender(Vector3(e[0], e[1], e[2]))
		var err := (got - exp).length()
		if err > max_error:
			max_error = err
			worst = k
	for i in saved.size():
		_applied[i] = saved[i]
	apply_now()
	return {"max_error": max_error, "worst": worst, "tolerance": tolerance, "passes": max_error <= tolerance}


# ---------------------------------------------------------------- internals


func _apply_fingers(op: float) -> void:
	var half := finger_half(op)
	_apply_finger(0, finger_l, half, -1.0)
	_apply_finger(1, finger_r, half, 1.0)


func _apply_finger(slot: int, node: Node3D, half: float, sign: float) -> void:
	if node == null:
		return
	var base: Vector3 = _finger_base[slot]
	node.position = base + _finger_axis * (sign * (half - _finger_half_at_import))


func _cache_fingers() -> void:
	_finger_base = [Vector3.ZERO, Vector3.ZERO]
	if finger_l != null:
		_finger_base[0] = finger_l.position
	if finger_r != null:
		_finger_base[1] = finger_r.position
	var sep: Vector3 = _finger_base[1] - _finger_base[0]
	if sep.length() > 1e-7:
		_finger_axis = sep.normalized()
	else:
		var a: Array = map.get("gripper", {}).get("unity_travel_axis", [0, 0, 1])
		_finger_axis = Vector3(a[0], a[1], a[2]).normalized()
	_finger_half_at_import = finger_half(opening_max())


func finger_half(op: float) -> float:
	return maxf(0.0, op) * 0.5 + float(map.get("gripper", {}).get("pad_face_offset", 0.024))


func opening_min() -> float:
	return float(map.get("gripper", {}).get("opening_min", 0.002))


func opening_max() -> float:
	return float(map.get("gripper", {}).get("opening_max", 0.074))


func _zeros() -> Array:
	var z: Array = []
	for i in _applied.size():
		z.append(0.0)
	return z


func limit_min(i: int) -> float:
	return float(map["joints"][i].get("limit_min_deg", -180.0))


func limit_max(i: int) -> float:
	return float(map["joints"][i].get("limit_max_deg", 180.0))


func _move_towards(cur: float, tgt: float, max_delta: float) -> float:
	var d := tgt - cur
	if absf(d) <= max_delta:
		return tgt
	return cur + signf(d) * max_delta
