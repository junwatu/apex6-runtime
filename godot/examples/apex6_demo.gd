extends Node3D
## Minimal APEX-6 demo for Godot 4.
##
## Attach this to a Node3D in an empty scene, copy `apex6_robot.glb` and
## `apex6_joint_map.json` into the project (e.g. at the root), press Play. It
## loads the model, adds an Apex6Controller + Apex6Cables, builds a small
## control panel in code and drives the arm.
##
## This file is an example only — the reusable code is the `addons/apex6` add-on.

@export_file("*.glb") var model_file: String = "res://apex6_robot.glb"
@export_file("*.json") var map_file: String = "res://apex6_joint_map.json"
@export var smooth: bool = true

const NAMES := ["J1 waist", "J2 shoulder", "J3 elbow", "J4 roll", "J5 pitch", "J6 roll"]
const POSES := {
	"Zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	"Reach": [0.0, -55.0, 95.0, 0.0, 35.0, 0.0],
	"Fold": [0.0, 60.0, -115.0, 0.0, -110.0, 0.0],
	"Inspect": [35.0, -25.0, 80.0, 20.0, 60.0, 45.0],
	"Wide": [-70.0, -20.0, 40.0, 90.0, -30.0, 150.0],
}

var robot: Apex6Controller
var cables: Apex6Cables
var _sliders: Array = []
var _labels: Array = []
var _grip: HSlider
var _readout: Label


func _ready() -> void:
	if not ResourceLoader.exists(model_file):
		push_error("APEX-6 demo: copy the model to %s (or set model_file)" % model_file)
		return

	var model: Node3D = (load(model_file) as PackedScene).instantiate()
	add_child(model)

	robot = Apex6Controller.new()
	robot.rig_root = model
	robot.map_path = map_file
	add_child(robot)
	if not robot.prepare():
		push_error("APEX-6 demo: controller could not prepare")
		return

	cables = Apex6Cables.new()
	cables.controller = robot
	cables.rig_root = model
	cables.spine_samples = 24
	cables.ring_segments = 14
	add_child(cables)
	cables.prepare()

	_setup_view()
	_build_ui()
	robot.apply_command_now()
	cables.update_tubes()


func _process(delta: float) -> void:
	if robot == null or not robot.ready_flag:
		return
	robot.update(delta, 1.0 if smooth else 0.0)
	cables.update_tubes()

	for i in _sliders.size():
		var s: HSlider = _sliders[i]
		if not s.has_focus():
			var v := robot.get_joint(i)
			s.value = v
			_labels[i].text = "%.1f" % v
	if _grip != null and not _grip.has_focus():
		_grip.value = robot.gripper() * 1000.0

	if _readout != null:
		var t := robot.tcp_world()
		_readout.text = "TCP  %+.3f  %+.3f  %+.3f m      gap  %.1f mm" % [
			t.x, t.y, t.z, robot.gripper() * 1000.0]


func _setup_view() -> void:
	var cam := Camera3D.new()
	cam.position = Vector3(2.6, 1.7, 2.9)
	add_child(cam)
	cam.look_at(Vector3(0.35, 0.85, 0.0))

	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-52.0, 38.0, 0.0)
	light.light_energy = 1.4
	add_child(light)

	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = Color(0.06, 0.08, 0.10)
	e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	e.ambient_light_color = Color(0.5, 0.6, 0.75)
	e.ambient_light_energy = 0.6
	env.environment = e
	add_child(env)

	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(12, 12)
	ground.mesh = plane
	var gm := StandardMaterial3D.new()
	gm.albedo_color = Color(0.10, 0.12, 0.15)
	gm.roughness = 1.0
	ground.material_override = gm
	add_child(ground)


func _build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	var panel := PanelContainer.new()
	panel.position = Vector2(12, 12)
	panel.custom_minimum_size = Vector2(300, 0)
	layer.add_child(panel)

	var vb := VBoxContainer.new()
	panel.add_child(vb)
	vb.add_child(_title("APEX-6 · godot runtime"))
	var st := robot.self_test()
	var info := _title("6 pivots · FK %.3f mm %s" % [float(st["max_error"]) * 1000.0, "OK" if st["passes"] else "FAIL"])
	info.add_theme_color_override("font_color", Color("7ee787") if st["passes"] else Color("ff7b72"))
	vb.add_child(info)

	for i in robot.joint_count():
		var row := HBoxContainer.new()
		vb.add_child(row)
		var name := Label.new()
		name.text = NAMES[i].split(" ")[0]
		name.custom_minimum_size.x = 34
		row.add_child(name)
		var s := HSlider.new()
		s.min_value = robot.limit_min(i)
		s.max_value = robot.limit_max(i)
		s.step = 0.5
		s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		s.custom_minimum_size.x = 170
		s.value_changed.connect(func(v: float): robot.set_joint(i, v))
		row.add_child(s)
		var lab := Label.new()
		lab.custom_minimum_size.x = 46
		lab.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		row.add_child(lab)
		_sliders.append(s)
		_labels.append(lab)

	var grow := HBoxContainer.new()
	vb.add_child(grow)
	grow.add_child(_title("gripper"))
	_grip = HSlider.new()
	_grip.min_value = robot.opening_min() * 1000.0
	_grip.max_value = robot.opening_max() * 1000.0
	_grip.step = 1.0
	_grip.value = robot.gripper() * 1000.0
	_grip.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_grip.value_changed.connect(func(v: float): robot.set_gripper(v / 1000.0))
	grow.add_child(_grip)

	var presets := HBoxContainer.new()
	vb.add_child(presets)
	for pose_name in POSES:
		var b := Button.new()
		b.text = pose_name
		b.pressed.connect(func(): robot.set_joints(POSES[pose_name]))
		presets.add_child(b)

	var toggles := HBoxContainer.new()
	vb.add_child(toggles)
	var smooth_btn := CheckButton.new()
	smooth_btn.text = "smooth"
	smooth_btn.button_pressed = smooth
	smooth_btn.toggled.connect(func(on: bool): smooth = on)
	toggles.add_child(smooth_btn)
	var cable_btn := CheckButton.new()
	cable_btn.text = "cables"
	cable_btn.button_pressed = true
	cable_btn.toggled.connect(func(on: bool): cables.routing = on; cables.update_tubes())
	toggles.add_child(cable_btn)

	_readout = _title("")
	vb.add_child(_readout)


func _title(text: String) -> Label:
	var l := Label.new()
	l.text = text
	return l
