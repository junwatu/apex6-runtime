@tool
class_name Apex6Cables
extends Node3D
## Rebuilds the flexible cable tubes every frame so they follow the arm.
##
## The `Cable_*` tubes are baked as static meshes at the export pose, so on
## their own they stay put while the arm moves. This rebuilds each one from the
## same spine Blender uses:
##
##     p(t) = pa + (pb - pa)*t + away * slack*sin(PI*t)
##     slack = slack_base * min(1, 0.55 + 0.6*|pb - pa|)
##
## Attachment points come from the joint-parented `*_StrainRelief` / `*_Clamp`
## objects; radius, slack and bow come from `apex6_joint_map.json` (generated
## from the same routes Blender animates from). The bow is mapped through the
## controller's solved conversion, so it is correct for any importer.
##
## Put this node in the scene, point `controller` at an Apex6Controller and
## `rig_root` at the same model root, then call `prepare()`.

## The controller that owns the rig (used for the solved conversion).
@export var controller: Apex6Controller
## The loaded model root. Defaults to the controller's `rig_root`.
@export var rig_root: Node3D
## Rebuild the tubes each frame. Turn off to leave them at the export pose.
@export var routing: bool = true
@export var spine_samples: int = 24
@export var ring_segments: int = 14

## One entry per route that was successfully prepared.
var cables: Array = []

var _ready_flag := false


func _ready() -> void:
	if Engine.is_editor_hint():
		return
	if controller != null and not _ready_flag:
		prepare()


func _process(delta: float) -> void:
	if routing:
		update_tubes()


func prepare() -> int:
	cables.clear()
	var root := rig_root
	if root == null and controller != null:
		root = controller.rig_root
	if root == null or controller == null:
		push_error("Apex6Cables: controller/rig_root not set")
		return 0

	var routes: Array = controller.map.get("cables", [])
	for route in routes:
		var name := String(route["name"])
		var mesh_node := _find_deep(root, name) as MeshInstance3D
		var relief := _find_deep(root, name + "_StrainRelief")
		var clamp_node := _find_deep(root, name + "_Clamp")
		var entry := {"route": route, "ok": false}
		if mesh_node == null or relief == null or clamp_node == null or mesh_node.mesh == null:
			cables.append(entry)
			continue

		entry["node"] = mesh_node
		var link_a: Node3D = relief.get_parent() as Node3D
		var link_b: Node3D = clamp_node.get_parent() as Node3D
		entry["link_a"] = link_a
		entry["link_b"] = link_b
		entry["pa_local"] = link_a.global_transform.affine_inverse() * relief.global_transform.origin
		entry["pb_local"] = link_b.global_transform.affine_inverse() * clamp_node.global_transform.origin

		var bow := Vector3(route["bow"][0], route["bow"][1], route["bow"][2])
		var away_world := controller.convert_blender(bow)
		var away_local := (link_a.global_transform.basis.inverse() * away_world).normalized()
		entry["away_local_a"] = away_local
		entry["radius"] = float(route["radius"])
		entry["slack_base"] = float(route["slack"])

		var rings := maxi(3, spine_samples) + 1
		var seg := maxi(4, ring_segments)
		entry["rings"] = rings
		entry["seg"] = seg
		entry["vert_count"] = rings * seg + 2
		entry["index"] = _make_index(rings, seg)

		var mesh := ArrayMesh.new()
		mesh_node.mesh = mesh
		entry["mesh"] = mesh
		entry["material"] = _surface_material(mesh_node)
		entry["verts"] = PackedVector3Array()
		entry["ok"] = true
		cables.append(entry)

	_ready_flag = true
	var n := 0
	for c in cables:
		if c["ok"]:
			n += 1
	return n


func update_tubes() -> void:
	if not _ready_flag or not routing:
		return
	for c in cables:
		if not c["ok"]:
			continue
		var node: Node3D = c["node"]
		var link_a: Node3D = c["link_a"]
		var link_b: Node3D = c["link_b"]
		var pa: Vector3 = link_a.global_transform * c["pa_local"]
		var pb: Vector3 = link_b.global_transform * c["pb_local"]
		var d := pb - pa
		var length := d.length()
		if length < 1e-4:
			continue
		var u := d / length

		var away: Vector3 = (link_a.global_transform.basis * (c["away_local_a"] as Vector3)).normalized()
		away = (away - u * away.dot(u))
		if away.length() < 1e-8:
			away = u.cross(Vector3(1, 0, 0))
		if away.length() < 1e-8:
			away = u.cross(Vector3(0, 1, 0))
		away = away.normalized()

		var slack: float = c["slack_base"] * minf(1.0, 0.55 + 0.6 * length)

		# build in the tube object's local frame
		var to_local := node.global_transform.affine_inverse()
		var pa_l: Vector3 = to_local * pa
		var pb_l: Vector3 = to_local * pb
		var d_l := pb_l - pa_l
		var u_l := d_l / maxf(length, 1e-9)
		var away_l: Vector3 = (node.global_transform.basis.inverse() * away).normalized()

		var verts := _build_tube(c, pa_l, d_l, u_l, away_l, slack)
		c["verts"] = verts
		_upload(c, verts)


func _build_tube(c: Dictionary, pa: Vector3, d: Vector3, u: Vector3, away: Vector3, slack: float) -> PackedVector3Array:
	var rings: int = c["rings"]
	var seg: int = c["seg"]
	var r: float = c["radius"]
	var verts := PackedVector3Array()
	verts.resize(c["vert_count"])
	var o := 0
	var prev_n1 := Vector3.ZERO
	var tangent_last := u

	for i in rings:
		var t := float(i) / float(rings - 1)
		var centre := pa + d * t + away * (slack * sin(PI * t))
		var tangent := d + away * (slack * PI * cos(PI * t))
		if tangent.length() < 1e-8:
			tangent = u
		tangent = tangent.normalized()
		tangent_last = tangent

		var n1: Vector3
		if i == 0:
			n1 = away - tangent * away.dot(tangent)
		else:
			n1 = prev_n1 - tangent * prev_n1.dot(tangent)
		if n1.length() < 1e-8:
			n1 = tangent.cross(Vector3(1, 0, 0))
		if n1.length() < 1e-8:
			n1 = tangent.cross(Vector3(0, 1, 0))
		n1 = n1.normalized()
		var n2 := tangent.cross(n1)
		prev_n1 = n1

		for s in seg:
			var a := TAU * float(s) / float(seg)
			var dir := (n1 * cos(a) + n2 * sin(a))
			verts[o] = centre + dir * r
			o += 1

	verts[o] = pa
	o += 1
	verts[o] = pa + d
	return verts


func _make_index(rings: int, seg: int) -> PackedInt32Array:
	var index := PackedInt32Array()
	for i in rings - 1:
		for s in seg:
			var s1 := (s + 1) % seg
			var a := i * seg + s
			var b := i * seg + s1
			var cc := (i + 1) * seg + s1
			var dd := (i + 1) * seg + s
			index.append_array([a, b, cc, a, cc, dd])
	var cap_a := rings * seg
	var cap_b := cap_a + 1
	for s in seg:
		var s1 := (s + 1) % seg
		index.append_array([cap_a, s1, s])
		index.append_array([cap_b, (rings - 1) * seg + s, (rings - 1) * seg + s1])
	return index


func _upload(c: Dictionary, verts: PackedVector3Array) -> void:
	var mesh: ArrayMesh = c["mesh"]
	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_INDEX] = c["index"]
	mesh.clear_surfaces()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	if c["material"] != null:
		mesh.surface_set_material(0, c["material"])


func _surface_material(mi: MeshInstance3D) -> Material:
	var mat := mi.get_active_material(0)
	if mat == null and mi.mesh != null and mi.mesh.get_surface_count() > 0:
		mat = mi.mesh.surface_get_material(0)
	return mat


func _find_deep(root: Node, target: String) -> Node:
	if target == "" or root == null:
		return null
	if String(root.name) == target:
		return root
	for child in root.get_children():
		var hit := _find_deep(child, target)
		if hit != null:
			return hit
	return null
