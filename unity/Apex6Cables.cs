// APEX-6 -- runtime cable routing for the exported rig
//
// WHY THIS EXISTS
// ---------------
// The flexible cable hoses are *animation-driven* geometry in Blender, not
// physics: every pose, robot_geometry.update_cables() rebuilds each cable's
// spline from the live world transforms of its two attachment links, so the
// hoses always stay connected. An interchange export (FBX/GLB) cannot carry
// that -- it bakes the tubes as static meshes onto the rig root at the export
// pose -- so in Unity the hoses stay put while the arm moves.
//
// This component restores the Blender behaviour at runtime: each frame it
// rebuilds every `Cable_*` tube from the live attachment points, using the same
// spine the Blender builder uses
//
//     p(t) = pa + (pb - pa)*t + away * slack*sin(pi*t)
//     slack = slackBase * min(1, 0.55 + 0.6*|pb - pa|)          (update_cables)
//
// Nothing is hand-copied and nothing is guessed:
//
//   * the attachment points come from the `*_StrainRelief` / `*_Clamp` objects,
//     which ARE joint-parented meshes and therefore already follow the arm;
//   * the tube radius, base slack and bow direction come from the generated
//     map (Apex6JointMap.CableRadius / CableSlack / CableBowBlender), which
//     scripts/tools/export_unity.py generates from
//     robot_geometry.CABLE_ROUTES -- the same table Blender routes from;
//   * the bow is a Blender (Z-up, root-frame) direction -- at Q_ZERO every
//     joint empty has an identity rotation, so a link-frame direction is also a
//     root-frame direction -- and Apex6Controller.ConvertBlender() maps it to
//     the imported rig's frame using the conversion the controller solved from
//     the rest positions. That keeps the loop on the correct side for both the
//     FBX (mirrored import) and the glTF (pre-converted import).
//
// No Read/Write access to the baked meshes is required: we only read the
// attachment transforms, then replace each tube's MeshFilter mesh with our own.
//
// The clamps / strain-reliefs are left alone. Disable `routing` to freeze the
// cables, or the whole component to fall back to the baked meshes.

using System.Collections.Generic;
using UnityEngine;

namespace Apex6
{
    [AddComponentMenu("APEX-6/Apex-6 Cables")]
    [RequireComponent(typeof(Apex6Controller))]
    [DefaultExecutionOrder(100)]   // must rebuild after Apex6Controller's LateUpdate
    public class Apex6Cables : MonoBehaviour
    {
        [Header("Wiring")]
        public Apex6Controller robot;
        [Tooltip("Rig root. Auto-filled from the controller / this transform.")]
        public Transform root;

        [Header("Routing")]
        [Tooltip("Rebuild the cable tubes every frame (off = keep the baked meshes).")]
        public bool routing = true;

        [Tooltip("Rings along each cable (higher = smoother).")]
        [Range(4, 64)] public int spineSamples = 24;

        [Tooltip("Vertices around each cable.")]
        [Range(5, 24)] public int ringSegments = 14;

        [Tooltip("Print the per-cable calibration once on Start.")]
        public bool logCalibration = false;

        class Cable
        {
            public string name;
            public MeshFilter filter;
            public Mesh mesh;
            public Transform linkA;      // parent of the strain relief
            public Transform linkB;      // parent of the clamp
            public Vector3 paLocal;      // attachment point in link A's frame
            public Vector3 pbLocal;      // attachment point in link B's frame
            public Vector3 awayLocalA;   // bow direction in link A's frame
            public float slackBase;      // route slack (before the length factor)
            public float radius;
            public bool ok;
        }

        readonly List<Cable> _cables = new List<Cable>();
        readonly List<Vector3> _verts = new List<Vector3>();
        readonly List<Vector3> _norms = new List<Vector3>();
        int[] _tris;
        bool _ready;

        void Reset()
        {
            robot = GetComponent<Apex6Controller>();
            root = transform;
        }

        void Start()
        {
            if (robot == null) robot = GetComponent<Apex6Controller>();
            if (root == null) root = (robot != null && robot.root != null) ? robot.root : transform;
            if (robot != null) robot.Prepare();     // finds the rig, leaves it at the zero pose
            Calibrate(root);
            _ready = true;
        }

        // ------------------------------------------------------------------
        //  calibration -- all of it is measured against the imported rig, so
        //  it must run while the arm is at its zero pose (Start, right after
        //  robot.Prepare()).
        // ------------------------------------------------------------------
        void Calibrate(Transform scope)
        {
            _cables.Clear();
            if (robot == null || scope == null) return;

            var names = Apex6JointMap.CableNames;
            for (int i = 0; i < names.Length; i++)
            {
                string name = names[i];
                var c = new Cable { name = name };
                if (i < Apex6JointMap.CableRadius.Length) c.radius = Apex6JointMap.CableRadius[i];
                if (i < Apex6JointMap.CableSlack.Length) c.slackBase = Apex6JointMap.CableSlack[i];

                c.filter = FindComponent<MeshFilter>(scope, name);
                Transform relief = FindDeep(scope, name + "_StrainRelief");
                Transform clamp = FindDeep(scope, name + "_Clamp");

                if (c.filter == null || clamp == null || relief == null)
                {
                    Debug.LogWarning("[Apex6Cables] " + name +
                        ": missing tube / strain relief / clamp -- leaving the baked mesh in place.");
                    _cables.Add(c);           // ok == false -> skipped every frame
                    continue;
                }

                c.linkA = relief.parent != null ? relief.parent : scope;
                c.linkB = clamp.parent != null ? clamp.parent : scope;

                c.paLocal = c.linkA.InverseTransformPoint(relief.position);
                c.pbLocal = c.linkB.InverseTransformPoint(clamp.position);

                // bow: Blender root-frame direction -> this import's world frame
                // -> link A's local frame (so it rotates with the link).
                Vector3 bowBlender = (i < Apex6JointMap.CableBowBlender.Length)
                    ? Apex6JointMap.CableBowBlender[i]
                    : new Vector3(-1f, 0f, 0f);
                Vector3 awayWorld = robot.ConvertBlender(bowBlender);
                Vector3 awayLocal = Quaternion.Inverse(c.linkA.rotation) * awayWorld;
                if (awayLocal.sqrMagnitude < 1e-8f) awayLocal = Vector3.up;
                c.awayLocalA = awayLocal.normalized;

                c.mesh = new Mesh();
                c.mesh.MarkDynamic();
                // Assign via sharedMesh: MeshFilter.mesh's getter instantiates a
                // copy in the Editor, which would freeze the tube at the pose it
                // was first read (the classic "cables don't follow" trap).
                c.filter.sharedMesh = c.mesh;
                c.ok = true;

                if (logCalibration)
                    Debug.Log(string.Format(
                        "[Apex6Cables] {0}: r={1:F4} m  slack(base)={2:F4} m  {3} -> {4}  awayLocal=({5:F3},{6:F3},{7:F3})",
                        name, c.radius, c.slackBase, c.linkA.name, c.linkB.name,
                        c.awayLocalA.x, c.awayLocalA.y, c.awayLocalA.z));

                _cables.Add(c);
            }

            EnsureTopology();
        }

        // ------------------------------------------------------------------
        //  topology -- rings x segments, plus two cap centres. Fixed for the
        //  lifetime, so the index buffer is built once.
        // ------------------------------------------------------------------
        void EnsureTopology()
        {
            int rings = Mathf.Max(3, spineSamples) + 1;
            int seg = Mathf.Max(4, ringSegments);
            var tris = new List<int>(rings * seg * 6 + seg * 6);

            for (int i = 0; i < rings - 1; i++)
            {
                for (int s = 0; s < seg; s++)
                {
                    int s1 = (s + 1) % seg;
                    int a = i * seg + s;
                    int b = i * seg + s1;
                    int c = (i + 1) * seg + s1;
                    int d = (i + 1) * seg + s;
                    tris.Add(a); tris.Add(b); tris.Add(c);
                    tris.Add(a); tris.Add(c); tris.Add(d);
                }
            }
            int capA = rings * seg;
            int capB = capA + 1;
            for (int s = 0; s < seg; s++)
            {
                int s1 = (s + 1) % seg;
                tris.Add(capA); tris.Add(s1); tris.Add(s);
                tris.Add(capB); tris.Add((rings - 1) * seg + s); tris.Add((rings - 1) * seg + s1);
            }
            _tris = tris.ToArray();
        }

        // ------------------------------------------------------------------
        //  per-frame rebuild
        // ------------------------------------------------------------------
        void LateUpdate()
        {
            if (!_ready || !routing) return;
            for (int i = 0; i < _cables.Count; i++)
            {
                var c = _cables[i];
                if (!c.ok) continue;

                Vector3 pa = c.linkA.TransformPoint(c.paLocal);
                Vector3 pb = c.linkB.TransformPoint(c.pbLocal);
                Vector3 d = pb - pa;
                float len = d.magnitude;
                if (len < 1e-4f) continue;                 // degenerate: keep last frame

                Vector3 u = d / len;
                Vector3 away = c.linkA.TransformDirection(c.awayLocalA);
                away -= u * Vector3.Dot(away, u);
                if (away.sqrMagnitude < 1e-10f)
                    away = Vector3.Cross(u, Vector3.right);
                if (away.sqrMagnitude < 1e-10f)
                    away = Vector3.Cross(u, Vector3.up);
                away = away.normalized;

                float slack = c.slackBase * Mathf.Min(1f, 0.55f + 0.6f * len);

                // The tube object carries the import's Z-up->Y-up rotation in its
                // own transform (measured: localEuler 270,0,0), so the rebuilt
                // mesh must be written in that object's LOCAL frame or it would
                // be rotated a second time at draw time.
                Transform tr = c.filter.transform;
                Vector3 paL = tr.InverseTransformPoint(pa);
                Vector3 pbL = tr.InverseTransformPoint(pb);
                Vector3 dL = pbL - paL;
                Vector3 uL = dL / len;
                Vector3 awayL = tr.InverseTransformDirection(away);

                BuildTube(c, paL, dL, uL, awayL, slack);
            }
        }

        void BuildTube(Cable c, Vector3 pa, Vector3 d, Vector3 u, Vector3 away, float slack)
        {
            int rings = Mathf.Max(3, spineSamples) + 1;
            int seg = Mathf.Max(4, ringSegments);
            float r = c.radius;
            float pi = Mathf.PI;

            _verts.Clear();
            _norms.Clear();
            Vector3 prevN1 = Vector3.zero;
            Vector3 tangentLast = u;

            for (int i = 0; i < rings; i++)
            {
                float t = (float)i / (rings - 1);
                Vector3 centre = pa + d * t + away * (slack * Mathf.Sin(pi * t));
                Vector3 tangent = d + away * (slack * pi * Mathf.Cos(pi * t));
                if (tangent.sqrMagnitude < 1e-12f) tangent = u;
                tangent = tangent.normalized;
                tangentLast = tangent;

                Vector3 n1;
                if (i == 0)
                    n1 = away - tangent * Vector3.Dot(away, tangent);
                else
                    n1 = prevN1 - tangent * Vector3.Dot(prevN1, tangent);   // parallel transport
                if (n1.sqrMagnitude < 1e-10f)
                    n1 = Vector3.Cross(tangent, Vector3.right);
                if (n1.sqrMagnitude < 1e-10f)
                    n1 = Vector3.Cross(tangent, Vector3.up);
                n1 = n1.normalized;
                Vector3 n2 = Vector3.Cross(tangent, n1);
                prevN1 = n1;

                for (int s = 0; s < seg; s++)
                {
                    float a = 2f * pi * s / seg;
                    Vector3 dir = n1 * Mathf.Cos(a) + n2 * Mathf.Sin(a);
                    _verts.Add(centre + dir * r);
                    _norms.Add(dir);
                }
            }

            // cap centres: pa (normal -u) and pb (normal +tangentLast)
            _verts.Add(pa);
            _norms.Add(-u);
            _verts.Add(pa + d);
            _norms.Add(tangentLast);

            c.mesh.Clear(false);
            c.mesh.SetVertices(_verts);
            c.mesh.SetNormals(_norms);
            c.mesh.SetTriangles(_tris, 0);
            c.mesh.RecalculateBounds();

            // self-heal: if anything (e.g. a MeshFilter.mesh read in the Editor)
            // swapped in an instance copy, point the filter back at our live mesh.
            if (c.filter.sharedMesh != c.mesh) c.filter.sharedMesh = c.mesh;
        }

        // ---- tiny hierarchy helpers (the controller's are private) --------
        static Transform FindDeep(Transform parent, string name)
        {
            if (parent == null) return null;
            if (parent.name == name) return parent;
            for (int i = 0; i < parent.childCount; i++)
            {
                var hit = FindDeep(parent.GetChild(i), name);
                if (hit != null) return hit;
            }
            return null;
        }

        static T FindComponent<T>(Transform scope, string name) where T : Component
        {
            Transform t = FindDeep(scope, name);
            return t != null ? t.GetComponent<T>() : null;
        }
    }
}
