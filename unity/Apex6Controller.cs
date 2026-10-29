// APEX-6 -- Unity FK controller
// Drives the six named joint transforms that the FBX/GLB already expose
// (CTRL_J1_Waist .. CTRL_J6_ToolRoll) plus the parallel-jaw gripper.
//
// The joints are pure transform pivots (no Armature, no Animator), so this is
// all that is needed to control the arm: set a joint angle in degrees and the
// child meshes follow.
//
// Axis note: Apex6JointMap.JointAxes holds each joint's rotation axis in its
// PARENT frame, already converted from Blender (Z-up) to Unity (Y-up) space.
// Run Apex6SelfTest to confirm the mapping on your import.

using UnityEngine;

namespace Apex6
{
    [AddComponentMenu("APEX-6/Apex-6 Controller")]
    public class Apex6Controller : MonoBehaviour
    {
        [Header("Rig (auto-found under Root when left empty)")]
        public Transform root;

        [Tooltip("Auto-find the joint / gripper transforms by name under Root.")]
        public bool autoFindRig = true;

        [Header("Joints (fill manually if autoFindRig is off)")]
        public Transform[] joints = new Transform[Apex6JointMap.JointCount];

        [Header("Tool")]
        public Transform gripperRoot;
        public Transform fingerL;
        public Transform fingerR;
        public Transform tcp;

        [Header("Motion")]
        [Tooltip("1 = full rated joint speed; 0 = jump instantly to the command.")]
        [Range(0f, 1f)] public float speedScale = 1f;

        [Tooltip("Temporarily amplify the rated speed used for smoothing.")]
        public float speedMultiplier = 1f;

        // commanded angles (degrees), clamped to the joint limits
        readonly float[] _command = new float[Apex6JointMap.JointCount];
        // angles actually applied (rate-limited toward _command when smoothing)
        readonly float[] _applied = new float[Apex6JointMap.JointCount];

        // per-joint rest state captured from the import (see Prepare): the
        // local rotation the importer gave the pivot, and the joint axis
        // expressed in that pivot's PARENT frame.
        readonly Quaternion[] _restRot = new Quaternion[Apex6JointMap.JointCount];
        readonly Vector3[] _axisLocal = new Vector3[Apex6JointMap.JointCount];

        Vector3[] _fingerBase;
        Vector3 _fingerAxis = Vector3.up;
        int[] _fingerSign = new int[2];
        float _opening = Apex6JointMap.OpeningMax;
        float _openingApplied = Apex6JointMap.OpeningMax;
        float _fingerHalfAtImport;

        // solved Blender->Unity conversion (see CalibrateAxes)
        double[,] _conv = new double[,] { { 1, 0, 0 }, { 0, 0, 1 }, { 0, -1, 0 } };
        bool _calibrated;

        /// <summary>Max rest-position error of the solved conversion (metres).</summary>
        public float ConversionResidual { get; private set; }
        /// <summary>Human-readable description of the solved conversion.</summary>
        public string ConversionNote { get; private set; }

        public bool Ready { get; private set; }

        // ---- lifecycle -----------------------------------------------------

        void Awake()
        {
            Prepare();
        }

        /// <summary>Find the rig, capture the imported finger rest positions and
        /// apply the zero pose.  <see cref="Awake"/> calls this, but Awake does
        /// not run in the editor, so anything driving the arm outside Play mode
        /// (the self test, an editor tool) must call it first.</summary>
        public void Prepare()
        {
            if (autoFindRig || root != null) FindRig();
            CalibrateAxes();
            _fingerHalfAtImport = Apex6JointMap.FingerHalf(Apex6JointMap.OpeningMax);
            CacheFingerBase();
            for (int i = 0; i < _command.Length; i++)
            {
                _command[i] = 0f;
                _applied[i] = 0f;
            }
            _opening = Apex6JointMap.OpeningMax;
            _openingApplied = Apex6JointMap.OpeningMax;
            ApplyNow();
        }

        /// <summary>
        /// Work out, from the rig as it was imported, (a) each pivot's rest
        /// local rotation, (b) the Blender->Unity conversion Unity actually
        /// applied, and (c) each joint axis expressed in the parent's frame.
        ///
        /// Why this is needed: Unity bakes the Z-up -> Y-up conversion in
        /// different places and with different handedness depending on the
        /// file.  The FBX arrives with the conversion inside
        /// <c>CTRL_J1_Waist.localRotation</c>, child locals still in Blender
        /// coordinates, and a <em>mirrored</em> frame (Unity is left-handed:
        /// measured map is (x,y,z) -> (-x, z, -y)).  The glTF arrives fully
        /// pre-converted with identity rest rotations and the proper map
        /// (x, z, -y).  Writing <c>localRotation = AngleAxis(deg, axis)</c>
        /// outright flattens the arm in the first case and picks the wrong axis
        /// in the second.
        ///
        /// So we do not assume the conversion -- we solve it from the known
        /// Blender rest positions of the joints, tool and fingers (carried in
        /// the generated map), then express each joint's world axis in the
        /// captured parent frame and pre-multiply, preserving the rest rotation.
        /// </summary>
        void CalibrateAxes()
        {
            if (_calibrated) return;
            _calibrated = true;
            var names = Apex6JointMap.RestPointNames;
            var bl = Apex6JointMap.RestPointsBlender;
            double[,] A = new double[3, 3];
            double[,] B = new double[3, 3];
            Transform scope = root != null ? root : transform;
            int used = 0;

            for (int k = 0; k < names.Length; k++)
            {
                Transform t = FindDeep(scope, names[k]);
                if (t == null) continue;
                Vector3 pb = bl[k];
                Vector3 pu = t.position;
                double[] b = { pb.x, pb.y, pb.z };
                double[] u = { pu.x, pu.y, pu.z };
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                    {
                        A[i, j] += b[i] * b[j];
                        B[i, j] += u[i] * b[j];
                    }
                used++;
            }

            double[,] ainv = Invert3(A);
            double[,] C;
            if (used < 4 || ainv == null)
            {
                // degenerate rig: fall back to the documented glTF conversion
                C = new double[,] { { 1, 0, 0 }, { 0, 0, 1 }, { 0, -1, 0 } };
                ConversionNote = "fallback (x,z,-y); rig not measurable";
            }
            else
            {
                C = Mat3Mul(B, ainv);
            }

            _conv = C;
            double det = Det3(C);
            ConversionNote = string.Format(
                "C=[({0:F4},{1:F4},{2:F4})({3:F4},{4:F4},{5:F4})({6:F4},{7:F4},{8:F4})] det={9:F4}{10}",
                C[0, 0], C[0, 1], C[0, 2],
                C[1, 0], C[1, 1], C[1, 2],
                C[2, 0], C[2, 1], C[2, 2], det,
                det < 0 ? "  (mirrored frame -> joint senses inverted)" : "");

            // residual of the rest-position fit -> an honest quality number
            ConversionResidual = 0f;
            for (int k = 0; k < names.Length; k++)
            {
                Transform t = FindDeep(scope, names[k]);
                if (t == null) continue;
                Vector3 pred = ConvertBlender(bl[k]);
                float err = (pred - t.position).magnitude;
                if (err > ConversionResidual) ConversionResidual = err;
            }

            for (int i = 0; i < Apex6JointMap.JointCount; i++)
            {
                Transform j = (joints != null && i < joints.Length) ? joints[i] : null;
                Vector3 worldAxis = ConvertBlender(Apex6JointMap.JointAxesBlender[i]);
                // A mirrored import maps Blender (x,y,z) -> (-x, z, -y); because
                // that map is improper it reverses each joint's sense, which is
                // exactly a negation of the axis (Rot(a,t) == Rot(-a,-t)).
                if (det < 0) worldAxis = -worldAxis;
                Quaternion parentRot = (j != null && j.parent != null)
                    ? j.parent.rotation : Quaternion.identity;
                _axisLocal[i] = Quaternion.Inverse(parentRot) * worldAxis;
                _restRot[i] = (j != null) ? j.localRotation : Quaternion.identity;
            }
        }

        /// <summary>Map a Blender (Z-up) point into the imported rig's frame.</summary>
        public Vector3 ConvertBlender(Vector3 p)
        {
            return new Vector3(
                (float)(_conv[0, 0] * p.x + _conv[0, 1] * p.y + _conv[0, 2] * p.z),
                (float)(_conv[1, 0] * p.x + _conv[1, 1] * p.y + _conv[1, 2] * p.z),
                (float)(_conv[2, 0] * p.x + _conv[2, 1] * p.y + _conv[2, 2] * p.z));
        }

        static double[,] Mat3Mul(double[,] A, double[,] B)
        {
            var C = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    double s = 0;
                    for (int k = 0; k < 3; k++) s += A[i, k] * B[k, j];
                    C[i, j] = s;
                }
            return C;
        }

        static double Det3(double[,] m)
        {
            return m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
                 - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
                 + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
        }

        static double[,] Invert3(double[,] m)
        {
            double det = Det3(m);
            if (System.Math.Abs(det) < 1e-12) return null;
            double id = 1.0 / det;
            var r = new double[3, 3];
            r[0, 0] = (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) * id;
            r[0, 1] = (m[0, 2] * m[2, 1] - m[0, 1] * m[2, 2]) * id;
            r[0, 2] = (m[0, 1] * m[1, 2] - m[0, 2] * m[1, 1]) * id;
            r[1, 0] = (m[1, 2] * m[2, 0] - m[1, 0] * m[2, 2]) * id;
            r[1, 1] = (m[0, 0] * m[2, 2] - m[0, 2] * m[2, 0]) * id;
            r[1, 2] = (m[0, 2] * m[1, 0] - m[0, 0] * m[1, 2]) * id;
            r[2, 0] = (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]) * id;
            r[2, 1] = (m[0, 1] * m[2, 0] - m[0, 0] * m[2, 1]) * id;
            r[2, 2] = (m[0, 0] * m[1, 1] - m[0, 1] * m[1, 0]) * id;
            return r;
        }

        void LateUpdate()
        {
            float dt = Time.deltaTime;
            bool moved = false;
            for (int i = 0; i < _applied.Length; i++)
            {
                float target = speedScale <= 0f
                    ? _command[i]
                    : Mathf.MoveTowards(_applied[i], _command[i],
                        Apex6JointMap.MaxSpeedDeg[i] * speedScale * speedMultiplier * dt);
                if (!Mathf.Approximately(target, _applied[i])) { _applied[i] = target; moved = true; }
            }
            if (speedScale > 0f && !Mathf.Approximately(_openingApplied, _opening))
            {
                // 0.15 m/s finger speed is plenty for a 74 mm travel
                _openingApplied = Mathf.MoveTowards(_openingApplied, _opening, 0.15f * dt);
                moved = true;
            }
            if (moved) ApplyNow();
        }

        // ---- rig discovery -------------------------------------------------

        [ContextMenu("Find Rig")]
        public void FindRig()
        {
            Transform scope = root != null ? root : transform;
            joints = new Transform[Apex6JointMap.JointCount];
            for (int i = 0; i < Apex6JointMap.JointCount; i++)
                joints[i] = FindDeep(scope, Apex6JointMap.JointNames[i]);
            gripperRoot = FindDeep(scope, Apex6JointMap.GripperRoot);
            fingerL = FindDeep(scope, Apex6JointMap.FingerL);
            fingerR = FindDeep(scope, Apex6JointMap.FingerR);
            tcp = FindDeep(scope, Apex6JointMap.TcpName);
            Ready = joints[0] != null && joints[5] != null;
        }

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

        void CacheFingerBase()
        {
            _fingerBase = new Vector3[2];
            if (fingerL != null) _fingerBase[0] = fingerL.localPosition;
            if (fingerR != null) _fingerBase[1] = fingerR.localPosition;

            // The travel axis must be read from the rig, not assumed: the FBX
            // puts it on one local axis and the glTF on another. The two fingers
            // are separated along it, so take the direction between their rest
            // positions and give each finger the sign that pushes it away.
            Vector3 sep = _fingerBase[1] - _fingerBase[0];
            if (sep.sqrMagnitude > 1e-12f)
            {
                _fingerAxis = sep.normalized;
                _fingerSign[0] = -1;   // L slides against the separation
                _fingerSign[1] = 1;    // R slides along it
            }
            else
            {
                _fingerAxis = Apex6JointMap.FingerTravelAxis;
                _fingerSign[0] = -1;
                _fingerSign[1] = 1;
            }
        }

        // ---- public control API -------------------------------------------

        /// <summary>Command a joint (0..5) to an angle in degrees (clamped).</summary>
        public void SetJoint(int index, float degrees)
        {
            if (index < 0 || index >= _command.Length) return;
            _command[index] = Apex6JointMap.Clamp(index, degrees);
        }

        /// <summary>Command all six joints (degrees); null entries are ignored.</summary>
        public void SetJoints(float[] degrees)
        {
            if (degrees == null) return;
            for (int i = 0; i < _command.Length && i < degrees.Length; i++)
                if (!float.IsNaN(degrees[i])) SetJoint(i, degrees[i]);
        }

        /// <summary>Command a joint in radians (handy for URDF/ROS parity).</summary>
        public void SetJointRadians(int index, float radians) =>
            SetJoint(index, radians * Mathf.Rad2Deg);

        public float GetJoint(int index) =>
            (index >= 0 && index < _command.Length) ? _command[index] : 0f;

        public float[] GetJoints() => (float[])_command.Clone();
        public float[] GetAppliedJoints() => (float[])_applied.Clone();

        /// <summary>Move smoothy to the configured home pose.</summary>
        public void GoHome() => SetJoints(Apex6JointMap.HomeDeg);

        /// <summary>Return every joint to zero.</summary>
        public void GoZero() => SetJoints(new float[Apex6JointMap.JointCount]);

        /// <summary>Set the pad-face gap in meters (clamped to the stroke).</summary>
        public void SetGripper(float openingMeters)
        {
            _opening = Mathf.Clamp(openingMeters,
                Apex6JointMap.OpeningMin, Apex6JointMap.OpeningMax);
        }

        public void OpenGripper() => SetGripper(Apex6JointMap.OpeningMax);
        public void CloseGripper() => SetGripper(Apex6JointMap.OpeningMin);
        public float GetGripper() => _opening;

        /// <summary>True when every joint has reached its commanded angle.</summary>
        public bool AtTarget()
        {
            for (int i = 0; i < _applied.Length; i++)
                if (Mathf.Abs(_applied[i] - _command[i]) > 0.01f) return false;
            return Mathf.Abs(_openingApplied - _opening) < 1e-4f;
        }

        /// <summary>Apply the current angles to the transforms immediately.</summary>
        public void ApplyNow()
        {
            for (int i = 0; i < _applied.Length; i++)
            {
                if (joints == null || i >= joints.Length || joints[i] == null) continue;
                // pre-multiply: rotate about the joint axis expressed in the
                // parent frame while keeping the importer's rest rotation
                joints[i].localRotation =
                    Quaternion.AngleAxis(_applied[i], _axisLocal[i]) * _restRot[i];
            }
            ApplyFingers(_openingApplied);
        }

        /// <summary>Push the commanded pose onto the transforms within this same
        /// frame, bypassing the rate limiter in LateUpdate. Solvers (and the self
        /// test) need the transforms to move immediately; plain <see cref="ApplyNow"/>
        /// only re-applies the rate-limited state and would appear to do nothing.</summary>
        public void ApplyCommandNow()
        {
            System.Array.Copy(_command, _applied, _applied.Length);
            _openingApplied = _opening;
            ApplyNow();
        }

        void ApplyFingers(float opening)
        {
            float half = Apex6JointMap.FingerHalf(opening);
            ApplyFinger(0, fingerL, half);
            ApplyFinger(1, fingerR, half);
        }

        void ApplyFinger(int slot, Transform finger, float half)
        {
            if (finger == null) return;
            Vector3 basePos = _fingerBase[slot];
            // slide outward/inward along the (import-derived) travel axis
            finger.localPosition = basePos +
                _fingerAxis * (_fingerSign[slot] * (half - _fingerHalfAtImport));
        }

        /// <summary>World pose of the tool centre point.</summary>
        public Vector3 TcpPosition => tcp != null ? tcp.position : transform.position;
        public Quaternion TcpRotation => tcp != null ? tcp.rotation : transform.rotation;

        /// <summary>Joint axis as solved for this import, in the parent frame.</summary>
        public Vector3 AxisLocal(int i) =>
            (i >= 0 && i < _axisLocal.Length) ? _axisLocal[i] : Vector3.zero;
    }
}
