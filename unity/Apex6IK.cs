// APEX-6 -- Unity IK solver (cyclic coordinate descent)
//
// A small, dependency-free IK solver that drives the Apex6Controller's joints so
// the tool centre point (CTRL_TCP) reaches a target Transform. Joint limits are
// respected, so it will not fold the arm past its measured travel.
//
// It is a *kinematic* solver (position only, no physics). For torque-accurate
// behaviour use the URDF route with ArticulationBody instead.

using UnityEngine;

namespace Apex6
{
    [AddComponentMenu("APEX-6/Apex-6 IK Solver")]
    [RequireComponent(typeof(Apex6Controller))]
    public class Apex6IK : MonoBehaviour
    {
        public Apex6Controller robot;

        [Tooltip("The TCP is driven toward this target each frame.")]
        public Transform target;

        public bool solveEveryFrame = true;

        [Tooltip("Stop when the TCP is this close to the target (meters).")]
        public float positionTolerance = 0.002f;

        [Range(1, 100)] public int maxIterations = 24;

        [Tooltip("Fraction of each computed correction to apply (damping).")]
        [Range(0.1f, 1f)] public float damping = 0.8f;

        [Tooltip("Optional second target this frame: keep the tool axis pointing here.")]
        public Transform aimAt;

        void Reset() { robot = GetComponent<Apex6Controller>(); }

        void LateUpdate()
        {
            if (solveEveryFrame && target != null) Solve();
        }

        /// <summary>Run the solver once. Returns true when within tolerance.</summary>
        public bool Solve()
        {
            if (robot == null || target == null) return false;

            var q = robot.GetJoints();

            for (int iter = 0; iter < maxIterations; iter++)
            {
                float err = Vector3.Distance(robot.TcpPosition, target.position);
                if (err <= positionTolerance) break;

                for (int i = Apex6JointMap.JointCount - 1; i >= 0; i--)
                {
                    var joint = robot.joints[i];
                    if (joint == null) continue;

                    Vector3 pivot = joint.position;
                    Vector3 toTcp = robot.TcpPosition - pivot;
                    Vector3 toTarget = target.position - pivot;
                    if (toTcp.sqrMagnitude < 1e-10f || toTarget.sqrMagnitude < 1e-10f)
                        continue;

                    // joint axis in world space = parent frame -> world
                    Vector3 axis = joint.parent != null
                        ? joint.parent.TransformDirection(Apex6JointMap.JointAxes[i])
                        : Apex6JointMap.JointAxes[i];

                    float angle = Vector3.SignedAngle(toTcp, toTarget, axis) * damping;
                    if (float.IsNaN(angle)) continue;

                    q[i] = Apex6JointMap.Clamp(i, q[i] + angle);
                }

                robot.SetJoints(q);
                robot.ApplyCommandNow();
            }

            if (aimAt != null) AimTool(aimAt.position);
            return Vector3.Distance(robot.TcpPosition, target.position) <= positionTolerance;
        }

        // Align the tool axis (TCP - wrist) to point at a world position, using J4/J5/J6.
        void AimTool(Vector3 worldPoint)
        {
            var q = robot.GetJoints();
            for (int i = 0; i < 12; i++)
            {
                Vector3 origin = robot.joints[3].position;
                Vector3 toolDir = (robot.TcpPosition - origin).normalized;
                Vector3 wantDir = (worldPoint - origin).normalized;
                if (Vector3.Angle(toolDir, wantDir) < 0.1f) break;

                for (int j = 5; j >= 3; j--)
                {
                    var joint = robot.joints[j];
                    Vector3 axis = joint.parent != null
                        ? joint.parent.TransformDirection(Apex6JointMap.JointAxes[j])
                        : Apex6JointMap.JointAxes[j];
                    Vector3 a = (robot.TcpPosition - joint.position).normalized;
                    Vector3 b = (worldPoint - joint.position).normalized;
                    if (a.sqrMagnitude < 1e-10f || b.sqrMagnitude < 1e-10f) continue;
                    float angle = Vector3.SignedAngle(a, b, axis) * 0.5f;
                    if (!float.IsNaN(angle))
                        q[j] = Apex6JointMap.Clamp(j, q[j] + angle);
                }
                robot.SetJoints(q);
                robot.ApplyCommandNow();
            }
        }
    }
}
