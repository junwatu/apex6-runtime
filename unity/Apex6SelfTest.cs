// APEX-6 -- Unity in-editor self test
//
// Confirms, on YOUR import, the things that depend on how Unity imported the
// file rather than on the model:
//   1. conversion -- the Blender->Unity frame Unity actually applied
//   2. scale      -- the tool centre point must sit 1.77 m up at the zero pose
//   3. kinematics -- live TCP must match the project's forward kinematics
//   4. gripper    -- the finger gap must track the commanded opening
//
// Drop this on the imported root, set 'root' to the imported root, then use the
// component's context menu (gear icon) -> "Run Self Test" and read the Console.

using System.Text;
using UnityEngine;

namespace Apex6
{
    [AddComponentMenu("APEX-6/Apex-6 Self Test")]
    public class Apex6SelfTest : MonoBehaviour
    {
        public Apex6Controller robot;
        public Transform root;
        [Tooltip("Expected tool-centre height at the zero pose (meters).")]
        public float expectedHeight = 1.770f;

        void Reset() { robot = GetComponent<Apex6Controller>(); }

        [ContextMenu("Run Self Test")]
        public void Run() { Debug.Log(Report()); }

        /// <summary>Run the checks and return the report (also handy from an
        /// editor tool, which cannot see the Console).</summary>
        public string Report()
        {
            var sb = new StringBuilder();
            sb.AppendLine("=== APEX-6 Unity self test ===");

            if (robot == null) robot = GetComponent<Apex6Controller>();
            if (robot == null) return sb.Append("no Apex6Controller").ToString();
            if (root == null) root = robot.transform;
            robot.root = root;
            robot.Prepare();
            robot.GoZero();
            robot.ApplyCommandNow();

            // ---- 1. importer conversion --------------------------------
            sb.AppendLine("conversion: " + robot.ConversionNote);
            sb.AppendLine(string.Format("  rest-position fit residual = {0:F3} mm",
                robot.ConversionResidual * 1000f));

            // ---- 2. scale / zero pose ----------------------------------
            Vector3 wantZero = robot.ConvertBlender(
                new Vector3(0.07f, 0f, expectedHeight));
            float zeroErr = Vector3.Distance(robot.TcpPosition, wantZero);
            sb.AppendLine(string.Format(
                "zero TCP = ({0:F3}, {1:F3}, {2:F3}) m  expected ({3:F3}, {4:F3}, {5:F3})  err={6:F2} mm",
                robot.TcpPosition.x, robot.TcpPosition.y, robot.TcpPosition.z,
                wantZero.x, wantZero.y, wantZero.z, zeroErr * 1000f));
            sb.AppendLine("  scale: " + (zeroErr < Apex6JointMap.SelfTestTolerance
                ? "OK (1 unit = 1 metre)" : "CHECK the FBX Scale Factor / import units"));

            // ---- 3. forward kinematics --------------------------------
            sb.AppendLine("forward-kinematics check (pose -> live TCP vs project FK):");
            int ok = 0, n = Apex6JointMap.SelfTestQ.Length;
            float worst = 0f;
            for (int k = 0; k < n; k++)
            {
                float[] q = Apex6JointMap.SelfTestQ[k];
                robot.SetJoints(q);
                robot.ApplyCommandNow();
                Vector3 want = robot.ConvertBlender(Apex6JointMap.SelfTestTcpBlender[k]);
                float err = Vector3.Distance(robot.TcpPosition, want);
                if (err > worst) worst = err;
                bool pass = err < Apex6JointMap.SelfTestTolerance;
                if (pass) ok++;
                sb.AppendLine(string.Format(
                    "  pose {0}: q=[{1}]  err={2:F3} mm  {3}",
                    k, string.Join(", ", System.Array.ConvertAll(q, x => x.ToString("0"))),
                    err * 1000f, pass ? "ok" : "FAIL"));
            }
            sb.AppendLine(string.Format("  {0}/{1} poses within {2:F0} mm  (worst {3:F3} mm)",
                ok, n, Apex6JointMap.SelfTestTolerance * 1000f, worst * 1000f));

            // ---- 4. per-joint response (+2 deg) ------------------------
            sb.AppendLine("joint response to +2 deg (informational):");
            for (int i = 0; i < Apex6JointMap.JointCount; i++)
            {
                robot.GoZero(); robot.ApplyCommandNow();
                var j = robot.joints[i];
                if (j == null) { sb.AppendLine(string.Format("  J{0}: MISSING", i + 1)); continue; }
                Vector3 before = robot.TcpPosition;
                robot.SetJoint(i, 2f); robot.ApplyCommandNow();
                Vector3 d = robot.TcpPosition - before;
                sb.AppendLine(string.Format(
                    "  J{0} {1,-18} axisLocal=({2:F2},{3:F2},{4:F2})  dTCP={5:F1} mm",
                    i + 1, j.name, robot.AxisLocal(i).x, robot.AxisLocal(i).y,
                    robot.AxisLocal(i).z, d.magnitude * 1000f));
            }

            // ---- 5. gripper --------------------------------------------
            robot.GoZero(); robot.SetGripper(Apex6JointMap.OpeningMax);
            robot.ApplyCommandNow();
            if (robot.fingerL != null && robot.fingerR != null)
            {
                float gapOpen = Vector3.Distance(robot.fingerL.position, robot.fingerR.position);
                robot.CloseGripper(); robot.ApplyCommandNow();
                float gapClosed = Vector3.Distance(robot.fingerL.position, robot.fingerR.position);
                sb.AppendLine(string.Format(
                    "gripper finger-centre gap: open={0:F1} mm  closed={1:F1} mm  (travel {2:F1} mm)",
                    gapOpen * 1000f, gapClosed * 1000f, (gapOpen - gapClosed) * 1000f));
            }
            else sb.AppendLine("gripper : finger transforms not found");

            robot.GoZero();
            sb.AppendLine(string.Format("self test: joints={0}, {1}/{2} FK poses ok",
                robot.Ready ? "ready" : "INCOMPLETE", ok, n));
            return sb.ToString();
        }
    }
}
