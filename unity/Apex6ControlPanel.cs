// APEX-6 -- Unity runtime control panel
//
// An on-screen panel that drives the arm at runtime:
//   * six joint sliders, bounded by the measured limits (typed entry too)
//   * gripper opening slider + Open / Close
//   * pose presets (zero / home / reach / fold / inspect / wide)
//   * motion speed limiting (rate limiter) and a "snap" mode
//   * an optional IK target you can drag with sliders, plus "Solve now"
//   * a live readout of the tool centre point and the applied joint angles
//
// Add it to the SAME GameObject as the Apex6Controller. It uses IMGUI, so it
// needs no Canvas, no prefab and no font asset, and it works in the Editor's
// Play mode and in a build. Press Play, then use the panel (Tab hides/shows it
// where the legacy Input API is available; the on-panel Hide button always
// works).

using System.Globalization;
using UnityEngine;

namespace Apex6
{
    [AddComponentMenu("APEX-6/Apex-6 Control Panel")]
    [RequireComponent(typeof(Apex6Controller))]
    public class Apex6ControlPanel : MonoBehaviour
    {
        [Header("Wiring")]
        public Apex6Controller robot;
        [Tooltip("Optional: enables the IK section (target + Solve now).")]
        public Apex6IK ik;
        [Tooltip("Optional: enables the demo sweep toggle.")]
        public Apex6Demo demo;
        [Tooltip("Optional: enables the cable-routing toggle.")]
        public Apex6Cables cables;

        [Header("Panel")]
        public bool visible = true;
        [Tooltip("Toggle key when the legacy Input API is available.")]
        public KeyCode toggleKey = KeyCode.Tab;
        public Vector2 position = new Vector2(14f, 14f);

        const float PanelWidth = 372f;
        const float PanelHeight = 620f;
        const string Q_FIELD = "apex6.q";
        const string GAP_FIELD = "apex6.gap";

        // pose presets, in degrees, all inside the measured limits
        static readonly string[] PresetNames = { "Zero", "Home", "Reach", "Fold", "Inspect", "Wide" };
        static readonly float[][] PresetQ =
        {
            new[] {   0f,   0f,   0f,   0f,   0f,   0f },
            new[] {   0f, -20f,  70f,   0f,  55f,   0f },
            new[] {   0f, -55f,  95f,   0f,  20f,   0f },
            new[] {  25f, -70f, 130f,   0f,  70f,  40f },
            new[] { -40f, -35f,  80f,  35f,  45f, -60f },
            new[] {  60f,  40f,  10f, -90f, -60f, 120f },
        };

        readonly float[] _q = new float[Apex6JointMap.JointCount];
        readonly string[] _qText = new string[Apex6JointMap.JointCount];
        float _opening;
        string _openText;
        bool _snap;
        Rect _rect;
        Vector2 _scroll;
        GUIStyle _header;
        GUIStyle _mini;
        bool _initialised;
        int _windowId;

        static int _nextWindowId = 0x41503600;   // stable, unique per panel instance

        static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        void Reset()
        {
            robot = GetComponent<Apex6Controller>();
            ik = GetComponent<Apex6IK>();
            demo = GetComponent<Apex6Demo>();
            cables = GetComponent<Apex6Cables>();
        }

        void Awake()
        {
            if (robot == null) robot = GetComponent<Apex6Controller>();
            if (ik == null) ik = GetComponent<Apex6IK>();
            if (demo == null) demo = GetComponent<Apex6Demo>();
            if (cables == null) cables = GetComponent<Apex6Cables>();
            _windowId = _nextWindowId++;
            _rect = new Rect(position.x, position.y, PanelWidth, PanelHeight);
        }

        void Start() { Pull(); }

        void Update()
        {
#if ENABLE_LEGACY_INPUT_MANAGER
            if (Input.GetKeyDown(toggleKey)) visible = !visible;
#endif
        }

        /// <summary>Copy the controller's current command into the panel state.</summary>
        void Pull()
        {
            if (robot == null) return;
            var q = robot.GetJoints();
            for (int i = 0; i < _q.Length && i < q.Length; i++)
            {
                _q[i] = q[i];
                _qText[i] = _q[i].ToString("0.0", Inv);
            }
            _opening = robot.GetGripper();
            _openText = (_opening * 1000f).ToString("0.0", Inv);
            _snap = robot.speedScale <= 0.0001f;
            _initialised = true;
        }

        // ------------------------------------------------------------------
        //  IMGUI
        // ------------------------------------------------------------------
        void OnGUI()
        {
            if (robot == null) return;
            if (!_initialised) Pull();
            else Sync();
            if (_header == null)
            {
                _header = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold };
                _mini = new GUIStyle(GUI.skin.button) { fontSize = 10 };
            }

            if (!visible)
            {
                float by = Mathf.Min(position.y, Mathf.Max(0f, Screen.height - 24f));
                if (GUI.Button(new Rect(position.x, by, 86f, 22f), "APEX-6 \u25B8"))
                    visible = true;
                return;
            }

            // keep the panel inside whatever Game view we actually have
            float maxH = Mathf.Max(220f, Screen.height - 28f);
            if (_rect.height > maxH) _rect.height = maxH;
            _rect.x = Mathf.Clamp(_rect.x, 0f, Mathf.Max(0f, Screen.width - _rect.width - 2f));
            _rect.y = Mathf.Clamp(_rect.y, 0f, Mathf.Max(0f, Screen.height - _rect.height - 4f));

            _rect = GUI.Window(_windowId, _rect, DrawWindow, "APEX-6  control");
            position = new Vector2(_rect.x, _rect.y);
        }

        /// <summary>Mirror the controller's live state, except for a joint or
        /// gripper field the user is currently typing into.  This keeps the panel
        /// honest when something else drives the arm (the IK solver, the demo
        /// sweep, or a script).</summary>
        void Sync()
        {
            if (robot == null) return;
            string focus = GUI.GetNameOfFocusedControl();
            var q = robot.GetJoints();
            for (int i = 0; i < _q.Length && i < q.Length; i++)
            {
                if (focus == Q_FIELD + i) continue;
                _q[i] = q[i];
                _qText[i] = q[i].ToString("0.0", Inv);
            }
            if (focus != GAP_FIELD)
            {
                _opening = robot.GetGripper();
                _openText = (_opening * 1000f).ToString("0.0", Inv);
            }
            _snap = robot.speedScale <= 0.0001f;
        }

        void DrawWindow(int id)
        {
            // fixed-size window with a bounded, scrollable content region
            var area = new Rect(3f, 19f, _rect.width - 6f, _rect.height - 22f);
            GUILayout.BeginArea(area);
            _scroll = GUILayout.BeginScrollView(_scroll);

            DrawJoints();
            DrawGripper();
            DrawPoses();
            DrawMotion();
            DrawIk();
            DrawReadout();

            GUILayout.Space(4);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Home")) { robot.GoHome(); Pull(); }
            if (GUILayout.Button("Zero")) { robot.GoZero(); Pull(); }
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("Hide")) visible = false;
            GUILayout.EndHorizontal();

            GUILayout.EndScrollView();
            GUILayout.EndArea();
            GUI.DragWindow(new Rect(0f, 0f, 10000f, 18f));
        }

        void DrawJoints()
        {
            GUILayout.Label("Joints (degrees)", _header);
            for (int i = 0; i < _q.Length; i++)
            {
                float lo = Apex6JointMap.LimitMinDeg[i];
                float hi = Apex6JointMap.LimitMaxDeg[i];

                GUILayout.BeginHorizontal();
                GUILayout.Label("J" + (i + 1), GUILayout.Width(20f));

                float v = GUILayout.HorizontalSlider(_q[i], lo, hi, GUILayout.Width(190f));
                if (Mathf.Abs(v - _q[i]) > 1e-4f)
                {
                    _q[i] = v;
                    _qText[i] = v.ToString("0.0", Inv);
                    robot.SetJoint(i, v);
                }

                GUI.SetNextControlName(Q_FIELD + i);
                string t = GUILayout.TextField(_qText[i], GUILayout.Width(56f));
                if (t != _qText[i])
                {
                    _qText[i] = t;
                    float parsed;
                    if (float.TryParse(t, NumberStyles.Float, Inv, out parsed))
                    {
                        _q[i] = Mathf.Clamp(parsed, lo, hi);
                        robot.SetJoint(i, _q[i]);
                    }
                }

                if (GUILayout.Button("0", _mini, GUILayout.Width(20f)))
                {
                    _q[i] = 0f;
                    _qText[i] = "0.0";
                    robot.SetJoint(i, 0f);
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.Label(string.Format(Inv, "limits J2 {0:0}\u2026{1:0}  J3 {2:0}\u2026{3:0}  J5 {4:0}\u2026{5:0}",
                Apex6JointMap.LimitMinDeg[1], Apex6JointMap.LimitMaxDeg[1],
                Apex6JointMap.LimitMinDeg[2], Apex6JointMap.LimitMaxDeg[2],
                Apex6JointMap.LimitMinDeg[4], Apex6JointMap.LimitMaxDeg[4]), _mini);
        }

        void DrawGripper()
        {
            GUILayout.Space(6);
            GUILayout.Label("Gripper (pad-face gap, mm)", _header);
            GUILayout.BeginHorizontal();
            float mm = GUILayout.HorizontalSlider(_opening * 1000f,
                Apex6JointMap.OpeningMin * 1000f, Apex6JointMap.OpeningMax * 1000f,
                GUILayout.Width(220f));
            if (Mathf.Abs(mm - _opening * 1000f) > 1e-3f)
            {
                _opening = mm / 1000f;
                _openText = mm.ToString("0.0", Inv);
                robot.SetGripper(_opening);
            }
            GUI.SetNextControlName(GAP_FIELD);
            string t = GUILayout.TextField(_openText, GUILayout.Width(56f));
            if (t != _openText)
            {
                _openText = t;
                float parsed;
                if (float.TryParse(t, NumberStyles.Float, Inv, out parsed))
                {
                    _opening = Mathf.Clamp(parsed / 1000f,
                        Apex6JointMap.OpeningMin, Apex6JointMap.OpeningMax);
                    robot.SetGripper(_opening);
                }
            }
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Open")) { robot.OpenGripper(); Pull(); }
            if (GUILayout.Button("Close")) { robot.CloseGripper(); Pull(); }
            GUILayout.Label(string.Format(Inv, "min {0:0} / max {1:0} mm",
                Apex6JointMap.OpeningMin * 1000f, Apex6JointMap.OpeningMax * 1000f), _mini);
            GUILayout.EndHorizontal();
        }

        void DrawPoses()
        {
            GUILayout.Space(6);
            GUILayout.Label("Pose presets", _header);
            GUILayout.BeginHorizontal();
            for (int i = 0; i < PresetNames.Length; i++)
            {
                if (GUILayout.Button(PresetNames[i], GUILayout.Width(52f)))
                {
                    robot.SetJoints(PresetQ[i]);
                    Pull();
                }
            }
            GUILayout.EndHorizontal();
        }

        void DrawMotion()
        {
            GUILayout.Space(6);
            GUILayout.Label("Motion", _header);
            GUILayout.BeginHorizontal();
            bool snap = GUILayout.Toggle(_snap, " Instant (no rate limit)");
            if (snap != _snap)
            {
                _snap = snap;
                robot.speedScale = _snap ? 0f : 1f;
            }
            GUILayout.FlexibleSpace();
            if (cables != null)
            {
                bool r = GUILayout.Toggle(cables.routing, " Cables");
                if (r != cables.routing) cables.routing = r;
            }
            if (demo != null)
            {
                bool run = GUILayout.Toggle(demo.enabled, " Demo sweep");
                if (run != demo.enabled) demo.enabled = run;
            }
            GUILayout.EndHorizontal();
            if (!_snap)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("speed", GUILayout.Width(42f));
                robot.speedScale = GUILayout.HorizontalSlider(robot.speedScale, 0.05f, 1f);
                GUILayout.Label((robot.speedScale * 100f).ToString("0", Inv) + "%",
                                GUILayout.Width(40f));
                GUILayout.EndHorizontal();
            }
        }

        void DrawIk()
        {
            if (ik == null) return;
            GUILayout.Space(6);
            GUILayout.Label("Inverse kinematics (world metres)", _header);

            GUILayout.BeginHorizontal();
            if (GUILayout.Button(ik.target == null ? "Create IK target" : "Move target to TCP"))
            {
                if (ik.target == null)
                {
                    var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    go.name = "IK Target";
                    go.transform.localScale = Vector3.one * 0.05f;
                    ik.target = go.transform;
                }
                ik.target.position = robot.TcpPosition;
            }
            if (ik.target != null && GUILayout.Button("Clear")) ik.target = null;
            GUILayout.EndHorizontal();

            if (ik.target != null)
            {
                Vector3 p = ik.target.position;
                p.x = Slider("X", p.x, -1.5f, 1.5f);
                p.y = Slider("Y", p.y, 0.0f, 1.9f);
                p.z = Slider("Z", p.z, -1.5f, 1.5f);
                ik.target.position = p;

                GUILayout.BeginHorizontal();
                ik.solveEveryFrame = GUILayout.Toggle(ik.solveEveryFrame, " Solve every frame");
                if (GUILayout.Button("Solve now")) ik.Solve();
                GUILayout.EndHorizontal();

                float err = Vector3.Distance(robot.TcpPosition, ik.target.position);
                GUILayout.Label(string.Format(Inv, "target reached to {0:0.0} mm  (tol {1:0.0} mm)",
                    err * 1000f, ik.positionTolerance * 1000f), _mini);
            }
        }

        float Slider(string label, float value, float min, float max)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(20f));
            float v = GUILayout.HorizontalSlider(value, min, max);
            GUILayout.Label(value.ToString("0.000", Inv) + " m", GUILayout.Width(64f));
            GUILayout.EndHorizontal();
            return v;
        }

        void DrawReadout()
        {
            GUILayout.Space(6);
            GUILayout.Label("Readout", _header);
            Vector3 tcp = robot.TcpPosition;
            Vector3 euler = robot.TcpRotation.eulerAngles;
            GUILayout.Label(string.Format(Inv, "TCP   pos ({0:0.000}, {1:0.000}, {2:0.000}) m",
                tcp.x, tcp.y, tcp.z), _mini);
            GUILayout.Label(string.Format(Inv, "      rot ({0:0.0}, {1:0.0}, {2:0.0}) deg",
                euler.x, euler.y, euler.z), _mini);

            var applied = robot.GetAppliedJoints();
            var cmd = robot.GetJoints();
            float worst = 0f;
            for (int i = 0; i < cmd.Length; i++)
                worst = Mathf.Max(worst, Mathf.Abs(applied[i] - cmd[i]));
            GUILayout.Label(string.Format(Inv, "joints {0}  (largest lag {1:0.00} deg)",
                robot.AtTarget() ? "at target" : "moving", worst), _mini);
            GUILayout.Label(string.Format(Inv, "gripper gap {0:0.0} mm   scale {1:0.00}",
                robot.GetGripper() * 1000f, robot.speedScale), _mini);
        }

        void OnDrawGizmosSelected()
        {
            if (ik == null || ik.target == null) return;
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(ik.target.position, 0.05f);
        }
    }
}
