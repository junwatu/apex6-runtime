// APEX-6 -- Unity demo driver
//
// A tiny example of "a Unity script controlling the arm": sweeps J2/J3/J5 and
// opens/closes the gripper on a loop, respecting every joint limit.
// Add it to the same GameObject as the Apex6Controller.

using UnityEngine;

namespace Apex6
{
    [AddComponentMenu("APEX-6/Apex-6 Demo Driver")]
    [RequireComponent(typeof(Apex6Controller))]
    public class Apex6Demo : MonoBehaviour
    {
        public Apex6Controller robot;
        [Tooltip("Seconds for one full sweep of the demo cycle.")]
        public float period = 6f;

        float _t;

        void Reset() { robot = GetComponent<Apex6Controller>(); }

        void Update()
        {
            if (robot == null) robot = GetComponent<Apex6Controller>();
            if (robot == null) return;

            _t += Time.deltaTime;
            float u = Mathf.Repeat(_t / Mathf.Max(period, 0.01f), 1f);
            float s = Mathf.Sin(u * Mathf.PI * 2f);           // -1..1
            float s2 = Mathf.Sin(u * Mathf.PI * 4f);          // -1..1, twice as fast

            robot.SetJoint(1, Mathf.Lerp(-40f, 50f, (s + 1f) * 0.5f));   // J2 shoulder
            robot.SetJoint(2, Mathf.Lerp(20f, 100f, (s2 + 1f) * 0.5f));  // J3 elbow
            robot.SetJoint(4, Mathf.Lerp(-70f, 70f, (s + 1f) * 0.5f));   // J5 wrist pitch
            robot.SetGripper(Mathf.Lerp(Apex6JointMap.OpeningMin,
                                        Apex6JointMap.OpeningMax,
                                        (s2 + 1f) * 0.5f));
        }
    }
}
