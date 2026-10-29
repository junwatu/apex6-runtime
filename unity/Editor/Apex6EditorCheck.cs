// APEX-6 -- Unity editor import check (editor-only)
//
// A one-click version of Apex6SelfTest that ALSO reports what Unity's importer
// did with the file, and writes the result to a text file next to the project
// so it can be inspected without the Console.
//
//     menu:  APEX-6 -> Run Import Validation  (FBX / GLB)
//     file:  <project>/apex6_selfcheck.txt
//
// Put this under any folder named "Editor".

#if UNITY_EDITOR
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace Apex6.EditorTools
{
    public static class Apex6EditorCheck
    {
        public static string ReportPath
        {
            get
            {
                var project = Directory.GetParent(Application.dataPath).FullName;
                return Path.Combine(project, "apex6_selfcheck.txt");
            }
        }

        [MenuItem("APEX-6/Run Import Validation (FBX)")]
        public static void MenuFbx() { FinishRun(Run(FindAsset(".fbx"))); }

        [MenuItem("APEX-6/Run Import Validation (GLB)")]
        public static void MenuGlb() { FinishRun(Run(FindAsset(".glb"))); }

        static void FinishRun(string text)
        {
            Debug.Log(text);
            EditorUtility.DisplayDialog("APEX-6", "Report written to:\n" + ReportPath, "OK");
        }

        /// <summary>Instantiate the given model prefab, drive it, write the
        /// report.  Pass null to auto-find an apex6 model under Assets/.</summary>
        public static string Run(string assetPath = null)
        {
            var sb = new StringBuilder();
            sb.AppendLine("=== APEX-6 Unity import validation ===");
            sb.AppendLine("unity " + Application.unityVersion);

            string path = assetPath != null ? assetPath : FindAsset(null);
            if (path == null)
            {
                sb.AppendLine("FAIL: no apex6 *.fbx / *.glb found under Assets/");
                return Finish(sb);
            }
            sb.AppendLine("asset : " + path);

            // ---- what the importer did with the file -------------------------
            var mi = AssetImporter.GetAtPath(path) as ModelImporter;
            if (mi != null)
                sb.AppendLine(string.Format(
                    "import: globalScale={0} useFileScale={1} fileScale={2}",
                    mi.globalScale, mi.useFileScale, mi.fileScale));

            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            if (prefab == null)
            {
                sb.AppendLine("FAIL: could not load " + path + " (still importing?)");
                return Finish(sb);
            }
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            inst.name = "Apex6ImportCheck";
            try
            {
                var ctrl = inst.GetComponent<Apex6Controller>();
                if (ctrl == null) ctrl = inst.AddComponent<Apex6Controller>();
                var st = inst.GetComponent<Apex6SelfTest>();
                if (st == null) st = inst.AddComponent<Apex6SelfTest>();
                ctrl.root = inst.transform;
                st.root = inst.transform;
                st.robot = ctrl;

                ctrl.Prepare();
                sb.Append(CheckHierarchy(inst.transform));
                sb.Append(st.Report());
            }
            finally
            {
                Object.DestroyImmediate(inst);
            }
            return Finish(sb);
        }

        static string CheckHierarchy(Transform root)
        {
            var sb = new StringBuilder();
            string[] wanted = {
                "CTRL_J1_Waist", "CTRL_J2_Shoulder", "CTRL_J3_Elbow", "CTRL_J4_WristRoll",
                "CTRL_J5_WristPitch", "CTRL_J6_ToolRoll",
                "CTRL_Gripper", "CTRL_FingerL", "CTRL_FingerR", "CTRL_TCP"
            };
            int found = 0;
            var missing = new StringBuilder();
            foreach (var w in wanted)
            {
                if (FindDeep(root, w) != null) found++;
                else missing.Append(w).Append(' ');
            }
            sb.AppendLine(string.Format("pivots: {0}/{1} named pivots present{2}",
                found, wanted.Length,
                found == wanted.Length ? "" : "   MISSING: " + missing));

            var renderers = root.GetComponentsInChildren<Renderer>();
            sb.AppendLine("renderers under root: " + renderers.Length);
            return sb.ToString();
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

        static string FindAsset(string ext)
        {
            string best = null;
            foreach (var guid in AssetDatabase.FindAssets("t:Model"))
            {
                var p = AssetDatabase.GUIDToAssetPath(guid);
                var f = Path.GetFileName(p).ToLowerInvariant();
                if (!f.Contains("apex6")) continue;
                if (ext != null && !f.EndsWith(ext)) continue;
                if (f.EndsWith(".fbx") || best == null) best = p;
            }
            return best;
        }

        static string Finish(StringBuilder sb)
        {
            var text = sb.ToString();
            try { File.WriteAllText(ReportPath, text); }
            catch { /* report is best-effort; the Console still has it */ }
            return text;
        }
    }
}
#endif
