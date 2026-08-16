#!/usr/bin/env python3
"""통합 URDF + obstacles.yaml → Unity 프로젝트 (URDF Importer 용) — 7주차 M4

Unity 는 **이 PC 에 설치해서 직접 확인**한다(사용자 결정 2026-08-14). 그래서 이 스크립트는
Unity 가 그대로 열 수 있는 **프로젝트 폴더**를 통째로 만든다.

  Assets/RdaRobot/rda_robot.urdf     통합 URDF (package:// 유지)
  Assets/RdaRobot/<pkg>/meshes/...   URDF 가 참조하는 mesh 를 package 이름 그대로 배치
                                     (URDF Importer 가 package:// 를 이 폴더 기준으로 푼다)
  Assets/RdaRobot/greenhouse.json    온실·작물 primitive 186개 (obstacles.yaml 단일 진실원)
  Assets/Editor/GreenhouseBuilder.cs 위 JSON 으로 온실을 씬에 세우는 에디터 스크립트
  Assets/Editor/RdaBatch.cs          배치모드 검증용(임포트→온실→리포트 JSON)
  Packages/manifest.json             URDF Importer(git) 의존성
  ProjectSettings/ProjectVersion.txt Unity 버전

좌표 변환은 Unity Robotics 의 규약(FLU→RUF)을 그대로 쓴다 — 위치 (x,y,z)→(−y,z,x),
회전 (x,y,z,w)→(−y,z,x,−w). URDF Importer 가 로봇에 쓰는 것과 같은 규약이라 온실과 로봇이
같은 좌표에 선다.

사용: ros2 run rda_robot_bringup gen_unity_assets.py --out ~/robot_ws/export/unity
"""
import argparse
import importlib.util
import json
import os
import re
import shutil
import sys

import numpy as np
import yaml

_here = os.path.dirname(os.path.abspath(__file__))


def _import_sibling(mod_name, file_name):
    p = os.path.join(_here, file_name)
    spec = importlib.util.spec_from_file_location(mod_name, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


G = _import_sibling("_gen_usd_scene", "gen_usd_scene.py")   # URDF 조립·파싱·mesh 해석 재사용

UNITY_VERSION = "2022.3.62f3"
URDF_IMPORTER = ("https://github.com/Unity-Technologies/URDF-Importer.git"
                 "?path=/com.unity.robotics.urdf-importer#v0.5.2")
# ROS 연동(2026-08-16) — 살아 있는 ROS 그래프의 /joint_states 를 Unity 가 그대로 재현한다.
# ROS 쪽 짝은 `ros_tcp_endpoint`(vendor, Apache-2.0, 같은 0.7.x).
ROS_TCP_CONNECTOR = ("https://github.com/Unity-Technologies/ROS-TCP-Connector.git"
                     "?path=/com.unity.robotics.ros-tcp-connector#v0.7.0")


# ────────────────────────────────────────────── C# (런타임 — ROS 연동, 2026-08-16 신설)
BRIDGE_CS = r'''// ROS 연동 — 살아 있는 ROS 그래프의 /joint_states 를 Unity 가 그대로 재현한다.
// (RDA 7주차 M4, 자동 생성물 — 생성기 = rda_robot_bringup/scripts/gen_unity_assets.py)
//
// ■ 무엇인가 / 무엇이 아닌가  ← 보고서에 쓸 때 반드시 구분할 것
//   이것은 **미러(재현)** 다. Gazebo/ros2_control 이 물리와 제어를 맡고, Unity 는 그 결과인
//   관절각을 받아 같은 자세를 그린다. Unity 가 물리를 계산해 되돌려 주는 것이 **아니다**
//   (그건 하드웨어 인터페이스 교체 = 별개의 큰 작업).
//   ⇒ 그래서 ArticulationBody 의 드라이브 목표가 아니라 **jointPosition 을 직접 세팅**한다.
//      드라이브로 밀면 Unity 물리가 개입해 '재현'이 아니라 '또 다른 시뮬'이 된다.
//
// ■ 단위·부호
//   URDF/ROS 관절각 = 라디안. Unity ArticulationBody 의 xDrive/jointPosition = **도(degree)**.
//   URDF Importer 가 관절축을 Unity 좌표로 이미 바꿔 놓았으므로 각도는 부호 그대로 쓴다
//   (위치 변환식과 달리 관절각에는 좌표 변환을 다시 걸지 않는다 — 걸면 이중 적용이 된다).
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RdaRosBridge : MonoBehaviour
{
    [Tooltip("ROS 쪽 ros_tcp_endpoint 주소")]
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;
    public string jointStatesTopic = "/joint_states";
    [Tooltip("헤드리스 검증용 기록 파일(비우면 기록 안 함)")]
    public string reportPath = "";
    [Tooltip("이 시간(초) 동안 돌고 스스로 종료. 0 이면 계속.")]
    public float quitAfter = 0f;

    // 🔴 ROS 관절 이름 ≠ Unity 오브젝트 이름.
    //    URDF Importer 는 ArticulationBody 를 **자식 링크** 이름의 오브젝트에 붙인다
    //    (base→link1 … rg2_finger_joint1→rg2_leftfinger). 그래서 생성기가 URDF 에서 뽑아 둔
    //    joint_map.json(관절→자식링크)을 읽어 맞춘다. 이름을 짐작하면 조용히 아무것도 안 움직인다.
    [Tooltip("Resources 의 joint_map.json (관절 이름 → 자식 링크 이름)")]
    public TextAsset jointMap;

    readonly Dictionary<string, ArticulationBody> _joints = new Dictionary<string, ArticulationBody>();
    readonly Dictionary<ArticulationBody, ArticulationBody> _rootOf = new Dictionary<ArticulationBody, ArticulationBody>();
    readonly Dictionary<ArticulationBody, int> _dofIndex = new Dictionary<ArticulationBody, int>();
    readonly Dictionary<ArticulationBody, List<float>> _dofBuf = new Dictionary<ArticulationBody, List<float>>();
    int _msgs = 0;
    float _t0;
    StreamWriter _log;
    string[] _lastNames = new string[0];
    double[] _lastPos = new double[0];

    void Start()
    {
        _t0 = Time.time;
        // 🔴 루트 아티큘레이션을 고정하지 않으면 로봇이 통째로 중력에 떨어진다.
        //    (실측: tcp 가 [0,1.2119,0.2074] 대신 [0.709,0.941,0.681] 로 나왔다 — 넘어지는 중)
        //    이 노드는 **미러**라 베이스는 월드에 고정돼 있어야 한다. 모바일 베이스 주행까지
        //    재현하려면 여기가 아니라 /odom 을 받아 루트를 옮기는 별도 작업이다.
        // ⚠ `isRoot` 만 보면 안 된다 — URDF Importer 가 바퀴·센서를 **별도 아티큘레이션 트리**로
        //    만들어 루트가 여러 개다(실측: front_left_wheel_link·sensor2_base_link·inertial_link…).
        //    전부 고정해야 하나라도 떨어지지 않는다.
        int fixedRoots = 0;
        foreach (var ab in FindObjectsOfType<ArticulationBody>(true))
            if (ab.isRoot) { ab.immovable = true; fixedRoots++; }
        Debug.Log($"[RdaRosBridge] 아티큘레이션 루트 {fixedRoots}개 고정(immovable)");

        var byObject = new Dictionary<string, ArticulationBody>();
        foreach (var ab in FindObjectsOfType<ArticulationBody>(true))
        {
            // 고정 관절은 각도가 없다 — 가동 관절만 잡는다.
            if (ab.jointType == ArticulationJointType.RevoluteJoint ||
                ab.jointType == ArticulationJointType.PrismaticJoint)
                byObject[ab.gameObject.name] = ab;
        }
        // joint_map.json = {"base":"link1", ...} — 최소 파서(외부 JSON 라이브러리 없이).
        var mapText = jointMap != null ? jointMap.text
                      : (Resources.Load<TextAsset>("joint_map") != null
                         ? Resources.Load<TextAsset>("joint_map").text : null);
        if (string.IsNullOrEmpty(mapText))
        {
            Debug.LogWarning("[RdaRosBridge] joint_map 을 못 찾았다 — 오브젝트 이름으로 직접 맞춘다(대개 실패한다)");
            foreach (var kv in byObject) _joints[kv.Key] = kv.Value;
        }
        else
        {
            foreach (var pair in mapText.Split(','))
            {
                var kv = pair.Split(':');
                if (kv.Length != 2) continue;
                var jn = kv[0].Trim().Trim('{', '}', '"', ' ', '\n', '\r', '\t');
                var ln = kv[1].Trim().Trim('{', '}', '"', ' ', '\n', '\r', '\t');
                if (jn.Length == 0 || ln.Length == 0) continue;
                if (byObject.TryGetValue(ln, out var ab)) _joints[jn] = ab;
                else Debug.LogWarning($"[RdaRosBridge] 링크 오브젝트 없음: {ln} (관절 {jn})");
            }
        }
        // 관절마다 '어느 루트의 몇 번째 dof 인가' 를 미리 구해 둔다.
        // GetDofStartIndices 는 **바디 인덱스 → dof 시작 인덱스** 표를 준다(루트에서 부른다).
        foreach (var kv in _joints)
        {
            var ab = kv.Value;
            var root = ab;
            while (!root.isRoot && root.transform.parent != null)
            {
                var p = root.transform.parent.GetComponentInParent<ArticulationBody>();
                if (p == null) break;
                root = p;
            }
            _rootOf[ab] = root;
            var starts = new List<int>();
            root.GetDofStartIndices(starts);
            _dofIndex[ab] = (ab.index >= 0 && ab.index < starts.Count) ? starts[ab.index] : -1;
        }
        // 미러이므로 중력은 끈다 — 켜 두면 메시지 사이 프레임마다 팔이 처져 '재현' 이 아니게 된다.
        foreach (var ab in FindObjectsOfType<ArticulationBody>(true)) ab.useGravity = false;

        Debug.Log($"[RdaRosBridge] 가동관절 {_joints.Count}개 매핑: {string.Join(",", _joints.Keys)}");
        foreach (var kv in _joints)
            Debug.Log($"[RdaRosBridge]   {kv.Key} → {kv.Value.name} (root={_rootOf[kv.Value].name}, dof={_dofIndex[kv.Value]})");

        if (!string.IsNullOrEmpty(reportPath))
            _log = new StreamWriter(reportPath, false);

        var ros = ROSConnection.GetOrCreateInstance();
        ros.RosIPAddress = rosIP;
        ros.RosPort = rosPort;
        ros.Connect();
        ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointState);
        Debug.Log($"[RdaRosBridge] {rosIP}:{rosPort} 구독 {jointStatesTopic}");
    }

    void OnJointState(JointStateMsg msg)
    {
        _msgs++;
        _lastNames = msg.name;
        _lastPos = msg.position;

        // 🔴 개별 `ab.jointPosition = …` 은 아티큘레이션에서 먹지 않는다(조용히 무시된다).
        //    실측: 관절값은 정확히 들어오는데 팔이 무너져 tcp 가 [-0.818, 0.039, 0.159] 로 나왔다
        //    (기대 [0, 1.2119, 0.2074]). 지원되는 경로는 **루트에서 dof 배열을 통째로** 쓰는 것이다.
        //    루트별로 모아서 GetJointPositions → 해당 dof 만 수정 → SetJointPositions.
        var touched = new HashSet<ArticulationBody>();
        for (int i = 0; i < msg.name.Length && i < msg.position.Length; i++)
        {
            if (!_joints.TryGetValue(msg.name[i], out var ab)) continue;
            var root = _rootOf[ab];
            if (!_dofBuf.TryGetValue(root, out var buf))
            {
                buf = new List<float>();
                _dofBuf[root] = buf;
            }
            if (!touched.Contains(root)) { root.GetJointPositions(buf); touched.Add(root); }
            int d = _dofIndex[ab];
            if (d >= 0 && d < buf.Count) buf[d] = (float)msg.position[i];
            // 드라이브 목표도 같이 맞춰 둔다(도 단위) — 물리가 되돌리려 하지 않게.
            var drive = ab.xDrive;
            drive.target = (float)(msg.position[i] * Mathf.Rad2Deg);
            ab.xDrive = drive;
        }
        foreach (var root in touched) root.SetJointPositions(_dofBuf[root]);
    }

    void Update()
    {
        if (quitAfter > 0f && Time.time - _t0 > quitAfter)
        {
            WriteReport();
            Debug.Log($"[RdaRosBridge] 완료 — 수신 {_msgs}건");
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit(0);
#endif
        }
    }

    void OnApplicationQuit() { WriteReport(); }

    void WriteReport()
    {
        if (_log == null) return;
        var tcp = GameObject.Find("tcp");
        var sb = new System.Text.StringBuilder();
        sb.Append("{\n");
        sb.Append($"  \"received\": {_msgs},\n");
        sb.Append($"  \"joints_in_scene\": {_joints.Count},\n");
        sb.Append("  \"last\": {");
        var parts = new List<string>();
        for (int i = 0; i < _lastNames.Length && i < _lastPos.Length; i++)
            parts.Add($"\"{_lastNames[i]}\": {_lastPos[i].ToString("F6")}");
        sb.Append(string.Join(", ", parts));
        sb.Append("},\n");
        if (tcp != null)
        {
            var p = tcp.transform.position;
            sb.Append($"  \"tcp\": [{p.x:F6}, {p.y:F6}, {p.z:F6}]\n");
        }
        else sb.Append("  \"tcp\": null\n");
        sb.Append("}\n");
        _log.Write(sb.ToString());
        _log.Flush();
        _log.Close();
        _log = null;
    }
}
'''


# ────────────────────────────────────────────────────────── C# (에디터 스크립트)
GREENHOUSE_CS = r'''// 온실·작물을 greenhouse.json 에서 읽어 씬에 세운다 (RDA 7주차 M4, 자동 생성물)
// ROS(FLU) → Unity(RUF) 변환은 Unity Robotics 규약과 동일: pos (x,y,z)→(−y,z,x)
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

public static class GreenhouseBuilder
{
    [System.Serializable] public class Item {
        public string name; public string type; public float[] xyz; public float[] rpy;
        public float[] size; public float radius; public float height; public float[] color;
    }
    [System.Serializable] public class Doc { public List<Item> items; }

    public const string Root = "Greenhouse";
    const string JsonPath = "Assets/RdaRobot/greenhouse.json";

    public static Vector3 RosToUnity(float x, float y, float z) => new Vector3(-y, z, x);

    public static Quaternion RosToUnity(Quaternion q) => new Quaternion(-q.y, q.z, q.x, -q.w);

    static Quaternion FromRpy(float r, float p, float y)   // URDF rpy(고정축 XYZ) → 쿼터니언
    {
        float cr = Mathf.Cos(r * 0.5f), sr = Mathf.Sin(r * 0.5f);
        float cp = Mathf.Cos(p * 0.5f), sp = Mathf.Sin(p * 0.5f);
        float cy = Mathf.Cos(y * 0.5f), sy = Mathf.Sin(y * 0.5f);
        return new Quaternion(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                              cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy);
    }

    [MenuItem("RDA/온실 세우기")]
    public static GameObject Build()
    {
        var old = GameObject.Find(Root);
        if (old != null) Object.DestroyImmediate(old);
        var doc = JsonUtility.FromJson<Doc>(File.ReadAllText(JsonPath));
        var root = new GameObject(Root);
        foreach (var it in doc.items)
        {
            GameObject go;
            if (it.type == "box") {
                go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.transform.localScale = new Vector3(it.size[1], it.size[2], it.size[0]); // 축 교환
            } else if (it.type == "cylinder") {
                go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);   // Unity 원기둥 축 = Y, 높이 2
                go.transform.localScale = new Vector3(it.radius * 2f, it.height * 0.5f, it.radius * 2f);
            } else {
                go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.transform.localScale = Vector3.one * (it.radius * 2f);
            }
            go.name = it.name;
            go.transform.SetParent(root.transform, false);
            go.transform.localPosition = RosToUnity(it.xyz[0], it.xyz[1], it.xyz[2]);
            var q = RosToUnity(FromRpy(it.rpy[0], it.rpy[1], it.rpy[2]));
            if (it.type == "cylinder")                      // ROS 원기둥 축 Z → Unity 축 Y 보정
                q = q * Quaternion.Euler(90f, 0f, 0f);
            go.transform.localRotation = q;
            go.isStatic = true;
            if (it.color != null && it.color.Length >= 3) {
                var mr = go.GetComponent<MeshRenderer>();
                var mat = new Material(Shader.Find("Standard"));
                mat.color = new Color(it.color[0], it.color[1], it.color[2],
                                      it.color.Length > 3 ? it.color[3] : 1f);
                mr.sharedMaterial = mat;
            }
        }
        Debug.Log($"[RDA] 온실 {doc.items.Count}개 생성");
        return root;
    }
}
'''

BATCH_CS = r'''// 배치모드 검증 — URDF 임포트 + 온실 생성 + 리포트 JSON (RDA 7주차 M4, 자동 생성물)
// URDF Importer 패키지는 리플렉션으로 부른다(패키지가 없어도 컴파일은 되게).
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class RdaBatch
{
    const string Urdf = "Assets/RdaRobot/rda_robot.urdf";
    const string Report = "rda_unity_report.json";

    public static void RunAll()
    {
        var lines = new List<string>();
        try {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var robot = ImportRobot();
            var green = GreenhouseBuilder.Build();
            WriteReport(robot, green);
            Debug.Log("[RDA] 완료");
            EditorApplication.Exit(0);
        } catch (Exception e) {
            Debug.LogError("[RDA] 실패: " + e);
            File.WriteAllText(Report, "{\"error\":\"" + e.Message.Replace("\"", "'") + "\"}");
            EditorApplication.Exit(1);
        }
    }

    static GameObject ImportRobot()
    {
        var ext = AppDomain.CurrentDomain.GetAssemblies()
            .SelectMany(a => { try { return a.GetTypes(); } catch { return new Type[0]; } })
            .FirstOrDefault(t => t.FullName == "Unity.Robotics.UrdfImporter.UrdfRobotExtensions");
        if (ext == null) throw new Exception("URDF Importer 패키지를 찾지 못했다(manifest.json 확인)");
        var settingsType = AppDomain.CurrentDomain.GetAssemblies()
            .SelectMany(a => { try { return a.GetTypes(); } catch { return new Type[0]; } })
            .First(t => t.FullName == "Unity.Robotics.UrdfImporter.ImportSettings");
        var settings = Activator.CreateInstance(settingsType);
        // 충돌체 분해는 unity(Convex) 로. 기본값이 vHACD 인데 mesh 23개에 매우 오래 걸린다.
        var convexField = settingsType.GetField("convexMethod");
        if (convexField != null) {
            var unityVal = Enum.Parse(convexField.FieldType, "unity");
            convexField.SetValue(settings, unityVal);
        }
        var create = ext.GetMethods(BindingFlags.Public | BindingFlags.Static)
            .First(m => m.Name == "Create" && m.GetParameters().Length >= 2);
        // 🔴 리플렉션은 C# 기본값 인자를 채워 주지 않는다 — 선언된 개수만큼 다 넘겨야 한다.
        //    v0.5.2 = Create(string, ImportSettings, bool loadStatus, bool forceRuntimeMode) 4개.
        //    개수를 가정하지 말고 선언에서 읽어 뒤를 false 로 채운다.
        var ps = create.GetParameters();
        var args = new object[ps.Length];
        args[0] = Path.GetFullPath(Urdf);
        args[1] = settings;
        for (int i = 2; i < ps.Length; i++) args[i] = false;
        var ret = create.Invoke(null, args);
        // 🔴 Create 는 IEnumerator<GameObject> 를 돌려주는 코루틴이라 호출만으로는 아무 일도 안 난다.
        //    loadStatus=false 면 중간에 양보하지 않으므로 MoveNext 로 끝까지 돌리면 동기 완료된다.
        GameObject imported = null;
        if (ret is System.Collections.IEnumerator it) {
            while (it.MoveNext()) { if (it.Current is GameObject g) imported = g; }
            if (it.Current is GameObject last) imported = last;
        }
        var go = imported ?? GameObject.Find("rda_robot") ?? GameObject.FindObjectsOfType<GameObject>()
            .FirstOrDefault(g => g.transform.parent == null && g.name != GreenhouseBuilder.Root);
        if (go == null) throw new Exception("임포트된 로봇 오브젝트를 찾지 못했다");
        return go;
    }

    static void WriteReport(GameObject robot, GameObject green)
    {
        var arts = robot.GetComponentsInChildren<ArticulationBody>(true);
        var sb = new System.Text.StringBuilder();
        sb.Append("{\n");
        sb.Append($"  \"unity\": \"{Application.unityVersion}\",\n");
        sb.Append($"  \"articulation_bodies\": {arts.Length},\n");
        sb.Append($"  \"revolute\": {arts.Count(a => a.jointType == ArticulationJointType.RevoluteJoint)},\n");
        sb.Append($"  \"colliders\": {robot.GetComponentsInChildren<Collider>(true).Length},\n");
        sb.Append($"  \"meshes\": {robot.GetComponentsInChildren<MeshFilter>(true).Length},\n");
        sb.Append($"  \"greenhouse\": {green.transform.childCount},\n");
        sb.Append("  \"frames\": {\n");
        var want = new[] { "tcp", "link0", "link6", "rg2_hand", "sensor1_camera_link" };
        var found = new List<string>();
        foreach (var n in want) {
            var t = FindDeep(robot.transform, n);
            if (t == null) continue;
            var p = t.position;
            found.Add($"    \"{n}\": [{p.x:F6}, {p.y:F6}, {p.z:F6}]");
        }
        sb.Append(string.Join(",\n", found));
        sb.Append("\n  }\n}\n");
        File.WriteAllText(Report, sb.ToString());
    }

    static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name) return root;
        foreach (Transform c in root) { var r = FindDeep(c, name); if (r != null) return r; }
        return null;
    }

    // ───────────────────────────── ROS 연동(2026-08-16 신설)
    const string ScenePath = "Assets/Scenes/RdaScene.unity";

    /// 🔴 ROS-TCP-Connector 의 ROS1/ROS2 는 **컴파일 타임 define** 으로 갈린다(`#if ROS2`).
    ///    GUI 에서는 Robotics ▸ ROS Settings 로 바꾸지만 배치에서는 여기서 넣어야 한다.
    ///    안 넣으면 연결·구독은 멀쩡히 되는데 **역직렬화만 깨진다**
    ///    (ArgumentOutOfRangeException, 수신 0건) — 원인이 가장 안 보이는 형태의 실패다.
    public static void SetupDefines()
    {
        try {
            var g = UnityEditor.BuildTargetGroup.Standalone;
            var cur = UnityEditor.PlayerSettings.GetScriptingDefineSymbolsForGroup(g) ?? "";
            if (!cur.Split(';').Contains("ROS2")) {
                UnityEditor.PlayerSettings.SetScriptingDefineSymbolsForGroup(
                    g, string.IsNullOrEmpty(cur) ? "ROS2" : cur + ";ROS2");
                Debug.Log("[RDA] scripting define 에 ROS2 추가");
            } else Debug.Log("[RDA] ROS2 define 이미 있음");
            AssetDatabase.SaveAssets();
            EditorApplication.Exit(0);
        } catch (Exception e) {
            Debug.LogError("[RDA] define 설정 실패: " + e);
            EditorApplication.Exit(1);
        }
    }

    /// 로봇+온실+ROS 브리지를 얹은 씬을 **에셋으로 저장**한다.
    /// RunAll 은 씬을 메모리에만 만들고 끝나 Play 할 것이 없었다 — 그래서 따로 둔다.
    public static void BuildScene()
    {
        try {
            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var robot = ImportRobot();
            GreenhouseBuilder.Build();

            var go = new GameObject("RdaRosBridge");
            var br = go.AddComponent<RdaRosBridge>();
            // 헤드리스 검증용 기본값 — 플레이어 실행 시 커맨드라인으로 덮어쓸 수 있게 남긴다.
            br.reportPath = Path.GetFullPath("rda_ros_bridge_report.json");
            br.quitAfter = 20f;

            Directory.CreateDirectory("Assets/Scenes");
            EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene(), ScenePath);
            Debug.Log($"[RDA] 씬 저장: {ScenePath} (로봇 {robot.name})");
            EditorApplication.Exit(0);
        } catch (Exception e) {
            Debug.LogError("[RDA] 씬 저장 실패: " + e);
            EditorApplication.Exit(1);
        }
    }

    /// 헤드리스(-batchmode -nographics) 로 돌릴 리눅스 플레이어를 빌드한다.
    /// ⚠ 사람이 Play 를 누르지 않아도 검증되도록 하려고 만든 것이다.
    public static void BuildPlayer()
    {
        try {
            var opts = new UnityEditor.BuildPlayerOptions {
                scenes = new[] { ScenePath },
                locationPathName = "Build/RdaPlayer",
                target = UnityEditor.BuildTarget.StandaloneLinux64,
                options = UnityEditor.BuildOptions.None,
            };
            var rep = UnityEditor.BuildPipeline.BuildPlayer(opts);
            var ok = rep.summary.result == UnityEditor.Build.Reporting.BuildResult.Succeeded;
            Debug.Log($"[RDA] 플레이어 빌드 {(ok ? "성공" : "실패")} — {rep.summary.totalErrors} errors");
            EditorApplication.Exit(ok ? 0 : 1);
        } catch (Exception e) {
            Debug.LogError("[RDA] 플레이어 빌드 실패: " + e);
            EditorApplication.Exit(1);
        }
    }
}
'''


def build(out_dir, urdf_path, obstacles, mesh_search):
    assets = os.path.join(out_dir, "Assets", "RdaRobot")
    editor = os.path.join(out_dir, "Assets", "Editor")
    os.makedirs(assets, exist_ok=True)
    os.makedirs(editor, exist_ok=True)
    os.makedirs(os.path.join(out_dir, "Packages"), exist_ok=True)
    os.makedirs(os.path.join(out_dir, "ProjectSettings"), exist_ok=True)

    # ① URDF + mesh — package:// 를 그대로 두고 폴더 이름을 패키지 이름으로 맞춘다
    urdf = open(urdf_path).read()
    copied, missing = 0, []
    for uri in sorted(set(re.findall(r'filename="(package://[^"]+)"', urdf))):
        m = re.match(r"package://([^/]+)/(.+)", uri)
        src = G.resolve_pkg(uri, mesh_search)
        if not src:
            missing.append(uri)
            continue
        dst = os.path.join(assets, m.group(1), m.group(2))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
        copied += 1
    open(os.path.join(assets, "rda_robot.urdf"), "w").write(urdf)

    # ② 온실 — obstacles.yaml 을 그대로 JSON 으로(형상·좌표는 Gazebo/USD 와 같은 소스)
    OB = _import_sibling("_obstacle_publisher", "obstacle_publisher.py")
    spec = yaml.safe_load(open(obstacles)) or {}
    try:
        OB.expand_crops(spec)
    except Exception as e:                                    # noqa: BLE001
        sys.stderr.write(f"[gen_unity_assets] expand_crops 경고: {e}\n")
    items = []
    for o in spec.get("obstacles", []):
        if o.get("kind") == "keepout" or o.get("type") not in ("box", "cylinder", "sphere"):
            continue
        pose = o.get("pose") or {}
        it = dict(name=str(o["name"]), type=o["type"],
                  xyz=[float(v) for v in pose.get("xyz", [0, 0, 0])],
                  rpy=[float(v) for v in pose.get("rpy", [0, 0, 0])],
                  color=[float(v) for v in (o.get("color") or [0.6, 0.6, 0.6, 1.0])])
        if o["type"] == "box":
            it["size"] = [float(v) for v in o["size"]]
        elif o["type"] == "cylinder":
            it["radius"], it["height"] = float(o["radius"]), float(o["height"])
        else:
            it["radius"] = float(o["radius"])
        items.append(it)
    json.dump({"items": items}, open(os.path.join(assets, "greenhouse.json"), "w"),
              ensure_ascii=False, indent=1)

    # ③ 에디터 스크립트 · 프로젝트 뼈대
    open(os.path.join(editor, "GreenhouseBuilder.cs"), "w").write(GREENHOUSE_CS)
    open(os.path.join(editor, "RdaBatch.cs"), "w").write(BATCH_CS)
    # 🔴 런타임 스크립트는 Assets/Editor 밖에 둬야 한다 — Editor 폴더 안의 코드는
    #    빌드된 플레이어에 들어가지 않아 MonoBehaviour 가 통째로 사라진다.
    runtime = os.path.join(out_dir, "Assets", "RdaRuntime")
    os.makedirs(runtime, exist_ok=True)
    open(os.path.join(runtime, "RdaRosBridge.cs"), "w").write(BRIDGE_CS)

    # ROS 관절 이름 → Unity 오브젝트(=URDF 자식 링크) 이름.
    # URDF Importer 가 ArticulationBody 를 자식 링크 오브젝트에 붙이기 때문에 필요하다.
    # Resources 에 둬야 런타임에 Resources.Load 로 읽힌다.
    res = os.path.join(out_dir, "Assets", "Resources")
    os.makedirs(res, exist_ok=True)
    jmap = {}
    for jn, j in G.parse_urdf(urdf_path)[1].items():
        if j["type"] != "fixed":
            jmap[jn] = j["child"]
    json.dump(jmap, open(os.path.join(res, "joint_map.json"), "w"), indent=1)
    json.dump({"dependencies": {
        "com.unity.robotics.urdf-importer": URDF_IMPORTER,
        "com.unity.robotics.ros-tcp-connector": ROS_TCP_CONNECTOR,
        "com.unity.modules.physics": "1.0.0",
        "com.unity.modules.imgui": "1.0.0",
        "com.unity.modules.jsonserialize": "1.0.0",
        # URDF Importer 의 UnityMeshImporter 가 Texture2D.LoadImage 를 쓴다.
        # 빠지면 패키지 컴파일이 error CS1061 로 깨져 배치모드가 통째로 실패한다
        # (2026-08-16 Unity 2022.3.62f3 실행에서 실측 — 로컬 검증으론 안 잡혔다).
        "com.unity.modules.imageconversion": "1.0.0",
    }}, open(os.path.join(out_dir, "Packages", "manifest.json"), "w"), indent=2)
    open(os.path.join(out_dir, "ProjectSettings", "ProjectVersion.txt"), "w").write(
        f"m_EditorVersion: {UNITY_VERSION}\n")
    # 🔴 ROS-TCP-Connector 의 ROS1/ROS2 는 **컴파일 타임 define**(`#if ROS2`)이다.
    #    안 넣으면 연결·구독은 되는데 **역직렬화만 깨진다**(수신 0건 · 원인이 안 보인다).
    #    에디터로 넣어도(RdaBatch.SetupDefines) 이 생성기가 재생성 때 ProjectSettings 를
    #    통째로 지우므로 매번 날아간다 → **생성기가 직접 써서 재생성에도 살아남게 한다.**
    #    (2026-08-16 실측으로 확인한 재발 경로)
    open(os.path.join(out_dir, "ProjectSettings", "ProjectSettings.asset"), "w").write(
        "%YAML 1.1\n"
        "%TAG !u! tag:unity3d.com,2011:\n"
        "--- !u!129 &1\n"
        "PlayerSettings:\n"
        "  m_ObjectHideFlags: 0\n"
        "  serializedVersion: 24\n"
        "  productName: RdaRobot\n"
        "  companyName: RDA\n"
        "  scriptingDefineSymbols:\n"
        "    Standalone: ROS2\n")
    return dict(meshes=copied, missing=missing, items=len(items))


README = """# Unity 프로젝트 (RDA 농업로봇) — 자동 생성물

생성기 = `rda_robot_bringup/scripts/gen_unity_assets.py`.
좌표·형상은 Gazebo 월드·Isaac USD 와 **같은 소스**(통합 URDF · obstacles.yaml)에서 나온다.

## 여는 법
1. Unity Hub 에서 **이 폴더**를 프로젝트로 추가 → Unity {ver} 로 연다.
   (첫 실행에 URDF Importer 패키지를 git 에서 받으므로 네트워크가 필요하다.)
2. `Assets/RdaRobot/rda_robot.urdf` 우클릭 → **Import Robot from Selected URDF file**
   · Mesh Orientation = **Y axis** 아님 주의: URDF 는 Z-up 이므로 기본값 그대로 둔다.
   · Collision Mesh Decomposition = VHACD 를 쓰면 오래 걸린다(기본 Convex 권장).
3. 메뉴 **RDA ▸ 온실 세우기** → `greenhouse.json` 의 primitive {n}개가 씬에 선다.

## 배치모드 검증(GUI 없이)
```bash
<Unity>/Editor/Unity -batchmode -nographics -quit \\
  -projectPath <이 폴더> -executeMethod RdaBatch.RunAll -logFile /tmp/unity.log
python3 src/docs/scripts/test_unity_export.py --project <이 폴더>
```
`rda_unity_report.json`(관절·충돌체·프레임 위치)을 URDF FK 와 대조한다.

## 알려진 한계
* URDF Importer 는 **v0.5.2**(2022년) — Unity {ver} 에서 경고가 날 수 있다.
* `mimic`(rg2_finger_joint2)은 URDF Importer 가 무시한다 → 손가락 2개가 독립 관절.
* 재질은 URDF 의 색만 반영된다(텍스처 없음). mesh 는 .dae/.stl 그대로 임포트.
* 온실은 **정적 primitive**(GameObject + Collider)로 세운다 — 물리 강체가 아니다.
* Unity 실행에는 **라이선스 활성화**가 필요하다(Personal 이면 Unity 계정 로그인).
"""


def main():
    ap = argparse.ArgumentParser(description="통합 URDF + obstacles.yaml → Unity 프로젝트")
    ap.add_argument("--out", default=os.path.expanduser("~/robot_ws/export/unity"))
    ap.add_argument("--urdf", default=None, help="생략하면 mounts.yaml 로 즉석 조립")
    ap.add_argument("--obstacles", default=None)
    ap.add_argument("--mesh-path", action="append", default=[])
    a = ap.parse_args()

    out_dir = os.path.abspath(os.path.expanduser(a.out))
    keep = os.path.join(out_dir, "Library")            # Unity 캐시는 살려 둔다(재임포트 비용)
    if os.path.isdir(out_dir):
        for n in os.listdir(out_dir):
            if os.path.join(out_dir, n) == keep:
                continue
            p = os.path.join(out_dir, n)
            shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)
    os.makedirs(out_dir, exist_ok=True)

    urdf = a.urdf or G.compose_urdf(os.path.join(out_dir, "rda_robot.urdf"))
    obstacles = a.obstacles or os.path.join(
        _here, "..", "..", "rda_robot_description", "config", "obstacles.yaml")
    if not os.path.exists(obstacles):
        from ament_index_python.packages import get_package_share_directory
        obstacles = os.path.join(get_package_share_directory("rda_robot_description"),
                                 "config", "obstacles.yaml")

    st = build(out_dir, urdf, obstacles, a.mesh_path)
    open(os.path.join(out_dir, "README.md"), "w").write(
        README.format(ver=UNITY_VERSION, n=st["items"]))
    sys.stderr.write(f"[gen_unity_assets] mesh {st['meshes']}개 복사 · 온실 {st['items']}개 · "
                     f"출력 {out_dir}\n")
    if st["missing"]:
        sys.stderr.write(f"[gen_unity_assets] ⚠ mesh 를 못 찾음: {st['missing']}\n")


if __name__ == "__main__":
    main()
