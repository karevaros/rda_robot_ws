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
    json.dump({"dependencies": {
        "com.unity.robotics.urdf-importer": URDF_IMPORTER,
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
