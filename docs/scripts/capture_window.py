#!/usr/bin/env python3
"""창 캡처 — 보고서·README 용 스크린샷 (7주차 마무리)

이 PC 에는 `import`(ImageMagick)·`scrot`·`gnome-screenshot`·`ffmpeg` 이 없다. 대신 조립기 GUI 때문에
**PyQt5 가 이미 깔려 있어** `QScreen.grabWindow()` 로 창을 그대로 뜬다(추가 설치 없음).

사용:
  python3 docs/scripts/capture_window.py --name RViz --out docs/images/3-7_execute.png
  python3 docs/scripts/capture_window.py --full --out /tmp/screen.png
  python3 docs/scripts/capture_window.py --list          # 잡히는 창 목록만 보기

⚠ 캡처 대상 창이 가려져 있으면 X11 에서 옛 내용이 찍힐 수 있다 → 기본으로 창을 앞으로 올린 뒤
   `--settle` 만큼 기다렸다 찍는다(`--no-raise` 로 끌 수 있다).
"""
import argparse
import os
import subprocess
import sys
import time


def windows():
    """xdotool 로 (id, 이름) 목록. 이름 없는 창·크기 0 은 뺀다."""
    out = subprocess.run(["xdotool", "search", "--onlyvisible", "--name", "."],
                         capture_output=True, text=True).stdout.split()
    res = []
    for wid in out:
        name = subprocess.run(["xdotool", "getwindowname", wid],
                              capture_output=True, text=True).stdout.strip()
        geo = subprocess.run(["xdotool", "getwindowgeometry", wid],
                             capture_output=True, text=True).stdout
        if not name:
            continue
        res.append((wid, name, geo.replace("\n", " ")))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", help="창 이름(부분 일치)")
    ap.add_argument("--winid", help="창 id 직접 지정")
    ap.add_argument("--full", action="store_true", help="화면 전체")
    ap.add_argument("--out", default="/tmp/capture.png")
    ap.add_argument("--delay", type=float, default=0.0, help="시작 전 대기(초)")
    ap.add_argument("--settle", type=float, default=0.8, help="창을 올린 뒤 대기(초)")
    ap.add_argument("--no-raise", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--size", help="캡처 전에 창 크기를 WxH 로 바꾼다(예: 1920x1080). "
                                   "발표자료용 가로형 1600px+ 요구를 맞추려고 추가(2026-08-18)")
    ap.add_argument("--move", help="캡처 전에 창을 X,Y 로 옮긴다(예: 0,0 — 듀얼 모니터에서 잘림 방지)")
    a = ap.parse_args()

    if a.list:
        for wid, name, geo in windows():
            print(f"{wid:>10}  {name}   [{geo.strip()}]")
        return

    if a.delay:
        time.sleep(a.delay)

    wid = 0
    if not a.full:
        if a.winid:
            wid = int(a.winid)
        else:
            cands = [w for w in windows() if a.name.lower() in w[1].lower()]
            if not cands:
                print(f"[capture] '{a.name}' 창을 못 찾음. --list 로 확인할 것", file=sys.stderr)
                sys.exit(2)
            cands.sort(key=lambda w: len(w[1]))
            wid = int(cands[0][0])
            print(f"[capture] 창 선택: {cands[0][1]} (id {wid})", file=sys.stderr)
        # 🔴 크기·위치를 먼저 바꾸고 나서 올린다 — 순서를 바꾸면 리사이즈 재그리기가
        #    캡처에 걸려 화면 절반이 빈 채로 찍힌다(2026-08-18 실측).
        if a.move:
            x, y = a.move.split(",")
            subprocess.run(["xdotool", "windowmove", str(wid), x, y], capture_output=True)
        if a.size:
            w, h = a.size.lower().split("x")
            subprocess.run(["xdotool", "windowsize", str(wid), w, h], capture_output=True)
            time.sleep(1.5)                      # 리사이즈 후 재그리기 대기
        if not a.no_raise:
            subprocess.run(["xdotool", "windowactivate", "--sync", str(wid)],
                           capture_output=True)
            subprocess.run(["xdotool", "windowraise", str(wid)], capture_output=True)
            time.sleep(a.settle)

    from PyQt5.QtWidgets import QApplication
    app = QApplication([])                       # noqa: F841 — grabWindow 에 필요
    screen = QApplication.primaryScreen()
    pix = screen.grabWindow(wid)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    if not pix.save(a.out):
        print("[capture] 저장 실패", file=sys.stderr)
        sys.exit(1)
    print(f"[capture] {a.out}  {pix.width()}x{pix.height()}")


if __name__ == "__main__":
    main()
