#!/usr/bin/env python3
"""전략·planner 비교 측정(bench_strategy) 원자료 → 보고용 마크다운 표.

사용:  bench_strategy_report.py <bench_strategy.csv> [출력.md]
       (원자료는 `pregrasp_demo.launch.py bench_strategy:=true` 가 남긴다)

집계 원칙(측정 코드와 동일):
  · 성공 = IK + 계획 산출     · 유효 = 충돌free · 접근 검증됨 · 폴백(2점 보간) 없음
  · 경로길이·계획시간의 중앙값/평균은 **유효 행만** 쓴다(폴백 행은 계획 없이 관절을
    직선보간한 것이라 길이를 과소평가한다).
표준편차 대신 중앙값·사분범위를 함께 낸다 — OMPL 은 확률적이라 분포가 치우친다.
"""
import csv
import statistics as st
import sys


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("t", "jl", "cl", "plan_t", "frac"):
            r[k] = num(r.get(k))
        for k in ("n", "bad", "plan_calls", "plan_fail", "rep"):
            r[k] = int(float(r[k])) if r.get(k) not in (None, "", "None") else None
        for k in ("ok", "checked", "fallback", "pre_fb", "home_fb"):
            r[k] = str(r.get(k, "")).strip().lower() in ("true", "1", "yes")
    return rows


def stats(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return None
    v.sort()
    q = (st.quantiles(v, n=4) if len(v) >= 4 else [v[0], st.median(v), v[-1]])
    return dict(n=len(v), mean=sum(v) / len(v), med=st.median(v), q1=q[0], q3=q[2])


def fmt(s, unit="", d=2):
    return "-" if s is None else f"{s['med']:.{d}f}{unit} ({s['q1']:.{d}f}–{s['q3']:.{d}f})"


def agg(rows):
    ok = [r for r in rows if r["ok"]]
    good = [r for r in ok if r["checked"] and not r["bad"] and not r["fallback"]]
    return dict(
        n=len(rows), ok=len(ok), good=len(good),
        t=stats([r["t"] for r in good]), t_all=stats([r["t"] for r in rows]),
        jl=stats([r["jl"] for r in good]), cl=stats([r["cl"] for r in good]),
        straight=sum(1 for r in ok if str(r["method"]).startswith("cartesian(")),
        fb=sum(1 for r in ok if r["fallback"]),
        bad=sum(r["bad"] for r in ok if r["bad"] is not None),
        pfail=sum(r["plan_fail"] for r in rows if r["plan_fail"] is not None))


def table(rows, keyfn, title, head):
    out = [f"### {title}", "",
           f"| {head} | 성공 | 유효 | 계획시간 중앙값(IQR) | 관절길이 rad | TCP길이 m | "
           "완전직선 | 폴백 | 충돌wp | 플래너실패 |",
           "|---|---|---|---|---|---|---|---|---|---|"]
    keys = []
    for r in rows:
        k = keyfn(r)
        if k not in keys:
            keys.append(k)
    for k in keys:
        a = agg([r for r in rows if keyfn(r) == k])
        out.append(
            f"| {k} | {a['ok']}/{a['n']} | {a['good']}/{a['n']} | {fmt(a['t'], 's')} | "
            f"{fmt(a['jl'])} | {fmt(a['cl'], '', 3)} | {a['straight']}/{a['ok'] or 1} | "
            f"{a['fb']} | {a['bad']} | {a['pfail']} |")
    out.append("")
    return out


def main():
    src = sys.argv[1]
    rows = load(src)
    fruits = sorted({r["name"] for r in rows})
    reps = max((r["rep"] or 0) for r in rows) + 1
    out = ["# 수확 전략 · OMPL planner 비교 측정", "",
           f"- 원자료: `{src}` — {len(rows)}회 계획",
           f"- 표본: 열매 {len(fruits)}개 × 반복 {reps} — {', '.join(fruits)}",
           "- 지표: 한 수확 사이클(home→pre→접근→파지→후퇴→home) 기준. 접근 구간은 "
           "후퇴에서 역재생되므로 길이에 2회 반영.",
           "- 성공 = IK+계획 · 유효 = 충돌free·접근 검증·폴백 없음 · 길이/시간 통계는 유효 행만.",
           ""]
    out += table(rows, lambda r: f"**{r['strategy']}**", "전략별(planner 통합)", "전략")
    out += table(rows, lambda r: r["planner"], "planner 별(전략 통합)", "planner")
    out += table(rows, lambda r: f"{r['strategy']} / {r['planner']}", "전략 × planner", "조합")
    md = "\n".join(out)
    if len(sys.argv) > 2:
        with open(sys.argv[2], "w") as f:
            f.write(md + "\n")
        print(f"저장: {sys.argv[2]}")
    else:
        print(md)


if __name__ == "__main__":
    main()
