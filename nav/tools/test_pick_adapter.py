#!/usr/bin/env python3
"""로봇 없이 pick_adapter.py 를 검증한다.

집기 담당자가 로봇을 잡기 전에 자기 노트북에서 먼저 돌려보는 용도다.
ROS2 설치가 제대로 됐는지, 어댑터가 규약대로 응답하는지까지 확인된다.
실기기·카메라·서보·라이다 전부 필요 없다.

    python3 nav/tools/test_pick_adapter.py

로봇이 켜져 있어도 안전하다 -- 로봇(42)·에이보(77)와 겹치지 않는 전용
도메인 91 에서, 이 컴퓨터 안에서만(ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
돈다. 도메인을 바꾸려면 TEST_DOMAIN_ID.

전부 통과하면 남은 것은 두 가지뿐이다.
  1) 노트북과 로봇이 서로 토픽을 보는가  ->  양쪽 ROS_DOMAIN_ID 를 42 로 맞추고
     ros2 topic list 에 /abo/state 가 뜨는지 확인
  2) 진짜 pick_cycle.py 가 --result-file 에 결과를 쓰는가

여기서 쓰는 가짜 pick_cycle.py 는 진짜와 같은 규약만 지킨다:
  - --color 는 red/green/blue
  - --result-file 경로에 {success: bool, ...} JSON 을 쓴다
  - 시작하자마자 이전 결과 파일을 지운다 (끝내지 못하고 죽으면 파일이 없다)
"""
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

HERE = Path(__file__).resolve().parent
ADAPTER = HERE.parent / "mission" / "pick_adapter.py"
RESULT = "/tmp/lekiwi_result_pill_pickup.json"

# 로봇은 42(lekiwi_profile.sh), 에이보는 77 을 쓴다. 시험이 진짜 미션에 끼어들지
# 않도록 전용 도메인에서, 네트워크로 나가지 않게 돈다.
DOMAIN = os.environ.get("TEST_DOMAIN_ID", "91")
LOCAL_ONLY = {"ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST", "ROS_STATIC_PEERS": ""}

# 가짜 집기. STUB_MODE 로 결과를 조종한다.
STUB = '''#!/usr/bin/env python3
import argparse, json, os, sys, time
from pathlib import Path
p = argparse.ArgumentParser()
p.add_argument("--color", required=True, choices=("red", "green", "blue"))
p.add_argument("--result-file", default="")
a, _ = p.parse_known_args()
if a.result_file:
    Path(a.result_file).unlink(missing_ok=True)   # 진짜 pick_cycle.py 와 같다
mode = os.environ.get("STUB_MODE", "ok")
time.sleep(float(os.environ.get("STUB_SEC", "0.3")))
if mode == "alternate":             # 호출마다 ok, fail 을 번갈아 (한 어댑터에서 성공 뒤 실패)
    cnt = Path(__file__).with_name("count")
    n = int(cnt.read_text()) if cnt.exists() else 0
    cnt.write_text(str(n + 1))
    mode = "ok" if n % 2 == 0 else "fail"
print(f"stub 진행 mode={mode}", flush=True)
if mode == "crash":                 # 결과를 안 쓰고 죽는다
    sys.exit(1)
if mode == "hang":                  # 제한시간을 넘긴다
    time.sleep(60)
Path(a.result_file).write_text(json.dumps({
    "skill": "pill_pickup", "color": a.color,
    "success": mode == "ok", "grasped": mode == "ok",
    "elapsed_sec": 0.3, "verdict_reason": "시험 " + mode,
}, ensure_ascii=False), encoding="utf-8")
'''


class Harness(Node):
    def __init__(self):
        super().__init__("test_pick_adapter")
        self.got = []
        self.create_subscription(Bool, "/abo/pick_done",
                                 lambda m: self.got.append(m.data), 10)
        self.pub = self.create_publisher(String, "/abo/pick_request", 10)

    def wait_adapter(self, sec=8.0):
        """어댑터가 구독을 붙일 때까지 기다린다."""
        t0 = time.time()
        while self.pub.get_subscription_count() == 0 and time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pub.get_subscription_count() > 0

    def collect(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)


def stub_repo(script="pick_cycle.py"):
    repo = Path(tempfile.mkdtemp(prefix="pickstub_"))
    (repo / "scripts").mkdir()
    (repo / "scripts" / script).write_text(STUB, encoding="utf-8")
    return repo


def start_adapter(repo, mode, timeout, stub_sec="0.3", extra=()):
    env = dict(os.environ, STUB_MODE=mode, STUB_SEC=stub_sec,
               ROS_DOMAIN_ID=DOMAIN, **LOCAL_ONLY)
    return subprocess.Popen(
        [sys.executable, str(ADAPTER), "--repo", str(repo),
         "--map", "center=green,left=red", "--timeout", str(timeout),
         "--python", sys.executable, *extra],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def stop(proc):
    proc.terminate()
    try:
        return proc.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.communicate()[0]


def show(out):
    for line in (out or "").splitlines()[-12:]:
        print("        " + line)


def case(name, mode, target, expect, timeout=8, wait=15.0):
    Path(RESULT).unlink(missing_ok=True)
    proc = start_adapter(stub_repo(), mode, timeout)
    out = ""
    try:
        h = Harness()
        if not h.wait_adapter():
            h.destroy_node()
            print(f"  [FAIL] {name}: 어댑터가 뜨지 않았다")
            show(stop(proc))
            return False
        h.got.clear()
        h.pub.publish(String(data=target))
        t0 = time.time()
        while not h.got and time.time() - t0 < wait:
            rclpy.spin_once(h, timeout_sec=0.1)
        got = h.got[0] if h.got else None
        h.destroy_node()
    finally:
        out = stop(proc)
    ok = got == expect
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: pick_done={got} (기대 {expect})")
    if not ok:
        show(out)
    return ok


def case_duplicate():
    """집는 중 들어온 중복 요청은 무시되어야 한다 -- 두 번 실행하면 팔이 엉킨다."""
    name = "집는 중 중복 요청 무시"
    Path(RESULT).unlink(missing_ok=True)
    proc = start_adapter(stub_repo(), "ok", 20, stub_sec="4")
    out = ""
    try:
        h = Harness()
        if not h.wait_adapter():
            h.destroy_node()
            print(f"  [FAIL] {name}: 어댑터가 뜨지 않았다")
            show(stop(proc))
            return False
        h.got.clear()
        h.pub.publish(String(data="center"))
        h.collect(1.0)                      # 집는 중(4초)
        h.pub.publish(String(data="center"))  # 한 번 더
        h.collect(9.0)
        n = len(h.got)
        h.destroy_node()
    finally:
        out = stop(proc)
    ok = n == 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: pick_done {n}회 (기대 1회)")
    if not ok:
        show(out)
    return ok


def case_explicit_color_overrides_map():
    """"center:red" 처럼 요청에 색이 실려오면 --map(center=green)보다 우선해야 한다.
    stub pick_cycle.py 가 실제로 받은 --color 를 결과 파일에 그대로 적어주므로
    그 필드로 확인한다(pick_done 자체는 색과 무관하게 mode=ok 면 항상 true라
    구분이 안 됨)."""
    name = "발화로 고른 색이 --map 보다 우선"
    Path(RESULT).unlink(missing_ok=True)
    proc = start_adapter(stub_repo(), "ok", 8)
    out = ""
    try:
        h = Harness()
        if not h.wait_adapter():
            h.destroy_node()
            print(f"  [FAIL] {name}: 어댑터가 뜨지 않았다")
            show(stop(proc))
            return False
        h.got.clear()
        h.pub.publish(String(data="center:red"))   # --map 은 center=green
        t0 = time.time()
        while not h.got and time.time() - t0 < 15.0:
            rclpy.spin_once(h, timeout_sec=0.1)
        h.destroy_node()
    finally:
        out = stop(proc)
    try:
        used_color = json.loads(Path(RESULT).read_text(encoding="utf-8"))["color"]
    except (OSError, ValueError, KeyError):
        used_color = None
    ok = used_color == "red"
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: 실제 사용된 색={used_color} (기대 red)")
    if not ok:
        show(out)
    return ok


def case_no_color_in_request_falls_back_to_map():
    """콜론 없는 요청("center")은 예전처럼 --map 값(green)을 그대로 써야 한다
    -- 기존 배치(웨이포인트별 고정 색)와의 하위호환 확인."""
    name = "색 없는 요청은 --map 으로 폴백"
    Path(RESULT).unlink(missing_ok=True)
    proc = start_adapter(stub_repo(), "ok", 8)
    out = ""
    try:
        h = Harness()
        if not h.wait_adapter():
            h.destroy_node()
            print(f"  [FAIL] {name}: 어댑터가 뜨지 않았다")
            show(stop(proc))
            return False
        h.got.clear()
        h.pub.publish(String(data="center"))
        t0 = time.time()
        while not h.got and time.time() - t0 < 15.0:
            rclpy.spin_once(h, timeout_sec=0.1)
        h.destroy_node()
    finally:
        out = stop(proc)
    try:
        used_color = json.loads(Path(RESULT).read_text(encoding="utf-8"))["color"]
    except (OSError, ValueError, KeyError):
        used_color = None
    ok = used_color == "green"
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: 실제 사용된 색={used_color} (기대 green)")
    if not ok:
        show(out)
    return ok


def case_pick_script_option():
    """--pick-script 로 다른 집기 스크립트(pick_worker_cycle.py)를 부를 수 있어야 한다.
    stub 은 scripts/pick_worker_cycle.py 로만 만들어 두므로, 기본 경로를 부르면 실패한다."""
    name = "--pick-script 로 다른 집기 스크립트 실행"
    Path(RESULT).unlink(missing_ok=True)
    proc = start_adapter(stub_repo("pick_worker_cycle.py"), "ok", 8,
                         extra=("--pick-script", "scripts/pick_worker_cycle.py"))
    out = ""
    try:
        h = Harness()
        if not h.wait_adapter():
            h.destroy_node()
            print(f"  [FAIL] {name}: 어댑터가 뜨지 않았다")
            show(stop(proc))
            return False
        h.got.clear()
        h.pub.publish(String(data="center:red"))
        t0 = time.time()
        while not h.got and time.time() - t0 < 15.0:
            rclpy.spin_once(h, timeout_sec=0.1)
        got = h.got[0] if h.got else None
        h.destroy_node()
    finally:
        out = stop(proc)
    ok = got is True and "pick_worker_cycle.py" in out
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: pick_done={got}")
    if not ok:
        show(out)
    return ok


def case_success_then_failure():
    """한 어댑터가 성공 뒤 실패를 처리해도 죽지 않아야 한다.
    2026-09-14 실기기: 성공(info) 뒤 실패(warn)를 같은 로그 줄에서 남기다 rclpy 가
    'Logger severity cannot be changed between calls' 를 내며 어댑터가 종료됐다.
    실행마다 --run-log-dir 에 집기 스크립트 출력 파일도 남아야 한다."""
    name = "성공 뒤 실패 -> 어댑터 계속 동작 + 실행 로그 파일 2개"
    Path(RESULT).unlink(missing_ok=True)
    logdir = Path(tempfile.mkdtemp(prefix="pickruns_"))
    proc = start_adapter(stub_repo(), "alternate", 8, extra=("--run-log-dir", str(logdir)))
    out, got, alive = "", [], False
    try:
        h = Harness()
        if not h.wait_adapter():
            h.destroy_node()
            print(f"  [FAIL] {name}: 어댑터가 뜨지 않았다")
            show(stop(proc))
            return False
        for _ in range(2):
            n0 = len(h.got)
            h.pub.publish(String(data="center"))
            t0 = time.time()
            while len(h.got) == n0 and time.time() - t0 < 15.0:
                rclpy.spin_once(h, timeout_sec=0.1)
            h.collect(1.5)                 # on_tick 이 로그를 남기고 busy 를 푸는 시간
        got = list(h.got)
        h.destroy_node()
        alive = proc.poll() is None
    finally:
        out = stop(proc)
    logs = sorted(logdir.glob("*_green.log"))
    logged = all("stub 진행" in f.read_text(encoding="utf-8") for f in logs)
    ok = got == [True, False] and alive and len(logs) == 2 and logged
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: pick_done={got} 어댑터살아있음={alive} "
          f"로그파일={len(logs)}개")
    if not ok:
        show(out)
    return ok


def main():
    if not ADAPTER.is_file():
        print(f"pick_adapter.py 를 찾지 못했다: {ADAPTER}")
        return 1
    # 로봇이 쓰는 도메인(42)을 쓰면 이 시험이 발행하는 가짜 /abo/pick_done 이
    # 진행 중인 진짜 미션에 섞여 들어간다. 그래서 셸 값을 물려받지 않고
    # 전용 도메인으로 못 박는다. 바꿔야 하면 TEST_DOMAIN_ID 로.
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ.update(LOCAL_ONLY)
    print(f"pick_adapter 규약 검증 (로봇 불필요)  ROS_DOMAIN_ID={DOMAIN}")
    rclpy.init()
    try:
        r = [
            case("집기 성공 -> true", "ok", "center", True),
            case("집기 실패 -> false", "fail", "center", False),
            case("결과파일 없이 죽음 -> false", "crash", "center", False),
            case("매핑 없는 목적지 -> false", "ok", "nowhere", False),
            case("두 번째 매핑도 동작(left=red)", "ok", "left", True),
            case("제한시간 초과 -> false", "hang", "center", False, wait=25.0),
            case_duplicate(),
            case_explicit_color_overrides_map(),
            case_no_color_in_request_falls_back_to_map(),
            case_pick_script_option(),
            case_success_then_failure(),
        ]
    finally:
        rclpy.shutdown()
    print(f"\n{sum(r)}/{len(r)} 통과")
    if all(r):
        print("어댑터는 정상이다. 다음: 로봇과 같은 ROS_DOMAIN_ID 로 맞추고 "
              "ros2 topic list 에 /abo/state 가 보이는지 확인할 것.")
    return 0 if all(r) else 1


if __name__ == "__main__":
    sys.exit(main())
