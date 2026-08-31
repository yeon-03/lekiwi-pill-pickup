#!/usr/bin/env python
"""바퀴 명령의 '실제' 방향을 한 번 측정해서 파일에 저장한다.

■ 왜 필요한가 (2026-08-27, 하루를 날린 원인)
  base_nudge.py 주석에는 x=전진(+), y=좌(+), theta=좌회전(+) 이라고 적혀 있다.
  그런데 실제로 돌려보면 회전이 반대로 동작했다. 부호를 확정하지 않은 채
  제어 루프를 돌리면, 오차를 줄이려는 명령이 오히려 오차를 키우고, 실패할
  때마다 로봇이 판에서 조금씩 더 멀어진다. 결국 판을 등지고 방을 가로질러
  도망갔다. 루프의 '자기교정'은 병이 화면에서 사라지면 비교 대상이 없어
  작동하지 못한다 — 그래서 사후 교정이 아니라 사전 측정이어야 한다.

■ 측정 방법
  병 3개가 다 보이는 상태에서 축마다 짧은 펄스를 주고, 화면이 어느 쪽으로
  움직이는지 본다. 베이스캠은 로봇에 붙어 있으므로 헤딩과 무관하게 성립한다.
    theta : 가운데 병의 가로 위치가 어디로 밀리는가
    y     : 좌우 박스 크기 균형이 어느 쪽으로 바뀌는가
    x     : 좌우 여백 합이 늘어나는가 줄어드는가(줄면 전진)
  각 축마다 펄스를 준 만큼 되돌려서 위치를 보존한다.

사용법:  ./venv/bin/python scripts/calibrate_base_signs.py
결과:    config/base_signs.json  (approach_board.py 가 기동 시 읽는다)
"""
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import approach_board as ab  # noqa: E402
from robot_link import RobotLink  # noqa: E402

OUT = Path(__file__).resolve().parent.parent / 'config' / 'base_signs.json'
SETTLE = 0.6
READS = 3


def _read(field):
    """같은 값을 여러 번 읽어 중앙값 — 한 프레임 튀는 것에 안 속게."""
    vals = []
    for _ in range(READS):
        st = ab.approach_state()
        if st is None or st['n'] < ab.REQUIRED_BOTTLES:
            return None
        vals.append(field(st))
        time.sleep(0.15)
    return statistics.median(vals)


def _probe(client, label, field, kwargs, dur, min_delta):
    """펄스를 주고 field 가 얼마나 변하는지 재고, 그만큼 되돌린다."""
    before = _read(field)
    if before is None:
        print(f'  {label}: 병 3개가 안 보여 측정 불가')
        return None
    ab.pulse(client, duration=dur, **kwargs)
    time.sleep(SETTLE)
    after = _read(field)
    # 되돌리기 — 측정 때문에 위치가 틀어지지 않게
    back = {k: -v for k, v in kwargs.items()}
    ab.pulse(client, duration=dur, **back)
    time.sleep(SETTLE)
    if after is None:
        print(f'  {label}: 펄스 후 병을 놓쳐 측정 불가')
        return None
    delta = after - before
    ok = abs(delta) >= min_delta
    print(f'  {label}: {before:+.1f} -> {after:+.1f}  (변화 {delta:+.1f})'
          f'{"" if ok else "  ← 변화가 너무 작아 못 믿음"}')
    return delta if ok else None


def main() -> None:
    client = RobotLink('192.168.0.201')
    client.connect()
    try:
        st = ab.approach_state()
        print(f'시작 상태: {ab.describe(st)}')
        if st is None or st['n'] < ab.REQUIRED_BOTTLES:
            raise SystemExit('병 3개가 다 보이는 위치에서 실행하세요')

        print('\n[1/3] theta (+) 를 주면 가운데 병이 화면에서 어디로 가는가')
        d_theta = _probe(client, 'theta +', lambda s: s['mid_off'],
                         {'vtheta': 22.0}, 0.35, 12.0)

        if d_theta is None:
            raise SystemExit('theta 측정 실패 — 병 3개가 안정적으로 보이는 자리에서')
        # 남은 측정을 하려면 병들이 화면 한가운데 있어야 한다. 한쪽 끝에 붙어
        # 있으면 전진 한 번에 시야 밖으로 나가서 측정이 깨진다(실측).
        theta_sign = -1.0 if d_theta > 0 else 1.0
        for _ in range(6):
            cur = ab.approach_state()
            if cur is None or not cur['mid_color_seen']:
                break
            off = cur['mid_off']
            if abs(off) <= 45:
                break
            print(f'  (측정 전 정렬) 가운데오차 {off:+.0f}px — 회전으로 중앙 이동')
            ab.pulse(client, vtheta=22.0 * theta_sign * (1 if off > 0 else -1),
                     duration=min(0.45, max(0.15, abs(off) / (98.5 / 0.35))))
            time.sleep(SETTLE)
        print(f'  정렬 후: {ab.describe(ab.approach_state())}')

        # ■ y 는 '균형 변화'로 재면 안 된다 — 이미 균형이 맞아 있으면 변화가
        #   0 이라 부호를 못 정한다(실측: 변화 -0.1). 대신 '화면이 어느 쪽으로
        #   밀리는가'로 잰다. 카메라가 로봇에 붙어 있으니 로봇이 왼쪽으로 가면
        #   장면은 오른쪽으로 밀린다(mid_off 증가). 그리고 왼쪽으로 가면
        #   왼쪽 병이 가까워져 커진다 — 이건 기하학적으로 항상 참이다.
        print('\n[2/3] y (+) 를 주면 화면이 어느 쪽으로 밀리는가')
        d_y = _probe(client, 'y +', lambda s: s['mid_off'],
                     {'vy': 0.07}, 0.35, 12.0)

        print('\n[3/3] x 를 주면 좌우 여백 합이 늘어나는가 줄어드는가')
        gapsum = lambda s: s['left_gap'] + s['right_gap']   # noqa: E731
        d_x = _probe(client, 'x +', gapsum, {'vx': 0.07}, 0.30, 12.0)
        if d_x is None:
            # 앞으로 가다 병을 놓치면 뒤로 재고 부호를 뒤집는다
            print('  x + 로는 못 쟀음 — x - 로 다시 시도')
            d_back = _probe(client, 'x -', gapsum, {'vx': -0.07}, 0.30, 12.0)
            if d_back is not None:
                d_x = -d_back

        if None in (d_y, d_x):
            raise SystemExit('\n측정 실패 — 병 3개가 안정적으로 보이는 자리에서 '
                             '다시 실행하세요')

        signs = {
            # 오차(mid_off)를 줄이려면 theta 에 어떤 부호를 곱해야 하는가.
            # d_theta > 0 이면 theta+ 가 mid_off 를 키우므로, 줄이려면 반대.
            'theta_to_reduce_mid_off': -1.0 if d_theta > 0 else 1.0,
            # d_y > 0 이면 y+ 로 장면이 오른쪽으로 밀린다 = 로봇이 왼쪽으로
            # 갔다는 뜻 = 왼쪽 병이 가까워져 커진다.
            'y_to_grow_left': 1.0 if d_y > 0 else -1.0,
            # 판에 가까워지려면(여백 합을 줄이려면) x 에 어떤 부호를?
            'x_forward': -1.0 if d_x > 0 else 1.0,
            'measured': {'d_theta_px': d_theta, 'd_balance': d_y,
                         'd_gapsum_px': d_x},
            'note': 'approach_board.py 가 기동 시 읽어 부호로 쓴다',
        }
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(json.dumps(signs, indent=2, ensure_ascii=False))
        print(f'\n저장: {OUT}')
        for k in ('theta_to_reduce_mid_off', 'y_to_grow_left', 'x_forward'):
            print(f'  {k} = {signs[k]:+g}')
    finally:
        try:
            ab.send_base(client, 0.0, 0.0, 0.0)
        except Exception:
            pass
        client.disconnect()


if __name__ == '__main__':
    main()
