# lekiwi01 집기 자세

`pick_worker_cycle.py --poses-dir` 가 읽는 팔 자세입니다. 값은 lerobot 캘리브레이션
(`lekiwi01.json`, `use_degrees=True`) 기준 **관절 도(°)**, 그리퍼는 **0~100** 입니다.

| 파일 | 쓰임 |
|---|---|
| `home.json` | 주행·대기 자세 |
| `pre_pick.json` | 약통 위 준비 자세 (손목캠에 약통이 보이는 위치) |
| `grasp.json` | 약통을 잡는 높이, 그리퍼 활짝 열림 |
| `grasp_closed.json` | **그리퍼 값만** 쓰인다 (닫는 정도) |

## 2026-09-14 다시 저장한 이유

이전 `pre_pick`(shoulder_pan +5.98) 에서 `grasp`(−5.10) 로 내려가는 동안 팔이 약 11° 왼쪽으로
쓸리며 약통을 밀었습니다. 손목 서보가 밀린 약통을 따라가다 pan 보정 한계(±20°)에 걸려 실패했습니다.

그래서 `pre_pick` 과 `grasp` 의 **shoulder_pan 을 같게**(−6.68 / −6.07, 차이 0.62°) 다시 잡아,
어깨·팔꿈치만 움직여 곧게 내려가게 했습니다. `grasp` 그리퍼는 이전 값 99.45(활짝 열림)를 유지하고,
`grasp_closed` 는 그리퍼만 22.13 으로 바꿨습니다(이전 17.80). `home` 은 그대로입니다.

## 쓰는 법

워커는 이 폴더를 바로 읽어도 되고, 기존처럼 `~/.PhysicalLabs` 에 복사해 써도 됩니다.

```bash
# 바로 읽기
--poses-dir ~/lekiwi-pill-pickup/scripts/poses/lekiwi01

# 또는 복사 (기존 파일은 먼저 백업)
cp -r ~/.PhysicalLabs/pickplace/lekiwi01/poses ~/.PhysicalLabs/pickplace/lekiwi01/poses_backup_$(date +%Y%m%d_%H%M%S)
cp scripts/poses/lekiwi01/*.json ~/.PhysicalLabs/pickplace/lekiwi01/poses/
```

로봇이나 캘리브레이션이 바뀌면 값이 맞지 않으니, 그 로봇에서 다시 저장하세요.
