# 이 저장소에서 `yolo/` 의 위치

원본: [roboseasy/YOLO](https://github.com/roboseasy/YOLO) (공개), 커밋 `0ac5218`.
파이프라인 자체의 사용법은 같은 폴더의 [README.md](README.md) 를 볼 것 — 원본을
그대로 두었다. 이 문서는 **약통 집기 전체 흐름에서 어디에 들어가는지**만 적는다.

## 셋으로 나뉜다

```
yolo/     모델을 만든다        촬영 → 분할 → 라벨링 → 학습 → best.pt
motion/   그 모델로 집는다     검출 → 접근 → 팔 → 손목 서보 → 판정
nav/      오가는 것을 맡는다   SLAM → AMCL → Nav2 → 왕복 복귀
```

`yolo/` 는 **제어를 하지 않는다.** 가중치 파일(`best.pt`) 하나를 만들어 내고
끝이다. 그 파일을 `motion/` 이 받아서 쓴다.

```python
# motion/services/pickplace/yolo_detect.py 가 받는 지점
YoloArgs(path="yolo/outputs/runs/pill/weights/best.pt", conf=0.4)
```

## `04.inference.py` 는 최종 경로가 아니다

화면에 상자를 그려 보여 주는 **확인용 뷰어**다. 좌표를 밖으로 내보내지 않으므로
집기에는 못 쓴다. 모델이 제대로 학습됐는지 눈으로 볼 때만 쓴다.

실제 추론은 `motion/services/pickplace/yolo_detect.py` 의 `infer()` 가 한다 —
front/wrist 두 뷰를 함께 처리하고 결과를 `Detection` 목록으로 돌려준다.

## 노트북에서 돈다

카메라는 로봇에 붙어 있지만 영상은 ZMQ 로 노트북에 온다. 로봇(라즈베리파이)은
YOLO 를 돌릴 성능이 없다.

```
로봇: lekiwi_host 가 카메라를 열어 JPEG 를 ZMQ(:5556)로 발행
노트북: frame_source.LeKiwiStream 이 받아서 추론
```

실측(2026-09-12, eduroam 무선): **front 28.0 FPS, wrist 28.7 FPS, 640x480**.
랜선 없이도 충분하다.

**한 host 에 클라이언트는 하나만 붙일 것.** ZMQ 가 프레임을 나눠 주므로 둘이
붙으면 각자 절반만 받아 둘 다 끊긴다. 촬영·추론·집기 중 하나만 실행한다.

## 로봇 주소를 바꿔 쓸 것

원본의 기본값은 랜선 직결 시절 주소다. 파일마다 다르기도 하다.

| 파일 | 기본 `--remote` |
|---|---|
| `frame_source.py` | `10.42.0.61` |
| `01.take_video.py` | `192.168.0.201` |
| `04.inference.py` | `192.168.0.201` |

무선을 쓰면 DHCP 주소라 매번 다르다. `--remote` 로 넘기거나 `lekiwi01.local`
(avahi) 을 쓰는 편이 낫다.

```bash
python 01.take_video.py --remote=<로봇 주소>
```

## 추론 기본 장치가 GPU 다

`04.inference.py` 와 `03.train.py` 의 `DEFAULT_DEVICE = "0"` 이다. CUDA 가 없는
노트북에서는 `--device cpu` 를 붙여야 한다. 학습은 GPU 가 사실상 필수다.

3070 8GB 에서는 `--batch 8` 을 권한다(기본 16 은 메모리가 빠듯하다).

## 클래스 이름은 맞출 필요가 없다

`motion/` 쪽은 **화면에서 가장 큰 검출**을 목표로 삼는다
(`approach.largest()`). 라벨 이름이 무엇이든 상관없고, 여러 종류를 학습시켰다면
`YoloArgs.classes=[인덱스]` 로 특정 클래스만 보게 할 수 있다.

## 데이터셋 받기

라벨링은 Roboflow 에서 한다. export 한 폴더를 `yolo/dataset/` 에 풀면
`03.train.py` 가 `dataset/data.yaml` 을 찾는다.

`dataset/`, `outputs/`, `*.pt` 는 `.gitignore` 에 있어 저장소에 올라가지 않는다.
여러 사람이 같은 데이터를 쓰려면 허깅페이스 데이터셋으로 주고받는 것이 편하다.

```bash
python -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='<계정>/pill-dataset', repo_type='dataset',
                  local_dir='yolo/dataset')
"
```
