# YOLO: 데이터 수집부터 라벨링 그리고 모델 학습과 추론까지

YOLO 기반 객체 탐지 프로젝트의 전체 파이프라인을 다룹니다. 웹캠으로 직접 데이터를 수집하고, 프레임 단위로 분할해 라벨링한 뒤, YOLO 모델을 학습시켜 추론까지 수행하는 과정을 단계별로 정리합니다.

## 프로젝트 구조

```
yolo/
├── frame_source.py           # 프레임 출처(로컬 웹캠 / LeKiwi ZMQ 스트림)
├── 01.take_video.py          # mp4 동영상 촬영
├── 02.splite_video2pic.py    # 동영상을 프레임 단위 이미지로 분할
├── 03.train.py               # YOLO 학습 + 검증
├── 04.inference.py           # 학습한 가중치로 실시간 추론
├── 05.upload_hf.py           # 학습한 가중치를 Hugging Face Hub 에 업로드
├── dataset/                  # 라벨링된 학습 데이터 (data.yaml)
├── outputs/                  # 생성물 저장소 (git 추적 제외)
│   ├── videos/               # 촬영된 mp4 파일
│   ├── pictures/             # 분할된 프레임 이미지
│   └── runs/                 # 학습 결과 (가중치, 그래프)
└── README.md
```

## 진행 단계

### 1. 데이터 수집 — 동영상 촬영
[01.take_video.py](01.take_video.py)

영상 출처는 `--source` 로 고릅니다. 기본값은 `lekiwi` 입니다.

| `--source` | 영상 출처 |
| --- | --- |
| `lekiwi` | 라즈베리파이의 `lekiwi_host` 가 ZMQ 로 뿌리는 스트림 (랜선 직결) |
| `local` | 노트북에 꽂은 USB 웹캠 |

- 실행 시 사용 가능한 카메라(로컬은 index, LeKiwi 는 `front` / `wrist`)를 출력하고 선택
- **Space**: 녹화 시작/중지 토글, **q / ESC**: 종료
- 저장 경로: `outputs/videos/YYYYMMDD_HHMMSS.mp4`

```bash
python 01.take_video.py                              # 기본값 그대로
python 01.take_video.py --remote=192.168.0.201       # LeKiwi host IP 지정
python 01.take_video.py --source local               # 노트북 USB 웹캠
python 01.take_video.py --help                       # 전체 옵션
```

주요 옵션 (`--help` 로 전부 확인):

| 옵션 | 설명 | 기본값 |
| --- | --- | --- |
| `--source` | `lekiwi` / `local` | `lekiwi` |
| `--remote` | LeKiwi host IP | `192.168.0.201` |
| `--port` | 관측 스트림 포트 | `5556` |
| `--camera` | LeKiwi 카메라 이름 | `front` |
| `--color-mode` | host 채널 순서 (`rgb` / `bgr`) | `rgb` |
| `--camera-index` | 로컬 웹캠 index (생략하면 실행 중 선택) | 없음 |
| `--capture-size` / `--display-size` | `WxH` | `640x480` |
| `--output-dir` | mp4 저장 폴더 | `outputs/videos` |
| `--fps` | 저장 FPS (생략하면 실측) | 실측 |

#### LeKiwi 카메라로 찍기

랜선 직결 기준입니다.

```
노트북 10.42.0.1  ──(유선)──  LeKiwi(라즈베리파이) 10.42.0.61
                              :5556  observation (영상 + 팔 관절값)  ← 우리가 쓰는 것
                              :5555  command (팔/베이스 제어)        ← 안 씀
```

**① 로봇에 host 를 먼저 띄웁니다.** 이게 안 떠 있으면 스크립트는 한 줄도 못 돕니다.

```bash
ssh roboseasy@10.42.0.61
./start_lekiwi_host.sh          # 이 SSH 창은 켜 둔 채로 둘 것 (닫으면 host 도 죽음)
```

`No command available` · `Stopping the base` 경고는 정상입니다. 이 저장소는 영상만 받고
명령(5555)은 안 보내기 때문입니다.

**② 노트북에서 촬영합니다.**

```bash
ping -c1 10.42.0.61             # 먼저 연결 확인
python 01.take_video.py --remote=10.42.0.61
```

- `lerobot` 를 `yolo` 환경에 깔 필요는 **없습니다.** host 가 보내는 메시지는 그냥 JSON 이라
  `pyzmq` 만 있으면 받습니다.
- 한 host 에는 **클라이언트를 하나만** 붙이세요. teleop·record 가 같이 붙어 있으면 프레임을
  나눠 가져서 뚝뚝 끊깁니다.
- 색이 이상하면(보라가 분홍, 노랑이 파랑) `--color-mode bgr` 로 실행합니다. lerobot 의
  `OpenCVCameraConfig` 기본값이 RGB 라 host 가 채널을 뒤집어 보내는 것을 되돌리는 값입니다.
- host 는 FPS 를 알려주지 않아서, 녹화 전에 2초간 실제 프레임 속도를 재서 그 값으로 저장합니다.

### 2. 데이터 전처리 — 프레임 분할
[02.splite_video2pic.py](02.splite_video2pic.py)

- `--video` 로 지정한 mp4 를 프레임 단위로 분할 (생략하면 `outputs/videos` 목록에서 고름)
- 저장 경로: `outputs/pictures/<영상이름>/<영상이름>_00000001.png` …
- `--stride N` 으로 N 프레임마다 한 장씩만 저장 (라벨링할 장수 줄이기)

```bash
python 02.splite_video2pic.py                                       # 목록에서 고르기
python 02.splite_video2pic.py --video outputs/videos/20260903_210000.mp4
python 02.splite_video2pic.py --stride 5                            # 5프레임에 1장
```

### 3. 라벨링 — Roboflow Auto-Label
분할된 이미지를 [Roboflow](https://roboflow.com)에 업로드한 뒤 Auto-Label 기능으로 자동 바운딩 박스 라벨링을 수행합니다.

- **비용**: 이미지 500장당 약 **5 크레딧** 소모
- **전략**: 크레딧 절약을 위해 **먼저 500장만 자동 라벨링**하여 작은 데이터셋을 확보
- 라벨링 결과는 YOLO 포맷으로 export하여 학습에 사용

### 4. 모델 학습 — 500장 데이터셋으로 시범 학습
우선 자동 라벨링된 500장만으로 Ultralytics YOLO 학습을 진행해 파이프라인이 정상 동작하는지, 성능이 어느 정도 나오는지 확인합니다. 결과를 보고 추가 데이터 라벨링 여부를 결정합니다.

`dataset` 이름의 폴더를 넣습니다.

[03.train.py](03.train.py)

```bash
python 03.train.py                                   # 기본값 그대로
python 03.train.py --epochs 200 --batch 8 --name cube2
python 03.train.py --device cpu                      # GPU 없이
python 03.train.py --help                            # 전체 옵션
```

- 결과 저장 경로: `outputs/runs/<--name>/` (가중치는 `weights/best.pt`)
- `--data`, `--pretrained`, `--imgsz`, `--patience` 도 옵션으로 바꿀 수 있습니다.

### 5. 추론
[04.inference.py](04.inference.py)

학습된 가중치로 실시간 스트림을 추론합니다. **q / ESC**: 종료.

```bash
python 04.inference.py                               # 기본 가중치 + lekiwi
python 04.inference.py --remote=192.168.0.201        # LeKiwi host IP 지정
python 04.inference.py --source local --camera-index 0
python 04.inference.py --weights outputs/runs/cube2/weights/best.pt --conf 0.4
python 04.inference.py --help                        # 전체 옵션
```

- 영상 소스 관련 옵션(`--source`, `--remote`, `--port`, `--camera`, `--color-mode`,
  `--camera-index`, `--capture-size`, `--display-size`)은 `01.take_video.py` 와 동일합니다.
- 추론 옵션: `--weights`, `--conf`, `--iou`, `--device`

### 6. 배포 — Hugging Face Hub 업로드
[05.upload_hf.py](05.upload_hf.py)

학습한 가중치를 모델 카드(성능 표·사용법·학습 설정)와 함께 Hub 에 올립니다.
모델 카드는 `results.csv` 와 `args.yaml` 을 읽어 자동으로 만듭니다.

**토큰은 코드에 적지 말고 환경변수로 넘기세요.** [토큰 발급](https://huggingface.co/settings/tokens)
시 write 권한이 필요합니다.

```bash
pip install huggingface_hub
export HF_TOKEN=hf_xxxxxxxx

python 05.upload_hf.py --dry-run                     # 올릴 파일 + 모델 카드 미리보기
python 05.upload_hf.py --run cube --public           # 실제 업로드
python 05.upload_hf.py --repo my_id/my_model --include all
python 05.upload_hf.py --help                        # 전체 옵션
```

`huggingface-cli login` 을 한 번 해 뒀다면 `--token` 도 환경변수도 필요 없습니다.

| 옵션 | 설명 | 기본값 |
| --- | --- | --- |
| `--repo` | 레포 이름 (`owner/name` 도 가능) | `red_cube_yolo` |
| `--owner` | 조직 또는 개인 계정 | `roboseasylabs` |
| `--token` | HF 액세스 토큰 | 환경변수 / CLI 로그인 |
| `--public` / `--private` | 공개 범위 | `--private` |
| `--run` | 올릴 학습 실행 (생략하면 mAP 와 함께 목록 표시) | 목록에서 선택 |
| `--include` | `weights` `metrics` `plots` `samples` `all` | `weights` |
| `--no-card` | 모델 카드를 만들지 않음 (Hub 에서 고친 카드 보존) | 카드 생성 |
| `--dry-run` | 업로드 없이 미리보기 | 꺼짐 |

업로드된 모델은 이렇게 씁니다.

```python
from huggingface_hub import hf_hub_download
from ultralytics import YOLO

model = YOLO(hf_hub_download("roboseasylabs/red_cube_yolo", "best.pt"))
results = model.predict("image.jpg", conf=0.25)
```

**공개된 모델**: [roboseasylabs/red_cube_yolo](https://huggingface.co/roboseasylabs/red_cube_yolo)
(`yolo26n`, 100 epochs, mAP50-95 **0.9892**)

## 요구사항

- Python 3.8+
- OpenCV (`pip install opencv-python`)
- Ultralytics, PyTorch (학습·추론)
- pyzmq (LeKiwi 스트림 수신)
- huggingface_hub (Hub 업로드)

```bash
pip install -r requirements.txt
```

## 비고

`outputs/` 폴더는 [.gitignore](.gitignore)로 git 추적에서 제외되어 있어 영상·이미지 결과물은 원격 저장소에 업로드되지 않습니다.
