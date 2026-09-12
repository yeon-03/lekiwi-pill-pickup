import argparse
import os
import time
from datetime import datetime

import cv2

from frame_source import (
    ANY_CAMERA,
    FrameSourceError,
    LeKiwiStream,
    LocalCamera,
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FOURCC = cv2.VideoWriter_fourcc(*"mp4v")
FALLBACK_FPS = 30.0

# 아래는 전부 기본값이고, 명령줄 옵션으로 덮어쓴다. (--help 참고)
#   python 01.take_video.py --remote=192.168.0.201
#   python 01.take_video.py --source local
DEFAULT_OUTPUT_DIR = os.path.join(BASE_DIR, "outputs", "videos")

# "lekiwi" : 라즈베리파이의 lekiwi_host 가 ZMQ 로 뿌리는 영상 (랜선 직결)
# "local"  : 노트북에 꽂은 USB 웹캠
DEFAULT_SOURCE = "lekiwi"

# --- source = "lekiwi" ---
DEFAULT_REMOTE_IP = "192.168.0.201"   # 랜선 직결. 노트북은 10.42.0.1
DEFAULT_CAMERA_NAME = "front"         # host 가 발행하는 이름. 실행 시 다시 고를 수 있다.
DEFAULT_PORT_OBSERVATIONS = 5556
DEFAULT_HOST_COLOR_MODE = "rgb"       # host 가 보내는 채널 순서 (rgb | bgr)

# --- source = "local" ---
DEFAULT_CAPTURE_WIDTH = 640
DEFAULT_CAPTURE_HEIGHT = 480
MAX_PROBE_INDEX = 10

DEFAULT_DISPLAY_WIDTH = 640
DEFAULT_DISPLAY_HEIGHT = 480
FPS_PROBE_SECONDS = 2.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="카메라 영상을 mp4 로 녹화한다. (스페이스바로 시작/중지)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--source", choices=("lekiwi", "local"), default=DEFAULT_SOURCE,
        help="영상 소스. lekiwi=라즈베리파이 ZMQ 스트림, local=USB 웹캠",
    )
    parser.add_argument(
        "--remote", "--remote-ip", dest="remote_ip", default=DEFAULT_REMOTE_IP,
        metavar="IP", help="lekiwi host 의 IP",
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT_OBSERVATIONS,
        help="lekiwi host 의 관측 스트림 포트",
    )
    parser.add_argument(
        "--camera", default=DEFAULT_CAMERA_NAME, metavar="NAME",
        help="lekiwi 카메라 이름 (실행 중에 다시 고를 수 있다)",
    )
    parser.add_argument(
        "--color-mode", choices=("rgb", "bgr"), default=DEFAULT_HOST_COLOR_MODE,
        help="host 가 보내는 채널 순서. 색이 뒤집혀 보이면 바꾼다",
    )
    parser.add_argument(
        "--camera-index", type=int, default=None, metavar="N",
        help="local 웹캠 장치 index (지정하지 않으면 실행 중에 고른다)",
    )
    parser.add_argument(
        "--capture-size", default=f"{DEFAULT_CAPTURE_WIDTH}x{DEFAULT_CAPTURE_HEIGHT}",
        metavar="WxH", help="local 웹캠 캡처 해상도",
    )
    parser.add_argument(
        "--display-size", default=f"{DEFAULT_DISPLAY_WIDTH}x{DEFAULT_DISPLAY_HEIGHT}",
        metavar="WxH", help="미리보기 창 크기",
    )
    parser.add_argument(
        "--output-dir", default=DEFAULT_OUTPUT_DIR, metavar="DIR",
        help="mp4 를 저장할 폴더",
    )
    parser.add_argument(
        "--fps", type=float, default=None,
        help="저장 FPS. 지정하지 않으면 실제 프레임 속도를 재서 쓴다",
    )
    args = parser.parse_args(argv)
    args.capture_size = parse_size(parser, "--capture-size", args.capture_size)
    args.display_size = parse_size(parser, "--display-size", args.display_size)
    return args


def parse_size(parser, option, raw):
    """'640x480' 을 (640, 480) 으로."""
    parts = str(raw).lower().split("x")
    if len(parts) != 2 or not all(p.strip().isdigit() for p in parts):
        parser.error(f"{option} 는 'WxH' 형식이어야 합니다 (예: 640x480): {raw!r}")
    return int(parts[0]), int(parts[1])


def list_available_cameras(max_index: int = MAX_PROBE_INDEX):
    available = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx)
        if cap is not None and cap.isOpened():
            ok, _ = cap.read()
            if ok:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                available.append((idx, w, h))
            cap.release()
    return available


def prompt_camera_index(available):
    if not available:
        raise RuntimeError("사용 가능한 카메라가 없습니다.")

    print("[사용 가능한 카메라]")
    for idx, w, h in available:
        print(f"  - index {idx}  ({w}x{h})")

    valid = {idx for idx, _, _ in available}
    default_idx = available[0][0]
    while True:
        raw = input(f"사용할 카메라 index 입력 (기본 {default_idx}): ").strip()
        if raw == "":
            return default_idx
        if raw.isdigit() and int(raw) in valid:
            return int(raw)
        print(f"  -> 유효한 index가 아닙니다. {sorted(valid)} 중에서 선택하세요.")


def prompt_camera_name(names, default):
    """host 가 발행 중인 카메라 중 하나를 고른다.

    키트마다 'front' 와 'wrist' 가 뒤바뀌어 있는 경우가 있어서 물어본다."""
    fallback = default if default in names else names[0]
    if len(names) == 1:
        return names[0]

    print("[host 가 발행 중인 카메라]")
    for n, name in enumerate(names, 1):
        tag = "   <- 기본값" if name == fallback else ""
        print(f"  {n}) {name}{tag}")

    while True:
        raw = input(f"사용할 카메라 (번호 또는 이름, 기본 {fallback}): ").strip()
        if raw == "":
            return fallback
        if raw.isdigit() and 1 <= int(raw) <= len(names):
            return names[int(raw) - 1]
        if raw in names:
            return raw
        print(f"  -> '{raw}' 는 없는 카메라입니다. {names} 중에서 선택하세요.")


def open_source(args):
    if args.source == "local":
        width, height = args.capture_size
        if args.camera_index is not None:
            return LocalCamera(args.camera_index, width, height)
        available = list_available_cameras()
        camera_index = prompt_camera_index(available)
        return LocalCamera(camera_index, width, height)

    if args.source == "lekiwi":
        cap = LeKiwiStream(
            remote_ip=args.remote_ip,
            cam_name=ANY_CAMERA,
            port=args.port,
            host_color_mode=args.color_mode,
        )
        cap.use_camera(prompt_camera_name(cap.camera_names, args.camera))
        return cap

    raise RuntimeError(f"알 수 없는 source '{args.source}' — 'local' 또는 'lekiwi'.")


def measure_fps(cap, seconds=FPS_PROBE_SECONDS):
    """들어오는 프레임 속도를 실제로 재서 VideoWriter 에 넣을 값을 얻는다.

    lekiwi host 는 FPS 를 알려주지 않고, 실제 속도는 네트워크와 host 주기에
    따라 달라진다. 틀린 값을 넣으면 저장된 영상이 빨라지거나 느려진다."""
    print(f"[측정] 프레임 속도 확인 중 ({seconds:g}초)...")
    frames = 0
    start = time.time()
    while time.time() - start < seconds:
        ok, _ = cap.read()
        if not ok:
            break
        frames += 1
    elapsed = time.time() - start
    if frames < 2 or elapsed <= 0:
        return FALLBACK_FPS
    return frames / elapsed


def main(argv=None):
    args = parse_args(argv)
    display_width, display_height = args.display_size
    os.makedirs(args.output_dir, exist_ok=True)

    try:
        cap = open_source(args)
    except FrameSourceError as e:
        raise SystemExit(f"[오류] {e}")

    print(f"[소스] {cap.describe()}")

    ok, frame = cap.read()
    if not ok:
        cap.release()
        raise SystemExit("[오류] 연결은 됐지만 프레임이 오지 않습니다.")
    height, width = frame.shape[:2]

    fps = args.fps or cap.fps or measure_fps(cap)
    print(f"[정보] {width}x{height} @ {fps:.1f} FPS 로 저장합니다.")

    print("[안내] 스페이스바: 녹화 시작/중지, q 또는 ESC: 종료")

    writer = None
    output_path = None
    recording = False
    record_start = 0.0

    window_name = "Recorder (Space=Start/Stop, q/ESC=Quit)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, display_width, display_height)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[경고] 프레임을 읽지 못했습니다.")
                break

            if (frame.shape[1], frame.shape[0]) != (display_width, display_height):
                display = cv2.resize(frame, (display_width, display_height))
            else:
                display = frame.copy()
            if recording:
                elapsed = time.time() - record_start
                cv2.circle(display, (25, 25), 10, (0, 0, 255), -1)
                cv2.putText(
                    display,
                    f"REC {elapsed:5.1f}s",
                    (45, 33),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                if writer is not None:
                    writer.write(frame)
            else:
                cv2.putText(
                    display,
                    "IDLE - press SPACE to record",
                    (15, 33),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (200, 200, 200),
                    2,
                )

            cv2.imshow(window_name, display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(" "):
                if not recording:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    output_path = os.path.join(args.output_dir, f"{timestamp}.mp4")
                    writer = cv2.VideoWriter(output_path, FOURCC, fps, (width, height))
                    if not writer.isOpened():
                        print(f"[오류] VideoWriter를 열 수 없습니다: {output_path}")
                        writer = None
                        continue
                    recording = True
                    record_start = time.time()
                    print(f"[녹화 시작] {output_path}")
                else:
                    recording = False
                    if writer is not None:
                        writer.release()
                        writer = None
                    print(f"[저장 완료] {output_path}")
                    output_path = None
            elif key in (ord("q"), 27):
                if recording and writer is not None:
                    writer.release()
                    writer = None
                    print(f"[저장 완료] {output_path}")
                break
    finally:
        if writer is not None:
            writer.release()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
