import argparse
import glob
import os

import cv2

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 기본값. 명령줄 옵션으로 덮어쓴다. (--help 참고)
#   python 02.splite_video2pic.py --video outputs/videos/20260903_210000.mp4
DEFAULT_VIDEO_DIR = os.path.join(BASE_DIR, "outputs", "videos")
DEFAULT_OUTPUT_ROOT = os.path.join(BASE_DIR, "outputs", "pictures")
DEFAULT_STRIDE = 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="mp4 를 프레임 단위 png 로 쪼갠다.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--video", default=None, metavar="PATH",
        help="쪼갤 mp4 경로 (지정하지 않으면 --video-dir 에서 고른다)",
    )
    parser.add_argument(
        "--video-dir", default=DEFAULT_VIDEO_DIR, metavar="DIR",
        help="mp4 를 찾을 폴더",
    )
    parser.add_argument(
        "--output-dir", default=None, metavar="DIR",
        help="png 를 저장할 폴더 (기본: <output-root>/<영상 이름>)",
    )
    parser.add_argument(
        "--output-root", default=DEFAULT_OUTPUT_ROOT, metavar="DIR",
        help="영상 이름별 폴더를 만들 상위 폴더",
    )
    parser.add_argument(
        "--stride", type=int, default=DEFAULT_STRIDE, metavar="N",
        help="N 프레임마다 한 장씩 저장",
    )
    args = parser.parse_args(argv)
    if args.stride < 1:
        parser.error(f"--stride 는 1 이상이어야 합니다: {args.stride}")
    return args


def select_video(video_dir: str) -> str:
    if not os.path.isdir(video_dir):
        raise FileNotFoundError(f"동영상 폴더를 찾을 수 없습니다: {video_dir}")

    videos = sorted(glob.glob(os.path.join(video_dir, "*.mp4")))
    if not videos:
        raise FileNotFoundError(f"{video_dir} 에 mp4 파일이 없습니다.")

    print("동영상을 고르세요")
    for i, path in enumerate(videos, start=1):
        print(f"  {i}. {os.path.basename(path)}")

    while True:
        raw = input(f"번호 입력 (1-{len(videos)}): ").strip()
        if raw.isdigit() and 1 <= int(raw) <= len(videos):
            return videos[int(raw) - 1]
        print(f"  -> 1 부터 {len(videos)} 사이의 번호를 입력하세요.")


def main(argv=None):
    args = parse_args(argv)

    if args.video:
        input_video = args.video
        if not os.path.isfile(input_video):
            raise SystemExit(f"[오류] 동영상을 찾을 수 없습니다: {input_video}")
    else:
        input_video = select_video(args.video_dir)

    video_name = os.path.splitext(os.path.basename(input_video))[0]
    output_dir = args.output_dir or os.path.join(args.output_root, video_name)
    os.makedirs(output_dir, exist_ok=True)

    cap = cv2.VideoCapture(input_video)
    if not cap.isOpened():
        raise RuntimeError(f"동영상을 열 수 없습니다: {input_video}")

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[정보] 총 프레임 수: {total}")
    print(f"[저장 위치] {output_dir}")

    read_count = 0
    saved = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            if read_count % args.stride != 0:
                read_count += 1
                continue
            read_count += 1

            saved += 1
            filename = f"{video_name}_{saved:08d}.png"
            out_path = os.path.join(output_dir, filename)
            if not cv2.imwrite(out_path, frame):
                print(f"[오류] 저장 실패: {out_path}")
                break

            if saved % 50 == 0:
                print(f"  - {saved} 장 저장 완료")
    finally:
        cap.release()

    print(f"[완료] 총 {saved}장의 이미지를 저장했습니다.")


if __name__ == "__main__":
    main()
