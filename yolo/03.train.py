import argparse
import os

from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 기본값. 명령줄 옵션으로 덮어쓴다. (--help 참고)
#   python 03.train.py --epochs 200 --name cube2 --device cpu
DEFAULT_DATA_YAML = os.path.join(BASE_DIR, "dataset", "data.yaml")
DEFAULT_PROJECT_DIR = os.path.join(BASE_DIR, "outputs", "runs")
DEFAULT_RUN_NAME = "cube"

DEFAULT_PRETRAINED = "yolo26n.pt"
DEFAULT_EPOCHS = 100
DEFAULT_IMG_SIZE = 640
DEFAULT_BATCH = 16
DEFAULT_DEVICE = "0"
DEFAULT_PATIENCE = 20


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="dataset/data.yaml 로 YOLO 를 학습하고 검증한다.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--data", default=DEFAULT_DATA_YAML, metavar="PATH",
                        help="data.yaml 경로")
    parser.add_argument("--pretrained", default=DEFAULT_PRETRAINED, metavar="PATH",
                        help="시작할 사전학습 가중치")
    parser.add_argument("--project", default=DEFAULT_PROJECT_DIR, metavar="DIR",
                        help="결과를 저장할 상위 폴더")
    parser.add_argument("--name", default=DEFAULT_RUN_NAME,
                        help="이번 학습 실행 이름")
    parser.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS)
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMG_SIZE,
                        help="학습 이미지 크기")
    parser.add_argument("--batch", type=int, default=DEFAULT_BATCH)
    parser.add_argument("--patience", type=int, default=DEFAULT_PATIENCE,
                        help="early stopping 대기 에폭 수")
    parser.add_argument("--device", default=DEFAULT_DEVICE,
                        help="학습 장치 (0, 0,1, cpu ...)")
    args = parser.parse_args(argv)
    args.device = normalize_device(args.device)
    return args


def normalize_device(raw):
    """ultralytics 는 GPU 를 문자열 '0' 보다 정수 0 으로 받는 쪽이 안전하다."""
    text = str(raw).strip()
    return int(text) if text.isdigit() else text


def main(argv=None):
    args = parse_args(argv)

    model = YOLO(args.pretrained)

    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        patience=args.patience,
        save=True,
        plots=True,
    )

    metrics = model.val(
        data=args.data,
        imgsz=args.imgsz,
        device=args.device,
        project=args.project,
        name=f"{args.name}_val",
    )
    print("[검증 결과]")
    print(f"  mAP50    : {metrics.box.map50:.4f}")
    print(f"  mAP50-95 : {metrics.box.map:.4f}")
    print(f"  Precision: {metrics.box.mp:.4f}")
    print(f"  Recall   : {metrics.box.mr:.4f}")


if __name__ == "__main__":
    main()
