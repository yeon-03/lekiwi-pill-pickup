"""학습한 YOLO 가중치를 Hugging Face Hub 에 올린다.

토큰은 코드에 적지 말고 환경변수로 넘기는 것을 권장한다.

    export HF_TOKEN=hf_xxxxxxxx
    python 05.upload_hf.py --owner roboseasylabs --repo red_cube_yolo

`huggingface-cli login` 을 한 번 해 뒀다면 --token 도 환경변수도 필요 없다.
"""
import argparse
import csv
import glob
import os
import shutil
import tempfile

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 기본값. 명령줄 옵션으로 덮어쓴다. (--help 참고)
DEFAULT_RUNS_DIR = os.path.join(BASE_DIR, "outputs", "runs")
DEFAULT_OWNER = "roboseasylabs"      # 조직 또는 개인 계정 이름
DEFAULT_REPO_NAME = "red_cube_yolo"
DEFAULT_LICENSE = "cc-by-4.0"
DEFAULT_DATASET_URL = "https://universe.roboflow.com/s-workspace-7c6k7/red_cube-rjoad"
DEFAULT_PROJECT_URL = "https://github.com/roboseasy/YOLO"

# 토큰을 찾을 환경변수 (앞에서부터 먼저 찾은 것을 쓴다)
TOKEN_ENV_VARS = ("HF_TOKEN", "HUGGINGFACE_TOKEN", "HUGGING_FACE_HUB_TOKEN")

WEIGHT_FILES = ("best.pt", "last.pt")
METRIC_FILES = ("results.csv", "args.yaml")
PLOT_PATTERNS = ("results.png", "*_curve.png", "confusion_matrix*.png", "labels.jpg")
SAMPLE_PATTERNS = ("train_batch*.jpg", "val_batch*.jpg")

INCLUDE_CHOICES = ("weights", "metrics", "plots", "samples", "all")

METRIC_COLUMNS = (
    ("Precision", "metrics/precision(B)"),
    ("Recall", "metrics/recall(B)"),
    ("mAP@50", "metrics/mAP50(B)"),
    ("mAP@50-95", "metrics/mAP50-95(B)"),
)


class HelpFormatter(argparse.RawDescriptionHelpFormatter,
                    argparse.ArgumentDefaultsHelpFormatter):
    """기본값을 보여주되, 켜고 끄는 플래그(--public 등)에는 붙이지 않는다.

    store_true/store_false 에 '(default: True)' 가 붙으면 무슨 뜻인지 헷갈린다."""

    def _get_help_string(self, action):
        if action.nargs == 0:
            return action.help
        return super()._get_help_string(action)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="학습한 YOLO 가중치를 Hugging Face Hub 에 업로드한다.",
        formatter_class=HelpFormatter,
        epilog=(
            "예)\n"
            "  export HF_TOKEN=hf_xxxx\n"
            "  python 05.upload_hf.py --repo red_cube_yolo --run cube --public\n"
            "  python 05.upload_hf.py --repo my_id/my_model --include all --dry-run\n"
        ),
    )
    parser.add_argument(
        "--repo", default=DEFAULT_REPO_NAME, metavar="NAME",
        help="모델 레포 이름. 'owner/name' 형태로 주면 --owner 는 무시된다",
    )
    parser.add_argument(
        "--owner", default=DEFAULT_OWNER, metavar="NAME",
        help="레포를 만들 조직 또는 개인 계정 이름",
    )
    parser.add_argument(
        "--token", default=None, metavar="HF_TOKEN",
        help=f"HF 액세스 토큰. 생략하면 {'/'.join(TOKEN_ENV_VARS)} 환경변수나 "
             "huggingface-cli 로그인 정보를 쓴다 (권장)",
    )
    visibility = parser.add_mutually_exclusive_group()
    visibility.add_argument(
        "--public", dest="private", action="store_false",
        help="공개 레포로 만든다",
    )
    visibility.add_argument(
        "--private", dest="private", action="store_true",
        help="비공개 레포로 만든다 (기본값)",
    )
    parser.set_defaults(private=True)
    parser.add_argument(
        "--run", default=None, metavar="NAME",
        help="올릴 학습 실행 이름 또는 경로 (생략하면 목록에서 고른다)",
    )
    parser.add_argument(
        "--runs-dir", default=DEFAULT_RUNS_DIR, metavar="DIR",
        help="학습 결과가 모여 있는 폴더",
    )
    parser.add_argument(
        "--include", nargs="+", default=["weights"], choices=INCLUDE_CHOICES,
        metavar="WHAT",
        help="함께 올릴 것 (%s)" % " | ".join(INCLUDE_CHOICES),
    )
    parser.add_argument(
        "--no-card", dest="write_card", action="store_false",
        help="모델 카드(README.md) 를 만들지 않는다 (Hub 에서 직접 고친 카드 보존)",
    )
    parser.add_argument(
        "--license", default=DEFAULT_LICENSE, help="모델 카드에 적을 라이선스",
    )
    parser.add_argument(
        "--dataset-url", default=DEFAULT_DATASET_URL, metavar="URL",
        help="모델 카드에 적을 데이터셋 주소",
    )
    parser.add_argument(
        "--project-url", default=DEFAULT_PROJECT_URL, metavar="URL",
        help="모델 카드에 적을 학습 파이프라인 저장소 주소",
    )
    parser.add_argument(
        "--message", default=None, metavar="TEXT", help="커밋 메시지",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="업로드하지 않고 올릴 파일 목록과 모델 카드만 보여준다",
    )
    args = parser.parse_args(argv)
    if "all" in args.include:
        args.include = [c for c in INCLUDE_CHOICES if c != "all"]
    args.repo_id = resolve_repo_id(parser, args.repo, args.owner)
    return args


def resolve_repo_id(parser, repo, owner):
    """'red_cube_yolo' + 'roboseasylabs' -> 'roboseasylabs/red_cube_yolo'."""
    repo = repo.strip().strip("/")
    if repo.count("/") == 1:
        return repo
    if "/" in repo:
        parser.error(f"--repo 는 'name' 또는 'owner/name' 형태여야 합니다: {repo!r}")
    if not owner.strip():
        parser.error("--owner 를 지정하거나 --repo 를 'owner/name' 형태로 주세요.")
    return f"{owner.strip().strip('/')}/{repo}"


def resolve_token(args):
    """(토큰, 어디서 온 토큰인지) 를 돌려준다. 권한 오류 메시지에 출처를 쓴다."""
    if args.token:
        return args.token, "--token 으로 받음"
    for name in TOKEN_ENV_VARS:
        token = os.environ.get(name)
        if token:
            print(f"[토큰] 환경변수 {name} 사용")
            return token, f"환경변수 {name}"
    print("[토큰] 환경변수가 없어 huggingface-cli 로그인 정보를 씁니다.")
    # None 을 주면 huggingface_hub 가 캐시된 로그인을 찾는다
    return None, "huggingface-cli 로그인 캐시"


def token_write_targets(who):
    """토큰이 쓸 수 있는 계정/조직 이름들. 판단할 수 없으면 None."""
    auth = (who.get("auth") or {}).get("accessToken") or {}
    role = auth.get("role")
    if role in ("write", "admin"):
        # 옛 방식(classic) write 토큰: 본인 계정과 소속 조직 전부에 쓸 수 있다
        return {who.get("name")} | {o.get("name") for o in who.get("orgs") or []}
    if role == "read":
        return set()
    if role != "fineGrained":
        return None                      # 모르는 형태면 막지 않고 그냥 시도한다

    targets = set()
    for scope in (auth.get("fineGrained") or {}).get("scoped") or []:
        permissions = scope.get("permissions") or []
        if "repo.write" not in permissions:
            continue
        name = (scope.get("entity") or {}).get("name")
        if name:
            targets.add(name)
    return targets


def verify_write_access(who, repo_id, token_hint):
    """업로드 도중 403 으로 죽지 않도록 미리 권한을 확인한다.

    whoami() 는 토큰 신원(읽기)만 확인하므로 통과해도 쓰기가 막힐 수 있다."""
    owner = repo_id.split("/")[0]
    targets = token_write_targets(who)
    if targets is None or owner in targets:
        return

    auth = (who.get("auth") or {}).get("accessToken") or {}
    writable = ", ".join(sorted(t for t in targets if t)) or "없음"
    raise SystemExit(
        f"[오류] 이 토큰은 '{owner}' 에 쓸 권한이 없습니다.\n"
        f"  토큰: {auth.get('displayName') or '?'} (종류: {auth.get('role') or '?'}, {token_hint})\n"
        f"  쓸 수 있는 곳: {writable}\n"
        "\n  해결 방법 (하나만 고르면 됩니다)\n"
        "  1) write 권한 토큰을 새로 만들어 환경변수로 넘기기\n"
        "     https://huggingface.co/settings/tokens 에서 'Write' 토큰 생성 후\n"
        "       export HF_TOKEN=hf_xxxxxxxx\n"
        f"  2) 지금 쓰는 fine-grained 토큰에 '{owner}' 조직 권한을 추가\n"
        f"     토큰 설정 > Repositories 에서 {owner} 를 넣고 'Write access to contents' 체크\n"
        f"  3) 개인 계정에 올리기\n"
        f"       python 05.upload_hf.py --owner {who.get('name')}"
    )


def read_metrics(run_dir):
    """results.csv 마지막 줄(= 마지막 epoch)의 지표를 꺼낸다."""
    path = os.path.join(run_dir, "results.csv")
    if not os.path.isfile(path):
        return {}
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    return {(k or "").strip(): (v or "").strip() for k, v in rows[-1].items()}


def read_train_args(run_dir):
    path = os.path.join(run_dir, "args.yaml")
    if not os.path.isfile(path):
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def read_class_names(weights_path):
    try:
        from ultralytics import YOLO
    except ImportError:
        return {}
    try:
        return YOLO(weights_path).names or {}
    except Exception as e:                       # 가중치만 올릴 때 카드용 정보라 치명적이지 않다
        print(f"[경고] 클래스 이름을 읽지 못했습니다: {e}")
        return {}


def list_runs(runs_dir):
    """weights/best.pt 가 있는 학습 폴더만 골라 (이름, mAP50-95) 로 돌려준다."""
    if not os.path.isdir(runs_dir):
        return []
    runs = []
    for name in sorted(os.listdir(runs_dir)):
        run_dir = os.path.join(runs_dir, name)
        if not os.path.isfile(os.path.join(run_dir, "weights", "best.pt")):
            continue
        metrics = read_metrics(run_dir)
        runs.append((name, metrics.get("metrics/mAP50-95(B)", "?"),
                     metrics.get("epoch", "?")))
    return runs


def prompt_run(runs_dir):
    runs = list_runs(runs_dir)
    if not runs:
        raise SystemExit(
            f"[오류] {runs_dir} 에 weights/best.pt 를 가진 학습 결과가 없습니다.\n"
            "  03.train.py 로 먼저 학습하세요."
        )
    if len(runs) == 1:
        print(f"[학습 결과] {runs[0][0]} 하나뿐이라 그대로 씁니다.")
        return runs[0][0]

    print("[올릴 학습 결과를 고르세요]")
    for n, (name, map5095, epoch) in enumerate(runs, 1):
        print(f"  {n}) {name:<16} mAP50-95 {map5095:<8} (epoch {epoch})")

    while True:
        raw = input(f"번호 또는 이름 (1-{len(runs)}): ").strip()
        if raw.isdigit() and 1 <= int(raw) <= len(runs):
            return runs[int(raw) - 1][0]
        if raw in {name for name, _, _ in runs}:
            return raw
        print(f"  -> 1 부터 {len(runs)} 사이의 번호나 폴더 이름을 입력하세요.")


def resolve_run_dir(args):
    run = args.run or prompt_run(args.runs_dir)
    candidates = [run, os.path.join(args.runs_dir, run)]
    for path in candidates:
        if os.path.isdir(path):
            return os.path.abspath(path)
    raise SystemExit(f"[오류] 학습 결과 폴더를 찾을 수 없습니다: {run}")


def collect_files(run_dir, include):
    """(원본 경로, 레포 안에서의 이름) 목록."""
    picked = []

    if "weights" in include:
        for name in WEIGHT_FILES:
            path = os.path.join(run_dir, "weights", name)
            if os.path.isfile(path):
                picked.append((path, name))
    if "metrics" in include:
        for name in METRIC_FILES:
            path = os.path.join(run_dir, name)
            if os.path.isfile(path):
                picked.append((path, name))
    for key, patterns, prefix in (
        ("plots", PLOT_PATTERNS, "plots"),
        ("samples", SAMPLE_PATTERNS, "samples"),
    ):
        if key not in include:
            continue
        for pattern in patterns:
            for path in sorted(glob.glob(os.path.join(run_dir, pattern))):
                picked.append((path, f"{prefix}/{os.path.basename(path)}"))

    seen = set()
    unique = []
    for path, name in picked:
        if name in seen:
            continue
        seen.add(name)
        unique.append((path, name))
    return unique


def format_metric(raw):
    try:
        return f"{float(raw):.4f}"
    except (TypeError, ValueError):
        return None


def build_model_card(args, run_dir, metrics, train_args, class_names):
    repo_name = args.repo_id.split("/")[-1]
    rows = [(label, format_metric(metrics.get(col))) for label, col in METRIC_COLUMNS]
    rows = [(label, value) for label, value in rows if value]

    base_model = str(train_args.get("model") or "").strip()
    imgsz = train_args.get("imgsz")
    names = list(class_names.values()) if class_names else []

    front = [
        "---",
        f"license: {args.license}",
        "library_name: ultralytics",
        "pipeline_tag: object-detection",
        "tags:",
        "  - object-detection",
        "  - yolo",
        "  - ultralytics",
    ]
    if base_model:
        front.append(f"base_model: {base_model}")
    if rows:
        front += [
            "model-index:",
            f"  - name: {repo_name}",
            "    results:",
            "      - task:",
            "          type: object-detection",
            "        metrics:",
        ]
        for label, value in rows:
            front += [
                f"          - type: {label.replace('@', '').replace('-', '_').lower()}",
                f"            value: {value}",
                f"            name: {label}",
            ]
    front.append("---")

    body = [
        "",
        f"# {repo_name}",
        "",
        f"Ultralytics YOLO 객체 탐지 모델입니다. 학습 실행 `{os.path.basename(run_dir)}` 의 결과입니다.",
        "",
    ]
    if names:
        body += [f"- **클래스** ({len(names)}개): " + ", ".join(f"`{n}`" for n in names)]
    if imgsz:
        body += [f"- **입력 해상도**: {imgsz} x {imgsz}"]
    if base_model:
        body += [f"- **베이스 모델**: `{base_model}`"]
    body += [""]

    if rows:
        epoch = metrics.get("epoch", "?")
        body += [
            "## 성능",
            "",
            f"{epoch} epoch 학습 후 validation 결과입니다.",
            "",
            "| 지표 | 값 |",
            "| --- | --- |",
        ]
        body += [f"| {label} | {value} |" for label, value in rows]
        body += [""]

    body += [
        "## 사용법",
        "",
        "```bash",
        "pip install ultralytics huggingface_hub",
        "```",
        "",
        "```python",
        "from huggingface_hub import hf_hub_download",
        "from ultralytics import YOLO",
        "",
        f'weights = hf_hub_download("{args.repo_id}", "best.pt")',
        "model = YOLO(weights)",
        "",
        'results = model.predict("image.jpg", conf=0.25, iou=0.45)',
        "results[0].show()",
        "```",
        "",
        "## 학습 설정",
        "",
        "| 항목 | 값 |",
        "| --- | --- |",
    ]
    for label, key in (("베이스 가중치", "model"), ("epochs", "epochs"),
                       ("imgsz", "imgsz"), ("batch", "batch"),
                       ("patience", "patience")):
        if train_args.get(key) is not None:
            body.append(f"| {label} | {train_args[key]} |")
    body += [""]

    body += [
        "## 데이터셋 · 파이프라인",
        "",
        f"- 데이터셋: {args.dataset_url}",
        f"- 학습 파이프라인(촬영 → 프레임 분할 → 라벨링 → 학습 → 추론): {args.project_url}",
        "",
        "## 한계",
        "",
        "- 단일 환경에서 수집한 데이터로 학습했습니다. 배경·조명·카메라가 바뀌면 성능이 떨어질 수 있습니다.",
    ]
    if imgsz:
        body.append(f"- {imgsz}x{imgsz} 입력 기준입니다. 아주 작게 찍힌 물체는 놓칠 수 있습니다.")
    body.append("")

    return "\n".join(front + body)


def stage_upload(stage_dir, files, card_text):
    for src, name in files:
        dst = os.path.join(stage_dir, name)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
    if card_text is not None:
        with open(os.path.join(stage_dir, "README.md"), "w", encoding="utf-8") as f:
            f.write(card_text)


def main(argv=None):
    args = parse_args(argv)
    run_dir = resolve_run_dir(args)

    files = collect_files(run_dir, args.include)
    if not files:
        raise SystemExit(
            f"[오류] 올릴 파일이 없습니다: {run_dir} (--include {' '.join(args.include)})"
        )

    metrics = read_metrics(run_dir)
    train_args = read_train_args(run_dir)
    weights = next((src for src, name in files if name == "best.pt"), None)
    class_names = read_class_names(weights) if (weights and args.write_card) else {}

    card_text = None
    if args.write_card:
        card_text = build_model_card(args, run_dir, metrics, train_args, class_names)

    total_mb = sum(os.path.getsize(src) for src, _ in files) / 1e6
    print(f"[학습 결과] {run_dir}")
    print(f"[레포] {args.repo_id}")
    print(f"[올릴 파일] {len(files)}개, {total_mb:.1f} MB")
    for _, name in files:
        print(f"  - {name}")
    if card_text is not None:
        print("  - README.md (모델 카드)")

    if args.dry_run:
        print(f"[공개 범위] 새로 만들 경우 {'private' if args.private else 'public'}")
        print("\n[dry-run] 업로드하지 않았습니다.")
        if card_text is not None:
            print("\n--- README.md ---")
            print(card_text)
        return

    try:
        from huggingface_hub import HfApi
    except ImportError as e:
        raise SystemExit(
            "[오류] huggingface_hub 가 필요합니다.\n  pip install huggingface_hub"
        ) from e

    token, token_hint = resolve_token(args)
    api = HfApi(token=token)
    try:
        who = api.whoami()
    except Exception as e:
        raise SystemExit(
            f"[오류] 토큰 확인 실패 ({token_hint}): {e}\n"
            "  write 권한이 있는 토큰인지 확인하세요. https://huggingface.co/settings/tokens"
        )
    print(f"[계정] {who.get('name')}")
    verify_write_access(who, args.repo_id, token_hint)

    api.create_repo(repo_id=args.repo_id, repo_type="model",
                    private=args.private, exist_ok=True)

    # 이미 있는 레포의 공개 범위는 create_repo 가 바꾸지 않는다. 실제 상태를 알려준다.
    actual_private = api.repo_info(args.repo_id, repo_type="model").private
    print(f"[공개 범위] {'private' if actual_private else 'public'}")
    if actual_private != args.private:
        want = "private" if args.private else "public"
        print(f"  -> 이미 있는 레포라 {want} 요청은 무시했습니다. "
              "바꾸려면 Hub 의 Settings 에서 변경하세요.")

    message = args.message or f"Upload YOLO weights from run '{os.path.basename(run_dir)}'"
    with tempfile.TemporaryDirectory() as stage_dir:
        stage_upload(stage_dir, files, card_text)
        commit = api.upload_folder(
            folder_path=stage_dir,
            repo_id=args.repo_id,
            repo_type="model",
            commit_message=message,
        )

    print(f"\n[완료] https://huggingface.co/{args.repo_id}")
    print(f"[커밋] {commit.commit_url}")


if __name__ == "__main__":
    main()
