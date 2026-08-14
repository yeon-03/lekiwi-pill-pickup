"""bbox 관측값으로부터 정렬 여부와 오차를 계산하는 순수 함수.

⚠️ 실제 관절 각도로 변환하는 게인(픽셀 오차 1당 몇 도 움직일지)은 이 모듈이 아니라
pick_pill_bottle.py(Task 12)에서 실기기 튜닝으로 결정한다 — 카메라-관절 기구학적
관계는 실측 없이 계산할 수 없어 여기 넣지 않았다."""
from dataclasses import dataclass


@dataclass
class AlignmentError:
    x_offset_px: float       # 양수=화면 중앙보다 오른쪽
    y_offset_px: float       # 양수=화면 중앙보다 아래
    distance_error_cm: float  # 양수=목표거리보다 더 멀리 있음(더 다가가야 함)


def compute_alignment_error(
    bbox_center_x: float, bbox_center_y: float,
    frame_width: int, frame_height: int,
    current_distance_cm: float, target_distance_cm: float,
) -> AlignmentError:
    return AlignmentError(
        x_offset_px=bbox_center_x - frame_width / 2,
        y_offset_px=bbox_center_y - frame_height / 2,
        distance_error_cm=current_distance_cm - target_distance_cm,
    )


def is_aligned(error: AlignmentError, center_tolerance_px: float,
               distance_tolerance_cm: float) -> bool:
    return (abs(error.x_offset_px) <= center_tolerance_px
            and abs(error.y_offset_px) <= center_tolerance_px
            and abs(error.distance_error_cm) <= distance_tolerance_cm)
