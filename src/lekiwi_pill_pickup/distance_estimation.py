"""단안 카메라 기준, 알려진 물체 실제 크기로 거리를 역산하는 순수 함수.

핀홀 카메라 모델: distance = (real_size * focal_length_px) / pixel_size.
focal_length_px는 Task 4(카메라 캘리브레이션)의 실측값을 호출부에서 넘겨준다 —
이 모듈은 캘리브레이션 자체를 하지 않는다(하드웨어 의존성 없는 순수 계산만)."""


def estimate_distance_cm(pixel_height: float, real_height_cm: float,
                          focal_length_px: float) -> float:
    if pixel_height <= 0:
        raise ValueError(f'pixel_height는 양수여야 함: {pixel_height}')
    return (real_height_cm * focal_length_px) / pixel_height
