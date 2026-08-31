"""체커보드 캘리브레이션 결과로 렌즈 왜곡을 보정한다.

calibrate_distortion.py가 저장한 camera_calib.npz를 읽어, 매 프레임 적용할 수 있는
remap 테이블을 미리 만들어둔다(프레임마다 재계산하면 느리다)."""
from pathlib import Path

import cv2
import numpy as np


class Undistorter:
    """캘리브레이션 파일 하나에 대응하는 왜곡 보정기.

    alpha=0이면 왜곡 보정 후 생기는 검은 가장자리를 잘라내고(화각 손실 있음),
    alpha=1이면 모든 픽셀을 남긴다(검은 영역 생김). 물체 탐지용으로는 잘라내는 쪽이
    깔끔해서 기본값 0을 쓴다."""

    def __init__(self, calib_path: str | Path, alpha: float = 0.0):
        data = np.load(str(calib_path))
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']
        self.image_size = tuple(int(v) for v in data['image_size'])

        self.new_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, self.image_size, alpha, self.image_size)
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, self.new_matrix,
            self.image_size, cv2.CV_16SC2)

    def apply(self, frame: np.ndarray, crop: bool = True) -> np.ndarray:
        result = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
        if crop:
            x, y, w, h = self.roi
            if w > 0 and h > 0:
                result = result[y:y + h, x:x + w]
        return result
