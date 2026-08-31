"""화면 픽셀 좌표를 팔 기준(base_link) 3D 좌표로 변환한다.

원리: 픽셀 -> 카메라 광선 -> 테이블 평면과 교차. 물체가 테이블 위에 놓여 있다는
전제(평면 가정)를 쓰기 때문에 물체 높이를 몰라도 위치가 나온다.

⚠️ 이동 베이스여도 문제없는 이유: 여기서 나오는 좌표는 세상 좌표가 아니라 팔 밑동
기준이고, 카메라와 팔이 같은 차체에 붙어 있어 로봇이 굴러가도 둘 사이 관계는 안 변한다.
IK도 팔 기준으로 계산하므로 로봇이 방 어디에 있는지는 알 필요가 없다.

⚠️ 손목 카메라를 쓸 때는 T_base_camera가 팔 자세에 따라 달라진다. 지금은 "정해진
스캔 자세"에서만 유효한 값 하나를 쓰는 전제다(FK/핸드아이 미구현). 다른 자세에서
쓰려면 T_base_camera = FK(관절값) @ T_wrist_camera 로 매번 계산해야 한다.

(수식과 구조는 사내 soarm101 저장소의 vision/geometry 모듈을 참고해 이 프로젝트에
맞게 옮긴 것 — 그쪽은 ROS2/MoveIt 스택이라 배관은 빼고 순수 계산만 가져왔다.)
"""
from __future__ import annotations

import cv2
import numpy as np


def pixel_to_camera_ray(u: float, v: float, K: np.ndarray,
                        D: np.ndarray | None = None) -> np.ndarray:
    """픽셀 좌표를 카메라 좌표계의 단위 방향벡터로 바꾼다(왜곡 보정 포함)."""
    if D is not None and np.asarray(D).size > 0:
        pixel = np.array([[[float(u), float(v)]]], dtype=float)
        normalized = cv2.undistortPoints(pixel, np.asarray(K, dtype=float),
                                         np.asarray(D, dtype=float))
        x, y = normalized.reshape(2)
        ray = np.array([x, y, 1.0], dtype=float)
    else:
        ray = np.linalg.inv(np.asarray(K, dtype=float)) @ np.array(
            [float(u), float(v), 1.0], dtype=float)

    norm = float(np.linalg.norm(ray))
    if norm == 0.0:
        raise ValueError('카메라 광선의 크기가 0입니다')
    return ray / norm


def intersect_ray_with_table(T_base_camera: np.ndarray, ray_camera: np.ndarray,
                             table_height_m: float) -> np.ndarray:
    """카메라 광선을 팔 기준 좌표계로 옮긴 뒤 테이블 평면(z=table_height)과 교차."""
    T = np.asarray(T_base_camera, dtype=float)
    origin_base = T[:3, 3]
    direction_base = T[:3, :3] @ ray_camera

    if abs(direction_base[2]) < 1e-9:
        raise ValueError('광선이 테이블 평면과 평행해서 교차점이 없습니다')

    scale = (float(table_height_m) - origin_base[2]) / direction_base[2]
    if scale <= 0.0:
        raise ValueError('교차점이 카메라 뒤쪽에 있습니다 — 외부 파라미터를 확인하세요')
    return origin_base + scale * direction_base


def project_pixel_to_base(u: float, v: float, K: np.ndarray, D: np.ndarray | None,
                          T_base_camera: np.ndarray,
                          table_height_m: float) -> np.ndarray:
    """픽셀 -> 팔 기준 3D 좌표 (x, y, z), 단위 m."""
    ray = pixel_to_camera_ray(u, v, K, D)
    return intersect_ray_with_table(T_base_camera, ray, table_height_m)


def extrinsic_from_checkerboard(T_base_board: np.ndarray, rvec: np.ndarray,
                                tvec: np.ndarray) -> np.ndarray:
    """체커보드로 카메라의 팔 기준 위치를 구한다.

    T_base_board: 체커보드가 팔 기준 어디에 놓였는지(사람이 자로 재서 넣는 값)
    rvec/tvec: solvePnP 결과(카메라가 본 보드의 위치)
    """
    T_camera_board = np.eye(4, dtype=float)
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    T_camera_board[:3, :3] = R
    T_camera_board[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return np.asarray(T_base_board, dtype=float) @ np.linalg.inv(T_camera_board)


def matrix_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    """위치(m)와 오일러각(rad, xyz 순서)으로 4x4 변환행렬을 만든다."""
    from scipy.spatial.transform import Rotation

    T = np.eye(4, dtype=float)
    T[:3, :3] = Rotation.from_euler('xyz', np.asarray(rpy, dtype=float)).as_matrix()
    T[:3, 3] = np.asarray(xyz, dtype=float).reshape(3)
    return T
