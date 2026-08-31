import numpy as np
import pytest

import cv2

from lekiwi_pill_pickup.pixel_to_base import (
    extrinsic_from_checkerboard,
    intersect_ray_with_table,
    matrix_from_xyz_rpy,
    pixel_to_camera_ray,
    project_pixel_to_base,
)

K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])


def test_center_pixel_ray_is_optical_axis():
    ray = pixel_to_camera_ray(320, 240, K, None)
    assert ray == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)


def test_ray_is_unit_length():
    ray = pixel_to_camera_ray(100, 50, K, None)
    assert np.linalg.norm(ray) == pytest.approx(1.0)


def test_off_center_pixel_ray_direction_matches_sign():
    # 화면 오른쪽(u가 중심보다 큼) 픽셀은 카메라 좌표계에서 +x 방향이어야 함
    ray = pixel_to_camera_ray(420, 240, K, None)
    assert ray[0] > 0
    assert ray[1] == pytest.approx(0.0, abs=1e-9)


def test_intersect_ray_with_table_straight_down():
    # 카메라가 테이블 위 0.5m에서 -z 방향(아래)을 보면, 교차점은 카메라 바로 아래
    T = np.eye(4)
    T[:3, 3] = [0.2, 0.1, 0.5]
    point = intersect_ray_with_table(T, np.array([0.0, 0.0, -1.0]), table_height_m=0.0)
    assert point == pytest.approx([0.2, 0.1, 0.0])


def test_intersect_ray_parallel_to_table_raises():
    T = np.eye(4)
    with pytest.raises(ValueError, match='평행'):
        intersect_ray_with_table(T, np.array([1.0, 0.0, 0.0]), table_height_m=0.0)


def test_intersect_ray_behind_camera_raises():
    # 테이블(z=0)이 카메라(z=0.5)보다 아래인데 광선이 +z(위쪽, 테이블 반대방향)를
    # 향하면 교차점이 카메라 뒤쪽으로 나옴 -> 무효
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, 0.5]
    with pytest.raises(ValueError, match='카메라 뒤쪽'):
        intersect_ray_with_table(T, np.array([0.0, 0.0, 1.0]), table_height_m=0.0)


def test_project_pixel_to_base_end_to_end():
    # 카메라를 180도 뒤집어 테이블을 정면으로 내려다보게 설정
    T = matrix_from_xyz_rpy([0.2, 0.0, 0.5], [np.pi, 0.0, 0.0])
    point = project_pixel_to_base(320, 240, K, None, T, table_height_m=0.0)
    assert point == pytest.approx([0.2, 0.0, 0.0], abs=1e-9)


def test_matrix_from_xyz_rpy_identity():
    T = matrix_from_xyz_rpy([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert T == pytest.approx(np.eye(4))


def test_matrix_from_xyz_rpy_translation_only():
    T = matrix_from_xyz_rpy([1.0, 2.0, 3.0], [0.0, 0.0, 0.0])
    assert T[:3, 3] == pytest.approx([1.0, 2.0, 3.0])
    assert T[:3, :3] == pytest.approx(np.eye(3))


def test_extrinsic_from_checkerboard_recovers_known_camera_pose():
    # 카메라 위치를 미리 정해두고, solvePnP가 낼 법한 rvec/tvec을 역산으로 만들어서
    # 함수가 원래 카메라 위치를 정확히 복원하는지 확인(왕복 일관성 테스트)
    true_T_base_camera = matrix_from_xyz_rpy(
        [0.1, 0.0, 0.3], [np.pi + np.radians(30), 0.0, 0.0])
    T_base_board = matrix_from_xyz_rpy([0.25, 0.0, 0.0], [np.pi, 0.0, 0.0])

    T_camera_board = np.linalg.inv(true_T_base_camera) @ T_base_board
    rvec, _ = cv2.Rodrigues(T_camera_board[:3, :3])
    tvec = T_camera_board[:3, 3]

    recovered = extrinsic_from_checkerboard(T_base_board, rvec, tvec)
    assert recovered[:3, 3] == pytest.approx(true_T_base_camera[:3, 3], abs=1e-9)
    assert recovered[:3, :3] == pytest.approx(true_T_base_camera[:3, :3], abs=1e-9)
