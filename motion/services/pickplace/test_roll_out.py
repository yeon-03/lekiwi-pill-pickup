from services.pickplace.roll_out import RollOutPlayer


def test_interpolates_toward_home_over_duration():
    current = {"arm_shoulder_pan.pos": 0.0, "arm_gripper.pos": 40.0}
    home = {"arm_shoulder_pan.pos": 10.0, "arm_gripper.pos": 0.0}
    player = RollOutPlayer(current, home, duration_s=2.0)

    pose = player.update(now=0.0)
    assert pose["arm_shoulder_pan.pos"] == 0.0
    assert not player.done

    pose = player.update(now=1.0)  # 절반 지남
    assert pose["arm_shoulder_pan.pos"] == 5.0
    assert not player.done

    pose = player.update(now=2.0)  # 다 지남
    assert pose["arm_shoulder_pan.pos"] == 10.0
    assert player.done


def test_gripper_is_never_touched():
    current = {"arm_shoulder_pan.pos": 0.0, "arm_gripper.pos": 40.0}
    home = {"arm_shoulder_pan.pos": 10.0, "arm_gripper.pos": 0.0}
    player = RollOutPlayer(current, home, duration_s=1.0)
    pose = player.update(now=1.0)
    assert pose["arm_gripper.pos"] == 40.0  # home 의 0.0 이 아니라 current 값 유지


def test_already_close_to_home_finishes_immediately():
    current = {"arm_shoulder_pan.pos": 10.1}
    home = {"arm_shoulder_pan.pos": 10.0}
    player = RollOutPlayer(current, home, duration_s=2.0)
    assert player.done
