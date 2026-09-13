from services.pickplace.robot_args import LeKiwiRobotArgs


def test_defaults():
    args = LeKiwiRobotArgs()
    assert args.remote_ip == "192.168.0.201"
    assert args.id == "lekiwi01"
    assert args.port_zmq_cmd == 5555
    assert args.port_zmq_observations == 5556


def test_to_config_builds_lekiwi_client_config():
    args = LeKiwiRobotArgs(
        remote_ip="10.42.0.141", id="lekiwi01", port_zmq_cmd=5555, port_zmq_observations=5556,
    )
    cfg = args.to_config()
    assert cfg.remote_ip == "10.42.0.141"
    assert cfg.id == "lekiwi01"
    assert cfg.port_zmq_cmd == 5555
    assert cfg.port_zmq_observations == 5556
