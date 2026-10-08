from src.agv_sim.vehicle import DifferentialDriveAGV


def test_straight_motion():
    agv = DifferentialDriveAGV()
    agv.reset(0.0, 0.0, 0.0)
    agv.step(1.0, 0.0, 1.0)
    assert abs(agv.state.x - 1.0) < 1e-9
    assert abs(agv.state.y) < 1e-9


def test_angular_motion_is_wrapped():
    agv = DifferentialDriveAGV()
    agv.reset(0.0, 0.0, 0.0)
    agv.step(0.0, 10.0, 1.0)
    assert -3.1415926536 <= agv.state.theta <= 3.1415926536
