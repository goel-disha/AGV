from src.agv_sim.controller import PurePursuitController
from src.agv_sim.vehicle import AGVState


def test_pure_pursuit_returns_forward_command():
    controller = PurePursuitController(lookahead_m=0.5, linear_speed_mps=0.4)
    state = AGVState(0.0, 0.0, 0.0)
    v, omega, done = controller.command(state, [(0.0, 0.0), (2.0, 0.0)])
    assert v > 0.0
    assert abs(omega) < 1e-9
    assert not done


def test_goal_stops_vehicle():
    controller = PurePursuitController(goal_tolerance_m=0.3)
    state = AGVState(1.0, 1.0, 0.0)
    v, omega, done = controller.command(state, [(1.0, 1.0)])
    assert v == 0.0
    assert omega == 0.0
    assert done
