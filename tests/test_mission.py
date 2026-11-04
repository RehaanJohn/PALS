from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/pals_drone_controller"))
from pals_drone_controller.mission import Mission, Observation


def observation(position=(10, 20, 0.3), armed=False, mode="POSCTL", fresh=True, landed=False):
    return Observation(True, armed, mode, position, fresh, landed)


class MissionTests(unittest.TestCase):
    def flying(self):
        mission = Mission()
        mission.step(0, observation())
        self.assertIsNone(mission.step(1.9, observation()))
        self.assertEqual(mission.step(2.1, observation()), "OFFBOARD")
        self.assertEqual(mission.step(2.2, observation(mode="OFFBOARD")), "ARM")
        mission.step(2.3, observation(armed=True, mode="OFFBOARD"))
        self.assertEqual(mission.state, "FLYING")
        return mission

    def test_waits_for_valid_pose_and_fresh_state(self):
        mission = Mission()
        mission.step(1, Observation())
        mission.step(2, observation(fresh=False))
        self.assertEqual(mission.state, "WAITING")
        self.assertIsNone(mission.target)

    def test_nonzero_local_origin_route_and_confirmed_landing(self):
        mission = self.flying()
        self.assertEqual(mission.target, (10, 20, 5.3))
        now = 3
        for point in list(mission.route):
            mission.step(now, observation(point, True, "OFFBOARD"))
            # Arrival requires continuous dwell, not a single passing sample.
            mission.step(now + 2.1, observation(point, True, "OFFBOARD"))
            now += 3
        self.assertEqual(mission.state, "LANDING")
        self.assertEqual(mission.step(now, observation(armed=True, mode="OFFBOARD")), "AUTO.LAND")
        mission.step(now + 1, observation(mode="AUTO.LAND"))
        self.assertEqual(mission.state, "LANDING")
        mission.step(now + 2, observation(mode="AUTO.LAND", landed=True))
        self.assertEqual(mission.state, "DONE")
        self.assertIsNone(mission.target)

    def test_dwell_resets_when_leaving_target(self):
        mission = self.flying()
        mission.step(3, observation(mission.target, True, "OFFBOARD"))
        mission.step(4, observation((0, 0, 0), True, "OFFBOARD"))
        mission.step(5, observation(mission.target, True, "OFFBOARD"))
        self.assertEqual(mission.index, 0)

    def test_stale_telemetry_stops_setpoints(self):
        mission = self.flying()
        mission.step(3, observation(armed=True, mode="OFFBOARD", fresh=False))
        self.assertEqual(mission.state, "FAULT")
        self.assertIsNone(mission.target)
        self.assertIsNone(mission.step(4, observation()))

    def test_mode_loss_never_rearms(self):
        mission = self.flying()
        self.assertIsNone(mission.step(3, observation(armed=True, mode="AUTO.LAND")))
        self.assertEqual(mission.state, "FAULT")

    def test_rejected_mode_has_bounded_timeout(self):
        mission = Mission()
        mission.step(0, observation())
        mission.step(3, observation())
        mission.step(19, observation())
        self.assertEqual(mission.state, "FAULT")

    def test_existing_armed_vehicle_is_not_adopted(self):
        mission = Mission()
        mission.step(0, observation(armed=True))
        self.assertEqual(mission.state, "FAULT")

    def test_mission_timeout(self):
        mission = self.flying()
        mission.step(181, observation(armed=True, mode="OFFBOARD"))
        self.assertEqual(mission.state, "FAULT")


if __name__ == "__main__":
    unittest.main()
