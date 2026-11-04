"""Dependency-free local waypoint state machine; all positions are local ENU."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Observation:
    connected: bool = False
    armed: bool = False
    mode: str = ""
    position: tuple = None
    fresh: bool = False
    landed: bool = False


class Mission:
    def __init__(self, altitude=5.0, radius=0.6, dwell=2.0, timeout=180.0):
        if not all(math.isfinite(x) and x > 0 for x in (altitude, radius, dwell, timeout)):
            raise ValueError("mission parameters must be finite and positive")
        self.altitude, self.radius, self.dwell, self.timeout = altitude, radius, dwell, timeout
        self.state, self.reason = "WAITING", "waiting for fresh telemetry"
        self.target = None
        self.route, self.index = [], 0
        self.entered, self.arrived, self.started = None, None, None

    def transition(self, state, now, reason=""):
        self.state, self.entered, self.reason = state, now, reason
        self.arrived = None

    def fail(self, now, reason):
        self.transition("FAULT", now, reason)
        # Stop Offboard proof-of-life and let configured PX4 loss behavior act.
        self.target = None

    def step(self, now, obs):
        """Return an idempotent requested mode/arm operation, or None.

        now is simulation time. Transport watchdogs belong to the ROS wrapper.
        Actual telemetry, never a service response alone, confirms transitions.
        """
        if self.state in ("DONE", "FAULT"):
            return None
        if self.state == "WAITING":
            if not obs.connected or not obs.fresh or obs.position is None:
                return None
            if obs.armed:
                self.fail(now, "refusing to adopt an already armed vehicle")
                return None
            x, y, z = obs.position
            height = z + self.altitude
            self.route = [(x, y, height), (x + 4, y, height),
                          (x + 4, y + 4, height), (x, y + 4, height), (x, y, height)]
            self.target, self.started = obs.position, now
            self.transition("PRESTREAM", now)
            return None
        if not obs.connected or not obs.fresh:
            self.fail(now, "lost fresh position/state telemetry")
            return None
        if now - self.started > self.timeout:
            self.fail(now, "mission timeout")
            return None
        if self.state == "PRESTREAM" and now - self.entered >= 2.0:
            self.transition("OFFBOARD", now)
        if self.state == "OFFBOARD":
            if obs.mode == "OFFBOARD":
                self.transition("ARMING", now)
            elif now - self.entered > 15:
                self.fail(now, "Offboard entry rejected")
            else:
                return "OFFBOARD"
        if self.state == "ARMING":
            if obs.mode != "OFFBOARD":
                self.fail(now, "Offboard lost during arming")
            elif obs.armed:
                self.target = self.route[0]
                self.transition("FLYING", now)
            elif now - self.entered > 15:
                self.fail(now, "arming rejected")
            else:
                return "ARM"
        if self.state == "FLYING":
            if not obs.armed or obs.mode != "OFFBOARD":
                self.fail(now, "unexpected disarm or flight mode change")
                return None
            if math.dist(obs.position, self.target) <= self.radius:
                if self.arrived is None:
                    self.arrived = now
                if now - self.arrived >= self.dwell:
                    self.index += 1
                    self.arrived = None
                    if self.index == len(self.route):
                        self.transition("LANDING", now)
                    else:
                        self.target = self.route[self.index]
            else:
                self.arrived = None
        if self.state == "LANDING":
            if obs.landed and not obs.armed:
                self.target = None
                self.transition("DONE", now, "landed and disarmed")
            elif now - self.entered > 45:
                self.fail(now, "landing timeout")
            elif obs.mode != "AUTO.LAND":
                return "AUTO.LAND"
        return None
