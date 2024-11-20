import math
import threading
import uuid

import rospy
from attrs import define, field
from clover.srv import GetTelemetry, GetTelemetryResponse, Navigate, SetPosition
from std_srvs.srv import Trigger

from skyros.peer import Channel, Peer


@define()
class Drone(Peer):
    name: str = field(factory=lambda: f"drone_{uuid.uuid4().hex[0:6]}")
    telemetry_rate: float = field(default=5.0)
    telemetry_frame: str = field(default="aruco_map")

    _telemtry_lock: threading.Lock = field(init=False, factory=threading.Lock)
    _telemetry_timer: rospy.Timer = field(init=False)
    _get_telemetry: rospy.ServiceProxy = field(init=False)
    telemetry_channel: Channel = field(init=False)

    navigate: rospy.ServiceProxy = field(init=False)
    autoland = rospy.ServiceProxy("land", Trigger)
    set_position = rospy.ServiceProxy("set_position", SetPosition)

    def __attrs_post_init__(self):
        super().__attrs_post_init__()

        self._get_telemetry = rospy.ServiceProxy("get_telemetry", GetTelemetry)
        self.telemetry_channel = Channel(key="telemetry/pose")
        self.add_channel(self.telemetry_channel)

        self.navigate = rospy.ServiceProxy("navigate", Navigate)

    def get_telemetry(self, frame_id: str = "aruco_map") -> GetTelemetryResponse:
        with self._telemtry_lock:
            return self._get_telemetry(frame_id=frame_id)

    def telemetry_sender(self, _):
        telem = self.get_telemetry(frame_id=self.telemetry_frame)
        self.telemetry_channel.send(
            {
                "x": telem.x,
                "y": telem.y,
                "z": telem.z,
                "yaw": telem.yaw,
                "vx": telem.vx,
                "vy": telem.vy,
                "vz": telem.vz,
                "yaw_rate": telem.yaw_rate,
                "frame_id": telem.frame_id,
            }
        )

    def start(self):
        super().start()
        timer_duration = rospy.Duration(1.0 / self.telemetry_rate)
        self._telemetry_timer = rospy.Timer(period=timer_duration, callback=self.telemetry_sender)
        # self._telemetry_timer.start()

    def stop(self):
        self._telemetry_timer.shutdown()
        super().stop()

    def wait(self, duration: float):
        rospy.sleep(duration)
        if rospy.is_shutdown():
            raise RuntimeError("rospy shutdown")

    def navigate_wait(self, x=0, y=0, z=0, yaw=float("nan"), speed=0.5, frame_id="", auto_arm=False, tolerance=0.2):
        self.logger.info(f"Navigating to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id}")
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while True:
            telem = self.get_telemetry(frame_id="navigate_target")
            if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
                self.wait(0.1)
                self.logger.info("Arrived at target")
                break
            self.wait(0.1)

    def takeoff(self, z=1.5, delay: float = 4.0):
        self.logger.info(f"Taking off to z={z:.2f}")
        self.navigate(z=1.5, frame_id="body", auto_arm=True)
        self.wait(1.0)
        telem: GetTelemetryResponse = self.get_telemetry(frame_id="body")
        if not telem.armed:
            raise RuntimeError("Arming failed!")
        self.wait(delay)
        self.logger.info("Takeoff done")

    def land(self, z=0.5, delay: float = 4.0, frame_id="aruco_map"):
        telem: GetTelemetryResponse = self.get_telemetry(frame_id=frame_id)
        self.logger.info("Pre-landing")
        self.navigate_wait(x=telem.x, y=telem.y, z=z, frame_id=frame_id)
        self.wait(1.0)
        self.logger.info("Landig")
        self.autoland()
        self.wait(delay)
        self.logger.info("Landed")
