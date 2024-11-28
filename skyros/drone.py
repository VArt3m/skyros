import math
import threading
import uuid
from typing import Tuple

from attrs import define, field

try:
    import rospy
    from clover.srv import GetTelemetry, GetTelemetryResponse, Navigate, SetPosition
    from std_srvs.srv import Trigger
except ImportError:
    rospy = None

from skyros.peer import Channel, Peer


@define()
class Drone(Peer):
    name: str = field(factory=lambda: f"drone_{uuid.uuid4().hex[0:6]}")
    telemetry_rate: float = field(default=20.0)
    telemetry_frame: str = field(default="aruco_map")

    _telemtry_lock: threading.Lock = field(init=False, factory=threading.Lock)
    _telemetry_timer: "rospy.Timer" = field(init=False)
    _get_telemetry: "rospy.ServiceProxy" = field(init=False)
    telemetry_channel: Channel = field(init=False)

    navigate: "rospy.ServiceProxy" = field(init=False)
    autoland: "rospy.ServiceProxy" = field(init=False)
    set_position: "rospy.ServiceProxy" = field(init=False)

    # Constants for collision avoidance
    COLLISION_RADIUS = 0.25  # drone radius in meters
    MAX_SPEED = 1.0  # maximum speed in meters per second
    MAX_ACCELERATION = 2.0  # maximum acceleration in meters per second^2
    REPULSION_STRENGTH = 500.0  # repulsion force strength
    ATTRACTION_STRENGTH = 2.0  # attraction to target strength
    NEAR_TARGET_REPULSION_MULT = 1.00  # Multiply repulsion when near target
    ARRIVAL_RADIUS = 0.75  # start slowing down within this distance
    TARGET_THRESHOLD = 0.2  # Distance at which target is considered "reached" (meters)
    TARGET_SPEED_THRESHOLD = 0.1  # Speed at which target is considered "reached" (meters per second)
    BASE_DAMPING = 0.5  # Base velocity damping factor
    FORCE_DAMPING_FACTOR = 0.05  # How much to increase damping based on force magnitude
    MAX_DAMPING = 0.99  # Maximum damping coefficient

    def __attrs_post_init__(self):
        super().__attrs_post_init__()

        if rospy is not None:
            self._get_telemetry = rospy.ServiceProxy("get_telemetry", GetTelemetry)
            self.autoland = rospy.ServiceProxy("autoland", Trigger)
            self.set_position = rospy.ServiceProxy("set_position", SetPosition)
            self.navigate = rospy.ServiceProxy("navigate", Navigate)

        self.telemetry_channel = Channel(key="telemetry/pose")
        self.add_channel(self.telemetry_channel)

    def get_telemetry(self, frame_id: str = "aruco_map") -> "GetTelemetryResponse":
        with self._telemtry_lock:
            return self._get_telemetry(frame_id=frame_id)

    def _telemetry_to_dict(self, telem: "GetTelemetryResponse") -> dict:
        return {
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

    def telemetry_sender(self, _):
        telem = self.get_telemetry(frame_id=self.telemetry_frame)
        self.telemetry_channel.send(self._telemetry_to_dict(telem))

    def start(self):
        super().start()
        timer_duration = rospy.Duration(1.0 / self.telemetry_rate)
        self._telemetry_timer = rospy.Timer(period=timer_duration, callback=self.telemetry_sender)

    def stop(self):
        self._telemetry_timer.shutdown()
        super().stop()

    def wait(self, duration: float):
        rospy.sleep(duration)
        if rospy.is_shutdown():
            raise RuntimeError("rospy shutdown")

    def calculate_repulsion_force(self, distance: float) -> float:
        """Calculate repulsion force that peaks at drone radius."""
        if distance >= self.COLLISION_RADIUS * 10:
            return 0

        if distance < self.COLLISION_RADIUS:
            return 0

        # Normalize distance to drone radius
        d = distance / self.COLLISION_RADIUS
        # Force peaks at d=1 (drone radius) and decays to both sides
        force = self.REPULSION_STRENGTH * math.exp(-((d - 1) ** 1.4))
        return force

    def get_avoidance_vector(
        self,
        my_telem: dict,
        target_x: float,
        target_y: float,
        target_z: float,
        prev_vx: float = 0,
        prev_vy: float = 0,
        prev_vz: float = 0,
        dt: float = 0.1,
    ) -> Tuple[float, float, float]:
        """Calculate avoidance vector based on other drones' positions."""
        fx = 0
        fy = 0
        # Calculate desired movement direction
        dx_target = target_x - my_telem["x"]
        dy_target = target_y - my_telem["y"]
        dist_to_target = math.sqrt(dx_target**2 + dy_target**2)

        # Clamp the magnitude
        if dist_to_target > self.ARRIVAL_RADIUS:
            dx_target = dx_target / dist_to_target * self.ARRIVAL_RADIUS
            dy_target = dy_target / dist_to_target * self.ARRIVAL_RADIUS

        attraction_mult = self.ATTRACTION_STRENGTH
        APPROACH_RADIUS = self.ARRIVAL_RADIUS * 1.5
        if dist_to_target < APPROACH_RADIUS:
            # Gradually decrease attraction as we get closer to target
            attraction_mult *= max(0.1, (dist_to_target / APPROACH_RADIUS) ** 1.0)

        fx += dx_target * attraction_mult
        fy += dy_target * attraction_mult

        all_telemetry = self.telemetry_channel.get_all()
        for drone_id, other_telem in all_telemetry.items():
            if drone_id == self.name:  # Skip self
                continue

            dx = my_telem["x"] - other_telem["x"]
            dy = my_telem["y"] - other_telem["y"]
            distance = math.sqrt(dx**2 + dy**2)

            # Predict future positions based on current velocities
            future_dx = (my_telem["x"] + my_telem["vx"] * dt) - (other_telem["x"] + other_telem["vx"] * dt)
            future_dy = (my_telem["y"] + my_telem["vy"] * dt) - (other_telem["y"] + other_telem["vy"] * dt)
            future_distance = math.sqrt(future_dx**2 + future_dy**2)

            # If future distance is less than current, increase repulsion
            if future_distance < distance:
                distance_factor = distance / max(future_distance, self.COLLISION_RADIUS * 0.1)
            else:
                distance_factor = 1.0

            # Calculate repulsion force
            force = self.calculate_repulsion_force(distance) * distance_factor
            if force == 0:
                continue

            # Normalize direction vector
            if distance > 0:
                dx /= distance
                dy /= distance

            # Add force components
            fx += dx * force
            fy += dy * force

            # Add extra repulsion based on relative velocity when getting closer
            if future_distance < distance:
                rel_vx = my_telem["vx"] - other_telem["vx"]
                rel_vy = my_telem["vy"] - other_telem["vy"]
                # Add opposing force to relative velocity
                fx -= rel_vx * force * 0.1
                fy -= rel_vy * force * 0.1

        # Calculate total force magnitude for adaptive damping
        total_force = math.sqrt(fx**2 + fy**2)
        # Apply forces to velocity with damping
        damping = min(self.BASE_DAMPING + total_force * self.FORCE_DAMPING_FACTOR, self.MAX_DAMPING)
        # damping = 0

        # Calculate desired new velocity
        desired_vx = prev_vx * damping + fx * (1 - damping)
        desired_vy = prev_vy * damping + fy * (1 - damping)

        # Calculate acceleration as change in velocity over time
        ax = (desired_vx - prev_vx) / dt
        ay = (desired_vy - prev_vy) / dt

        # Limit acceleration magnitude while preserving direction
        acceleration = math.sqrt(ax * ax + ay * ay)
        if acceleration > self.MAX_ACCELERATION:
            ax *= self.MAX_ACCELERATION / acceleration
            ay *= self.MAX_ACCELERATION / acceleration

        # Apply limited acceleration to velocity
        vx = prev_vx + ax * dt
        vy = prev_vy + ay * dt
        vz = 0

        # Limit velocity magnitude
        velocity = math.sqrt(vx * vx + vy * vy)
        if velocity > self.MAX_SPEED:
            vx *= self.MAX_SPEED / velocity
            vy *= self.MAX_SPEED / velocity

        if velocity < self.TARGET_SPEED_THRESHOLD and dist_to_target < self.TARGET_THRESHOLD:
            vx = 0
            vy = 0
            vz = 0

        return vx, vy, vz

    def navigate_with_avoidance(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = float("nan"),
        frame_id: str = "",
    ):
        """Navigate to target position while avoiding other drones."""
        self.logger.info(f"Navigating to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id} with collision avoidance")

        rate_hz = 10
        rate = rospy.Rate(rate_hz)
        target_dt = 1.0 / rate_hz
        vx, vy, vz = 0, 0, 0

        while True:
            telem = self.get_telemetry(frame_id)
            vx, vy, vz = self.get_avoidance_vector(self._telemetry_to_dict(telem), x, y, z, vx, vy, vz, dt=target_dt)
            if vx == 0 and vy == 0 and vz == 0:
                self.logger.info("Arrived at target")
                break

            # Apply avoidance vector to movement
            next_x = telem.x + vx * target_dt
            next_y = telem.y + vy * target_dt
            next_z = telem.z + vz * target_dt
            self.set_position(x=next_x, y=next_y, z=next_z, yaw=yaw, frame_id=frame_id)

            rate.sleep()

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
        telem = self.get_telemetry(frame_id="body")
        if not telem.armed:
            raise RuntimeError("Arming failed!")
        self.wait(delay)
        self.logger.info("Takeoff done")

    def land(self, z=0.5, delay: float = 4.0, frame_id="aruco_map"):
        telem = self.get_telemetry(frame_id=frame_id)
        self.logger.info("Pre-landing")
        self.navigate_wait(x=telem.x, y=telem.y, z=z, frame_id=frame_id)
        self.wait(1.0)
        self.logger.info("Landig")
        self.autoland()
        self.wait(delay)
        self.logger.info("Landed")
