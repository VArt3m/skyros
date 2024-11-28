import logging
import pathlib
import sys

import rospy

from skyros.drone import Drone

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize drone
peer = Drone(pathlib.Path(__file__).parent / "zenoh-config.json5")
peer.logger.setLevel(logging.INFO)
peer.logger.addHandler(logging.StreamHandler(stream=sys.stdout))
rospy.init_node(peer.name)

with peer:
    logger.info("Drone started...")
    logger.info("Waiting for other drones...")
    peer.wait_for_peer_amount(2)  # Wait for at least one other drone
    logger.info(f"Connected peers: {peer.get_peers()}")

    # Take off
    peer.takeoff(z=1.5)
    peer.wait(2)

    # Navigate with collision avoidance
    waypoints1 = [
        (0.8, 0.0, 1.5),
        (-0.8, -0.64, 1.5),
        (0.8, 0.15, 1.5),
        (0.0, -0.3, 1.5),
    ]

    waypoints2 = [
        (-0.8, 0.0, 1.5),
        (0.8, 0.15, 1.5),
        (-0.8, -0.64, 1.5),
        (0.0, -0.3, 1.5),
    ]
    waypoints = waypoints1

    try:
        for x, y, z in waypoints:
            logger.info(f"Navigating to waypoint: ({x}, {y}, {z})")
            peer.navigate_with_avoidance(x=x, y=y, z=z, frame_id="aruco_map", timeout=20.0)
            peer.wait(1)
    finally:
        # Return to start and land
        peer.navigate_wait(x=0.8, y=0, z=1.0, frame_id="aruco_map")
        peer.land()
