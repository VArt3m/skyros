import logging
import pathlib
import sys

import rospy
import zenoh

from skyros.drone import Drone

zenoh.init_log_from_env_or("INFO")
peer = Drone(pathlib.Path(__file__).parent / "zenoh-config.json5")
peer.logger.setLevel(logging.INFO)
peer.logger.addHandler(logging.StreamHandler(stream=sys.stdout))
rospy.init_node(peer.name)
logging.basicConfig(level=logging.INFO)

with peer:
    print("Peer started...")
    print(peer.get_telemetry(frame_id="aruco_map"))
    print("Waiting for peers...")
    peer.wait_for_peer_amount(2)
    print(f"Got peers: {peer.get_peers()}")

    while not rospy.is_shutdown():
        peer.wait(1.0)
