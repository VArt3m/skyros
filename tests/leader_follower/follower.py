import logging
import pathlib
import sys

import rospy
import zenoh

from skyros.drone import Drone

zenoh.init_log_from_env_or("INFO")
peer = Drone(pathlib.Path(__file__).parent.parent / "zenoh-config.json5")
peer.logger.setLevel(logging.INFO)
peer.logger.addHandler(logging.StreamHandler(stream=sys.stdout))
rospy.init_node(peer.name)
logging.basicConfig(level=logging.INFO)

with peer:
    print("Peer started...")
    print("Waiting for peers...")
    peer.wait_for_peer_amount(1)
    print(f"Got peers: {peer.get_peers()}")
    leader = peer.get_peers(me=False)[0]

    peer.takeoff()
    peer.navigate_wait(x=0.5, y=0.5, z=1.5, frame_id="aruco_map")
    peer.wait(1)

    telem = peer.telemetry_channel.get_for(leader)
    peer.navigate_wait(x=telem["x"] + 0.75, y=telem["y"], z=telem["z"], frame_id="aruco_map")
    peer.wait(3)

    while not rospy.is_shutdown():
        telem = peer.telemetry_channel.get_for(leader)
        if telem["z"] < 0.5:
            break
        if leader not in peer.get_peers():
            break
        peer.set_position(x=telem["x"] + 0.75, y=telem["y"], z=telem["z"], frame_id="aruco_map")
        peer.wait(0.1)

    peer.land()
