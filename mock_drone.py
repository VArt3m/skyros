import logging
import pathlib

import zenoh

from skyros.peer import Channel, Peer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

zenoh.init_log_from_env_or("INFO")
peer = Peer(pathlib.Path(__file__).parent / "zenoh-config.json5")
telemetry_channel = Channel(key="telemetry/pose")
peer.add_channel(telemetry_channel)

mock_telem = {
    "x": 1.0,
    "y": 1.0,
    "z": 1.5,
    "yaw": 0.0,
    "vx": 0.0,
    "vy": 0.0,
    "vz": 0.0,
    "yaw_rate": 0.0,
    "frame_id": "aruco_map",
}

with peer:
    while True:
        telemetry_channel.send(mock_telem)
        peer.wait(0.1)
