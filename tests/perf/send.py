import logging
import os

import zenoh

from skyros.peer import Peer

logging.basicConfig(level=logging.DEBUG)

# config_path = Path(__file__).parent.parent / "zenoh-config.json5"
# config = cast(dict, json5.loads(config_path.read_text()))
config = {}
config["mode"] = "peer"
config["timestamping"] = dict(enabled=True)
peer = Peer(config, known_addrs=[])
zenoh.init_log_from_env_or("INFO")

with peer:
    perf_sender = peer.zenoh_session.declare_publisher("perf/data", congestion_control=zenoh.CongestionControl.BLOCK)
    print("Peer started...")
    print("Waiting for peers...")
    peer.wait_for_peer_amount(1)
    print(f"Got peers: {peer.get_peers()}")
    print("Sending data...")
    while True:
        # Generate random bytes with 128 KiB
        perf_sender.put(os.urandom(128 * 1024))
