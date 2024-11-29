import logging
import os

import click
import zenoh

from skyros.peer import Peer

logging.basicConfig(level=logging.DEBUG)


@click.command()
@click.option("--peer", multiple=True)
@click.option("--packet-size", default=32)
@click.option("--latency/--throughput", default=False)
def main(peer, packet_size, latency):
    print(f"Known peers: {peer}")
    print(f"Packet size: {packet_size}")
    print(f"Latency mode: {latency}")
    # config_path = Path(__file__).parent.parent / "zenoh-config.json5"
    # config = cast(dict, json5.loads(config_path.read_text()))
    config = {}
    config["mode"] = "peer"
    config["timestamping"] = dict(enabled=True)
    peer = Peer(config, known_addrs=list(peer))
    zenoh.init_log_from_env_or("INFO")

    with peer:
        perf_sender = peer.zenoh_session.declare_publisher(
            "perf/data",
            congestion_control=zenoh.CongestionControl.BLOCK,
            priority=zenoh.Priority.REAL_TIME,
            express=True,
        )
        print("Peer started...")
        print("Waiting for peers...")
        peer.wait_for_peer_amount(1)
        print(f"Got peers: {peer.get_peers()}")
        print("Sending data...")
        size = packet_size * 1024  # Generate random bytes with X KiB
        if latency:  # to prevernt if inside the loop for performance reasons
            while True:
                perf_sender.put(os.urandom(size))
                peer.wait(1)
        else:
            while True:
                perf_sender.put(os.urandom(size))


if __name__ == "__main__":
    main()
