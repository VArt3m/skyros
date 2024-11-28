import json
import logging
import pathlib
import threading
import time
import uuid
from typing import Any, Callable, Dict, List, cast

import json5
import zenoh
from attrs import define, field


@define()
class Peer:
    zenoh_config_path: pathlib.Path
    known_peer_addrs: List[str] = field(factory=list)
    channels: List["Channel"] = field(factory=list)

    zenoh_config: zenoh.Config = field(init=False)
    zenoh_session: zenoh.Session = field(init=False)
    _liveliness_token: zenoh.LivelinessToken = field(init=False)

    running: bool = field(init=False, default=False)
    name: str = field(factory=lambda: f"peer-{uuid.uuid4().hex[0:6]}")
    logger: logging.Logger = field(init=False)

    _peers: List[str] = field(factory=list, init=False)
    _peers_lock: threading.Lock = field(factory=threading.Lock, init=False)

    def __attrs_post_init__(self):
        self.logger = logging.getLogger(self.name)
        raw_config = cast(dict, json5.loads(self.zenoh_config_path.read_text()))

        config_endpoints = raw_config["connect"]["endpoints"]
        self.logger.debug(f"Config endpoints: {config_endpoints}")
        config_endpoints += []
        self.logger.info(f"Total endpoints: {config_endpoints}")

        self.zenoh_config = zenoh.Config.from_json5(cast(str, json5.dumps(raw_config)))

    def start(self):
        self.logger.info("Starting")
        self.zenoh_session = zenoh.open(self.zenoh_config)
        self._liveliness_token = self.liveliness.declare_token(f"liveliness/{self.name}")
        self.liveliness.declare_subscriber("liveliness/*", self._liveliness_handler, history=True)
        self.running = True
        self.zenoh_session.info.peers_zid()
        self.logger.info("Started")

        for channel in self.channels:
            channel._open(self)
            self.logger.info(f"Opened channel {channel.key}")

    def stop(self):
        self.logger.info("Stopping")
        self._liveliness_token.undeclare()
        if not self.zenoh_session.is_closed():
            self.zenoh_session.close()
        self.running = False
        self.logger.info("Stopped")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    @property
    def liveliness(self):
        return self.zenoh_session.liveliness()

    def _liveliness_handler(self, sample: zenoh.Sample):
        peer_name = str(sample.key_expr).split("/")[1]
        if sample.kind == zenoh.SampleKind.PUT:
            with self._peers_lock:
                self._peers.append(peer_name)
            self.logger.info(f"New alive token '{peer_name}' ({len(self.get_peers())} total)")
            self.on_connect(peer_name)
        elif sample.kind == zenoh.SampleKind.DELETE:
            with self._peers_lock:
                self._peers.remove(peer_name)
            self.logger.info(f"Dropped alive token '{peer_name}' ({len(self.get_peers())} total)")
            self.on_disconnect(peer_name)

    def on_connect(self, peer: str):
        pass

    def on_disconnect(self, peer: str):
        pass

    def wait(self, duration: float):
        time.sleep(duration)

    def wait_for_peers(self, *peers: str, duration: float = 0.1):
        while not set(peers).issubset(set(self.get_peers())):
            self.wait(duration)

    def wait_for_peer_amount(self, amount: int, duration: float = 0.1):
        while len(self.get_peers(me=True)) < amount:
            self.wait(duration)

    def get_peers(self, me: bool = False):
        with self._peers_lock:
            peers = self._peers.copy()
        if not me and self.name in peers:
            peers.remove(self.name)
        return peers

    def add_channel(self, channel: "Channel") -> None:
        if self.running:
            channel._open(self)
            self.logger.debug(f"Deferred opening channel {channel.key}")
        self.channels.append(channel)
        self.logger.info(f"Added channel {channel.key}")


def _default_callback(sender: str, data: dict) -> None:
    return None


@define()
class Channel:
    key: str
    callback_func: Callable[[str, dict], Any] = field(default=_default_callback)

    _peer: Peer = field(init=False)
    _subscriber: zenoh.Subscriber = field(init=False)
    _publisher: zenoh.Publisher = field(init=False)
    _logger: logging.Logger = field(init=False)

    _cache: Dict[str, dict] = field(init=False, factory=dict)
    _cache_lock: threading.Lock = field(init=False, factory=threading.Lock)

    @property
    def my_key(self) -> str:
        return f"{self.key}/{self._peer.name}"

    @property
    def all_key(self) -> str:
        return f"{self.key}/*"

    def _open(self, peer: Peer):
        self._peer = peer
        self._publisher = peer.zenoh_session.declare_publisher(
            self.my_key, congestion_control=zenoh.CongestionControl.DROP
        )
        self._subscriber = peer.zenoh_session.declare_subscriber(self.all_key, self._callback)

    def _callback(self, sample: zenoh.Sample):
        if sample.key_expr.intersects(self.my_key):
            return  # Ignore our own messages

        sender = str(sample.key_expr).split("/")[-1]
        data = json.loads(sample.payload.to_string())
        # self._peer.logger.info(f"Received [{sample.key_expr}] {data}")
        self.callback(sender, data)
        with self._cache_lock:
            self._cache[sender] = data

    def callback(self, sender: str, data: dict):
        self.callback_func(sender, data)

    def send(self, data: dict):
        # self._peer.logger.info(f"Sending {data}")
        self._publisher.put(json.dumps(data).encode("utf-8"))

    def get_for(self, name: str) -> dict:
        with self._cache_lock:
            return self._cache[name]

    def get_all(self) -> Dict[str, dict]:
        with self._cache_lock:
            return self._cache.copy()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    zenoh.init_log_from_env_or("INFO")
    with Peer(pathlib.Path(__file__).parent.parent / "zenoh-config.json5") as peer:
        peer.add_channel(Channel(key="telemetry/pose"))

        print("Peer started...")
        print("Waiting for peers...")
        peer.wait_for_peer_amount(2)
        print(f"Got peers: {peer.get_peers()}")
        while True:
            time.sleep(1)
