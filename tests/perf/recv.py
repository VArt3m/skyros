import logging
import statistics
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List

import click
import zenoh

from skyros.peer import Peer

logging.basicConfig(level=logging.DEBUG)


@dataclass
class MessageStats:
    size: int
    recv_time: float
    send_time: float | None


@dataclass
class RecordedStats:
    throughput: float
    message_rate: float
    absolute_latency: float = 0.0
    # relative_jitter: float


class ThroughputMonitor:
    def __init__(
        self,
        window_size: int = 100,
        print_delay: float = 2,
        latency_mode: bool = False,
    ):
        self.window_size = window_size
        self.messages: Deque[MessageStats] = deque(maxlen=window_size)
        self.recorded_stats: List[RecordedStats] = []
        self.print_delay = print_delay
        self.latency_mode = latency_mode
        self.last_print = time.time()

    def add_message(self, stats: MessageStats):
        self.messages.append(stats)

        if time.time() - self.last_print > self.print_delay:
            self.print_stats()
            self.last_print = time.time()

    def format_size(self, size_bytes: float) -> str:
        for unit in ["B", "KB", "MB", "GB"]:
            if size_bytes < 1024:
                return f"{size_bytes:.2f} {unit}"
            size_bytes /= 1024
        return f"{size_bytes:.2f} TB"

    def format_speed(self, size_bytes: float) -> str:
        return f"{size_bytes / 125000 :.2f} Mbps"

    def print_stats(self):
        if len(self.messages) < 2:
            return

        window_duration = self.messages[-1].recv_time - self.messages[0].recv_time
        if window_duration <= 0:
            return

        # Calculate throughput
        total_bytes = sum(msg.size for msg in self.messages)
        throughput = total_bytes / window_duration
        msg_rate = len(self.messages) / window_duration

        record = RecordedStats(
            throughput=throughput,
            message_rate=msg_rate,
            absolute_latency=0.0,
        )

        # Calculate latencies
        absolute_latencies = []
        relative_latencies = []

        prev_recv_time = None
        for msg in self.messages:
            if msg.send_time is not None:
                absolute_latencies.append(msg.recv_time - msg.send_time)
            if prev_recv_time is not None:
                relative_latencies.append(msg.recv_time - prev_recv_time)
            prev_recv_time = msg.recv_time

        if absolute_latencies:
            abs_median = statistics.median(absolute_latencies) * 1000
            record.absolute_latency = abs_median
        # if relative_latencies:
        # rel_median = statistics.median(relative_latencies) * 1000
        # rel_jitter = statistics.stdev(relative_latencies) * 1000 if len(relative_latencies) > 1 else 0

        self.recorded_stats.append(record)

        print()
        print("===== Performance Statistics ======")
        print(f"Window size:        {len(self.messages)} messages")
        print(f"Window duration:    {window_duration:.2f} s")
        print(f"Message rate:       {msg_rate:.2f} msg/s")
        if not self.latency_mode:
            # print(f"Throughput:         {self.format_size(throughput)}/s")
            print(f"Throughput:         {self.format_speed(throughput)}")
        elif absolute_latencies:
            print(f"Absolute latency:   {abs_median:.2f} ms")

        # if relative_latencies:
        # print(f"Inter-message time: {rel_median:.4f} ms")
        # print(f"Relative jitter:   ±{rel_jitter:.4f} ms")
        print("===================================")

    def print_recorded_stats(self):
        if not self.recorded_stats:
            print("No recorded stats to display")
            return

        median_throughput = statistics.median([stats.throughput for stats in self.recorded_stats])
        median_msg_rate = statistics.median([stats.message_rate for stats in self.recorded_stats])
        median_latency = statistics.median([stats.absolute_latency for stats in self.recorded_stats])

        print()
        print("===== Recorded Statistics ======")
        print(f"Median message rate:{median_msg_rate:.2f} msg/s")
        if not self.latency_mode:
            print(f"Median latency:     {median_latency:.2f} ms")
        else:
            print(f"Median throughput:  {self.format_speed(median_throughput)}")
        print("==================================")


@click.command()
@click.option("--window-size", default=100)
@click.option("--print-delay", default=2.0)
@click.option("--latency/--throughput", default=False)
def main(window_size, print_delay, latency):
    print(f"Window size: {window_size}")
    print(f"Print delay: {print_delay}")
    print(f"Latency mode: {latency}")

    monitor = ThroughputMonitor(window_size=window_size, print_delay=print_delay, latency_mode=latency)

    def perf_handler(sample: zenoh.Sample):
        send_timestamp = sample.timestamp.get_time()
        recv_timestamp = peer.zenoh_session.new_timestamp().get_time()
        stats = MessageStats(
            size=len(sample.payload.to_bytes()),
            recv_time=recv_timestamp.timestamp(),
            send_time=send_timestamp.timestamp(),
        )
        monitor.add_message(stats)

    # config_path = Path(__file__).parent.parent / "zenoh-config.json5"
    # config = cast(dict, json5.loads(config_path.read_text()))
    config = {}
    config["mode"] = "router"
    config["timestamping"] = dict(enabled=True)
    peer = Peer(config)
    zenoh.init_log_from_env_or("INFO")

    try:
        with peer:
            peer.zenoh_session.declare_subscriber("perf/data", perf_handler)
            print("Peer started...")
            print("Waiting for peers...")
            peer.wait_for_peer_amount(1)
            print(f"Got peers: {peer.get_peers()}")
            while peer.peer_amount > 0:
                peer.wait(10)
                # monitor.print_stats()
    finally:
        monitor.print_recorded_stats()


if __name__ == "__main__":
    main()
