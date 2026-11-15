#!/usr/bin/env python3
"""Raw CDR two-domain CycloneDDS relay between CARLA (Domain 0) and Autoware (Domain 1).

Runs one fixed-port (ParticipantIndex=1, UDP 7412/7413) CycloneDDS worker on Domain 0
to talk directly to native CARLA ROS 2 on the macOS host, and one loopback worker on
Domain 1 for all Autoware nodes inside Docker. Avoids Docker Desktop macOS NAT
dropping per-node UDP port forwards across launches.
"""

from __future__ import annotations

import json
import multiprocessing as mp
import os
import signal
import socket
import struct
import sys
import threading
import time

HEADER_STRUCT = struct.Struct("!BHI")
OP_REGISTER = 1
OP_DATA = 2

CARLA_TO_AUTOWARE_PREFIXES = ("/clock", "/sensing/", "/vehicle/status/")
AUTOWARE_TO_CARLA_TOPICS = {
    "/control/command/control_cmd",
    "/control/command/gear_cmd",
}

BRIDGE_XML = "/carla_example/configs/cyclonedds/cyclonedds_bridge.xml"
CONTAINER_XML = "/carla_example/configs/cyclonedds/cyclonedds_container.xml"


def _send_frame(sock: socket.socket, send_lock: threading.Lock, op: int, topic_id: int, payload: bytes) -> None:
    header = HEADER_STRUCT.pack(op, topic_id, len(payload))
    with send_lock:
        sock.sendall(header)
        if payload:
            sock.sendall(payload)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        chunk = sock.recv_into(view[got:])
        if chunk == 0:
            raise EOFError("relay socket closed")
        got += chunk
    return bytes(buf)


def _recv_frame(sock: socket.socket) -> tuple[int, int, bytes]:
    header = _recv_exact(sock, HEADER_STRUCT.size)
    op, topic_id, length = HEADER_STRUCT.unpack(header)
    payload = _recv_exact(sock, length) if length else b""
    return op, topic_id, payload


def _make_qos(reliability: str, durability: str, depth: int):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rel = (
        ReliabilityPolicy.RELIABLE
        if "RELIABLE" in reliability
        else ReliabilityPolicy.BEST_EFFORT
    )
    dur = (
        DurabilityPolicy.TRANSIENT_LOCAL
        if "TRANSIENT" in durability
        else DurabilityPolicy.VOLATILE
    )
    return QoSProfile(
        reliability=rel,
        durability=dur,
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, min(int(depth or 10), 20)),
    )


def _domain0_worker(sock: socket.socket, stop_event: mp.Event) -> None:
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = f"file://{BRIDGE_XML}"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"
    os.environ["ROS_DOMAIN_ID"] = "0"

    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    rclpy.init(domain_id=0)
    node = Node("carla_dds_relay_host")
    send_lock = threading.Lock()
    state_lock = threading.Lock()

    subs: dict[str, object] = {}
    pubs_by_id: dict[int, object] = {}
    pubs_by_topic: set[str] = set()
    next_id = 1
    msg_types_cache: dict[str, type] = {}

    def load_msg_type(type_str: str) -> type:
        if type_str not in msg_types_cache:
            msg_types_cache[type_str] = get_message(type_str)
        return msg_types_cache[type_str]

    def reader_loop() -> None:
        while not stop_event.is_set():
            try:
                op, topic_id, payload = _recv_frame(sock)
            except (EOFError, OSError):
                break
            if op == OP_REGISTER:
                meta = json.loads(payload.decode("utf-8"))
                tname = meta["topic"]
                ttype = meta["type"]
                if tname in pubs_by_topic:
                    continue
                msg_cls = load_msg_type(ttype)
                sub_qos_list = node.get_subscriptions_info_by_topic(tname)
                if sub_qos_list:
                    qp = sub_qos_list[0].qos_profile
                    qos = _make_qos(qp.reliability.name, qp.durability.name, qp.depth or 10)
                else:
                    qos = _make_qos(meta["reliability"], meta["durability"], meta["depth"])
                with state_lock:
                    pubs_by_id[topic_id] = node.create_publisher(msg_cls, tname, qos)
                    pubs_by_topic.add(tname)
            elif op == OP_DATA:
                with state_lock:
                    pub = pubs_by_id.get(topic_id)
                if pub is not None:
                    pub.publish(payload)

    def poll_topics() -> None:
        nonlocal next_id
        for topic, types in node.get_topic_names_and_types():
            if topic in subs or not types:
                continue
            if not topic.startswith(CARLA_TO_AUTOWARE_PREFIXES):
                continue
            infos = node.get_publishers_info_by_topic(topic)
            if not infos:
                continue
            qp = infos[0].qos_profile
            ttype = types[0]
            try:
                msg_cls = load_msg_type(ttype)
            except Exception as exc:
                node.get_logger().warn(f"Skip unsupported type {ttype} on {topic}: {exc}")
                continue

            tid = next_id
            next_id += 1
            rel = qp.reliability.name
            dur = qp.durability.name
            depth = int(qp.depth or 10)
            qos = _make_qos(rel, dur, depth)

            meta = json.dumps(
                {
                    "topic": topic,
                    "type": ttype,
                    "reliability": rel,
                    "durability": dur,
                    "depth": depth,
                }
            ).encode("utf-8")
            _send_frame(sock, send_lock, OP_REGISTER, tid, meta)

            def _cb(raw_msg: bytes, _tid: int = tid) -> None:
                try:
                    _send_frame(sock, send_lock, OP_DATA, _tid, raw_msg)
                except OSError:
                    pass

            subs[topic] = node.create_subscription(msg_cls, topic, _cb, qos, raw=True)
            node.get_logger().info(f"Relaying Domain 0 -> 1: {topic} [{ttype}, {rel}]")

    timer = node.create_timer(0.5, poll_topics)
    _ = timer
    reader = threading.Thread(target=reader_loop, daemon=True)
    reader.start()

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        try:
            sock.close()
        except OSError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _domain1_worker(sock: socket.socket, stop_event: mp.Event) -> None:
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = f"file://{CONTAINER_XML}"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"
    os.environ["ROS_DOMAIN_ID"] = "1"

    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    rclpy.init(domain_id=1)
    node = Node("carla_dds_relay_autoware")
    send_lock = threading.Lock()
    state_lock = threading.Lock()

    subs: dict[str, object] = {}
    pubs_by_id: dict[int, object] = {}
    next_id = 1000
    msg_types_cache: dict[str, type] = {}

    def load_msg_type(type_str: str) -> type:
        if type_str not in msg_types_cache:
            msg_types_cache[type_str] = get_message(type_str)
        return msg_types_cache[type_str]

    def reader_loop() -> None:
        while not stop_event.is_set():
            try:
                op, topic_id, payload = _recv_frame(sock)
            except (EOFError, OSError):
                break
            if op == OP_REGISTER:
                meta = json.loads(payload.decode("utf-8"))
                msg_cls = load_msg_type(meta["type"])
                qos = _make_qos(meta["reliability"], meta["durability"], meta["depth"])
                with state_lock:
                    if topic_id not in pubs_by_id:
                        pubs_by_id[topic_id] = node.create_publisher(msg_cls, meta["topic"], qos)
            elif op == OP_DATA:
                with state_lock:
                    pub = pubs_by_id.get(topic_id)
                if pub is not None:
                    pub.publish(payload)

    def poll_control_topics() -> None:
        nonlocal next_id
        for topic, types in node.get_topic_names_and_types():
            if topic not in AUTOWARE_TO_CARLA_TOPICS or topic in subs or not types:
                continue
            infos = node.get_publishers_info_by_topic(topic)
            if not infos:
                continue
            qp = infos[0].qos_profile
            ttype = types[0]
            msg_cls = load_msg_type(ttype)
            tid = next_id
            next_id += 1
            rel = qp.reliability.name
            dur = qp.durability.name
            depth = int(qp.depth or 10)
            qos = _make_qos(rel, dur, depth)

            meta = json.dumps(
                {
                    "topic": topic,
                    "type": ttype,
                    "reliability": rel,
                    "durability": dur,
                    "depth": depth,
                }
            ).encode("utf-8")
            _send_frame(sock, send_lock, OP_REGISTER, tid, meta)

            def _cb(raw_msg: bytes, _tid: int = tid) -> None:
                try:
                    _send_frame(sock, send_lock, OP_DATA, _tid, raw_msg)
                except OSError:
                    pass

            subs[topic] = node.create_subscription(msg_cls, topic, _cb, qos, raw=True)
            node.get_logger().info(f"Relaying Domain 1 -> 0: {topic} [{ttype}, {rel}]")

    timer = node.create_timer(0.5, poll_control_topics)
    _ = timer
    reader = threading.Thread(target=reader_loop, daemon=True)
    reader.start()

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        try:
            sock.close()
        except OSError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    mp.set_start_method("fork", force=True)
    sock0, sock1 = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
    for s in (sock0, sock1):
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        except OSError:
            pass
    stop_event = mp.Event()

    p0 = mp.Process(target=_domain0_worker, args=(sock0, stop_event), name="relay_domain0")
    p1 = mp.Process(target=_domain1_worker, args=(sock1, stop_event), name="relay_domain1")
    p0.start()
    p1.start()
    sock0.close()
    sock1.close()

    def _handle_sig(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    while not stop_event.is_set():
        if not p0.is_alive() or not p1.is_alive():
            stop_event.set()
            break
        time.sleep(0.25)

    for p in (p0, p1):
        p.join(timeout=3.0)
        if p.is_alive():
            p.terminate()
            p.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
