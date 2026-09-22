#!/usr/bin/env python3
"""Length-prefixed JSON framing for the local mqtt_bridge <-> mqtt_client_daemon
IPC socket.

mqtt_bridge.py (ROS2 side, DDS participant) and mqtt_client_daemon.py
(paho/cert-holding side, separate Linux user) can't share a process —
cross-user ROS2/DDS discovery is a known, unresolved upstream limitation
once the host has network connectivity (eProsima/Fast-DDS#1750, open
since 2021). This module is the minimal glue between the two halves.

This channel carries only VDA5050 order/state/instantActions/connection/
visualization JSON — the same content that already transits MQTT in the
clear at the application layer — so it intentionally has no encryption of
its own. Its only real security property is the Unix socket file's
permissions (owner mqtt-bridge-user, group shared with whichever account
runs the ROS2 stack — see fleet/mqtt-daemon.sh --mqtt-user). No
certificate or key material ever crosses this socket.
"""
import json
import os
import socket
import struct

DEFAULT_SOCKET_PATH = os.environ.get("OSC_MQTT_SOCKET_PATH", "/run/osc-mqtt-bridge/bridge.sock")

_HEADER = struct.Struct("!I")  # 4-byte big-endian length prefix
MAX_FRAME_BYTES = 4 * 1024 * 1024  # generous upper bound, VDA5050 orders can be large-ish


def send_frame(sock: socket.socket, topic_type: str, payload: str) -> None:
    """Send one {"topic_type": ..., "payload": ...} frame. payload is a JSON string."""
    body = json.dumps({"topic_type": topic_type, "payload": payload}).encode("utf-8")
    if len(body) > MAX_FRAME_BYTES:
        raise ValueError(f"IPC frame too large ({len(body)} bytes) for topic_type={topic_type!r}")
    sock.sendall(_HEADER.pack(len(body)) + body)


def recv_frame(sock: socket.socket) -> dict:
    """Blocking read of exactly one frame. Raises ConnectionError on EOF/short read."""
    header = _recv_exact(sock, _HEADER.size)
    (length,) = _HEADER.unpack(header)
    if length > MAX_FRAME_BYTES:
        raise ValueError(f"IPC frame header declares {length} bytes, exceeds MAX_FRAME_BYTES")
    body = _recv_exact(sock, length)
    return json.loads(body.decode("utf-8"))


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("IPC socket closed (short read)")
        buf.extend(chunk)
    return bytes(buf)
