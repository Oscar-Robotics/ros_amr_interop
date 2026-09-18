#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the InOrbit, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
mqtt_client_daemon.py — the paho/cert-holding half of the mqtt_bridge split.

Runs as a separate, low-privilege Linux user that owns the runtime certifications allowing communication with the
MQTT broker, isolated from the rest of the ROS2 stack.

Communication with ROS2 facing mqtt_bridge.py node over a local Unix socket (unix_socket_protocol.py), forwarding
VDA5050 JSON payloads verbatim in both directions.

Configuration is via environment variables:

  MQTT_ADDRESS, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD
  USE_ENCRYPTION (0/1), MQTT_TLS_CA_CERT, MQTT_TLS_CLIENT_CERT, MQTT_TLS_CLIENT_KEY
  VDA5050_PROTOCOL_VERSION (default 2.0.0)
  MANUFACTURER_NAME, SERIAL_NUMBER, INTERFACE_NAME (default vda5050)
  UNIX_SOCKET_PATH (default /run/osc-mqtt-bridge.sock)
"""

import json
import logging
import os
import socket
import ssl
import threading
import time

from paho.mqtt import client as mqtt_client
from paho.mqtt.client import error_string

from vda5050_connector_py import unix_socket_protocol
from vda5050_connector_py.utils import get_vda5050_mqtt_topic

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO"),
    format="%(asctime)s [%(levelname)s] [mqtt_client_daemon]: %(message)s",
)
log = logging.getLogger("mqtt_client_daemon")

# Deliberately NOT imported from vda5050_controller.py — that module
# imports rclpy.action/rclpy.node directly, and this daemon must not
# depend on rclpy at all (see module docstring). Duplicated here instead;
# it's a stable VDA5050-spec constant, not something that changes often.
SUPPORTED_PROTOCOL_VERSIONS = ["1.1.0", "2.0.0"]

# Frames the daemon forwards ROS2-ward (received from MQTT).
_MQTT_TO_ROS2_TOPICS = ("order", "instantActions")
# Frames the daemon accepts from the ROS2 side and republishes to MQTT.
_ROS2_TO_MQTT_TOPICS = ("state", "connection", "visualization")


def _env(name, default=None, required=False):
    val = os.environ.get(name, default)
    if required and not val:
        raise RuntimeError(f"Missing required environment variable: {name}")
    return val


def _generate_vda5050_topic_alias(vda_version):
    if vda_version in SUPPORTED_PROTOCOL_VERSIONS:
        return f"v{vda_version[0]}"
    raise ValueError(
        f"Invalid protocol major version. Supported versions are: {SUPPORTED_PROTOCOL_VERSIONS},"
        f" but got {vda_version}"
    )


def _connection_json(manufacturer, serial_number, version, state):
    """Build a VDA5050 Connection JSON payload without depending on ROS2 msg types."""
    return json.dumps(
        {
            "headerId": 0,
            "version": version,
            "timestamp": "1970-01-01T12:00:00.00Z",
            "manufacturer": manufacturer,
            "serialNumber": serial_number,
            "connectionState": state,
        }
    )


class MQTTClient:
    """Owns the paho MQTT client + the runtime cert. No ROS2 dependency."""

    def __init__(self):
        self._mqtt_address = _env("MQTT_ADDRESS", "localhost")
        self._mqtt_port = int(_env("MQTT_PORT", "8883"))
        mqtt_username = _env("MQTT_USERNAME", "")
        mqtt_password = _env("MQTT_PASSWORD", "")

        self.vda5050_version = _env("VDA5050_PROTOCOL_VERSION", "2.0.0")
        self.vda5050_version_alias = _generate_vda5050_topic_alias(self.vda5050_version)

        self._manufacturer_name = _env("MANUFACTURER_NAME", required=True)
        self._serial_number = _env("SERIAL_NUMBER", required=True)
        self._interface_name = _env("INTERFACE_NAME", "vda5050")

        self._socket_path = _env("UNIX_SOCKET_PATH", unix_socket_protocol.DEFAULT_SOCKET_PATH)

        # IPC state — one client at a time (the ROS2-side mqtt_bridge node).
        self._ipc_lock = threading.Lock()
        self._ipc_conn: socket.socket | None = None

        self.mqtt_client = mqtt_client.Client(
            client_id=f"{self._manufacturer_name}_{self._serial_number}",
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2,
        )
        self.mqtt_client.on_connect = self._on_connect_mqtt
        self.mqtt_client.on_message = self._on_message_mqtt
        self.mqtt_client.on_disconnect = self._on_disconnect_mqtt

        use_encryption = _env("USE_ENCRYPTION", "1") not in ("0", "false", "False", "")
        if use_encryption:
            context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
            if hasattr(ssl, "TLSVersion"):
                try:
                    context.minimum_version = ssl.TLSVersion.TLSv1_3
                    context.maximum_version = ssl.TLSVersion.TLSv1_3
                except Exception:
                    pass

            ca_path = _env("MQTT_TLS_CA_CERT", "")
            if ca_path:
                context.load_verify_locations(cafile=ca_path)

            cert_file = _env("MQTT_TLS_CLIENT_CERT", "")
            key_file = _env("MQTT_TLS_CLIENT_KEY", "")
            if cert_file and key_file:
                context.load_cert_chain(certfile=cert_file, keyfile=key_file)
            else:
                log.warning(
                    "mTLS: no client cert/key configured — broker will reject the connection. "
                    "Set MQTT_TLS_CLIENT_CERT and MQTT_TLS_CLIENT_KEY."
                )
            self.mqtt_client.tls_set_context(context)

        if mqtt_username:
            self.mqtt_client.username_pw_set(username=mqtt_username, password=mqtt_password)

        will_topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name,
        )
        will_payload = _connection_json(
            self._manufacturer_name, self._serial_number, self.vda5050_version, "CONNECTIONBROKEN"
        )
        self.mqtt_client.will_set(topic=will_topic, payload=will_payload, qos=1, retain=True)

        self._stop = threading.Event()

    # ── MQTT lifecycle ──────────────────────────────────────────
    def _connect_to_broker(self):
        if not self.mqtt_client.is_connected():
            try:
                self.mqtt_client.connect_async(host=self._mqtt_address, port=self._mqtt_port)
                log.info(f"Attempting to connect to MQTT broker at {self._mqtt_address}:{self._mqtt_port}...")
            except Exception as e:
                log.error(f"Error during connection attempt: {e}. Will retry.")

    def _on_connect_mqtt(self, client, userdata, connect_flags, reason_code, properties):
        if reason_code != 0:
            log.error(f"Failed to connect, return code {reason_code}")
            return
        log.info("Connected to MQTT broker.")
        for topic_type in _MQTT_TO_ROS2_TOPICS:
            client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic=topic_type,
                    major_version=self.vda5050_version_alias,
                    interface_name=self._interface_name,
                )
            )
        self._publish_connection_state("ONLINE")

    def _on_disconnect_mqtt(self, client, userdata, disconnect_flags, reason_code, properties):
        if reason_code != 0:
            log.info(f"MQTT client disconnected (rc: {reason_code}, {error_string(reason_code.value)}). Reconnecting.")
        else:
            log.info("Disconnected from MQTT broker.")

    def _on_message_mqtt(self, client, userdata, msg):
        """Forward an MQTT order/instantActions payload to the ROS2 side, verbatim."""
        try:
            # Validate it's well-formed JSON before forwarding, but forward the
            # ORIGINAL bytes — camelCase->snake_case conversion is the ROS2
            # side's job (it needs to build ROS2 message objects from it).
            json.loads(msg.payload)
        except json.decoder.JSONDecodeError:
            log.error(f"Failed to decode MQTT message on {msg.topic}: {msg.payload!r}")
            return

        if msg.topic.endswith("order"):
            topic_type = "order"
        elif msg.topic.endswith("instantActions"):
            topic_type = "instantActions"
        else:
            return

        self._forward_to_ipc(topic_type, msg.payload.decode("utf-8"))

    def _publish_to_mqtt(self, topic_type, payload_json):
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic=topic_type,
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name,
        )
        self.mqtt_client.publish(topic, payload_json)

    def _publish_connection_state(self, state):
        self._publish_to_mqtt(
            "connection",
            _connection_json(self._manufacturer_name, self._serial_number, self.vda5050_version, state),
        )

    # ── IPC server (accepts the ROS2-side mqtt_bridge node) ─────
    def _forward_to_ipc(self, topic_type, payload_json):
        with self._ipc_lock:
            if self._ipc_conn is None:
                log.debug(f"No IPC client connected — dropping {topic_type} frame")
                return
            try:
                unix_socket_protocol.send_frame(self._ipc_conn, topic_type, payload_json)
            except OSError as e:
                log.warning(f"IPC send failed ({e}), dropping client")
                self._close_ipc_locked()

    def _close_ipc_locked(self):
        if self._ipc_conn is not None:
            try:
                self._ipc_conn.close()
            except OSError:
                pass
            self._ipc_conn = None
            log.warning("ROS2-side IPC client disconnected — publishing OFFLINE on its behalf")
            try:
                self._publish_connection_state("OFFLINE")
            except Exception as e:
                log.error(f"Failed to publish OFFLINE connection state: {e}")

    def _ipc_accept_loop(self):
        try:
            os.unlink(self._socket_path)
        except FileNotFoundError:
            pass

        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self._socket_path)
        os.chmod(self._socket_path, 0o660)
        srv.listen(1)
        log.info(f"IPC socket listening at {self._socket_path}")

        while not self._stop.is_set():
            srv.settimeout(1.0)
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            log.info("ROS2-side mqtt_bridge connected over IPC")
            with self._ipc_lock:
                if self._ipc_conn is not None:
                    self._close_ipc_locked()
                self._ipc_conn = conn
            self._ipc_read_loop(conn)

    def _ipc_read_loop(self, conn):
        conn.settimeout(1.0)
        while not self._stop.is_set():
            try:
                frame = unix_socket_protocol.recv_frame(conn)
            except socket.timeout:
                continue
            except (ConnectionError, OSError, ValueError, json.JSONDecodeError) as e:
                log.warning(f"IPC read loop ending ({e})")
                break

            topic_type = frame.get("topic_type")
            payload = frame.get("payload")
            if topic_type not in _ROS2_TO_MQTT_TOPICS or payload is None:
                log.warning(f"Ignoring malformed IPC frame: {frame!r}")
                continue
            self._publish_to_mqtt(topic_type, payload)

        with self._ipc_lock:
            if self._ipc_conn is conn:
                self._close_ipc_locked()

    # ── Entrypoint ───────────────────────────────────────────────
    def run(self):
        self._connect_to_broker()
        self.mqtt_client.loop_start()

        ipc_thread = threading.Thread(target=self._ipc_accept_loop, daemon=True)
        ipc_thread.start()

        try:
            while not self._stop.is_set():
                if not self.mqtt_client.is_connected():
                    self._connect_to_broker()
                time.sleep(5.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        log.info("Shutting down — publishing OFFLINE connection state")
        try:
            self._publish_connection_state("OFFLINE")
        except Exception:
            pass
        self._stop.set()
        with self._ipc_lock:
            self._close_ipc_locked()
        self.mqtt_client.disconnect()
        self.mqtt_client.loop_stop()
        try:
            os.unlink(self._socket_path)
        except FileNotFoundError:
            pass


def main():
    daemon = MQTTClient()
    daemon.run()


if __name__ == "__main__":
    main()
