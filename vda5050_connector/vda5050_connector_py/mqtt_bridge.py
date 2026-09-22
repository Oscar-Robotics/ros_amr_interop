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

# Python dependencies
import copy
import json
import socket
import threading

# ROS dependencies / utils
from rclpy.node import Node

from vda5050_connector_py import unix_socket_protocol
from vda5050_connector_py.utils import get_vda5050_ros2_topic
from vda5050_connector_py.utils import json_camel_to_snake_case
from vda5050_connector_py.utils import convert_ros_message_to_json
from vda5050_connector_py.ros_utils import read_str_parameter

# ROS msgs / srvs / actions
from vda5050_msgs.msg import Action as VDAAction
from vda5050_msgs.msg import ActionParameter as VDAActionParameter
from vda5050_msgs.msg import Connection as VDAConnection
from vda5050_msgs.msg import ControlPoint as VDAControlPoint
from vda5050_msgs.msg import Edge as VDAEdge
from vda5050_msgs.msg import InstantActions as VDAInstantActions
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import NodePosition as VDANodePosition
from vda5050_msgs.msg import Order as VDAOrder
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import Corridor as VDACorridor
from vda5050_msgs.msg import Trajectory as VDATrajectory
from vda5050_msgs.msg import Visualization as VDAVisualization

NODE_NAME = "mqtt_bridge"


def generate_vda_order_msg(order):
    """
    Convert an Order message into a ROS2 Order message represented as a dict.

    Args:
    ----
        order (VDAOrder): VDA5050 Order message

    Returns
    -------
        Order dict message for building a ROS2 Order object

    """
    vda_order = copy.deepcopy(order)
    for node in vda_order["nodes"]:
        # Force all numbers to float. Values with no decimals are
        # interpret as integers, causing the validation errors
        for k in ["x", "y", "theta"]:
            node["node_position"][k] = float(node["node_position"][k])
        node["node_position"] = VDANodePosition(**node["node_position"])
        for action in node["actions"]:
            if "action_parameters" in action:
                action["action_parameters"] = [
                    VDAActionParameter(
                        key=action_parameter["key"],
                        value=str(action_parameter["value"]),
                    )
                    for action_parameter in action["action_parameters"]
                ]
        node["actions"] = [VDAAction(**action) for action in node["actions"]]

    vda_order["nodes"] = [VDANode(**node) for node in vda_order["nodes"]]
    for edge in vda_order["edges"]:
        for action in edge["actions"]:
            if "action_parameters" in action:
                action["action_parameters"] = [
                    VDAActionParameter(
                        key=action_parameter["key"],
                        value=str(action_parameter["value"]),
                    )
                    for action_parameter in action["action_parameters"]
                ]
        edge["actions"] = [VDAAction(**action) for action in edge["actions"]]

        # Force all numbers to float. Values with no decimals are
        # interpreted as integers, causing the validation errors.
        for k in [
            "max_speed",
            "max_height",
            "min_height",
            "orientation",
            "max_rotation_speed",
            "length",
        ]:
            if k in edge:
                edge[k] = float(edge[k])

        if "trajectory" in edge:
            edge["trajectory"] = VDATrajectory(
                degree=float(edge["trajectory"]["degree"]),
                knot_vector=edge["trajectory"]["knot_vector"],
                control_points=[
                    VDAControlPoint(
                        x=float(cp["x"]),
                        y=float(cp["y"]),
                        orientation=float(cp["orientation"]),
                        weight=float(cp.get("weight", 1)),
                    )
                    for cp in edge["trajectory"]["control_points"]
                ],
            )

        if "corridor" in edge:
            c = edge["corridor"]
            edge["has_corridor"] = True
            edge["corridor"] = VDACorridor(
                left_width=float(c["left_width"]),
                right_width=float(c["right_width"]),
                corridor_ref_point=c.get("corridor_ref_point", ""),
                release_required=bool(c.get("release_required", False)),
                release_loss_behavior=c.get("release_loss_behavior", ""),
            )
        else:
            edge["has_corridor"] = False

    vda_order["edges"] = [VDAEdge(**edge) for edge in vda_order["edges"]]
    # TODO(@leandropineda): Consider returning a ROS2 Order message
    return vda_order


def generate_vda_instant_action_msg(instant_action):
    """
    Convert an Instant Action message into a ROS2 Instant Action message represented as a dict.

    Args:
    ----
        instant_action (VDAInstantActions): VDA5050 Instant Action message

    Returns
    -------
        Instant Action dict message for building a ROS2 Instant Action object

    """
    # Values on the `instant_action` parameter will be modified. Create
    # a copy of the object to avoid overrides
    vda_instant_action = copy.deepcopy(instant_action)

    # HACK: VDA5050 instantActions message schema differs from v1 to v2. In particular,
    # the ``instantActions`` field from v1 has been renamed to ``actions`` on v2.
    is_v1 = "instant_actions" in vda_instant_action.keys()

    instant_actions_field = "instant_actions" if is_v1 else "actions"

    for action in vda_instant_action[instant_actions_field]:
        try:
            # Force all action parameter values to string. VDA5050 supports
            # value types `array`, `boolean`, `number` and `string` but
            # VDAActionParameter expects str values
            for action_parameters in action["action_parameters"]:
                action_parameters["value"] = str(action_parameters["value"])
            action["action_parameters"] = [
                VDAActionParameter(**action_parameter)
                for action_parameter in action["action_parameters"]
            ]
        except KeyError:
            # Action parameters are not required
            pass

    vda_instant_action["actions"] = [
        VDAAction(**action) for action in vda_instant_action[instant_actions_field]
    ]

    # HACK: As the internal representation of all messages
    # is v2 only, remove the v1 ``instant_actions`` field
    if is_v1:
        vda_instant_action.pop("instant_actions")

    # TODO(@leandropineda): Consider returning a ROS2 Order message
    return vda_instant_action


class MQTTBridge(Node):
    """Translates VDA5050 messages between ROS2 topics and the local mqtt_client IPC socket."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        self._manufacturer_name = read_str_parameter(
            self, "manufacturer_name", "robots"
        )
        self._serial_number = read_str_parameter(self, "serial_number", "robot_1")
        self._interface_name = read_str_parameter(self, "interface_name", "vda5050")
        self._socket_path = read_str_parameter(
            self, "unix_socket_path", unix_socket_protocol.DEFAULT_SOCKET_PATH
        )

        self._sock = None
        self._sock_lock = threading.Lock()
        self._stop = threading.Event()

        self._connect_to_mqtt_client()
        self._connect_timer = self.create_timer(
            timer_period_sec=5.0,
            callback=self._connect_to_mqtt_client,
        )

        self.on_configure()

        self.logger.info(f"Node {NODE_NAME} has started successfully.")

    # ── Unix socket client (connects to mqtt_client_daemon) ─────────────
    def _connect_to_mqtt_client(self):
        with self._sock_lock:
            if self._sock is not None:
                return
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.connect(self._socket_path)
                self._sock = sock
                self.logger.info(f"Connected to mqtt_client_daemon at {self._socket_path}")
            except OSError as e:
                self.logger.info(
                    f"Attempting to connect to mqtt_client_daemon at {self._socket_path}... ({e})"
                )
                return

        reader = threading.Thread(target=self._ipc_read_loop, args=(sock,), daemon=True)
        reader.start()

    def _ipc_read_loop(self, sock):
        sock.settimeout(1.0)
        while not self._stop.is_set():
            try:
                frame = unix_socket_protocol.recv_frame(sock)
            except socket.timeout:
                continue
            except (ConnectionError, OSError, ValueError, json.JSONDecodeError) as e:
                self.logger.info(f"Lost connection to mqtt_client_daemon ({e}). Will reconnect.")
                break

            topic_type = frame.get("topic_type")
            payload = frame.get("payload")
            if payload is None:
                continue
            try:
                self._handle_daemon_frame(topic_type, payload)
            except KeyError as ex:
                self.logger.warn(f"Ignoring invalid VDA5050 message: {ex}.")

        with self._sock_lock:
            if self._sock is sock:
                self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass

    def _handle_daemon_frame(self, topic_type, payload_json):
        msg_json = json_camel_to_snake_case(payload_json)
        self.logger.debug(f"Received '{msg_json}' for topic_type '{topic_type}'")

        if topic_type == "order":
            vda_order_msg = VDAOrder(**generate_vda_order_msg(msg_json))
            self._order_pub.publish(msg=vda_order_msg)
        elif topic_type == "instantActions":
            vda_instant_actions_message = VDAInstantActions(
                **generate_vda_instant_action_msg(msg_json)
            )
            self._instant_actions_pub.publish(msg=vda_instant_actions_message)

    def _send_to_mqtt_client(self, topic_type, payload_json):
        with self._sock_lock:
            sock = self._sock
            if sock is None:
                self.logger.debug(f"Not connected to mqtt_client_daemon — dropping {topic_type} frame")
                return
            try:
                unix_socket_protocol.send_frame(sock, topic_type, payload_json)
            except OSError as e:
                self.logger.warn(f"IPC send failed ({e}), will reconnect")
                self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass

    def on_configure(self):
        """
        Subscribe to relevant ROS2 topics.

        This method registers callbacks for translating
        ROS2 messages to VDA5050 MQTT messages transmitted to mqtt_client.
        """
        self.logger.info("Configuring ROS topics")
        self._state_sub = self.create_subscription(
            msg_type=VDAOrderState,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="state",
                interface_name=self._interface_name
            ),
            callback=self._publish_state,
            qos_profile=10,
        )

        self._connection_sub = self.create_subscription(
            msg_type=VDAConnection,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="connection",
                interface_name=self._interface_name
            ),
            callback=self._publish_connection,
            qos_profile=10,
        )

        self._visualization_sub = self.create_subscription(
            msg_type=VDAVisualization,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="visualization",
                interface_name=self._interface_name
            ),
            callback=self._publish_visualization,
            qos_profile=10,
        )

        self._order_pub = self.create_publisher(
            msg_type=VDAOrder,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="order",
                interface_name=self._interface_name
            ),
            qos_profile=10,
        )

        self._instant_actions_pub = self.create_publisher(
            msg_type=VDAInstantActions,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="instantActions",
                interface_name=self._interface_name
            ),
            qos_profile=10,
        )
        self.logger.info("Finished configuring ROS topics")

    def on_shutdown(self):
        """
        Perform all necessary teardown steps.

        Closing the IPC socket is enough — mqtt_client_daemon detects the
        disconnect and publishes the VDA5050 OFFLINE connection state on
        this robot's behalf (see MqttClientDaemon._close_ipc_locked). The
        two processes no longer share a lifetime, so the daemon — not this
        node — is responsible for that safety net now.
        """
        self.logger.info("Closing connection to mqtt_client_daemon")
        self._stop.set()
        if hasattr(self, '_connect_timer'):
            self._connect_timer.cancel()
        with self._sock_lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    def _publish_state(self, msg: VDAOrderState):
        self._send_to_mqtt_client("state", convert_ros_message_to_json(msg))

    def _publish_connection(self, msg: VDAConnection):
        self._send_to_mqtt_client("connection", convert_ros_message_to_json(msg))

    def _publish_visualization(self, msg: VDAVisualization):
        self._send_to_mqtt_client("visualization", convert_ros_message_to_json(msg))
