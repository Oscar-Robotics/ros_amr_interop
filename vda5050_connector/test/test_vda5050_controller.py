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

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.task import Future

from uuid import uuid4

from vda5050_connector_py.vda5050_controller import OrderAcceptModes
from vda5050_connector_py.vda5050_controller import OrderRejectErrors
from vda5050_connector_py.utils import get_vda5050_ts
from action_msgs.msg import GoalStatus
from vda5050_connector.action import NavigateToNode
from vda5050_connector.srv import GetState

from vda5050_msgs.msg import AGVPosition
from vda5050_msgs.msg import OrderState
from vda5050_msgs.msg import Order
from vda5050_msgs.msg import Node
from vda5050_msgs.msg import Edge
from vda5050_msgs.msg import NodePosition
from vda5050_msgs.msg import Action
from vda5050_msgs.msg import ActionParameter
from vda5050_msgs.msg import InstantActions


def _navigation_succeeded():
    """What the navigate-to-node action client's result future resolves to."""
    return NavigateToNode.Impl.GetResultService.Response(
        status=GoalStatus.STATUS_SUCCEEDED, result=NavigateToNode.Result()
    )


def get_order_new(order_id=str(uuid4()), order_update_id=0):
    return Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=order_update_id,
        nodes=[
            Node(
                node_id="node1",
                sequence_id=0,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node2",
                sequence_id=2,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node3",
                sequence_id=4,
                released=True,
                node_position=NodePosition(
                    x=-0.38,
                    y=1.89,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node4",
                sequence_id=6,
                released=True,
                node_position=NodePosition(
                    x=-0.17,
                    y=1.74,
                    theta=-2.6,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node1",
                sequence_id=8,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
        ],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=1,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge2",
                sequence_id=3,
                released=True,
                start_node_id="node2",
                end_node_id="node3",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge3",
                sequence_id=5,
                released=True,
                start_node_id="node3",
                end_node_id="node4",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge4",
                sequence_id=7,
                released=True,
                start_node_id="node4",
                end_node_id="node1",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )


def get_order_update(order_id=str(uuid4()), order_update_id=0):
    return Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=order_update_id,
        nodes=[
            Node(
                node_id="node1",
                sequence_id=8,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node2",
                sequence_id=10,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
        ],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=9,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            )
        ],
    )


def get_stitch_orders(order_id=str(uuid4())):
    action1 = Action(
        action_type="foo",
        action_id=str(uuid4()),
        action_description="Foo description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="foo", value="bar")
        ]
    )
    action2 = Action(
        action_type="bar",
        action_id=str(uuid4()),
        action_description="Bar description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="foo", value="bar")
        ]
    )
    action3 = Action(
        action_type="foobar",
        action_id=str(uuid4()),
        action_description="FooBar description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="abc", value="xyz")
        ]
    )
    base_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=0,
        nodes=[
            Node(
                node_id="node1",
                sequence_id=0,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action1
                ],
            ),
            Node(
                node_id="node2",
                sequence_id=2,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action2
                ],
            ),
        ],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=1,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )

    stitch_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=1,
        nodes=[
            Node(
                node_id="node2",
                sequence_id=2,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action2
                ],
            ),
            Node(
                node_id="node3",
                sequence_id=4,
                released=True,
                node_position=NodePosition(
                    x=-0.38,
                    y=1.89,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action3
                ],
            ),
        ],
        edges=[
            Edge(
                edge_id="edge2",
                sequence_id=3,
                released=True,
                start_node_id="node2",
                end_node_id="node3",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )

    return [base_order, stitch_order]


def test_vda5050_controller_node_new_order(
    mocker,
    adapter_node,
    controller_node,
):

    node = controller_node
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate used navigation goal parameters
    spy_send_adapter_navigate_to_node = mocker.spy(
        node, "send_adapter_navigate_to_node"
    )

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # generate an order and let the node process it
    order_id = str(uuid4())
    order = get_order_new(order_id)
    node.process_order(order)

    rclpy.spin_once(node)

    spy_accept_order.assert_called_once_with(order=order, mode=OrderAcceptModes.NEW)

    rclpy.spin_once(adapter_node)

    # check node states were properly updated
    assert node._current_order == order
    assert node._current_state.order_id == order_id
    assert node._current_state.order_update_id == 0

    # The order has 5 nodes and 4 edges but the first edge and node
    # are processed as soon as the order is accepted.
    assert len(node._current_state.node_states) == 4
    assert len(node._current_state.edge_states) == 4

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 0

    # Assert the first navigation goal was sent to the adapter,
    # and that the parameters matches order's first edge and second node.
    # Note: the standard assumes the vehicle is on the first node already,
    # so the first navigation command is to the second order node.
    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[0], node=order.nodes[1]
    )

    # Future for invoking adapter navigation goal result callback
    future = Future()
    future.set_result(result=_navigation_succeeded())

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[1], node=order.nodes[2]
    )

    assert len(node._current_state.node_states) == 3
    assert len(node._current_state.edge_states) == 3
    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 2

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[2], node=order.nodes[3]
    )

    assert len(node._current_state.node_states) == 2
    assert len(node._current_state.edge_states) == 2
    assert node._current_state.last_node_id == "node3"
    assert node._current_state.last_node_sequence_id == 4

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[3], node=order.nodes[4]
    )
    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1
    assert node._current_state.last_node_id == "node4"
    assert node._current_state.last_node_sequence_id == 6

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 8


def test_vda5050_controller_node_update_order(
    mocker,
    adapter_node,
    controller_node,
):
    node = controller_node
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate used navigation goal parameters
    spy_send_adapter_navigate_to_node = mocker.spy(
        node, "send_adapter_navigate_to_node"
    )

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # Send first new order
    order_id = str(uuid4())
    order = get_order_new(order_id)
    node.process_order(order)

    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)

    # Simulate the adapter reached navigation goals
    future = Future()
    future.set_result(result=_navigation_succeeded())

    # The NEW order contains 5 nodes and 4 edges. The first node (in deviation range)
    # is processed and remove, and 4 nodes are send to navigate to.
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    # Finish initial order

    spy_accept_order.reset_mock()
    spy_send_adapter_navigate_to_node.reset_mock()

    # Send update order
    order = get_order_update(order_id, 1)  # Same order id
    node.process_order(order)
    node._on_active_order()

    spy_accept_order.assert_called_once_with(order=order, mode=OrderAcceptModes.UPDATE)

    # check node states were properly updated
    assert node._current_order == order
    assert node._current_state.order_id == order_id
    assert node._current_state.order_update_id == 1

    # The order has 2 nodes and 1 edges but the first edge and node
    # are processed as soon as the order is accepted.
    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 8

    # Assert the first navigation goal was sent to the adapter,
    # and that the parameters matches order's first edge and second node.
    # Note: the standard assumes the vehicle is on the first node already,
    # so the first navigation command is to the second order node.
    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[0], node=order.nodes[1]
    )

    # Future for invoking adapter navigation goal result callback
    future = Future()
    future.set_result(result=_navigation_succeeded())

    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 10


def test_vda5050_controller_node_stitch_order(
    mocker,
    adapter_node,
    controller_node,
):
    node = controller_node
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # Get the base order and the stitch order, both with two nodes
    # and one edge each. The resulting (stitched) order should contain
    # three nodes and two edges:
    # (A -> B) + (B -> C) = A -> B -> C
    # All three nodes have an action.
    [base_order, stitch_order] = get_stitch_orders()

    node.process_order(base_order)

    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)

    # Simulate the adapter reached navigation goals
    future = Future()
    future.set_result(result=_navigation_succeeded())

    # The base order contains 2 nodes and 1 edge. The first node (in deviation range)
    # is processed and removed, and 1 node is sent to navigate to.
    node._navigate_to_node_result_callback(future)

    assert len(node._current_order.nodes) == 2
    assert len(node._current_order.edges) == 1
    assert len(node._current_state.action_states) == 2

    spy_accept_order.reset_mock()
    # Process the stitching order
    node.process_order(stitch_order)
    spy_accept_order.assert_called_once_with(order=stitch_order, mode=OrderAcceptModes.STITCH)

    assert node._current_state.order_id == stitch_order.order_id
    assert node._current_state.order_update_id == 1

    assert len(node._current_order.nodes) == 3
    assert len(node._current_order.edges) == 2

    assert len(node._current_state.node_states) == 2
    assert len(node._current_state.edge_states) == 1
    assert len(node._current_state.action_states) == 3

    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 2

    node._navigate_to_node_result_callback(future)
    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 0
    assert len(node._current_state.action_states) == 3

    assert node._current_state.last_node_id == "node3"
    assert node._current_state.last_node_sequence_id == 4


def test_vda5050_controller_node_reject_order(
    mocker,
    controller_node,
):
    node = controller_node
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate that the order has been rejected
    spy_reject_order = mocker.spy(node, "_reject_order")

    # UPDATE test fail - lower order_update_id

    # Send first new order
    order_id = str(uuid4())
    order = get_order_new(order_id, 1)
    node.process_order(order)

    # Simulate the adapter reached navigation goals
    future = Future()
    future.set_result(result=_navigation_succeeded())

    # The NEW order contains 5 nodes and 4 edges. The first node (in deviation range)
    # is processed and remove, and 4 nodes are send to navigate to.
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    # Finish initial order

    spy_reject_order.reset_mock()

    order = get_order_update(order_id, 0)  # Same order id, lower order_update_id
    node.process_order(order)

    spy_reject_order.assert_called_once_with(
        order=order,
        error=OrderRejectErrors.ORDER_UPDATE_ERROR,
        description="New update id 0 lower than old update id 1",
    )


def test_get_state_from_adapter_never_blocks_the_node(
    mocker,
    adapter_node,
    controller_node,
):
    """A dropped/slow GetState response must never wedge the node — only the
    async path (call_async) is used, never the blocking call(). The client
    methods are fully replaced (not spied) so a real blocking call() — the
    exact production hang — can't execute even if this hasn't been fixed yet."""
    node = controller_node

    mock_call = mocker.patch.object(
        node._get_adapter_state_svc_cli,
        "call",
        return_value=GetState.Response(state=OrderState(agv_position=AGVPosition())),
    )
    mock_call_async = mocker.patch.object(
        node._get_adapter_state_svc_cli, "call_async", return_value=Future()
    )

    node.get_state_from_adapter()

    mock_call.assert_not_called()
    mock_call_async.assert_called_once()


def test_get_state_from_adapter_skips_a_call_while_one_is_in_flight(
    mocker,
    controller_node,
):
    """A second request must not pile up on the client while the first has
    not resolved yet — cheap insurance against a period shorter than the
    round-trip time."""
    node = controller_node

    mock_call_async = mocker.patch.object(
        node._get_adapter_state_svc_cli, "call_async", return_value=Future()
    )

    node.get_state_from_adapter()
    node.get_state_from_adapter()

    mock_call_async.assert_called_once()


def test_get_state_from_adapter_allows_a_new_call_once_the_response_arrives(
    mocker,
    controller_node,
):
    """The in-flight guard must clear once the response is delivered, so the
    next timer tick isn't permanently skipped."""
    node = controller_node

    future = Future()
    mock_call_async = mocker.patch.object(
        node._get_adapter_state_svc_cli, "call_async", return_value=future
    )

    node.get_state_from_adapter()
    future.set_result(GetState.Response(state=OrderState(agv_position=AGVPosition())))

    node.get_state_from_adapter()

    assert mock_call_async.call_count == 2


def test_publish_visualization_never_blocks_the_node(
    mocker,
    controller_node,
):
    """The visualization timer's periodic state refresh must go through the
    same never-blocks path as get_state_from_adapter()."""
    node = controller_node

    mock_call = mocker.patch.object(node._get_adapter_state_svc_cli, "call")
    mock_call_async = mocker.patch.object(
        node._get_adapter_state_svc_cli, "call_async", return_value=Future()
    )

    node._publish_visualization()

    mock_call.assert_not_called()
    mock_call_async.assert_called_once()


# ---- Messages resent by master control over a lossy link ----

def _instant_actions(*actions):
    return InstantActions(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="2.0.0",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        actions=[
            Action(action_id=action_id, action_type=action_type, blocking_type="HARD")
            for action_id, action_type in actions
        ],
    )


def _reported_action_ids(node):
    return [a.action_id for a in node._current_state.action_states]


def test_instant_action_with_a_known_action_id_is_ignored(controller_node):
    node = controller_node
    node.process_instant_actions(_instant_actions(("cancel-1", "cancelOrder")))
    node._on_active_order()

    node.process_instant_actions(_instant_actions(("cancel-1", "cancelOrder")))
    node._on_active_order()

    assert _reported_action_ids(node).count("cancel-1") == 1


def _navigation_cancelled():
    return NavigateToNode.Impl.GetResultService.Response(
        status=GoalStatus.STATUS_CANCELED, result=NavigateToNode.Result()
    )


def _action_status(node, action_id):
    return [a.action_status for a in node._current_state.action_states if a.action_id == action_id]


def test_cancel_received_while_another_cancel_runs_completes_with_it(
    adapter_node,
    controller_node,
):
    node = controller_node
    node.process_order(get_order_new(str(uuid4())))
    for _ in range(10):
        if node._is_navigation_active():
            break
        rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.spin_once(adapter_node, timeout_sec=0.1)
    assert node._is_navigation_active()

    node.process_instant_actions(_instant_actions(("cancel-1", "cancelOrder")))
    node._on_active_order()
    node.process_instant_actions(_instant_actions(("cancel-2", "cancelOrder")))
    future = Future()
    future.set_result(result=_navigation_cancelled())
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    assert _action_status(node, "cancel-1") == ["FINISHED"]
    assert _action_status(node, "cancel-2") == ["FINISHED"]
    assert not node._has_current_order()


def _drive_new_order_to_its_end(node, adapter_node, order_id):
    node.process_order(get_order_new(order_id))
    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)
    future = Future()
    future.set_result(result=_navigation_succeeded())
    for _ in range(4):
        node._navigate_to_node_result_callback(future)
        node._on_active_order()


def test_order_update_keeps_the_reported_action_states(adapter_node, controller_node):
    node = controller_node
    order_id = str(uuid4())
    _drive_new_order_to_its_end(node, adapter_node, order_id)
    node.process_instant_actions(_instant_actions(("cancel-1", "cancelOrder")))
    node._on_active_order()

    node.process_order(get_order_update(order_id, 1))

    assert "cancel-1" in _reported_action_ids(node)


def test_duplicate_order_is_discarded_before_stitch_validation(mocker, adapter_node, controller_node):
    node = controller_node
    order_id = str(uuid4())
    _drive_new_order_to_its_end(node, adapter_node, order_id)
    node.process_order(get_order_update(order_id, 1))
    spy_match_stitch_nodes = mocker.spy(node, "_match_stitch_nodes")
    spy_reject_order = mocker.spy(node, "_reject_order")

    node.process_order(get_order_update(order_id, 1))

    spy_match_stitch_nodes.assert_not_called()
    spy_reject_order.assert_not_called()
    assert node._current_state.order_update_id == 1


def test_state_is_published_on_receiving_an_order(mocker, controller_node):
    node = controller_node
    spy_publish_state = mocker.spy(node, "_publish_state")

    node.process_order(get_order_new(str(uuid4())))

    spy_publish_state.assert_called()


def test_state_is_published_on_receiving_a_duplicate_order(mocker, adapter_node, controller_node):
    node = controller_node
    order_id = str(uuid4())
    _drive_new_order_to_its_end(node, adapter_node, order_id)
    node.process_order(get_order_update(order_id, 1))
    spy_publish_state = mocker.spy(node, "_publish_state")

    node.process_order(get_order_update(order_id, 1))

    spy_publish_state.assert_called()


def test_state_is_published_on_receiving_instant_actions(mocker, controller_node):
    node = controller_node
    spy_publish_state = mocker.spy(node, "_publish_state")

    node.process_instant_actions(_instant_actions(("cancel-1", "cancelOrder")))

    spy_publish_state.assert_called()
