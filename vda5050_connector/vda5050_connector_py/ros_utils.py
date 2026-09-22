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
#
# Split out of utils.py 2026-09-18 (cybersecurity/issues/01): everything
# here genuinely needs a live rclpy Node (declare_parameter/get_parameter
# are real ROS2 API calls), so importing this module pulls in the full
# rclpy native stack. utils.py now holds only the pure-Python VDA5050
# helpers that mqtt_client_daemon.py (deliberately not a ROS2 node) needs
# to import without dragging rclpy in — see that module's docstring.

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType


def read_bool_parameter(node: Node, param_name: str, alternative: bool) -> bool:
    """Declare and read a bool parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == bool else param.get_parameter_value().bool_value


def read_str_parameter(node: Node, param_name: str, alternative: str) -> str:
    """Declare and read a string parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == str else param.get_parameter_value().string_value


def read_int_parameter(node: Node, param_name: str, alternative: int) -> int:
    """Declare and read a int parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == int else param.get_parameter_value().integer_value


def read_double_parameter(node: Node, param_name: str, alternative: float) -> float:
    """Declare and read a double (float) parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == float else param.get_parameter_value().double_value


def read_str_array_parameter(node: Node, param_name: str, alternative: list) -> list:
    """Declare and read a string array parameter."""
    node.declare_parameter(
        param_name,
        descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        value=alternative,
    )
    param = node.get_parameter(param_name)
    return param if type(param) == list else param.get_parameter_value().string_array_value
