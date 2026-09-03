# Copyright 2026 AIRclub UdeSA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for failure propagation in the strict X3 platform launch."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch_ros.actions import Node


def _load_launch_module():
    """Load the X3 launch file directly from the source tree."""
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "yahboomcar_bringup_X3_launch.py"
    )
    spec = importlib.util.spec_from_file_location("x3_platform_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _process_exit(action, name):
    """Create a deterministic process-exit event for a launch action."""
    return ProcessExited(
        action=action,
        name=name,
        cmd=[name],
        cwd=None,
        env=None,
        pid=1234,
        returncode=1,
    )


def test_motor_driver_exit_requests_whole_platform_shutdown(
    monkeypatch, tmp_path
):
    """A motor-driver exit emits the launch-global platform shutdown event."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    launch_module = _load_launch_module()

    description_share = tmp_path / "yahboomcar_description"
    bringup_share = tmp_path / "yahboomcar_bringup"
    astra_share = tmp_path / "yahboomcar_astra"
    (description_share / "urdf").mkdir(parents=True)
    (bringup_share / "param").mkdir(parents=True)
    (astra_share / "launch").mkdir(parents=True)
    (description_share / "urdf" / "yahboomcar_X3.urdf.xacro").write_text(
        '<robot name="test_x3"/>\n'
    )
    for filename in ("x3_driver.yaml", "x3_odometry.yaml", "imu_filter_param.yaml"):
        (bringup_share / "param" / filename).write_text("{}\n")
    (astra_share / "launch" / "astra_platform.launch.py").write_text(
        "from launch import LaunchDescription\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription()\n"
    )

    shares = {
        "yahboomcar_bringup": bringup_share,
        "yahboomcar_astra": astra_share,
    }
    monkeypatch.setattr(
        launch_module,
        "get_package_share_path",
        lambda package: description_share,
    )
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package: str(shares[package]),
    )

    entities = launch_module.generate_launch_description().entities
    nodes = [entity for entity in entities if isinstance(entity, Node)]
    motor_nodes = [
        node
        for node in nodes
        if node.node_package == "yahboomcar_bringup"
        and node.node_executable == "Mcnamu_driver_X3"
    ]
    assert len(motor_nodes) == 1
    motor_node = motor_nodes[0]
    non_motor_node = next(node for node in nodes if node is not motor_node)

    handlers = [
        entity.event_handler
        for entity in entities
        if isinstance(entity, RegisterEventHandler)
    ]
    motor_exit = _process_exit(motor_node, "driver_node")
    matching_handlers = [handler for handler in handlers if handler.matches(motor_exit)]
    assert len(matching_handlers) == 1
    motor_handler = matching_handlers[0]

    non_motor_exit = _process_exit(non_motor_node, "non_motor_node")
    assert not motor_handler.matches(non_motor_exit)

    actions = motor_handler.handle(motor_exit, LaunchContext())
    assert actions is not None
    assert len(actions) == 1
    assert isinstance(actions[0], EmitEvent)
    assert isinstance(actions[0].event, Shutdown)
    assert actions[0].event.reason == "required platform process exited: motor driver"
