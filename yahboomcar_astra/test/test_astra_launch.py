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

"""Tests for the strict Astra platform launch."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "astra_platform.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "astra_platform_launch", launch_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _additional_environment(action):
    context = LaunchContext()
    return {
        perform_substitutions(context, name): perform_substitutions(
            context, value
        )
        for name, value in action.additional_env
    }


def test_adapter_limits_openblas_workers():
    """Keep point-cloud conversion independent of the operator's shell."""
    launch_module = _load_launch_module()
    environments = [
        _additional_environment(action)
        for action in launch_module.generate_launch_description().entities
        if isinstance(action, Node)
    ]

    assert {"OPENBLAS_NUM_THREADS": "1"} in environments
