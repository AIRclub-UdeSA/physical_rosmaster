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

"""
Check config/robot_contract.yaml against the simulator's parity ledger.

The simulator (AIRclub-UdeSA/yahboom_rosmaster) grades itself with this
repository's tools/physical_contract_probe.py, in its own CI (#43 step 9). This
repository has no Gazebo, so what it can check is that the contract it publishes
says what the simulator's ledger says the robot delivers, at the simulator
commit named by ``simulator_reference.commit``: the topics, types and frame ids,
the required frames, the hardware-only topics, the ones the simulator publishes
too, and the wheel joints. It also checks the probe's own constants against the
contract, so the three cannot drift apart.

The ledger is yahboom_rosmaster_gazebo/config/real_robot_contract.yaml at that
commit. CI checks it out into a sparse checkout and passes its path in
SIMULATOR_LEDGER. Without it the tests skip on a workstation and fail in CI: a
check that quietly stops running proves nothing.
"""

import os
from pathlib import Path
import re
import subprocess

import pytest
import yaml

import physical_contract_probe as probe_module


CONTRACT_PATH = Path(__file__).resolve().parents[1] / "config" / "robot_contract.yaml"
ENVIRONMENT_VARIABLE = "SIMULATOR_LEDGER"
LEDGER_RELATIVE = Path("yahboom_rosmaster_gazebo") / "config" / "real_robot_contract.yaml"
FULL_SHA = re.compile(r"^[0-9a-f]{40}$")
IN_CI = bool(os.environ.get("GITHUB_ACTIONS") or os.environ.get("CI"))


def load_yaml(path):
    """Return the parsed YAML file at ``path``."""
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


CONTRACT = load_yaml(CONTRACT_PATH)


@pytest.fixture(scope="module")
def ledger_path():
    """Return the simulator ledger's path, or skip (workstation) or fail (CI) without one."""
    value = os.environ.get(ENVIRONMENT_VARIABLE)
    if not value or not Path(value).is_file():
        message = (
            "%s must name the simulator's real_robot_contract.yaml at the commit in "
            "robot_contract.yaml simulator_reference (got %r)" % (ENVIRONMENT_VARIABLE, value)
        )
        if IN_CI:
            pytest.fail(message)
        pytest.skip(message)
    return Path(value)


@pytest.fixture(scope="module")
def ledger(ledger_path):
    """Return the simulator ledger's physical section."""
    return load_yaml(ledger_path)["physical"]


def test_the_simulator_reference_is_a_full_sha():
    """A short or moving reference would let the check read a different ledger."""
    reference = CONTRACT["simulator_reference"]
    assert FULL_SHA.match(reference["commit"]), reference["commit"]
    assert reference["repository"].endswith("/yahboom_rosmaster.git")


def test_the_ledger_is_the_one_at_the_pinned_commit(ledger_path):
    """In CI the checkout must be exactly the commit the contract names."""
    root = ledger_path.parents[2]
    if not (root / ".git").exists():
        if IN_CI:
            pytest.fail("the ledger checkout %s is not a git repository" % root)
        pytest.skip("not a git checkout")
    head = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    if head != CONTRACT["simulator_reference"]["commit"] and not IN_CI:
        pytest.skip("workstation checkout is at %s, not the pin" % head)
    assert head == CONTRACT["simulator_reference"]["commit"]
    assert ledger_path == root / LEDGER_RELATIVE


@pytest.mark.parametrize("topic", sorted(CONTRACT["topics"]))
def test_topic_matches_the_ledger(topic, ledger):
    """Type, frame ids and encoding are what the ledger records for the robot."""
    declared = CONTRACT["topics"][topic]
    recorded = ledger["topics"][topic]
    assert declared["type"] == recorded["type"]
    for key in ("frame_id", "child_frame_id", "encoding"):
        if key in declared:
            assert declared[key] == recorded[key], "%s %s" % (topic, key)


def test_joint_states_declare_the_frame_the_driver_sets(ledger):
    """The driver sets a non-empty frame_id; the probe requires one."""
    assert CONTRACT["topics"]["/joint_states"]["frame_id"] == "joint_states"
    assert ledger["topics"]["/joint_states"]["frame_id"] == "joint_states"


def test_wheel_joints_match_the_ledger(ledger):
    assert CONTRACT["topics"]["/joint_states"]["joints"] == ledger["joint_states"]["names"]


def test_required_frames_are_the_ledgers_frames(ledger):
    """base_footprint, base_link and every mount or camera frame the ledger lists."""
    frames = ledger["frames"]
    expected = {"base_footprint", "base_link"} | set(frames["mounts"]) | set(
        frames["camera_frames"]
    )
    assert set(CONTRACT["tf"]["required_frames"]) == expected


def test_hardware_extensions_match_the_ledger(ledger):
    """What the simulator does not replicate, and what it publishes too."""
    recorded = ledger["hardware_extensions"]
    assert set(CONTRACT["hardware_extensions"]) == set(recorded["topics"])
    assert set(CONTRACT["simulator_published_extensions"]) == set(recorded["simulator_published"])
    # /imu/data_raw is the Madgwick filter's input, on the robot and in the simulator.
    assert "/imu/data_raw" in CONTRACT["simulator_published_extensions"]
    assert "/imu/data_raw" not in CONTRACT["hardware_extensions"]
    assert not set(CONTRACT["hardware_extensions"]) & set(CONTRACT["simulator_published_extensions"])


def test_the_probe_grades_what_the_contract_declares():
    """The probe's constants and the contract cannot disagree (no ledger needed)."""
    probe_types = {
        topic: expected
        for topic, (_, expected) in probe_module.TOPIC_TYPES.items()
        if topic not in ("/diagnostics", "/tf", "/tf_static")
    }
    contract_types = {topic: entry["type"] for topic, entry in CONTRACT["topics"].items()}
    # /imu/data_raw is declared as a simulator-published extension, not graded.
    assert probe_types == contract_types
    assert probe_module.EXPECTED_WHEEL_JOINTS == set(CONTRACT["topics"]["/joint_states"]["joints"])
    assert probe_module.REQUIRED_STATIC_FRAMES == set(CONTRACT["tf"]["required_frames"]) - {
        probe_module.REQUIRED_DYNAMIC_TF_EDGE[1]
    }
    dynamic = CONTRACT["tf"]["dynamic"][0]
    assert (dynamic["parent"], dynamic["child"]) == probe_module.REQUIRED_DYNAMIC_TF_EDGE
