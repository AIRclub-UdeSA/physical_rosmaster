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

"""Hardware-free tests for the Rosmaster library compatibility preflight."""

from pathlib import Path

import pytest

import rosmaster_lib_probe as probe


def _install_source_report(monkeypatch, digest):
    """Make the preflight inspect one deterministic source report."""
    vendor_class = object()
    monkeypatch.setattr(probe, "load_rosmaster_class", lambda: vendor_class)
    monkeypatch.setattr(
        probe,
        "hash_source",
        lambda inspected: (
            Path("/test/Rosmaster_Lib.py"),
            digest,
            57997,
            "# V3.3.9",
        ),
    )
    monkeypatch.setattr(
        probe,
        "sample_hardware",
        lambda *_args, **_kwargs: pytest.fail(
            "--hash-only must not open the serial device"
        ),
    )
    return vendor_class


def test_hash_only_accepts_the_allowlisted_source(monkeypatch, capsys):
    """The reviewed V3.3.9 digest is a successful passive preflight."""
    _install_source_report(monkeypatch, probe.PUBLIC_V3_3_9_SHA256)
    monkeypatch.setattr(
        probe.sys, "argv", ["rosmaster_lib_probe.py", "--hash-only"]
    )

    assert probe.main() == 0
    captured = capsys.readouterr()
    assert "matches_public_v3_3_9: true" in captured.out
    assert captured.err == ""


def test_hash_only_rejects_an_unreviewed_source(monkeypatch, capsys):
    """A printed mismatch must also make the preflight process fail."""
    _install_source_report(monkeypatch, "0" * 64)
    monkeypatch.setattr(
        probe.sys, "argv", ["rosmaster_lib_probe.py", "--hash-only"]
    )

    assert probe.main() == 2
    captured = capsys.readouterr()
    assert "matches_public_v3_3_9: false" in captured.out
    assert "ERROR: unsupported Rosmaster_Lib source" in captured.err
    assert probe.PUBLIC_V3_3_9_SHA256 in captured.err
