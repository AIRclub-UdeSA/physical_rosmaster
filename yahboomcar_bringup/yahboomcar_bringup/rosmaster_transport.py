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

"""Monitor real Rosmaster report arrivals without trusting cached getters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import hashlib
import inspect
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable, Mapping, Optional


_PRIVATE_PARSE_HOOK = "_Rosmaster__parse_data"
_PRIVATE_RECEIVE_HOOK = "_Rosmaster__receive_data"
_REQUIRED_CHANNELS = ("speed", "encoder", "imu_raw")
_REPORT_CHANNELS = {
    0x0A: "speed",
    0x0B: "imu_raw",
    0x0D: "encoder",
    0x0E: "imu_raw",
}
PUBLIC_V3_3_9_SHA256 = (
    "e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c"
)


class TransportState(Enum):
    """Lifecycle and feedback-health state of a monitored transport."""

    CREATED = "created"
    WAITING = "waiting"
    HEALTHY = "healthy"
    FAILED = "failed"
    CLOSED = "closed"


@dataclass(frozen=True)
class TransportStatus:
    """Immutable report of the transport's current feedback health."""

    state: TransportState
    reason: str
    report_ages: Mapping[str, Optional[float]]
    report_sequence: int

    @property
    def healthy(self) -> bool:
        """Return whether every required controller report is fresh."""
        return self.state is TransportState.HEALTHY


class RosmasterCompatibilityError(RuntimeError):
    """Raised when a vendor class lacks the exact monitored private hooks."""


class RosmasterTransportError(RuntimeError):
    """Raised when an operation requires feedback from an unhealthy transport."""


class _MonitoredSerial:
    """Expose the vendor serial API while making swallowed writes observable."""

    def __init__(self, serial_port: Any, monitor: "RosmasterTransport") -> None:
        self._serial_port = serial_port
        self._monitor = monitor

    def __getattr__(self, name: str) -> Any:
        return getattr(self._serial_port, name)

    def write(self, payload: Any) -> Any:
        """Latch write errors and short writes before vendor bare-except blocks."""
        try:
            written = self._serial_port.write(payload)
        except BaseException as exc:
            self._monitor._serial_write_failed(
                "controller serial write raised %s: %s"
                % (type(exc).__name__, exc)
            )
            raise

        try:
            expected = len(payload)
        except TypeError:
            expected = None
        if (
            expected is not None
            and isinstance(written, int)
            and written != expected
        ):
            reason = "controller serial short write: %d of %d bytes" % (
                written,
                expected,
            )
            self._monitor._serial_write_failed(reason)
            raise OSError(reason)
        return written


class _RosmasterMonitorMixin:
    """Intercept the exact private entry points used by Rosmaster V3.3.9."""

    def __init__(
        self,
        *args: Any,
        _transport_monitor: "RosmasterTransport",
        **kwargs: Any,
    ) -> None:
        self._transport_monitor = _transport_monitor
        super().__init__(*args, **kwargs)

    def _Rosmaster__parse_data(self, ext_type: int, ext_data: Any) -> Any:
        parser = getattr(super(), _PRIVATE_PARSE_HOOK)
        return self._transport_monitor._parse_vendor_report(
            parser, ext_type, ext_data
        )

    def _Rosmaster__receive_data(self) -> None:
        receiver = getattr(super(), _PRIVATE_RECEIVE_HOOK)
        try:
            receiver()
        except BaseException as exc:
            self._transport_monitor._receive_ended(exc)
        else:
            self._transport_monitor._receive_ended(None)

    def __del__(self) -> None:
        """Close a partially initialized vendor serial object without raising."""
        serial_port = getattr(self, "ser", None)
        close = getattr(serial_port, "close", None)
        if callable(close):
            try:
                close()
            except Exception:
                pass


def _monitored_vendor_class(vendor_base: type) -> type:
    """Return a dynamic subclass that intercepts V3.3.9 private methods."""
    if not isinstance(vendor_base, type):
        raise RosmasterCompatibilityError("vendor_base must be a class")

    missing = [
        name
        for name in (_PRIVATE_PARSE_HOOK, _PRIVATE_RECEIVE_HOOK)
        if not callable(getattr(vendor_base, name, None))
    ]
    if missing:
        raise RosmasterCompatibilityError(
            "vendor class lacks exact Rosmaster V3.3.9 private hook(s): %s"
            % ", ".join(missing)
        )

    return type(
        "Monitored%s" % vendor_base.__name__,
        (_RosmasterMonitorMixin, vendor_base),
        {},
    )


class RosmasterTransport:
    """Wrap a Rosmaster-compatible class and monitor actual report arrivals."""

    def __init__(
        self,
        vendor_base: type,
        *vendor_args: Any,
        clock: Callable[[], float] = time.monotonic,
        startup_timeout: float = 2.0,
        stale_timeout: float = 0.5,
        write_timeout: float = 0.05,
        **vendor_kwargs: Any,
    ) -> None:
        """Construct a monitored vendor instance without starting its reader."""
        self._clock = clock
        self._startup_timeout = self._positive_timeout(
            "startup_timeout", startup_timeout
        )
        self._stale_timeout = self._positive_timeout(
            "stale_timeout", stale_timeout
        )
        self._write_timeout = self._positive_timeout(
            "write_timeout", write_timeout
        )
        if not callable(clock):
            raise ValueError("clock must be callable")

        self._lock = threading.RLock()
        self._started_at: Optional[float] = None
        self._last_clock: Optional[float] = None
        self._last_reports = {
            channel: None for channel in _REQUIRED_CHANNELS
        }
        self._report_sequence = 0
        self._failure_reason: Optional[str] = None
        self._serial_write_failures = []
        self._closing = False
        self._closed = False
        self._thread: Optional[threading.Thread] = None

        monitored_class = _monitored_vendor_class(vendor_base)
        self._vendor = monitored_class(
            *vendor_args,
            _transport_monitor=self,
            **vendor_kwargs,
        )
        serial_port = getattr(self._vendor, "ser", None)
        if serial_port is None or not callable(getattr(serial_port, "write", None)):
            raise RosmasterCompatibilityError(
                "vendor instance lacks a writable serial transport"
            )
        try:
            serial_port.write_timeout = self._write_timeout
            effective_write_timeout = float(serial_port.write_timeout)
        except Exception as exc:
            raise RosmasterCompatibilityError(
                "could not configure bounded controller serial writes: %s"
                % exc
            ) from exc
        if (
            not math.isfinite(effective_write_timeout)
            or effective_write_timeout <= 0.0
            or effective_write_timeout > self._write_timeout
        ):
            raise RosmasterCompatibilityError(
                "controller serial write timeout is not bounded at %.6fs"
                % self._write_timeout
            )
        self._vendor.ser = _MonitoredSerial(serial_port, self)

    @staticmethod
    def _positive_timeout(name: str, value: float) -> float:
        """Return a finite positive timeout or reject the configuration."""
        try:
            timeout = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("%s must be a finite positive number" % name) from exc
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("%s must be a finite positive number" % name)
        return timeout

    @property
    def vendor(self) -> Any:
        """Expose the wrapped vendor only for its existing controller API."""
        return self._vendor

    @property
    def receiver_thread(self) -> Optional[threading.Thread]:
        """Return the retained receive thread, if it has been started."""
        with self._lock:
            return self._thread

    @property
    def serial_write_failure_count(self) -> int:
        """Return the number of observed write errors, including swallowed ones."""
        with self._lock:
            return len(self._serial_write_failures)

    @property
    def latest_serial_write_failure(self) -> Optional[str]:
        """Return the latest observed serial-write error, if any."""
        with self._lock:
            if not self._serial_write_failures:
                return None
            return self._serial_write_failures[-1]

    def _latch_failure_locked(self, reason: str) -> None:
        """Record the first terminal failure while the transport is active."""
        if (
            self._failure_reason is None
            and not self._closing
            and not self._closed
        ):
            self._failure_reason = reason

    def _clock_now_locked(self, context: str) -> Optional[float]:
        """Read and validate the injected monotonic clock under the lock."""
        try:
            now = float(self._clock())
        except Exception as exc:
            self._latch_failure_locked(
                "%s clock read failed: %s: %s"
                % (context, type(exc).__name__, exc)
            )
            return None

        if not math.isfinite(now):
            self._latch_failure_locked(
                "%s clock returned non-finite time %r" % (context, now)
            )
            return None
        if self._last_clock is not None and now < self._last_clock:
            self._latch_failure_locked(
                "%s clock moved backwards from %.9f to %.9f"
                % (context, self._last_clock, now)
            )
            return None
        self._last_clock = now
        return now

    def _check_deadlines_locked(self, now: float) -> None:
        """Latch startup or per-channel feedback timeouts at ``now``."""
        if self._failure_reason is not None or self._started_at is None:
            return

        missing = [
            channel
            for channel, received_at in self._last_reports.items()
            if received_at is None
        ]
        if missing:
            if now - self._started_at > self._startup_timeout:
                self._latch_failure_locked(
                    "startup feedback timeout; missing report channel(s): %s"
                    % ", ".join(missing)
                )
            return

        stale = [
            (channel, now - received_at)
            for channel, received_at in self._last_reports.items()
            if received_at is not None
            and now - received_at > self._stale_timeout
        ]
        if stale:
            details = ", ".join(
                "%s=%.6fs" % (channel, age) for channel, age in stale
            )
            self._latch_failure_locked(
                "controller feedback stale; %s (limit %.6fs)"
                % (details, self._stale_timeout)
            )

    def _parse_vendor_report(
        self,
        parser: Callable[[int, Any], Any],
        ext_type: int,
        ext_data: Any,
    ) -> Any:
        """Run the vendor parser and then mark a recognized valid report."""
        with self._lock:
            try:
                result = parser(ext_type, ext_data)
            except BaseException as exc:
                self._latch_failure_locked(
                    "vendor report parser raised %s: %s"
                    % (type(exc).__name__, exc)
                )
                raise

            channel = _REPORT_CHANNELS.get(ext_type)
            if (
                channel is None
                or self._started_at is None
                or self._failure_reason is not None
                or self._closing
                or self._closed
            ):
                return result

            now = self._clock_now_locked("report")
            if now is None:
                return result
            self._check_deadlines_locked(now)
            if self._failure_reason is not None:
                return result

            self._last_reports[channel] = now
            self._report_sequence += 1
            return result

    def _receive_ended(self, error: Optional[BaseException]) -> None:
        """Capture a receive exception or an unexpected normal thread exit."""
        with self._lock:
            if self._closing or self._closed:
                return
            if error is None:
                self._latch_failure_locked(
                    "controller receive thread exited unexpectedly"
                )
            else:
                self._latch_failure_locked(
                    "controller receive thread raised %s: %s"
                    % (type(error).__name__, error)
                )

    def _serial_write_failed(self, reason: str) -> None:
        """Record an outbound error even when V3.3.9 swallows the exception."""
        with self._lock:
            self._serial_write_failures.append(reason)
            self._latch_failure_locked(reason)

    def start(self) -> None:
        """Start one retained daemon thread for the vendor receive loop."""
        with self._lock:
            if self._closed or self._closing:
                raise RosmasterTransportError("transport is closed")
            if self._failure_reason is not None:
                raise RosmasterTransportError(self._failure_reason)
            if self._thread is not None:
                return

            now = self._clock_now_locked("startup")
            if now is None:
                return
            self._started_at = now
            receiver = getattr(self._vendor, _PRIVATE_RECEIVE_HOOK)
            thread = threading.Thread(
                target=receiver,
                name="task_serial_receive_monitored",
                daemon=True,
            )
            self._thread = thread
            try:
                # Publish and start the thread atomically with respect to
                # status() and close(); the receiver can wait on this RLock.
                thread.start()
            except BaseException as exc:
                self._latch_failure_locked(
                    "could not start controller receive thread: %s: %s"
                    % (type(exc).__name__, exc)
                )

    def _report_ages_locked(
        self, now: Optional[float]
    ) -> Mapping[str, Optional[float]]:
        """Return a detached channel-age mapping for a status result."""
        return {
            channel: (
                None
                if now is None or received_at is None
                else max(0.0, now - received_at)
            )
            for channel, received_at in self._last_reports.items()
        }

    def status(self) -> TransportStatus:
        """Evaluate deadlines and return the latched transport state."""
        with self._lock:
            if self._closed or self._closing:
                return TransportStatus(
                    TransportState.CLOSED,
                    "transport is closed",
                    self._report_ages_locked(self._last_clock),
                    self._report_sequence,
                )
            if self._failure_reason is not None:
                now = self._clock_now_locked("failed health")
                return TransportStatus(
                    TransportState.FAILED,
                    self._failure_reason,
                    self._report_ages_locked(
                        now if now is not None else self._last_clock
                    ),
                    self._report_sequence,
                )
            if self._started_at is None:
                return TransportStatus(
                    TransportState.CREATED,
                    "controller receive thread has not started",
                    self._report_ages_locked(None),
                    self._report_sequence,
                )

            now = self._clock_now_locked("health")
            if now is not None:
                self._check_deadlines_locked(now)
            if self._failure_reason is not None:
                return TransportStatus(
                    TransportState.FAILED,
                    self._failure_reason,
                    self._report_ages_locked(now),
                    self._report_sequence,
                )

            thread = self._thread
            if thread is None or not thread.is_alive():
                self._latch_failure_locked(
                    "controller receive thread is not running"
                )
                return TransportStatus(
                    TransportState.FAILED,
                    self._failure_reason or "controller receive thread failed",
                    self._report_ages_locked(now),
                    self._report_sequence,
                )

            missing = [
                channel
                for channel, received_at in self._last_reports.items()
                if received_at is None
            ]
            if missing:
                return TransportStatus(
                    TransportState.WAITING,
                    "waiting for report channel(s): %s" % ", ".join(missing),
                    self._report_ages_locked(now),
                    self._report_sequence,
                )

            return TransportStatus(
                TransportState.HEALTHY,
                "all required controller report channels are fresh",
                self._report_ages_locked(now),
                self._report_sequence,
            )

    def require_healthy(self) -> TransportStatus:
        """Return healthy status or raise with the fail-closed reason."""
        status = self.status()
        if not status.healthy:
            raise RosmasterTransportError(status.reason)
        return status

    def latch_failure(self, reason: str) -> None:
        """Latch a driver-observed cache or validation failure."""
        detail = str(reason).strip()
        if not detail:
            detail = "unspecified controller transport failure"
        with self._lock:
            self._latch_failure_locked(detail)

    def read_cached(self, reader: Callable[[Any], Any]) -> Any:
        """Copy vendor caches atomically while feedback remains authorized."""
        if not callable(reader):
            raise TypeError("reader must be callable")
        with self._lock:
            self.require_healthy()
            try:
                result = reader(self._vendor)
            except BaseException as exc:
                self._latch_failure_locked(
                    "controller cache reader raised %s: %s"
                    % (type(exc).__name__, exc)
                )
                raise
            self.require_healthy()
            return result

    def perform_while_healthy(
        self, label: str, action: Callable[[Any], Any]
    ) -> Any:
        """Run one action atomically with health authorization and recheck it."""
        if not callable(action):
            raise TypeError("action must be callable")
        detail = str(label).strip() or "controller action"
        with self._lock:
            self.require_healthy()
            try:
                result = action(self._vendor)
            except BaseException as exc:
                self._latch_failure_locked(
                    "%s raised %s: %s" % (detail, type(exc).__name__, exc)
                )
                raise
            self.require_healthy()
            return result

    def close(self, join_timeout: float = 0.25) -> None:
        """Close serial safely and briefly join the retained receive thread."""
        with self._lock:
            if self._closed:
                return
            self._closing = True
            vendor = getattr(self, "_vendor", None)
            thread = self._thread

        serial_port = getattr(vendor, "ser", None)
        close = getattr(serial_port, "close", None)
        if callable(close):
            try:
                close()
            except Exception:
                pass

        if (
            thread is not None
            and thread is not threading.current_thread()
            and thread.is_alive()
        ):
            thread.join(timeout=max(0.0, float(join_timeout)))

        with self._lock:
            self._closed = True
            self._closing = False


def rosmaster_source_sha256(vendor_base: type) -> tuple[Path, str]:
    """Locate and hash an importable Rosmaster implementation's source."""
    try:
        source_file = inspect.getsourcefile(vendor_base)
    except (OSError, TypeError) as exc:
        raise RosmasterCompatibilityError(
            "could not locate installed Rosmaster source: %s" % exc
        ) from exc
    if source_file is None:
        raise RosmasterCompatibilityError(
            "could not locate installed Rosmaster source"
        )

    source_path = Path(source_file)
    try:
        source_bytes = source_path.read_bytes()
    except OSError:
        module = sys.modules.get(vendor_base.__module__)
        module_spec = getattr(module, "__spec__", None)
        loader = getattr(module, "__loader__", None) or getattr(
            module_spec, "loader", None
        )
        get_data = getattr(loader, "get_data", None)
        if not callable(get_data):
            raise RosmasterCompatibilityError(
                "could not read installed Rosmaster source at %s" % source_path
            )
        try:
            source_bytes = get_data(source_file)
        except Exception as exc:
            raise RosmasterCompatibilityError(
                "could not read installed Rosmaster source at %s: %s"
                % (source_path, exc)
            ) from exc
        if not isinstance(source_bytes, (bytes, bytearray, memoryview)):
            raise RosmasterCompatibilityError(
                "loader returned non-bytes Rosmaster source at %s"
                % source_path
            )
    return source_path, hashlib.sha256(source_bytes).hexdigest()


def create_verified_rosmaster_transport(**kwargs: Any) -> RosmasterTransport:
    """Load only the verified V3.3.9 vendor implementation, then wrap it."""
    try:
        from Rosmaster_Lib import Rosmaster  # type: ignore
    except Exception as exc:
        raise RosmasterCompatibilityError(
            "failed to import robot-provided Rosmaster_Lib: %s" % exc
        ) from exc

    source_path, digest = rosmaster_source_sha256(Rosmaster)
    if digest != PUBLIC_V3_3_9_SHA256:
        raise RosmasterCompatibilityError(
            "unsupported Rosmaster_Lib source at %s: SHA256 %s; expected %s"
            % (source_path, digest, PUBLIC_V3_3_9_SHA256)
        )
    return RosmasterTransport(Rosmaster, **kwargs)
