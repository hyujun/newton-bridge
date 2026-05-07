"""Runtime /tf toggle: SetBool service handler logic.

The full SimBridgeNode requires rclpy + Newton at instantiation, but
`_on_set_publish_tf` is a plain method that only touches `self`-attributes
and `self.get_logger()`. We exercise it against a stand-in `self` to lock
down on/off semantics and the log-on-change behavior. This still imports
`node` though, which pulls in rclpy/sensor_msgs — skip on hosts without ROS.
"""

from __future__ import annotations

import types

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")

from newton_bridge.node import SimBridgeNode  # noqa: E402


class _FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []

    def info(self, msg: str) -> None:
        self.infos.append(msg)


def _make_fake_self(initial: bool) -> types.SimpleNamespace:
    fake = types.SimpleNamespace()
    fake._publish_tf_enabled = initial
    logger = _FakeLogger()
    fake.get_logger = lambda: logger
    fake._logger = logger  # for assertion access
    return fake


def _call(fake, data: bool):
    request = types.SimpleNamespace(data=data)
    response = types.SimpleNamespace(success=False, message="")
    SimBridgeNode._on_set_publish_tf(fake, request, response)
    return response


def test_toggle_off_then_on() -> None:
    fake = _make_fake_self(initial=True)
    r = _call(fake, False)
    assert fake._publish_tf_enabled is False
    assert r.success is True
    assert r.message == "off"

    r = _call(fake, True)
    assert fake._publish_tf_enabled is True
    assert r.message == "on"


def test_no_change_no_log() -> None:
    """Setting to the current state succeeds but does not log a transition."""
    fake = _make_fake_self(initial=True)
    r = _call(fake, True)
    assert fake._publish_tf_enabled is True
    assert r.success is True
    assert fake._logger.infos == []  # no transition log


def test_change_logs_once() -> None:
    fake = _make_fake_self(initial=True)
    _call(fake, False)
    assert any("OFF" in m for m in fake._logger.infos)
    _call(fake, True)
    assert any("ON" in m for m in fake._logger.infos)
