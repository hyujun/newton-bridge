"""Sensor wiring (Phase 5).

Maps a pack's `sensors:` block to Newton sensor instances and ROS publishers.
Only standard ROS msg types are used (decision E) — no custom newton_bridge_msgs
package.

Pack schema:

    sensors:
      contact:
        - label: ee_contact
          bodies: ["*wrist_3_link*"]   # fnmatch globs matched against body_label
          # shapes: [...]               # alternative; either bodies or shapes
          measure_total: true            # aggregate (default true)
          topic: /contact_wrenches/ee
          frame_id: wrist_3_link         # msg.header.frame_id
      imu:
        - label: base_imu
          site: base_site                # MJCF site label. URDF packs must
          # sites: [base_site, tcp_site] # add sites manually via builder.
          topic: /imu/base
          frame_id: base_link

Each contact sensor publishes `geometry_msgs/WrenchStamped` with `force` taken
from SensorContact.total_force (aggregated over matched bodies). Torque stays
zero — Newton 1.1.0's SensorContact total_force is a vec3, not a wrench.

Each IMU sensor publishes `sensor_msgs/Imu` with linear_acceleration and
angular_velocity from SensorIMU; orientation field is zero with covariance
set to -1 to signal "not provided" per the Imu.msg spec.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


def _contact_targets(entry: dict) -> dict[str, Any]:
    """yaml `bodies`/`shapes` -> Newton kwargs `sensing_obj_bodies`/`sensing_obj_shapes`."""
    bodies = entry.get("bodies")
    shapes = entry.get("shapes")
    if bodies is not None and shapes is not None:
        raise ValueError(f"sensor {entry.get('label')!r}: specify bodies OR shapes, not both")
    if bodies is not None:
        return {"sensing_obj_bodies": list(bodies) if isinstance(bodies, (list, tuple)) else bodies}
    if shapes is not None:
        return {"sensing_obj_shapes": list(shapes) if isinstance(shapes, (list, tuple)) else shapes}
    raise ValueError(f"sensor {entry.get('label')!r}: must specify bodies or shapes")


def _imu_targets(entry: dict) -> list:
    site = entry.get("site")
    sites = entry.get("sites")
    if site is None and sites is None:
        raise ValueError(f"sensor {entry.get('label')!r}: must specify site or sites")
    return list(sites) if sites is not None else [site]


@dataclass
class ContactSensorSpec:
    label: str
    topic: str
    frame_id: str
    sensor: Any = None  # newton.sensors.SensorContact


@dataclass
class IMUSensorSpec:
    label: str
    topic: str
    frame_id: str
    sensor: Any = None
    prev_linear_velocity: np.ndarray | None = None   # for finite-diff accel
    dt: float = 0.0


@dataclass
class SensorBundle:
    contact: list[ContactSensorSpec] = field(default_factory=list)
    imu: list[IMUSensorSpec] = field(default_factory=list)

    def empty(self) -> bool:
        return not self.contact and not self.imu


def build_sensors(pack: dict, model) -> SensorBundle:
    """Construct Newton sensors described in pack['sensors']. Empty -> empty bundle."""
    spec = pack.get("sensors") or {}
    bundle = SensorBundle()

    from newton.sensors import SensorContact, SensorIMU  # lazy: Newton-only

    for entry in spec.get("contact", []) or []:
        tgt = _contact_targets(entry)
        sensor = SensorContact(
            model,
            measure_total=bool(entry.get("measure_total", True)),
            verbose=False,
            **tgt,
        )
        bundle.contact.append(
            ContactSensorSpec(
                label=str(entry["label"]),
                topic=str(entry.get("topic", f"/contact_wrenches/{entry['label']}")),
                frame_id=str(entry.get("frame_id", "world")),
                sensor=sensor,
            )
        )

    for entry in spec.get("imu", []) or []:
        sensor = SensorIMU(model, sites=_imu_targets(entry), verbose=False)
        bundle.imu.append(
            IMUSensorSpec(
                label=str(entry["label"]),
                topic=str(entry.get("topic", f"/imu/{entry['label']}")),
                frame_id=str(entry.get("frame_id", "world")),
                sensor=sensor,
            )
        )

    return bundle


def contact_force_vec3(spec: ContactSensorSpec) -> tuple[float, float, float]:
    """Sum per-body contact forces to a single 3-vector for WrenchStamped.force."""
    arr = spec.sensor.total_force.numpy()  # shape (N, 3) for N matched bodies
    total = np.asarray(arr).reshape(-1, 3).sum(axis=0)
    return float(total[0]), float(total[1]), float(total[2])


def imu_readings(spec: IMUSensorSpec) -> dict:
    """Extract linear_acceleration + angular_velocity from SensorIMU.

    SensorIMU in Newton 1.1.0 exposes different attribute names across dev
    builds; this helper looks up the current ones and falls back gracefully.
    """
    sensor = spec.sensor
    lin_acc = _first_attr(sensor, ["linear_acceleration", "accel", "acceleration"])
    ang_vel = _first_attr(sensor, ["angular_velocity", "gyro", "ang_vel"])
    out = {"linear_acceleration": (0.0, 0.0, 0.0), "angular_velocity": (0.0, 0.0, 0.0)}
    if lin_acc is not None:
        a = lin_acc.numpy().reshape(-1, 3)[0]
        out["linear_acceleration"] = (float(a[0]), float(a[1]), float(a[2]))
    if ang_vel is not None:
        a = ang_vel.numpy().reshape(-1, 3)[0]
        out["angular_velocity"] = (float(a[0]), float(a[1]), float(a[2]))
    return out


def _first_attr(obj, names):
    for n in names:
        v = getattr(obj, n, None)
        if v is not None and hasattr(v, "numpy"):
            return v
    return None
