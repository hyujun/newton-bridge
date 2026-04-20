#!/usr/bin/env bash
# In-container smoke test. Mirrors Newton guide §7 plus a load-every-pack sweep.
set -u

PASS=0
FAIL=0

banner() { printf '\n\033[1;34m== %s ==\033[0m\n' "$*"; }
ok()     { printf '\033[1;32m[PASS]\033[0m %s\n' "$*"; PASS=$((PASS+1)); }
ko()     { printf '\033[1;31m[FAIL]\033[0m %s\n' "$*"; FAIL=$((FAIL+1)); }
run()    { local name="$1"; shift; if "$@"; then ok "${name}"; else ko "${name}"; fi; }

banner "1. Warp CUDA init"
run "warp.init + cuda devices" python3 -c "
import warp as wp; wp.init()
devs = wp.get_cuda_devices()
assert devs, 'no CUDA devices visible'
print(devs)
"

banner "2. Newton version"
run "newton module import" python3 -c "
import newton
print('newton', getattr(newton, '__version__', '<unknown>'))
"

banner "3. newton.examples registry"
# Newton 1.1.0 made the CLI take <example_name> positionally, so `-m newton.examples --help`
# exits non-zero. Import the package and walk submodules instead — same registry, no CLI coupling.
run "newton.examples discoverable" python3 -c "
import pkgutil
from newton import examples
mods = [m.name for m in pkgutil.walk_packages(examples.__path__, examples.__name__ + '.')]
assert mods, 'no newton example modules discovered'
print(f'newton.examples: {len(mods)} modules discoverable')
"

banner "4. quick step on basic_pendulum"
# Run a short headless step to confirm the sim path works end-to-end.
# --viewer null avoids GL dependency in CI-like runs; example names vary
# between Newton releases so we just import + construct World manually.
run "newton.ModelBuilder finalize + 10 steps" python3 - <<'PY'
import warp as wp, newton
wp.init()
b = newton.ModelBuilder()
body = b.add_body(xform=wp.transform((0,0,1), wp.quat_identity()))
joint = b.add_joint_revolute(parent=-1, child=body, axis=(0,1,0))
b.add_articulation([joint])
b.add_ground_plane()
m = b.finalize()
s0, s1 = m.state(), m.state()
c = m.control()
solver = newton.solvers.SolverXPBD(m)
for _ in range(10):
    contacts = m.collide(s0)
    s0.clear_forces()
    solver.step(s0, s1, c, contacts, 1/240.)
    s0, s1 = s1, s0
print('stepped 10x OK')
PY

banner "5. rclpy import"
run "rclpy import (ROS 2 Jazzy)" python3 -c "
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
print('rclpy', rclpy.__version__ if hasattr(rclpy,'__version__') else 'ok')
"

banner "6. load every robot pack + drive responds to target"
for pack in /workspace/robots/*/; do
    name="$(basename "${pack%/}")"
    run "pack ${name} parses + finalizes + drive moves" python3 - <<PY
import os
os.environ['ROBOT_PACK'] = "${pack%/}"
# Exercise load path and confirm PD drive actually moves a joint when a
# target is commanded. Regression guard for the pre-1.1.0 joint_target /
# joint_act confusion that left the arm frozen under /joint_command.
import warp as wp; wp.init()
from pathlib import Path
from newton_bridge.robot_pack import load_pack
from newton_bridge.world import NewtonWorld
pack = load_pack(Path("${pack%/}"))
world = NewtonWorld(pack)

# ArticulationView integrity: pack's joint_names is a subset (possibly proper)
# of the view's DOFs — extras exist for packs with unexposed fingers etc.
exposed = list(pack["joint_names"])
view_set = set(world.joint_dof_names)
missing = [n for n in exposed if n not in view_set]
assert not missing, f"pack joints missing from view: {missing}"
assert world.exposed_joint_names == exposed
assert world.total_dof >= len(exposed)

for _ in range(5):
    world.step()

first_joint = world.joint_dof_names[0]
q0 = world.read_joint_positions()[first_joint]
world.set_joint_targets([first_joint], [q0 + 0.3])
for _ in range(200):  # ~0.5s sim time at 400Hz
    world.step()
q1 = world.read_joint_positions()[first_joint]
dq = abs(q1 - q0)
assert dq > 0.05, f"{first_joint} did not respond: Δq={dq:.4f} rad (target offset 0.3)"

# reset() must restore home_pose q and zero velocity.
world.reset()
q_after_reset = world.read_joint_positions()
for name, expected in (pack.get("home_pose") or {}).items():
    got = q_after_reset[name]
    assert abs(got - expected) < 1e-4, f"reset: {name} q={got:.4f} expected {expected:.4f}"

# Phase 4: rich readback APIs must match the pack's exposed joints / bodies.
qd_map = world.read_joint_velocities()
eff_map = world.read_joint_efforts()
assert set(qd_map) == set(pack["joint_names"]), qd_map
assert set(eff_map) == set(pack["joint_names"]), eff_map
tf_map = world.read_body_transforms()
assert len(tf_map) == len(world.view.body_names), (len(tf_map), len(world.view.body_names))
for name, (pos, quat) in tf_map.items():
    assert len(pos) == 3 and len(quat) == 4, (name, pos, quat)
    assert all(v == v for v in pos + quat), f"NaN in tf[{name}]"  # NaN check

print(f"ok: dof={world.total_dof}, joints={len(world.joint_dof_names)}, "
      f"bodies={len(tf_map)}, Δq({first_joint})={dq:.4f} rad, t={world.sim_time:.4f}")
PY
done

banner "7. set_joint_targets wires all 3 channels (pos/vel/effort) into control"
# Solver-level semantics vary per solver (XPBD ignores target channels for
# manually-built joints, MuJoCo's topological_sort rejects the auto-joint
# pattern add_body creates, Featherstone NaNs without mass). The reliable
# check is API-wiring: NewtonWorld.set_joint_targets writes into
# control.joint_target_pos / joint_target_vel / joint_f arrays.
run "set_joint_targets writes all channels" python3 - <<'PY'
import os, numpy as np
os.environ['ROBOT_PACK'] = '/workspace/robots/ur5e'
import warp as wp; wp.init()
from pathlib import Path
from newton_bridge.robot_pack import load_pack
from newton_bridge.world import NewtonWorld

pack = load_pack(Path(os.environ['ROBOT_PACK']))
world = NewtonWorld(pack)
j0 = world.exposed_joint_names[0]

# POSITION
world.set_joint_targets([j0], positions=[0.42])
pos = world.control.joint_target_pos.numpy()
i = world._dof_index[j0]
assert abs(pos[i] - 0.42) < 1e-5, f"target_pos[{i}]={pos[i]}"

# VELOCITY
world.set_joint_targets([j0], velocities=[0.17])
vel = world.control.joint_target_vel.numpy()
assert abs(vel[i] - 0.17) < 1e-5, f"target_vel[{i}]={vel[i]}"

# EFFORT
world.set_joint_targets([j0], efforts=[3.5])
eff = world.control.joint_f.numpy()
assert abs(eff[i] - 3.5) < 1e-5, f"joint_f[{i}]={eff[i]}"

# None-channel leaves previous values untouched.
world.set_joint_targets([j0], positions=[0.99])  # efforts not set
eff2 = world.control.joint_f.numpy()
assert abs(eff2[i] - 3.5) < 1e-5, f"effort cleared unexpectedly: {eff2[i]}"
print(f"ok: all 3 channels wired, dof={world.total_dof}")
PY

banner "8. per-joint drive override resolves correctly"
run "per-joint drive override" python3 - <<'PY'
from newton_bridge.world import parse_drive_mode
import newton

# top-level + override merge
pack = {
    "drive": {"mode": "position", "stiffness": 500.0, "damping": 50.0},
    "joints": {
        "shoulder_pan_joint": {
            "drive": {"mode": "velocity", "damping": 5.0},
            "effort_limit": 120.0,
        },
    },
}

class _DummyWorld:
    pack = pack
    _resolve_joint_drive = __import__("newton_bridge.world", fromlist=["NewtonWorld"]).NewtonWorld._resolve_joint_drive

d = _DummyWorld()
a = d._resolve_joint_drive("shoulder_pan_joint")
assert a["mode"] == "velocity", a
assert a["damping"] == 5.0, a
assert a["stiffness"] == 500.0, a   # unspecified => inherits

b = d._resolve_joint_drive("shoulder_lift_joint")
assert b == {"mode": "position", "stiffness": 500.0, "damping": 50.0}, b

# parse_drive_mode sanity
assert int(parse_drive_mode("position")) == int(newton.JointTargetMode.POSITION)
assert int(parse_drive_mode("VELOCITY")) == int(newton.JointTargetMode.VELOCITY)
try:
    parse_drive_mode("linear"); raise SystemExit("expected ValueError")
except ValueError:
    pass
print("ok: drive override merge + mode parsing")
PY

banner "9. SensorContact builds + publishes without NaN"
run "SensorContact wired through scene.sensors" python3 - <<'PY'
import os, numpy as np
os.environ['ROBOT_PACK'] = '/workspace/robots/ur5e'
import warp as wp; wp.init()
from pathlib import Path
from newton_bridge.robot_pack import load_pack
from newton_bridge.world import NewtonWorld
from newton_bridge.sensors import build_sensors, contact_force_vec3

pack = load_pack(Path(os.environ['ROBOT_PACK']))
# Inject a sensors block at runtime (simulates a scene.yaml with Phase 5
# sensors). Body label includes articulation prefix, so use a glob.
pack["sensors"] = {
    "contact": [
        {"label": "base", "bodies": ["*base_link*"],
         "topic": "/contact_wrenches/base", "frame_id": "base_link",
         "measure_total": True},
    ],
}
world = NewtonWorld(pack)
bundle = build_sensors(pack, world.model)
assert len(bundle.contact) == 1 and bundle.contact[0].label == "base"

# Step + update sensor
for _ in range(20):
    world.step()
bundle.contact[0].sensor.update(world.state_0, world.last_contacts)
fx, fy, fz = contact_force_vec3(bundle.contact[0])
# UR5e at rest on ground: some non-NaN force, roughly gravity-opposing.
assert np.isfinite(fx) and np.isfinite(fy) and np.isfinite(fz), (fx, fy, fz)
print(f"ok: contact force=({fx:+.2f}, {fy:+.2f}, {fz:+.2f}) N")
PY

banner "10. VIEWER=null dispatches + end_frame runs"
run "viewer factory null mode" bash -c "
VIEWER=null python3 - <<'PY'
import os
os.environ['VIEWER'] = 'null'
os.environ['ROBOT_PACK'] = '/workspace/robots/ur5e'
import warp as wp; wp.init()
from pathlib import Path
from newton_bridge.robot_pack import load_pack
from newton_bridge.world import NewtonWorld
from newton_bridge.viewer import build_viewer, resolve_mode
assert resolve_mode() == 'null'
world = NewtonWorld(load_pack(Path(os.environ['ROBOT_PACK'])))
v = build_viewer(world)
assert v is not None
v.begin_frame(0.0)
v.log_state(world.state_0)
v.end_frame()
v.close()
print('ok: null viewer end-to-end')
PY
"

banner "Summary"
printf 'passed: %d   failed: %d\n' "${PASS}" "${FAIL}"
exit "${FAIL}"
