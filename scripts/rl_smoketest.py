#!/usr/bin/env python3
"""Tiny smoketest to prove torch-cu12 extra is functional inside the container.

Not a real RL pipeline — just imports torch, confirms CUDA, runs a toy forward
pass of a policy-shaped MLP on the GPU. Newton's RL examples (e.g.
robot_anymal_c_walk) can be run via `./run.sh example robot_anymal_c_walk`.
"""

from __future__ import annotations

import sys


def main() -> int:
    try:
        import torch
    except ImportError:
        print("[rl-smoke] torch not installed — did you build with the torch-cu12 extra?", file=sys.stderr)
        return 1

    print(f"[rl-smoke] torch {torch.__version__}")
    if not torch.cuda.is_available():
        print("[rl-smoke] torch.cuda NOT available", file=sys.stderr)
        return 2
    print(f"[rl-smoke] cuda device: {torch.cuda.get_device_name(0)}")

    # tiny policy-shape MLP
    obs_dim, act_dim, hidden = 48, 12, 256
    model = torch.nn.Sequential(
        torch.nn.Linear(obs_dim, hidden),
        torch.nn.ELU(),
        torch.nn.Linear(hidden, hidden),
        torch.nn.ELU(),
        torch.nn.Linear(hidden, act_dim),
    ).cuda()
    x = torch.randn(4096, obs_dim, device="cuda")
    y = model(x)
    print(f"[rl-smoke] forward OK: in={tuple(x.shape)} -> out={tuple(y.shape)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
