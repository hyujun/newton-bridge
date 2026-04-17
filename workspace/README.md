# workspace/

Host ↔ container shared scratch area. Bind-mounted read-write into the
container at `/workspace/workspace`.

| Dir | Purpose |
|---|---|
| `outputs/`   | Simulation artifacts (rollouts, logs, .rrd, .usd) |
| `models/`    | Trained RL policies (`.pt`, checkpoints) |
| `notebooks/` | Jupyter notebooks — `./scripts/host/run.sh jupyter` serves from here |

Container writes are UID-matched via `HOST_UID`/`HOST_GID` in
`docker/compose.yml`, so files show up host-owned (no `sudo chown` needed).
