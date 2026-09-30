# ARX5 backend

The backend is the only layer allowed to access the robot, cameras, calibration
processes, or data stores. The browser talks to the versioned API and never
imports robot-control code.

```text
arx5_backend/
  core/          process settings and lifecycle
  protocol/      versioned client/server envelopes
  transport/     FastAPI HTTP and WebSocket adapters
  features/      independently registered feature commands
```

Run the control API from the repository root:

```bash
PYTHONPATH=src .venv-handeye/bin/python -m backend.arx5_backend \
  --host 127.0.0.1 --port 8766
```

The primary control socket is `/api/v1/control`. HTTP discovery is available
at `/api/v1/system/health` and `/api/v1/system/capabilities`.
