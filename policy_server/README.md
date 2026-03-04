# Policy Server

Standalone Layer 3 policy service for the robo-arm project.

It exposes a plain HTTP `/predict` endpoint. The Pi-side `policy_client` sends:

- the latest camera frame as base64-encoded JPEG
- the latest joint state

The server responds with the next joint target.

## Run

```bash
cd policy_server
uv sync
uv run policy-server --host 0.0.0.0 --port 8000
```

## Current behavior

The first implementation is intentionally safe and simple: it echoes the current
joint positions back as the action, so the end-to-end policy loop can be tested
without introducing motion from the server itself.
