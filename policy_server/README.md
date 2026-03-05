# Policy Server

Standalone Layer 3 policy service for the robo-arm project.

It exposes a plain HTTP `/predict` endpoint. The client sends:

- `images_b64`: list of base64-encoded JPEG strings (camera order is fixed by the client/policy)
- `joints`: list of 4 joint positions in fixed order `[base, shoulder, elbow, hand]`

The server responds with the next joint target.

## Run

```bash
cd policy_server
uv sync
uv run serve-policy policy_demo.py:policy --task "pick up the coffee cup" --host 0.0.0.0 --port 8000
```
