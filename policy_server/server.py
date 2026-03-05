"""HTTP policy server for the robo-arm project."""

import argparse
from collections import deque
from contextlib import asynccontextmanager
import importlib.util
import inspect
import logging
from pathlib import Path
import threading
import time
from types import ModuleType
from typing import Callable

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel, Field


class PredictRequest(BaseModel):
    images_b64: list[str] = Field(min_length=1)
    joints: list[float] = Field(min_length=4, max_length=4)


class PredictResponse(BaseModel):
    action: list[float] = Field(min_length=4, max_length=4)


LOGGER = logging.getLogger("uvicorn.error")
RATE_WINDOW_SEC = 5.0
RATE_LOG_INTERVAL_SEC = 5.0

PolicyCallable = Callable[[list[str], list[float], str], list[float]]
POLICY_FN: PolicyCallable | None = None
POLICY_TASK: str | None = None


class RequestStats:
    def __init__(self):
        self._lock = threading.Lock()
        self._recent = deque()
        self._requests_total = 0

    def record_request(self):
        now = time.monotonic()
        with self._lock:
            self._requests_total += 1
            self._recent.append(now)
            self._prune_locked(now)

    def snapshot(self) -> tuple[int, int, float]:
        now = time.monotonic()
        with self._lock:
            self._prune_locked(now)
            recent_count = len(self._recent)
            total = self._requests_total
        return total, recent_count, recent_count / RATE_WINDOW_SEC

    def _prune_locked(self, now: float):
        cutoff = now - RATE_WINDOW_SEC
        while self._recent and self._recent[0] < cutoff:
            self._recent.popleft()


stats = RequestStats()
_stop_event = threading.Event()
_log_thread: threading.Thread | None = None

app = FastAPI()


def _split_policy_target(target: str) -> tuple[Path, str]:
    file_path, separator, function_name = target.rpartition(":")
    if separator == "" or not file_path or not function_name:
        raise ValueError("Policy target must be '<file:function>'")
    path = Path(file_path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"Policy file not found: {path}")
    return path.resolve(), function_name


def _load_module_from_file(path: Path) -> ModuleType:
    module_name = f"_serve_policy_{path.stem}_{abs(hash(path))}"
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load module from file: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _validate_policy_signature(function: Callable[..., object], target: str):
    params = list(inspect.signature(function).parameters.values())
    if len(params) != 3:
        raise TypeError(
            f"Policy '{target}' must have signature policy(images_b64, joints, task) -> action"
        )
    allowed_kinds = {
        inspect.Parameter.POSITIONAL_ONLY,
        inspect.Parameter.POSITIONAL_OR_KEYWORD,
    }
    if any(parameter.kind not in allowed_kinds for parameter in params):
        raise TypeError(
            f"Policy '{target}' must use positional parameters: (images_b64, joints, task)"
        )


def load_policy(target: str) -> PolicyCallable:

    path, function_name = _split_policy_target(target)
    module = _load_module_from_file(path)
    function = getattr(module, function_name, None)

    if function is None:
        raise AttributeError(f"Function '{function_name}' not found in {path}")
    if not callable(function):
        raise TypeError(f"Target '{target}' is not callable")

    _validate_policy_signature(function, target)

    return function


@app.post("/predict", response_model=PredictResponse)
async def predict(request: PredictRequest):
    if POLICY_FN is None or POLICY_TASK is None:
        raise RuntimeError("Policy server is not initialized with a policy and task")

    stats.record_request()
    images_b64 = list(request.images_b64)
    joints = [float(value) for value in request.joints]
    raw_action = POLICY_FN(images_b64, joints, POLICY_TASK)
    if not isinstance(raw_action, list):
        raise RuntimeError("Policy returned invalid action type; expected list[float]")
    action = [float(value) for value in raw_action]
    if len(action) != 4:
        raise RuntimeError("Policy returned invalid action; expected exactly 4 values")

    return PredictResponse(action=action)


def main():
    parser = argparse.ArgumentParser(description="Serve a policy file over HTTP")
    parser.add_argument(
        "policy_target",
        help="Policy target in '<file:function>' form (example: policy_pi05.py:policy)",
    )
    parser.add_argument(
        "--task",
        required=True,
        help="Task string passed to policy(images_b64, joints, task)",
    )
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8000, help="HTTP port (default: 8000)")
    args = parser.parse_args()

    global POLICY_FN, POLICY_TASK
    try:
        POLICY_FN = load_policy(args.policy_target)
    except Exception as exc:
        parser.error(str(exc))
    POLICY_TASK = args.task

    uvicorn.run(app, host=args.host, port=args.port, access_log=False)


if __name__ == "__main__":
    main()
