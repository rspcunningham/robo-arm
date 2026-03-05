"""HTTP policy server for the robo-arm project."""

import argparse
from collections import deque
from contextlib import asynccontextmanager
import logging
import threading
import time

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel, Field

from replay_policy import policy


class JointStatePayload(BaseModel):
    name: list[str] = Field(default_factory=list)
    position: list[float] = Field(default_factory=list)
    effort: list[float] = Field(default_factory=list)


class PredictRequest(BaseModel):
    image_jpeg_b64: str
    joint_state: JointStatePayload


class PredictResponse(BaseModel):
    action: list[float]


LOGGER = logging.getLogger("uvicorn.error")
RATE_WINDOW_SEC = 5.0
RATE_LOG_INTERVAL_SEC = 5.0


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


def _rate_logger():
    while not _stop_event.wait(RATE_LOG_INTERVAL_SEC):
        total, recent_count, rate_hz = stats.snapshot()
        LOGGER.info(
            "predict rate %.1f Hz over last %.0fs (%d requests, total %d)",
            rate_hz,
            RATE_WINDOW_SEC,
            recent_count,
            total,
        )


@asynccontextmanager
async def lifespan(_app: FastAPI):
    global _log_thread

    _stop_event.clear()
    _log_thread = threading.Thread(target=_rate_logger, daemon=True)
    _log_thread.start()
    try:
        yield
    finally:
        _stop_event.set()
        if _log_thread is not None:
            _log_thread.join(timeout=1.0)
            _log_thread = None


app = FastAPI(lifespan=lifespan)


@app.post("/predict", response_model=PredictResponse)
async def predict(request: PredictRequest):
    stats.record_request()
    positions = request.joint_state.position
    torques = request.joint_state.effort

    new_action = policy(positions, torques)

    return PredictResponse(action=new_action)


def main():
    parser = argparse.ArgumentParser(description="Run the robo-arm policy server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8000, help="HTTP port (default: 8000)")
    args = parser.parse_args()

    uvicorn.run('server:app', host=args.host, port=args.port, access_log=False, reload=True)


if __name__ == "__main__":
    main()
