from __future__ import annotations

import logging
import socket
import time
from typing import Any

import websockets.exceptions
import websockets.sync.client

from openpi_client import base_policy as _base_policy
from openpi_client import msgpack_numpy


class WebsocketClientPolicy(_base_policy.BasePolicy):
    """Policy client that communicates with an OpenPI websocket server."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int | None = None,
        api_key: str | None = None,
        connect_timeout_sec: float | None = None,
        response_timeout_sec: float | None = None,
        retry_poll_sec: float = 5.0,
    ) -> None:
        if host.startswith("ws"):
            self._uri = host
        else:
            self._uri = f"ws://{host}"
        if port is not None:
            self._uri += f":{port}"

        self._api_key = api_key
        self._connect_timeout_sec = connect_timeout_sec
        self._response_timeout_sec = response_timeout_sec
        self._retry_poll_sec = retry_poll_sec
        self._packer = msgpack_numpy.Packer()
        self._ws, self._server_metadata = self._wait_for_server()

    def get_server_metadata(self) -> dict[str, Any]:
        return self._server_metadata

    def close(self) -> None:
        self._server_metadata = {}
        if hasattr(self, "_ws") and self._ws is not None:
            self._ws.close()
            self._ws = None

    def _wait_for_server(self):
        logging.info("Waiting for server at %s...", self._uri)
        start = time.monotonic()
        while True:
            try:
                headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
                conn = websockets.sync.client.connect(
                    self._uri,
                    compression=None,
                    max_size=None,
                    additional_headers=headers,
                    open_timeout=self._connect_timeout_sec,
                )
                metadata = msgpack_numpy.unpackb(conn.recv(timeout=self._response_timeout_sec))
                return conn, metadata
            except (
                TimeoutError,
                ConnectionRefusedError,
                socket.gaierror,
                OSError,
                websockets.exceptions.WebSocketException,
            ) as exc:
                if self._connect_timeout_sec is not None:
                    elapsed = time.monotonic() - start
                    if elapsed >= self._connect_timeout_sec:
                        raise TimeoutError(f"Timed out connecting to {self._uri}") from exc
                    sleep_for = min(self._retry_poll_sec, self._connect_timeout_sec - elapsed)
                else:
                    sleep_for = self._retry_poll_sec

                if self._connect_timeout_sec is not None and sleep_for <= 0:
                    raise TimeoutError(f"Timed out connecting to {self._uri}") from exc
                logging.info("Still waiting for server at %s...", self._uri)
                time.sleep(sleep_for)

    def infer(self, obs: dict[str, Any]) -> dict[str, Any]:
        data = self._packer.pack(obs)
        self._ws.send(data)
        response = self._ws.recv(timeout=self._response_timeout_sec)
        if isinstance(response, str):
            raise RuntimeError(f"Error in inference server:\n{response}")
        return msgpack_numpy.unpackb(response)

    def reset(self) -> None:
        pass
