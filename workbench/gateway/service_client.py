"""Async NDJSON client for the existing workbench service."""
from __future__ import annotations

import asyncio
import json
import time


class ServiceClient:
    def __init__(self, socket_path, event_callback=None):
        self.socket_path = str(socket_path)
        self.event_callback = event_callback or (lambda row: None)
        self.reader = None
        self.writer = None
        self.pending = {}
        self.sequence = 0
        self.connected = False
        self.last_error = ""
        self._connect_lock = asyncio.Lock()
        self._reader_task = None

    async def connect(self):
        if self.writer and not self.writer.is_closing():
            return
        async with self._connect_lock:
            if self.writer and not self.writer.is_closing():
                return
            self.reader, self.writer = await asyncio.open_unix_connection(self.socket_path)
            self.connected = True
            self.last_error = ""
            self._reader_task = asyncio.create_task(self._read_loop())

    async def _read_loop(self):
        try:
            while True:
                line = await self.reader.readline()
                if not line:
                    break
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if "event" in row:
                    self.event_callback(row)
                    continue
                request_id = str(row.get("id", ""))
                future = self.pending.pop(request_id, None)
                if future and not future.done():
                    future.set_result(row)
        except Exception as exc:
            self.last_error = str(exc)
        finally:
            self.connected = False
            error = RuntimeError("工作台后台连接已断开")
            for future in self.pending.values():
                if not future.done():
                    future.set_exception(error)
                    future.exception()
            self.pending.clear()

    async def call(self, op, args=None, timeout=4.0):
        await self.connect()
        self.sequence += 1
        request_id = f"gateway-{self.sequence}"
        future = asyncio.get_running_loop().create_future()
        self.pending[request_id] = future
        payload = {"id": request_id, "op": op, "args": args or {}}
        self.writer.write((json.dumps(payload, ensure_ascii=False) + "\n").encode())
        await self.writer.drain()
        try:
            row = await asyncio.wait_for(future, timeout)
        except Exception:
            self.pending.pop(request_id, None)
            raise
        if not row.get("ok"):
            error = row.get("error") or {}
            raise RuntimeError(error.get("message", str(error) or "工作台命令失败"))
        return row.get("result")

    async def close(self):
        if self.writer:
            self.writer.close()
            try:
                await self.writer.wait_closed()
            except Exception:
                pass

