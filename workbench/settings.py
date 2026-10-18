import copy
import json
import os
import pathlib
import threading


def atomic_json(path, data):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    os.replace(temp, path)


class JsonStore:
    def __init__(self, path, default):
        self.path = pathlib.Path(path)
        self.lock = threading.RLock()
        self.value = copy.deepcopy(default)
        self.error = ''
        self._mtime = None
        self._reload()
        if self.error:
            previous = self.path.with_suffix(self.path.suffix + '.previous')
            try:
                data = json.loads(previous.read_text())
                self.validate(data)
                self.value = data
            except (OSError, ValueError, KeyError, TypeError):
                pass

    def validate(self, data):
        if not isinstance(data, dict):
            raise ValueError('配置顶层必须是 JSON 对象')

    def _reload(self):
        if not self.path.exists():
            return
        stamp = self.path.stat().st_mtime_ns
        self._mtime = stamp
        try:
            data = json.loads(self.path.read_text())
            self.validate(data)
            self.value = data
            self.error = ''
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.error = f'{self.path}: {exc}；保留上一个有效快照'

    def get(self):
        with self.lock:
            self._reload()
            return copy.deepcopy(self.value)

    def save(self, data):
        with self.lock:
            self.validate(data)
            if self.path.exists() and not self.error:
                self.path.with_suffix(self.path.suffix + '.previous').write_bytes(self.path.read_bytes())
            atomic_json(self.path, data)
            self.value = copy.deepcopy(data)
            self._mtime = self.path.stat().st_mtime_ns
            self.error = ''
        return self.get()
