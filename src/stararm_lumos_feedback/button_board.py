"""Local button-board selection; verification is never persisted or inferred.

This setting overrides only the next connection's button presence choice. It
does not edit the reviewed session, calibration, profile or binding, and it
never accesses hardware.
"""

import json
import os
import tempfile
import threading
from pathlib import Path


class ButtonBoard:
    def __init__(self, profile, path=None):
        teleop = profile.session.get("teleop", {})
        enabled = teleop.get("button_enabled", True)
        button_id = teleop.get("button_id", 7)
        self._check_enabled(enabled)
        if type(button_id) is not int or not 7 <= button_id <= 253:
            raise ValueError("Button board ID must be an integer in 7..253")
        self.path = None if path is None else Path(path).expanduser().resolve()
        if self.path is not None and self.path.exists():
            value = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(value, dict) or set(value) != {"enabled"}:
                raise ValueError("Button board settings must contain only enabled")
            self._check_enabled(value["enabled"])
            enabled = value["enabled"]
        self._enabled, self.id, self._verified = enabled, button_id, None
        self._lock = threading.RLock()

    @staticmethod
    def _check_enabled(enabled):
        if type(enabled) is not bool:
            raise ValueError("Button board enabled must be a boolean")

    @property
    def enabled(self):
        with self._lock:
            return self._enabled

    @property
    def verified(self):
        with self._lock:
            return self._verified

    def get(self):
        with self._lock:
            return dict(enabled=self._enabled, id=self.id, verified=self._verified)

    def set(self, enabled):
        self._check_enabled(enabled)
        with self._lock:
            if self.path is not None:
                self._save(enabled)
            # Commit only after successful persistence; a failed save must also
            # retain the previous connection's verification result.
            self._enabled, self._verified = enabled, None
            return self.get()

    def _save(self, enabled):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=self.path.parent,
                prefix=self.path.name + ".", suffix=".tmp", delete=False,
            ) as stream:
                temporary = Path(stream.name)
                os.fchmod(stream.fileno(), 0o600)
                json.dump(dict(enabled=enabled), stream, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def record(self, verified):
        if verified is not None and type(verified) is not bool:
            raise ValueError("Button board verification must be a boolean or None")
        with self._lock:
            self._verified = verified
            return self.get()
