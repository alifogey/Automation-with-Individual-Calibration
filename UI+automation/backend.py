# backend.py
import os
import sys
import json
import time
import threading
from typing import Optional, Dict, List

import serial
from serial.tools import list_ports

from PySide6.QtCore import QObject, Slot, Signal

# -------- Serial defaults --------
DEFAULT_PORT = "COM4" if sys.platform.startswith("win") else "/dev/ttyACM0"
SERIAL_PORT = os.environ.get("PUMP_SERIAL_PORT", DEFAULT_PORT)
BAUD = 115200
OPEN_RETRY_SEC = 2.0


class PumpLink:
    """
    Low-level serial link to a single Arduino running the RAMPS pump firmware.
    Responsible ONLY for sending JSON commands and reading back lines.
    """

    def __init__(self, port: str = SERIAL_PORT, baud: int = BAUD):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self._stop = False
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()

    # ---------- Port management ----------

    def open(self) -> bool:
        """Open serial port, retrying until success or stop flag."""
        if self.ser and self.ser.is_open:
            return True

        while not self._stop:
            try:
                print(f"[PumpLink] Opening {self.port} @ {self.baud}…")
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                if not self._rx_thread or not self._rx_thread.is_alive():
                    self._rx_thread = threading.Thread(
                        target=self._rx_loop, daemon=True
                    )
                    self._rx_thread.start()
                print("[PumpLink] Port open ✓")
                return True
            except Exception as e:
                print(f"[PumpLink] open failed: {e}; retrying in {OPEN_RETRY_SEC}s")
                time.sleep(OPEN_RETRY_SEC)
        return False

    def close(self):
        self._stop = True
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("[PumpLink] Port closed")
        except Exception as e:
            print(f"[PumpLink] close error: {e}")

    # ---------- TX helper ----------

    def _send(self, obj: dict):
        data = (json.dumps(obj) + "\n").encode("utf-8")
        with self._tx_lock:
            try:
                if not self.ser or not self.ser.is_open:
                    if not self.open():
                        print("[PumpLink] write skipped (port not open)")
                        return
                print(f"[PumpLink] TX: {data!r}")
                self.ser.write(data)
                self.ser.flush()
            except Exception as e:
                print(f"[PumpLink] write error: {e}")

    # ---------- Public API used by QBackend ----------

    def set_flow(self, pump: int, ul_per_min: float):
        """
        Constant flow command:
        Arduino expects: {"pump": N, "flow": F}
        """
        self._send({"pump": int(pump), "flow": float(ul_per_min)})

    def prime(self, pump: int):
        """
        Prime ON (continuous until explicit stop):
        Arduino expects: {"prime": N}
        """
        self._send({"prime": int(pump)})

    def stop(self, pump: int):
        """Stop a single pump."""
        self._send({"stop": int(pump)})

    def stop_all(self):
        """Stop all pumps."""
        self._send({"stop_all": True})

    def start_wave(
        self,
        pump: int,
        shape: str,
        period_sec: float,
        duty_fraction: float,
        min_flow_ul_min: Optional[float] = None,
        max_flow_ul_min: Optional[float] = None,
        base_flow_ul_min: Optional[float] = None,
    ):
        """
        Pulsatile wave command.

        New Arduino firmware expects (per your updated sketch):

          {
            "wave": {
              "pump":      N,
              "shape":     "Square" | "Sinusoidal" | "off",
              "period":    T_seconds,
              "duty":      D_percent,        # 0..100, only used for Square
              "min_flow":  Fmin_ul_per_min,
              "max_flow":  Fmax_ul_per_min
            }
          }

        This helper is flexible:

        - If min/max are provided, we send those.
        - If only base_flow_ul_min is provided, we treat:
            min_flow = 0
            max_flow = base_flow_ul_min
          (this keeps old “0..base” behavior working).
        """
        pump = int(pump)
        shape = str(shape)
        period_sec = float(period_sec) if period_sec > 0 else 1.0

        # Convert duty fraction (0..1) to percent (0..100) for Arduino
        duty_fraction = float(duty_fraction)
        if duty_fraction <= 0:
            duty_fraction = 0.5
        if duty_fraction >= 1:
            duty_fraction = 0.99
        duty_percent = duty_fraction * 100.0

        # Derive min/max if only base_flow was given
        if min_flow_ul_min is None and max_flow_ul_min is None and base_flow_ul_min is not None:
            min_flow_ul_min = 0.0
            max_flow_ul_min = float(base_flow_ul_min)

        # Final sanity
        if min_flow_ul_min is None:
            min_flow_ul_min = 0.0
        if max_flow_ul_min is None:
            max_flow_ul_min = min_flow_ul_min

        min_flow_ul_min = max(0.0, float(min_flow_ul_min))
        max_flow_ul_min = max(0.0, float(max_flow_ul_min))
        if max_flow_ul_min < min_flow_ul_min:
            # swap if user accidentally inverted them
            min_flow_ul_min, max_flow_ul_min = max_flow_ul_min, min_flow_ul_min

        cmd = {
            "wave": {
                "pump": pump,
                "shape": shape,
                "period": period_sec,
                "duty": duty_percent,
                "min_flow": min_flow_ul_min,
                "max_flow": max_flow_ul_min,
            }
        }
        self._send(cmd)

    def wave_off(self, pump: int, fallback_flow: float = 0.0):
        """
        Turn off pulsatile mode for a pump, optionally setting a constant flow.

        Implemented as a special case of start_wave with shape="off".
        """
        self.start_wave(
            pump=pump,
            shape="off",
            period_sec=1.0,
            duty_fraction=0.5,
            min_flow_ul_min=0.0,
            max_flow_ul_min=float(fallback_flow),
        )

    # ---------- RX loop ----------

    def _rx_loop(self):
        buf = b""
        while not self._stop:
            try:
                chunk = self.ser.read(256) if self.ser else b""
                if not chunk:
                    time.sleep(0.01)
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        txt = line.decode("utf-8", "ignore")
                        print(f"[PumpLink] RX(raw): {txt}")
                    except Exception:
                        print(f"[PumpLink] RX(bytes): {line!r}")
            except Exception as e:
                print(f"[PumpLink] rx error: {e}")
                time.sleep(0.25)


# ===========================================================
#                    QML-facing backend
# ===========================================================

class QBackend(QObject):
    """
    QObject wrapper exported to QML as 'backend'.

    - Talks to PumpLink (Arduino).
    - Remembers flows for pause/resume and pulsatile.
    """

    connectionChanged = Signal(bool)
    lastErrorChanged = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.link = PumpLink()
        # last constant flow commanded for each pump (µL/min)
        self.last_flows: Dict[int, float] = {}
        # saved flows for "pause selected" so we can resume
        self.paused_flows: Dict[int, float] = {}
        self._last_error = ""

    # ---------- Internal error helper ----------

    def _set_error(self, msg: str):
        if msg != self._last_error:
            self._last_error = msg
            print(f"[QBackend] ERROR: {msg}")
            self.lastErrorChanged.emit(msg)

    # ---------- Lifecycle ----------

    @Slot(result=bool)
    def open(self) -> bool:
        ok = self.link.open()
        self.connectionChanged.emit(ok)
        if not ok:
            self._set_error("Failed to open serial port")
        else:
            self._set_error("")
        return ok

    @Slot()
    def close(self):
        self.link.close()
        self.connectionChanged.emit(False)

    # ---------- Port discovery (optional) ----------

    @Slot()
    def refreshPorts(self):
        """
        Just prints available ports to the console.
        You can later expose this to QML if you want a dropdown.
        """
        ports = list_ports.comports()
        print("[QBackend] Available serial ports:")
        for p in ports:
            print("  ", p.device)

    # ---------- Basic pump commands ----------

    @Slot("QVariant")
    @Slot(int)
    @Slot(float)
    def prime(self, pump):
        """Prime ON: UI toggles, Arduino runs until stop()."""
        p = int(pump)
        print(f"[QBackend] prime({p})")
        self.link.prime(p)

    @Slot("QVariant")
    @Slot(int)
    @Slot(float)
    def stop(self, pump):
        """Stop a single pump and clear any wave for it."""
        p = int(pump)
        print(f"[QBackend] stop({p})")
        self.link.stop(p)
        # Also ensure pulsatile is off for that pump
        self.link.wave_off(p, 0.0)
        # don't clear last_flows; we may still want them for resume

    @Slot()
    def stopAll(self):
        """Stop all pumps and clear all waves."""
        print("[QBackend] stopAll()")
        self.link.stop_all()
        # we intentionally keep last_flows so resume could use it if needed

    # Alias for QML (some of your code uses stopAll, some stop_all)
    @Slot()
    def stop_all(self):
        self.stopAll()

    @Slot("QVariant", "QVariant")
    @Slot(int, float)
    @Slot(int, int)
    def set_flow(self, pump, ul_per_min):
        """
        Constant flow set; also remembers for pause/resume and pulsatile base.
        """
        p = int(pump)
        f = float(ul_per_min)
        print(f"[QBackend] set_flow(pump={p}, flow={f} µL/min)")
        self.last_flows[p] = f
        # If there is any previous wave on this pump, Arduino code will
        # treat a new constant command as override until another wave command.
        self.link.set_flow(p, f)

    # ---------- Pause / resume for Run tab ----------

    @Slot()
    def pauseAll(self):
        """Pause all by stopping them and saving flows."""
        print("[QBackend] pauseAll()")
        # Save current last_flows as paused_flows snapshot
        self.paused_flows = dict(self.last_flows)
        self.link.stop_all()

    # for compatibility with existing QML call `backend.pauseAll`
    @Slot()
    def pause_all(self):
        self.pauseAll()

    @Slot("QVariantList")
    def pausePumps(self, pumpIds):
        """
        Pause a subset of pumps: stop them and save their last flows
        so resumePumps can restore.
        """
        ids: List[int] = [int(p) for p in pumpIds]
        print(f"[QBackend] pausePumps({ids})")
        for p in ids:
            # Save flow if we have one
            if p in self.last_flows:
                self.paused_flows[p] = self.last_flows[p]
            self.link.stop(p)

    @Slot("QVariantList")
    def resumePumps(self, pumpIds):
        """
        Resume previously paused pumps by re-sending their last flows.
        If we don't have a saved paused flow, fall back to last_flows.
        """
        ids: List[int] = [int(p) for p in pumpIds]
        print(f"[QBackend] resumePumps({ids})")
        for p in ids:
            # prefer paused snapshot, else last known constant flow
            flow = self.paused_flows.get(p, self.last_flows.get(p, 0.0))
            if flow > 0:
                print(f"[QBackend]  -> restoring pump {p} to {flow} µL/min")
                self.link.set_flow(p, flow)
                self.last_flows[p] = flow
            else:
                print(f"[QBackend]  -> no saved flow for pump {p}, leaving off")

        # clear them out from paused_flows
        for p in ids:
            self.paused_flows.pop(p, None)

    # ---------- Automation (legacy, still supported) ----------

    @Slot("QVariantList", str, str, float, float, float)
    def startAutomation(self, pumpsVar, mode, shape, minutes, period, dutyFraction):
        """
        Original Automation entry-point. Kept for compatibility with your
        existing Main.qml.

        - pumpsVar: list of pump IDs (1..9 in UI; Arduino uses up to 5)
        - mode: "Constant" or "Pulsatile"
        - shape: "Square" or "Sinusoidal" (for Pulsatile)
        - minutes: total run time (UI uses this only for timer/labels)
        - period: period in seconds (for Pulsatile)
        - dutyFraction: for Square, UI passes 0..1 fraction here

        For "Constant": Run tab's Start button already calls set_flow()
        For "Pulsatile": we start a wave that goes from 0 .. last_flow[p]
        for each pump (so old behavior still works even without min/max).
        """
        try:
            pumps = [int(p) for p in pumpsVar]
        except Exception:
            pumps = []
        print(
            f"[QBackend] startAutomation(pumps={pumps}, mode={mode}, "
            f"shape={shape}, minutes={minutes}, period={period}, dutyFraction={dutyFraction})"
        )

        mode = str(mode)

        if mode.lower().startswith("constant"):
            # Constant runs are handled by Run tab -> set_flow + timer in QML
            print("[QBackend] Constant automation: backend will not modify flows here.")
            return

        # Only do something special for Pulsatile
        if not mode.lower().startswith("pulsatile"):
            print("[QBackend] Unknown automation mode, ignoring.")
            return

        for p in pumps:
            base = self.last_flows.get(p, 0.0)
            if base <= 0:
                print(f"[QBackend]  -> pump {p} has no base flow set; skipping wave")
                continue

            print(
                f"[QBackend]  -> starting LEGACY wave on pump {p}: "
                f"shape={shape}, period={period}s, dutyFraction={dutyFraction}, "
                f"min=0, max={base} µL/min"
            )

            # legacy behavior: 0 .. base
            self.link.start_wave(
                pump=p,
                shape=shape,
                period_sec=period,
                duty_fraction=dutyFraction,
                min_flow_ul_min=0.0,
                max_flow_ul_min=base,
            )

    # ---------- NEW: per-pump pulsatile with min/max ----------

    @Slot(int, str, float, float, float, float)
    def startWaveForPump(
        self,
        pump: int,
        shape: str,
        period_sec: float,
        dutyFraction: float,
        minFlow: float,
        maxFlow: float,
    ):
        """
        New API for AutomationPageForm/Main.qml when you want to send
        a *per-pump* pulsatile configuration including min/max flow.

        QML can call:
          backend.startWaveForPump(
              pid,
              shapeText,        // "Square" or "Sinusoidal"
              periodSeconds,
              dutyFraction,     // 0..1, only used for Square
              minFlowUlPerMin,
              maxFlowUlPerMin
          )
        """
        p = int(pump)
        shape = str(shape)
        period_sec = float(period_sec)
        dutyFraction = float(dutyFraction)
        minFlow = float(minFlow)
        maxFlow = float(maxFlow)

        print(
            f"[QBackend] startWaveForPump(pump={p}, shape={shape}, "
            f"period={period_sec}s, dutyFraction={dutyFraction}, "
            f"min={minFlow} µL/min, max={maxFlow} µL/min)"
        )

        # For completeness, remember the "max" as last flow hint
        if maxFlow > 0:
            self.last_flows[p] = maxFlow

        self.link.start_wave(
            pump=p,
            shape=shape,
            period_sec=period_sec,
            duty_fraction=dutyFraction,
            min_flow_ul_min=minFlow,
            max_flow_ul_min=maxFlow,
        )

    # ---------- Calibration stub (no-op) ----------

    @Slot("QVariant", "QVariant")
    @Slot(int, float)
    @Slot(int, int)
    def set_calibration(self, pump, ul_per_rev):
        """
        Your current Arduino sketch does not use calibration messages.
        This slot is kept as a harmless stub to avoid QML errors.
        """
        print(
            f"[QBackend] set_calibration(pump={int(pump)}, ul_per_rev={float(ul_per_rev)})  (no-op)"
        )




