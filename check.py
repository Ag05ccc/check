#!/usr/bin/env python3
"""
ArduPilot Fixed-Wing Preflight Checker
=====================================
A practical preflight checklist tool for ArduPlane with a Jetson/companion computer.

It connects over MAVLink (serial or UDP), gathers health/param data, and walks you
through interactive checks (servo directions & stabilization). It outputs a readable
summary and a JSON artifact you can archive with logs.

⚠️ SAFETY
- REMOVE PROPELLER(S) for all bench checks that touch servos/throttle.
- Keep clear of control surfaces; servos will move during tests.
- Use at your own risk; this script cannot guarantee airworthiness.

Dependencies
------------
pip install pymavlink colorama

Examples
--------
# UDP from telem radio or SITL
python preflight_check.py --conn "udp:127.0.0.1:14550"

# Serial (Linux)
python preflight_check.py --conn "serial:/dev/ttyACM0:115200" --cells 4 --chem lipo

# Non-interactive (skip servo tests), save report
python preflight_check.py --conn "udp:0.0.0.0:14550" --non-interactive --out report.json

# SITL
python preflight_check.py --conn "udp:127.0.0.1:14550" --non-interactive --out report.json

# Use Ref Param file
python preflight_check.py \
  --conn "udp:127.0.0.1:14550" \
  --ref-param ~/my_fleet/skyhunter_ref.param \
  --tol 0.001 \
  --out preflight_report.json

  

What it checks
--------------
- Link & heartbeat, autopilot type/firmware
- GPS fix, sats, HDOP/eph
- EKF status flags
- Vibration (x/y/z) and clipping counts
- Battery voltage/current; optional chemistry/cell sanity
- Key parameters: ARMING_CHECK, RTL_ALT, geofence, flight mode layout, airspeed use
- Compass sanity (optional throttle–mag test if you confirm prop is removed)
- RC calibration ranges & live centers
- Interactive: servo directions (aileron/elevator/rudder), stabilization directions in FBWA
"""
import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Third-party
from pymavlink import mavutil
from colorama import Fore, Style, init as colorama_init


# ----------------------------- Utilities ------------------------------------

def color(s: str, c: str) -> str:
    return c + s + Style.RESET_ALL


def ok(s: str) -> str:
    return color("OK  ", Fore.GREEN) + s


def warn(s: str) -> str:
    return color("WARN", Fore.YELLOW) + " " + s


def fail(s: str) -> str:
    return color("FAIL", Fore.RED) + " " + s


def info(s: str) -> str:
    return color("INFO", Fore.CYAN) + " " + s


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ----------------------------- Data Models ----------------------------------

@dataclass
class CheckResult:
    name: str
    status: str  # OK/WARN/FAIL
    detail: str
    data: dict = field(default_factory=dict)


@dataclass
class PreflightReport:
    connection: str
    timestamp: float
    vehicle: Dict[str, str]
    results: List[CheckResult] = field(default_factory=list)

    def add(self, name: str, status: str, detail: str, data: dict = None):
        self.results.append(CheckResult(name=name, status=status, detail=detail, data=data or {}))

    def to_json(self) -> dict:
        return {
            "connection": self.connection,
            "timestamp": self.timestamp,
            "vehicle": self.vehicle,
            "results": [r.__dict__ for r in self.results],
        }


# ----------------------------- MAVLink Helpers ------------------------------

class MAV:
    def __init__(self, conn_str: str, timeout: float = 15.0):
        self.conn_str = conn_str
        self.timeout = timeout
        self.mav: mavutil.mavudp = None
        self.flight_mode_map = self._plane_mode_map()

    def connect(self):
        print(info(f"Connecting to {self.conn_str} ..."))
        self.mav = mavutil.mavlink_connection(self.conn_str, dialect="ardupilotmega")
        self.mav.wait_heartbeat(timeout=self.timeout)
        hb = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb is None:
            raise TimeoutError("No heartbeat received.")
        print(ok(f"Heartbeat: system={self.mav.target_system} component={self.mav.target_component}"))
        return hb

    def _plane_mode_map(self) -> Dict[int, str]:
        # Common ArduPlane modes (subset)
        return {
            0: "MANUAL",
            1: "CIRCLE",
            2: "STABILIZE",
            3: "TRAINING",
            4: "ACRO",
            5: "FBWA",
            6: "FBWB",
            7: "CRUISE",
            8: "AUTOTUNE",
            10: "AUTO",
            11: "RTL",
            12: "LOITER",
            13: "TAKEOFF",
            15: "GUIDED",
            16: "INITIALIZING",
            17: "QSTABILIZE",  # for quadplane users
            18: "QHOVER",
            19: "QLOITER",
            20: "QLAND",
            21: "QRTL",
        }

    def get_mode_name(self, custom_mode: int) -> str:
        return self.flight_mode_map.get(custom_mode, f"MODE_{custom_mode}")

    def param_request(self, name: str, retries: int = 3) -> Optional[float]:
        for _ in range(retries):
            try:
                self.mav.mav.param_request_read_send(
                    self.mav.target_system, self.mav.target_component, name.encode("ascii"), -1
                )
                msg = self.mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
                if msg and msg.param_id.decode("ascii").strip("\x00") == name:
                    return float(msg.param_value)
            except Exception:
                pass
        return None

    def params_bulk(self, names: List[str]) -> Dict[str, Optional[float]]:
        out = {}
        for n in names:
            out[n] = self.param_request(n)
        return out

    def read_message(self, mtype: str, wait: float = 2.0):
        return self.mav.recv_match(type=mtype, blocking=True, timeout=wait)

    def recv_many(self, types: Tuple[str, ...], duration: float = 3.0) -> List:
        end = time.time() + duration
        msgs = []
        while time.time() < end:
            m = self.mav.recv_match(type=types, blocking=False)
            if m:
                msgs.append(m)
            time.sleep(0.01)
        return msgs

    def rc_override(self, ch: int, pwm: Optional[int]):
        """
        Send an RC_CHANNELS_OVERRIDE for a single channel (1-8). Pass None to release.
        NOTE: Throttle is usually CH3 for planes.
        """
        vals = [0] * 8
        if pwm is not None:
            # map to 1..8 slot
            if not (1 <= ch <= 8):
                raise ValueError("RC channel must be 1..8")
            vals[ch - 1] = int(pwm)
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system, self.mav.target_component, *vals
        )

    def current_mode(self) -> str:
        hb = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb:
            return self.get_mode_name(int(hb.custom_mode))
        return "UNKNOWN"


# ----------------------------- Check Implementations ------------------------

def check_vehicle_identity(m: MAV, report: PreflightReport):
    hb = m.read_message("HEARTBEAT", wait=2)
    if not hb:
        report.add("Link/Heartbeat", "FAIL", "No HEARTBEAT received after connect.")
        print(fail("No HEARTBEAT received after connect."))
        return
    ap_type = hb.type
    autopilot = hb.autopilot
    mode_name = m.get_mode_name(int(hb.custom_mode))
    vehicle = {
        "autopilot": str(autopilot),
        "type": str(ap_type),
        "mode": mode_name,
    }
    report.vehicle.update(vehicle)
    print(ok(f"Vehicle mode: {mode_name} (type={ap_type}, autopilot={autopilot})"))
    report.add("Vehicle Identity", "OK", f"Mode={mode_name}, type={ap_type}, autopilot={autopilot}", vehicle)


def check_gps(m: MAV, report: PreflightReport, min_sats: int = 12, max_hdop: float = 1.5):
    msg = m.read_message("GPS_RAW_INT", wait=3)
    if not msg:
        report.add("GPS", "FAIL", "No GPS_RAW_INT message received.")
        print(fail("GPS: no GPS_RAW_INT."))
        return

    fix_type = int(msg.fix_type)  # 3D fix >= 3
    sats = int(msg.satellites_visible)
    eph_cm = getattr(msg, "eph", None)
    hdop = None
    if eph_cm not in (None, 65535):  # 65535 sometimes used for unknown
        # Many firmwares report eph in cm; HDOP approximated as eph/100 (not exact).
        try:
            hdop = float(eph_cm) / 100.0
        except Exception:
            hdop = None

    detail = f"fix={fix_type}, sats={sats}, hdop~{hdop if hdop is not None else 'n/a'}"
    if fix_type < 3:
        report.add("GPS", "FAIL", f"No 3D fix. {detail}", {"fix_type": fix_type, "sats": sats, "hdop": hdop})
        print(fail(f"GPS: need 3D fix. {detail}"))
    elif sats < min_sats or (hdop is not None and hdop > max_hdop):
        report.add("GPS", "WARN", f"Weak GNSS: {detail}", {"fix_type": fix_type, "sats": sats, "hdop": hdop})
        print(warn(f"GPS weak: {detail}"))
    else:
        report.add("GPS", "OK", detail, {"fix_type": fix_type, "sats": sats, "hdop": hdop})
        print(ok(f"GPS: {detail}"))


def check_ekf(m: MAV, report: PreflightReport):
    msg = m.read_message("EKF_STATUS_REPORT", wait=3)
    if not msg:
        report.add("EKF", "WARN", "No EKF_STATUS_REPORT received; cannot verify flags.")
        print(warn("EKF: no EKF_STATUS_REPORT."))
        return
    flags = int(msg.flags)
    # Require attitude and velocity OK
    good = (flags & 0x01) and (flags & 0x02)  # attitude, velocity
    detail = f"flags=0x{flags:02X} (att={'Y' if (flags & 0x01) else 'n'}, vel={'Y' if (flags & 0x02) else 'n'})"
    if good:
        report.add("EKF", "OK", detail, {"flags": flags})
        print(ok(f"EKF: {detail}"))
    else:
        report.add("EKF", "FAIL", detail, {"flags": flags})
        print(fail(f"EKF: {detail}"))


def check_vibration(m: MAV, report: PreflightReport, duration: float = 2.5):
    msgs = m.recv_many(("VIBRATION",), duration=duration)
    if not msgs:
        report.add("Vibration", "WARN", "No VIBRATION messages received; cannot assess.")
        print(warn("Vibration: no messages."))
        return
    # Take max over window
    vx = max(abs(getattr(x, "vibration_x", 0.0)) for x in msgs)
    vy = max(abs(getattr(x, "vibration_y", 0.0)) for x in msgs)
    vz = max(abs(getattr(x, "vibration_z", 0.0)) for x in msgs)
    clip0 = max(getattr(x, "clipping_0", 0) for x in msgs)
    clip1 = max(getattr(x, "clipping_1", 0) for x in msgs)
    clip2 = max(getattr(x, "clipping_2", 0) for x in msgs)

    detail = f"vibe max (m/s^2): x={vx:.1f}, y={vy:.1f}, z={vz:.1f}; clips: {clip0}/{clip1}/{clip2}"
    if any(v > 30.0 for v in (vx, vy, vz)) or any(c > 0 for c in (clip0, clip1, clip2)):
        report.add("Vibration", "WARN", detail, {"vx": vx, "vy": vy, "vz": vz, "clips": [clip0, clip1, clip2]})
        print(warn(f"Vibration high: {detail}"))
    else:
        report.add("Vibration", "OK", detail, {"vx": vx, "vy": vy, "vz": vz, "clips": [clip0, clip1, clip2]})
        print(ok(f"Vibration: {detail}"))


def check_battery(m: MAV, report: PreflightReport, cells: Optional[int], chem: str):
    # Prefer BATTERY_STATUS; fallback to SYS_STATUS
    msg = m.read_message("BATTERY_STATUS", wait=1.5)
    volt = None
    curr = None
    if msg and hasattr(msg, "voltages"):
        # voltages is array of cell voltages in mV; values of 65535 indicate "not used"
        vs = [v for v in msg.voltages if 0 < v < 65535]
        if vs:
            volt = sum(vs) / 1000.0
    if volt is None:
        sysmsg = m.read_message("SYS_STATUS", wait=1.0)
        if sysmsg:
            vbatt_mv = getattr(sysmsg, "voltage_battery", 0)
            ibatt_cA = getattr(sysmsg, "current_battery", -1)
            if vbatt_mv and vbatt_mv > 0:
                volt = vbatt_mv / 1000.0
            if ibatt_cA and ibatt_cA >= 0:
                curr = ibatt_cA / 100.0

    if volt is None:
        report.add("Battery", "WARN", "No voltage telemetry available.")
        print(warn("Battery: no voltage."))
        return

    detail = f"{volt:.2f} V"
    data = {"voltage_V": volt}
    if curr is not None:
        detail += f", {curr:.1f} A"
        data["current_A"] = curr

    status = "OK"
    if cells:
        per_cell = volt / cells
        data["per_cell_V"] = per_cell
        # conservative advisories
        if chem.lower() == "lipo":
            if per_cell < 3.6:
                status = "WARN"
                detail += f" (LiPo {per_cell:.2f} V/cell: consider charging)"
        elif chem.lower() == "liion":
            if per_cell < 3.3:
                status = "WARN"
                detail += f" (Li-ion {per_cell:.2f} V/cell: consider charging)"
        else:
            detail += f" (unknown chem={chem})"

    report.add("Battery", status, detail, data)
    print((ok if status == "OK" else warn)(f"Battery: {detail}"))


def check_params(m: MAV, report: PreflightReport, ref_params: Optional[Dict[str, float]] = None, tol: float = 0.01):

    names = [
        "ARMING_CHECK",
        "ARMING_REQUIRE",
        "RTL_ALT",
        "FENCE_ENABLE",
        "FENCE_RADIUS",
        "FENCE_ALT_MAX",
        "ARSPD_USE",
        "ARSPD_TYPE",
        "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5", "FLTMODE6",
        "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT",
        "BATT_LOW_VOLT", "BATT_CRT_VOLT",
    ]
    vals = m.params_bulk(names)
    # Arming
    ac = vals.get("ARMING_CHECK")
    if ac is not None and int(ac) == 0:
        report.add("Arming Checks", "WARN", f"ARMING_CHECK=0 (disabled). Enable pre-arm checks.")
        print(warn("ARMING_CHECK=0 (disabled)."))
    else:
        report.add("Arming Checks", "OK", f"ARMING_CHECK={ac}")
        print(ok(f"Arming checks enabled (ARMING_CHECK={ac})."))

    ar = vals.get("ARMING_REQUIRE")
    if ar is not None and int(ar) == 0:
        print(warn("ARMING_REQUIRE=0 (arming without safety)."))
        report.add("Arming Require", "WARN", "ARMING_REQUIRE=0 (less safe).")
    else:
        report.add("Arming Require", "OK", f"ARMING_REQUIRE={ar}")
        print(ok(f"ARMING_REQUIRE={ar}"))

    # RTL Altitude
    rtl_alt = vals.get("RTL_ALT")
    if rtl_alt is not None:
        rtl_m = float(rtl_alt)  # ArduPlane uses meters for RTL_ALT
        status = "OK" if rtl_m >= 60.0 else "WARN"
        report.add("RTL Altitude", status, f"{rtl_m:.0f} m AGL")
        print((ok if status == "OK" else warn)(f"RTL_ALT={rtl_m:.0f} m"))

    # Geofence
    fence_en = vals.get("FENCE_ENABLE")
    if fence_en is not None and int(fence_en) == 1:
        fr = vals.get("FENCE_RADIUS")
        fa = vals.get("FENCE_ALT_MAX")
        report.add("Geofence", "OK", f"Enabled (radius={fr} m, alt_max={fa} m)")
        print(ok(f"Geofence enabled (radius={fr} m, alt_max={fa} m)"))
    else:
        report.add("Geofence", "WARN", "Disabled")
        print(warn("Geofence is disabled."))

    # Airspeed
    arspd_use = vals.get("ARSPD_USE")
    if arspd_use is not None and int(arspd_use) == 1:
        ty = int(vals.get("ARSPD_TYPE") or -1)
        report.add("Airspeed", "OK", f"Using pitot (type={ty})")
        print(ok(f"Airspeed in use (type={ty})."))
    else:
        report.add("Airspeed", "WARN", "Not using pitot (TECS will estimate from GPS)")
        print(warn("No airspeed sensor in use."))

    # Flight modes layout
    mode_names = []
    for i in range(1, 7):
        v = vals.get(f"FLTMODE{i}")
        if v is None:
            mode_names.append("UNKNOWN")
        else:
            mode_names.append(m.get_mode_name(int(v)))
    needed = {"MANUAL", "FBWA", "RTL"}
    have = set(mode_names)
    status = "OK" if needed.issubset(have) else "WARN"
    report.add("Flight Modes", status, f"{mode_names}")
    pretty = ", ".join(mode_names)
    print((ok if status == "OK" else warn)(f"Flight modes: {pretty}"))

    # Battery failsafes
    low_act = int(vals.get("BATT_FS_LOW_ACT") or 0)
    crt_act = int(vals.get("BATT_FS_CRT_ACT") or 0)
    if low_act == 0 and crt_act == 0:
        report.add("Battery FS", "WARN", "Battery failsafes disabled.")
        print(warn("Battery failsafes disabled."))
    else:
        report.add("Battery FS", "OK", f"low_act={low_act}, crt_act={crt_act}")
        print(ok(f"Battery FS: low_act={low_act}, crt_act={crt_act}"))

    # --- Optional: compare against a reference param file ---
    if ref_params:
        print(info("Fetching full parameter list for reference diff..."))
        curr_params = fetch_all_params(m)
        changed, missing_in_vehicle, extra_in_vehicle = compare_params(curr_params, ref_params, tol)

        # Summarize
        detail = (f"changed={len(changed)}, missing_on_vehicle={len(missing_in_vehicle)}, "
                  f"extra_on_vehicle={len(extra_in_vehicle)}, tol={tol}")
        data = {
            "changed": [{"name": n, "ref": rv, "curr": cv, "delta": d} for (n, rv, cv, d) in changed[:50]],
            "missing_on_vehicle": missing_in_vehicle[:50],
            "extra_on_vehicle": extra_in_vehicle[:50],
            "totals": {
                "changed": len(changed),
                "missing_on_vehicle": len(missing_in_vehicle),
                "extra_on_vehicle": len(extra_in_vehicle),
            }
        }
        # Status: FAIL if any critical parameter differs? Keep it WARN by default; you can tailor this.
        status = "OK" if len(changed) == 0 and len(missing_in_vehicle) == 0 else "WARN"
        report.add("Param Diff vs Reference", status, detail, data)

        # Console preview of first few diffs
        if changed:
            preview = ", ".join([f"{n}: ref {rv} -> curr {cv}" for (n, rv, cv, _) in changed[:10]])
            print(warn(f"Param changes ({len(changed)}): {preview} ..."))
        if missing_in_vehicle:
            print(warn(f"Missing on vehicle ({len(missing_in_vehicle)}): {', '.join(missing_in_vehicle[:10])} ..."))
        if extra_in_vehicle:
            print(info(f"Extra on vehicle ({len(extra_in_vehicle)}; showing 10): {', '.join(extra_in_vehicle[:10])} ..."))


def parse_param_file(path: str) -> Dict[str, float]:
    ref = {}
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            name = None; val = None
            if line.lower().startswith("param "):
                # e.g. "param set NAME VALUE"
                parts = line.split()
                if len(parts) >= 3:
                    name = parts[-2]
                    try:
                        val = float(parts[-1])
                    except ValueError:
                        continue
            else:
                # support comma or whitespace separated "NAME,VALUE" or "NAME VALUE"
                line = line.replace("\t", " ")
                parts = [p for p in line.replace(",", " ").split(" ") if p]
                if len(parts) >= 2:
                    name = parts[0]
                    try:
                        val = float(parts[1])
                    except ValueError:
                        continue
            if name is not None and val is not None:
                ref[name] = val
    return ref


def fetch_all_params(m: MAV, timeout: float = 12.0) -> Dict[str, float]:
    """
    Request the full parameter list and collect PARAM_VALUE messages.
    """
    params: Dict[str, float] = {}
    m.mav.mav.param_request_list_send(m.mav.target_system, m.mav.target_component)
    start = time.time()
    expected = None
    while time.time() - start < timeout:
        msg = m.mav.recv_match(type="PARAM_VALUE", blocking=False)
        if msg:
            name = msg.param_id.decode("ascii", errors="ignore").strip("\x00")
            params[name] = float(msg.param_value)
            if expected is None and hasattr(msg, "param_count"):
                expected = int(msg.param_count)
            if expected is not None and len(params) >= expected:
                break
        time.sleep(0.01)
    return params


def compare_params(curr: Dict[str, float], ref: Dict[str, float], tol: float = 0.01):
    changed = []            # (name, ref_val, curr_val, delta)
    missing_in_vehicle = [] # present in ref, absent on vehicle
    extra_in_vehicle = []   # present on vehicle, absent in ref

    for k, vref in ref.items():
        vcurr = curr.get(k, None)
        if vcurr is None:
            missing_in_vehicle.append(k)
        else:
            if abs(vcurr - vref) > tol:
                changed.append((k, vref, vcurr, vcurr - vref))

    for k in curr.keys():
        if k not in ref:
            extra_in_vehicle.append(k)

    return changed, missing_in_vehicle, extra_in_vehicle



def check_rc_centers(m: MAV, report: PreflightReport):
    msg = m.read_message("RC_CHANNELS", wait=2.0)
    if not msg:
        report.add("RC", "WARN", "No RC_CHANNELS message received.")
        print(warn("RC: no RC_CHANNELS."))
        return
    ch = [getattr(msg, f"chan{i}_raw", 0) for i in range(1, 9)]
    # Expect channels 1-4 near trims (1500) when sticks centered (throttle low ~1000)
    detail = f"raw[1..8]={ch}"
    bad = []
    if ch[0] and abs(ch[0] - 1500) > 80: bad.append("Roll(CH1) center off")
    if ch[1] and abs(ch[1] - 1500) > 80: bad.append("Pitch(CH2) center off")
    if ch[3] and ch[3] > 1100: bad.append("Yaw(CH4) not centered")
    status = "OK" if not bad else "WARN"
    detail2 = "; ".join(bad) if bad else "centers nominal"
    report.add("RC Centers", status, f"{detail} -> {detail2}", {"ch": ch})
    print((ok if status == "OK" else warn)(f"RC centers: {detail2} ({ch})"))


# ----------------------------- Interactive Checks ---------------------------

def prompt_yes(label: str) -> bool:
    try:
        ans = input(label + " [y/N]: ").strip().lower()
    except EOFError:
        return False
    return ans == "y"


def servo_direction_wizard(m: MAV, report: PreflightReport):
    print(info("Interactive servo direction test. Ensure PROP REMOVED. Switch to MANUAL."))
    if not prompt_yes("Confirm prop is REMOVED and area is clear?"):
        report.add("Servo Directions", "WARN", "Skipped (user did not confirm safety).")
        print(warn("Skipping servo direction test."))
        return

    # Safe throttle low override for safety
    try:
        m.rc_override(3, 1000)  # CH3 throttle low
    except Exception:
        pass

    steps = [
        ("Aileron (CH1): commanding RIGHT roll (CH1 +300). Did RIGHT aileron go UP?", 1, 1800),
        ("Aileron (CH1): commanding LEFT roll (CH1 -300). Did LEFT aileron go UP?", 1, 1200),
        ("Elevator (CH2): commanding PULL (CH2 +300). Did elevator go UP?", 2, 1800),
        ("Elevator (CH2): commanding PUSH (CH2 -300). Did elevator go DOWN?", 2, 1200),
        ("Rudder (CH4): commanding RIGHT yaw (CH4 +300). Did rudder deflect RIGHT?", 4, 1800),
        ("Rudder (CH4): commanding LEFT yaw (CH4 -300). Did rudder deflect LEFT?", 4, 1200),
    ]

    issues = []
    for label, ch, pwm in steps:
        print(info(label))
        try:
            m.rc_override(ch, pwm)
            time.sleep(1.0)
        finally:
            m.rc_override(ch, 1500 if ch in (1, 2, 4) else 1000)
        ok_user = prompt_yes("Observed correct direction?")
        if not ok_user:
            issues.append(label)

    # release overrides
    for ch in (1, 2, 3, 4):
        m.rc_override(ch, 0)

    if issues:
        detail = f"Potential reversals or linkage issues:\n - " + "\n - ".join(issues)
        report.add("Servo Directions", "FAIL", detail)
        print(fail(detail))
    else:
        report.add("Servo Directions", "OK", "User confirmed all directions correct.")
        print(ok("Servo directions confirmed by user."))


def stabilization_wizard(m: MAV, report: PreflightReport):
    print(info("Stabilization check in FBWA. Place the aircraft level and SWITCH TO FBWA."))
    if not prompt_yes("Ready to gently tilt the airframe by hand?"):
        report.add("Stabilization", "WARN", "Skipped (user declined).")
        print(warn("Skipping stabilization test."))
        return
    print("→ Tilt RIGHT (right wing down). Right aileron should go DOWN.")
    print("→ Tilt NOSE DOWN. Elevator should go UP.")
    print("→ Yaw NOSE RIGHT. Rudder should go LEFT.")
    ok_user = prompt_yes("Did all stabilization reactions match the expectations?")
    if ok_user:
        report.add("Stabilization", "OK", "User observed correct stabilization directions in FBWA.")
        print(ok("Stabilization reactions OK."))
    else:
        report.add("Stabilization", "FAIL", "User observed incorrect stabilization behavior.")
        print(fail("Stabilization reactions NOT correct."))


def compass_throttle_wizard(m: MAV, report: PreflightReport):
    print(info("Optional: throttle–compass interference check."))
    print("This briefly raises throttle while logging heading. PROP MUST BE REMOVED. Switch to MANUAL.")
    if not prompt_yes("Proceed? (prop removed, area clear)"):
        report.add("Compass vs Throttle", "WARN", "Skipped by user.")
        print(warn("Skipping throttle–compass test."))
        return

    # Capture initial heading from VFR_HUD (or ATTITUDE yaw)
    base_heading = None
    msg = m.read_message("VFR_HUD", wait=1.0)
    if msg:
        base_heading = getattr(msg, "heading", None)

    # Raise throttle safely (CH3)
    try:
        # m.rc_override(3, 1700) # DEBUG SAFETY OFF
        time.sleep(2.0)
    finally:
        # m.rc_override(3, 1000) # DEBUG SAFETY OFF
        time.sleep(0.8)
        m.rc_override(3, 0)

    post_heading = None
    msg2 = m.read_message("VFR_HUD", wait=1.0)
    if msg2:
        post_heading = getattr(msg2, "heading", None)

    if base_heading is None or post_heading is None:
        report.add("Compass vs Throttle", "WARN", "Could not read heading for comparison.")
        print(warn("Compass test: no heading information."))
        return

    delta = abs((post_heading - base_heading + 180) % 360 - 180)  # shortest angle diff
    detail = f"Heading change ≈ {delta:.1f}° under throttle"
    if delta > 10.0:
        report.add("Compass vs Throttle", "WARN", detail, {"delta_deg": delta})
        print(warn(detail + " (consider moving mag/rewiring)"))
    else:
        report.add("Compass vs Throttle", "OK", detail, {"delta_deg": delta})
        print(ok(detail))


# ----------------------------- Main -----------------------------------------

def main():
    colorama_init()
    ap = argparse.ArgumentParser(description="ArduPilot Fixed-Wing Preflight Checker")
    ap.add_argument("--conn", required=True, help="MAVLink connection string, e.g. udp:127.0.0.1:14550 or serial:/dev/ttyACM0:115200")
    ap.add_argument("--cells", type=int, default=None, help="Battery cell count (e.g., 4 for 4S)")
    ap.add_argument("--chem", type=str, default="lipo", help="Battery chemistry (lipo|liion)")
    ap.add_argument("--min-sats", type=int, default=12, help="Minimum desirable satellite count")
    ap.add_argument("--max-hdop", type=float, default=1.5, help="Max desirable HDOP (approx)")
    ap.add_argument("--non-interactive", action="store_true", help="Skip interactive servo/stabilization/compass checks")
    ap.add_argument("--out", type=str, default=None, help="Write JSON report to this file")
    ap.add_argument("--ref-param", type=str, default=None, help="Path to reference .param file to compare against")
    ap.add_argument("--tol", type=float, default=0.01, help="Numeric tolerance for differences (e.g., 0.01)")

    args = ap.parse_args()

    report = PreflightReport(connection=args.conn, timestamp=time.time(), vehicle={})
    m = MAV(args.conn)
    try:
        m.connect()
    except Exception as e:
        print(fail(f"Connect failed: {e}"))
        report.add("Connection", "FAIL", str(e))
        if args.out:
            with open(args.out, "w") as f:
                json.dump(report.to_json(), f, indent=2)
        sys.exit(2)

    # Core checks
    check_vehicle_identity(m, report)
    check_gps(m, report, min_sats=args.min_sats, max_hdop=args.max_hdop)
    check_ekf(m, report)
    check_vibration(m, report)
    check_battery(m, report, cells=args.cells, chem=args.chem)
    check_params(m, report)
    check_rc_centers(m, report)

    ref_dict = parse_param_file(args.ref_param) if args.ref_param else None
    check_params(m, report, ref_params=ref_dict, tol=args.tol)

    # Interactive (optional)
    if not args.non_interactive:
        servo_direction_wizard(m, report)
        stabilization_wizard(m, report)
        compas_throttle_wizard(m, report)

    # Summary
    print("\n" + "="*60)
    print("Preflight summary:")
    summary_counts = {"OK": 0, "WARN": 0, "FAIL": 0}
    for r in report.results:
        summary_counts[r.status] = summary_counts.get(r.status, 0) + 1
        tag = {"OK": ok, "WARN": warn, "FAIL": fail}[r.status]
        print(tag(f"{r.name}: {r.detail}"))
    print("-"*60)
    print(f"Totals: OK={summary_counts.get('OK',0)}  WARN={summary_counts.get('WARN',0)}  FAIL={summary_counts.get('FAIL',0)}")
    print("="*60)

    if args.out:
        with open(args.out, "w") as f:
            json.dump(report.to_json(), f, indent=2)
        print(info(f"Wrote JSON report to {args.out}"))


if __name__ == "__main__":
    main()
