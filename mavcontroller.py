# mavtools_sdk.py
from pymavlink import mavutil
import time
from typing import List, Dict

ARDUPILOT_COPTER_MODES = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "CIRCLE": 7,
    "LAND": 9,
    "DRIFT": 11,
    "SPORT": 13,
    "FLIP": 14,
    "AUTOTUNE": 15,
    "POSHOLD": 16,
    "BRAKE": 17,
}

class MavController:
    def __init__(self, endpoint: str = "", baud: int = 57600, src_sysid: int = 255):
        self.endpoint = endpoint
        self.baud = baud
        self.src_sysid = src_sysid
        self.master = None
        self.connect()

    # Connection
    def connect(self):
        if self.endpoint:
            self.master = self._connect(self.endpoint, self.baud, self.src_sysid)
        else:
            self.master = self._auto_connect(self.baud, self.src_sysid)

    def _connect(self, endpoint: str, baud: int, src_sysid: int):
        print(f"[INFO] Connecting: {endpoint}")
        if endpoint.startswith("/dev/"):
            master = mavutil.mavlink_connection(endpoint, baud=baud, source_system=src_sysid)
        else:
            master = mavutil.mavlink_connection(endpoint, source_system=src_sysid)
        master.wait_heartbeat(timeout=5)
        if not master.target_system:
            master.target_system = 1
        if not master.target_component:
            master.target_component = 1
        print(f"[OK] Connected: sys={master.target_system} comp={master.target_component}")
        return master

    def _auto_connect(self, baud: int, src_sysid: int):
        import glob
        candidates = ["udp:127.0.0.1:14550", "udp:127.0.0.1:14540"]
        candidates += sorted(glob.glob("/dev/ttyUSB*"))
        candidates += sorted(glob.glob("/dev/ttyACM*"))
        for ep in candidates:
            try:
                print(f"[INFO] Auto-connect trying: {ep}")
                m = self._connect(ep, baud, src_sysid)
                print(f"[OK] Auto-connect success: {ep}")
                return m
            except Exception as e:
                print(f"[WARN] Auto-connect failed: {ep} → {e}")
                continue
        raise RuntimeError("Auto-connect failed.")

    # Mode Change
    def set_mode(self, mode_name: str):
        mode = mode_name.upper()
        if mode not in ARDUPILOT_COPTER_MODES:
            raise RuntimeError(f"Unknown mode: {mode}")
        custom_mode = ARDUPILOT_COPTER_MODES[mode]
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            base_mode,
            custom_mode,
            0, 0, 0, 0, 0,
        )
        time.sleep(1)
        print(f"[INFO] Mode change sent: {mode}")

    # Arm / Disarm
    def arm(self):
        print("[RUN] ARM")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
        print("[OK] ARM command sent.")

    def disarm(self):
        print("[RUN] DISARM")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
        print("[OK] DISARM command sent.")

    # TAKEOFF
    def takeoff(self, altitude: float):
        print(f"[RUN] TAKEOFF → {altitude}m")
        self.set_mode("GUIDED")
        self.arm()
        time.sleep(2)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, float(altitude)
        )
        while True:
            pos = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if pos and (pos.relative_alt / 1000) >= altitude * 0.95:
                break
        print("[OK] TAKEOFF completed.")

    # GUIDED Waypoint uçuşu
    def goto_waypoints_guided(self, waypoints: List[Dict]):
        print("[RUN] GUIDED waypoint flight")
        self.set_mode("GUIDED")
        for wp in waypoints:
            lat = float(wp["lat"])
            lon = float(wp["lon"])
            alt = float(wp["alt"])

            print(f"[INFO] Moving to waypoint: {lat}, {lon}, {alt}")

            reached = False
            while not reached:
                # Hedef pozisyonu gönder
                self.master.mav.set_position_target_global_int_send(
                    0,
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b0000111111111000,
                    int(lat*1e7),
                    int(lon*1e7),
                    alt,
                    0,0,0,0,0,0,0,0
                )

                # Mevcut pozisyonu oku
                pos = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if pos:
                    dlat = abs(pos.lat/1e7 - lat)
                    dlon = abs(pos.lon/1e7 - lon)
                    dalt = abs(pos.relative_alt/1000 - alt)
                    if dlat < 0.00001 and dlon < 0.00001 and dalt < 0.5:
                        reached = True
                        print(f"[OK] Waypoint reached: {lat}, {lon}, {alt}")
                time.sleep(0.5)

    # LAND
    def land(self):
        print("[RUN] LAND")
        self.set_mode("LAND")
        time.sleep(1)
        print("[OK] LAND sent.")

    # RTL
    def rtl(self):
        print("[RUN] RTL")
        self.set_mode("RTL")
        time.sleep(1)
        print("[OK] RTL sent.")

# ------------------------------------------------------------
# Main / test block
# ------------------------------------------------------------
if __name__ == "__main__":
    try:
        mc = MavController()
        mc.takeoff(altitude=10)

        waypoints = [
            {"lat": -35.3633222, "lon": 149.1660118, "alt": 10},
            {"lat": -35.3639609, "lon": 149.1654861, "alt": 15},
        ]

        mc.goto_waypoints_guided(waypoints)
        print("[INFO] Waypoints flight completed…")

        # RTL ile geri dön
        mc.rtl()
        time.sleep(10)

        mc.land()
        time.sleep(5)
        mc.disarm()
        print("[OK] Mission completed.")

    except Exception as e:
        print(f"[ERROR] {e}")
