# mavlink_drone.py
# GİRİFT İHA TAKIMI - MAVLink Kontrol Kütüphanesi (FİNAL OPTİMİZE)
# Düzeltmeler: Fazlalık mod geçişleri kalktı, Görev fonksiyonu akıllandı, Mod listesi eşitlendi.

from pymavlink import mavutil
import time
import logging
import math
import json
from typing import Optional, Dict, List, Tuple

# Log ayarları
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

class MavlinkDrone:
    def __init__(self):
        # =====================================================================
        # 1. VEHICLE SINIFI ÖZELLİKLERİ
        # =====================================================================
        self.connection_string: str = ""
        self.baudrate: int = 57600
        self.vehicle: Optional[mavutil.mavlink_connection] = None
        self.master = None
        self.is_connected: bool = False
        self.last_heartbeat: Optional[Dict] = None
        
        self.is_armed: bool = False
        self.mode: str = "UNKNOWN"
        self.system_status: str = "STANDBY"
        self.vehicle_type: str = ""
        self.firmware_version: str = "Unknown"
        
        self.position: Dict[str, float] = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
        self.velocity: Dict[str, float] = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self.attitude: Dict[str, float] = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.battery: Dict[str, float] = {"voltage": 0.0, "current": 0.0, "level": 0.0}
        
        self.mission: List[Tuple[float, float, float]] = []
        self.home_location: Dict = {"lat": None, "lon": None, "alt": None}
        self.flight_start_time: float = 0.0
        self.last_message_time: float = 0.0
        self.satellites_visible: int = 0
        self.gps_fix_type: int = 0
        self.last_command_ack: Dict = {}
        self.message_buffer: List = []

    # =========================================================================
    # 2. FONKSİYON LİSTESİ (26 ADET)
    # =========================================================================

    # --- BAĞLANTI GRUBU ---

    # 1. Connect
    def connect(self, connection_string: str, baudrate: int = 57600, timeout: int = 10):
        self.connection_string = connection_string
        self.baudrate = baudrate
        logger.info(f"Bağlantı kuruluyor: {connection_string}")
        try:
            self.vehicle = mavutil.mavlink_connection(connection_string, baud=baudrate)
            self.master = self.vehicle
            start = time.time()
            while time.time() - start < timeout:
                msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    self.is_connected = True
                    self.vehicle_type = str(msg.type)
                    self.log_mavlink_message(msg)
                    logger.info("Heartbeat alındı.")
                    try:
                        self.vehicle.mav.request_data_stream_send(
                            self.vehicle.target_system, self.vehicle.target_component,
                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
                        )
                    except: pass
                    return self.vehicle
            raise TimeoutError("Heartbeat alınamadı.")
        except Exception as e:
            logger.error(f"Hata: {e}")
            raise

    # 2. Wait Heartbeat
    def wait_heartbeat(self, timeout: int = 10) -> bool:
        if not self.vehicle: return False
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if msg:
            self.last_heartbeat = msg.to_dict()
            self.last_message_time = time.time()
            self.log_mavlink_message(msg)
            return True
        return False

    # 3. Get Last Heartbeat
    def get_last_heartbeat(self) -> Optional[Dict]:
        if not self.vehicle: return None
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            self.last_heartbeat = msg.to_dict()
            self.is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
            self.system_status = str(msg.system_status)
            self.log_mavlink_message(msg)
            return self.last_heartbeat
        return None

    # --- TELEMETRİ GRUBU ---

    # 4. Get GPS
    def get_gps(self) -> Optional[Dict]:
        if not self.vehicle: return None
        msg = self.vehicle.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            data = msg.to_dict()
            self.position["lat"] = data.get("lat", 0) / 1e7
            self.position["lon"] = data.get("lon", 0) / 1e7
            self.position["alt"] = data.get("alt", 0) / 1000
            self.satellites_visible = data.get("satellites_visible", 0)
            self.gps_fix_type = data.get("fix_type", 0)
            self.log_mavlink_message(msg)
            return self.position.copy()
        return None

    # 5. Get Attitude
    def get_attitude(self) -> Optional[Dict]:
        if not self.vehicle: return None
        msg = self.vehicle.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            self.attitude = {"roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw}
            self.log_mavlink_message(msg)
            return self.attitude.copy()
        return None

    # 6. Get Global Position
    def get_global_position(self) -> Optional[Dict]:
        if not self.vehicle: return None
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            self.log_mavlink_message(msg)
            return {
                "lat": msg.lat / 1e7, "lon": msg.lon / 1e7, 
                "alt": msg.alt / 1000.0, "relative_alt": msg.relative_alt / 1000.0
            }
        return None

    # 7. Get Velocity
    def get_velocity(self) -> Optional[Dict]:
        if not self.vehicle: return None
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            self.velocity = {"vx": msg.vx/100.0, "vy": msg.vy/100.0, "vz": msg.vz/100.0}
            self.log_mavlink_message(msg)
            return self.velocity.copy()
        return None

    # 8. Get Battery
    def get_battery(self) -> Optional[Dict]:
        if not self.vehicle: return None
        msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=False)
        if msg:
            self.battery = {
                "voltage": msg.voltage_battery / 1000.0,
                "current": msg.current_battery / 100.0,
                "level": msg.battery_remaining
            }
            self.log_mavlink_message(msg)
            return self.battery.copy()
        return None

    # --- UÇUŞ KONTROL GRUBU ---

    # 9. Set Mode (Kısıtlanmış Liste)
    def set_flight_mode(self, mode: str) -> bool:
        if not self.vehicle: return False
        
        mode_mapping = {
            'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4,
            'LOITER': 5, 'RTL': 6, 'LAND': 9, 'POSHOLD': 16
        }
        
        mode_id = mode_mapping.get(mode.upper())
        if mode_id is None: 
            logger.error(f"Geçersiz Mod: {mode}")
            return False
            
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0
        )
        return True

    # 10. Get Mode (Kısıtlanmış Liste ile Eşitlendi)
    def get_flight_mode(self) -> str:
        if not self.vehicle: return "NO_CONN"
        while self.vehicle.recv_match(type='HEARTBEAT', blocking=False): pass
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            mode_map = {
                0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
                5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD'
            }
            self.mode = mode_map.get(msg.custom_mode, f"UNKNOWN({msg.custom_mode})")
            return self.mode
        return "UNKNOWN"

    # 11. Goto GPS (Fazlalık silindi: Artık mod değiştirmiyor)
    def goto_gps_location(self, lat: float, lon: float, alt: float) -> bool:
        if not self.vehicle: return False
        self.vehicle.mav.set_position_target_global_int_send(
            0, self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b110111111000,
            int(lat*1e7), int(lon*1e7), alt, 0,0,0, 0,0,0, 0,0
        )
        return True

    # 12. Set Velocity (Fazlalık silindi)
    def set_velocity(self, vx: float, vy: float, vz: float, duration: float = 1.0) -> bool:
        return self.send_ned_velocity(vx, vy, vz, duration)

    # 13. Set Attitude (Fazlalık silindi)
    def set_attitude(self, roll: float, pitch: float, yaw: float, duration: float = 1.0) -> bool:
        if not self.vehicle: return False
        
        r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
        cr, sr, cp, sp, cy, sy = math.cos(r/2), math.sin(r/2), math.cos(p/2), math.sin(p/2), math.cos(y/2), math.sin(y/2)
        q = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]
        
        start = time.time()
        while time.time() - start < duration:
            self.vehicle.mav.set_attitude_target_send(
                0, self.vehicle.target_system, self.vehicle.target_component,
                0b00000111, q, 0, 0, 0, 0.5
            )
            time.sleep(0.1)
        return True

    # 14. Send NED Velocity (Fazlalık silindi)
    def send_ned_velocity(self, vx: float, vy: float, vz: float, duration: float = 1.0) -> bool:
        if not self.vehicle: return False
        
        start = time.time()
        while time.time() - start < duration:
            self.vehicle.mav.set_position_target_local_ned_send(
                0, self.vehicle.target_system, self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
                0,0,0, vx, vy, vz, 0,0,0, 0,0
            )
            time.sleep(0.1)
        # Fren
        self.vehicle.mav.set_position_target_local_ned_send(
             0, self.vehicle.target_system, self.vehicle.target_component,
             mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111, 0,0,0, 0,0,0, 0,0,0, 0,0)
        return True

    # --- GÖREV GRUBU ---

    # 15. Load Mission
    def load_mission(self, waypoints: List[Tuple[float, float, float]]) -> bool:
        self.mission = waypoints
        logger.info(f"{len(waypoints)} Waypoint hafızaya yüklendi.")
        return True

    # 16. Start Mission (Akıllı Versiyon: İrtifa ve Mesafe Kontrolü)
    def start_mission(self) -> bool:
        if not self.mission:
            logger.error("Hata: Görev listesi boş!")
            return False
        
        logger.info("Görev Başlatılıyor...")
        
        self.set_flight_mode("GUIDED")
        self.arm()
        time.sleep(1)
        
        target_alt = 10.0
        logger.info(f"Kalkış yapılıyor: {target_alt}m")
        self.takeoff(target_alt)
        
        # Yükselmeyi bekle
        for _ in range(20):
            pos = self.get_global_position()
            if pos and pos['relative_alt'] >= target_alt * 0.95:
                logger.info("Hedef irtifaya ulaşıldı.")
                break
            time.sleep(1)
        
        # Waypointleri gez
        for i, wp in enumerate(self.mission):
            lat, lon, alt = wp
            logger.info(f"Hedef {i+1} gidiliyor: {lat}, {lon}")
            self.goto_gps_location(lat, lon, alt)
            
            # Varmayı bekle
            reached = False
            for _ in range(30): # 30 saniye bekleme süresi
                curr_pos = self.get_gps()
                if curr_pos:
                    d_lat = abs(curr_pos['lat'] - lat)
                    d_lon = abs(curr_pos['lon'] - lon)
                    if d_lat < 0.00005 and d_lon < 0.00005:
                        logger.info(f"Hedef {i+1} tamamlandı.")
                        reached = True
                        break
                time.sleep(1)
            
            if not reached:
                logger.warning(f"Hedef {i+1} zaman aşımı.")
        
        logger.info("Görev Bitti. RTL yapılıyor.")
        self.rtl()
        return True
    
    # 17. Pause Mission
    def pause_mission(self) -> bool:
        return self.set_flight_mode("LOITER")

    # 18. Clear Mission
    def clear_mission(self) -> bool:
        self.mission = []
        if self.vehicle:
            self.vehicle.mav.mission_clear_all_send(self.vehicle.target_system, self.vehicle.target_component)
        return True

    # --- SİSTEM GRUBU ---

    # 19. RC Override
    def set_rc_channel_pwm(self, channel_id: int, pwm: int = 1500) -> bool:
        if not self.vehicle: return False
        rc_channels = [65535] * 8
        if 1 <= channel_id <= 8:
            rc_channels[channel_id - 1] = pwm
        self.vehicle.mav.rc_channels_override_send(
            self.vehicle.target_system, self.vehicle.target_component, *rc_channels
        )
        return True

    # 20. Arm
    def arm(self) -> bool:
        if not self.vehicle: return False
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )
        self.is_armed = True
        return True

    # 21. Disarm
    def disarm(self) -> bool:
        if not self.vehicle: return False
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
        self.is_armed = False
        return True

    # 22. RTL
    def rtl(self) -> bool:
        return self.set_flight_mode("RTL")

    # 23. Land
    def land(self) -> bool:
        return self.set_flight_mode("LAND")

    # 24. Takeoff
    def takeoff(self, altitude: float) -> bool:
        if not self.vehicle: return False
        self.set_flight_mode("GUIDED") 
        self.arm()
        time.sleep(1)
        self.flight_start_time = time.time()
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
        )
        return True

    # 25. Change Altitude
    def change_altitude(self, altitude: float) -> bool:
        pos = self.get_global_position()
        if pos:
            return self.goto_gps_location(pos['lat'], pos['lon'], altitude)
        return False

    # 26. Log Message
    def log_mavlink_message(self, msg) -> None:
        if not msg: return
        timestamp = time.time()
        if len(self.message_buffer) > 100:
            self.message_buffer.pop(0)
        self.message_buffer.append({"time": timestamp, "msg": msg.to_dict()})
        
        def save_log_txt(self, path: str = "mavlink_log.txt", last_only: bool = True) -> int:
            """
            message_buffer içeriğini TXT dosyasına ekler (append).
            last_only=True ise sadece son kaydı yazar; False ise tüm buffer'ı yazar.
            Dönüş: yazılan kayıt sayısı.
            """
            if not self.message_buffer:
                return 0

            records = [self.message_buffer[-1]] if last_only else self.message_buffer

            with open(path, "a", encoding="utf-8") as f:
                for rec in records:
                    ts = rec.get("time")
                    msg = rec.get("msg", {})
                    # Mesaj tipini (varsa) al
                    msg_type = None
                    if isinstance(msg, dict):
                        msg_type = msg.get("_type", msg.get("type", None))
                    # Başlık satırı
                    f.write(f"[time={ts}]")
                    if msg_type:
                        f.write(f" type={msg_type}")
                    f.write("\n")
                    # Ham mesajı yaz (sözlük veya string olabilir)
                    try:
                        f.write(json.dumps(msg, ensure_ascii=False) + "\n")
                    except Exception:
                        f.write(f"{msg}\n")
                    # Ayırıcı çizgi
                    f.write("-" * 40 + "\n")

            return len(records)

        
        