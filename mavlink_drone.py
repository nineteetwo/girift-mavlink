# mavlink_drone.py
from pymavlink import mavutil
import time
import logging
from typing import Optional, Dict
import math

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

class MavlinkDrone:
    def __init__(self):
        self.vehicle: Optional[mavutil.mavlink_connection] = None

    # 1️⃣ MAVLink bağlantısı kurar
    def connect(self, connection_string: str, baudrate: int = 57600, timeout: int = 10):
        logger.info(f"Bağlantı kuruluyor: {connection_string} @ {baudrate}")
        try:
            self.vehicle = mavutil.mavlink_connection(connection_string, baud=baudrate)
            start = time.time()
            while time.time() - start < timeout:
                msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    logger.info("Heartbeat alındı — bağlantı doğrulandı.")
                    try:
                        self.vehicle.mav.request_data_stream_send(
                            self.vehicle.target_system,
                            self.vehicle.target_component,
                            mavutil.mavlink.MAV_DATA_STREAM_ALL,
                            10, 1
                        )
                        logger.info("Data stream isteği gönderildi.")
                    except Exception as e:
                        logger.debug(f"Data stream isteği gönderilemedi: {e}")
                    return self.vehicle
            raise TimeoutError("Heartbeat alınamadı, bağlantı başarısız.")
        except Exception as e:
            logger.error(f"Bağlantı hatası: {e}")
            raise

    # 2️⃣ Heartbeat mesajını bekleyip bağlantıyı doğrular
    def wait_heartbeat(self, vehicle: Optional[mavutil.mavlink_connection] = None, timeout: int = 10) -> bool:
        if vehicle is None: vehicle = self.vehicle
        start = time.time()
        try:
            while time.time() - start < timeout:
                msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    return True
            return False
        except Exception as e:
            logger.error(f"wait_heartbeat hatası: {e}")
            return False

    # 3️⃣ Son heartbeat bilgisini döndürür (GÜNCELLENDİ)
    def get_last_heartbeat(self, vehicle: Optional[mavutil.mavlink_connection] = None) -> Optional[Dict]:
        if vehicle is None: vehicle = self.vehicle
        if vehicle is None: return None
        
        # DÜZELTME: Timeout'u 3 saniye yaptık. 
        # Drone saniyede 1 sinyal atar, 1 saniye beklemek bazen ucu ucuna kaçırabilir.
        # 3 saniye bekleyince "None" dönme ihtimali kalmaz.
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        
        if msg:
            return msg.to_dict()
        return None

    # 4️⃣ GPS verisi
    def get_gps(self, vehicle: Optional[mavutil.mavlink_connection] = None) -> Optional[Dict]:
        if vehicle is None: vehicle = self.vehicle
        if vehicle is None: return None
        msg = vehicle.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            data = msg.to_dict()
            return {
                "lat": data.get("lat", 0) / 1e7,
                "lon": data.get("lon", 0) / 1e7,
                "alt": data.get("alt", 0) / 1000,
                "satellites": data.get("satellites_visible", 0),
                "fix_type": data.get("fix_type", 0),
            }
        return None

    # 5️⃣ Tutum (Attitude)
    def get_attitude(self, vehicle: Optional[mavutil.mavlink_connection] = None) -> Optional[Dict]:
        if vehicle is None: vehicle = self.vehicle
        if vehicle is None: return None
        msg = vehicle.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            data = msg.to_dict()
            return {
                "roll": data.get("roll", 0.0),
                "pitch": data.get("pitch", 0.0),
                "yaw": data.get("yaw", 0.0),
            }
        return None

    # 6️⃣ Küresel konum
    def get_global_position(self, vehicle: Optional[mavutil.mavlink_connection] = None) -> Optional[Dict]:
        if vehicle is None: vehicle = self.vehicle
        if vehicle is None: return None
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            data = msg.to_dict()
            return {
                "lat": data.get("lat", 0) / 1e7,
                "lon": data.get("lon", 0) / 1e7,
                "alt": data.get("alt", 0) / 1000,
                "relative_alt": data.get("relative_alt", 0) / 1000,
                "vx": data.get("vx", 0) / 100,
                "vy": data.get("vy", 0) / 100,
                "vz": data.get("vz", 0) / 100,
            }
        return None

    # 7️⃣ Hız
    def get_velocity(self, vehicle: Optional[mavutil.mavlink_connection] = None) -> Optional[Dict]:
        if vehicle is None: vehicle = self.vehicle
        if vehicle is None: return None
        msg = vehicle.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            data = msg.to_dict()
            return {"airspeed": data.get("airspeed", 0), "groundspeed": data.get("groundspeed", 0), "climb": data.get("climb", 0)}
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            data = msg.to_dict()
            return {"vx": data.get("vx", 0)/100, "vy": data.get("vy", 0)/100, "vz": data.get("vz", 0)/100}
        return None

    # 8️⃣ Batarya
    def get_battery(self, vehicle: Optional[mavutil.mavlink_connection] = None) -> Optional[Dict]:
        if vehicle is None: vehicle = self.vehicle
        if vehicle is None: return None
        
        msg = vehicle.recv_match(type='BATTERY_STATUS', blocking=False)
        if msg:
            d = msg.to_dict()
            return {"source": "BATTERY_STATUS", "voltage": (d.get("voltages", [0])[0]/1000), "current": d.get("current_battery")/100.0, "level": d.get("battery_remaining")}
            
        msg = vehicle.recv_match(type='SYS_STATUS', blocking=False)
        if msg:
            d = msg.to_dict()
            return {"source": "SYS_STATUS", "voltage": d.get("voltage_battery")/1000, "current": d.get("current_battery"), "level": d.get("battery_remaining")}
            
        return None

    # 9️⃣ Uçuş modu değiştir
    def set_flight_mode(self, vehicle: Optional[mavutil.mavlink_connection], mode: str) -> bool:
        try:
            if vehicle is None: vehicle = self.vehicle
            if vehicle is None: return False
            mode = mode.strip().upper()
            mode_mapping = {
                'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4,
                'LOITER': 5, 'RTL': 6, 'CIRCLE': 7, 'LAND': 9, 'POSHOLD': 16
            }
            if mode not in mode_mapping:
                logger.error(f"Geçersiz mod: {mode}")
                return False

            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_mapping[mode], 0, 0, 0, 0, 0
            )

            start = time.time()
            while time.time() - start < 3:
                msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
                if not msg: continue
                if msg.get_srcSystem() != vehicle.target_system: continue
                if msg.custom_mode == mode_mapping[mode]:
                    logger.info(f"Mod değişti: {mode}")
                    return True
            return False
        except Exception as e:
            logger.error(f"Mod hatası: {e}")
            return False

    # 🔟 GPS konumuna git
    def goto_gps_location(self, vehicle, lat: float, lon: float, alt: float) -> bool:
        try:
            if vehicle is None: vehicle = self.vehicle
            if vehicle is None: return False
            if not self.set_flight_mode(vehicle, "GUIDED"):
                logger.warning("GUIDED moda geçilemedi")
            logger.info(f"GPS Hedef: {lat}, {lon}, {alt}")
            vehicle.mav.set_position_target_global_int_send(
                0, vehicle.target_system, vehicle.target_component,
                6, 0b110111111000, int(lat * 1e7), int(lon * 1e7), alt,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            logger.error(f"GPS git hatası: {e}")
            return False

    # 1️⃣1️⃣ Hız vektörünü ayarla
    def set_velocity(self, vehicle, vx: float, vy: float, vz: float, duration: float = 1.0) -> bool:
        try:
            if vehicle is None: vehicle = self.vehicle
            if vehicle is None: return False
            if not self.set_flight_mode(vehicle, "GUIDED"):
                logger.warning("GUIDED moda geçilemedi")
            logger.info(f"Hız ayarlanıyor: vx={vx}, vy={vy}, vz={vz}, süre={duration}s")
            start = time.time()
            while time.time() - start < duration:
                vehicle.mav.set_position_target_local_ned_send(
                    0, vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,
                    0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
                )
                time.sleep(0.1)
            vehicle.mav.set_position_target_local_ned_send(
                    0, vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111, 0,0,0, 0,0,0, 0,0,0, 0,0)
            return True
        except Exception as e:
            logger.error(f"Hız hatası: {e}")
            return False

    # 1️⃣2️⃣ Yönelim ayarla
    def set_attitude(self, vehicle, roll: float, pitch: float, yaw: float, thrust: float = 0.5, duration: float = 1.0) -> bool:
        try:
            if vehicle is None: vehicle = self.vehicle
            if vehicle is None: return False
            if not self.set_flight_mode(vehicle, "GUIDED"):
                logger.warning("GUIDED moda geçilemedi")
            r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
            cr, sr = math.cos(r/2), math.sin(r/2)
            cp, sp = math.cos(p/2), math.sin(p/2)
            cy, sy = math.cos(y/2), math.sin(y/2)
            q = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]
            type_mask = 0b00000111 
            logger.info(f"Attitude: Roll={roll}, Pitch={pitch}, Süre={duration}s")
            start = time.time()
            while time.time() - start < duration:
                vehicle.mav.set_attitude_target_send(
                    0, vehicle.target_system, vehicle.target_component,
                    type_mask, q, 0, 0, 0, thrust
                )
                time.sleep(0.1)
            return True
        except Exception as e:
            logger.error(f"Attitude hatası: {e}")
            return False

    # 1️⃣3️⃣ Mevcut Modu Oku (GÜNCELLENDİ - KUYRUK TEMİZLİĞİ EKLENDİ)
    def get_flight_mode(self, vehicle: Optional[mavutil.mavlink_connection] = None) -> str:
        if vehicle is None: vehicle = self.vehicle
        if vehicle is None: return "NO_CONNECTION"
        
        mode_mapping = {
            0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
            5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD'
        }
        
        # ADIM 1: Tamponu (Buffer) Temizle
        # Kuyrukta bekleyen eski Heartbeat'leri hızlıca okuyup çöpe atıyoruz.
        # Böylece elimize 10 saniye önceki değil, şu anki veri geçecek.
        while vehicle.recv_match(type='HEARTBEAT', blocking=False):
            pass

        # ADIM 2: Taze Veriyi Bekle
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        
        if msg and msg.get_srcSystem() == vehicle.target_system:
            return mode_mapping.get(msg.custom_mode, f"UNKNOWN({msg.custom_mode})")
        
        return "UNKNOWN"