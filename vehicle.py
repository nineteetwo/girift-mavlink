# mavlink_drone.py
# GİRİFT İHA TAKIMI - MAVLink Kontrol Kütüphanesi (FİNAL OPTİMİZE)

from pymavlink import mavutil
import time
import logging
import math
import json
import threading
from typing import Optional, Dict, List, Tuple
import datetime

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

        # Log sıklığı (saniye) ve son log zamanı
        self.log_interval: float = 4.0   # 1 saniyede bir log
        self.last_log_time: float = 0.0
        # Telemetri cache (her mesaj türünün son hali)
        self._cache_gps = None
        self._cache_att = None
        self._cache_global = None
        self._cache_battery = None
        self._cache_heartbeat = None



        # === OTOMATİK TELEMETRİ AYARLARI ===
        # Nesne oluşturulur oluşturulmaz arka planda canlı veri toplayan
        # bir thread başlar. Bağlantı yoksa sadece uyur, bağlantı gelince
        # HEARTBEAT / GPS / GLOBAL_POSITION_INT / ATTITUDE / SYS_STATUS
        # mesajlarını okuyup ilgili alanları günceller.
        self._telemetry_running: bool = True
        self._telemetry_hz: float = 5.0  # saniyede 5 kez
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop,
            daemon=True
        )
        self._telemetry_thread.start()

    def _telemetry_loop(self):
        """Arka planda sürekli MAVLink mesajlarını okuyup
        position / velocity / attitude / battery gibi alanları günceller.
        Nesne oluşturulur oluşturulmaz thread olarak çalışmaya başlar."""
        while self._telemetry_running:
            start = time.time()

            # Bağlantı yoksa direkt uykuya geç
            if not (self.vehicle and self.is_connected):
                time.sleep(1.0)
                continue

            try:
                # Buffer'daki bütün mesajları hızlıca çek
                while True:
                    msg = self.vehicle.recv_match(blocking=False)
                    if not msg:
                        break

                    mtype = msg.get_type()
                    
                    # Telemetri cache doldurma (log için son değerleri saklıyoruz)
                    if mtype == "GLOBAL_POSITION_INT":
                        self._cache_global = msg.to_dict()

                    elif mtype == "ATTITUDE":
                        self._cache_att = msg.to_dict()

                    elif mtype == "GPS_RAW_INT":
                        self._cache_gps = msg.to_dict()

                    elif mtype == "BATTERY_STATUS":
                        self._cache_battery = msg.to_dict()

                    elif mtype == "HEARTBEAT":
                        self._cache_heartbeat = msg.to_dict()


                    # HEARTBEAT -> is_armed, system_status
                    if mtype == "HEARTBEAT":
                        self.last_heartbeat = msg.to_dict()
                        # ARM bilgisi
                        try:
                            self.is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0
                        except Exception:
                            pass
                        # Sistem durumu
                        self.system_status = str(getattr(msg, "system_status", self.system_status))
                        # UÇUŞ MODU (Mission Planner ile aynı isim)
                        try:
                            self.mode = mavutil.mode_string_v10(msg)
                        except Exception:
                            pass

                    # GPS ham verisi
                    elif mtype == "GPS_RAW_INT":
                        # HAM GPS verisi
                        data = msg.to_dict()

                        # GPS'ten gelen lat/lon'u da güncellemek istiyorsan:
                        self.position["lat"] = data.get("lat", 0) / 1e7
                        self.position["lon"] = data.get("lon", 0) / 1e7

                        # GPS irtifasını AYRI bir alanda tut (artık alt'ı BOZMAYACAĞIZ)
                        self.position["alt_gps"] = data.get("alt", 0) / 1000.0

                        # Uydu ve fix bilgisi
                        self.satellites_visible = data.get("satellites_visible", 0)
                        self.gps_fix_type = data.get("fix_type", 0)

                    elif mtype == "GLOBAL_POSITION_INT":
                        # Konum
                        self.position["lat"] = msg.lat / 1e7
                        self.position["lon"] = msg.lon / 1e7

                        # Deniz seviyesine göre yükseklik (AMSL)
                        self.position["alt_amsl"] = msg.alt / 1000.0

                        # Home (takeoff) noktasına göre yükseklik (normalde işimize yarayan bu)
                        self.position["alt_rel"] = msg.relative_alt / 1000.0

                        # Varsayılan alt → ARTIK her zaman yerden yükseklik
                        self.position["alt"] = self.position["alt_rel"]

                        # Hız vektörü
                        self.velocity = {
                            "vx": msg.vx / 100.0,
                            "vy": msg.vy / 100.0,
                            "vz": msg.vz / 100.0,
                        }




                    # Tutum
                    elif mtype == "ATTITUDE":
                        self.attitude = {
                            "roll": msg.roll,
                            "pitch": msg.pitch,
                            "yaw": msg.yaw,
                        }

                    # Batarya
                    elif mtype == "SYS_STATUS":
                        # Batarya bilgisi bazı sistemlerde eksik olabilir
                        try:
                            voltage = getattr(msg, "voltage_battery", 0) / 1000.0
                            current = getattr(msg, "current_battery", 0) / 100.0
                            level = getattr(msg, "battery_remaining", 0)
                        except Exception:
                            voltage, current, level = 0.0, 0.0, 0.0
                        self.battery = {
                            "voltage": voltage,
                            "current": current,
                            "level": level,
                        }

                    # Log buffer'ına kaydet (mevcut helper)
                    self.log_mavlink_message(msg)

            except Exception as e:
                logger.error(f"Telemetri döngüsü hatası: {e}")

            # Hız kontrolü (saniyede ~5 kez)
            elapsed = time.time() - start
            sleep_time = max(0.0, (1.0 / self._telemetry_hz) - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_telemetry_snapshot(self) -> Dict[str, Dict]:
        """Panel için tek seferde mümkün olduğunca kapsamlı anlık telemetri verisi."""
        return {
            # Konum / hız / tutum
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
            "attitude": self.attitude.copy(),

            # Enerji
            "battery": self.battery.copy(),

            # Durum
            "is_armed": self.is_armed,
            "mode": self.mode,
            "system_status": self.system_status,
            "satellites_visible": self.satellites_visible,
            "gps_fix_type": self.gps_fix_type,

            # Bağlantı ve ham veriler
            "is_connected": self.is_connected,
            "connection_string": self.connection_string,
            "last_heartbeat": self.last_heartbeat,
            "message_buffer_len": len(self.message_buffer),
            "last_message_time": self.last_message_time,
        }

    def stop_telemetry(self):
        """Telemetri thread'ini temiz şekilde durdurur."""
        self._telemetry_running = False

    # =========================================================================
    # 2. FONKSİYON LİSTESİ
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
                    self.last_heartbeat = msg.to_dict()
                    self.is_connected = True
                    try:
                        self.mode = mavutil.mode_string_v10(msg)
                    except Exception:
                        pass
                    logger.info(f"Bağlantı kuruldu, mod: {self.mode}")
                    return True

            logger.error("ZAMAN AŞIMI: Heartbeat alınamadı.")
            self.is_connected = False
            return False
        except Exception as e:
            logger.error(f"Bağlantı hatası: {e}")
            self.is_connected = False
            return False

    # 2. wait_heartbeat
    def wait_heartbeat(self, timeout: int = 10):
        if not self.vehicle:
            logger.error("Önce connect() çağrılmalı.")
            return None
        logger.info("Heartbeat bekleniyor...")
        try:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
            if msg:
                self.last_heartbeat = msg.to_dict()
                self.is_connected = True
                try:
                    self.mode = mavutil.mode_string_v10(msg)
                except Exception:
                    pass
                logger.info(f"Heartbeat alındı, mod: {self.mode}")
                return self.last_heartbeat

            logger.error("Heartbeat alınamadı (zaman aşımı).")
            return None
        except Exception as e:
            logger.error(f"Heartbeat hatası: {e}")
            return None

    # 3. get_last_heartbeat
    def get_last_heartbeat(self):
        return self.last_heartbeat

    # --- TELEMETRİ OKUMA GRUBU ---

    # 4. get_gps
    def get_gps(self):
        if not self.vehicle:
            logger.error("GPS verisi için bağlantı yok.")
            return None
        try:
            msg = self.vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if not msg:
                logger.warning("GPS verisi alınamadı.")
                return None
            data = msg.to_dict()
            self.position["lat"] = data.get("lat", 0) / 1e7
            self.position["lon"] = data.get("lon", 0) / 1e7
            self.position["alt"] = data.get("alt", 0) / 1000
            self.satellites_visible = data.get("satellites_visible", 0)
            self.gps_fix_type = data.get("fix_type", 0)
            return {
                "lat": self.position["lat"],
                "lon": self.position["lon"],
                "alt": self.position["alt"],
                "satellites": self.satellites_visible,
                "fix_type": self.gps_fix_type
            }
        except Exception as e:
            logger.error(f"GPS hatası: {e}")
            return None

    # 5. get_attitude
    def get_attitude(self):
        if not self.vehicle:
            logger.error("Tutum verisi için bağlantı yok.")
            return None
        try:
            msg = self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=1)
            if not msg:
                logger.warning("ATTITUDE mesajı alınamadı.")
                return None
            self.attitude["roll"] = msg.roll
            self.attitude["pitch"] = msg.pitch
            self.attitude["yaw"] = msg.yaw
            return self.attitude.copy()
        except Exception as e:
            logger.error(f"Attitude hatası: {e}")
            return None

    # 6. get_global_position
    def get_global_position(self):
        if not self.vehicle:
            logger.error("Global konum için bağlantı yok.")
            return None
        try:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if not msg:
                logger.warning("GLOBAL_POSITION_INT alınamadı.")
                return None
            self.position["lat"] = msg.lat / 1e7
            self.position["lon"] = msg.lon / 1e7
            self.position["alt"] = msg.alt / 1000.0
            return self.position.copy()
        except Exception as e:
            logger.error(f"Global konum hatası: {e}")
            return None

    # 7. get_velocity
    def get_velocity(self):
        if not self.vehicle:
            logger.error("Hız verisi için bağlantı yok.")
            return None
        try:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if not msg:
                logger.warning("GLOBAL_POSITION_INT hız için alınamadı.")
                return None
            self.velocity["vx"] = msg.vx / 100.0
            self.velocity["vy"] = msg.vy / 100.0
            self.velocity["vz"] = msg.vz / 100.0
            return self.velocity.copy()
        except Exception as e:
            logger.error(f"Hız hatası: {e}")
            return None

    # 8. get_battery
    def get_battery(self):
        if not self.vehicle:
            logger.error("Batarya verisi için bağlantı yok.")
            return None
        try:
            msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
            if not msg:
                logger.warning("SYS_STATUS mesajı alınamadı.")
                return None
            self.battery["voltage"] = msg.voltage_battery / 1000.0
            self.battery["current"] = msg.current_battery / 100.0
            self.battery["level"] = msg.battery_remaining
            return self.battery.copy()
        except Exception as e:
            logger.error(f"Batarya hatası: {e}")
            return None

    # --- MOD ve KONTROL KOMUTLARI ---

    # 9. set_flight_mode
    def set_flight_mode(self, mode: str):
        if not self.vehicle:
            logger.error("Önce connect() çağrılmalı.")
            return False
        mode = mode.upper()
        try:
            if mode not in self.vehicle.mode_mapping():
                logger.error(f"Geçersiz mod: {mode}")
                return False
            mode_id = self.vehicle.mode_mapping()[mode]
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            self.mode = mode
            logger.info(f"Mod değiştirildi: {mode}")
            return True
        except Exception as e:
            logger.error(f"Mod değiştirme hatası: {e}")
            return False

    # 10. get_flight_mode
    def get_flight_mode(self):
        return self.mode

    # 11. goto_gps_location
    def goto_gps_location(self, lat: float, lon: float, alt: float):
        if not self.vehicle:
            logger.error("goto_gps_location için bağlantı yok.")
            return False
        try:
            logger.info(f"Yeni konuma gidiliyor: lat={lat}, lon={lon}, alt={alt}")
            self.set_flight_mode("GUIDED")
            self.vehicle.mav.set_position_target_global_int_send(
                0,
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b110111111000,
                int(lat * 1e7),
                int(lon * 1e7),
                alt,
                0, 0, 0,
                0, 0, 0,
                0, 0
            )
            return True
        except Exception as e:
            logger.error(f"Goto hatası: {e}")
            return False

    # 12. set_velocity (NED)
    def set_velocity(self, vx: float, vy: float, vz: float):
        if not self.vehicle:
            logger.error("Hız komutu için bağlantı yok.")
            return False
        try:
            self.vehicle.mav.set_position_target_local_ned_send(
                0,
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, 0
            )
            logger.info(f"Hız komutu gönderildi: vx={vx}, vy={vy}, vz={vz}")
            return True
        except Exception as e:
            logger.error(f"Hız komutu hatası: {e}")
            return False

    # 13. set_attitude
    def set_attitude(self, roll: float, pitch: float, yaw: float, thrust: float=0.5):
        if not self.vehicle:
            logger.error("Attitude komutu için bağlantı yok.")
            return False
        try:
            self.vehicle.mav.set_attitude_target_send(
                0,
                self.vehicle.target_system,
                self.vehicle.target_component,
                0b00000111,
                self._to_quaternion(roll, pitch, yaw),
                0, 0, 0,
                thrust
            )
            logger.info(f"Attitude komutu gönderildi: roll={roll}, pitch={pitch}, yaw={yaw}")
            return True
        except Exception as e:
            logger.error(f"Attitude komutu hatası: {e}")
            return False

    # 14. send_ned_velocity (doküman uyumlu)
    def send_ned_velocity(self, vx: float, vy: float, vz: float, duration: float = 1.0):
        if not self.vehicle:
            logger.error("send_ned_velocity için bağlantı yok.")
            return False
        try:
            end_time = time.time() + duration
            while time.time() < end_time:
                self.vehicle.mav.set_position_target_local_ned_send(
                    0,
                    self.vehicle.target_system,
                    self.vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000111,
                    0, 0, 0,
                    vx, vy, vz,
                    0, 0, 0,
                    0, 0
                )
                time.sleep(0.1)
            logger.info(f"NED velocity {duration}s boyunca uygulandı.")
            return True
        except Exception as e:
            logger.error(f"send_ned_velocity hatası: {e}")
            return False

    # 15. load_mission
    def load_mission(self, mission_items: List[Tuple[float, float, float]]):
        self.mission = mission_items
        logger.info(f"Görev yüklendi: {len(mission_items)} adet WP")
        return True

    # 16. start_mission
    def start_mission(self):
        if not self.vehicle:
            logger.error("Görev başlatmak için bağlantı yok.")
            return False
        if not self.mission:
            logger.error("Görev listesi boş.")
            return False
        try:
            logger.info("Görev başlatılıyor...")
            # Basit örnek: GUIDED modda sırayla goto_gps_location çağırma
            for (lat, lon, alt) in self.mission:
                self.goto_gps_location(lat, lon, alt)
                time.sleep(2)
            return True
        except Exception as e:
            logger.error(f"start_mission hatası: {e}")
            return False

    # 17. pause_mission (basitleştirilmiş)
    def pause_mission(self):
        logger.info("Görev duraklatıldı (sadece log; gerçek hold uygulanmadı).")
        return True

    # 18. clear_mission
    def clear_mission(self):
        self.mission.clear()
        logger.info("Görev temizlendi.")
        return True

    # 19. set_rc_channel_pwm
    def set_rc_channel_pwm(self, channel: int, pwm: int):
        if not self.vehicle:
            logger.error("RC override için bağlantı yok.")
            return False
        try:
            rc_channel_values = [65535] * 18
            rc_channel_values[channel - 1] = pwm
            self.vehicle.mav.rc_channels_override_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                *rc_channel_values
            )
            logger.info(f"RC channel {channel} override: {pwm}")
            return True
        except Exception as e:
            logger.error(f"RC override hatası: {e}")
            return False

    # 20. arm
    def arm(self):
        if not self.vehicle:
            logger.error("ARM için bağlantı yok.")
            return False
        try:
            self.vehicle.arducopter_arm()
            self.is_armed = True
            logger.info("Motorlar ARM edildi.")
            return True
        except Exception as e:
            logger.error(f"ARM hatası: {e}")
            return False

    # 21. disarm
    def disarm(self):
        if not self.vehicle:
            logger.error("DISARM için bağlantı yok.")
            return False
        try:
            self.vehicle.arducopter_disarm()
            self.is_armed = False
            logger.info("Motorlar DISARM edildi.")
            return True
        except Exception as e:
            logger.error(f"DISARM hatası: {e}")
            return False

    # 22. rtl
    def rtl(self):
        return self.set_flight_mode("RTL")

    # 23. land
    def land(self):
        return self.set_flight_mode("LAND")

    # 24. takeoff
    def takeoff(self, altitude: float):
        if not self.vehicle:
            logger.error("Kalkış için bağlantı yok.")
            return False
        try:
            self.set_flight_mode("GUIDED")
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0,
                0, 0,
                altitude
            )
            logger.info(f"{altitude}m irtifaya kalkış komutu gönderildi.")
            return True
        except Exception as e:
            logger.error(f"Takeoff hatası: {e}")
            return False

    # 25. change_altitude
    def change_altitude(self, altitude: float):
        if not self.vehicle:
            logger.error("İrtifa değiştirmek için bağlantı yok.")
            return False
        try:
            pos = self.get_gps()
            if not pos:
                logger.error("Mevcut konum alınamadı.")
                return False
            self.goto_gps_location(pos["lat"], pos["lon"], altitude)
            logger.info(f"İrtifa {altitude}m olarak değiştiriliyor.")
            return True
        except Exception as e:
            logger.error(f"İrtifa değiştirme hatası: {e}")
            return False

    # 26. log_mavlink_message
    def log_mavlink_message(self, msg):
        """Her 5 saniyede bir: tüm telemetri türlerini tek satır olarak logla."""
        try:
            now = time.time()

            # 5 saniye dolmadıysa kayıt alma
            if now - self.last_log_time < self.log_interval:
                return

            # Log satırı: tüm telemetri türleri tek pakette
            log_entry = {
                "time": datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S"),
                "gps": self._cache_gps,
                "attitude": self._cache_att,
                "global_position": self._cache_global,
                "battery": self._cache_battery,
                "heartbeat": self._cache_heartbeat,
            }

            self.message_buffer.append(log_entry)
            self.last_log_time = now

        except Exception as e:
            logger.error(f"Log hatası: {e}")




    # Yardımcı: quaternion hesaplama
    def _to_quaternion(self, roll: float, pitch: float, yaw: float):
        t0 = math.cos(yaw * 0.5)
        t1 = math.sin(yaw * 0.5)
        t2 = math.cos(roll * 0.5)
        t3 = math.sin(roll * 0.5)
        t4 = math.cos(pitch * 0.5)
        t5 = math.sin(pitch * 0.5)

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        return [w, x, y, z]

    # JSON export
    def export_message_buffer_to_json(self, filepath: str):
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(self.message_buffer, f, ensure_ascii=False, indent=2)
            logger.info(f"Mesaj buffer JSON olarak kaydedildi: {filepath}")
            return True
        except Exception as e:
            logger.error(f"JSON yazma hatası: {e}")
            return False
