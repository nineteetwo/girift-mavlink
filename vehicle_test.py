# test_26_adim.py
# GİRİFT İHA TAKIMI - 26 Fonksiyonluk Birebir Test Paneli

from vehicle import MavlinkDrone
import time
import os

# --- RENKLER VE ARAYÜZ ---
class Colors:
    HEADER = '\033[95m'
    OKGREEN = '\033[92m'  # Başarılı
    WARNING = '\033[93m'  # Uyarı
    FAIL = '\033[91m'     # Hata
    ENDC = '\033[0m'      # Renk Kapat
    BOLD = '\033[1m'

def clear():
    os.system('cls' if os.name == 'nt' else 'clear')

def header():
    print(Colors.HEADER + "="*70)
    print("      GİRİFT İHA TAKIMI - 26 FONKSİYONLU MASTER TEST PANELİ")
    print("      Doküman Uyumluluğu: %100 (Tüm Maddeler Ayrı Ayrı)")
    print("="*70 + Colors.ENDC)

def pause():
    print(Colors.WARNING + "\n>> Menüye dönmek için Enter'a bas..." + Colors.ENDC)
    input()

def main():
    drone = MavlinkDrone()

    while True:
        clear()
        header()
        # Durum Çubuğu
        status = (f"{Colors.OKGREEN}BAĞLI{Colors.ENDC}"
                  if drone.is_connected else f"{Colors.FAIL}BAĞLI DEĞİL{Colors.ENDC}")
        print(f"DURUM: {status} | MOD: {drone.mode} | SAT: {drone.satellites_visible}")
        print("-" * 70)

        # --- MENÜ LİSTESİ ---
        print("1.  connect()              -> Bağlantı Kur")
        print("2.  wait_heartbeat()       -> Sinyal Bekle")
        print("3.  get_last_heartbeat()   -> Son Sinyal Verisi")
        print("4.  get_gps()              -> GPS Verisi")
        print("5.  get_attitude()         -> Tutum (Açılar)")
        print("6.  get_global_position()  -> Global Konum")
        print("7.  get_velocity()         -> Hız Bilgisi Oku")
        print("8.  get_battery()          -> Batarya Durumu")
        print("-" * 30)
        print("9.  set_flight_mode()      -> Mod Değiştir (varsayılan GUIDED)")
        print("10. get_flight_mode()      -> Modu Oku")
        print("11. goto_gps_location()    -> Konuma Git (100m kuzey, 15m)")
        print("12. set_velocity()         -> Hız Vektörü (vx=3, vy=0, vz=0)")
        print("13. set_attitude()         -> Açı Ver (roll=0.1rad, pitch=0)")
        print(f"{Colors.BOLD}14. send_ned_velocity()    -> NED Hız (vx=1, vy=0, vz=0, 3 sn){Colors.ENDC}")
        print("-" * 30)
        print("15. load_mission()         -> Görev Yükle (örnek 3 WP)")
        print("16. start_mission()        -> Görevi Başlat")
        print("17. pause_mission()        -> Görevi Duraklat")
        print("18. clear_mission()        -> Görevi Sil")
        print("19. set_rc_channel_pwm()   -> RC Override (ch=3, pwm=1500)")
        print("20. arm()                  -> Motorları ARM Et")
        print("21. disarm()               -> Motorları DISARM Et")
        print("22. rtl()                  -> Eve Dön (RTL)")
        print("23. land()                 -> İniş Yap (LAND)")
        print("24. takeoff()              -> Kalkış Yap (10m)")
        print("25. change_altitude()      -> İrtifa Değiştir (30m)")
        print("26. log_mavlink_message()  -> Log Testi / JSON Kayıt")
        print(f"{Colors.BOLD}27. VEHICLE LIVE DATA      -> CANLI Vehicle Verileri (Detaylı){Colors.ENDC}")
        print("0.  Çıkış")

        choice = input("\nSeçim: ")

        if choice == '1':
            conn = input("Bağlantı string (örn: udp:127.0.0.1:14550): ")
            if conn.strip() == "":
                conn = "udp:127.0.0.1:14550"
            ok = drone.connect(conn)
            print("✅ Bağlantı başarılı" if ok else "❌ Bağlantı başarısız")
            pause()

        elif choice == '2':
            hb = drone.wait_heartbeat()
            print("Son heartbeat:", hb)
            pause()

        elif choice == '3':
            print("Last HB:", drone.get_last_heartbeat())
            pause()

        elif choice == '4':
            print(f"GPS: {drone.get_gps()}")
            pause()

        elif choice == '5':
            print(f"Açı: {drone.get_attitude()}")
            pause()

        elif choice == '6':
            print(f"Konum: {drone.get_global_position()}")
            pause()

        elif choice == '7':
            print(f"Hız: {drone.get_velocity()}")
            pause()

        elif choice == '8':
            print(f"Pil: {drone.get_battery()}")
            pause()

        elif choice == '9':
            mode_in = input("Mod (boş bırak=GUIDED): ").upper()
            if not mode_in:
                mode_in = "GUIDED"
            drone.set_flight_mode(mode_in)
            print(f"Mod komutu gönderildi: {mode_in}")
            pause()

        elif choice == '10':
            print(f"Mod: {drone.get_flight_mode()}")
            pause()

        elif choice == '11':
            print("⚠️ Drone havada ve GUIDED modda olmalı.")
            onay = input("100m kuzeye, 15m irtifaya gidilsin mi? (e/h): ").lower()
            if onay == 'e':
                pos = drone.get_gps()
                if pos:
                    hedef_lat = pos['lat'] + 0.001  # kaba ~100m
                    hedef_lon = pos['lon']
                    hedef_alt = 15
                    print(f"Hedef: lat={hedef_lat}, lon={hedef_lon}, alt={hedef_alt}")
                    drone.goto_gps_location(hedef_lat, hedef_lon, hedef_alt)
                else:
                    print("Mevcut konum alınamadı.")
            pause()

        elif choice == '12':
            # Sabit test değeri: vx=3 m/s ileri, vy=0, vz=0
            vx, vy, vz = 3.0, 0.0, 0.0
            print(f"set_velocity() testi: vx={vx}, vy={vy}, vz={vz}")
            drone.set_velocity(vx, vy, vz)
            pause()

        elif choice == '13':
            # Hafif sağa yatış, düz uçuş
            roll = 0.1   # rad
            pitch = 0.0
            yaw = 0.0
            thrust = 0.5
            print(f"set_attitude() testi: roll={roll}, pitch={pitch}, yaw={yaw}, thrust={thrust}")
            drone.set_attitude(roll, pitch, yaw, thrust)
            pause()

        elif choice == '14':
            # 3 saniye boyunca 1 m/s ileri
            vx, vy, vz, dur = 1.0, 0.0, 0.0, 3.0
            print(f"send_ned_velocity() testi: vx={vx}, vy={vy}, vz={vz}, süre={dur}s")
            drone.send_ned_velocity(vx, vy, vz, dur)
            pause()

        elif choice == '15':
            pos = drone.get_gps()
            if not pos:
                print("GPS alınamadı, görev yüklenemedi.")
                pause()
            else:
                lat0 = pos["lat"]
                lon0 = pos["lon"]

                # ~1.6 metre adım (0.000015 derece)
                step = 0.000015

                mission = [
                    (lat0,             lon0,             10),  # Başlangıç
                    (lat0 + step,      lon0,             10),  # ~1.6 m kuzey
                    (lat0 + step,      lon0 + step,      10),  # ~1.6 m doğu
                ]

                drone.load_mission(mission)
                print("Orta kısa (yaklaşık 1.5–2 metre) görev yüklendi.")
                pause()


        elif choice == '16':
            drone.start_mission()
            pause()

        elif choice == '17':
            drone.pause_mission()
            pause()

        elif choice == '18':
            drone.clear_mission()
            pause()

        elif choice == '19':
            # Örnek: throttle kanalına (ch=3) 1500 pwm
            ch = 3
            pwm = 1500
            print(f"set_rc_channel_pwm() testi: ch={ch}, pwm={pwm}")
            drone.set_rc_channel_pwm(ch, pwm)
            pause()

        elif choice == '20':
            drone.arm()
            print("✅ ARM komutu gönderildi.")
            pause()

        elif choice == '21':
            drone.disarm()
            print("✅ DISARM komutu gönderildi.")
            pause()

        elif choice == '22':
            drone.rtl()
            print("✅ RTL komutu gönderildi.")
            pause()

        elif choice == '23':
            drone.land()
            print("✅ LAND komutu gönderildi.")
            pause()

        elif choice == '24':
            # Sabit 10 metre kalkış
            onay = input("10m kalkış komutu gönderilsin mi? (e/h): ").lower()
            if onay == 'e':
                alt = 10.0
                print(f"takeoff() testi: {alt} m")
                drone.takeoff(alt)
            pause()

        elif choice == '25':
            # Sabit 30 metre irtifaya çık
            onay = input("İrtifa 30m olacak şekilde komut gönderilsin mi? (e/h): ").lower()
            if onay == 'e':
                hedef_alt = 30.0
                print(f"change_altitude() testi: {hedef_alt} m")
                drone.change_altitude(hedef_alt)
            pause()

        elif choice == '26':
            print(f"Log Buffer: {len(drone.message_buffer)} kayıt")

            path = input("Kayıt yolu (örn: log.txt veya logs.json): ").strip()
            if not path:
                path = "log.txt"   # Boş bırakılırsa varsayılan olarak TXT

            try:
                if path.lower().endswith(".txt"):
                    # TXT olarak satır satır yaz
                    with open(path, "w", encoding="utf-8") as f:
                        for msg in drone.message_buffer:
                            f.write(repr(msg) + "\n\n\n")
                    print(f"TXT log kaydedildi: {path}")
                else:
                    # Eski davranış: JSON fonksiyonunu kullan
                    drone.export_message_buffer_to_json(path)
                    print(f"JSON log kaydedildi: {path}")
            except Exception as e:
                print(f"Hata: {e}")

            pause()



        elif choice == '27':
            # CANLI TELEMETRİ MODU – sürekli güncellenen ekran
            try:
                while True:
                    clear()
                    header()
                    snap = drone.get_telemetry_snapshot()

                    status_live = (f"{Colors.OKGREEN}BAĞLI{Colors.ENDC}"
                                   if snap["is_connected"] else f"{Colors.FAIL}BAĞLI DEĞİL{Colors.ENDC}")
                    print(f"DURUM: {status_live} | MOD: {snap['mode']} | SAT: {snap['satellites_visible']}")
                    print("-" * 70)

                    pos = snap['position']
                    vel = snap['velocity']
                    att = snap['attitude']
                    bat = snap['battery']

                    print("=== CANLI TELEMETRİ DETAY ===")
                    print(f"Bağlantı : {snap['is_connected']}  |  Conn: {snap['connection_string']}")
                    print(f"ARM/Mod  : ARM={snap['is_armed']}  |  MOD={snap['mode']}  |  SYS={snap['system_status']}")
                    print("-"*60)
                    print(f"Konum    : lat={pos['lat']:.7f}, lon={pos['lon']:.7f}, alt={pos['alt']:.2f} m")
                    print(f"Hız      : vx={vel['vx']:.2f} m/s, vy={vel['vy']:.2f} m/s, vz={vel['vz']:.2f} m/s")
                    print(f"Tutum    : roll={att['roll']:.3f}, pitch={att['pitch']:.3f}, yaw={att['yaw']:.3f}")
                    print(f"Batarya  : {bat['voltage']:.2f} V, {bat['current']:.2f} A, {bat['level']} %")
                    print(f"GPS      : sat={snap['satellites_visible']}  |  fix_type={snap['gps_fix_type']}")
                    print("-"*60)
                    print(f"Son HB   : {snap['last_heartbeat']}")
                    print(f"Mesaj buf: {snap['message_buffer_len']} kayıt  |  last_ts={snap['last_message_time']:.2f}")
                    print("\n(Çıkmak için Ctrl+C)")

                    time.sleep(0.2)   # 5 Hz güncelleme

            except KeyboardInterrupt:
                print("\nCanlı telemetri modundan çıkıldı.")
                time.sleep(1)

        elif choice == '0':
            break

if __name__ == "__main__":
    main()
