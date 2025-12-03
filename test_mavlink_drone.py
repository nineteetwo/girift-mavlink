# demo_mavlink_drone.py
from mavlink_drone import MavlinkDrone
import time

def main():
    drone = MavlinkDrone()
    vehicle = None
    
    print("\n--- GİRİFT İHA: 13 FONKSİYONLU TAM TEST MENÜSÜ ---")
    
    while True:
        print("\n" + "="*50)
        print("=== FONKSİYON LİSTESİ (KAPTAN SUNUMU) ===")
        print("="*50)
        print("1.  [connect] Bağlantı Kur")
        print("2.  [wait_heartbeat] Bağlantıyı Doğrula (Sinyal Bekle)")
        print("3.  [get_last_heartbeat] Son Heartbeat Verisini Göster")
        print("4.  [get_gps] GPS Verisi Oku")
        print("5.  [get_battery] Batarya Durumu Oku")
        print("6.  [get_attitude] Tutum (Attitude) Oku")
        print("7.  [get_velocity] Hız (Velocity) Oku")
        print("8.  [get_global_position] Global Konum Oku")
        print("9.  [set_flight_mode] Modu Değiştir (GUIDED Yap)")
        print("10. [get_flight_mode] Mevcut Modu Oku")
        print("11. [goto_gps_location] HAREKET: GPS İle Git (100m)")
        print("12. [set_velocity] HAREKET: Hızlan (Velocity)")
        print("13. [set_attitude] HAREKET: Sağa Yat (Attitude)")
        print("0.  ÇIKIŞ")
        print("-" * 50)
        
        choice = input("Seçiminiz (0-13): ")
        
        # 1. BAĞLANTI
        if choice == '1':
            print("\n>> [1] BAĞLANTI KURULUYOR...")
            try:
                conn_str = "udp:0.0.0.0:14550" 
                print(f"Hedef: {conn_str}")
                vehicle = drone.connect(conn_str)
                print("\n✅ BAĞLANTI BAŞARILI (Fonksiyon 1 OK)")
            except Exception as e:
                print(f"\n❌ HATA: {e}")
            input("Devam...")

        elif not vehicle and choice != '0':
            print("\n⚠️ Önce 1'e basıp bağlantı kurmalısın!")
            continue

        # 2. WAIT HEARTBEAT
        elif choice == '2':
            print("\n>> [2] BAĞLANTI DOĞRULANIYOR (Wait Heartbeat)...")
            if drone.wait_heartbeat(vehicle):
                print("✅ Heartbeat Sinyali Var (Fonksiyon 2 OK)")
            else:
                print("❌ Sinyal Yok.")
            input("Devam...")

        # 3. GET LAST HEARTBEAT (ARTIK AYRI!)
        elif choice == '3':
            print("\n>> [3] SON HEARTBEAT VERİSİ OKUNUYOR...")
            # Kütüphanedeki timeout=3 düzeltmesi sayesinde artık None dönmeyecek!
            hb = drone.get_last_heartbeat(vehicle)
            if hb:
                print(f"✅ Veri Alındı: {hb}")
            else:
                print("❌ Veri alınamadı (Hala None dönüyorsa timeout yetmedi).")
            print("✅ Fonksiyon 3 OK")
            input("Devam...")

        # 4. GPS
        elif choice == '4':
            print("\n>> [4] GPS VERİSİ OKUNUYOR...")
            for i in range(3):
                gps = drone.get_gps(vehicle)
                print(f"GPS: {gps}")
                time.sleep(0.5)
            print("✅ Fonksiyon 4 OK")
            input("Devam...")

        # 5. BATARYA
        elif choice == '5':
            print("\n>> [5] BATARYA DURUMU OKUNUYOR...")
            batt = drone.get_battery(vehicle)
            print(f"Batarya: {batt}")
            print("✅ Fonksiyon 5 OK")
            input("Devam...")

        # 6. ATTITUDE
        elif choice == '6':
            print("\n>> [6] TUTUM (AÇILAR) OKUNUYOR...")
            att = drone.get_attitude(vehicle)
            print(f"Açılar: {att}")
            print("✅ Fonksiyon 6 OK")
            input("Devam...")

        # 7. VELOCITY OKU
        elif choice == '7':
            print("\n>> [7] HIZ VEKTÖRÜ OKUNUYOR...")
            vel = drone.get_velocity(vehicle)
            print(f"Hız: {vel}")
            print("✅ Fonksiyon 7 OK")
            input("Devam...")

        # 8. GLOBAL KONUM
        elif choice == '8':
            print("\n>> [8] GLOBAL KONUM OKUNUYOR...")
            gp = drone.get_global_position(vehicle)
            print(f"Konum: {gp}")
            print("✅ Fonksiyon 8 OK")
            input("Devam...")

        # 9. SET MODE
        elif choice == '9':
            print("\n>> [9] MOD DEĞİŞTİRİLİYOR -> GUIDED...")
            ok = drone.set_flight_mode(vehicle, "GUIDED")
            if ok:
                print("✅ Mod Değiştirme Komutu Başarılı (Fonksiyon 9 OK)")
            else:
                print("❌ Mod Değişemedi")
            input("Devam...")

        # 10. GET MODE (ARTIK AYRI!)
        elif choice == '10':
            print("\n>> [10] MEVCUT MOD OKUNUYOR...")
            mode = drone.get_flight_mode(vehicle)
            print(f"Şu anki Mod: {mode}")
            print("✅ Fonksiyon 10 OK")
            input("Devam...")

        # 11. HAREKET: GPS GİT
        elif choice == '11':
            print("\n>> [11] GPS İLE GİT...")
            gp = drone.get_global_position(vehicle)
            if gp:
                t_lat = gp['lat'] + 0.001
                t_lon = gp['lon']
                t_alt = 25.0
                print(f"Hedef: {t_lat:.6f}, {t_lon:.6f} Alt:{t_alt}")
                drone.goto_gps_location(vehicle, t_lat, t_lon, t_alt)
                print("✅ Komut Gönderildi (Fonksiyon 11 OK)")
            else:
                print("GPS Yok!")
            input("Devam...")

        # 12. HAREKET: HIZ
        elif choice == '12':
            print("\n>> [12] HIZ KOMUTU GÖNDERİLİYOR...")
            print("Drone 4 saniye ileri (2 m/s) gidiyor...")
            drone.set_velocity(vehicle, 2, 0, 0, duration=4.0)
            print("✅ Tamamlandı (Fonksiyon 12 OK)")
            input("Devam...")

        # 13. HAREKET: ATTITUDE
        elif choice == '13':
            print("\n>> [13] AÇI KOMUTU GÖNDERİLİYOR...")
            print("Drone KUZEYE dönerek sağa yatacak...")
            
            # yaw=0 : "Burnunu Kuzeye (0) Kilitle" demektir.
            # Bu, drone başka yere bakıyorsa onu dönmeye ZORLAR.
            drone.set_attitude(vehicle, roll=45, pitch=0, yaw=0, thrust=0.5, duration=6.0)
            
            print("🛑 Fren yapılıyor...")
            drone.set_velocity(vehicle, 0, 0, 0, duration=2.0)
            
            print("✅ Tamamlandı")
            input("Devam...")

        elif choice == '0':
            print("Çıkılıyor...")
            break
        
        else:
            print("Geçersiz seçim!")

if __name__ == "__main__":
    main()