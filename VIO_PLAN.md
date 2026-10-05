# Yeni proje: IMU'lu gorsel-ataletsel odometri (VIO)

Dal: `vio-euroc`. Onceki proje (sensorsuz, sekil odakli) `KAPANIS_RAPORU.md` ile kapatildi.

## Hedef
Kamera + IMU ile metrik olcekli yorunge; olcegin IMU'dan gozlemlenebilir olmasiyla sensorsuz calismanin sinirini asmak.

## Asamalar
1. **Veri dogrulama (tamam):** EuRoC V1_01_easy: kamera 20 Hz, IMU 200 Hz, GT 200 Hz, zaman ortusur, T_BS gecerli, yercekimi ~9.78 m/s^2.
2. **Kamera-IMU zaman hizalama ve on entegrasyon:** Kareler arasi IMU olcumlerinden hareket tahmini; hizasi GT ile kontrol.
3. **Baslatma (initialization):** Olcek, yercekimi yonu ve bias ilk tahmini; GT ile karsilastirma.
4. **Pencereli optimizasyon:** Gorsel yeniden projeksiyon + IMU kisitlari; olcek gozlemlenebilirlik testi.
5. **Degerlendirme:** Mevcut kanonik olcutler (ATE, RPE) + metrik konum hatasi (IMU ile olcekli, hizalama GT'den bagimsiz tanim).
6. **Ikinci veri seti:** MUN-FRL veya INSANE (dis mekan, IMU'lu).

## Ilkeler
- Her asamada tek bir hipotez, tek bir degisiklik, GT ile dogrulama.
- Ara sonuclar `results/` altinda, GT iceren cikti repoya girmez.
- Sayilar kanonik betikle uretilir.
