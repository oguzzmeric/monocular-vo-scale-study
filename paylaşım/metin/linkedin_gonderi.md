Tek kameradan, GPS olmadan bir drone'un yörüngesini çıkarmaya çalıştım.

Ana ölçütüm yön hatası: hızın yönü ile GT'deki hız yönü arasındaki açının medyanı. Ölçek çözülemediği için konum hatası (RMSE) sistemin kalitesini tam yansıtmıyor; yine de yarışmanın ölçütü olduğu için raporluyorum.

Üç uçuş kullandım: TEKNOFEST 2026 yarışma verisi (2026), 2025 oturum_3 verisi (oturum_3) ve 2024 verisi. 2024 yalnızca XY düzleminde değerlendirilebiliyor.

Yöntem: SuperPoint (kare başına 3000 öznitelik) ve LightGlue ile eşleştirme, esansiyel matrisle poz (homografi kapalı), birim ötelemelerin birikimiyle yörünge. Karşılaştırma için klasik ORB kullandım.

Sonuç: ORB'dan SuperPoint+LightGlue'ye geçince yön hatası üç uçuşun üçünde de düştü. 2026'da medyan 22,0°'den 6,2°'ye indi (%72 azalma). KITTI tarzı alt yol hatası (100–800 m) %29–32, Sim(3) hizalı şekil hatası %31–39 azaldı.

Parametre taramalarından çıkardıklarım: ön uç farkı uçuşa bağlı, evrensel bir en iyi LightGlue ayarı yok. SuperPoint'te homografi seçimi çöktüğü için homografi yolunu kapattım. 2024'te depth_confidence 0,99 ve width_confidence -1 kombinasyonu felakete yol açtı; bu kombinasyonu kodda kilitledim. Paralaks kirliliği ölçek hatasının kaynağı çıkmadı, GT irtifası da tek başına ölçeği düzeltmedi. Ayrıca rijit SE(3) grafiği ölçek taşıyamadığı için Sim(3) gerektiğini gördüm.

Konum hatası ise 2026'da 67,7 m'den 113,1 m'ye arttı. Sebep büyük olasılıkla ölçek; bu sayıyı sistemin sınırı olarak okumak gerekiyor.

Ölçeği çözmek için paralaks, yer irtifası, bundle adjustment ve loop closure denedim; hiçbiri ölçeği çözmedi. Bundle adjustment, monoküler sistemde ölçeğin belirlenemediği bir serbestlik olduğu için ölçeği düzeltmiyor; pencereli ve tam BA konum sonucunu da iyileştirmedi. Loop closure için 66 aday buldum, ama düzeltme tutarlı bir ölçek istiyor. Ölçeğimiz kaydığı için sonuç değişmedi, bazı denemeler ise felakete yol açtı. Bu yüzden loop closure'ı şimdilik park ettim. Bu iki deneme de ön uçtaki sistematik hatayı değil, rastgele gürültüyü düzeltiyor gibi görünüyor.

Ölçeği IMU ile çözmek için IMU'lu görsel-ataletsel odometri üzerine yeni bir projeye başladım; henüz başlangıç aşamasında.

Sınırlar: iki ön uç, üç uçuş ve ayar seçimi aynı uçuşlarda yapıldı. 2026 üretim yapılandırması GT irtifasını kullandığı için mutlak sayılar iyimser olabilir. Bu bir üretim sistemi değil, bir deney.

Kod, sonuç tablosu ve değerlendirme betikleri: https://github.com/oguzzmeric/sementic_slam_pose_estimation

Veriyi TEKNOFEST Havacılıkta Yapay Zeka Yarışması sağladı, teşekkürler.

#SLAM #GörselOdometri #Drone #ComputerVision #Robotik
