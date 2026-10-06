Tek kameradan, GPS kullanmadan bir drone'un yörüngesini çıkarmaya çalıştım. Hedef konum hatasını düşürmekti; ama zamanla bunun bu veriyle ulaşılamayacağını gördüm ve ölçütü değiştirdim.

Sorun ölçek. Tek kamera dönüşü ve yönü bulabiliyor, kaydığı mesafenin kaç metre olduğunu bulamıyor. Denediğim yollar (paralaks, yer irtifası, bundle adjustment) ölçeği çözmedi.

Sonuçlarda şekil ve yön tarafı net kazandı: SuperPoint+LightGlue, üç uçuşun üçünde de ORB'dan daha düşük şekil hatası verdi (yaklaşık yüzde 32 ile 39 arası azalma). 2026'da medyan yön hatası 22°'den 6°'ye indi. Ama konum hatası ölçek yüzünden tutarlı değil; bazı uçuşlarda daha kötü.

Bu bir üretim sistemi değil, bir deney. Ayar seçimi ve değerlendirme aynı iki uçuşta yapıldı; sonuçları genellemiyorum.

Ölçeği asıl çözecek şeyin IMU olacağını düşünüyorum. Bu yüzden IMU'lu görsel-ataletsel odometri üzerine yeni bir projeye başladım; henüz başlangıç aşamasında.

Yazının tamamı ve kaynakça ekte. Veriyi TEKNOFEST Havacılıkta Yapay Zeka Yarışması sağladı, teşekkürler.

#SLAM #GörselOdometri #Drone #ComputerVision #Robotik
