Tek kameradan, GPS kullanmadan bir drone'un yörüngesini çıkarmaya çalıştım. Hedef konum hatasını düşürmekti; ama zamanla bunun bu veriyle ulaşılamayacağını gördüm ve ölçütü değiştirdim.

Sorun ölçek. Tek kamera dönüşü ve yönü bulabiliyor, kaydığı mesafenin kaç metre olduğunu bulamıyor. Denediğim yollar (paralaks, yer irtifası, bundle adjustment) ölçeği çözmedi.

ORB'dan SuperPoint+LightGlue'ye geçince yön hatası üç uçuşun üçünde de düştü; 2026'da medyan 22,0°'den 6,2°'ye indi. Konum hatası ise 2026'da arttı: ölçek çözülemediği için bu sayıyı ana ölçüt olarak kullanmıyorum. Ayrıntılar yazıda ve GitHub'da.

Bu bir üretim sistemi değil, bir deney. Ayar seçimi ve değerlendirme aynı iki uçuşta yapıldı; sonuçları genellemiyorum.

Ölçeği asıl çözecek şeyin IMU olacağını düşünüyorum. Bu yüzden IMU'lu görsel-ataletsel odometri üzerine yeni bir projeye başladım; henüz başlangıç aşamasında.

Yazının tamamı ve kaynakça ekte. Veriyi TEKNOFEST Havacılıkta Yapay Zeka Yarışması sağladı, teşekkürler.

#SLAM #GörselOdometri #Drone #ComputerVision #Robotik
