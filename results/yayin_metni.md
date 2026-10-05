# Yayın metni (yeni yapı, 4–5 dakika, Türkçe)

> Tüm sayılar `results/canonical_table.md` ve `experiments/evaluate_canonical.py` ile üretilmiştir. Görseller `results/figures/` altındadır.

---

## 1. Hedef ve başlangıç (0:00–0:50)

Bu projede hedef, tek bir kamerayla, GPS kullanmadan bir drone'un uçuş yolunu çıkarmaktı. Buna görsel odometri deniyor. Başlangıçta amacım, yarışmadaki gibi ham konum hatasını (RMSE) en düşük seviyeye indirmekti. Bu hedefin peşinde üç ay geçti; sonunda hedefi değiştirdim. Bunun nasıl olduğunu anlatacağım, çünkü sonuçtan çok yol önemli.

## 2. Ölçek neden zor (0:50–2:00)

Tek kamera, iki kare arasında dönüşü ve yönü bulabilir. Ama gidilen mesafeyi metre cinsinden bilemez. Buna ölçek belirsizliği denir.

Ölçeği bulmak için birçok hipotez denedik ve çoğunu eledik:

- **Paralaks:** Yakın ve uzak noktalar farklı kayıyor olabilir. Düzlem ayrımıyla denedik; oran değişmedi. Elendi.
- **İrtifa:** Yerdeki yükseklik ölçümü ölçeği düzeltir mi? Yarışmanın GT irtifasını ideal bir barometre gibi kullandık. 2026'da hata düzelmedi, hatta kötüleşti (oran 1,04'ten 1,16'ya). Semantik irtifa ise bir uçuşta zayıf kaldı.
- **Başlangıç irtifası:** Yarışmanın GT'si ilk kareye göre yer değiştirme veriyor; başlangıç irtifası verilmiyor. Bu, irtifaya dayalı yöntemleri belirsiz kılıyor.
- **Yön ölçümü:** Yön hatasını ölçmek için kurduğumuz araç, hizalama titreşimini ölçüyormuş. Bu sonucu geri çektik.

Önemli bir kırılma: rotasyonun sorun olduğunu gördük. Adım başına yaklaşık yarım derecelik bir hata var ve uzun uçuşta birikiyor. Rotasyon adımlarını GT'ye ayarlayınca şekil neredeyse düzeldi; sabit bir yaw düzeltmesi ise kötüleştirdi. Yani sorun sabit bir kayma değil, birikme.

## 3. Hedefi değiştirmek (2:00–2:50)

Sonunda bir karar verdim: tek kameradan sensörsüz ölçeği güvenilir biçimde çözemedik. Bu, literatürde de bilinen bir sınır; sensörlerle (IMU, barometre, mesafe ölçer) çözülüyor. Bu projede bu sensörler yoktu.

Bu nedenle ana ölçütü değiştirdim: yörüngenin GT şekline ne kadar oturduğu. Ölçek ve konum ikincil oldu.

Bu değişikliği saklamıyorum. Önce konum RMSE'siyle karşılaştırdık; orada ORB bazı uçuşlarda öndeydi. Sonra şekil ölçütünü keşfettik ve ana ölçüt yaptık. Bu bir ölçüt değişikliği ve dokümanda açıkça kayıtlı.

## 4. Ön uç kararı (2:50–3:40)

Ön ucu iki yönteme göre karşılaştırdım: ORB ve SuperPoint+LightGlue. İkisine de aynı bütçeyle, altı ayar denedim; seçimi yalnızca şekil ölçütüne göre yaptım.

- SuperPoint, adım gürültüsünü iki uçuşta da yarıya indirdi.
- Homografi yolu SuperPoint'te sonucu değiştirmedi; ORB'da kısmi bir kaynaktı. Bu yüzden poz ayrıştırmayı yalnızca esansiyel matrisle yaptık.

Seçilen ayarlarla şekil hatası (metre): 2026'da ORB 40,9, SuperPoint 28,0; oturum_3'te 21,0 ve 14,0; 2024'te (yalnızca XY) 72,1 ve 44,1.

## 5. Senaryolar: GPS ne zaman var (3:40–4:10)

Yarışmanın GPS kuralları belirsiz: ilk 450 karede GT var, sonra bazen hiç veri yok, bazen kısa anlık veriler geliyor. Bu yüzden dört senaryo tanımladım: yalnızca başlangıç, düzenli kısa patlamalar, hiç GPS yok ve sık ama çok kısa anlar. Son senaryo, konum fix'i gibi davrandığı için gerçekçi bulmadım ve sonuçlarda yalnızca bir not olarak bıraktım.

## 6. Sınırlar (4:10–4:45)

- **Ölçek çözülmedi.** Şekil doğru olabiliyor ama boyut yanlış; konum hatası büyük. 2026'da konum RMSE'si SuperPoint için 113 m, ORB için 68 m.
- **Konum açısından DL, 2026'da ORB'dan kötü.** Şekil için seçilen yöntem, metrede her zaman daha iyi değil.
- **Şekil ideal bir ölçüm.** Hizalama için tüm yörünge GT ile kullanıldı; uçuşta elde edilebilecek bir çıktı değil.
- **2024 yalnızca XY.** Diğer uçuşlarla mutlak karşılaştırma yapılmıyor.
- **Rotasyon birikimi düzeltilmedi.** Şekil hatasının ana kaynağı bu.

## 7. Literatürle bağlam (4:45–5:15)

Sonuçları literatürle yan yana koymak istiyorum, ama önce bir uyarı: aşağıdaki karşılaştırmalar aynı ölçütle yapılmadı; yalnızca bağlam için.

Kendi şekil hatamızı yol uzunluğunun yüzdesi olarak ifade edersek, SuperPoint+LG için 2026'da yüzde 3,6, oturum_3'te yüzde 1,7, 2024'te (yalnızca XY) yüzde 2,1 çıkıyor. ORB için bu sayılar sırasıyla yüzde 5,3, 2,5 ve 3,5.

Literatürde KITTI sürücü verisinde ORB-SLAM3 monoküler, döngü kapanışı ve düzeltme ile yaklaşık yüzde 1,1 çevirme hatası veriyor. Bizim sistemimizde bunların hiçbiri yok, ve ölçümümüz tüm yörünge tek hizalamayla yapıldığı için birebir aynı değil. Yani kendi sonucumuz, bu güçlü sistemin yaklaşık 1,5 ile 3 katı.

Hava verisinde tablo daha farklı. MovingDrone benchmark'ında DROID-SLAM hizalamayla 0,34 metre ATE veriyor; ama aynı yöntem gerçek hava verisi UAVScenes'te 104 metreye çöküyor. Yani hava verisi, veri setine çok bağlı ve zor. Ortofoto haritalarına bağlanan OrthoTrack ise hizalama olmadan 0,67 metre ATE elde ediyor; bu, ölçeği ve konumu bizim kullanmadığımız bir kaynakla sabitliyor. Yüksek irtifa drone çalışmasında ise 800 metrelik yolda en iyi sonuç 2,19 metre; bu yaklaşık yüzde 0,3, ama yöntemi ayrıntılı kontrol etmedim.

Bu bağlamda bizim sistemimiz, sensörsüz ve çapasız bir klasik-derin öğrenme görsel odometrisinin dürüst bir ölçümü. Literatürün en iyi sonuçları ya sensör ya da harita ile çapalanmış sistemlerden geliyor; bu yüzden aynı ligde değiliz. Bunu bilerek ve açıkça söylüyorum.

Kaynaklar: SuperPoint-SLAM3 (arXiv 2506.13089), OrthoTrack (arXiv 2606.25245), yüksek irtifa drone VIO çalışması (ResearchGate, 366892190).

## 8. Ne öğrendik (5:15–5:45)

Bu çalışma bir ürün değil. Tek kameradan şekil tahmini ne kadar iyi olabilir, ölçek nerede takılıyor ve hangi sensör hangi boşluğu kapatır: bunları ölçtük ve kayıt altına aldık. Yanlış çıkan birçok yorumu da geri çektik; bunlar hikâyenin parçası.

Kodu ve sonuç tablosunu tek komutla yeniden üretilebilir hale getirdim. Gelecekte bir jiroskop ya da barometre eklenirse, yukarıdaki sınırların hangilerini kapatacağı önceden belli.
