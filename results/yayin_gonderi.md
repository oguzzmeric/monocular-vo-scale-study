# Paylaşım metni (taslak)

Tek kameradan, GNSS kullanmadan drone yörüngesi tahmini üzerine bir çalışma yaptım. Hedef, yörüngenin şeklini GT'ye ne kadar doğru yakalayabildiğimi ölçmekti.

Ölçek sensörsüz tek kameradan çözülemiyor; bunu üç ayrı hipotezi test ederek gösterdim (paralaks, irtifa, yön ölçümü) ve sınırını raporladım. Bu yüzden ana ölçüt olarak yörüngenin GT'ye Sim(3) ile hizalanmış şekil hatasını (ATE) seçtim; literatürde monoküler görsel odometri için standart olan bu ölçüt.

Sonuçlar (üç uçuş):
- **Şekil hatası:** SuperPoint+LightGlue, üç uçuşun üçünde de ORB'dan daha düşük (2026: 28 m'ye karşı 41 m; oturum_3: 14 m'ye karşı 21 m; 2024: 44 m'ye karşı 72 m; 2024 yalnızca XY).
- **Konum hatası:** 2026'da SuperPoint+LG (113 m) ORB'dan (68 m) daha büyük. Fark, ölçeğin yanlış olmasından geliyor.

Dikkat edilmesi gerekenler:
- Şekil ölçütü GT'yi hizalama için kullanır; bu bir ideal ölçüm, uçuşta elde edilebilecek bir sonuç değil.
- Video, kaydedilmiş adım çıktısının canlandırmasıdır; sistem gösterimde canlı çalışmaz.
- 2026 yapılandırması GT irtifa bilgisini kullanır; mutlak rakamlar ideal bilgiyle iyileşmiş olabilir. İki ön uç aynı koşulda karşılaştırıldığı için göreli sonuç geçerlidir.

Sonraki adım: ölçeği IMU ile test etmek (EuRoC) ve konum hatasını bu ayrıma göre ele almak.

Kod ve sonuç tablosu: [repo bağlantısı]
