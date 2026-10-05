# Kapanış raporu: tek kameralı (sensörsüz) yörünge tahmini

Tarih: 5 Ekim 2026. Dal: `shape-evaluation`.

## 1. Kapsam

GNSS kullanmadan, tek kameradan drone yörüngesi tahmini. Yöntem: SuperPoint+LightGlue (ya da ORB) eşleştirme, esansiyel matrisle poz, birim öteleme ve rotasyonların birikimi. Ölçek sensörsüz çözülmedi.

## 2. Ölçülen sonuçlar

Seçilen ayarlarla, tek kanonik betikle (`experiments/evaluate_canonical.py`):

| uçuş | ön uç | şekil ATE (m) | alt yol RPE (%) | yön (°) | konum RMSE (m) |
|---|---|---|---|---|---|
| 2026 | ORB | 40,9 | 24,7 | 22,0 | 67,7 |
| 2026 | SuperPoint+LG | 28,0 | 16,7 | 6,2 | 113,1 |
| oturum_3 | ORB | 21,0 | 11,8 | 5,5 | 41,2 |
| oturum_3 | SuperPoint+LG | 14,0 | 8,3 | 3,9 | 41,3 |
| 2024 (yalnız XY) | ORB | 72,1 | 19,4 | 17,6 | 153,2 |
| 2024 (yalnız XY) | SuperPoint+LG | 44,1 | 13,3 | 13,5 | 119,2 |

Şekil ATE, GT'ye Sim(3) ile tüm yörünge üzerinden hizalanarak ölçüldü; bu ideal bir ölçümdür. Konum RMSE ölçekli, ilk 450 karenin GT hizalamasıyla hesaplanmış gerçek konum hatasıdır.

EuRoC V1_01_easy (IMU'lu, kapalı alan): proje hattıyla SuperPoint+LG şekil hatası 1,79 m (yolun %3,1'i); bağımsız ORB taban ölçümü 1,79 m (%3,4). Bu iki hat aynı değildir.

## 3. Öğrenilenler (negatif sonuçlar dahil)

- Paralaks ayrımı, GT irtifa bilgisi, semantik irtifa, eksen ve yön ölçümü ölçek hatasını açıklamadı.
- GT irtifa "ideal barometre" gibi kullanıldığında sonuç iyileşmedi.
- Adım başına rotasyon hatası küçük (~0,5°) ama birikiyor; şekil bozulmasının ana kaynağı bu.
- Homografi yolu ORB'da kısmi bir kaynak; SuperPoint'te etkisiz.
- Ölçüt değişikliği: RMSE'den şekil ölçütüne sonradan geçildi; bu açıkça belgelendi.
- Ayarsız SIFT+FLANN, yarışma verisinde SuperPoint'e şekilde yakın çıktı; SuperPoint'in üstünlüğü yalnızca ORB'a karşı gösterildi.
- Hatalı yapılandırmalar (2026 üretim yapılandırmasında GT irtifa kullanımı, oturum_3 ofseti 0) bulundu; mutlak sayılar bu nedenle iyimser olabilir.
- 2024 uçuşunda yalnızca XY metriği mevcut (GT yüksekliği yok).

## 4. Kapsamda olmayanlar (ölçülmedi)

- Uçuş sırasında gerçek zamanlı çalışma.
- Döngü kapanışı ve pencereli optimizasyon (BA).
- IMU ile ölçek.
- Hava koşulları, bulanıklık, aydınlatma gibi sağlamlık testleri.
- Metrik ölçekli konum hatasının kabul edilebilir bir seviyeye indirilmesi.

## 5. Sonuç

Sensörsüz tek kameralı yaklaşım, yarışma verisinde şekil açısından makul ama metrik konum açısından yetersiz. Ölçek sorunu sensörsüz çözülemedi. Bu, ölçülmüş bir sınır olarak kayıtlı.

## 6. Sonraki adım (ikinci proje)

IMU'lu görsel-ataletsel pipeline: kamera-IMU kalibrasyonu, zaman senkronizasyonu, IMU ön entegrasyonu ve pencereli optimizasyon. Veri: EuRoC (elimizde), ardından MUN-FRL ya da INSANE. Bu, ayrı bir proje olarak başlatılacak ve mevcut sonuçlar onun temel referansı olacak.

## 7. Yeniden üretme

```
python experiments/evaluate_canonical.py      # veri yollarını kendi kopyanıza göre ayarlayın
```

Ground truth ve yarışma verisi bu repoda yoktur; sonuçları yeniden üretmek için kendi verinizi sağlamanız gerekir.
