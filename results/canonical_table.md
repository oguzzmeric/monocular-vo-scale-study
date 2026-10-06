# Kanonik sonuc tablosu

| uçuş | ön uç | şekil ATE (m, ana) | alt yol RPE (%) | yön (° med.) | konum RMSE (m, ikincil) | ölçek s | geçerli |
|---|---|---|---|---|---|---|
| 2026 | ORB | 40.9 | 24.70 | 22.0 | 67.7 | 0.92 | 1.00 |
| 2026 | SP | 28.0 | 16.69 | 6.2 | 113.1 | 0.65 | 1.00 |
| oturum_3 | ORB | 21.0 | 11.76 | 5.5 | 41.2 | 1.10 | 0.99 |
| oturum_3 | SP | 14.0 | 8.34 | 3.9 | 41.3 | 0.91 | 1.00 |
| 2024 | ORB | 72.1 | 19.36 | 17.6 | 153.2 | 0.89 | 0.99 |
| 2024 | SP | 44.1 | 13.31 | 13.5 | 119.2 | 1.08 | 0.99 |

Notlar: Şekil ATE, GT ile tüm yörünge üzerinden Sim(3) hizalanarak ölçülür (ideal ölçüm; literatürdeki standart ATE-Sim3).
Alt yol RPE (%), KITTI tarzı: 100–800 m alt yollarda Sim(3) hizalı yörüngenin göreli ötelemesi hatası, yol uzunluğuna oranla.
Konum RMSE, yalnızca warmup (kare < 450) hizalamasıyla ve üretim ölçeğiyle hesaplanır.
2024'te GT z yoktur; metrikler XY düzleminde hesaplanır, diğer uçuşlarla mutlak karşılaştırma yapılmaz.
Ölçek ayrı bir sorundur ve demo kapsamı dışındadır.
