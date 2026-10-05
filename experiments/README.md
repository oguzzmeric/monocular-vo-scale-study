# Kanonik değerlendirme

Tek komutla üç uçuş × iki ön uç (ORB, SuperPoint+LightGlue) için sonuç tablosunu üretir.

## Çalıştırma

Proje kökünden:

```
python experiments/evaluate_canonical.py
```

Çıktılar:
- `results/canonical_table.md` — okunabilir tablo
- `results/canonical_table.csv` — makine tarafından okunabilir

## Girdi

Adım kayıtları (`data/grid_<uçuş>_<ön uç>.pkl`), her biri seçilmiş ayar anahtarıyla:
- ORB: Lowe 0.75, H/E eşiği 0.45
- SuperPoint+LG: LG filtre eşiği 0.10, H/E eşiği 0.30

Bu kayıtlar `data/protocol_grid.py` ile üretilir. Yeniden üretmek için o betiği ilgili yapılandırmayla çalıştırmak gerekir (ayrıca bkz. `config_final_sp_eonly.yaml`).

GT dosyaları: `data/ground-truth.csv` (2026), `data_2025_oturum_3/...` (oturum_3), `data_2024/ground-truth.csv` (2024).

## Metrikler

| metrik | anlamı | rol |
|---|---|---|
| şekil (m) | yörünge tüm uçuş boyunca GT'ye Sim(3) ile hizalanır; kalan 3D RMS | **ana** |
| yön (°) | hizalanmış yörüngenin hız yönü ile GT hız yönü farkı (medyan) | yardımcı |
| konum RMSE (m) | yalnızca warmup (kare < 450) hizalaması, otonom karelerde 3D RMSE | ikincil |
| ölçek s | tüm yörünge Sim(3) ölçeği | ayrı rapor (demo dışı) |
| geçerli | geçerli poz oranı | kalite kontrolü |

## Önemli sınırlar

- Şekil ölçütü GT'yi tüm yörünge üzerinden kullanır: bu ideal bir ölçümdür, sistemin uçuş sırasında elde ettiği bir çıktı değildir.
- 2024 verisinde GT z yoktur; metrikler XY düzleminde hesaplanır. 2024 sonuçları diğer uçuşlarla mutlak olarak karşılaştırılmamalıdır; yalnızca kendi içinde ORB–SP kıyası geçerlidir.
- Ölçek sorunu çözülmemiştir ve bu tablo demo kapsamı dışındadır.
