# Harita: Ölçek Problemi, Yöntemler, Sorunlar ve Kavramlar

> Tarih: 5 Ekim 2026 itibarıyla durum. Bu dosya, proje boyunca yapılan tüm denemeleri, bunların sonuçlarını, neden yapıldıklarını ve hangi kararlara bağlandıklarını tek yerde toplar. Geri çekilen (yanlış çıkan) yorumlar da açıkça işaretlidir.

---

## 0. Bu dosyayı nasıl okumalı?

1. **Bölüm 1–2:** Projenin ne olduğu ve yarışma kuralları. Önce bunları oku.
2. **Bölüm 3–4:** Veri setleri ve sistem mimarisi.
3. **Bölüm 5:** Tüm denemeler, kronolojik. Her deneme için: soru, yöntem, sonuç, ne öğrettiği.
4. **Bölüm 6:** Açık sorunlar ve durumları.
5. **Bölüm 7:** Sayılar tablosu (hangi rakam hangi koşulda).
6. **Bölüm 8:** Kavramlar sözlüğü.
7. **Bölüm 9:** Geri çekilen ve düzeltilen yorumlar.
8. **Bölüm 10:** Kararlar, bekleyen onaylar, öneriler.
9. **Bölüm 11:** Özet: küçük perspektif (teknik) ve büyük perspektif (proje).

---

## 1. Proje nedir?

**Hedef:** Bir drone'un yalnızca kamera görüntüsüyle (GNSS yokken) uçuş yolunu ve konumunu tahmin etmek. Bu, **monoküler görsel odometri (VO)** ya da **görsel SLAM** problemi.

**Temel zorluk:** Tek kamera, derinliği ve metre cinsinden uzunluğu doğrudan ölçemez. Bir kare çiftinden kameranın **yönünü** (rotasyon ve translasyonun yönünü) çıkarabilir, ama **uzunluğu** (metre) çıkaramaz. Buna **ölçek belirsizliği** (scale ambiguity) denir.

**Sonuç:** Yönelim ve şekil doğru çıkabilir, ama yörünge büyük ya da küçük çıkar. Bu, hatanın büyük kısmının kaynağıdır.

**Proje konumu:** Yarışma bağlamlı bir çalışma (TEKNOFEST benzeri "Havacılıkta Yapay Zeka" şartnamesi). Yarışmaya hazırlık kararı verilmiş değil; amaç teknik kapasiteyi ölçmek, demo ve portföy.

---

## 2. Yarışma kuralları (şartname ve Soru-Cevap'tan)

Bu bölüm, neyi neden optimize ettiğimizi belirler.

| konu | kural | etkisi |
|---|---|---|
| **GT z tanımı** | Z, **ilk kareye göre yer değiştirme**. z0 = 0. | Mutlak irtifa verilmiyor. Başlangıç yüksekliği (offset) bilinmiyor. |
| **Metrik** | Ham **RMSE**. Sim(3) hizalaması **kullanılmıyor**; ölçekleme ve hizalamayı yarışmacı yapar. | Ölçek mutlak doğru olmalı. Hizalamaya sığınılamaz. "Sim3 yapmadan gönderirseniz hata çok yüksek çıkar" uyarısı var. |
| **GT verilen süre** | İlk **450 kare** GT (head_status = 1) verilir. | Kalibrasyon penceresi bu. |
| **GPS kesintisi** | GT kesilir; bazen hiç veri gelmez, bazen 50–100 karede bir anlık veri. Önceden bildirilmez. | Sistem GPS'e bağımlı olamaz; ara ara gelen GT patlamaları kullanılabilir. |
| **Resmi kalibrasyon** | İlk 450 karede **ölçek, yönelim ve öteleme** hesaplanır. GT kesilince bu değerlerle devam. GT tekrar gelirse **drift sıfırlanıp ölçek yeniden hesaplanır**. | Problem "tek seferlik ölçek" değil, **periyodik yeniden çapalama** problemi. |
| **Yönelim** | Kamera düzlemi ile dünya düzlemi farklı olduğu için yönelim de hesaplanmalı; yanlışsa ölçek de yanlış görünür. | Yönelim ayrı bir kalibrasyon adımı. |
| **Kamera** | Yere 70–90° açıyla bakar; kalibrasyon parametreleri önceden verilmiş. | Nadire yakın geometri; zemin düzlemi yaklaşımı uygun. |
| **Donanım/süre** | Tek kısıt toplam süre. Donanım, model boyutu, parametre sınırı yok. | DL modelleri kural açısından engel değil. |
| **Loop closure** | "Olabilir de olmayabilir de." | Döngü kapanışına güvenilmez. |
| **Puanlama** | Skor dönüşüm formülü paylaşılmıyor; amaç en düşük RMSE. | Hedefimiz RMSE. |

**Şartnamedeki telemetri:** Sunucudan istek üzerine GPS konumu (X, Y, Z) ve sağlık durumu (health_status) gelir. Video ile senkron verilir.

---

## 3. Veri setleri

| veri seti | kare | GT z | offset | konfig (üretim/test) | notlar |
|---|---|---|---|---|---|
| **2026** | 2250 (frame_step 5 → 449 adım) | Var | **41.6** (doğrulanmadı) | `config.yaml` (altitude_source gt) | Üretim konfigürasyonu GT irtifa kullanıyor. Ölçek ve yörünge analizlerinde en çok kullanılan uçuş. |
| **oturum_3** | 2250 (449 adım) | Var | **~63** (otonom uyumdan tahmin, doğrulanmadı) | `config_2025_oturum3.yaml` (altitude_source semantic, offset 0 — yanlış) | Test kopyalarında 63 kullanıldı. Üretimde offset hâlâ 0. |
| **2024** | Ön-seyreltilmiş (.jpg, 1920×1080, adım 1) | **Yok** (tüm z = 0) | — | `config_2024.yaml` (semantic) | GT irtifa yok → ölçek testlerine giremiyor. LightGlue'da felaket ayarları var. Gerçek kayıt (kullanıcı doğruladı). |

**Önemli kavram — offset:** Yerden irtifa = `offset − z_gt` (NED: z aşağı pozitif). Offset, uçuşun başlangıç irtifası. Şartname bunu vermiyor.

**Önemli kavram — adım:** Analizlerimizde 5 karelik adımlar kullanılıyor (`frame_step: 5`). `warmup_frames: 150` = **150 adım** = **fn < 750** kare. Bu, bir süre yanlış okunmuştu (bkz. bölüm 9).

---

## 4. Sistem mimarisi (üretim boru hattı)

Kare sırası: her kare için aşağıdaki adımlar.

1. **Düzeltme (undistort):** Kalibrasyon ile lens bozulmasını giderme.
2. **Özellik çıkarımı** (`core/feature_extractor.py`):
   - **ORB** (üretim): köşe + ikili tanımlayıcı, 3000 nokta.
   - **SuperPoint** (DL, test): öğrenilmiş nokta ve tanımlayıcı, 256-boyutlu, L2 normalize.
   - Semantik maske: tespit kutularının içindeki noktalar dışlanır.
3. **Eşleştirme** (`core/matcher.py`):
   - **Klasik:** Brute-force eşleştirme + Lowe oran testi (`lowe_ratio = 0.70`).
   - **LightGlue** (DL, test): transformer tabanlı eşleştirici. `filter_threshold`, `depth_confidence`, `width_confidence` ayarları var.
4. **Poz tahmini** (`core/motion_estimator.py`):
   - **Esansiyel matris (E)** ya da **homografi (H)** RANSAC ile hesaplanır.
   - Hangisinin seçileceği: H skoru / E skoru oranı, `homography_score_ratio_threshold` (üretim 0.45, SuperPoint testi 0.30).
   - Ayrıştırma: cheirality (derinlik pozitif) + paralaks + yeniden projeksiyon hatası ile oylama.
   - Çıktı: R (rotasyon), **t (birim vektör)**, inlier maskesi.
5. **Ölçek kurtarma** (`core/scale_recovery.py`): t birim olduğu için, metrik adım `t_scaled = scale · t`.
   - **Warmup** (ilk 150 adım): GT ile ölçek öğrenilir (`k = median(s_ref / Z)`).
   - **Otonom:** `scale = c · flow · Z / f` (flow modu) ya da `scale = k · Z` (flow'suz mod). Z: semantik ya da GT irtifa.
6. **Poz grafiği** (`core/pose_graph.py`):
   - `T_world ← T_world · [R | t_scaled]`; `force_2d` açıksa `T_world[2,3] = 0`.
   - Raporlanan konum: warmup'ta GT, GT kesilince **Sim(3)** (Umeyama) ile hizalanmış konum.
7. **Opsiyonel (test):** Pencereli BA, tam BA, loop closure (DL ya da klasik aday üretimi), GTSAM tabanlı PGO.

**Önemli:** Üretim raporlaması, warmup GT çiftlerinden tek bir Sim(3) fit ediyor. Yani bugüne kadarki 3D RMSE rakamları **warmup GT'sinden öğrenilmiş global bir ölçek ve rotasyon** içeriyor. Yarışmanın "hizalamayı yarışmacı yapar" kuralıyla uyumlu.

---

## 5. Denemeler (kronolojik)

Her denemenin sorusu, yöntemi, sonucu ve öğrettiği.

### 5.1 Başlangıç: ORB üretim ve SuperPoint+LightGlue entegrasyonu (1–4 Ekim)
- **Soru:** Daha iyi ön uç var mı?
- **Yöntem:** SuperPoint ve LightGlue entegre edildi; CUDA kurulumu, torch sürüm çakışmaları çözüldü; parametre taramaları yapıldı.
- **Sonuç (3D, m):** 2026 ORB 47.3 / SP 57.8; oturum_3 ORB 53.5 / SP 39.3; 2024 ORB 174.7 / SP 170.9.
- **Öğrettiği:** Ön uç farkı uçuşa bağlı; tek bir kazanç yok. Ayrıca SP'de H seçimi çöküyor (%0).

### 5.2 H-catastrophe ve H/E seçimi (2–3 Ekim)
- **Soru:** 2024'te neden felaket var?
- **Bulgu:** H (homografi) seçimi kötü; ayrıştırma yanlış çözüme oy veriyor. Eşik `homography_score_ratio_threshold` konfigürasyona taşındı.
- **Öğrettiği:** H/E seçimi hem pozu hem ölçek ölçümünü etkiler.

### 5.3 Lowe oranı ve RANSAC eşikleri (3–4 Ekim)
- Lowe oranı 0.70 (SP için). RANSAC eşiği 3.0 px.
- 2024'te `ransac_threshold = 1.0` SVD yakınsamama hatası veriyor (düzeltilmedi).

### 5.4 LightGlue parametre taraması (3–4 Ekim)
- `depth_confidence = −1` (erken çıkışı kapat): oturum_3'te en iyi (35.8 m, ORB 53.5'e karşı).
- 2026'da hiçbir ayar hem pozu hem yönü aynı anda iyileştirmedi.
- 2024'te `depth_confidence = 0.99` ve `width_confidence = −1` felaket (163–280 m). **Kod seviyesinde kilitlenmedi.**
- **Öğrettiği:** Evrensel "en iyi LG ayarı" yok.

### 5.5 BA (bundle adjustment) yeniden testi (4 Ekim) — **kapandı, negatif**
- Pencereli BA: 70.31 m (BA'sız 64.97 m). Tam BA: 68.72 m, yön %43 kötü.
- **Öğrettiği:** BA, monoküler ölçeği düzeltmez (ölçek bir gauge serbestliği). Kazanç yok.

### 5.6 Loop closure (DL adayları, 4 Ekim) — **park**
- 66 DL döngü adayı bulundu. Düzeltme uygulanınca sonuç değişmedi (70.31 m), bazı durumlarda felaket (3062 m, Sim(3) denemesinde).
- **Öğrettiği:** Döngü düzeltmesi tutarlı ölçek gerektirir; ölçek kayarken anlamlı değil.

### 5.7 Sim(3) yerel ölçek düzeltmesi (4 Ekim) — **park, yarım**
- Amaç: döngüler arası segment ölçeklerini düzeltmek.
- İlk deneme çarpımsal ve dejenere üçgenleme yüzünden 3062 m. Sonra kısıtlandı; sonuç belirsiz (70.31 → 69.95 m).
- **Öğrettiği:** Sim(3) ile rijit SE(3) farkı; rijit grafik ölçek taşıyamaz.

### 5.8 Oturum_3 ve 2026 ölçek testleri: altitude ve offset (5 Ekim)
- **Soru:** Ölçek neden kayıyor?
- **Bulgular:**
  - Semantik irtifa ve GT irtifa birlikte değişiyor ama semantik zayıf (oturum_3 eğim 1.19 → 0.34).
  - Oturum_3 offset 0 yerine 63 denendiğinde k değişmedi (warmup'ta k = 0.05168, ratio 0.662).
  - 2026 GT irtifa: 1.17 drift (1.03→1.28); semantik 1.01 (0.92→1.12). GT irtifa "ideal barometre" olarak yardımcı olmadı.
- **Öğrettiği:** Irtifa tek başına kök değil.

### 5.9 Adım–irtifa eğimi (5 Ekim)
- `log(g/rc) ~ log(Z)` eğimi:
  - oturum_3, GT Z: **+1.19** (korelasyon 0.56) → model tutuyor.
  - oturum_3, semantik Z: +0.34 (0.23).
  - 2026, GT Z: **+0.11** (0.04) → model tutmuyor.
- **Öğrettiği:** Oturum_3'te adım irtifayla orantılı; 2026'da değil. Flow→adım modeli 2026'da kırılıyor.

### 5.10 Paralaks (zemin düzlemi ayrımı) (5 Ekim) — **reddedildi**
- Yöntem: inlier'lar planar homografi (RANSAC 3 px) ile düzlem ve düzlem dışı diye ayrıldı; her biri için ayrı c ve oran.
- Sonuç: oturum_3: tüm 0.899, düzlem 0.914, düzlem dışı 0.891. 2026: tüm 1.040, düzlem 1.033, düzlem dışı 1.146.
- **Öğrettiği:** Paralaks kirliliği ölçek hatasının kaynağı değil.

### 5.11 Z mi flow mu drift'in kaynağı? (5 Ekim) — **flow tarafı**
- `zreq = g/(c·rc)` ile flow'un gerektirdiği irtifa.
- 2026: zs/zreq 0.99 → 0.95 → 1.22; zg/zreq 1.01 → 1.07 → 1.29. Gerçek irtifa artarken zreq düz (28–29 m).
- **Öğrettiği:** Drift semantik irtifadan değil; flow→adım modelinden geliyor.

### 5.12 Rotasyon bileşenleri (5 Ekim)
- 2026 ORB: roll ile hata korelasyonu 0.43 / 0.80 / 0.69 (segmentlere göre); pitch ve yaw ilişkisiz.
- **Durum:** Kısmen geri çekildi (roll, hareketin vekili olabilir; kesin değil).

### 5.13 Eksen oranı ve yön (5 Ekim) — **kısmen geri çekildi**
- `|t_x|` ile hata korelasyonu −0.41 (adım), −0.67 (blok).
- **Düzeltme:** t neredeyse hep kamera y-ekseninde (|t_y| ≈ 0.95). "Yanal/ileri" iki kategori yok. Korelasyon küçük sapmalara aitti. Yön-bağımlı c testi çalışmadı (warmup'ta yanal-baskın adım = 0).

### 5.14 Yön hatası ölçümü (5 Ekim) — **geri çekildi**
- Yöntem: GT XY vektörü ile tahmini t arasındaki açı; kayan pencere (±25 adım) ile ofset.
- Bulgu (ilk): 2026 warmup medyan ~10°, oturum_3 ~19°.
- **Doğrulama:** Global ofset kullanınca hatalar 40–110°. Ofset uçuş boyunca sabit değil (2026: +10 ve −12.5°; oturum_3: +131 ve +146°). Kayan pencere yalnızca titreşimi ölçüyor, sistematik sapmayı göremez.
- **Sonuç:** Yön hatası ölçümü güvenilir değil; "yön ölçek kökü" yorumu geri çekildi.

### 5.15 Ölçek oranı: SuperPoint vs ORB (5 Ekim)
| uçuş | metrik | ORB | SP |
|---|---|---|---|
| 2026 | oran medyanı | 1.098 | 1.094 |
| 2026 | drift | +0.062 | +0.102 |
| 2026 | log-std | 0.73 | 0.30 |
| oturum_3 | oran medyanı | 0.899 | 0.979 |
| oturum_3 | drift | −0.046 | −0.055 |
| oturum_3 | log-std | 0.48 | 0.23 |
- **Öğrettiği:** SP adım gürültüsünü yarıya indiriyor; sistematik ölçek hatasını tutarlı düzeltmiyor.

### 5.16 SuperPoint H/E eşik taraması (5 Ekim)
| eşik | H oranı | oran medyanı | drift | log-std |
|---|---|---|---|---|
| 0.20 | 8.9% | 1.097 | +0.073 | 0.302 |
| 0.30 | 0.4% | 1.094 | +0.102 | 0.297 |
| 0.45 | 0% | 1.094 | +0.102 | 0.297 |
- **Öğrettiği:** 0.30 ve üstü H'yi devre dışı bırakıyor. 0.20 H'yi geri getiriyor; ölçek üzerinde etkisi küçük. Uçtan uca etkisi henüz test edilmedi.

### 5.17 GT irtifa ve semantik irtifa karşılaştırması (5 Ekim)
| uçuş | irtifa | oran medyanı | drift | log-std |
|---|---|---|---|---|
| 2026 | GT Z | 1.155 | +0.274 | 0.77 |
| 2026 | semantik | 1.040 | +0.229 | 0.78 |
| oturum_3 | GT Z | 0.875 | +0.102 | 0.31 |
| oturum_3 | semantik | 0.899 | +0.034 | 0.40 |
- **Öğrettiği:** GT irtifa (ideal barometre) ölçek sorununu çözmüyor.

### 5.18 Homografi yöntemi (literatür) (5 Ekim)
- **Yöntem:** RANSAC homografi (tüm eşleşmeler) → `decomposeHomographyMat` → 4 çözüm. Zamansal tutarlılıkla çözüm seçimi (normal vektörü). Metrik adım = `d · |t/d|`, `d` = GT irtifa (offset − z).
- **Sonuç:** 2026 warmup 1.11, otonom 1.36 (drift +0.21). oturum_3 warmup 1.37, otonom 1.24 (drift −0.16). En iyi çözüm seçimi bile |log| ≈ 0.25–0.32.
- **Offset duyarlılığı:** Warmup'ta oranı 1 yapan offset: 2026 ~37.5 (−4 m), oturum_3 ~51 (−12 m).
- **Öğrettiği:** Yöntem tek başına ölçeği bulmuyor. Offset belirsizliği kısmen açıklıyor ama drift açıklamıyor.

### 5.19 Simülasyon (GPS kesintisi) (5 Ekim)
- **Amaç:** Resmi senaryo: 450 karelik kalibrasyon, sonra kesinti; GT patlamalarında yeniden çapalama.
- **Kurallar:**
  - **A:** Üretim ölçeği (her adımın kayıtlı scale değeri); Sim(3) fit (warmup).
  - **B:** Sabit ölçek (GT adım medyanı, warmup); Sim(3) fit.
  - **C2:** B + burst'lerde yalnızca ölçek ve öteleme yeniden fit (rotasyon sabit).
- **Doğrulama:** Üretimin kaydettiği yörünge ile simülasyonun birleştirmesi karşılaştırıldı; dünya Z'sinin her adımda sıfırlanması (force_2d) eksikti; eklenince artık 24 m'den 6 m'ye düştü. Kalan fark muhtemelen warmup karelerinde raporlama farkı.
- **Warmup uzunluğu hatası:** İlk koşularda "150" fn<150 (30 adım) olarak yanlış yorumlandı. Doğrusu fn<750.

**Sonuçlar (otonom RMSE, m):**

| koşul | ORB 2026 | ORB oturum_3 | DL 2026 | DL oturum_3 |
|---|---|---|---|---|
| warmup 750 (üretim eşdeğeri), A | 56.5 | 44.4 | — | — |
| warmup 750, B | 51.8 | 49.5 | — | — |
| warmup 750, C2 (50 kare burst) | 81.8 | 23.1 | — | — |
| warmup 450, A | 67.5 | 41.3 | 112.5 | 41.1 |
| warmup 450, C2 burst 50 | 53.9 | 31.0 | 35.6 | 31.3 |
| warmup 450, C2 burst 150 | 38.4 | 22.3 | 26.8 | 24.3 |
| warmup 450, C2 burst 250 | 31.4 | 19.0 | 19.5 | 19.8 |

- **Okuma:** Yeniden çapalama uzun pencerelerle iyileşiyor; ama burst 250 uçuşun yarısında GT görünmesi demek. Gerçekçi kısa patlamada (50 kare) ORB ve DL 31–54 m aralığında.
- **Önemli:** Yeniden çapalamanın kazancı burst uzunluğuna ve uçuşa bağlı; 2026'da 50 karelik C2 bazen B'den kötü (81.8 m) — kısa pencerede aşırı uyum.

### 5.20 DL vs ORB karar testi (5 Ekim)
- **Kural (önceden):** DL, iki uçuşta da ve üç burst uzunluğunda ≥%10 daha iyi olmalı.
- **Sonuç:** 2026'da DL %30–38 daha iyi; oturum_3'te eşit ya da %1–9 kötü. Kural sağlanmadı.
- **Karar (önerilen, onay bekliyor):** ORB'da kal. DL'nin 2026 kazancı tutarlı değil; üretim ölçeğiyle DL daha kötü (112 vs 67 m, 2026).

---

## 6. Açık sorunlar (durum ile)

### A. Ölçek (ana problem)
| # | sorun | durum |
|---|---|---|
| A1 | Ölçeğin kök nedeni bilinmiyor | **Açık.** Hipotezler elendi (paralaks, irtifa, eksen, yön ölçümü). |
| A2 | Semantik irtifa zayıf (oturum_3 eğim 0.34 vs GT 1.19) | Açık; ölçek modeli bunu kullanıyor. |
| A3 | 2026'da adım–irtifa bağı yok (eğim 0.11) | Açık; flow→adım modeli kırılıyor. |
| A4 | Uçuşlar arası sabit c tutmuyor | Açık; yeniden çapalama ile kısmen yönetiliyor. |
| A5 | Adım-başı gürültü (log-std 0.5–0.8 ORB) | Kısmen: SP ile ~0.3'e iniyor. |
| A6 | Drift (2026 +0.06 ile +0.27 arasında) | Açık. Offset değil. |
| A7 | Offset doğrulanmadı (2026: 41.6, oturum_3: ~63) | **Açık, organizatöre sorulabilir.** Şartname vermiyor. |
| A8 | Yönelim (kamera–dünya) kalibrasyonu ayrı adım değil | Açık; şartname bunu istiyor. |
| A9 | Yeniden çapalama tutarsız (2026'da C2 kötü, oturum_3'te iyi) | Açık; kısa pencerede aşırı uyum. |
| A10 | Z drift | Ölçekten bağımlı. Açık. |
| A11 | Loop closure / Sim(3) | Park (tutarlı ölçek gerektirir). |

### B. Ön uç ve eşleştirme
| # | sorun | durum |
|---|---|---|
| B1 | ORB mi SP mi (production kararı) | Karar kuralı önerildi; DL kriteri sağlanmadı → ORB önerisi, onay bekliyor. |
| B2 | SP H/E eşiği uçtan uca test edilmedi | Açık (0.20 önerildi, onay bekliyor). |
| B3 | LightGlue ayarları uçuşa bağlı | Açık; evrensel ayar yok. |

### C. Güvenlik ve kod
| # | sorun | durum |
|---|---|---|
| C1 | 2024: `depth_confidence = 0.99`, `width_confidence = −1` felaket | **Kilitlenmedi.** Hemen yapılabilir. |
| C2 | SVD crash (`_decompose_homography`, 2024) ve "Decomposition failed" uyarıları | Açık. |
| C3 | Bellek baskısı; arka plan işleri öldürülebiliyor | Kısmen (streaming npz). |
| C4 | `featcache_2024.pkl` (6.2 GB) eski | Silinebilir. |

### D. Kayıt ve tutarlılık
| # | sorun | durum |
|---|---|---|
| D1 | Metrik tanımı tutarsız (XY 44.61 vs 3D 47.28) | Düzeltme tablosu referans alınmalı; tek script'e bağlanmalı. |
| D2 | 2024 rakamları bölümler arasında çelişiyor (50–175 m) | Doğrulanmalı. |
| D3 | Bugünkü deneyler problemler.md'de yok | Kullanıcı isteği: çözülünce kaydet. Karar onayına göre. |
| D4 | Repo dağınık (kökte çok betik) | Açık; `experiments/` önerisi. |
| D5 | Git: çok sayıda değişiklik, commit yok | Açık. |

### E. Demo ve paylaşım
| # | sorun | durum |
|---|---|---|
| E1 | Demo tanımı belirsiz | Kullanıcı: jüri kardeşi, sonra LinkedIn/portföy. |
| E2 | Veri paylaşım koşulları kontrol edilmedi | Şartname okunmalı. |
| E3 | 25 m altı 2 haftada gerçekçi değil | Dürüst sınır olarak raporlanacak. |

---

## 7. Sayılar tablosu (hangi rakam hangi koşulda)

**Uyarı:** Farklı bölümlerde farklı metrikler kullanıldı. Aşağıdaki tablo, belirtilen koşulda geçerlidir.

### 7.1 Uçtan uca 3D RMSE (üretim/test, 3 Ekim referans tablosu)
| uçuş | ORB (üretim) | SP+LG (varsayılan) | en iyi bulunan (test) |
|---|---|---|---|
| 2026 | 47.28 m | 57.77 m (4 Ekim BA'sız: 64.97 m) | ~51.8 m |
| oturum_3 | 53.50 m | 39.29 m | ~32.5–35.8 m |
| 2024 | 174.74 m | 170.91 m | ~49.7 m (varsayılan LG) |
| **ortalama** | **~92 m** (2024 yüzünden) | ~89 m | **~45 m** |

### 7.2 Ölçek oranı (GT Z oracle, warmup c)
| uçuş | yöntem | oran | drift | log-std |
|---|---|---|---|---|
| 2026 | ORB | 1.098 | +0.062 | 0.73 |
| 2026 | SP | 1.094 | +0.102 | 0.30 |
| oturum_3 | ORB | 0.899 | −0.046 | 0.48 |
| oturum_3 | SP | 0.979 | −0.055 | 0.23 |

### 7.3 Simülasyon (GPS kesintili, otonom RMSE)
Bölüm 5.19'daki tabloya bakın.

### 7.4 Yeniden çapalama: GT görünen oran
| burst (kare / 500) | GT görünen oran |
|---|---|
| 50 | ~%10 |
| 150 | ~%30 |
| 250 | ~%50 |

Gerçekçi kısa patlama (50–100 kare): ~%10–20. Sonuçları bu oranla birlikte raporlamalıyız.

---

## 8. Kavramlar sözlüğü

Her kavram: **küçük perspektif** (ne demek) ve **proje bağlamı** (bizde ne işe yarıyor).

### 8.1 Temel görüntü geometrisi
- **Monoküler kamera:** Tek kamera. Derinlik ve metrik ölçek doğrudan ölçülemez.
- **Piksel ve kalibrasyon (K):** Görüntüdeki piksel koordinatlarını gerçek ışın yönlerine çeviren 3×3 matris (odak uzaklığı, merkez). *Proje:* Kalibrasyon parametreleri önceden verilmiş.
- **Lens bozulması (distorsiyon):** Kenarlardaki eğrilik. *Proje:* Undistort ile düzeltiliyor.
- **Optik akış (flow):** İki kare arasında piksellerin kayması. *Proje:* Ölçek formülünde kullanılıyor (`flow · Z / f`).
- **Rotasyon kompanzasyonu (rc):** Kameranın dönmesinden kaynaklanan kaymayı çıkarma (H = K R K⁻¹). *Proje:* Flow'dan yalnızca öteleme kaynaklı kısmı almak için.
- **Paralaks:** Kameraya yakın nesnelerin uzak nesnelerden daha çok kayması. *Proje:* Düzlem dışı noktalar paralaks taşır; ayrım denendi, ölçek kaynağı değildi.
- **Odak uzaklığı (f):** Kamera odak parametresi. *Proje:* `scale = c · flow · Z / f`.

### 8.2 Poz tahmini
- **Esansiyel matris (E):** İki kamera arasındaki göreli rotasyon ve öteleme yönünü (ölçeksiz) veren matris. Sahne düz değilse kullanılır.
- **Homografi (H):** Düz bir düzlemin iki görüntü arasındaki dönüşümü. Düz zemin ya da uzak sahnede kullanılır.
- **H/E ayrımı:** Hangi modelin seçileceği. *Proje:* Oran eşiği ile; SP'de H çöküyor.
- **RANSAC:** Aykırı (outlier) noktaları eleyerek model fit etmek. *Proje:* Hem E hem H için; eşik 3 px.
- **Cheirality:** Noktaların kameranın önünde (derinlik pozitif) çıkması. *Proje:* Ayrıştırma çözümlerini eler.
- **Yeniden projeksiyon hatası:** Triangülasyonla bulunan 3B noktanın görüntüye geri düşürülüp farkının ölçülmesi. *Proje:* Çözüm oylamasında.
- **Birim translasyon (t):** E veya H'den gelen öteleme yalnızca yön verir; uzunluk ölçeksizdir. *Proje:* Bu yüzden ölçek ayrıca gerekir.
- **decomposeHomographyMat:** Homografiyi 4 olası (R, t/d, n) çözümüne ayırır. *Proje:* Literatür yönteminde kullanıldı.
- **Plane normal (n):** Zemin düzlemine dik vektör. *Proje:* Çözüm seçiminde zamansal tutarlılık için.

### 8.3 Eşleştirme ve özellikler
- **ORB:** Hızlı, ikili tanımlayıcılı özellik. *Proje:* Üretim.
- **SuperPoint:** Öğrenilmiş nokta ve tanımlayıcı (DL). *Proje:* Test.
- **LightGlue:** Öğrenilmiş eşleştirici (transformer). *Proje:* Test; `depth_confidence` ve `width_confidence` erken çıkış ayarları.
- **Lowe oran testi:** En iyi iki eşleşmenin mesafe oranı eşiğin altındaysa eşleşme kabul edilir. *Proje:* 0.70.
- **KLT (Kanade-Lucas-Tomasi):** Noktaları kareler arasında izleyen optik akış yöntemi. *Proje:* BA ve izleme tracklerinde.

### 8.4 Ölçek ve kalibrasyon
- **Ölçek belirsizliği:** Monoküler sistemde uzunluk bilinmez. *Proje:* Ana problem.
- **Warmup:** Başlangıçta GT ile ölçeğin öğrenildiği pencere. *Proje:* Üretimde 150 adım (fn<750); şartnamede 450 kare.
- **Yeniden çapalama (re-anchoring):** GT geldiğinde ölçeği/hizalamayı yeniden hesaplama. *Proje:* Resmi tasarımın parçası; C2 simülasyonu.
- **Drift:** Hatanın zamanla birikmesi. *Proje:* Ölçek oranının uçuş boyunca kayması.
- **Oran (ratio):** Tahmin edilen adım / GT adımı. 1 ideal.
- **Log-std:** Adım-başı oranın logaritmasının standart sapması. *Proje:* Gürültü ölçüsü.
- **Eğim (slope):** `log(adım) ~ log(Z)` regresyonunun eğimi. 1 ise adım irtifayla orantılı.
- **Z (irtifa):** Yerden yükseklik. *Proje:* Semantik (bbox) ya da GT (offset − z).
- **Offset:** Başlangıç irtifası; `irtifa = offset − z_gt`. *Proje:* Doğrulanmadı.
- **NED:** North-East-Down koordinat sistemi (z aşağı pozitif). *Proje:* GT z konvansiyonu.
- **Semantik irtifa:** Tespit kutusunun boyutu ve bilinen nesne boyutundan derinlik tahmini. *Proje:* `d = f · H_real / d_piksel · cos²θ`.
- **Metrik derinlik modeli:** Görüntüden metre cinsinden derinlik veren öğrenilmiş model (örn. Depth Anything, UniDepth). *Proje:* Değerlendirilmedi; literatür uyarı veriyor (hava verisinde uyum sorunu).
- **Barometre:** Basınçtan irtifa ölçen sensör. *Proje:* Donanım yok; GT irtifa "ideal barometre" olarak test edildi.
- **IMU:** İvme ve dönüş ölçen sensör. *Proje:* Veri setinde yok.
- **Zemin düzlemi homografisi:** Düz zemin ve bilinen irtifa ile metrik translasyon (t/d). *Proje:* Literatür yöntemi; denendi.

### 8.5 Optimizasyon ve grafik
- **Pose graph:** Kare pozlarını düğüm, adımları kenar olarak tutan grafik. *Proje:* `T_world ← T_world · T_local`.
- **BA (bundle adjustment):** Poz ve 3B noktaları birlikte optimize etme. *Proje:* Pencereli ve tam BA denendi; kazanç yok.
- **Gauge serbestliği:** Monoküler sistemde ölçeğin ve yönelimin kendiliğinden belirlenememesi. *Proje:* BA ölçeği düzeltmez.
- **Loop closure:** Aynı yere geri dönüldüğünde birikmiş hatayı düzeltme. *Proje:* DL adayları bulundu; düzeltme ölçek bağımlı.
- **Sim(3):** Ölçek + rotasyon + öteleme dönüşümü. *Proje:* Raporlama hizalaması.
- **SE(3):** Rotasyon + öteleme (ölçek yok). *Proje:* Rijit pose graph SE(3) kullanır.
- **Umeyama:** Noktalar arasında en küçük kareler ile Sim(3) bulma yöntemi. *Proje:* Raporlamada ve simülasyonda.
- **force_2d:** Dünya Z'sini sıfırlayıp adım uzunluğunu koruma. *Proje:* Üretimde açık.

### 8.6 Değerlendirme
- **GT (ground truth):** Referans konum. *Proje:* İlk 450 karede verilir; sonrası test için.
- **RMSE:** Kök ortalama kare hata. *Proje:* Yarışmanın metriği; 3D ve XY varyantları karıştırıldı.
- **XY vs 3D:** Yalnızca yatay düzlem ya da tam üç boyut. *Proje:* Karışıklık kaynağı.
- **Hizasız (ham) RMSE:** Hizalama yapılmadan. *Proje:* Yarışma bunu istiyor.
- **Burst (patlama):** GT'nin kısa süreli görünmesi. *Proje:* Simülasyonda yeniden çapalama için.

### 8.7 Derin öğrenme terimleri
- **Önceden eğitilmiş (pretrained):** Başka veriyle eğitilmiş model. *Proje:* SP ve LG ağırlıkları.
- **Transformer:** Dikkat (attention) mekanizmalı mimari. *Proje:* LightGlue.
- **Confidence (erken çıkış):** Model kendinden emin olunca hesabı kesme. *Proje:* `depth_confidence = −1` kapatır.
- **Filter threshold:** Düşük güvenli eşleşmeleri eleme eşiği. *Proje:* 0.10.
- **Tanımlayıcı (descriptor):** Noktayı sayısal olarak tanımlayan vektör. *Proje:* SP 256-boyutlu.

---

## 9. Geri çekilen ve düzeltilen yorumlar

Bu bölüm, yanlış çıkan ya da kanıtlanmamış yorumları açıkça listeler.

| yorum | durum | neden |
|---|---|---|
| "Paralaks ölçek hatasının kaynağı" | **Reddedildi** | Düzlem ayrımı oranı değiştirmedi. |
| "Yön hatası ölçek hatasının kökü" | **Geri çekildi** | Yön ölçüm aracı hizalama titreşimini ölçüyor. |
| "Yanal/ileri hareket ölçek bağımlılığı" | **Geri çekildi** | Hareket neredeyse hep tek eksende; korelasyon küçük sapmalara aitti. |
| "Radyal flow alanı mekanizması" | **Doğrulanmadı** | Sadece varsayım; veriyle desteklenmedi. |
| "Warmup 150 = fn<150" | **Hatalı** | 150 adım = fn<750. |
| "C (yeniden çapalama) her zaman iyileştirir" | **Geri çekildi** | Doğru warmup ile 2026'da kötüleşti. |
| Grafikte GT adımı ölçeği | **Hatalı çizim düzeltildi** | Ardışık satır yerine 5 karelik adım kullanılmalıydı. |
| "Barometre ölçeği çözer" | **Yanlış** | GT irtifa ile bile drift kaldı. |
| "2026'da drift çözüldü" (erken) | **Yanlış** | GT irtifa ile bile drift 0.27. |
| "2024 senaryosu scripted" | **Reddedildi** | Kullanıcı gerçek kayıt olduğunu doğruladı. |
| "Sim(3) kullanmadan RMSE düşük" | **Düzeltildi** | Yarışma Sim(3) kullanılmıyor diyor; ama raporlama hizalaması var. |

---

## 10. Kararlar, bekleyen onaylar, öneriler

### 10.1 Kararlar (önerilen)
1. **Üretim ön ucu: ORB.** DL kriteri (iki uçuşta ≥%10) sağlanmadı.
2. **Ölçek:** Tek sensörsüz çözüm yok. Yeniden çapalama (GPS patlamaları) ana yol; ama tutarlılığı artırılmalı.
3. **Demo:** Ölçek sınırlaması dürüstçe belgelenerek; sayılar her zaman GT görünen oran ile birlikte.

### 10.2 Bekleyen onaylar
- DL kararı (ORB'da kal).
- 2024 güvenlik kilidi (`depth_confidence = 0.99`, `width_confidence = −1` yasak).
- SVD crash düzeltmesi.
- Oturum_3 offset (63) production'a alınsın mı.
- SP H/E eşiği (0.20) uçtan uca test.
- Bugünkü deneylerin problemler.md'ye kaydı (ne zaman?).

### 10.3 Senin yapabileceklerin
1. **Organizatöre sor:** "İlk karede yerden başlangıç irtifası nedir?" (offset için).
2. **Veri klasörlerine bak:** Telemetri, uçuş logu, barometre ya da IMU kaydı var mı?
3. **Şartnameyi oku:** Veri paylaşım ve yayın kuralları (LinkedIn/portföy öncesi).
4. **Yönelim:** Uçuş logunda heading (yaw) var mı? Yön ölçümünü düzeltmek için gerekir.
5. **Demo kararı:** Jüri ve hedef kitle; sınırlama dili.

### 10.4 Önerilen sonraki adımlar (kısa)
1. Yeniden çapalama tutarlılığı: uzun pencere, sağlam tahmin (medyan oranı), kısa pencerede aşırı uyumu önleme.
2. Yönelim kalibrasyonunu ayrı adım olarak eklemek.
3. Offset doğrulaması (organizatör ya da dokümantasyon).
4. DL için H/E eşiği uçtan uca.
5. Demo: 2 haftalık plan (bkz. önceki plan).

---

## 11. Özet

### 11.1 Küçük perspektif (teknik)
- **Ne ölçüyoruz?** Her kare çifti için: flow, rotasyon, birim translasyon. Ölçeği bu birim vektöre uygulayıp yörünge kuruyoruz.
- **Ne yanlış?** Adım uzunluğu (ölçek) GT'den sapıyor: 2026'da oran 1.10 civarı ve kayıyor (drift); oturum_3'te ~0.90. Adım gürültüsü ORB'da yüksek, SP'de yarıya iniyor.
- **Neden yanlış?** Flow→adım modeli 2026'da irtifayla orantılı değil; homografi yöntemi de tek başına ~1'e gelmiyor. Offset belirsiz.
- **Ne denedik?** Paralaks ayrımı, GT irtifa, semantik irtifa, rotasyon, eksen, yön, homografi, SP, BA, loop closure, Sim(3), simülasyon (yeniden çapalama).
- **Ne işe yaradı?** SP gürültüyü azaltıyor; yeniden çapalama (uzun pencere) hatayı belirgin düşürüyor (oturum_3'te 19 m).
- **Ne işe yaramadı?** BA, loop closure (park), GT irtifa ile ölçek düzeltme, yön ölçümü.
- **Şu anki en iyi rakam:** ORB ile 50 kare patlamada 31–54 m; 250 kare patlamada 19–31 m (GT görünen oran ~%50).

### 11.2 Büyük perspektif (proje)
- **Problem:** Tek kamerayla GNSS'siz metrik konum tahmini. Bu, literatürde de sensörsüz çözülmemiş bir problem; herkes ya sensör (IMU, barometre, mesafe ölçer) ya da adapte edilmiş derinlik modeli kullanıyor.
- **Yarışmanın tasarımı:** Kalibrasyon için ilk 450 karede GT; sonra GPS kesintisi ve ara ara GT. Yani "tam sensörsüz" değil, "ara ara GPS ile yeniden çapalanan VO" bekleniyor. Bu, bizim yaklaşımımızla uyumlu.
- **Bizim durum:** Ölçeğin kökü bulunamadı; ama yeniden çapalama ile makul sonuçlar (19–38 m) elde edildi; bunlar GT görünen oranla koşullu.
- **25 m altı:** Gerçekçi değil (2 hafta), ama yeniden çapalama + uzun pencere ile yaklaşılıyor; GT oranı arttıkça daha iyi.
- **DL:** Gürültüde kazanç var, ölçek ve tutarlılıkta net kazanç yok. Üretim ORB.
- **Demo:** Dürüst sınırlama ile çalışan pipeline. Startup için: sensör ya da doğrulanmış ölçek yöntemi olmadan pitch riskli.
- **Veri ve paylaşım:** TEKNOFEST benzeri veri; paylaşım koşulları kontrol edilmeli.
- **Karar riski:** İki uçuş, oynak sonuçlar, uçtan uca H/E testi eksik. Sonuçlar "işaret" olarak okunmalı.

---

*Bu dosya proje durumunu özetler; kesin rakamlar ilgili bölümdeki koşulla birlikte okunmalıdır. Güncelleme: 5 Ekim 2026.*

---

## 12. Karar kaydı: ana ölçüt şekil (5 Ekim, sonuçlar gelmeden önce)

**Karar:** Projenin ana hedefi, yörüngenin GT şekline üst üste oturması. Büyüklük (ölçek) ayrı bir sorun olarak izlenir.

**Ana ölçüt (şekil):** Tüm yörünge GT'ye Sim(3) ile (ölçek + rotasyon + öteleme) hizalanır; kalan 3D RMS = şekil hatası. Her uçuş için ayrı raporlanır (2026, oturum_3 3D; 2024 XY).

**İkincil ölçütler (raporlanır, karar değil):** Üretim ölçeğiyle konum RMSE'si; ölçek oranı ve drift; sabit ölçekli şekil hatası.

**Karar kuralı (ORB vs DL):** DL, şekil ölçütünde üç uçuşun en az ikisinde ≥%10 daha düşük şekil hatası verirse ve hiçbir uçuşta ORB'dan %5'ten fazla kötü değilse tercih edilir. Aksi halde ORB üretimde kalır.

**Ayar seçimi:** Ayar taraması (protokol ızgarası) şekil hatasına göre yapılır; seçim 2026 ve oturum_3 üzerinden, 2024 bağımsız kontrol.

**Önceki bulgu (keşifsel, bu karardan önce):** 2026 ve oturum_3 için tüm-yörünge Sim(3) şekil hatası: DL 28.0 / 14.0 m, ORB 40.9 / 21.0 m (üretim ölçeği). Bu sonuçlar karar kuralına dahil edilmez; yalnızca yönlendiricidir.

**Dikkat:** Şekil ölçütü GT'yi tümüyle kullanır (hizalama için); sistemin gerçek çıktısını değil, şekil uyumunu ölçer. Ölçek hatası ayrıca raporlanır.

### 12.1 Protokol sonuçları ve 2024 kararı (5 Ekim, 18:xx)

**Ayar seçimi (şekil ölçütü, 2026 + oturum_3 ortalaması, protokol ızgarası):**
- ORB: Lowe 0.75, H/E 0.45 (üretim ayarı) — 31.0 m ortalama.
- SuperPoint+LG: LG eşiği 0.10, H/E 0.30 — 21.0 m ortalama. (0.45 ile aynı sonuç; H/E seçimi bu ölçütte etkisiz.)

**Şekil hatası (3D; 2024 XY), seçilen ayarlarla:**

| uçuş | ORB | SuperPoint+LG | SP farkı |
|---|---|---|---|
| 2026 | 40.9 m | 28.0 m | −32% |
| oturum_3 | 21.0 m | 14.0 m | −33% |
| 2024 (XY) | 72.1 m | 44.1 m | −39% |

**Karar kuralı (§12) değerlendirmesi:**
- ≥%10 daha iyi üç uçuşun üçünde: ✅
- Hiçbir uçuşta ORB'dan %5'ten fazla kötü değil: ✅
- Ek koşul (H/E 0.20 ile 3D RMSE testinde ORB'dan kötü değil): ⏳ **Karar bekliyor.** Seçilen ayar 0.30 olduğu için bu koşul tanımı yeniden netleştirilmeli.

**Kayıt notu:** 2024 yalnızca XY metriğiyle değerlendirildi; diğer uçuşlarla mutlak karşılaştırma yapılmamalı, yalnızca her uçuş içinde ORB–SP kıyası geçerli. Şekil ölçütü GT'yi hizalamada kullandığı için ideal bir ölçüm; sistemin gerçek çıktısı değil.

**Ara durum:** Şekil kriteri (üç uçuşta) SuperPoint+LG lehine; ek koşul ve ölçüt değişikliğinin açıkça kabulü bekleniyor. Üretim ön ucu kararı, bu iki madde netleşince verilecek.

### 12.2 GÜNCEL KARAR (5 Ekim akşamı): şekil ana ölçüt

**Karar:** Projenin ana ölçütü yörüngenin GT şekline üst üste oturması (tüm yörünge GT'ye Sim(3) ile hizalanır; kalan 3D RMS = şekil hatası). Ölçek, ayrı ve açıkça raporlanan bir sorun olarak izlenir.

**Gerekçe:** Monoküler VO araştırmasında Sim(3) hizalamalı ATE yaygın bir değerlendirme yoludur. Ölçek sensörsüz çözülemediği için yöntemin geometrik kalitesini ayrı ölçmek anlamlıdır. Sensörlerle (IMU, barometre, mesafe ölçer) ölçek çözülebilir; sensörsüz kesin bir çözüm yoktur.

**Kayıt notu (dürüstlük):** Şekil ölçütü, RMSE sonuçları görüldükten sonra keşfedildi ve ana ölçüt yapıldı. Bu, ölçüt değişikliğidir. Raporlarda bu açıkça belirtilecek. Şartnamedeki resmi puanlama konum ortalama hatasına dayanır; bu, projenin ana hedefi değildir.

**Karar kuralı (güncel):** DL, şekil ölçütünde üç uçuşun en az ikisinde ≥%10 daha düşük şekil hatası verirse ve hiçbirinde ORB'dan %5'ten fazla kötü değilse tercih edilir. (Önceki "3D RMSE ek koşulu" bu kararla ana kapıdan çıkarıldı; 3D RMSE ikincil metrik olarak raporlanır.)

**Güncel sonuçlar (seçilen ayarlarla, şekil hatası):**

| uçuş | ORB | SuperPoint+LG | fark |
|---|---|---|---|
| 2026 | 40.9 m | 28.0 m | −32% |
| oturum_3 | 21.0 m | 14.0 m | −33% |
| 2024 (XY) | 72.1 m | 44.1 m | −39% |

**Karar sonucu:** Kural sağlanıyor → **DL (SuperPoint+LG) şekil ölçütünde tercih edilir.** Üretim ön ucu olarak DL önerilir; ORB, karşılaştırma tabanı olarak raporlanır. (Son onay kullanıcıya aittir.)

**İkincil metrikler (raporlanır):** Konum 3D RMSE (ORB 2026 47 m civarı, SP 2026 ~58 m; oturum_3 SP tarafı yakın); ölçek oranı ve drift; geçerli kare oranı.

### 13. Yol haritası (5–19 Ekim)

**Hafta 1 (6–12 Ekim): Temizlik ve tekrarlanabilirlik**
- 6 Ekim: §12.2'nin son onayı; organizatöre iki soru (başlangıç irtifası, GPS kesinti sıklığı); şartname veri paylaşım kuralı kontrolü (kullanıcı).
- 6–7 Ekim: Güvenlik düzeltmeleri: 2024'te `depth_confidence=0.99` ve `width_confidence=-1` kod seviyesinde yasaklanır; `_decompose_homography` SVD çöküşü korunur.
- 7–9 Ekim: Tek kanonik değerlendirme betiği: şekil hatası, 3D RMSE, ölçek oranı; üç uçuş × iki ön uç için tek tablo üretir. Eski test betikleri `experiments/` altına taşınır.
- 9–10 Ekim: Tohum varyansı: seçilen SP ve ORB ayarlarında iki tohumla şekil hatası; fark varyans içindeyse bunu raporda belirtilir.
- 10–12 Ekim: Şekil hatasının nerede yoğunlaştığı (oturum_3 üst kavis, 2026 1500–2250 segmenti); tek hipotez, tek değişiklik kuralıyla iyileştirme denemesi (zaman sınırı 2 gün).

**Hafta 2 (13–19 Ekim): Gösterim ve paylaşım hazırlığı**
- 13–14 Ekim: Demo görselleri: üç uçuşta GT ile üst üste bindirme (ORB ve SP), şekil hatası çubuk grafiği, ölçek oranı grafiği. Canlı yörünge oynatıcı istenirse yerelde hazırlanır.
- 14–15 Ekim: `README.md` ve tek komutla yeniden üretim; sonuç tablosu ve sınırlamalar bölümü (ölçek, GT kullanımı, 2024 XY).
- 15–16 Ekim: `problemler.md` kaydı (ölçüt değişikliği ve protokol dahil); `harita.md` güncellemesi.
- 16–17 Ekim: Paylaşım kararı (şartname veri kuralına göre: kod ve metodoloji mi, görseller mi).
- 18–19 Ekim: Kardeşinle prova ve son düzeltmeler.

**Paralel, düşük öncelik:** Ölçek için sensör seçeneği (barometre/IMU) yalnızca veri setinde varsa değerlendirilir; yoksa bu, projenin açık sınırı olarak yazılır.

**Riskler:** 2024 SP sonuçlarının tek uçuşta olması; organizatör cevabının gecikmesi; şekil iyileştirme denemesinin beklenenden fazla sürmesi (zaman sınırı ile yönetilir).

### 13.1 Düzeltme: belirsizlikler ve planın güncellenmesi (5 Ekim)

**Başlangıç irtifası:** Yarışma sahipleri de bilmiyor; GT yalnızca ilk kareye göre yer değiştirme veriyor. Bu, veri setinin kalıcı bir eksiğidir. Sonuç: offset'e dayanan yöntemler (flow·Z, homografi d) sınırlı güvenilirlikte; şekil ölçütü ise offset'e bağımlı değil. Organizatöre bu soru artık sorulmayacak.

**GPS kesintisi sıklığı:** Soru-Cevap'ta "bazen hiç bilgi gelmeyebilir ya da 50–100 frame aralıklarla anlık gelebilir; önceden bilgilendirilmez" ifadesi var. Ama bu, kesin bir sıklık tanımı değil. Sonuç: gerçek senaryo tanımsız; gerçek bir "tek sayı" yok.

**Plan değişikliği:** Gerçek senaryo tek bir sayıyla değil, bir **aralık** olarak ele alınır:
- Senaryo A (seyrek): patlamalar yok, yalnızca başlangıç kalibrasyonu (450 kare).
- Senaryo B (orta): her 500 karede 50 kare GT (mevcut simülasyon).
- Senaryo C (sık, kısa): her 50–100 karede 1–3 kare GT (şartnamedeki okumaya en yakın).
Bu üç senaryoda şekil hatası raporlanır. Karar, senaryolar arasında kuralın tutarlı olup olmadığına bakılarak verilir.

**Güncellenen görev (Hafta 1, 6 Ekim):** Organizatör sorusu yerine "50–100 frame aralıklarla anlık" ifadesinin **kaç kare** GT anlamına geldiğini netleştiren tek bir soru (kullanıcı, isteğe bağlı). Cevap gelmezse senaryo taraması yukarıdaki üç senaryoyla yapılır.

**Güncellenen risk:** GPS sıklığı belirsizliği ve başlangıç irtifası eksikliği, sonuçların mutlak değerini (metre) belirsiz kılar; şekil ölçütü bu belirsizlikten görece bağımsızdır, bu yüzden ana ölçüt olarak daha dayanıklıdır.

### 13.2 Senaryo tanımları (GPS/GT erişimi) ve güncel durum (5 Ekim)

Yarışmada GPS/GT erişimi tanımlı değil; bu yüzden dört senaryo ile çalışılır. Her senaryo, sistemin GT'yi ne zaman gördüğünü belirler.

| senaryo | GT erişimi | durum |
|---|---|---|
| **A (seyrek)** | Yalnızca ilk 450 kare; sonra hiç yok | Kalibrasyon sonrası yalnızca VO. Simüle edildi (üretim ölçeği ve sabit ölçek). |
| **B (orta)** | Her 500 karede 50 kare GT (yeniden çapalama) | Simüle edildi (C2, burst 50/150/250). |
| **C (sık, kısa)** | Her 50–100 karede 1–3 kare GT | Henüz uygulanmadı. Yalnızca 1–3 karelik GT ile ölçek yeniden hesaplanamaz (en az ~10 adım gerekir); yalnızca konum düzeltilebilir. Tasarım kararı gerekir. |
| **D (hiç GPS yok)** | Hiç GT yok (450 kare kalibrasyon bile yok veya yalnızca ilk kareler) | Şekil tablosu bu senaryonun şekil sonucudur: §12.2'deki sayılar (ORB 40.9/21.0/72.1 m; SP 28.0/14.0/44.1 m) yalnızca üretim ölçeği ve sabit ölçekle, yeniden çapalama olmadan hesaplanmıştır. |

**Not:** Şekil hizalaması (Sim(3)) tüm GT'yi kullanır; bu, senaryolardan bağımsız bir "şekil ideal hizalama" ölçümüdür. Senaryo ayrımı, ölçeğin ve konumun nasıl güncellendiğini ayırır; şekil ölçütü ise yalnızca yörüngenin yapısını değerlendirir. Bu nedenle D senaryosunda şekil sonucu geçerlidir, ama konum (metre) sonucu ağır bir ölçek hatası içerir.

**Yapılacaklar:**
- Senaryo C için tasarım kararı: yalnızca konum düzeltmesi mi (translasyon), yoksa ölçek de mi? Önerim: yalnızca translasyon, ölçeği değiştirmeden; böylece C'nin etkisi ayrı ölçülür.
- Senaryo B ve D için şekil hatası, üç uçuşta ORB ve SP için tek tabloda (kanonik betikle) üretilir.
- Karar kuralı her senaryoda ayrı uygulanır; kural bütün senaryolarda tutarlıysa karar güçlenir, tutarsızsa bu raporlanır.

### 13.3 Senaryo C ve D sonuçları (5 Ekim, seçilen ayarlar, kanonik betik: data/scenario_c_d.py)

| uçuş | ön uç | şekil D | şekil C | konum RMSE D | konum RMSE C |
|---|---|---|---|---|---|
| 2026 | ORB | 40.9 | 12.3 | 67.7 | 14.7 |
| 2026 | SP | 28.0 | 6.3 | 113.1 | 7.8 |
| oturum_3 | ORB | 21.0 | 7.5 | 41.2 | 8.0 |
| oturum_3 | SP | 14.0 | 7.2 | 41.3 | 7.7 |
| 2024 (XY) | ORB | 72.1 | 5.4 | 153.2 | 5.7 |
| 2024 (XY) | SP | 44.1 | 8.7 | 119.2 | 9.3 |

**Senaryo C tanımı (uygulanan):** 450. kareden sonra her ~75 karede bir (en yakın adıma) GT konumu verilir; düzeltme raporlama çerçevesinde yapılır (warmup Sim(3)), ölçek ve rotasyon değişmez.

**Okuma:** C, sık konum fix'i anlamına gelir; konum hatasını 8–15 m'ye indiriyor. Bu, GPS-denied gerçekçi bir senaryo değil; C'nin sayıları yalnızca "konum fix'i varsa ne olur" sorusunun cevabıdır.

**Sınırlar:** (1) Düzeltme 2 karelik pencere yerine tek adımda uygulanıyor. (2) Şekil C'de düzeltilmiş yörüngeye dayanıyor, bu yüzden D ile doğrudan kıyaslanamaz. (3) 2024'te 121 düzeltme anı var, diğer uçuşlarda 24.

**Önceki beklenti düzeltmesi:** C'nin etkisinin küçük ve testere dişi olacağı beklentisi yanlıştı; konum fix'i etkili.

### 13.4 Karar (5 Ekim): kapsam ve öncelik

- **Ölçek (challenge 1) demo dışında tutulur.** Demodan sonraki ikinci paylaşımda ele alınır.
- **Senaryo C gerçekçi bulunmadı** (sık konum fix'i = GPS benzeri). Sonuçlar yalnızca "konum fix'i varsa ne olur" sorusunun cevabı olarak raporlanır; ana senaryo D ve B'dir.
- **Şekil (challenge 2) şimdi öncelikli.** Adım 1: şekil hatasının yörünge üzerindeki dağılımını teşhis etmek (analiz, üretim kodu değişmez). Adım 2 ve sonrası: tek hipotez, tek değişiklik, 2 günlük zaman sınırı.

### 13.5 Şekil teşhisi (5 Ekim, analiz; üretim kodu değişmedi)

**Konum dağılımı (şekil hatası, beş dilim, medyan m):**
- 2026 ORB: 18, 17, 18, 45, 65 (ikinci yarıda birikiyor)
- 2026 SP: 22, 14, 13, 27, 38
- oturum_3 ORB: 27, 15, 20, 16, 20 (dağınık)
- oturum_3 SP: 13, 12, 13, 11, 16 (düzgün, en düşük)

**Yön hatası (hizalanmış yörüngenin hız yönü ile GT hız yönü farkı, derece):**
- 2026 ORB: −17, −18, −21, +15, +44 (medyan |hata| 22°)
- 2026 SP: −1, +1, −4, −1, +12 (medyan |hata| 6°)
- oturum_3 ORB: +7, +8, +7, +2, +2 (medyan |hata| 5.5°)
- oturum_3 SP: +2, +1, +2, +2, +6 (medyan |hata| 4°)

**Ana hipotez:** Şekil bozulması, birikmiş yön (heading) hatasından kaynaklanıyor; yön ölçütü ölçekten bağımsızdır.

**Sınırlar:** Yön hatası yerel gürültüyü de içerebilir; son dilim dönüş bölgesinde olabilir, artefakt ayrılmadı. Yön ölçütü, hizalanmış yörüngeden türetildiği için GT'yi kullanır (ideal ölçüm).

**Sonraki adım (önerilen, onay bekliyor):** Tek değişiklik: rotasyon yumuşatma; yalnızca test kopyasında, 2026 ORB ve SP için şekil hatası ve yön hatası ölçülür. Zaman sınırı 2 gün.

### 13.6 Rotasyon yumuşatma testi (5 Ekim): hipotez reddedildi

**Test:** Adım dönüşleri (rotasyon vektörleri) komşu W adımla hareketli ortalama; W = 1 (temel), 5, 15. Yalnızca test kopyası; üretim kodu değişmedi. Kanonik betik: `data/rot_smooth_test.py`, sonuçlar `data/rot_smooth_results.txt`.

| uçuş | ön uç | W=1 | W=5 | W=15 |
|---|---|---|---|---|
| 2026 | ORB | 40.9 m | 41.1 m | 42.0 m |
| 2026 | SP | 28.0 m | 28.0 m | 28.4 m |
| oturum_3 | ORB | 21.0 m | 20.8 m | 20.0 m |
| oturum_3 | SP | 14.0 m | 14.1 m | 15.1 m |

Yön hatası (medyan |derece|) da yumuşatmayla değişmedi (2026 ORB: 22° → 22° → 22°).

**Sonuç:** Hipotez "yön hatası gürültü birikiminden" reddedildi. Yön hatası sistematik bir sapma; yumuşatma yardımcı olmuyor. W=1 temel sonuçları önceki tablolarla birebir aynı, dolayısıyla test düzgün çalışıyor.

**Bir sonraki tek hipotez (analiz):** Yön hatası R'den mi geliyor, yoksa adım yön vektörü u'dan mı? Her adımda GT'nin yaw değişimi ile tahmin edilen R'nin yaw değişimi karşılaştırılacak. Üretim kodu değişmeyecek.

### 13.7 Dönüş kazancı testi (5 Ekim): hipotez reddedildi (2026), düzeltme

**Test:** GT yaw değişimi = b0 + b1 · (tahmin R-yaw değişimi). Kanonik betik: `data/yaw_gain_slope_test.py`, sonuçlar `data/yaw_gain_slope_results.txt`.

| uçuş | ön uç | b1 (eğim) | b0 | R² |
|---|---|---|---|---|
| 2026 | ORB | 0.99 | −0.12° | 0.73 |
| 2026 | SP | 1.00 | −0.11° | 0.73 |
| oturum_3 | ORB | 0.78 | −0.26° | 0.27 |
| oturum_3 | SP | 0.79 | −0.27° | 0.27 |

**Sonuç:** 2026'da rotasyonun genliği doğru (b1 ≈ 1). Önceki §13.6'daki "yarı genlik" okuması, medyan değerlerin ve GT yaw'ının öteleme yönünü de içermesinin karıştırılmasından kaynaklanıyordu; bu yorum düzeltildi. oturum_3'te b1 ≈ 0.8, uyum zayıf; ama oradaki heading hatası zaten küçük (~5°).

**Sonuç çıkarımı:** Yörünge yönü = R · u. 2026'da R doğruysa, büyük yön kayması (ORB'da −20°'den +44°'ye) öteleme yönü u'dan (E/H ayrıştırmasından) gelmeli.

**Sonraki tek hipotez (teşhis):** Yön hatasını R katkısı ve u katkısı olarak ayırmak. R katkısı küçükse, u suçlu.

### 13.8 Yön hatası ayrımı: R mi, u mu? (5 Ekim): kök neden R'de

**Test:** (a) R-oracle: her adımın z-ekseni dönüşü GT yaw değişimiyle değiştirildi, u korundu. (b) b0: R'ye sabit adım başına yaw bias'ı çıkarıldı. Kanonik betik: `data/heading_decomp_test.py`, sonuçlar `data/heading_decomp_results.txt`.

| uçuş | ön uç | temel şekil | R-oracle şekil | b0 şekil |
|---|---|---|---|---|
| 2026 | ORB | 40.9 (yön 22°) | 16.1 (yön 3.5°) | 44.4 (yön 36°) |
| 2026 | SP | 28.0 (yön 6°) | 14.3 (yön 2.9°) | 30.2 (yön 15°) |
| oturum_3 | ORB | 21.0 (yön 5.5°) | 12.0 (yön 3.1°) | 40.0 (yön 29°) |
| oturum_3 | SP | 14.0 (yön 3.9°) | 18.0 (yön 3.4°) | 41.1 (yön 35°) |

**Sonuç:**
- Yön hatası R'den geliyor. 2026'da R-oracle yönü ve şekli neredeyse düzeltiyor.
- Sabit bias hipotezi reddedildi: b0 çıkarmak her uçuşta kötüleştirdi. Hata adımdan adıma değişen ve birikerek büyüyen bir hata.
- oturum_3 SP'de R-oracle kötüleştiriyor; GT yaw değişiminin de gürültülü olabileceği, bu ölçümün tek başına kesin olmadığı anlamına geliyor.
- Uyarı: R-oracle, yaw'ı GT'ye ayarladığı için yönü düzeltmesi kısmen kendi kendini doğrulamaya dayanır; yine de hatanın R'de olduğunu gösteriyor.

**Challenge 2'nin kök nedeni (çalışma hipotezi):** Adım başına rotasyon tahminlerinin (E/H ayrıştırması) yeterince doğru olmaması, ve bu hatanın yörünge boyunca birikmesi.

**Sonraki tek hipotez (analiz):** Adım başına R hatası hangi koşullarda büyüyor? (düşük inlier, büyük flow, H seçilen kareler). Üretim kodu değişmeyecek.

### 13.9 R hatası analizi (5 Ekim): H ile seçilen adımlar kaynak adayı

**Analiz:** Adım başına R yaw hatası (R yaw − GT yaw değişimi). Kanonik betik: `data/r_error_conditions.py`, sonuçlar `data/r_error_conditions_results.txt`.

| uçuş | ön uç | medyan \|R hatası\| | H-seçilen adım | E-seçilen adım |
|---|---|---|---|---|
| 2026 | ORB | 0.55° | **1.92°** (54 adım) | 0.54° |
| 2026 | SP | 0.53° | — (2 adım) | 0.53° |
| oturum_3 | ORB | 0.50° | — (29 adım) | 0.50° |
| oturum_3 | SP | 0.51° | — (0 adım) | 0.51° |

**Korelasyonlar:** Dönüş büyüklüğü ile hata +0.3 (tüm koşullarda); inlier sayısı ile hata +0.1 ile +0.2 (zayıf).

**Sonuç (hipotez düzeyinde):** 2026 ORB'daki büyük rotasyon hatası, H ile seçilen adımlardan geliyor. SuperPoint'in şekil üstünlüğü, H seçiminin neredeyse olmamasıyla ilişkili olabilir.

**Uyarılar:** R hatası GT yaw gürültüsünü de içerir (vekil ölçüm). H seçimi zemin düzlemi dominantken doğru olabilir; "yanlış" değil "daha büyük hata veriyor" olarak okunmalı.

**Sonraki tek test (kopyada):** ORB 2026'da E-only ayrıştırma (H devre dışı). Şekil hatası 40.9 m'den belirgin düşerse H kök nedeni.

### 13.10 E-only testi (5 Ekim): H kısmen sorumlu

**Test:** ORB 2026, H/E eşiği 10 (homografi hiç seçilmez), Lowe 0.75. Kanonik betik: `data/orb_eonly_2026.py` (kayıt: `data/orb_eonly_2026.pkl`). Tüm 449 adım E ile ayrıştırıldı.

| koşul | şekil hatası | \|yön hatası\| medyan |
|---|---|---|
| ORB temel (H/E 0.45, H adımları dahil) | 40.9 m | 22.0° |
| ORB E-only (H yok) | 36.9 m | 14.5° |
| SuperPoint+LG (seçilen) | 28.0 m | 6.2° |

**Sonuç:**
- H seçimini kapatmak şekli ~%10, yönü ~%35 iyileştiriyor: H, kısmi bir kaynak.
- Ama E-only ORB hâlâ SuperPoint'in gerisinde (36.9 vs 28.0 m). Yani SuperPoint'in üstünlüğü yalnızca "H'den kaçınma" ile açıklanamaz; ön ucun eşleştirme kalitesi de katkıda bulunuyor.
- Kalan yön hatası (~14.5°) E-ayrıştırmasının kendi hatasından geliyor olabilir; bu, bir sonraki teşhis konusu.

**Sonraki adım (önerilen):** Eşleştirme kalitesinin yön hatasına etkisini ayırmak: aynı E-only koşulda SuperPoint eşleşmesiyle karşılaştırmak (SP E-only). Bu, ORB ile SP arasındaki kalan farkın ön uçtan mı yoksa ayrıştırma seçiminden mi geldiğini gösterir.

### 13.11 Veri izni (5 Ekim)

Yarışma sahiplerinden yazılı olarak yayına engel olmadığı teyit edildi (kullanıcı bildirdi). Teyit metni/kapsamı proje dosyalarına eklenmeli; kullanıcı tarafından saklanmalı. Yayın hedefi: 16 Ekim (cuma).

### 13.12 SuperPoint E-only testi ve mimari karar (5 Ekim)

| koşul (2026, şekil) | şekil hatası | \|yön\| med |
|---|---|---|
| ORB E-only | 36.9 m | 14.5° |
| SuperPoint E-only | 28.1 m | 6.2° |
| SuperPoint temel (H/E 0.30) | 28.0 m | 6.2° |

**Sonuç:**
- SuperPoint'te H seçimi sonucu değiştirmiyor (28.1 ≈ 28.0). Yani SP'nin yönü ve şekli zaten E ile ayrıştırmada.
- Aynı E-only koşulda SP, ORB'dan ~%24 daha iyi şekil veriyor: SP'nin üstünlüğü ayrıştırmadan değil, özellik/eşleştirme kalitesinden geliyor.
- ORB'da H seçimi bir kaynak (E-only ile ~%10 iyileşme), SP'de değil.

**Mimari kararı (önerim):** Poz ayrıştırma yalnızca E (homografi yolu devre dışı). Gerekçe: SP'de H sonucu değiştirmiyor, H yolu kod karmaşıklığı ve ek hata kaynağı; E-only daha sade ve tekrarlanabilir. Bu karar, kullanıcı onayına bağlı.

**Mimari kararlardan kalan:** (1) Ön uç: SP mi ORB mi (karar kuralı ve 2024 kontrolü); (3) ölçek modülünün rolü.

### 13.13 Kullanıcı kararları (5 Ekim)

- **Poz ayrıştırma:** Yalnızca E (homografi yolu kapalı). Gerekçe: SuperPoint'te H etkisiz, ORB'da kısmi kaynak.
- **Üretim ön ucu:** SuperPoint+LightGlue (LG eşiği 0.10). Gerekçe: §12.2 şekil kuralı üç uçuşta sağlandı (≥%10 daha iyi), hiçbirinde ORB'dan %5'ten fazla kötü değil.
- **Açıklama zorunluluğu:** Ek koşul ("H/E 0.20 ile 3D RMSE testi") §12.2'de şekil ölçütüyle yerine geçirildi; 3D RMSE ikincil metrik. Yayında şu açıkça belirtilmeli: DL 2026'da konum (3D RMSE) olarak ORB'dan kötü (~58 m'ye karşı ~47 m, üretim koşulu); şekil iyi, konum/ölçek değil.
- **Ölçek:** Demo ve yayın kapsamı dışında; ayrı rapor.
- **Senaryolar:** B ve D raporlanır; C gerçekçi bulunmadığı için yayında yalnızca "konum fix'i varsa" notu olarak geçer.

### 13.14 Uygulama (5 Ekim): güvenlik ve üretim yapılandırması

- **LightGlue kilidi (core/matcher.py):** `validate_lightglue_conf` — `width_confidence < 0` ve `depth_confidence >= 0.99` reddedilir (2024 felaket kombinasyonu). Doğrulandı: 5 durumun 5'i beklenen sonucu verdi.
- **SVD çöküşü (core/motion_estimator.py):** Homografi ayrıştırması `np.linalg.LinAlgError` yakalar, pozu geçersiz sayar (E yolu kapalı olduğu için üretimde H çağrılmaz; güvenlik ağı olarak eklendi).
- **Üretim yapılandırması:** `config_final_sp_eonly.yaml` — SuperPoint + LightGlue, yalnızca E (`homography_score_ratio_threshold: 10.0`). `config.yaml` (ORB) değiştirilmedi, yedek olarak duruyor.
- **Doğrulama:** Yapılandırmadan Matcher kuruldu (lightglue, cuda).

Sıradaki: §3 (tekrarlanabilirlik) — kanonik betik ve README.

### 13.15 Tekrarlanabilirlik (5 Ekim): kanonik betik

- **Betik:** `experiments/evaluate_canonical.py` — üç uçuş × iki ön uç, tek tablo (`results/canonical_table.md`, `.csv`).
- **Doğrulama:** Betik çalıştırıldı; şekil ve konum değerleri daha önce ayrı ayrı hesaplanan sayılarla birebir uyuşuyor (2026 ORB 40.9 / SP 28.0; oturum_3 21.0 / 14.0; 2024 72.1 / 44.1 m).
- **Belgeleme:** `experiments/README.md` (çalıştırma, girdi, metrikler, sınırlar).
- **Açık:** Eski test betikleri henüz `experiments/` altına taşınmadı (kökte duruyorlar); bu, ayrı bir temizlik adımıdır.

### 13.16 Görseller ve metin (5 Ekim)

- **Görseller:** `experiments/figures.py` → `results/figures/`: overlay (2026, oturum_3, 2024; ORB ve SP yan yana), şekil ve yön çubuk grafikleri. Sayılar kanonik tabloyla aynı.
- **Metin:** `results/yayin_metni.md` — 3–5 dakikalık Türkçe anlatı; sınırlar ve ölçüt değişikliği açıkça yazılı.
- **Açık:** Metnin kullanıcı tarafından gözden geçirilmesi; teşhis şeması ve yörünge canlandırması henüz yapılmadı.

### 13.17 EuRoC indirme durumu (5 Ekim)

- **Lisans:** "In Copyright – Non-Commercial Use Permitted" (ETH kaydı, arama özetinden; doğrudan teyit edilemedi). Portföy ve araştırma için uygun, ticari kullanım yok.
- **İndirme:** Bu ortamdan başarısız. `robotics.ethz.ch` zaman aşımı; ETH araştırma kataloğu ve GitHub erişilemiyor (ağ kısıtı). Veri indirilmedi.
- **Hazırlık:** `data/euroc/` klasörü oluşturuldu. Kullanıcı EuRoC'u (örn. MH_01_easy, V1_01_easy) elle indirip buraya açarsa, adaptör yazılıp kanonik değerlendirmeye eklenebilir.
- **Gerekli adaptör:** Kamera parametreleri (cam0 sensor.yaml: fx, fy, cx, cy, radtan bozulma), görüntü zamanları ile Vicon GT (state_groundtruth_estimate0) eşleştirmesi, adım kaydı üretimi.
