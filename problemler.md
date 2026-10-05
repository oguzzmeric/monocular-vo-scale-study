# Problemler ve Çözümler — Kapsamlı Kayıt

Bu dosya, `drone_semantic_slam` projesinde başından beri yaşanan **tüm
problemleri**, bulunan/denenen **çözümleri**, ve bunlara dair aldığımız
**notları** tek bir yerde topluyor. Amaç: ileride bu projeye dönen biri
(ya da unutan biz) kavram karmaşası yaşamadan "ne oldu, neden oldu, ne
yaptık" sorularına cevap bulabilsin.

Kronolojik olarak ilerliyor (en eski en üstte). En güncel, açık
problemler için en alttaki **"Güncel Açık Problemler"** bölümüne bakın.

---

## 0. Kullanılan Teknolojiler ve Teknikler — Kavramlar

Aşağıdaki bölümlerde sürekli geçecek terimlerin kısa, sezgisel
açıklamaları. Detaylı "neden" için ilgili problem bölümüne bakın.

### Temel Pipeline Bileşenleri

- **ORB (Oriented FAST and Rotated BRIEF):** Görüntüde "köşe/ilginç nokta"
  (keypoint) bulan ve her birini 32 baytlık bir "parmak izi"ne (descriptor)
  çeviren klasik (öğrenilmemiş, elle tasarlanmış) bir algoritma. İki karede
  aynı fiziksel noktayı bulmak için kullanılıyor. `core/feature_extractor.py`.
- **Lowe's Ratio Test:** Bir noktanın bir sonraki karedeki en iyi eşleşmesi
  ile ikinci en iyi eşleşmesi birbirine çok yakınsa (mesafe oranı bir eşiğin
  üzerindeyse), o eşleşmeyi **belirsiz** sayıp atar. Tekrarlayan dokularda
  (aynı görünen birden fazla nokta) yanlış eşleşmeyi engellemek için var.
  `core/matcher.py`, config: `features.lowe_ratio` (varsayılan 0.75).
- **RANSAC (Random Sample Consensus):** Eşleşmeler arasında aykırı
  (outlier/yanlış) olanlar varken, doğru geometrik modeli (H ya da E
  matrisi) bulmaya yarayan sağlam bir tahmin yöntemi — rastgele küçük
  alt-kümelerle model dener, en çok noktayı açıklayan modeli seçer.
- **Homography (H) matrisi:** İki görüntü arasındaki dönüşümü, sahnenin
  **düz bir düzlem** olduğu varsayımıyla modelleyen 3x3 matris. Nadir
  (aşağı bakan) kamerada zemin düzleme yakınsa iyi çalışır.
- **Essential (E) matrisi:** İki görüntü arasındaki dönüşümü, genel
  (düzlemsel olmayan) sahne için modelleyen matris — kameranın gerçek
  dönüş+öteleme (R,t) bilgisini taşır ama ölçeksiz (t sadece yön, büyüklük
  değil).
- **H vs E model seçimi:** Her kare çiftinde HEM H HEM E deneniyor, hangisi
  eşleşen noktaları daha iyi açıklıyorsa (skor oranı `R_H`) o seçiliyor.
- **Disambiguation (belirsizlik giderme) / Cheirality:** H ve E matrislerinin
  her biri matematiksel olarak **4 farklı** (R,t) adayı üretir (işaret
  belirsizliği yüzünden), ama sadece biri fiziksel olarak mümkün. Bunu
  bulmak için: adayı kullanarak birkaç noktayı 3B'ye üçgenleyip (triangulate),
  o noktanın HER İKİ kameranın da ÖNÜNDE (pozitif derinlik) olup olmadığına
  bakılır ("cheirality" testi). Ek olarak **reprojeksiyon hatası** kontrolü
  (üçgenlenen nokta gerçekten doğru piksele mi düşüyor) ve **paralaks
  istisnası** (taban çizgisi çok kısaysa o noktayı oylamadan çıkar)
  eklenerek oylama güçlendirilir.
- **KLT (Kanade-Lucas-Tomasi) optik akış:** Bir noktayı, descriptor
  eşleştirmesi olmadan, ardışık kareler arasında piksel-seviyesinde takip
  eden bir yöntem — pencereli BA'nın "sürekli iz" (persistent track)
  oluşturmasında kullanılıyor.
- **Ölçek kurtarma (scale recovery):** Monoküler kamera mutlak ölçeği
  (metre) bilemez, sadece yön bilir. Bizim sistemde iki kaynaktan ölçek
  tahmini birleştiriliyor: (1) semantik derinlik (YOLO ile tespit edilen,
  bilinen boyutlu nesnelerden — ör. insan, araç — kamera-nesne mesafesi
  hesaplanır), (2) optik akış büyüklüğü (kalibre edilmiş bir çarpanla).
  `core/scale_recovery.py`.

### Optimizasyon / BA

- **Bundle Adjustment (BA):** Birden fazla kameranın pozlarını VE
  gözlenen 3B noktaların konumlarını **birlikte**, tüm piksel gözlemleriyle
  tutarlı olacak şekilde optimize eden bir problem (toplam reprojeksiyon
  hatasını minimize eder).
- **GTSAM:** BA problemini kurup çözmek için kullandığımız yazılım
  kütüphanesi (Factor graph tabanlı, Levenberg-Marquardt optimizasyonu).
  Windows'ta çalışmıyor, sadece WSL/Linux.
- **Pencereli (windowed) BA:** Tüm uçuşu tek seferde değil, örtüşen
  küçük pencerelerde (15 kare) optimize ediyoruz — hesap yükünü sınırlı
  tutmak için. `core/pose_graph.py: refine_with_persistent_map`.
- **`anchor_sigma`:** Her pencerenin ilk pozunu bir önceki pencerenin
  bitişine ne kadar "sıkı" bağladığımızı belirleyen parametre. Gevşek
  olursa pencere kendi içinde tutarlı ama küresel olarak yanlış bir
  çözüme kayabiliyor.
- **Loop closure:** Drone daha önce gördüğü bir yere geri döndüğünde,
  bunu tespit edip (görüntü eşleştirmesiyle) ekstra bir geometrik kısıt
  ekleyerek aradaki sürüklenmeyi kapatmaya çalışan teknik.
- **Sim(3) vs SE(3):** SE(3) = rijit dönüşüm (dönüş+öteleme, 6 serbestlik
  derecesi, ÖLÇEK SABİT). Sim(3) = benzerlik dönüşümü (dönüş+öteleme+ölçek,
  7 serbestlik derecesi). Monoküler SLAM'de ölçek zamanla sürüklendiği
  için, ORB-SLAM2'nin loop closure'ı özellikle Sim(3) kullanır — bizim
  sistemimiz şu an sadece SE(3) (`BetweenFactorPose3`) kullanıyor.

### Değerlendirme / Metodoloji

- **Sim(3) hizalama (Umeyama/Kabsch):** Tahmin edilen trajektoriyi GT'ye
  en iyi oturacak şekilde (ölçek+dönüş+öteleme) hizalayan kapalı-form
  çözüm — ATE (Absolute Trajectory Error) hesaplamadan önce uygulanır.
- **"Otonom-sadece" (autonomous-only) metrik:** Isınma (warmup) karelerinde
  sistem pozisyonu doğrudan GT'den kopyalıyor (bedava sıfır hata) — bu
  yüzden tüm-uçuş ortalaması yanıltıcı. 28 Eylül'den beri SADECE ısınma
  sonrası (frame >= warmup_frames×frame_step) kareler üzerinden hata
  raporlanıyor.
- **`force_2d`:** Z (irtifa) eksenindeki sistematik sürüklenmeyi bastırmak
  için, her adımda Z bileşenini sıfırlayıp XY'nin uzunluğunu koruyan
  pratik düzeltme (kök neden hâlâ bilinmiyor, bkz. ilgili bölüm).
- **Uzamsal dağılım (`spatial_distribution`) / quadtree-lite:** ORB'un
  ham çıktısında keypoint'ler görüntüde kümelenebilir. Bu, görüntüyü
  bir ızgaraya bölüp her hücrede en güçlü tek keypoint'i tutarak
  (ORB-SLAM2'nin quadtree'sinin basitleştirilmiş hali) engellenebilir.

---

## 1. Kronolojik Problem Geçmişi

### 1.1 DLT/Triangulation ve K matrisi hatası (21 Eylül)

**Problem:** Homography decomposition'da triangulation için kullanılan
projeksiyon matrislerine (P1/P2) kamera kalibrasyon matrisi (K) yanlışlıkla
iki kez uygulanıyordu (bir kez K_inv ile normalize edilmiş noktalarda,
bir kez de P matrislerinde). Bu, üçgenlenen 3B nokta konumlarını bozuyordu.

**Çözüm:** P1/P2'den K çıkarıldı (zaten normalize edilmiş koordinatlarla
çalışıldığı için P1=[I|0] olmalı). Düzeltme sonrası ham RMSE 239m'den
135m'ye düştü.

**Not:** Bu düzeltme aynı zamanda `t_norm` normalizasyonunun sırasının
önemli olmadığını da ortaya çıkardı (H zaten neredeyse hiç seçilmediği
için o kısımdaki değişikliğin etkisi yoktu) — 239→135m'lik iyileşmenin
kaynağının koordinat bayrağı değişikliği olduğu anlaşıldı.

### 1.2 Essential-matrix `retval` kapısı (21 Eylül, defalarca yeniden değerlendirildi)

**Problem:** `cv2.recoverPose`, E matrisinin 4 adayından hiçbirinin
cheirality testini geçemediği durumlarda (`retval<=0`) yine de bir poz
döndürüyordu — bu kareler 120-180° arası çılgın yön hatası üretiyordu.

**Çözüm (21 Eylül):** `retval<=0` ise pozu tamamen reddet. İlk ölçüm
karışıktı: Sim(3)-hizalı hata çok iyileşti (51.80→25.99m) ama ham/
hizalanmamış hata 2 kat kötüleşti (108.6→204.9m), ölçek %17 sıçradı.

**Not (21 Eylül):** O zamanki `min_inlier_count` eşiği (40) fiilen
işlevsizdi — sadece log basıyordu, `pose.is_valid`'i etkilemiyordu. Bu
ayrı bug sonradan (tarih net değil, muhtemelen 22-24 Eylül civarı)
düzeltildi, artık gerçekten reddediyor.

**29 Eylül'de yeniden test edildi (force_2d + E-yolu kapıları + otonom-
sadece metodolojisiyle):** Flight-bağımlı bir çatışma bulundu —
2026'da kapı AÇIK çok daha iyi (53.90 vs kapalıyken 87.71m, +%63
kötüleşme kapatılırsa), oturum_3'te kapı KAPALI daha iyi (61.39 vs
kapalıyken 47.76m, -%22 iyileşir kapatılırsa). En-kötü-senaryo
karşılaştırmasıyla (açık: -%28.5 en kötü, kapalı: -%62.7 en kötü)
**kapı AÇIK (mevcut) korundu** — değiştirilmedi.

### 1.3 Ölçek sürüklenmesi / optik akıştan ölçek (21-22 Eylül)

**Problem:** Sabit/yumuşatılmış bir ölçek çarpanı (`k_factor`, semantik
derinlikten), gerçek zamanlı ölçek değişkenliğini (irtifa+hız ikisi de
değişken) yakalayamıyordu.

**Denenen ve elenen:** SO(3) izdüşümü (R zaten geçerldi, katkı yok),
30°/90° açı filtresi (geçerli pozları eledi), sabit hizalama açısı
(açı uçuş boyunca 200° yayılıyor — tek sabit açı yok).

**Çözüm:** Homography'den (`decomposeHomographyMat`) çıkan `t`'nin
aslında `t/d` (d=düzlem mesafesi) olduğu fark edildi — ham optik akıştan
ölçek (`s = akış_px · Z / f`) denendi, kalibrasyonsuz haliyle kazanmadı
ama ipucu verdi (GT'nin adım-adım değişkenliğini üretimden daha iyi
izliyordu). Bir kalibrasyon çarpanı (`c`, warmup'ta öğreniliyor) eklenince
küçük ama tutarlı iyileşme sağladı. **22 Eylül'de entegre edildi**
(`optical_flow_scale: true`, `core/scale_recovery.py`) — hâlâ aktif.

### 1.4 Bundle Adjustment aracı kararı — scipy vs GTSAM (22 Eylül)

**Problem:** Motion-only (sadece poz, nokta yok) BA'yı scipy'nin genel
`least_squares`'iyle elle yazmak denendi (v2-v4), hiçbiri üretimi
geçemedi; gevşek regularizasyon kötüleşiyordu.

**Karar:** GTSAM'a geçildi — seyrek poz+nokta problemleri için
optimize edilmiş, hazır Levenberg-Marquardt çözücüsü olan bir kütüphane.
**Dezavantaj:** Windows'ta PyPI wheel'i yok, sadece WSL/Linux'ta çalışıyor.

### 1.5 Persistent-map (kalıcı harita) BA entegrasyonu (24 Eylül)

**Problem:** Normal pencereli BA'da, bir pencerenin sonunda hayatta olan
bir nokta (track), sonraki pencere başlayınca "hiç yaşamamış" gibi
kayboluyordu — track'ler pencere sınırına hapsolmuştu.

**Çözüm:** Tüm uçuş boyunca SÜREKLİ bir KLT takibi (pencere sınırı yok,
besleme ile) yapılıp, ortaya çıkan (bazıları pencere sınırını aşabilen)
track'ler GTSAM'e verilirken bloklara kırpılıyor — track'in kendisi
sınırdan etkilenmiyor. `core/pose_graph.py: refine_with_persistent_map`.
Ölçülen etki: hizalanmamış hata 147.94m→141.29m (+%4.5).

### 1.6 Persistent BA parametre taraması (24 Eylül) — %48.3 iyileşme

**Problem:** `window`, `replenish_threshold`, `replenish_exclude_radius`,
`min_track_len_in_window`, `max_tracks_per_window`, `parallax_cos_threshold`,
`pixel_noise_sigma`, `anchor_sigma`, `klt_win_size`, `klt_max_level`,
`klt_fb_threshold` — 11 parametrenin hiçbiri sistematik taranmamıştı.

**Çözüm:** Tümü tek tek tarandı (tüm-uçuş metrikle, o zamanki
metodolojiyle). Sonuç: %7.8'lik başlangıç iyileşmesinden %48.3'e
çıkıldı. `window=25` kötü (window=15 kaldı), `replenish_exclude_radius=8`
hafif kötü (15 kaldı), `min_track_len_in_window=12` kötü (8 kaldı),
`pixel_noise_sigma` duyarsız (1.5 kaldı), `parallax_cos_threshold=0.999`
en iyi bulundu.

**ÖNEMLİ NOT (29 Eylül'de eklendi):** Bu tarama **tüm-uçuş** metriğiyle
VE Z-sürüklenmesi/E-yolu düzeltmesi öncesi yapıldı — yani metodolojik
olarak güncel değildi. 29 Eylül'de hem force_2d+otonom-sadece hem de
E-yolu düzeltmesi sonrası **iki kez yeniden doğrulandı** (bkz. 2.4 ve
2.7) — ilginç şekilde parametrelerin "en iyi" değerleri değişmedi, ama
BA'nın genel faydası kayboldu.

### 1.7 Genellenebilirlik testi, 3 yeni uçuş, `anchor_sigma` döngü anomalisi (25 Eylül)

**Problem:** `data_2024/`, `data_2025_oturum_3/`, `data_2025_oturum_4/`
(termal) test edildi. `anchor_sigma=1e-3` (24 Eylül'ün "en iyi" değeri),
2026'da +%48.3 verirken oturum_3'te **-%41.6** (felaket) verdi.

**Kök neden teşhisi:** Gevşek `anchor_sigma`, zayıf kanıtlı (kısa iz,
düşük paralaks) pencerelerde GTSAM'in kendi içinde tutarlı ama küresel
olarak yanlış yöne dönük bir çözüme kaymasına izin veriyordu — bu,
projenin en başından beri açık olan **yön sürüklenmesi** sorununun BA
içindeki tezahürüydü. Görsel olarak: BA'nın çıkardığı trajektoride
GT'de hiç karşılığı olmayan büyük döngüler oluşuyordu.

**Çözüm:** `anchor_sigma` sıkılaştırıldı (`1e-3`→`1e-6`). Tüm 4 uçuşta:
2026 +%27.4 (düştü ama hâlâ pozitif), 2024 +%9.7 (değişmedi), oturum_3
-%3.9 (neredeyse nötr), oturum_4 -%1.5 (kapsam dışı, termal). `1e-6`
hiçbir uçuşta felaket yaratmadığı için yeni varsayılan oldu.

**Yan bulgu:** Windows/WSL OpenCV sürüm farkı (4.13.0 vs 5.0.0) tespit
edildi — kural: production/BA kıyaslaması hep aynı ortamda (WSL) yapılmalı.

**Ayrıca:** GT-tabanlı revizit kontrolü yapıldı, 3 RGB uçuşta da gerçek,
güçlü kendine-yakın-geçiş (revisit) bulundu — loop closure denemesine
zemin hazırladı.

### 1.8 Loop closure — 6+ deneme, hiçbiri işe yaramadı (25-28 Eylül)

**v1 — Taze 2-görünümlü triangülasyon:** Loop çiftinden (i,j) doğrudan
landmark üçgelemek denendi. **Başarısız:** loop çiftleri TANIM GEREĞİ
aynı fiziksel yerde, yani taban çizgisi (baseline) neredeyse sıfır —
triangulation her zaman dejenere oluyordu (`n_landmarks=0`).

**v2 — Dönüş-only kısıt:** E-matrisinden çıkan göreli DÖNMEyi doğrudan
`BetweenFactorPose3` olarak eklemek. **Başarısız:** eşik/komşu-doğrulama
ne kadar sıkılaştırılırsa sıkılaştırılsın (40→100→150 inlier eşiği,
loop_odom_rot/trans_sigma ayarları) düz BA'yı (45.48m) hiç geçemedi
(50-65m arası kaldı, monoton bile değildi).

**v3 — Landmark-tabanlı 3-görünümlü triangülasyon (ORB-SLAM2 ışığında):**
Loop noktasını kendi uçuş geçmişinde (i-back_k'ya kadar) KLT ile geriye
izleyip GERÇEK bir taban çizgisiyle 3 görünümlü (i-back_k, i, j)
triangülasyon yapmak. 738 landmark üretti ama **hiçbir ölçülebilir
etki** yaratmadı (sıkı ve gevşek odom sigma'da da aynı sonuç).

**Teori (25-28 Eylül):** Loop kısıtının rotasyon bilgisi de AYNI yanlı
`motion_estimator`'dan geliyor — bozuk bir cetvelle düzeltme yapmaya
çalışıyoruz.

**29 Eylül — Sim(3) eksikliği bulundu:** Kod incelendi
(`core/pose_graph.py:476`, `_apply_loop_closure_pgo`) — sadece
`BetweenFactorPose3` (rijit SE(3)) kullanılıyor, `Similarity3` HİÇ YOK.
ORB-SLAM2'nin monoküler loop closure'ının asıl katkısı (ölçek
sürüklenmesini de düzeltmek) bizde yapısal olarak eksik. **Henüz
denenmedi**, olası ek açıklama.

**29 Eylül — E-yolu düzeltmesiyle yeniden test edildi:** Hâlâ BA'yı/
üretimi geçemiyor (üretim 48.81m, BA 50.21m, BA+loop 51.16m, otonom-
sadece). Ama BA'nın kendisi de artık üretimi geçmiyor (bkz. 2.7) —
loop closure'ın "gerçekten" işe yarayıp yaramadığı hâlâ net değil,
çünkü üzerine inşa edildiği BA'nın zemini kaymış durumda.

### 1.9 Z (irtifa) sürüklenmesi keşfi ve `force_2d` (28 Eylül) — projenin en büyük tek bulgusu

**Keşif yolu:** LinkedIn için demo video hazırlarken (izometrik harita
görünümü), kullanıcı haritanın belirli bölgelerde bozulduğunu fark etti.

**Teşhis:** `motion_estimator`'ın ürettiği yerel-çerçeve öteleme yönünde
sistematik negatif-Z yanlılığı bulundu (adımların %85'i aşağı yönlü,
beklenen ~%50 yerine) — frame 0'dan itibaren, hem Homography hem
Essential-matrix yollarında. Kamera nadir baktığı ve dönüş yaw-ağırlıklı
olduğu için, yerel-Z dünya-Z ile yakın hizalı kalıyor — bu da adım-başı
yanlılığın neredeyse doğrusal olarak dünya-Z'ye birikmesine yol açıyordu
(GT irtifa -5m/+25m aralığında kalırken, düzeltmesiz tahmin -186m'ye
kadar sürüklendi).

**Elenen kök neden adayları:** İzole irtifa/derinlik tahmini testi (GT'yi
iyi takip ediyor, sistematik yanlılık yok), 3 farklı kamera kalibrasyonu
denemesi (hiçbiri tz-yanlılığını değiştirmedi), kamera gimbal/eğim
(kullanıcı donanımın gerçekten nadir olduğunu doğruladı).

**Çözüm (pratik, kök neden değil):** `force_2d` — her adımda Z
bileşenini sıfırlayıp XY'nin uzunluğunu koruma. Otonom-sadece ölçümle
%48.1 iyileşme (93.68m→48.59m, üretim).

**KRİTİK METODOLOJİ DERSİ — "ısınma bedavası":** Isınma karelerinde
(`mode=="warmup"`) `PoseGraph.update()` pozisyonu doğrudan GT'den
kopyalıyor (bedava sıfır hata). Bu, tüm-uçuş ortalamasını çarpıtıyor.
**Bu ders İKİ KEZ, kendi kendini düzeltme yoluyla öğrenildi:**
1. "BA neredeyse GT'nin döngüsünü mükemmel eşliyor" iddiası, incelemenin
   büyük kısmının ısınma penceresinde olduğu (bedava-sıfır-hata) ortaya
   çıkınca yanlış bulundu — gerçek sapma frame 750'de (ısınma bitişinde)
   başlıyordu.
2. "BA nihayet üretimi geçti" iddiası (30.71 vs 31.25m, tüm-uçuş),
   otonom-sadece ayrıştırılınca TERS ÇIKTI: üretim 48.59m, BA 54.10m —
   BA aslında %11 daha kötüydü.

**Yeni standart kural:** Bundan sonra HER ZAMAN otonom-sadece (frame >=
ısınma sınırı) hata raporlanacak, tüm-uçuş değil.

### 1.10 `anchor_sigma` yeniden değerlendirme (28 Eylül, doğru metodolojiyle)

Force_2d + otonom-sadece ile yeniden ölçüldüğünde: `anchor_sigma=1e-3`,
2026'da üretimi +%2.5 geçiyor (ilk gerçek BA kazancı) ama oturum_3'te
-%41.6 batıyor. **`1e-6` güvenli varsayılan olarak kesin karar verildi**
— tek-uçuş optimumu değil ama hiçbir uçuşta felaket yaratmıyor.

### 1.11 Yön (heading) sürüklenmesi — kanıt ve karakterizasyon (süregelen)

**Görsel kanıt (28 Eylül):** GT'nin gerçek bir döngü-içinde-döngü
yaptığı yerde, BA sonucu çok daha yassı/uzamış bir döngü çiziyor
(GT ~50m×52m yuvarlak, tahmin ~15m×60m uzamış) — dönüş sırasında gerçek
açısal değişimi hafife aldığımızın kanıtı.

**Karakterizasyon (`turncheck.py`, tarih net değil — 22 Eylül civarı):**
yön hatası ile GT dönüş hızı arasında korelasyon **yok** (-0.008) — hata
dönüş anlarına özgü değil, sürekli/sistematik.

**Karakterizasyon (`hecheck.py`, 21 Eylül):** `retval`/nokta-sayısı oranı
ile yön hatası arasında monoton ilişki VAR — kanıt zayıfladıkça hata
büyüyor (tam konsensüs: 27.5°, retval=0: 122.9°). Ama tam konsensüslü
karelerde bile 27.5°'lik bir taban hata kalıyor — hipotez sıçramaları
açıklıyor ama taban hatayı açıklamıyor.

**Karakterizasyon (`rhcheck.py`, tarih net değil):** `R_H` eşiğine (0.45)
yakınlık ile yön hatası arasında korelasyon yok (-0.15).

**Sonuç (21-28 Eylül arası):** yön hatası, hem "yanlış aday seçimi"
hem "dönüş anına özgü olma" hipotezleriyle açıklanamıyor — sürekli,
kanıt-gücüne bağlı, ama mekanizması tam çözülmemiş bir sorun olarak
29 Eylül'e devrediliyor.

---

## 2. 29 Eylül — Bugünkü Çalışmalar (Z-düzeltmesi sonrası "eski reddedilenleri yeniden dene" günü)

**Motivasyon:** `force_2d` Z-sürüklenmesini bastırdığı için, ondan ÖNCE
"kötü sonuç" diye reddedilen bazı yöntemlerin aslında Z-gürültüsü
yüzünden kötü göründüğü, artık işe yarayabileceği hipotezi test edildi.

### 2.1 Pencereli R-yumuşatma (`windowcheck.py`) — reddedildi

Ardışık adımların dönüşünü (Rodrigues vektörü üzerinden) merkezli
medyan filtresiyle yumuşatmak. Pencere=7: 2026'da otonom-sadece hata
-%12.6 AMA oturum_3'te +%18.1 (genellenmiyor). Ölçüldü: oturum_3'ün
gerçek dönüş içeriği 2026'dan 2.5-6x fazla (`turnrate_compare.py`) —
sabit pencere gerçek dönüşleri bulanıklaştırıyor. `core/pose_graph.py`'de
`refine_with_rotation_smoothing()` olarak kod duruyor ama **kullanılmıyor**.

**Yan not — AMF (Adaptif Medyan Filtresi) neden kullanılmadı:** Klasik
AMF görüntü gürültüsü (salt-and-pepper) için tasarlanmış, bizim rotasyon
verimize doğrudan uygulanamaz. Ayrıca fikrin kendisi (pencere büyüklüğünü
yerel varyansa göre uyarlamak) önce basit/sabit pencereli versiyonla
"prensipte işe yarıyor mu" diye ucuza test edildi (windowcheck.py'nin
amacı buydu) — sonuç negatif çıkınca adaptif versiyona yatırım
yapılmadı.

### 2.2 Essential-matrix `retval` kapısı — yeniden doğrulandı, değişmedi

Bkz. 1.2'nin sonundaki 29 Eylül notu — flight-bağımlı çatışma, kapı
AÇIK (mevcut) korundu.

### 2.3 30°/90° dönüş-açısı filtresi — tamamen etkisiz, kapatıldı

Eskiden ("geçerli pozları eledi" diye) reddedilmişti. force_2d + E-yolu
sonrası yeniden test edildi: hiçbir karede (449 kare, iki uçuş) gerçek
dönüş açısı 30°'yi bile geçmiyor — filtre hiç tetiklenmiyor, tamamen
ölü/gereksiz. Kodda `_MAX_ROTATION_DEG=30.0` sabiti hâlâ duruyor ama
kullanılmıyor (temizlik gerekiyor).

### 2.4 BA parametreleri yeniden doğrulama (E-yolu ÖNCESİ) — değişiklik yok

`parallax_cos_threshold`, `pixel_noise_sigma`, `min_track_len_in_window`
— force_2d + otonom-sadece ile yeniden tarandı, üçü de zaten en iyi/en
iyiye yakın değerlerinde. Config değişmedi.

### 2.5 Mimari inceleme (fork + ORB-SLAM2 makalesi) — 4 somut bulgu

1. **E-yolunda disambiguation kapıları eksik** (H'de var) — **bulundu,
   düzeltildi, bkz. 2.6**.
2. `_MAX_ROTATION_DEG=30.0` ölü kod (bkz. 2.3).
3. ORB'da uzamsal dağılım (quadtree) yok — **bulundu, denendi, bkz. 2.8**.
4. H ve E aynı `ransac_threshold`'u paylaşıyor, hiç ayrı taranmadı —
   **henüz denenmedi**.

### 2.6 E-yolu disambiguation kapıları — BUGÜNÜN EN BÜYÜK KAZANCI, kalıcı entegre edildi

H'nin 3 kapısı (cheirality + reprojeksiyon-hatası) `cv2.recoverPose`'un
kaba iç kontrolü yerine, `decomposeEssentialMat`'ın 4 adayına elle
taşındı (`core/motion_estimator.py: _decompose_essential`, tamamen
yeniden yazıldı). İKİ uçuşta da AYNI YÖNDE, benzer büyüklükte iyileşme:

```
                    E-YOLU KAPISIZ (eski)   E-YOLU KAPILI (yeni)
2026 otonom:            53.90 m                 47.28 m   (-%12.3)
oturum_3 otonom:         61.39 m                 53.29 m   (-%13.2)
2026 yön hatası medyan:  16.98°/23.21°           4.98°/18.80°  (senkron olmayan iki ayrı ölçüm, ikisi de büyük iyileşme)
geçerli kare (2026):     444/449                 448/449   (daha az ret)
geçerli kare (oturum_3): 441/449                 446/449   (daha az ret)
```

Mekanizma: `recoverPose`'un paralaks-istisnası ve reprojeksiyon kontrolü
yok, kaba/tek başına cheirality kullanıyor — belirsiz karelerde "hiçbir
aday güvenilir değil" (retval=0) deyip pozu tamamen reddediyordu; H'nin
titiz oylaması aynı kareleri doğru çözebiliyordu.

**Yön hatası ikili yapısı netleşti:**
- Zayıf-kanıt bileşeni → bu düzeltmeyle **çözüldü**.
- Taban/sistematik bileşen (`turncheck.py`'nin ~27.5°'i) → hâlâ kısmen
  açık.

### 2.7 BA parametreleri yeniden doğrulama (E-yolu SONRASI) — BA artık kazanmıyor

Aynı parametre taraması, YENİ (E-yolu düzeltmeli) ön-uçla tekrarlandı.
**2026'da hiçbir BA parametre kombinasyonu artık üretimi anlamlı şekilde
geçemiyor** (en iyi durumda üretimle eşit, 0.04m fark — gürültü
seviyesinde). Yorum: BA'nın eski küçük kazancı, ön-uçtaki zayıflığı
"onarmaktan" geliyormuş — ön-uç düzelince BA'nın düzeltecek bir şeyi
kalmadı, kendi yaklaşıklıkları (pencereleme) artık net katkıdan çok
gürültü ekliyor.

**oturum_3'te karışık/tutarsız sonuç:** aynı script'in İKİ farklı
çalıştırması çelişen üretim sayıları verdi (53.29m vs 37.68m, aynı
config). RANSAC rastgeleliği OLMADIĞI test edilerek doğrulandı
(`cv2.setRNGSeed` ile/-siz iki çalıştırma bit-bit aynı çıktı verdi) —
**kaynağı hâlâ bulunamadı, açık bir metodoloji bulmacası.**

### 2.8 Uzamsal dağılım (`spatial_distribution`) — güçlü ama flight-bağımlı, riskli

**Hipotez ölçümü (`quadtree_check.py`):** inlier keypoint'lerin
görüntüdeki 5x5 ızgara doluluk oranı ile |yön hatası| arasında
korelasyon = -0.21 (orta, gerçek). Doluluk<0.2: medyan 66.49°,
doluluk>=0.6 (karelerin %82'si): medyan 12.89°.

**Uygulama:** `core/feature_extractor.py` — ORB'u 3x fazla-örnekleyip
(`spatial_distribution_oversample`), görüntüyü ~max_features hücrelik
bir ızgaraya bölüp her hücrede en güçlü (response) tek keypoint'i
tutan `_spatial_bucket()` eklendi. Config: `features.spatial_distribution`
(varsayılan **false**).

**3 uçuşta test sonucu — tutarsız, öngörülemez:**
```
2026:      -%29.6 (44.61m -> 31.42m)   HARİKA
oturum_3:  +%221  (53.50m -> 172.02m)  FELAKET
2024:      -%1.2  (166.38m -> 164.34m) NÖTR
```

**oturum_3 felaketinin kök nedeni araştırması:**
- En kötü 20 adımın hepsi Essential-matrix yolundan, hata hep ~170-179°
  (neredeyse tam TERS yön — yanlış aday seçimi imzası).
- Oy dağılımı incelendi (`vote_margin_check.py`): kazanan aday EZİCİ
  çoğunlukla kazanıyor (40-50/50), diğer 3 aday SIFIR oy alıyor — yakın/
  belirsiz oylama DEĞİL, YÜKSEK GÜVENLE YANLIŞ seçim.
- GT taban çizgisi (translation) hipotezi elendi: kötü karelerin GT adım
  uzunluğu (2.22m medyan) ortalamadan BÜYÜK, düşük değil.
- GT dönüş hızı hipotezi elendi: kötü karelerin yön değişimi (2.78°
  medyan) ortalamanın (9.87°) altında — bunlar sakin/düz uçuş anları.
- Radyal mesafe (lens kenarına yakınlık) hipotezi zayıf kaldı: kötü
  kareler (1191.8px) normal karelerden (1151.2px) sadece %3.5 daha
  periferik — güçlü bir örüntü değil.
- **Görsel inceleme (`bad_frame_inspect.png`) kritik bir ipucu verdi:**
  Sahne sentetik/render edilmiş bir park (büyük "UAP" logolu düz bir
  daire + ağaçlar). Hipotez: ağaç yaprak dokusu tekrarlayan/döşeme
  (tiled) bir doku olabilir — ORB eşleştirmesi için klasik bir tuzak,
  farklı iki nokta birbirine "yüksek güvenle" yanlış eşleşebilir. Bu
  yanlış eşleşme kendi içinde geometrik tutarlı olabilir (cheirality +
  reprojeksiyon testi bunu YAKALAYAMAZ, çünkü bu testler sadece
  "kendi içinde tutarlı mı" diye bakar, "gerçekten doğru fiziksel
  noktaya mı karşılık geliyor" diye bakamaz).

**Yan bulgu:** oturum_3'ün lens distortion katsayıları 2026'dan
BİREBİR KOPYALANMIŞ (bağımsız ölçülmemiş) — ayrı bir metodoloji
eksikliği, ama radyal-mesafe testi zayıf çıktığı için bu felaketin ana
sebebi gibi görünmüyor.

**Denenen düzeltme — `lowe_ratio` sıkılaştırma (test edildi, kapatıldı):**

*(a) spatial_distribution AÇIKKEN (oturum_3):* 0.75→0.60 arası
sıkılaştırdıkça düzeliyor (172.02m/139.14°→63.06m/21.88°, tam
beklediğimiz gibi tekrarlayan-doku belirsizliğini süzüyor), ama 0.50'de
aşırı sıkılaştırma yeni bir soruna yol açıp tekrar felakete dönüyor
(113.01m/132.26°) — çok az nokta kalınca cheirality oylaması yine
güvenilmez oluyor. En iyi durumda bile (0.60) hâlâ orijinal (spatial_
distribution kapalı) duruma göre daha kötü.

*(b) spatial_distribution KAPALIYKEN (mevcut varsayılan, hem 2026 hem
oturum_3'te test edildi):* **Sonuç tutarsız ve güvensiz.** 2026'da
0.65 kötüleşiyor (47.28→92.79m) ama 0.60 toparlanıyor (47.20m, hatta
yön hatası iyileşiyor: 16.98→8.91°) — monoton bile değil. oturum_3'te
ise 0.60 ciddi zarar veriyor (53.50m→95.38m, +%78, yön hatası
5.62→66.37°).

**KARAR: hem `spatial_distribution` HEM `lowe_ratio` değişikliği
kapatıldı/geri alındı.** `lowe_ratio=0.60`, spatial_distribution'ın
yarattığı spesifik soruna özel bir "tedavi" — genel/güvenli bir
iyileştirme değil, tek başına ciddi zarar verebiliyor. Her ikisi de
mevcut/varsayılan değerlerinde kalıyor (`spatial_distribution: false`,
`lowe_ratio: 0.75`). Bu alt-araştırma iyi karakterize edildi (kök
neden: tekrarlayan/sentetik doku + cheirality'nin "iç-tutarlı ama
yanlış" eşleşmeleri yakalayamaması) ama güvenli bir çözüm bulunamadı —
bkz. Güncel Açık Problemler'deki önerilen sonraki adımlar.

### 2.9 Komşu-kare tutarlılık kontrolü ("gating") — mimari kusur nedeniyle reddedildi

**Fikir:** bir adımın dünya-çerçevesi öteleme yönü, son birkaç kabul
edilmiş adımın ortalama yönünden çok sapıyorsa (~90°'den fazla —
felaket karelerin ~170-179°'lik "sıçrama" imzasını yakalamak için)
reddet. Literatürdeki en yakın karşılığı **gating** (Kalman filtresi/
hedef takibi literatüründen, Mahalanobis gating).

**1. deneme — başarısız (tasarım hatası):** Referans penceresi
(`recent_dirs`) SADECE kabul edilen adımlarla güncelleniyordu. Erken
bir yanlış-red, pencereyi dondurup gerçek trajektorinin zamanla o eski
referanstan doğal olarak uzaklaşmasına (kendi kendini besleyen bir
çöküş sarmalına) yol açtı — 2026'da 411 kare (neredeyse hepsi!)
reddedildi, hata 47.28m→205.59m'ye fırladı.

**2. deneme — düzeltildi ama yine başarısız (daha derin bir mimari
kusur):** Pencere artık kabul/red durumundan bağımsız her adımda
güncelleniyor (donma sorunu çözüldü, red sayıları makul seviyeye
indi: 4-19 arası). Ama sonuç HÂLÂ kötü — oturum_3'te sadece 4 kare
reddedilmesi bile hatayı 53.50m→226.80m'ye çıkardı. **Kök neden:**
reddedilen bir adımda pozisyonu "dondurmak" ("hiç hareket etmedi"
varsaymak) zincirleme (dead-reckoning) bir sistemde çok pahalı —
drone GERÇEKTEN hareket etti, sadece o hareketi kaydetmedik; kaçırılan
mesafe asla telafi edilmiyor, tahmin GT'nin gerisinde kalıp bir daha
yetişemiyor (VO hatasının biriken/kümülatif doğası, bkz. Not).

**Doğru yaklaşım (denenmedi, daha büyük bir iş):** "reddet ve dur"
yerine "reddet ve TAHMİN ET" (son birkaç adımın ortalama hız/yönüyle
o adımı doldurmak) — küçük bir hareket-tahmin modeli (Kalman filtresi
tarzı) gerektirir, bugünün kapsamı dışında bırakıldı.

**KARAR: komşu-tutarlılık kontrolü reddedildi**, hem `spatial_distribution`
açık hem kapalı durumda, iki uçuşta da durumu kötüleştirdi.

### 2.10 (30 Eylül) Tam BA — denendi, işe yaramadı; taban yön hatası E-yolu düzeltmesiyle zaten büyük ölçüde düzelmiş

**Tam BA (`refine_with_full_ba`, `core/pose_graph.py`):** pencereli BA'nın AYNI sürekli-KLT altyapısını kullanır ama uçuşu bloklara bölmeden TEK bir GTSAM grafiğinde çözer. Test (2026, oturum_3, otonom-sadece):
```
              ÜRETİM    PENCERELİ BA   TAM BA
2026:          48.81m      50.21m       51.32m   (kötü)
oturum_3:      37.68m      39.02m       37.68m   (nötr, 2135 gerçek iz kullanıldı — bug değil)
```
**Sonuç: değiştirmiyor/kötüleştiriyor, entegre edilmedi.** Muhtemel sebep: `_solve_window_gtsam`'daki her-poza-özel PriorFactorPose3 (ön-uç güveninden türetilen sigma) optimizasyonu zaten ön-uç tahminine çok sıkı tutuyor, izlerin çekim gücü bunu aşamıyor.

**Taban yön hatası ölçümü (hecheck.py'nin 21 Eylül'deki AYNI metodolojisiyle tekrarlandı):** eski bulgu "tam konsensüslü karelerde bile medyan 27.5° kalıyor" idi. E-yolu düzeltmesi sonrası: **medyan 6.16°'ye düştü** — yani E-yolu kapıları sadece zayıf-kanıt kuyruğunu değil, TABAN/sistematik bileşeni de büyük ölçüde düzeltmiş. Ortalama hâlâ yüksek (21.05°) — kuyruk hâlâ var ama taban artık sağlıklı. `data/viz/trajectory_guncel_en_iyi_30eylul.png`: GT'nin gerçek döngüsü hâlâ tam yakalanamıyor (kalıntı sistematik hata görünür) ama alt döngüde GT ile neredeyse birebir örtüşüyor.

**Nadir UAV makalesi tam okundu (ar5iv üzerinden, sadece özet değil):** 5 sistemin (ORB-SLAM3, DROID-SLAM, DPVO, MASt3R-SLAM, VGGT-SLAM2) tam sonuç tablosu çıkarıldı — **önemli düzeltme: "DL her zaman daha iyi" değil.** Klasik ORB-SLAM3, ortalamada 2 DL sistemini (MASt3R-SLAM, VGGT-SLAM2) geçiyor; yüksek-irtifa (ALTO) veri setinde ORB-SLAM3 EN İYİSİ, MASt3R-SLAM EN KÖTÜSÜ. Hangi sistemin kazandığı veri setine göre tamamen değişiyor — bizim kendi "flight-bağımlı, evrensel kazanan yok" deneyimimizle (retval kapısı, R-yumuşatma, spatial_distribution) birebir örtüşüyor. Detay: `okunacaklar.md`.

**Sim(3) loop closure ertelendi:** GTSAM'de `Similarity3`/`BetweenFactorSimilarity3` mevcut (kontrol edildi), ama tam BA'nın da işe yaramaması "arka-uç düzeltmeleri (BA/Sim3) ön-uçtaki SİSTEMATİK yanlılığı düzeltemez, sadece rastgele gürültüyü düzeltir" teorisini güçlendirdi — Sim(3)'e yatırım yapmadan önce taban yön hatasına odaklanma kararı alındı (yukarıdaki ölçüm bunun sonucu).

---

## Parametre/Teknik → İlgili Olduğu Şey (Hızlı Referans Tablosu)

Bugün (29 Eylül) denenen her parametrenin/tekniğin TAM OLARAK neyi
hedeflediğini gösteren tablo — kavram karmaşası olmasın diye.

| Parametre/Teknik | Bulunduğu Yer | Neyin Parametresi/Neyle İlgili | Hedeflediği Sorun |
|---|---|---|---|
| `_decompose_essential` (E-yolu kapıları) | `core/motion_estimator.py` | Essential matrix'in 4 (R,t) adayından doğrusunu seçme mantığı | Zayıf-kanıtlı karelerde yanlış/hiç aday seçilememesi |
| `retval<=0` kapısı | `core/motion_estimator.py` | `cv2.recoverPose`'un iç güven sinyali | Hiçbir adayın cheirality'yi geçemediği dejenere kareler |
| `_MAX_ROTATION_DEG` (ölü kod) | `core/motion_estimator.py` | Adım-başı dönüş açısı üst sınırı | (Artık tetiklenmiyor, temizlik gerekiyor) |
| `min_inlier_count` (40) | `core/motion_estimator.py` | RANSAC sonrası inlier nokta sayısı eşiği | Çok az kanıtla poz üretmeyi engellemek |
| `lowe_ratio` (0.75) | `core/matcher.py` | En iyi eşleşme / ikinci en iyi eşleşme mesafe ORANI | Belirsiz (tekrarlayan dokulu) eşleşmeleri elemek |
| `spatial_distribution` | `core/feature_extractor.py` | ORB keypoint'lerinin görüntüdeki uzamsal dağılımı (ızgara doluluk) | Keypoint kümelenmesinin dönüş tahminini yerel geometriye bağımlı kılması |
| `parallax_cos_threshold` | `config.yaml: persistent_ba` | BA penceresinde bir track'in kabul edilmesi için gereken min. paralaks açısı | Dejenere (düşük taban çizgili) track'lerin BA'yı bozması |
| `min_track_len_in_window` | `config.yaml: persistent_ba` | Bir track'in pencere içinde kaç kare hayatta kalması gerektiği | Çok kısa/güvenilmez izlerin BA'ya girmesi |
| `anchor_sigma` | `config.yaml: persistent_ba` | Her BA penceresinin ilk pozunun önceki pencereye ne kadar sıkı bağlandığı | Pencerelerin kendi içinde tutarlı ama küresel yanlış çözüme kayması |
| `force_2d` | `config.yaml: evaluation` | Her adımda Z bileşeninin sıfırlanıp XY uzunluğunun korunması | Z-eksenindeki sistematik sürüklenme |
| Komşu-kare tutarlılık kontrolü | (test edildi, entegre edilmedi) | Bir adımın yönünün son birkaç adımın ortalamasından sapması | Ani ~180°'lik "sıçrama" hataları (denendi, mimari kusur yüzünden reddedildi) |
| Pencereli R-yumuşatma | `core/pose_graph.py: refine_with_rotation_smoothing` | Ardışık adımların dönüşünün (Rodrigues vektörü) medyan filtresiyle yumuşatılması | Kare-kare rastgele dönüş gürültüsü (ama gerçek dönüşleri de bulanıklaştırıyor) |

---

## DL'ye geçersek bugünkü çalışma boşa mı gider?

**Hayır — çoğu doğrudan taşınır.** Ayrım şu:

- **Front-end'den BAĞIMSIZ (kesinlikle taşınır):** E-yolu disambiguation
  düzeltmesi (2.6) — SuperPoint'e geçsek bile E-matrisin 4 adayından
  doğrusunu seçme problemi aynen duruyor. `force_2d`, `anchor_sigma`,
  `retval` kapısı, BA parametreleri — bunlar arka-uç (BA/pose-graph/
  scale-recovery) kararları, noktaların kaynağına bakmaz. Otonom-sadece
  ölçüm metodolojisi ve çok-uçuşlu doğrulama disiplini — DL'ye geçilse
  bile aynı titizlikle doğrulama yapmak gerekecek.
- **Front-end'e ÖZGÜ (muhtemelen anlamsızlaşır):** `spatial_distribution`,
  `lowe_ratio` ayarları — SuperPoint/LightGlue'nun kendi dağılım/eşleştirme
  mantığı farklı olacağı için bu spesifik parametreler geçerliliğini
  yitirebilir.
- **En önemlisi:** bugünkü "tekrarlayan/sentetik doku → cheirality'yi
  kandıran yanlış-ama-iç-tutarlı eşleşme" teşhisi, DL'ye geçme kararının
  ASIL GEREKÇESİ oldu — yani "başarısız" denemeler boşa gitmedi, tam
  tersine bu kararı haklı çıkaran kanıtı ürettiler.

### 2.11 (30 Eylül) H-matrisi felaketi bulundu — projenin en eski sorusuna (yassı döngü) muhtemel cevap

**Tam BA ve Sim(3) rafa kaldırıldı** (bkz. 2.10) — dikkat yön hatasının
kalıntı/taban bileşenine yöneltildi. `hecheck.py`'nin (21 Eylül) AYNI
metodolojisi tekrarlandığında **medyan taban hatasının 27.5°'den 6.16°'ye
düştüğü** bulundu (E-yolu düzeltmesi, 2.6, sadece kuyruğu değil tabanı da
düzeltmiş).

**Sistematik hata-katkısı analizi (`error_attribution.py`, tüm uçuş,
matrix_type + inlier çeyreklikleri):** H-matrisi seçilen kareler E'den
**5-10 kat** daha kötü, iki uçuşta da:
```
        n_H (%pay)   H medyan    H ortalama   n_E    E medyan   E ortalama   H'nin toplam hataya payı
2026:    54 (%12)     127.21°     109.66°      394     11.74°      27.64°           %35.2
oturum_3: 29 (%6.5)    97.41°      86.33°      416      5.15°      17.19°           %25.9
```

**H'nin GT dönüş hızıyla korelasyonu (`h_selection_vs_turnrate.py`) —
projenin en eski sorusuna (yassı/uzamış döngü) muhtemel cevap:** H-seçilen
adımların ORTALAMA gerçek dönüş hızı, E-seçilenlerden 6-7 kat fazla:
```
        H ortalama donus    E ortalama donus
2026:        17.18°/adım        2.73°/adım
oturum_3:    51.46°/adım        7.03°/adım
```
Teori: sert dönüş anlarında H (yanlış şekilde) seçiliyor, H o anlarda
geometrik olarak güvenilmez, sonuç GERÇEK dönüşün sistematik olarak hafife
alınması — tam olarak 28 Eylül'den beri görülen yassı-döngü görsel kanıtı.

**"Sadece E, H hiç seçilmesin" testi — evrensel çözüm DEĞİL:**
```
                MEVCUT (H+E)         SADECE E
2026:      44.61m / 16.98°     46.27m / 12.24°   (pozisyon hafif kötü, yön daha iyi)
oturum_3:  53.50m /  5.62°     46.18m /  4.55°   (İKİSİ DE daha iyi)
2024:     166.38m / 32.90°    269.99m / 66.33°   (ÇOK KÖTÜ — ama bkz. asagidaki bug notu, 2024 sayilari BOZUK)
```
H bazı (muhtemelen gerçek düşük-paralakslı/düz) sahnelerde hâlâ gerekli —
tamamen kapatmak yerine `_HOMOGRAPHY_SCORE_RATIO_THRESHOLD` (0.45) taraması
başlatıldı (0.45→0.85), 3 uçuşta.

**2024 DÜZELTİLMİŞ (autonom_thresh=600) sonuçlar — hikaye aynı, sayılar
biraz kaydı:**
```
MEVCUT (H+E, güncel):    174.74m / 32.90°
ESKİ (E-yolu kapısız):   170.49m / 32.88°   (E-yolu düzeltmesi burada nötr, doğrulandı)
SADECE E (H kapalı):     283.56m / 66.33°   (hâlâ FELAKET)
Eşik: 0.45 iyi, 0.55 zaten kötü (231.01m), 0.65+ plato (283.56m — "sadece E" ile aynı)
```
**Kesin sonuç: 2024, H'ye GERÇEKTEN ihtiyaç duyuyor.** Üç uçuş arasında
gerçek bir çatışma var (2026/oturum_3 H bastırılsın ister, 2024 açık
kalsın ister), ve hiçbir sabit eşik (adım-fonksiyonu deseni yüzünden)
ikisini birden memnun edemiyor. **Sonraki adım: H'ye E'ye yaptığımız gibi
kendi oy-marjı/güven kontrolünü eklemek** — statik eşik yerine HER KAREDE
dinamik bir karar.

**KRİTİK METODOLOJİ HATASI BULUNDU VE DÜZELTİLDİ — 2024'ün `autonom_thresh`
değeri YANLIŞTI, tüm bugünkü 2024 sonuçları bozuk çıktı:** 2024'ün
`raw_frames` klasörü kaynakta zaten native-4-kare aralıkla seyreltilmiş
(`frame_000000, 000004, 000008...`), `frame_step: 1` bu yüzden DataLoader'a
"ek seyreltme yapma" diyor. Ama `autonom_thresh = warmup_frames × frame_step
= 150×1 = 150` formülü bunu hesaba katmıyordu — gerçek ısınma sınırı
**listelenen 150. kare** olup bu native numarada **600**'e denk geliyor
(`loader.frame_list[149]` = `frame_000596`, `[150]` = `frame_000600`),
150 değil. Doğrulama: frame_000100-000260 arası native numaralarda tahmin
ile GT'nin BİREBİR aynı çıkması (hâlâ ısınmada, "bedava sıfır hata")
farkedilerek bulundu — kullanıcının "warmup'tan sonra neden ters yöne
gitmiş" sorusu sayesinde. **Bugün 2024 üzerinde yapılan E-yolu A/B,
"sadece E" ve eşik taraması sonuçlarının HEPSİ, gerçekte hâlâ ısınmada
olan ~450 native-numaralık bedava-sıfır-hata bir dilimi yanlışlıkla
"otonom" saymıştı — düzeltilip (`autonom_thresh=600`) yeniden koşuluyor.**
Bu, 28 Eylül'deki "ısınma bedavası" dersinin yeni bir versiyonu — sadece
bu kez sebep frame numaralandırma/örnekleme aralığı uyumsuzluğuydu.

**2024 DÜZELTİLMİŞ (autonom_thresh=600) kesin sonuç:** MEVCUT 174.74m/
32.90°, ESKİ (E-yolu kapısız) 170.49m/32.88° (E-yolu düzeltmesi burada
nötr, doğrulandı), SADECE E (H kapalı) 283.56m/66.33° (hâlâ FELAKET).
Eşik taraması aynı adım-fonksiyonu deseniyle doğrulandı: 0.45 iyi,
0.55 zaten kötü (231.01m), 0.65+ "sadece E" ile aynı platoya oturuyor.
**Kesin: 2024 H'ye gerçekten ihtiyaç duyuyor, üç uçuş arasında hiçbir
sabit eşiğin çözemeyeceği gerçek bir çatışma var.**

**H'nin kendi oy-marjı testi (`h_vote_margin_check.py`) — İLK fikir
ÇÜRÜDÜ:** dünkü E-yolu mantığıyla ("H'ye de kendi güven kontrolünü
ekle") umduğumuzun aksine, H-seçilen TÜM kareler (2026: 53/53,
oturum_3: 29/29) zaten oran>=0.9 (yüksek güven) — korelasyon
hesaplanamıyor bile (varyans yok). **H'nin oylama mekaniği kendi
içinde tutarlı çalışıyor, sorun oylamanın kalitesi değil — YANLIŞ bir
varsayıma (sahne düz) dayanarak EMİNLİKLE yanlış sonuca varıyor.**
Hangi adayı seçersek seçelim, düzlemsel-olmayan bir sahnede düzlemsel
varsayımın kendisi hatalı. Yani H'ye güven eşiği eklemek işe yaramaz.

**E'nin önerdiği dönüş açısını proxy olarak kullanma testi
(`e_rotation_as_proxy_check.py`) — İKİNCİ fikir de ÇÜRÜDÜ:** H
seçildiğinde E'yi de paralel çözüp, E'nin önerdiği dönüş açısının H'nin
o kareyi ne kadar kötü çözdüğünü tahmin edip etmediğine bakıldı.
Korelasyon 2026'da neredeyse yok (+0.01, n=47 küçük-açı grubunda bile
medyan 138° hata!), oturum_3'te zayıf (+0.33, küçük-açı grubunda bile
73° hata). **E'nin önerdiği açı, H'nin başarısız olacağını güvenilir
şekilde önceden haber vermiyor.** Olası açıklama: H'nin seçildiği
sahneler zaten düşük-paralakslı — bu, E için de zor bir durum (E'nin
doğru çalışması iyi paralaks gerektiriyor), yani bu sahne TÜRÜ iki
modelin de (H VE E) tek-çift-kare (2-view) geometrisiyle zorlandığı bir
bölge olabilir.

**Sonuç — iki makul klasik fikir de temiz bir ayırt edici sinyal
vermedi.** Bu, "belki bu spesifik sorun 2-view klasik geometriyle
çözülemez, çok-kareli/öğrenilmiş yöntemler gerekir" tezini güçlendiriyor
— DL'ye (SuperPoint/LightGlue, ya da tam öğrenilmiş bir front-end)
geçme gerekçesi artık İKİ bağımsız kanıtla destekleniyor: (1) dün
bulunan tekrarlayan-doku/eşleştirme belirsizliği, (2) bugün bulunan,
2-view geometrinin düşük-paralakslı sahnelerde H/E ayrımından bağımsız
olarak zorlanması. **H/E ayrık model-seçimi mimarisinin kendisi,
öğrenilmiş/yoğun (dense) yöntemlerin doğal olarak sahip olmadığı bir
sınırlama** — DL'ye geçmenin faydası sadece "daha iyi eşleşme" değil,
bu sınıf sorunu YAPISAL olarak ortadan kaldırması.

**ÖNEMLİ BAĞLAM NOTU (kullanıcı, 30 Eylül):** proje şu anda TEKNOFEST'e
resmi başvuru için hazırlanmıyor — yarışma metriği (hizalanmamış hata)
disiplin/karşılaştırılabilirlik için kullanılmaya devam ediyor, ama
şartname uyumluluğu ya da başvuru takvimi şu an aktif bir kısıt değil.

### 2.12 (30 Eylül) 2024 görsel "aynalanma" izlenimi — H-felaketinin
kare-numaralı somut kanıtı; SuperPoint/LightGlue okundu, model kararı verildi

**Kullanıcı gözlemi:** `trajectory_2024_guncel_30eylul.png` görselinde
bazı kollarda tahmin GT'nin X-ekseninde aynalanmış gibi görünüyor —
"belki -X yerine +X kullanılıyor" hipotezi ölçüldü (`mirror_x_2024_check.py`).

**Mirror-X hipotezi ÇÜRÜDÜ:** X'i ters çevirmek hatayı KÖTÜLEŞTİRİYOR
(76.80°→122.32° ortalama). Mirror-Y karışık sinyal veriyor (ortalama
iyileşiyor 76.80°→57.68°, medyan kötüleşiyor 47.43°→60.29°) — temiz bir
eksen-aynalaması sinyali YOK.

**Asıl mekanizma — en kötü 15 tekil adıma bakınca netleşti:**
```
frame_003244  H-secildi  GT-donus=107.4°/adım  hata=179.97°
frame_004436  H-secildi  GT-donus= 83.3°/adım  hata=179.80°
frame_005140-005968 kümesi  E-secildi  GT-donus≈0.1-2.5°/adım  hata≈179.7-180°
```
İki H-seçilen kare gerçek sert dönüş anlarında (zaten bilinen H-felaketi
mekanizması, 2.11) — ama SONRASINDAKİ birçok E-seçilen kare, o anda GT
DÜZ gitmesine rağmen (dönüş hızı ~0) hâlâ ~180° hata gösteriyor. Sebep:
sistem dead-reckoning (zincirleme) — bir kere H sert dönüşte yanlış
adayı seçip global yönelimi ~180°'ye kilitleyince, kendisi tamamen
doğru olan SONRAKİ kareler bile o yanlış temel yönelim üzerinden
hesaplanıyor. 40'lık blok analizinde hata düzenli artmıyor, kesikli
zıplıyor (13°→160°→13°) — sürekli kayma değil, ayrı ayrı olaylar.

**Sonuç:** Görseldeki "aynalanma" izlenimi bir eksen-işareti hatası
DEĞİL, zaten bilinen H-felaketi olayının (2.11) dead-reckoning zinciri
boyunca kalıcı etkisinin görsel imzası. Yeni bir bug değil, mevcut
bulgunun 2024'te somut kare numaralarıyla doğrulanması — DL gerekçesini
tekrar güçlendiriyor, yeni bir düzeltme gerektirmiyor.

**SuperPoint (arXiv:1712.07629) ve LightGlue (arXiv:2306.13643)
tam okundu** (`okunacaklar.md`'ye detaylı eklendi). Kritik bulgu:
LightGlue'nun kendi makalesi InLoc testinde tekrarlayan-doku
eşleştirme hatasını itiraf ediyor ("sometimes matches repeated
objects... instead of the geometric structure") — kullanıcının
29 Eylül'den beri savunduğu nadir-açı/tekrarlayan-arka-plan
hipotezinin SOTA bir DL sisteminde bile kısmen açık kaldığının
makale-içi kanıtı; DL bu sorunu AZALTIR, tam ORTADAN KALDIRMAZ.

**Model kararı: SuperPoint + LightGlue (birincil), DISK + LightGlue
(yedek/yükseltme).** Gerekçe: ikisi de `cvg/LightGlue` deposunda tek
API altında geliyor, `feature_extractor.py` zaten bu swap'ı bekleyecek
şekilde tasarlanmıştı. DISK+LightGlue makalede daha güçlü sonuç
veriyor (IMC 2021'de +6-8% AUC, geniş-açı değişiminde daha dirençli)
— eğer SuperPoint sonrası tekrarlayan-doku sorunu hâlâ belirgin
kalırsa, SADECE detektörü DISK'e değiştirip (LightGlue sabit)
tek-değişken karşılaştırması yapılacak.

**Hibrit (sadece sert dönüşte DL kullan) fikri değerlendirildi —
şimdilik ertelendi:** Güvenilir, ucuz bir "sert dönüş oluyor" tetikleyici
sinyali GEREKİYOR ama böyle bir sinyal zaten bugün test edilip
çürütülmüştü (E'nin önerdiği açı proxy'si, yukarıda). SuperPoint+LightGlue
zaten hafif/gerçek-zamanlı olduğu için (~13-44ms) hesaplama kısıtı
olmadıkça tam swap, güvenilmez bir tetikleyici gerektiren hibrit
mimariden daha basit ve daha az riskli.

**DL kurulum/test planı (1 Ekim):** CUDA kurulumu → SuperPoint
entegrasyonu (`feature_extractor.py`, arayüz değişmeden) → eşleştirici
başta Hamming→L2/kosinüs (LightGlue opsiyonel ikinci adım) → 3 uçuşta
(2026, oturum_3, 2024) H/E dağılımı ve yön hatası yeniden ölçülecek,
özellikle sert-dönüş karelerindeki hata ayrı raporlanacak.

### 2.13 (30 Eylül) oturum_3 "düşük yön hatası, yüksek pozisyon hatası"
paradoksu çözüldü — Z-sürüklenmesinin somut payı ölçüldü

**Kullanıcı gözlemi:** oturum_3'ün yön hatası en düşük (5.62°) olmasına
rağmen, pozisyon hatası 2026'dan (44.61m) daha yüksek (53.50m) —
yorumlanması istendi.

**Ölçüm 1 — yol uzunluğu farkı değil:** 2026=726.9m, oturum_3=766.9m
(sadece %5.5 fark, 2250 kare ikisinde de) — "daha uzun uçuş" açıklaması
elenir.

**Ölçüm 2 — GT irtifa (Z) aralığı çok farklı:** 2026 Z aralığı 30.2m
(std 7.5m), oturum_3 **47.8m (std 10.8m)**.

**Ölçüm 3 — pozisyon hatasının yatay(XY)/dikey(Z) ayrımı
(`horiz_vert_split.py`, otonom-sadece):**
```
              3D hata   Yatay(XY) hata   Dikey(Z) hata   Z payı
2026:         33.77m       31.86m           5.96m         %17.6
oturum_3:     38.22m       22.01m          30.11m         %78.8
```
(Not: bu script basitleştirilmiş `autonom_thresh=150` kullandı, bu
yüzden mutlak sayılar üretim tablosundaki 44.61m/53.50m'den farklı —
ama YATAY/DİKEY ORANI, yani asıl bulgu, sağlam.)

**Sonuç:** Paradoks yok — oturum_3'ün YATAY (yön-hatasıyla ilişkili)
tahmini 2026'dan gerçekten DAHA İYİ, tam düşük yön hatasıyla (5.62°)
tutarlı. Ama `force_2d` kendi Z tahminimizi bastırdığı için, GT'nin
(oturum_3'te çok daha fazla değişen) irtifası neredeyse olduğu gibi 3D
hataya giriyor. Heading metriği (X-Y açısı) bunu hiç yakalamıyor. Bu,
Z-sürüklenmesi probleminin bazı uçuşlarda toplam hatanın BASKIN kaynağı
olabileceğinin ilk somut, sayısal kanıtı.

**DÜZELTME — doğru eşiklerle yeniden koşturulunca (`autonom_thresh`:
2026=750, oturum_3=750, 2024=600, üçü de AYNI script/formülle, tam 3D):**
```
                3D hata    Yatay(XY)   Dikey(Z)   Z payı
2026:           47.28m      44.61m       8.34m     %17.6
oturum_3:       53.50m      30.82m      42.15m     %78.8
2024:          174.74m     174.74m       0.00m     %0.0  (bu ucuşta GT Z verisi hic yok)
```

**METODOLOJİ UYARISI (kullanıcı fark etti):** Önceki "resmi" tabloda
2026 için verilen 44.61m, yukarıdaki tam-3D sonucuna (47.28m) değil,
bu testin YATAY (44.61m) bileşenine virgülden sonra 2 haneye kadar
BİREBİR eşit çıkıyor — oysa oturum_3'ün "resmi" 53.50m'si tam-3D
toplamına eşit. Bu, iki uçuşun "resmi" sayılarının GEÇMİŞTE FARKLI
formüllerle (biri XY-only, biri tam-3D) hesaplanmış olabileceğine işaret
ediyor; kaynak script bu oturumda doğrulanamadı. **Bundan sonra yukarıdaki
DÜZELTME tablosu (üçü de aynı script/tam-3D/doğru eşik) referans
alınmalı**, eski 44.61m/53.50m/174.74m tablosu değil (2024 ve oturum_3
zaten aynı çıktığı için onlarda sorun yok, sadece 2026'nın eski sayısı
şüpheli).

### 2.14 (3 Ekim) SuperPoint entegre edildi; ilk (parametre-taraması
ÖNCESİ, ham) 3-uçuş ölçümü — karışık sonuç, H-felaketi sıklığı neredeyse
sıfıra indi ama 2024 bundan zarar gördü

**Kurulum:** CUDA'lı PyTorch (`torch 2.6.0+cu124`, GTX 1650) + resmi
`cvg/LightGlue` paketi (SuperPoint+LightGlue+DISK tek API'de) kuruldu.
`core/feature_extractor.py`'ye `detector_type` config anahtarıyla
dallanan SuperPoint entegrasyonu eklendi (`FrameFeatures` arayüzü
AYNEN kaldı — `cv2.KeyPoint` listesi + float32 descriptor array,
semantic mask post-hoc filtre olarak uygulanıyor). `core/matcher.py`'ye
`descriptor_norm` (hamming/l2) config anahtarı eklendi. Üretim
`config.yaml` DEĞİŞMEDİ (`detector_type: orb` varsayılan) — test için
ayrı `config_superpoint_test.yaml`. Ağırlıklar `torch.hub` ile otomatik
indirilip önbelleğe alınıyor, ayrı `weights/` klasörü gerekmedi.
Duman testi (6 kare): ~2900 keypoint/kare, eşleşme sayısı ORB'un ÇOK
üzerinde (~2300-2500 vs ORB'un tipik yüz mertebesi).

**3 uçuşta ham ölçüm (parametreler HENÜZ yeniden taranmadı — tüm
eşikler, RANSAC ayarları, lowe_ratio hâlâ ORB için ayarlanmış
değerlerde):**
```
                    3D hata     Yön(medyan)   H secimi    Sert-donus(yon medyan)
2026 (ORB, eski)    47.28m      16.98°        %12.0         -
2026 (SuperPoint)   57.77m      11.74°        %0.0        39.99°

oturum_3 (ORB)      53.50m       5.62°        %6.5          -
oturum_3 (SP)       39.29m       5.93°        %0.0        28.46°

2024 (ORB)         174.74m      32.90°         (var)        -
2024 (SuperPoint)  170.91m      54.73°        %1.2        87.21°
```

**En çarpıcı bulgu: H neredeyse hiç seçilmiyor artık** (2026/oturum_3
%0, 2024 sadece %1.2 — ORB'da %6-12 civarıydı). SuperPoint'in çok daha
yoğun/düzgün dağılmış noktaları, RANSAC'ın E skorunu neredeyse her
zaman H'den yüksek çıkarmasına sebep oluyor gibi görünüyor — H-
felaketinin (2.11/2.12) SIKLIĞI dramatik şekilde azaldı.

**Bunun iki yüzü var:**
- **İyi (oturum_3):** pozisyon hatası -%27 (53.50→39.29m) — oturum_3
  zaten H'yi bastırmak istiyordu (§2.11), DL bunu otomatik yapmış.
- **Kötü (2024):** yön hatası NEREDEYSE 2 KATINA çıktı (32.90°→54.73°)
  — çünkü 2024 GERÇEKTEN H'ye ihtiyaç duyuyordu (§2.11: "sadece-E"
  testinde 2024 felaketti, 283.56m/66.33°). H otomatik bastırılınca,
  2024 o felakete yaklaşmış — **3-uçuş çatışması (2.11) ortadan
  kalkmadı, sadece "varsayılan davranış H'yi bastırma yönüne kaymış"
  hale geldi.**
- **Karışık (2026):** medyan yön hatası iyileşti (16.98°→11.74°) ama
  pozisyon hatası kötüleşti (+%22, 47.28→57.77m) — sebep henüz
  bilinmiyor, muhtemelen ölçek/büyüklük tarafında (parametreler
  yeniden taranınca netleşebilir).

**Sert-dönüş kareleri (GT dönüş≥15°/adım) HÂLÂ çok kötü** (2024'te 87°
medyan!) — beklendiği gibi, DL sihirli değnek değil, 2-view geometrinin
gerçek sert dönüşlerdeki temel zorluğu devam ediyor.

**Sonuç — net bir "DL kazandı/kaybetti" iddiası HENÜZ yapılamaz:** bu
ölçüm, `_HOMOGRAPHY_SCORE_RATIO_THRESHOLD`, RANSAC eşiği, lowe_ratio/L2
eşdeğeri gibi TÜM parametreler hâlâ ORB'a göre ayarlanmışken alındı —
planın (2.12) ikinci adımı (parametre taraması) henüz yapılmadı. Sıradaki
adım: bu parametreleri SuperPoint'in istatistiklerine göre sıfırdan tara.

**`_HOMOGRAPHY_SCORE_RATIO_THRESHOLD` taraması (3 Ekim, SuperPoint,
onbelleklenmis match+ucuz replay yontemiyle, 9 deger x 3 ucus):**
```
              esik=0.20        esik=0.30         esik=0.45(eski)    esik>=0.50
2026:      51.32m/25.11° H%14   57.77m/11.74° H%0   57.77m/11.74° H%0   57.77m/11.74° H%0
oturum_3:  52.24m/ 7.95° H%13   39.29m/ 5.93° H%0   39.29m/ 5.93° H%0   39.29m/ 5.93° H%0
2024:      63.07m/11.65° H%99   107.37m/12.85°H%61   170.91m/54.73°H%1   189.04m/55.51°H%0
```
**Çatışma ÇÖZÜLMEDİ, DAHA DA KESKİNLEŞTİ.** 2024 için esik=0.20
muazzam bir kazanç (170.91m→63.07m, -%63; yön 54.73°→11.65°, -%79) —
SuperPoint ile 2024'ün H ihtiyacı hafiflemedi, KESİNLEŞTİ (artık
neredeyse HER karede H istiyor, %99.3). Ama AYNI esik 2026/oturum_3'u
belirgin kötülestiriyor (ikisi de esik>=0.30 istiyor, H'yi tamamen
kapatarak). **ORB döneminin "adım fonksiyonu, evrensel deger yok"
sonucu (2.11) SuperPoint'le de GEÇERLİ, hatta farklar büyüdü** — tek bir
sabit esik üç ucusu birden memnun edemiyor. Olası sonraki yollar: (a)
her ucus icin kendi esigini config'te sabitlemek (pratik ama
"genellenebilir" degil, zaten bilinen bir odun), (b) adaptif/per-frame
bir sinyal (H'nin kendi guveni, E'nin acisi gibi fikirler ORB'da
cururmustu -- SuperPoint'in cok farkli eslesme istatistikleriyle
YENIDEN denenmeye deger olabilir), (c) 2024'un GERCEKTEN neden bu kadar
H-bagimli oldugunu arastirmak (hipotez: duz/az-relief arazi ya da
dusuk-paralaks ucus profili, henuz dogrudan olculmedi).

**Pratik çözüm uygulandı (3 Ekim, kullanıcı kararı): her uçuşun kendi
eşiğini config'te sabitle.** `_HOMOGRAPHY_SCORE_RATIO_THRESHOLD` artık
`core/motion_estimator.py`'de sabit (hardcoded) DEĞİL — `config.yaml`'ın
`features.homography_score_ratio_threshold` anahtarından okunuyor,
varsayılan 0.45 (ORB production davranışı DEĞİŞMEDİ, doğrulandı).
Yeni SuperPoint test config'leri, flight-özel tarama sonucuna göre:
`config_superpoint_test.yaml` (2026) → 0.30, yeni
`config_2025_oturum3_superpoint_test.yaml` → 0.30, yeni
`config_2024_superpoint_test.yaml` → 0.20. Üçü de onbelleklenmiş
match-cache'lerle (`matchcache_*.pkl`) doğrulandı — config'ten okunan
değer tarama scriptindeki override ile birebir eşleşiyor.

### 2.15 (3 Ekim) `lowe_ratio` taraması (L2, SuperPoint) — H/E eşiğinin
aksine ÜÇ uçuşu da aynı anda iyileştiren ortak bir değer bulundu

Pahalı kısım (SuperPoint çıkarımı) her uçuş için bir kere yapılıp
(`featcache_*.pkl`) önbelleklendi, ucuz kısım (eşleştirme+tahmin+ölçek+
poz-grafiği) her `lowe_ratio` değeri için yeniden koşuldu (9 değer,
0.50-0.95).

```
                 eski (0.75)        yeni (0.70)
2026:            57.66m/11.74°  →   51.07m/10.10°   (-%11)
oturum_3:        39.29m/ 5.93°  →   35.70m/ 6.38°   (-%9, yön hafif kötü, ihmal edilebilir)
2024:            63.07m/11.65°  →   50.71m/ 9.83°   (-%20)
```

**H/E eşiğinin (2.14) aksine burada gerçek bir ÇATIŞMA YOK** — `0.70`
üç uçuşta da aynı anda iyileşme veriyor (H/E eşiğinde her uçuş zıt yön
istiyordu, burada hepsi aynı yönde). Tarama tam monoton değil (gürültülü,
örn. 2026'da 0.85 daha da iyi yön hatası veriyor — 8.01°) ama 0.70,
üçü için de tutarlı şekilde iyi bir ortak nokta. Üç SuperPoint test
config'i de `lowe_ratio: 0.70` olarak güncellendi.

### 2.16 (3 Ekim) `ransac_threshold` taraması — pozisyon/yön ödünleşimi
VE gerçek bir sağlamlık açığı bulundu (SVD yakınsamıyor)

2026/oturum_3'te eşik arttıkça (1.0→4.0) pozisyon hatası iyileşiyor ama
yön hatası sürekli kötüleşiyor, 5.0'da İKİSİ DE felakete dönüşüyor
(2026: 108m/133°, oturum_3: 79m/26-46°) — RANSAC artık gerçek-dışı
eşleşmeleri inlier kabul etmeye başlıyor. Şu anki değer (3.0) bu
felaketten güvenli ama optimal değil.

**Gerçek kod hatası bulundu (tarama sırasında, 2024'te):**
`ransac_threshold=1.0`'da `core/motion_estimator.py`'deki
`_decompose_homography`'nin içindeki `np.linalg.svd(A)` çağrısı
`LinAlgError: SVD did not converge` ile çöküyor — çok sıkı eşikte çok
az inlier kalınca DLT matrisi (A) dejenere/ill-conditioned hale geliyor.
Bu, production'da şu ana kadar hiç ortaya çıkmamıştı çünkü eşik hiç
3.0'ın altına inmemişti. **Kod şu an bu hatayı yakalamıyor (try/except
yok)** — düşük `ransac_threshold` denemelerinde gerçek bir çökme riski.
Henüz düzeltilmedi (kullanıcıya haber verildi, ayrı bir karar
gerektiriyor: ya SVD'yi try/except'e al, ya da bu kadar düşük eşikleri
hiç denemeyeceğimize karar ver).

**2024 tam sonucu (1.0 haric, cokme bugu yuzunden atlandi):**
```
esik=1.5   63.60m/ 5.34°  (en iyi yon)
esik=2.0   65.39m/ 7.90°
esik=2.5   65.80m/ 8.23°
esik=3.0   50.71m/ 9.83°  (su anki)
esik=3.5   51.88m/10.74°
esik=4.0   49.66m/10.45°
esik=5.0   45.45m/11.70°  (en iyi pozisyon)
esik=6.0  135.57m/84.23°  (FELAKET -- 2026/oturum_3'un 5.0'daki
                            felaketiyle ayni desen, sadece esigi 6.0)
```

**KARAR: `ransac_threshold` SU ANKI degerinde (3.0) birakildi,
degistirilmedi.** Hicbir alternatif deger uc ucusta BIRDEN net
ustunluk saglamiyor (hep pozisyon/yon odunlesimi var), dusuk degerler
SVD-cokme riski tasiyor, yuksek degerler (>=5.0/6.0) felakete cok
yakin. "Zaten dengeli, degismiyor" da gecerli bir tarama sonucu.

### 2.17 (3-4 Ekim) LightGlue entegre edildi + 3 ic parametresi (3
ucusta) tam tarandi — mühendislik destanı (bellek sorunu) + karışık
bilimsel sonuç

**Entegrasyon:** `core/matcher.py`'ye `matcher_type: lightglue` dallanması
eklendi (resmi `cvg/LightGlue` paketi, `features="superpoint"`).
`FrameFeatures`'tan (keypoints+descriptors+clean_frame.shape) LightGlue'nun
gerektirdiği (keypoints+descriptors+image_size) torch dict'i YENİDEN
İNŞA EDİLİYOR — hiçbir yeni alan eklenmeden (arayüz aynen korundu). Yeni
test config'leri: `config_superpoint_lightglue_test.yaml` (+ oturum_3/2024
eşdeğerleri).

**Mühendislik yan-hikayesi (gelecekte tekrar karşılaşılabilir, not
düşülüyor):** 45 kombinasyonluk (`filter_threshold`×6, `depth_confidence`×5,
`width_confidence`×4, ×3 uçuş) tam tarama üç kez bellek yüzünden
çöktü — sırasıyla (a) bilgisayarın uyku moduna geçmesi (süreç donuyor,
kod hatası değil), (b) her parametre denemesinde `featcache_2024.pkl`'ın
(6.2GB!) DİSKTEN YENİDEN yüklenmesi, (c) LightGlue modelinin her denemede
YENİDEN GPU'ya yüklenmesi. Kalıcı çözüm: model SADECE BİR KERE kurulup
`.conf.X = deger` ile ayarları runtime'da değiştirildi (doğrulandı:
`lightglue.py` bu 3 parametreyi HER forward()'da `self.conf`'tan taze
okuyor). 2024'ün dev önbelleği de kare-başına küçük `.npz` dosyalarına
bölünüp AKIŞ-TABANLI (bellekte aynı anda sadece 1 kare) okunacak şekilde
yeniden yapılandırıldı — boş bellek 0.04GB'tan 6+GB'a çıktı, sorun
kökünden çözüldü.

**TAM sonuçlar (3 uçuş, 3 parametre, tümü varsayılana göre tek-tek
taranmış):**
```
2026:              filter_th    depth_conf      width_conf
en iyi poz:        0.01→51.79m  0.80→58.61m     0.95→56.08m
en iyi yon:         varsay→7.53° varsay→7.53°    varsay→7.53°
(varsayilan: 64.97m/7.53°)

oturum_3:          filter_th    depth_conf      width_conf
en iyi poz:        0.20→35.16m   -1→35.78m      0.95→32.46m (EN IYI GENEL)
en iyi yon:        0.01→5.00°    -1→4.12° (EN IYI GENEL)  varsay→6.35°
(varsayilan: 38.36m/6.35°)

2024:              filter_th    depth_conf      width_conf
en iyi poz:        0.30→51.34m  varsay→49.74m   varsay→49.74m
en iyi yon:        0.30→10.99°   -1→9.23°       varsay→11.25°
(varsayilan: 49.74m/11.25°)
**UYARI: depth_confidence=0.99 VE width_confidence=-1, 2024'TE TAM
FELAKET** (163.90m/107.15° ve 158.26m/116.55°) — 2026/oturum_3'te
hic gorulmeyen, 2024'e OZGU yeni bir kirilganlik. Bu iki deger 2024
icin kesinlikle KULLANILMAMALI.
```

**Yorum:**
- **oturum_3 — net, cifte kazanc:** `depth_confidence=-1` (erken-cikisi
  tamamen kapat) hem pozisyonu hem yonu ORB'un (53.50m/5.62°) bile
  ONUNE geciriyor (35.78m/4.12°) — odunlesimsiz, temiz bir kazanc.
- **2026 — hala odunlesimli:** Hicbir ayar hem pozisyonu hem yonu
  SuperPoint+klasik sonucundan (51.07m/10.10°) birden iyilestirmiyor.
- **2024 — LightGlue varsayilani zaten iyi, ekstra kazanc yok ama
  risk var:** En iyi bulunan deger (varsayilan, 49.74m/11.25°)
  SuperPoint+klasik'e (50.71m/9.83°) kabaca esit — net kazanc yok,
  AMA bazi "agresif" ayarlar (0.99/-1) felakete yol aciyor, bu riskin
  bilinmesi onemli.
- **Genel karar:** Universal "en iyi LightGlue ayari" yok (projenin
  tekrarlayan temasi) — ama `depth_confidence=0.99` ve
  `width_confidence=-1`'in 2024'te KESIN YASAKLI oldugu NET.

### 2.18 (4 Ekim) BA, DL ön-uçle (SuperPoint+LightGlue) yeniden test
edildi — HÂLÂ yardımcı olmuyor, hipotez doğrulanmadı

**Yöntem:** Windows'ta SuperPoint+LightGlue (varsayılan ayarlar) ile
online pipeline koşulup `pose_graph._local_steps` WSL'e aktarıldı (BA'nın
kendi KLT track-building'i -- `_build_full_flight_tracks` -- tamamen
klasik optik akış, SuperPoint/LightGlue'dan BAĞIMSIZ; WSL'de DL kurulumuna
hiç gerek kalmadı). 2026 uçuşunda hem `refine_with_persistent_map`
(pencereli) hem `refine_with_full_ba` (tam) denendi.

```
ONLINE (SuperPoint+LightGlue, BA'siz):     64.97m / 7.53°
PENCERELİ BA (üstüne):                     70.31m / 7.09°   (poz +%8 kötü, yön hafif iyi)
TAM BA (üstüne):                           68.72m / 10.77°  (poz +%6 kötü, yön +%43 KÖTÜ)
```

**Sonuç: "DL ön-uç olgunlaşınca BA tekrar işe yarar" hipotezi
DOĞRULANMADI** (en azından 2026'da, LightGlue varsayılan ayarlarıyla).
BA hem pozisyonu hem (tam BA'da) yönü KÖTÜLEŞTİRDİ — ORB döneminin
"BA artık yardımcı olmuyor" sonucu (2.7) DL sonrasında da geçerli.
Spekülasyon değil, gerçek ölçüm. BA, bu projede front-end kalitesi
ne olursa olsun tutarlı şekilde iyileştirici olmayı başaramadı.

### 2.19 (4 Ekim) Loop closure DL (SuperPoint+LightGlue) ile yeniden
test edildi — aday tespiti gerçekten iyileşti, AMA sonuç yine hiç
değişmedi; Sim(3) eksikliği hipotezi DOĞRULANDI

**Yöntem:** `_detect_loop_closures` GTSAM gerektirmiyor (sadece
feature_extractor+matcher+motion_estimator) — Windows'ta DOĞRUDAN
SuperPoint+LightGlue ile çalıştırılıp loop adayları (`i`,`j`,`R`,
`inlier_count`,`pts_i`,`pts_j`) pickle'landı. WSL'de `local_steps` +
bu adaylar yüklenip `refine_with_loop_closure`'ın geri kalan mantığı
(KLT backbone + `_apply_loop_closure_pgo`, ikisi de DL'den bağımsız)
elle çalıştırıldı.

```
ONLINE (SuperPoint+LightGlue, refine yok):     64.97m / 7.53°
PENCERELİ BA backbone (loop'suz):              70.31m / 7.09°
BA + LOOP CLOSURE (66 DL-adayıyla):            70.31m / 7.09°   <- BİREBİR AYNI
```

**66 loop adayı bulundu** (klasik ORB-dönemi denemelerinde 14-61
arasıydı) — DL, aday SAYISINI/KALİTESİNİ gerçekten artırdı. AMA PGO
adımı sonucu **hiçbir ondalık basamakta bile değiştirmedi.**

**Sonuç — Sim(3) hipotezi kesin doğrulandı:** Loop closure'ın sorunu
"iyi aday bulamıyoruz" DEĞİLMİŞ — DL çok daha fazla/güvenilir aday
buldu ama optimizasyon sonucu sıfır etkilendi. Kök neden, zaten
şüphelendiğimiz gibi, `_apply_loop_closure_pgo`'nun SADECE rijit
(`BetweenFactorPose3`, SE(3)) kısıtlar kullanması — ölçek-esnek
(`Similarity3`, Sim(3)) kısıt hiç implement edilmemiş, bu yüzden
kaç/ne kadar iyi aday bulunursa bulunsun optimizasyona kaldıraç gücü
veremiyor. **Bu, DL'nin ÇÖZEMEYECEĞİ, front-end'den tamamen bağımsız
bir mühendislik eksiği olduğunu kesinleştiriyor** — Sim(3) implement
edilmeden loop closure hiçbir front-end'le (klasik ya da DL) işe
yaramayacak.

### 2.20 (4 Ekim) Sim(3)-ruhlu ölçek düzeltmesi — BAŞLATILDI, YARIM
KALDI, DEVAM EDİLECEK (kapatılmadı)

**Yaklaşım (GTSAM'in gerçek `Similarity3`'ü yerine, API riski düşük
bir alternatif):** Her loop closure adayı için, (k,i,j) üç-görünümlü
üçgenlenen noktaların yeniden-izdüşüm hatasını minimize eden TEK bir
ölçek çarpanı aranıyor, bulununca SADECE o loop'un [i,j) segmentine
uygulanıyor — rijit kısıtların yapamadığı "bu segment aslında %X
daha uzun/kısaymış" düzeltmesini hedefliyor. Script: `data/
sim3_scale_correction_wsl.py`.

**1. deneme — gerçek bir kod hatası, bilimsel sonuç DEĞİL:** Çakışan
loop segmentlerine (örn. aynı `i=190` ile başlayan 7 farklı loop)
ayrı ayrı ölçek uygulanması ÇARPIMSAL bozulmaya yol açtı, üçgenleme
bazı ölçek değerlerinde dejenere olup saçma büyük (milyonlar/milyarlar)
hata üretti. Sonuç: 3062.70m (rijit'in 70.31m'sinden **43 kat kötü**)
— GEÇERSİZ, kaydedilmedi.

**2. deneme — düzeltmeler uygulandı, sonuç artık SAĞLAM ama
SONUÇSUZ:** (a) çakışan loop'lar elendi (66→5 çakışmayan), (b)
üçgenlenen noktanın HER ÜÇ görünümde de kameranın önünde (pozitif
derinlik) olması şart koşuldu, (c) tek nokta hatası 200px'de
kırpıldı (robust clip), (d) arama aralığı [0.3,3.0]'dan [0.7,1.5]'e
daraltıldı. Sonuç: hata değerleri artık makul (40000, 36592 gibi,
milyonlar değil) ama 4 test edilebilir loop'un **HİÇBİRİNDE** anlamlı
iyileşme bulunamadı (70.31m değişmeden kaldı) — ne doğrulayan ne
çürüten, VERİ YETERSİZLİĞİNDEN sonuçsuz bir deneme.

**AÇIK, DEVAM EDİLECEK — kapatılmadı.** Olası sonraki adımlar (henüz
denenmedi): (1) çakışan loop'ları TAMAMEN elemek yerine ORTAK bir
ölçek değişkeninde BİRLEŞTİRMEK (hepsini aynı anda, tek bir ortak
ölçek için optimize etmek — daha fazla veri kullanır), (2) 200px
kırpma eşiğinin çok sıkı olup olmadığını kontrol etmek, (3) GTSAM'in
gerçek `Similarity3`/`BetweenFactorSimilarity3`'ünü (daha riskli ama
"doğru" çözüm) zaman ayırıp düzgün denemek, (4) 2024/oturum_3'te de
tekrarlamak (2026'ya özgü bir sonuç olmayabilir).

---

## 3. Güncel Açık Problemler (30 Eylül, en son durum)

### 🔴 Yön (heading) sürüklenmesi — kaynağı artık NET, ama çözümü hâlâ bulunamadı
- Zayıf-kanıt bileşeni → **çözüldü** (E-yolu kapıları, 2.6).
- Taban/sistematik bileşen → **büyük ölçüde çözüldü** (2.11): E-yolu
  düzeltmesi taban hatayı da 27.5°→6.16° medyana indirmiş.
- **Kalan asıl kaynak artık biliniyor (2.11): H-matrisi, drone'un
  gerçekten hızlı döndüğü anlarda orantısız seçiliyor (H-seçilen
  adımların ortalama gerçek dönüş hızı E'den 6-7 kat fazla) ve o
  anlarda 5-10 kat daha kötü tahmin veriyor (H toplam hatanın
  %26-35'inden sorumlu, sadece %6-12 kareyle).**
- Uzamsal kümelenme bileşeni → **denendi, güvenli çözüm bulunamadı,
  KAPALI tutuluyor** (2.8).

> **Denendi, İKİSİ DE ÇÜRÜDÜ (30 Eylül, 2.11):**
> - H'ye kendi oy-marjı/güven eşiği eklemek — H'nin oylaması zaten
>   HER ZAMAN yüksek güvenli (varyans yok), ayırt edici değil.
> - E'nin önerdiği dönüş açısını proxy olarak kullanmak — H'nin
>   başarısız olacağını güvenilir şekilde önceden haber vermiyor.
>
> **Kalan olası çözüm yolları:**
> 1. Statik `_HOMOGRAPHY_SCORE_RATIO_THRESHOLD` taraması denendi —
>    adım-fonksiyonu gibi davranıyor, 3 uçuşu birden memnun eden bir
>    değer yok (2026/oturum_3 H bastırılsın ister, 2024 açık kalsın
>    ister) — kapatıldı, tek başına yeterli değil.
> 2. Komşu-kare tutarlılık kontrolü (gating) — zaten denenip mimari
>    kusur yüzünden reddedildi (2.9), ama "reddet ve TAHMİN ET"
>    versiyonu henüz denenmedi.
> 3. **En güçlü aday artık DL front-end (SuperPoint/LightGlue) ya da
>    tam öğrenilmiş bir SLAM sistemi** — H/E ayrık model-seçimi
>    mimarisinin kendisi bu sorunun kaynağı, öğrenilmiş/yoğun
>    yöntemlerde bu sınıf sorun yapısal olarak yok.
>
> **2024'te kare-numaralı somut doğrulama (2.12):** H-felaketi
> olayının (frame_003244, frame_004436 — gerçek sert dönüşlerde H
> seçimi) dead-reckoning zinciri boyunca ~180° yönelim kilidi
> yaratıp SONRAKİ düz-giden karelere bile yayıldığı gösterildi —
> kullanıcının görselde fark ettiği "aynalanma" izleniminin kaynağı.
> Model kararı verildi: SuperPoint+LightGlue (yedek: DISK+LightGlue),
> kurulum/test planı 1 Ekim.

### 🟢 E-yolu cheirality/reprojeksiyon oylaması — çözüldü, artık düşük öncelik
Dün bulunan bu sorun (2.6) entegre edildi ve taban yön hatasını da
büyük ölçüde çözdü (yukarıya bakın). H'nin sorunu ise FARKLI bir
mekanizma (2.11) — E'nin eski "zayıf oylama" sorunuyla karıştırılmamalı.

### 🔴 Z-sürüklenmesinin kök nedeni hâlâ bilinmiyor
`force_2d` ile etkisi bastırıldı (kalıcı çözüm) ama NEDEN oluyor
bilmiyoruz. **Somut payı artık ölçüldü (2.13):** oturum_3'te toplam
3D pozisyon hatasının %78.8'i sadece Z'den geliyor (2026'da %17.6) —
Z-sürüklenmesi bazı uçuşlarda baskın hata kaynağı olabiliyor, uçuşun
kendi irtifa profiline (GT Z aralığı) bağlı.

> **Olası çözüm yolları:** rolling shutter kontrolü; sahne-korelasyonu
> testi; farklı bir front-end (SuperPoint/LightGlue) ile izolasyon.

### 🟡 Loop closure hâlâ çalışmıyor, zemin belirsiz
E-yolu düzeltmesiyle yeniden test edildi, hâlâ üretimi geçemiyor. Ama
BA'nın kendisi de artık üretimi geçmiyor (bkz. altta) — loop closure'ın
"gerçekten" işe yarayıp yaramadığı net değil, çünkü üzerine inşa
edildiği BA'nın zemini kaymış durumda. Sim(3) eksikliği (rijit SE(3)
kullanıyoruz) henüz denenmedi.

> **Olası çözüm yolları:** Sim(3) tabanlı loop kısıtı dene; önce BA'nın
> genel durumunu netleştir (aşağıdaki madde).

### 🟡 Pencereli BA artık net katkı sağlamıyor, VE oturum_3'te tutarsız ölçüm var
2026'da E-yolu sonrası BA hiçbir parametre kombinasyonunda üretimi
geçemedi. oturum_3'te iki farklı script çalıştırması çelişen üretim
sayıları verdi (53.29m vs 37.68m) — RANSAC değil (test edildi,
deterministik), kaynağı hâlâ bulunamadı.

> **Olası çözüm yolları:** oturum_3 tutarsızlığını temiz bir tekil
> script'le tekrar ölç (hangi kod farkı sorumlu, bul); BA'yı production
> pipeline'dan tamamen çıkarmayı değerlendir (basitleştirme).

### 🟡 `anchor_sigma` evrensel değil
Düşük öncelik, `1e-6`'da kalındı (bkz. 1.7, 1.10) — hiçbir uçuşta
felaket yaratmıyor, "en iyi" değil ama "en güvenli".

### 🟢 Genellenebilirlik — artık 3 RGB uçuşta karşılaştırma var (2026, oturum_3, 2024)
Ama her teknik ayrı ayrı, tutarlı 4-uçuşluk bir taban çizgisi tablosu
henüz güncel (E-yolu sonrası) metodolojiyle çıkarılmadı.

> **Olası çözüm yolları:** 4 uçuşun (termal hariç ya da dahil, karar
> ver) güncel, tutarlı bir taban çizgisi tablosunu çıkar.

---

## Notlar

- **Otonom-sadece ölçüm standart** — hiçbir karşılaştırma tüm-uçuş
  metriğiyle yapılmamalı (bkz. 1.9).
- **Resmi/varsayılan ayarlar (29 Eylül itibarıyla):** `force_2d: true`
  (2026, oturum_3 için false çünkü GT'de gerçek Z var), `anchor_sigma:
  1e-6`, `spatial_distribution: false`, E-yolu disambiguation kapıları
  aktif (kod, config bayrağı yok — her zaman çalışıyor).
- **Windows/WSL ayrımı:** GTSAM (BA, loop closure) sadece WSL'de
  çalışıyor. `core/*.py` iki ortamda da (bir Windows kopyası, bir WSL
  native kopyası `/root/drone_semantic_slam`) senkron tutulmalı.
- TEKNOFEST 2025'te 7. ulusal derece — dış doğrulama olarak unutma.
