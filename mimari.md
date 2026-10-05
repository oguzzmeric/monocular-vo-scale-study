# Mimari — Sistemin Tam Teknik Haritası

Bu dosya, `drone_semantic_slam` projesinin **tamamını**, en ince ayrıntısına kadar çiziyor ve her kutuda kullanılan **her teknolojiyi** ayrı ayrı açıklıyor. `SORULAR_VE_CEVAPLAR.md`'deki diyagram bunun özetlenmiş/sadeleştirilmiş hali; burada sadeleştirme yok.

Detaylı problem geçmişi ve denenen/reddedilen yöntemler için `problemler.md`'ye bakın — bu dosya SADECE mevcut, güncel mimariyi anlatır.

---

## 0. Bugünkü (29 Eylül) Kazanç Özeti

Bugünün TEK kalıcı, sayısal olarak doğrulanmış kazancı — **E-yolu (Essential
matrix) disambiguation kapıları** (bkz. §1, MotionEstimator kutusu,
TEKNOLOJİ 7):

```
                    E-yolu kapısız (eski)   E-yolu kapılı (yeni, güncel)
2026 otonom-sadece:      53.90 m                 47.28 m   (-%12.3)
oturum_3 otonom-sadece:  61.39 m                 53.29 m   (-%13.2)
```

İki uçuşta da AYNI yönde, benzer büyüklükte — bugüne kadarki diğer
denemelerin (R-yumuşatma, retval kapısı, spatial_distribution, komşu-
tutarlılık kontrolü) hepsi flight-bağımlı çeliştiği ya da mimari kusur
yüzünden reddedildiği için, bu tek başına ayrıksı bir güvenilir kazanç.

**İki günlük (28-29 Eylül) toplam resim:** `force_2d` (28 Eylül, Z-
sürüklenmesi düzeltmesi) + bugünkü E-yolu düzeltmesi birlikte,
2026 uçuşunda otonom-sadece hatayı 93.68m'den ~47m'ye indirdi (**~%50
toplam iyileşme**). Diğer tüm bugünkü denemeler (aşağıda listelenen çoğu)
ya "zaten optimal, değişiklik yok" ya da "denendi, reddedildi" sonucuyla
kapandı — sıfır ek sayısal kazanç ama değerli teşhis/eleme bilgisi
ürettiler (detay: `problemler.md`).

**Mimarideki diğer değişiklikler (bugün):**
- `spatial_distribution` (ORB uzamsal dağılım zorlaması) — kodda hazır,
  `core/feature_extractor.py`, varsayılan **kapalı** (flight-bağımlı,
  bir uçuşta felaket).
- Pencereli BA artık production'da net bir katkı sağlamıyor (E-yolu
  düzeltmesi sonrası) — hâlâ çağrılabilir (`refine_with_persistent_map`)
  ama üretim sonucu genelde onu geçiyor/eşitliyor.
- Loop closure hâlâ kapalı/kullanılmıyor (bkz. §1, offline kutusu).

---

## 0.1 30 Eylül Özeti — H-felaketinin 2024 doğrulaması, DL model kararı, Z-sürüklenmesinin somut payı

**H-matrisi felaketi 2024'te kare-numaralı olarak doğrulandı** (detay:
`problemler.md` §2.11/2.12): drone'un gerçekten sert döndüğü anlarda
(`frame_003244` GT-dönüş=107°, `frame_004436` GT-dönüş=83°) H yanlış
adayı seçiyor, dead-reckoning zinciri bu hatayı ~180° yönelim kilidi
olarak SONRAKİ tüm karelere (kendileri tamamen doğru olsa bile) yayıyor
— görselde "sanki eksen aynalanmış" izlenimi yaratan mekanizma bu.
İki klasik düzeltme fikri (H'nin kendi oy-marjısı, E'nin açı-proxy'si)
ikisi de çürütüldü.

**DL model kararı verildi:** SuperPoint (arXiv:1712.07629) ve LightGlue
(arXiv:2306.13643) tam okundu. **Birincil seçim: SuperPoint + LightGlue,
yedek: DISK + LightGlue** (ikisi de `cvg/LightGlue` deposunda tek API).
Kritik nüans: LightGlue'nun kendi makalesi tekrarlayan-doku eşleştirme
hatasını itiraf ediyor (InLoc testi) — DL bu sorunu AZALTIR, tam
ORTADAN KALDIRMAZ. **Önemli kavramsal netleştirme:** bu bir ön-uç
(feature+matcher) swap'ı — `core/motion_estimator.py`'deki H/E
ayrıştırma KALKMIYOR, sadece ona giren eşleşmeler iyileşiyor (bkz.
§5.4 altındaki not). Matrisleri tamamen kaldırmak (DROID-SLAM/MASt3R
tarzı tam-öğrenilmiş poz tahmini) ayrı, çok daha ağır bir mimari
değişiklik — ~30GB VRAM gerektiriyor, şu an gündemde değil.

**Z-sürüklenmesinin toplam hataya somut payı ölçüldü** (detay:
`problemler.md` §2.13): pozisyon hatası yatay(XY)/dikey(Z) bileşenlerine
ayrıldığında,
```
              3D hata    Yatay(XY)   Dikey(Z)   Z payı
2026:         47.28m      44.61m       8.34m     %17.6
oturum_3:     53.50m      30.82m      42.15m     %78.8
2024:        174.74m     174.74m       0.00m     %0.0  (GT Z verisi yok)
```
oturum_3'ün YATAY tahmini 2026'dan aslında daha iyi (düşük yön hatasıyla
tutarlı) — ama `force_2d` kendi Z tahminini bastırdığı için GT'nin
(oturum_3'te çok daha fazla değişen, 47.5m aralık) irtifası neredeyse
olduğu gibi 3D hataya giriyor. Z-sürüklenmesi bazı uçuşlarda toplam
hatanın BASKIN kaynağı olabiliyor — §5.5'teki DL-derinlik önerisini
somut sayılarla güçlendiriyor.

## 0.2 4 Ekim Özeti — DL ön-uç TAMAMEN entegre edildi, BA/loop closure
kapandı, Sim(3) başlatıldı

**SuperPoint + LightGlue artık tam çalışan, test edilmiş bir seçenek**
(`detector_type`/`matcher_type` config anahtarlarıyla, `core/
feature_extractor.py` + `core/matcher.py`). Üç parametre (H/E eşiği,
`lowe_ratio`, `ransac_threshold`, LightGlue'nun kendi `filter_threshold`/
`depth_confidence`/`width_confidence`'ı) 3 uçuşta tam tarandı. Üretim
`config.yaml` hâlâ ORB — DL henüz VARSAYILAN yapılmadı, sonuçlar
karışık (oturum_3'te net kazanç, 2026'da ödünleşim, 2024'te nötr).

**BA ve loop closure, DL ön-uçle YENİDEN test edildi, ikisi de
KAPANDI (negatif ama kesin sonuç):**
- BA (pencereli + tam) hâlâ yardımcı olmuyor, hatta kötüleştiriyor.
- Loop closure'ın aday-tespiti DL ile gerçekten iyileşti (66 aday,
  eskisinden çok daha fazla) ama sonuç HİÇ değişmedi — kök neden
  kesinleşti: Sim(3) (ölçek-esnek kısıt) eksikliği, front-end'den
  bağımsız.

**Sim(3)-ruhlu ölçek düzeltmesi başlatıldı, YARIM kaldı (problemler.md
§2.20) — devam edilecek.** GTSAM'in Similarity3'ü yerine özel bir
ölçek-arama yöntemi yazıldı, ilk deneme kod hatasıyla başarısız oldu,
düzeltilince sağlam ama sonuçsuz çıktı (veri/örneklem yetersizliği).

---

## 1. Tam sistem diyagramı (algoritma seviyesinde)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              config.yaml                                 │
│  camera_rgb · target_resolution · features · semantic · hybrid ·        │
│  evaluation · persistent_ba · logging                                    │
└──────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  utils/data_loader.py  (DataLoader)                                      │
│  - raw_frames/ klasorunu lexicographic sirayla listeler, frame_step      │
│    uygular (her N'inciyi al)                                             │
│  - ground-truth.csv -> GroundTruthRecord (tx,ty,tz per frame)            │
│  - detections.json  -> Detection[] per frame (YOLO onceden calistirilmis)│
│  - HICBIR gorsel/matris islemi yapmaz, sadece dagitir                    │
└───┬─────────────────────┬─────────────────────┬──────────────────────────┘
    │ kare (BGR, ham)      │ Detection listesi    │ GroundTruthRecord
    ▼                     │                      │
                            │                      │
════════════════════════ ÖN UÇ (FRONT-END) ═══════│══════════════════════
  Her karede calisir, GERIYE DONUP DUZELTME YAPMAZ  │
                            │                      │
┌───────────────────────┐  │                      │
│ utils/camera_calibration.py                      │
│ (CameraCalibration)   │  │                      │
│                        │  │                      │
│ TEKNOLOJI: Pinhole kamera modeli + Brown-Conrady  │
│ radyal/tanjansiyel bozulma modeli (cv2.undistort) │
│ Girdi : fx,fy,cx,cy + [k1,k2,p1,p2,k3]           │
│ Islev : lens bukulmesini duzeltir (duz cizgiler   │
│         goruntude duz cizgi olarak gorunsun diye) │
└───────────┬────────────┘  │                      │
            ▼                ▼                      │
┌───────────────────────────────────────────────┐   │
│ core/feature_extractor.py  (FeatureExtractor)  │   │
│                                                  │   │
│ TEKNOLOJI 1: ORB (Oriented FAST and Rotated     │   │
│   BRIEF) — koseleri (FAST) bulur, her kosenin   │   │
│   yonunu hesaplar, dondurulmus bir ikili        │   │
│   parmak izi (BRIEF) cikarir. Hizli, olcek+     │   │
│   donme degismezligine sahip, lisanssiz (SIFT   │   │
│   gibi patentli degil).                          │   │
│ TEKNOLOJI 2: Goruntu piramidi (scale_factor=1.2,│   │
│   n_levels=8) — ayni goruntuyu 8 farkli olcekte  │   │
│   tarar, boylece drone yaklasip uzaklastikca     │   │
│   ayni fiziksel nokta hala bulunabilir.          │   │
│ TEKNOLOJI 3: Semantik maske — YOLO'nun bulduğu   │   │
│   hareketli sinif kutulari (arac, insan) icindeki│   │
│   ORB noktalari ATILIR. Neden: bir arabanin      │   │
│   uzerindeki kose, kameranin degil ARABANIN      │   │
│   hareketini anlatir — bu "yalan soyleyen"       │   │
│   noktalar VO'yu bozar.                          │   │
│ TEKNOLOJI 4 (29 Eylul, VARSAYILAN KAPALI):       │   │
│   spatial_distribution -- ORB'u 3x fazla-orneklep│   │
│   goruntuyu ~max_features hucrelik izgaraya      │   │
│   bolup her hucrede en guclu tek noktayi tutar   │   │
│   (ORB-SLAM2 quadtree'sinin basit hali). Kok neden│   │
│   iyi ama flight-bagimli (1 harika/1 felaket/1   │   │
│   notr) -- KAPALI, bkz. problemler.md §2.8.      │   │
└───────────┬─────────────────────────────────────┘   │
            │ FrameFeatures (keypoints + descriptors)   │
            ▼                                          │
┌───────────────────────────────────────────────┐     │
│ core/matcher.py  (Matcher)                     │     │
│                                                  │     │
│ TEKNOLOJI 1: Brute-Force Hamming eslestirme —    │     │
│   ORB ikili (binary) tanimlayicilar oldugu icin, │     │
│   iki tanimlayici arasi "farkli bit sayisi"      │     │
│   (Hamming mesafesi) ile en yakin esi bulunur.    │     │
│   Binary oldugu icin bu XOR+popcount kadar hizli. │     │
│ TEKNOLOJI 2: Lowe's ratio test (0.75) — her       │     │
│   noktanin EN YAKIN ve IKINCI EN YAKIN esini      │     │
│   bulur; oran (en_yakin/ikinci_yakin) 0.75'ten    │     │
│   buyukse eslesme ATILIR. Amac: belirsiz/         │     │
│   tekrarlayan desenli (ör. asfalt, cim) yanlis    │     │
│   eslesmeleri elemek.                              │     │
└───────────┬─────────────────────────────────────┘     │
            │ MatchResult (pts_prev, pts_curr, indeksler)│
            ▼                                          │
┌───────────────────────────────────────────────┐     │
│ core/motion_estimator.py  (MotionEstimator)    │     │
│                                                  │     │
│ TEKNOLOJI 1: RANSAC (RANdom SAmple Consensus) — │     │
│   eslesen noktalarin bir kismi hatali (outlier)   │     │
│   olabilir. RANSAC rastgele kucuk alt kumeler     │     │
│   secip her birinden bir model kurar, en cok       │     │
│   noktayi (inlier) aciklayan modeli secer.         │     │
│ TEKNOLOJI 2: Homografi matrisi (H) — eger sahne   │     │
│   DUZ bir yuzeyse (ya da kamera sadece doner),     │     │
│   iki goruntu arasindaki iliski 3x3'luk bir H      │     │
│   matrisiyle tam aciklanir.                        │     │
│ TEKNOLOJI 3: Esas matris (Essential Matrix, E) —   │     │
│   sahne DUZ DEGILSE (genel 3B hareket), iliski     │     │
│   E = [t]_x * R ile aciklanir (kalibre edilmis     │     │
│   kameralar icin epipolar geometri).               │     │
│ TEKNOLOJI 4: R_H skor orani — H ve E paralel       │     │
│   hesaplanip hangisinin sahneyi daha iyi aciklad-  │     │
│   igina (RANSAC skoru oranina) gore aralarinda     │     │
│   secim yapilir (klasik "H vs F/E secimi" — ORB-   │     │
│   SLAM'in de kullandigi teknik, dar-baseline/duz-   │     │
│   sahne durumlarinda H daha kararli oldugu icin).  │     │
│ TEKNOLOJI 5: E'nin ayristirilmasi (SVD ile) — E    │     │
│   matrisinden 4 aday (R,t) cikar (SVD'nin isaret    │     │
│   belirsizligi yuzunden). Sadece 1 tanesi fiziksel  │     │
│   olarak gecerlidir (noktalar HER IKI kameranin da  │     │
│   ONUNDE olmali — buna "cheirality" denir).         │     │
│ TEKNOLOJI 6: DLT triangulasyonu (Direct Linear      │     │
│   Transform) — 4 adaydan hangisinin dogru oldugunu  │     │
│   bulmak icin her adayla noktalari 3B'ye triangule  │     │
│   eder, pozitif derinlik veren aday kazanir.        │     │
│ TEKNOLOJI 7 (29 Eylul, GUNCEL): E-yolu disambiguation│     │
│   kapilari -- H-yolunun cheirality+reprojeksiyon-    │     │
│   hatasi oylamasi (Teknoloji 5+6) artik E'nin 4      │     │
│   adayina da uygulaniyor -- eskiden E-yolu SADECE    │     │
│   cv2.recoverPose'un kaba ic kontrolune guveniyordu. │     │
│   Olculen etki: iki ucusta da otonom-sadece hata     │     │
│   -%12-13 (bkz. §0). BUGUNUN EN BUYUK KAZANCI.       │     │
│ NOT: Bu asamada CIKAN t sadece bir YON birim         │     │
│   vektorudur — olcek (kac metre) BILINMIYOR. Mono-   │     │
│   kuler kameranin temel sinirlamasi budur.           │     │
└───────────┬─────────────────────────────────────┘     │
            │ PoseEstimate (R, t_birim_yon, inlier_sayisi)│
            ▼                                          ▼ (sadece ISINMA'da)
════════════════════════ ARKA UÇ (BACK-END) — ÇEVRİMİÇİ ══════════════════
  Gecmisi biriktirir, sinirli duzeltme yapar (Sim3 hizalama)
┌─────────────────────────────────────────────────────────────────────────┐
│ core/scale_recovery.py  (ScaleRecovery)                                  │
│                                                                            │
│ TEKNOLOJI 1: Isinma-tabanli olcek ogrenimi — ilk N karede GT verilir.     │
│   O donemde: gercek_adim_uzunlugu / t_birim_yon_buyuklugu = k orani       │
│   hesaplanir (median ile, tek bir aykiri kareye duyarli olmasin diye).    │
│ TEKNOLOJI 2: Optik akis kalibrasyonu (c carpani) — piksel-uzayindaki      │
│   akis (kac piksel kaydi) ile gercek metre-uzayindaki hareket arasindaki  │
│   oran, ayrica bir c sabitiyle kalibre edilir (f/Z iliskisinin pratikte   │
│   tam dogru olmamasini telafi eder).                                     │
│ TEKNOLOJI 3: Semantik derinlik tahmini (bbox-tabanli) — ISINMA BITTIKTEN  │
│   SONRA (otonom modda GT yok), YOLO'nun bulduğu bilinen-boyutlu nesne-    │
│   lerin (arac=1.5m, vb. -- config'teki reference_objects tablosu)        │
│   kutu-yuksekligi/piksel oranindan "pinhole benzerlik" (f*H/h=Z)         │
│   ile derinlik (irtifa) tahmin edilir -- GT olmadan calisan tek yontem.  │
│ TEKNOLOJI 4: k*Z olcek formulu -- yon vektorunu metreye cevirirken,       │
│   olcek = k * irtifa(Z) kullanilir (aci hizi * zaman = kat edilen yol,    │
│   irtifa buyudukce ayni aci degisimi daha fazla metreye karsilik gelir). │
└───────────┬─────────────────────────────────────────────────────────────┘
            │ ScaleResult (scale, t_metre_cinsinden, mode)
            ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ core/pose_graph.py  (PoseGraph)                                          │
│                                                                            │
│ TEKNOLOJI 1: SE(3) poz zincirleme -- her adimda                          │
│   T_world_yeni = T_world_eski * [R | scale*t] carpimiyla konum birikir    │
│   (klasik "dead reckoning" -- GPS'siz konum takibinin temeli).           │
│ TEKNOLOJI 2: Umeyama algoritmasi (Sim(3) kapali-form hizalama) -- ısınma  │
│   sirasinda toplanan (yerel_konum, GT_konum) ciftlerinden, ikisini en    │
│   iyi orten bir benzerlik donusumu (donme + olcek + oteleme, TEK BIR     │
│   islemde, en kucuk kareler ile) hesaplar. Neden gerekli: VO'nun yerel   │
│   koordinat cercevesi (baslangic yonelimi keyfi) ile GT'nin dunya        │
│   cercevesi farkli -- bu donusum olmadan konumlar anlamsiz kalir.        │
│ TEKNOLOJI 3: force_2d bayragi -- Z birikimini karistan siler (GT'de Z    │
│   olmayan veri setlerinde dogru kiyaslama icin).                         │
└───────────┬─────────────────────────────────────────────────────────────┘
            │ TrajectoryPoint listesi (ÜRETİM çıktısı)
════════════│══════════════════════════════════════════════════════════════
            │           (main.py burada durur -- aşağısı opsiyonel, WSL)
            ▼
════════════════════════ ARKA UÇ — OFFLINE İYİLEŞTİRME ════════════════════
  Gecmisi TOPTAN gozden gecirir, coklu pozu BIRLIKTE duzeltebilir
┌─────────────────────────────────────────────────────────────────────────┐
│ core/pose_graph.py :: refine_with_persistent_map()                       │
│                                                                            │
│ TEKNOLOJI 1: KLT optik akis takibi (Kanade-Lucas-Tomasi, Lucas-Kanade    │
│   piramitli versiyonu, cv2.calcOpticalFlowPyrLK) -- bir kosenin bir      │
│   sonraki karede TAM OLARAK nereye gittigini (alt-piksel hassasiyette)   │
│   takip eder. ORB'dan farki: ORB her karede "yeniden tanima" yapar       │
│   (bagimsiz), KLT ise "izleme" yapar (bir onceki konumdan baslayip       │
│   kucuk bir arama penceresinde en olasi yeni konumu bulur) -- bu yuzden  │
│   uzun-bazli (15+ kare) surekli izler (track) uretebilir, ORB+matcher    │
│   zincirlemesi ise her adimda %X ihtimalle takibi kaybeder (uzun         │
│   zincirlerde neredeyse hicbir track hayatta kalmaz).                   │
│ TEKNOLOJI 2: Ileri-geri tutarlilik kontrolu (forward-backward check) --  │
│   bir noktayi kare A'dan B'ye, sonra B'den tekrar A'ya izler; baslangic  │
│   noktasindan cok uzaklastiysa (klt_fb_threshold=1.5px) o track'e        │
│   guvenilmez, atilir. KLT'nin sessizce yanlis izlemesine karsi tek       │
│   savunma.                                                                │
│ TEKNOLOJI 3: Koşe yenileme (replenishment, cv2.goodFeaturesToTrack /     │
│   Shi-Tomasi kosesellik) -- aktif track sayisi bir esigin (50) altina    │
│   dusunce, var olan track'lerden uzak (replenish_exclude_radius=15px)    │
│   yeni kosler eklenir -- boylece uzun ucus boyunca "elde takip edilecek  │
│   nokta" tukenmez.                                                        │
│ TEKNOLOJI 4: Paralaks kapisi (parallax_cos_threshold=0.999) -- bir       │
│   noktanin iki farkli kareden gorulme acilari birbirine COK yakinsa      │
│   (kamera neredeyse hic hareket etmemisse o nokta icin), o noktanin      │
│   derinligi/3B konumu GUVENILMEZ sekilde belirlenir (triangulasyonun     │
│   temel zaafi) -- bu track'ler BA'ya sokulmadan once elenir.             │
│ TEKNOLOJI 5: GTSAM (Georgia Tech Smoothing and Mapping) -- robotikte     │
│   standart bir FAKTOR GRAFI (factor graph) optimizasyon kutuphanesi.     │
│   Her bilinmeyen (poz, 3B nokta) bir DUGUM (node); her olcum/kisit       │
│   (bir kosenin bir karede su pikselde gorulmesi, ya da "bu pozun         │
│   onceki pencerenin sonucuna yakin olmasi gerekir" gibi bir on-bilgi)    │
│   bir FAKTOR (factor, dugumler arasi kisit).                             │
│   - PriorFactorPose3: bir pencerenin ILK pozunu, onceki pencerenin       │
│     sonucuna GEVSEK bir kisitla baglar (anchor_sigma=1e-3 -- ne kadar    │
│     kucukse o kadar KATI/sabit, ne kadar buyukse o kadar SERBEST).       │
│   - GenericProjectionFactorCal3_S2: "bu 3B nokta, bu pozdan bakildiginda │
│     TAM OLARAK bu pikselde gorulmeli" kisiti (yeniden-izdusum/           │
│     reprojection hatasi). Butun KLT track'leri bu tur faktore donusur.   │
│   - noiseModel.Isotropic.Sigma(pixel_noise_sigma=1.5): her piksel        │
│     olcumune ne kadar guvenildigini soyler (dusuk deger = "bu olcume     │
│     cok guveniyorum, optimize ederken zorla uydur").                     │
│ TEKNOLOJI 6: Levenberg-Marquardt optimizasyonu -- GTSAM'in ic cozucusu.  │
│   Gradyan-inis (yavas ama guvenli) ile Gauss-Newton (hizli ama          │
│   kararsiz olabilir) arasinda otomatik gecis yapan, dogrusal-olmayan     │
│   en-kucuk-kareler problemlerinin standart cozucusu. Butun faktorlerin   │
│   toplam hatasini (residual) en kucukleyen poz+nokta kumesini bulur.    │
│ TEKNOLOJI 7: Pencereli (windowed) motion-only BA -- TUM ucusu tek       │
│   seferde optimize etmek yerine, 15'lik kaydirmali pencerelerde         │
│   calisilir (hesap maliyeti O(pencere^3) yerine O(pencere_sayisi)).     │
│   "Motion-only" = 3B nokta konumlari da optimize edilebilir ama asil    │
│   hedef pozlari (kameranin nerede oldugunu) duzeltmek -- klasik "full   │
│   BA" (harita+poz) yerine daha hafif bir versiyon.                      │
└───────────┬─────────────────────────────────────────────────────────────┘
            │ Duzeltilmis TrajectoryPoint listesi
            ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ utils/visualizer.py, utils/evaluate_evo.py  (Degerlendirme)              │
│                                                                            │
│ TEKNOLOJI 1: YARISMA METRIGI -- ortalama 3B Oklid hatasi, HIZALAMASIZ    │
│   (E = (1/N) * sum(||tahmin_i - GT_i||)) -- sartnamenin Denklem 2'si ile │
│   birebir ayni, hicbir donusum/kaydirma uygulanmadan olculur.            │
│ TEKNOLOJI 2: (teshis amacli, resmi metrik degil) Sim3-hizali ATE (evo    │
│   kutuphanesi) -- SLAM literaturunde standart, ama yarismada             │
│   KULLANILMIYOR (yarismaci kendi hizalamasini yapmali, degerlendirici    │
│   hizalamiyor).                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Katman katman teknoloji özeti

### 2.1 Ön uç (front-end) teknolojileri — "ne görüyorum, ne kadar hareket ettim (yönsüz)"

| Teknoloji | Ne işe yarar | Neden bu seçildi |
|---|---|---|
| **Pinhole kamera modeli + lens bozulma düzeltme** | Gerçek lensin eğriliğini matematiksel olarak iptal eder, düz çizgiler görüntüde düz kalsın | Her geometrik hesap (H, E, DLT) düz-çizgili (ideal) bir kamera varsayar |
| **ORB (Oriented FAST + Rotated BRIEF)** | Görüntüde tekrar tanınabilir "köşe" noktaları bulur ve her birine bir parmak izi (descriptor) verir | Hızlı, patentsiz (SIFT/SURF'ün aksine), dönmeye/ölçeğe karşı dayanıklı |
| **Görüntü piramidi** | Aynı sahneyi farklı büyütme seviyelerinde tarar | Drone yaklaşıp uzaklaştıkça aynı nokta hâlâ bulunabilsin |
| **Semantik maske (YOLO çıktısıyla)** | Hareketli nesnelerin (araç, insan) üzerindeki noktaları eler | Hareketli nesne üzerindeki bir köşe, kameranın değil O NESNENİN hareketini anlatır — VO'yu yanıltır |
| **Brute-Force Hamming eşleştirme** | İki karedeki noktaları birbirine bağlar | ORB ikili (binary) olduğu için Hamming mesafesi hem doğru hem çok hızlı |
| **Lowe's ratio test** | Belirsiz/yanlış eşleşmeleri eler | Asfalt, çim gibi tekrarlayan dokularda yanlış eşleşme riski yüksek |
| **RANSAC** | Eşleşmelerdeki aykırı (yanlış) noktalara karşı dayanıklı model kurar | Hiçbir eşleştirme %100 temiz değildir |
| **Homografi (H) / Esas matris (E)** | İki kare arası geometrik ilişkiyi (dönüş + yön) çözer | H düz sahne/sadece dönüşte, E genel 3B harekette daha kararlı — ikisi paralel denenip seçilir |
| **SVD ile E ayrıştırma + cheirality testi** | 4 adaydan fiziksel olarak geçerli olan (R,t)'yi bulur | Matematiksel olarak E'den 4 çözüm çıkar, sadece 1'i gerçek dünyada mümkün |
| **DLT triangülasyonu** | Adaylardan hangisinin noktaları kameraların ÖNÜNE koyduğunu test eder | Cheirality testinin uygulanma şekli |
| **E-yolu disambiguation kapıları (29 Eylül, güncel)** | H'nin titiz cheirality+reprojeksiyon oylamasını E'nin 4 adayına da uygular | Eskiden E sadece `cv2.recoverPose`'un kaba iç kontrolüne güveniyordu — bugünün en büyük doğrulanmış kazancı (bkz. §0) |
| **spatial_distribution (29 Eylül, varsayılan kapalı)** | ORB keypoint'lerini bir ızgaraya bölüp her hücrede en güçlü tek noktayı tutar | Keypoint kümelenmesi dönüş tahminini yerel geometriye aşırı bağımlı kılıyor — ama flight-bağımlı risk taşıyor, kapalı |

### 2.2 Arka uç (back-end) — çevrimiçi — "yönü metreye çevir, biriktir"

| Teknoloji | Ne işe yarar | Neden bu seçildi |
|---|---|---|
| **Isınma-tabanlı ölçek öğrenme (k, c)** | Ön ucun ürettiği "yön birim vektörünü" gerçek metreye çevirmenin kalibrasyon sabitlerini bulur | Monoküler kamera doğası gereği ölçeği bilemez — GT ile bir kez kalibre edip sonra taşımak gerekir |
| **Semantik bbox-derinlik tahmini** | GT olmadan (otonom modda) irtifa/derinlik tahmini yapar | Bilinen gerçek boyutlu nesnelerin (araç boyu vb.) piksel boyutundan pinhole benzerliğiyle derinlik çıkarılabilir |
| **SE(3) zincirleme** | Her adımı bir öncekinin üzerine ekleyerek dünya konumunu biriktirir | Klasik dead-reckoning — GPS olmadan konum takibinin temeli |
| **Umeyama Sim(3) hizalama** | Yerel VO çerçevesini dünya/GT çerçevesine (dönüş+ölçek+öteleme) bir kerede oturtur | Yarışmacının kendi ölçek/hizalamasını yapması gerekiyor (organizatör yapmıyor) |

### 2.3 Arka uç — offline iyileştirme — "geçmişi toptan gözden geçir"

| Teknoloji | Ne işe yarar | Neden bu seçildi |
|---|---|---|
| **KLT optik akış** | Bir noktayı kareler arasında sürekli, uzun-bazlı izler | ORB+eşleştirme zincirlemesi uzun izlerde (15+ kare) neredeyse hep kaybolur, KLT kaybolmaz |
| **İleri-geri tutarlılık kontrolü** | Sessizce yanlış giden KLT izlerini yakalar | KLT'nin tek zayıf noktası — kendi kendine "yanlış ama emin" izleyebilir |
| **Köşe yenileme (Shi-Tomasi)** | Tükenen izlenebilir nokta havuzunu yeniler | Uzun uçuşta başlangıç köşeleri kamera görüş alanından çıkar |
| **Paralaks kapısı** | Güvenilmez derinlik veren (neredeyse aynı açıdan görülen) izleri eler | Triangülasyonun temel matematiksel zaafı — dar açı = büyük belirsizlik |
| **GTSAM faktör grafiği** | Birçok kısıtı (yeniden izdüşüm hataları, önceki pencereye yakınlık) TEK SEFERDE, birlikte optimize eder | Tek tek/sıralı düzeltme yerine, tüm kanıtları aynı anda dengeleyerek daha tutarlı bir çözüm bulur |
| **Levenberg-Marquardt** | Faktör grafiğinin toplam hatasını en küçükleyen çözümü bulan sayısal yöntem | Doğrusal olmayan en-küçük-kareler probleminin endüstri standardı çözücüsü |
| **Pencereli motion-only BA** | Hesap maliyetini kontrol altında tutarken, geçmiş pozları kanıtla yeniden düzeltir | Tüm uçuşu tek seferde optimize etmek hesaplama açısından çok pahalı olurdu |

**Güncel durum notu (29 Eylül):** E-yolu düzeltmesinden (§0) sonra ön-uç
zaten iyileştiği için, BA'nın eskiden verdiği küçük kazanç (ön-ucun
zayıflığını "onarmaktan" geliyordu) kayboldu — şu an production BA'yı
genelde geçiyor/eşitliyor. BA kod olarak duruyor (`refine_with_
persistent_map`), production varsayılanı değil. Loop closure de (aynı
dosyada, `refine_with_loop_closure`) benzer sebeple kapalı — ayrıca
rijit `BetweenFactorPose3` (SE(3)) kullanıyor, ORB-SLAM2'nin monoküler
loop closure'ının `Similarity3` (ölçek-düzeltmeli) yaklaşımı eksik.

### 2.4 Değerlendirme

| Teknoloji | Ne işe yarar |
|---|---|
| **Yarışma metriği (hizalamasız ortalama 3B Öklid hatası)** | Şartnamenin resmi puanlama formülüyle birebir — gerçek başarı ölçütü |
| **Sim3-hizalı ATE (evo)** | Sadece teşhis amaçlı — "trajektorinin ŞEKLİ ne kadar doğru" sorusuna cevap verir, resmi puanla ilgisi yok |

---

## 3. Neden bu ayrım (ön uç / arka uç / offline) mantıklı

Ön uç **her zaman hızlı ve tek yönlü** olmalı — drone gerçek zamanlı çalışacaksa, her karede saniyeler süren bir optimizasyon bekleyemez. Bu yüzden ön uç asla geriye dönmez, sadece "şu an ne görüyorum" sorusuna cevap verir.

Arka uç (çevrimiçi kısım) **ucuz ve anlık** düzeltmeler yapar (ölçek kalibrasyonu, Sim3 hizalama) — bunlar da her adımda ya da bir kerede hesaplanabilecek kadar hafif.

Offline iyileştirme ise **pahalı ama güçlü** — GTSAM'in faktör grafiği optimizasyonu saniyeler/dakikalar sürebilir, bu yüzden gerçek zamanlı olamaz, ama karşılığında geçmişteki birçok pozu **birlikte, kanıta dayalı** şekilde düzeltebilir. Gerçek ORB-SLAM3 gibi sistemlerde bu iş ayrı bir thread'de (arka planda) sürekli çalışır; bizim projemizde şimdilik ayrı bir script (`refine_trajectory.py`) olarak, isteğe bağlı çalıştırılıyor.

---

## 4. Çalışma altyapısı: Windows + WSL ikilisi nasıl işliyor

Bu bölüm, algoritmanın kendisiyle ilgili değil — **bu projeyi geliştirirken kullandığımız iki-bilgisayarlı düzenin** nasıl çalıştığıyla ilgili. Bunu anlamak, neden bazı şeylerin "WSL'de çalıştır" diye ayrıca belirtildiğini, neden veri kopyaladığımızı ve bazı garip hataların (Windows/WSL sayı farkları gibi) nereden geldiğini açıklıyor.

### 4.1 WSL nedir, somut olarak

**WSL (Windows Subsystem for Linux)**, Windows'un içine gömülü, **gerçek bir Linux (Ubuntu) işletim sistemi**. Ayrı bir bilgisayar almana ya da ağır bir sanal makine (VM) kurmana gerek kalmadan, Windows'unun içinde "gerçek Linux" çalıştırmanı sağlıyor. Hayal et: aynı fiziksel masaüstünde, aynı anda **iki ayrı bilgisayar** çalışıyor — biri Windows (senin normalde kullandığın), diğeri Ubuntu (bir terminal komutuyla açılan) — ikisi de aynı diski (SSD'yi) paylaşıyor ama **her biri kendi dosya sistemine, kendi kurulu programlarına, kendi Python ortamına sahip.**

### 4.2 Neden buna ihtiyacımız var: GTSAM

`GTSAM` kütüphanesinin (kalıcı-harita BA'nın kalbi) **Windows için resmi bir paket (PyPI wheel) yok** — sadece Linux için var. Yani `pip install gtsam` Windows'ta çalışmıyor. Bu projenin geri kalanı (ORB, RANSAC, KLT — hepsi OpenCV üzerinden) Windows'ta sorunsuz çalışıyor, ama GTSAM'i çalıştırabilmek için gerçek bir Linux ortamına ihtiyacımız var — işte WSL tam burada devreye giriyor.

### 4.3 İki ayrı dosya sistemi, bir köprü

- **Windows tarafı:** `C:\Users\8VH0A\Desktop\drone_semantic_slam` — senin normalde açtığın, düzenlediğin yer.
- **WSL (Ubuntu) tarafı:** `~/drone_semantic_slam` (yani `/root/drone_semantic_slam` veya `/home/kullanici/...`) — Ubuntu'nun **kendi** dosya sistemi (ext4), Windows'unkinden tamamen ayrı.
- **Köprü:** WSL içinden, Windows diskine `/mnt/c/Users/8VH0A/...` yoluyla erişilebiliyor (`/mnt/c` = "C: sürücüsünü buraya bağladım" demek).

**Neden dosyaları her seferinde WSL'in kendi tarafına KOPYALIYORUZ, direkt `/mnt/c/...` üzerinden çalıştırmıyoruz?** Performans. `/mnt/c` köprüsü üzerinden binlerce küçük dosyayı (uçuş kareleri gibi) okumak, WSL'in **kendi** ext4 diskinden okumaktan çok daha yavaş (bilinen bir WSL2 zaafı — köprü her dosya erişiminde Windows'a gidip geliyor). Bu yüzden bu oturumda hep aynı deseni kullandık: `cp`/`rsync` ile veriyi `/mnt/c/...`'den `~/drone_semantic_slam/...`'a **taşı**, sonra WSL'in kendi hızlı diskinden çalıştır.

### 4.4 İki ayrı Python ortamı (venv) — ve bunun açığa çıkardığı gerçek bir hata

Windows'ta `venv/Scripts/python`, WSL'de ise `venv/bin/activate` ile ayrı ayrı kurulmuş iki sanal ortam var — **ikisi de aynı `requirements`'tan kurulmuş olsa bile, aynı paket sürümlerine sahip olmak ZORUNDA değil.** Bunun somut, gerçek bir sonucu oldu bu oturumda: Windows'ta `opencv-python 4.13.0`, WSL'de `opencv-python 5.0.0` kurulu çıktı — kimse fark etmemişti, ve "zor" veri setlerinde (2024 uçuşu gibi) bu fark, sonucu %70 oranında değiştirebiliyordu (bkz. `tanı.md`, 25 Eylül notu). **Ders:** iki ortamdan gelen sayıları asla birbirine karıştırma; production/BA kıyaslaması her zaman **aynı ortamda** (bu projede: WSL) yapılmalı.

### 4.5 WSL'i Windows'tan tek satırla çalıştırmak

Ubuntu'yu elle açıp içine girmek yerine, Windows tarafından tek bir komutla WSL'e "şunu çalıştır, sonucu bana ver" diyebiliyoruz:

```bash
wsl -d Ubuntu -- bash -c "cd ~/drone_semantic_slam && source venv/bin/activate && python3 refine_trajectory.py config.yaml"
```

`-d Ubuntu` = hangi Linux dağıtımı (birden fazla WSL dağıtımı kurulu olabilir), `-- bash -c "..."` = Ubuntu içinde çalıştırılacak komut. Bu, Windows'tan çıkmadan Linux'a "emir gönderme" yöntemi.

### 4.6 Arka planda çalıştırma tuzağı (bu oturumda gerçekten yaşandı)

Uzun süren işlemleri (BA, YOLO tespiti gibi) beklerken konuşmayı bloke etmemek için arka planda çalıştırıyoruz. Ama şuna dikkat: **arka plana almayı WSL komutunun İÇİNE koymak işe yaramıyor** —

```bash
# YANLIŞ -- sessizce hiçbir şey yapmaz:
wsl -d Ubuntu -- bash -c "python3 agir_islem.py & disown"

# DOĞRU -- WSL çağrısının TAMAMINI dıştan arka plana al:
# (Bash tool'un kendi run_in_background=true özelliğiyle)
```

Neden: `wsl.exe`, sarmaladığı komut bittiği an kendini kapatıyor. Komutu WSL'in içinde `&` ile arka plana atsan bile, `wsl.exe`'nin kendisi hemen dönüyor ve arkasındaki Ubuntu sürecini (henüz bitmemişken) sonlandırıyor — işlem sessizce hiç olmamış gibi kayboluyor. Doğrusu, WSL çağrısının **tamamını** dıştaki araç (bu projede Bash tool'un `run_in_background`'ı) ile beklemeye almak.

---

## 5. Derin Öğrenme Alternatifleri — Her Bölüm İçin, Nerede/Neden Mantıklı

Mimariyi yukarıdaki gibi bölümlere ayırıp, her birinin (a) hangi
sorunların olası kaynağı olduğunu, (b) klasik yöntem yerine bir DL
modeli koymanın mantıklı olup olmadığını değerlendiriyoruz. 29 Eylül'de
bulunan somut kanıtlara dayanıyor (`problemler.md`).

### 5.1 Kamera Kalibrasyonu & Undistort (`utils/camera_calibration.py`)
**Olası sorun kaynağı:** oturum_3'ün distortion katsayıları 2026'dan
kopyalanmış (bağımsız ölçülmemiş) — kenar bölgelerde kalıntı hata
olası, ama test edildi (`radial_check.py`) ve zayıf bir etki bulundu.
**DL mantıklı mı:** **Hayır.** Klasik/deterministik bir problem, tek
ihtiyaç bağımsız ölçüm — DL'nin buraya katkısı yok.

### 5.2 Özellik Çıkarımı (`core/feature_extractor.py` — ORB)
**Olası sorun kaynağı:** BUGÜNÜN İKİ büyük bulgusunun kaynağı burası —
uzamsal kümelenme (§0'da bahsedilen `spatial_distribution`) ve
tekrarlayan/sentetik doku kaynaklı belirsiz noktalar (oturum_3
felaketinin kökü, `bad_frame_inspect.png`'de görsel kanıt).
**DL mantıklı mı — KARAR VERİLDİ (30 Eylül):** **SuperPoint**, yedek
aday **DISK**. Öğrenilmiş bir dedektör+descriptor, tekrarlayan
dokuları ORB'un saf yerel piksel-desenine kıyasla çok daha iyi ayırt
eder (daha geniş bağlamdan/receptive field'dan öğreniyor). Kod zaten
bunu öngörmüş — `FeatureExtractor`'ın docstring'i baştan beri
"Phase 2'de SuperPoint ile swap edilir, interface değişmez" diyor.
Entegrasyon planı: 1 Ekim, `problemler.md` §2.12.

### 5.3 Eşleştirme (`core/matcher.py` — BFMatcher + Lowe ratio)
**Olası sorun kaynağı:** `lowe_ratio` taramasının gösterdiği gibi, saf
mesafe-tabanlı eşleştirme tekrarlayan dokuda güvenilir değil.
**DL mantıklı mı — KARAR VERİLDİ (30 Eylül):** **LightGlue.**
Noktaları tek tek değil, TÜM noktaları birlikte, global bağlamla
(self+cross-attention) eşleştirir — "eşleşme kendi içinde tutarlı
ama yanlış" senaryosunu ORB+Lowe'dan çok daha iyi çözer. SuperPoint'in
descriptor'larıyla birlikte kullanıldığında en güçlü — ama entegrasyon
sırası: önce SuperPoint (1 Ekim), LightGlue ikinci/opsiyonel aşama
(tek değişken kuralı). **Nüans (LightGlue makalesi, InLoc testi):**
kendi yazarları bile "sometimes matches repeated objects... instead
of the geometric structure" diye itiraf ediyor — tekrarlayan-doku
sorunu AZALIR ama garantili ORTADAN KALKMAZ.

### 5.4 Hareket Tahmini (`core/motion_estimator.py` — H/E, RANSAC, disambiguation)
**Olası sorun kaynağı:** Bugün büyük ölçüde düzeltildi (§0), ama
cheirality'nin "iç-tutarlı ama yanlış" eşleşmeleri hâlâ yakalayamaması
kalıntı bir risk. 30 Eylül'de 2024'te kare-numaralı somut kanıt bulundu
(H-felaketi, §0.1) — bu katmandaki hata hâlâ en büyük tek kaynak.
**DL mantıklı mı:** Düşük öncelik, **AMA ÖNEMLİ NETLEŞTİRME (30 Eylül,
kullanıcı sorusu üzerine):** 5.2/5.3'teki SuperPoint/LightGlue swap'ı
bu katmanı KALDIRMIYOR — H/E ayrıştırma matematiksel olarak hâlâ
gerekli, çünkü eşleştirici (LightGlue dahil) sadece "hangi pikseller
karşılık geliyor" der, kamera hareketini (R,t) SEN hâlâ epipolar
geometriyle çıkarman gerekiyor. Sadece bu katmana giren veri
(eşleşmeler) iyileşiyor, kod/mantık aynı kalıyor — parametreleri
(`_HOMOGRAPHY_SCORE_RATIO_THRESHOLD` vb.) yeniden taramak gerekecek
ama H/E ayrımının kendisi kalkmıyor. Matrisleri gerçekten kaldırmak
isteseydik DROID-SLAM/MASt3R tarzı tam-öğrenilmiş poz regresyonuna
geçmemiz gerekirdi — bu çok daha ağır (~30GB VRAM) bir mimari
değişiklik, şu an tercih edilmiyor.

### 5.5 Ölçek Kurtarma (`core/scale_recovery.py` — semantik derinlik + optik akış)
**Olası sorun kaynağı:** Z-sürüklenmesinin kök nedeni burada olabilir
(hâlâ bulunamadı — izole test edildi, suçsuz çıktı ama tam temize
çıkmadı).
**DL mantıklı mı — GÜÇLÜ ADAY:** **Tek-görüntülü derinlik tahmini**
(Depth Anything, MiDaS gibi) — YOLO'nun "bilinen nesne boyutu"
yöntemine bağımlı kalmadan HER PİKSEL için göreli derinlik verir.
Nadir UAV makalesinin de vurguladığı zayıf nokta (dikey/ölçek
doğruluğu) tam burada — en somut, hedefli DL yatırımı.

### 5.6 Poz Grafiği / Entegrasyon (`core/pose_graph.py: update()`)
**Olası sorun kaynağı:** Saf muhasebe/entegrasyon katmanı, kendi
başına bir hata kaynağı değil — yukarıdaki katmanların hatasını
biriktiriyor.
**DL mantıklı mı:** **Hayır** — matematiksel/deterministik katman.

### 5.7 Offline İyileştirme (BA + Loop Closure)
**Olası sorun kaynağı:** BA artık net katkı sağlamıyor (§2.3 notu),
loop closure Sim(3) eksikliği yüzünden ölçek sürüklenmesine
dokunamıyor.
**DL mantıklı mı:** Düşük öncelik — bu klasik optimizasyon (GTSAM)
alanı, öğrenilmiş bir "pose refiner" araştırma aşamasında/olgun değil.

### 5.8 Hibrit yaklaşım — tam swap yerine "ihtiyaç anında DL" fikri

Tam swap yerine düşünülen bir alternatif: klasik yöntemi (ucuz, hızlı)
varsayılan olarak kullanıp, DL modelini (SuperPoint/LightGlue) SADECE
şüpheli/düşük-güvenli karelerde (düşük inlier sayısı, düşük oy marjı,
gating'in reddettiği kareler) devreye sokmak. Avantajı: hesap maliyeti
çok daha düşük (sorun zaten azınlıkta — bkz. §0), iyi çalışan
çoğunluk yolu hiç değişmiyor (risk düşük). Zorluğu: tetikleyici eşiği
doğru ayarlamak (bugünkü felaket karelerinin inlier sayısı 47-97 —
dramatik düşük değildi, basit bir eşik yetmeyebilir) ve iki farklı
özellik sistemi arasında geçişte pencereli BA'nın izlerinin (track)
sürekliliğinin bozulmaması.

**GÜNCELLEME (30 Eylül) — tetikleyici fikri test edildi, ÇÜRÜDÜ:** en
doğal aday (E'nin önerdiği dönüş açısını "sert dönüş oluyor" proxy'si
olarak kullanmak) ölçüldü, güvenilir bulunmadı (korelasyon 2026'da
~0, oturum_3'te zayıf +0.33). Güvenilir, ucuz bir tetikleyici sinyal
şu an YOK — bu yüzden 1 Ekim planı hibrit değil, SuperPoint+LightGlue
TAM swap (zaten hafif/gerçek-zamanlı, ~13-44ms). Hibrit fikri rafa
kalkmadı ama önkoşulu (güvenilir tetikleyici) hâlâ eksik.
