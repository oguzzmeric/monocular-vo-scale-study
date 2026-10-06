# Kavramlar Rehberi — Sıfırdan Anlatım

Bu dosya `problemler.md`/`mimari.md` gibi bir kayıt/referans DEĞİL — bu bir
**öğretici**. Amaç: projede geçen (ve benim sürekli kullandığım) terimleri
sıfırdan, birbirine dayanarak, kavram karmaşası olmadan anlaman. Sırayla
oku — her bölüm bir öncekine dayanıyor, atlama yapma.

---

## 1. Temel Soru: Kamera ile Nasıl Konum Bulunur?

Drone'umuzda GPS yok (ya da güvenmiyoruz), sadece bir kamera var. Kamera
her saniyede bir fotoğraf çekiyor. Sorumuz: **sadece bu fotoğraflara
bakarak, drone'un nerede olduğunu ve nasıl hareket ettiğini nasıl
anlarız?**

Bu alana **Görsel Odometri / SLAM** (Simultaneous Localization and
Mapping) deniyor. Temel fikir basit: eğer iki ardışık fotoğrafta AYNI
fiziksel noktaları (örn. bir ağacın tepesi) bulabilirsek, o noktaların
görüntüdeki KONUMUNUN nasıl değiştiğine bakarak kameranın nasıl hareket
ettiğini (döndü mü, ileri mi gitti) geometrik olarak hesaplayabiliriz.

**Bunun için 4 adım gerekiyor**, ve bu dosyanın geri kalanı bu 4 adımı
tek tek açıklıyor:

1. Her fotoğrafta "ilginç noktalar" bul (özellik/**feature** çıkarımı)
2. İki fotoğraf arasında AYNI noktaları eşleştir (**matching**)
3. Bu eşleşmelerden kameranın nasıl döndüğünü/hareket ettiğini hesapla
   (**hareket tahmini** — bu dosyanın en uzun, en karmaşık bölümü)
4. Bulunan hareketleri zaman içinde topla, gerçek konum çıkar
   (**poz zincirleme**)

---

## 2. Adım 1-2: Özellik Çıkarımı ve Eşleştirme

### 2.1 Özellik (feature) nedir?

Bir fotoğrafta HER piksel eşit derecede kullanışlı değil. Gökyüzünün
ortasındaki bir piksel, komşularına o kadar benziyor ki bir sonraki
karede "bu hangi piksel" diye ayırt edemezsin. Ama bir binanın köşesi,
bir ağacın dalı gibi **ayırt edici** noktalar var — bunlara **köşe
(corner)** ya da **özellik (feature)** diyoruz.

Bizim projede bunu bulan algoritma **ORB** (Oriented FAST and Rotated
BRIEF). İki işi var:

- **Nerede** ilginç noktalar var bulmak (köşe tespiti)
- Her noktanın **"parmak izi"**ni (descriptor — 32 baytlık bir sayı
  dizisi, o noktanın etrafındaki piksel deseni) çıkarmak

Parmak izi önemli çünkü bir sonraki karede "bu nokta hangisiydi" diye
ARAYACAĞIZ — parmak izleri birbirine ne kadar yakınsa, aynı fiziksel
nokta olma ihtimali o kadar yüksek.

### 2.2 Eşleştirme (matching) nedir?

İki kareden (önceki ve şimdiki) çıkardığımız noktaların parmak izlerini
karşılaştırıp "bu ikisi aynı fiziksel nokta" diyoruz. Bizim projede
**Brute-Force Hamming eşleştirme** kullanılıyor — her noktayı diğer
karedeki HER noktayla karşılaştırıp en yakın parmak izini buluyoruz.

**Sorun:** bazen İKİ farklı nokta birbirine çok benzer parmak izi
verebilir (örn. tekrarlayan bir doku — aynı görünen tuğlalar). Bunu
önlemek için **Lowe's Ratio Test** var: bir noktanın EN YAKIN eşleşmesi
ile İKİNCİ EN YAKIN eşleşmesi birbirine çok yakınsa ("emin değilim"),
o eşleşmeyi ATIYORUZ. Bu, "belirsiz eşleşmeleri erkenden ele" mantığı.

**Sonuç:** elimizde artık "kare 1'deki şu piksel, kare 2'deki şu piksele
karşılık geliyor" diye bir liste var (yüzlerce/binlerce çift).

---

## 3. Adım 3: Hareket Tahmini — Asıl Zor Kısım

Elimizde eşleşen nokta çiftleri var. Şimdi soru: **bu eşleşmelerden,
kameranın NASIL hareket ettiğini (dönüş + yön) nasıl çıkarırız?**

### 3.1 İki farklı matematiksel model: H ve E

Buradaki matematik, sahnenin **düz bir yüzey mi (düzlem) yoksa genel bir
3B yapı mı** olduğuna göre farklılaşıyor:

- **Homography (H) matrisi:** sahne DÜZ bir düzlemse (ya da kamera
  sadece kendi ekseni etrafında dönüyorsa), iki görüntü arasındaki
  ilişki 3x3'lük tek bir matrisle TAM olarak açıklanabilir.
- **Essential (E) matrisi:** sahne genel (düzlemsel olmayan, gerçek 3B)
  ise, ilişkiyi E matrisi açıklıyor — bu matris kameranın dönüşünü (R)
  ve hareket YÖNÜNÜ (t, ama büyüklüğü değil — buna birazdan geleceğiz)
  kodluyor.

**Neden ikisi de var, sadece biri değil?** Çünkü hangisinin doğru
olduğunu ÖNCEDEN bilemeyiz — sahne bazen düzlemsel görünüyor (örn.
düz bir tarla üzerinde uçarken), bazen değil. Bu yüzden HER KAREDE
İKİSİNİ DE deniyoruz, hangisi eşleşen noktaları daha iyi açıklıyorsa
(RANSAC skoru — bkz. 3.2) ONU seçiyoruz. Bu seçime `matrix_type`
diyoruz (H ya da E).

**BUGÜNKÜ BÜYÜK BULGUMUZ tam burada:** H'nin seçildiği kareler, E'nin
seçildiği karelerden 5-10 kat daha kötü tahmin veriyor — ve H,
drone'un GERÇEKTEN hızlı döndüğü anlarda daha çok seçiliyor. Yani "hangi
modeli seçtiğimiz" kararı, projenin en büyük hata kaynağı.

### 3.2 RANSAC — yanlış eşleşmelere karşı savunma

Eşleştirme adımından gelen yüzlerce çiftin bazıları YANLIŞ olabilir
(Lowe testi hepsini yakalayamaz). H ya da E matrisini hesaplarken bu
yanlış çiftler bizi yanıltabilir.

**RANSAC (RANdom SAmple Consensus)** buna karşı bir savunma: rastgele
küçük bir alt-küme nokta seçip onlardan bir model (H ya da E) kurar,
sonra "bu model, TÜM noktaların ne kadarını doğru açıklıyor" diye sayar
(bu sayıya **inlier sayısı** diyoruz — modeli destekleyen nokta sayısı).
Bunu yüzlerce kez tekrarlayıp en çok noktayı açıklayan modeli seçer.
**İnlier sayısı ne kadar yüksekse, o modele o kadar güvenebiliriz.**

### 3.3 Disambiguation (belirsizlik giderme) — 4 aday, 1 gerçek

İşte burada kavram karmaşası genelde başlıyor, yavaş gidelim.

H ya da E matrisini, kameranın gerçek dönüş+hareketine (R, t) çevirmek
istiyoruz. Ama matematik burada bize **TEK bir cevap değil, 4 farklı
aday** veriyor. Bunun sebebi cebirsel (matrisin kendisi işaret/yön
belirsizliği taşıyor) — teknik detayına girmeden, şunu bil: **matris
doğru olsa bile, ondan (R,t)'ye çevirirken 4 farklı olasılık çıkıyor,
ve SADECE BİRİ gerçek dünyada mümkün.**

**"Disambiguation" (belirsizlik giderme) = bu 4 adaydan doğrusunu
bulma işlemi.** Nasıl buluyoruz? İki test:

1. **Cheirality (kiralite) testi:** her adayı kullanarak birkaç eşleşen
   noktayı 3B uzaya "üçgenle" (triangulate — iki farklı açıdan bakılan
   bir noktanın 3B konumunu geometrik olarak hesapla). Gerçek doğru
   aday, bu 3B noktaları HER İKİ kameranın da ÖNÜNE koyar (kamera
   arkasındaki bir nokta görüntülenemez, fiziksel olarak imkansız).
   Yanlış adaylar noktaları kameranın ARKASINA koyar — bu onları eler.
2. **Reprojeksiyon hatası testi:** cheirality'yi geçen bir aday bile
   YANLIŞ olabilir (nokta önde ama YANLIŞ yerde). Bu yüzden üçgenlenen
   3B noktayı tekrar 2B görüntüye "geri izdüşürüp" (reproject), gerçekte
   gözlenen pikselle karşılaştırıyoruz — çok farklıysa o aday da elenir.

Her aday için, test edilen noktaların KAÇI bu iki testi geçiyor
sayıyoruz — buna **oy (vote)** diyoruz. En çok oy alan aday kazanıyor.
**"Konsensüs oranı" = kazanan adayın oyu / test edilen toplam nokta
sayısı.** Oran 1.0'a yakınsa (neredeyse tüm noktalar aynı adayı
destekliyor) güçlü bir karar; 0.5 civarındaysa belirsiz/riskli.

**Bugüne kadarki en büyük düzeltmemiz burada oldu:** H-yolu bu iki
testi hep yapıyordu, ama E-yolu (yanlışlıkla) sadece OpenCV'nin kaba,
tek-testli (sadece cheirality, reprojeksiyon yok) bir fonksiyonuna
güveniyordu. Bunu E'ye de ekleyince (H'nin yaptığının aynısı), iki
uçuşta %12-13'lük gerçek bir iyileşme aldık.

### 3.4 Ölçek belirsizliği — "ne kadar" bilinmiyor, sadece "hangi yön"

Önemli bir sınırlama: E matrisinden çıkan `t` (öteleme), sadece bir
**YÖN** vektörü — kaç METRE gittiğimizi söylemiyor! Tek bir kameradan
(monoküler), gerçek dünya ölçeğini matematiksel olarak bilmek
İMKANSIZ (aynı görüntü, hem "1 metre öteden çekilmiş küçük bir nesne"
hem "100 metre öteden çekilmiş büyük bir nesne" olabilir — kamera
ayırt edemez). Buna **ölçek belirsizliği** deniyor, monoküler SLAM'in
temel/kaçınılmaz zorluğu.

**Çözümümüz (`core/scale_recovery.py`):** YOLO ile tespit ettiğimiz
bilinen-boyutlu nesnelerden (örn. "bu araç gerçekte ~4.5m uzun, görüntüde
şu kadar piksel — demek ki kamera şu kadar uzaktaymış") kaba bir mesafe
tahmini çıkarıp, `t` yön vektörünü gerçek metreye çeviriyoruz.

---

## 4. Adım 4: Poz Zincirleme — Konum Nasıl Birikir

Her kare çifti için bir (R, t) — yani "bu adımda ne kadar döndük, ne
kadar gittik" — bulduk. Şimdi bunları ZİNCİRLEME topluyoruz: "başlangıç
konumu + 1. adım + 2. adım + ... = şu anki konum". Buna **dead
reckoning** (klasik navigasyon terimi, "pusula+hız ile konum takibi")
deniyor.

**Kritik risk:** her adımdaki KÜÇÜK bir hata, sonraki TÜM adımlara
taşınır ve BİRİKİR. 100. adımda %1 hatalı bir dönüş yaparsan, 101-500.
adımlar da o yanlış yöne göre hesaplanır — hata asla "unutulmaz",
sadece büyür. **Bu yüzden projenin başından beri "yön sürüklenmesi"
(heading drift) en büyük düşman: küçük, sistematik bir dönüş hatası,
zincirleme yüzünden koca bir konum hatasına dönüşüyor.**

**Önemli bir nüans (30 Eylül, 2024 uçuşunda somut olarak görüldü):**
hata her zaman YAVAŞ YAVAŞ birikmiyor. Bazen TEK bir karede (drone
gerçekten çok sert döndüğü bir anda, H yanlış model seçildiğinde) ANİ,
büyük (neredeyse 180°'ye yakın) bir yön hatası oluyor. Zincirleme
mantığı gereği, o andan sonraki kareler — kendileri o anda tamamen
DOĞRU hareket etseler bile — artık YANLIŞ bir temel yönelim üzerinden
hesaplanmaya devam ediyor. Görsel olarak bu, sanki tahmin edilen
yörüngenin bir bölümü "aynadaki gibi ters" duruyormuş izlenimi
yaratabiliyor — aslında eksen/işaret hatası değil, tek bir kötü anın
zincir boyunca kalıcı hale gelmesi.

---

## 5. Bundle Adjustment (BA) — Geçmişi Toptan Düzeltme

Yukarıdaki zincirleme SADECE ileriye bakıyor (her adım bir öncekine
güveniyor, geriye dönüp düzeltme yapmıyor). **BA**, bunun tersini
yapıyor: birden fazla karenin TÜM pozlarını VE gördüğü 3B noktaları
**birlikte, aynı anda** optimize ediyor — "bu noktaların TÜM
gözlemleriyle en tutarlı poz+nokta kombinasyonu ne" diye soruyor.

Bunu çözmek için **GTSAM** kütüphanesini kullanıyoruz (bir "faktör
grafiği" — her bilinmeyen bir düğüm, her kısıt bir bağlantı — kurup
`LevenbergMarquardtOptimizer` ile çözüyoruz).

**Neden artık kullanmıyoruz:** BA, rastgele GÜRÜLTÜYE karşı iyi (çok
kanıt arasında ortalama alıp gürültüyü söndürür) ama SİSTEMATİK bir
yanlılığa (örn. H'nin dönüş anlarında hep yanlış çıkması gibi) karşı
çaresiz — o yanlılığı da "kendi içinde tutarlı" şekilde taşır, düzeltmez.
Ön-uç (H/E) düzelince BA'nın düzeltecek bir şeyi kalmadı.

---

## 6. Sim(3) vs SE(3) — Rijit mi, Esnek mi?

- **SE(3):** dönüş + öteleme (6 serbestlik derecesi). ÖLÇEK SABİT —
  bir zinciri "esnetip büzemezsin", sadece döndürüp kaydırabilirsin.
- **Sim(3):** dönüş + öteleme + ÖLÇEK (7 serbestlik derecesi). Bir
  zincirin boyunu da ayarlayabilir.

**Neden önemli:** monoküler kamera ölçeği (bkz. 3.4) her adımda AYRI
tahmin ediliyor, küçük hatalarla zamanla kayabilir. Eğer drone daha
önce gördüğü yere geri dönerse (**loop closure**), ve o zincirde ölçek
kayması varsa, SE(3) (rijit) bir düzeltme bunu ASLA telafi edemez —
sadece Sim(3) (ölçek de serbest) edebilir. Biz şu an SADECE SE(3)
kullanıyoruz — bu, loop closure'ımızın hiç işe yaramamasının olası bir
sebebi (henüz test edilmedi).

---

## 7. Metrikler — "İyi" ne demek, nasıl ölçüyoruz

- **Otonom-sadece hata (metre):** her karede GERÇEK konum ile TAHMİN
  arasındaki 3B mesafe, SADECE ısınma bitince (ısınmada zaten GT
  kopyalanıyor, "bedava" sıfır hata sayılmaz).
- **Yön hatası (derece):** GT'nin gittiği yön ile tahminin gittiği yön
  arasındaki açı farkı, adım başına.
- **Medyan vs ortalama:** medyan "tipik kare" ne kadar kötü; ortalama
  birkaç FELAKET karenin çektiği yukarı şişme. İkisi arasındaki büyük
  fark = "az ama çok kötü kare var" demek.
- **% yol uzunluğu:** hata / toplam kat edilen mesafe — farklı
  uçuşları/makaleleri adil kıyaslamak için.
- **Sim(3)-hizalı ATE:** SADECE teşhis amaçlı — trajektoriyi GT'ye en
  iyi oturacak şekilde döndürüp/kaydırıp/ölçekleyip SONRA hatayı ölçer
  (yani "şeklimiz ne kadar doğru"). Yarışma metriği bu DEĞİL.
- **Yatay(XY) / Dikey(Z) ayrımı (30 Eylül'de eklendi):** "Otonom-sadece
  hata" aslında 3 BOYUTLU bir mesafe (X,Y,Z hepsi birden). Ama biz
  `force_2d` ile kendi Z tahminimizi bilerek düz/sabit tutuyoruz (Z-
  sürüklenmesi problemi çözülemediği için, bkz. §7 üstü). Yani GT'nin
  irtifada ne kadar hareket ettiğiyse, o fark bizim hiçbir tahminimize
  bağlı olmadan direkt 3D hataya giriyor. Bu yüzden toplam hatayı ikiye
  ayırıp bakıyoruz: **yatay hata** (gerçekten bizim yön/hareket
  tahminimizin kalitesi, yön hatasıyla ilişkili) ve **dikey hata**
  (sadece GT'nin kendi irtifa değişiminin, bizim hiç tahmin etmeye
  çalışmadığımız yansıması). Somut örnek: oturum_3'ün yön hatası düşük
  (5.62°) ama toplam pozisyon hatası yüksek çıkmıştı — ayrıştırınca
  görüldü ki yatay tahmini aslında 2026'dan DAHA İYİ, sadece o uçuşta
  GT irtifa çok daha fazla değiştiği için görmediğimiz o fark toplam
  hatanın %78.8'ini oluşturuyormuş.

---

## 8. DL'ye Geçiş — SuperPoint + LightGlue (3-4 Ekim'de TAMAMLANDI)

Şu ana kadar anlatılan her şey (ORB, Hamming eşleştirme, Lowe testi) —
**ön-uç** dediğimiz katman. Yarın bunun yerine iki öğrenilmiş (deep
learning) model koyuyoruz. Neyin değişip neyin değişmediğini netleştirmek
önemli, çünkü kolayca yanlış anlaşılıyor:

**SuperPoint** — ORB'un yerine geçecek. ORB gibi "önce köşe bul, sonra
parmak izini ayrı çıkar" değil, TEK bir sinir ağı ikisini birlikte
üretiyor. Parmak izleri artık 32-baytlık ikili sayı değil, 256 sayılık
ondalıklı (float) bir liste. Elle etiketlenmiş veri YOK — kendi kendini
öğretiyor: önce basit sentetik şekillerle (üçgen, köşe) bir temel
öğreniyor, sonra gerçek fotoğrafları yüzlerce farklı şekilde bükerek
("bu köşe, resmi nasıl döndürürsen döndür hep aynı yerde bulunuyor mu")
kendi kendine etiket üretiyor.

**LightGlue** — Brute-Force Hamming eşleştirmenin (§2.2) yerine geçecek
(SuperPoint'ten sonra, ikinci/opsiyonel adım). Şu anki yöntem her noktayı
TEK BAŞINA, diğerinden habersiz karşılaştırıyor. LightGlue bir sinir ağı
(transformer) kullanarak İKİ görüntüdeki TÜM noktalara BİRLİKTE bakıyor —
"bu eşleşme, diğer tüm eşleşmelerle GEOMETRİK OLARAK tutarlı mı" diye
soruyor. Tekrarlayan dokuda (aynı görünen ev sıraları, çim, araba —
tam bizim nadir-açı sorunumuz) bu yüzden daha güvenilir; ama kendi
makalesi bile "bazen tekrarlayan nesneleri yanlış eşleştirebiliyorum"
diyor — yani sorunu AZALTIYOR, SIFIRLAMIYOR.

**NELERİN DEĞİŞMEDİĞİ — en çok karışan kısım burası:** SuperPoint ve
LightGlue sana sadece "kare 1'deki şu piksel, kare 2'deki şu piksele
karşılık geliyor" der — yani sadece EŞLEŞTİRME kalitesini iyileştirir.
Bu eşleşmelerden kameranın gerçekte NASIL döndüğünü/hareket ettiğini
(R, t) çıkarmak hâlâ §3'teki H/E matrisi + disambiguation (cheirality +
reprojeksiyon oylaması) mantığına ihtiyaç duyuyor — bu KISIM AYNEN
KALIYOR, sadece ona giren eşleşmeler daha temiz olacak. Matrisleri
tamamen ortadan kaldırmak (kameranın hareketini de doğrudan bir sinir
ağıyla tahmin etmek, DROID-SLAM/MASt3R gibi sistemlerin yaptığı) çok
daha büyük, ~30GB VRAM isteyen ayrı bir mimari — biz o yola GİTMİYORUZ.

---

## 9. Hızlı Özet Tablosu

| Terim                           | Tek cümlede                                                               |
| ------------------------------- | ------------------------------------------------------------------------- |
| ORB                             | Görüntüde ayırt edici noktaları bulan + "parmak izi" çıkaran algoritma    |
| Eşleştirme (matching)           | İki karedeki aynı fiziksel noktayı bulma                                  |
| Lowe's Ratio Test               | Belirsiz eşleşmeleri erkenden eleme                                       |
| RANSAC                          | Yanlış eşleşmelere dayanıklı model kurma yöntemi                          |
| İnlier sayısı                   | Bir modeli destekleyen (doğru kabul edilen) nokta sayısı                  |
| Homography (H)                  | "Sahne düz" varsayımıyla kamera hareketini açıklayan matris               |
| Essential (E)                   | Genel (düzlemsel olmayan) sahne için kamera hareketini açıklayan matris   |
| Disambiguation                  | Bir matristen çıkan 4 (R,t) adayından gerçek olanı bulma                  |
| Cheirality testi                | "Nokta her iki kameranın da önünde mi" kontrolü                           |
| Reprojeksiyon hatası            | "Üçgenlenen nokta gerçekten doğru piksele mi düşüyor" kontrolü            |
| Konsensüs oranı                 | Kazanan adayın oyu / toplam test edilen nokta                             |
| Ölçek belirsizliği              | Monoküler kameranın gerçek mesafeyi bilememesi                            |
| Poz zincirleme / dead reckoning | Adım adım hareketleri toplayıp konum bulma                                |
| Yön (heading) sürüklenmesi      | Küçük dönüş hatalarının zincirleme birikip büyük konum hatasına dönüşmesi |
| Bundle Adjustment (BA)          | Birden fazla karenin pozunu+3B noktalarını birlikte optimize etme         |
| GTSAM                           | BA'yı çözmek için kullandığımız kütüphane (faktör grafiği)                |
| SE(3)                           | Rijit dönüşüm (dönüş+öteleme, ölçek sabit)                                |
| Sim(3)                          | Esnek dönüşüm (dönüş+öteleme+ölçek)                                       |
| Loop closure                    | Daha önce görülen bir yere dönüşü tespit edip sürüklenmeyi düzeltme       |
| Otonom-sadece hata              | Isınma hariç, gerçek konumla tahmin arası ortalama mesafe                 |
| Yön hatası                      | GT yönü ile tahmin yönü arasındaki açı farkı                              |
| % yol uzunluğu                  | Hata / toplam kat edilen mesafe (adil kıyaslama için)                     |
| Sim(3)-ATE                      | Şekil-bazlı hata (teşhis amaçlı, resmi metrik değil)                      |
| Yatay(XY)/Dikey(Z) ayrımı       | 3D hatayı, "bizim tahmin kalitemiz" (XY) ile "hiç tahmin etmediğimiz Z"  |
| SuperPoint                      | ORB'un yerine geçecek, öğrenilmiş nokta+parmak izi çıkarıcı               |
| LightGlue                       | Hamming eşleştirmenin yerine geçecek, öğrenilmiş/global eşleştirici       |
| Ön-uç DL swap ≠ matris kaldırma | SuperPoint/LightGlue eşleşmeyi iyileştirir, H/E ayrıştırma yine de gerekli |
