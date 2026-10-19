# Saildrone / Yelkenli Katamaran Simülatörü — Hata Analizi, Matematiksel Model ve Navigasyon Test Raporu

**Tarih:** 19 Ekim 2026  **Kapsam:** `app.py` (Streamlit simülatörü) ve bu çalışmada eklenen `saildrone_model.py`, `navigation.py`, `tests/`, `analysis/`

---

## 1. Özet

| Başlık | Sonuç |
|---|---|
| Bulunan mantık hatası | **8** (6'sı sayısal olarak kanıtlandı, hepsi düzeltildi) |
| Çıkarılan model | Polar + trim verimi + dümen cezası → hedef hız; 1. derece surge ataleti; kinematik dönüş + orsaya kaçma; 2B kinematik konum; VMG tabanlı güdüm + layline/koridor kontra değiştirme + ters-kinematik dümen kontrolcüsü |
| Birim testleri | **174 / 174 geçti** (`pytest`, model 78 + navigasyon 96) |
| Navigasyon senaryo matrisi | **72 / 72 hedefe ulaşıldı** (3 profil × 4 rüzgar hızı × 6 başlangıç–hedef çifti) |
| 360° erişilebilirlik | **144 / 144 hedefe ulaşıldı** (1 km yarıçapta 15°'de bir hedef, 3 profil, 3 ve 8 m/s) |
| Saildrone gidebilir mi? | **Evet**, şu koşullarla: (1) başlangıç pruvası ölü açıda (TWA ≤ 30°) olmamalı, (2) süre sınırı yeterli olmalı, (3) tack yapılamıyorsa varış yarıçapı dönüş çapından (≈2R) büyük olmalı. |

---

## 2. Proje yapısı ve çalıştırma

```
app.py                       Streamlit arayüzü (Serbest Seyir + Hedefe Navigasyon sekmeleri)
saildrone_model.py           Fizik modeli (Streamlit'ten bağımsız, test edilebilir)
navigation.py                Güdüm, manevra kararı ve dümen kontrolcüsü
tests/test_model.py          Model birim testleri
tests/test_navigation.py     Navigasyon (başlangıç → hedef) testleri
analysis/legacy_bug_evidence.py  Orijinal koddaki hataları sayısal olarak gösterir
analysis/run_experiments.py      Bu rapordaki tüm tablo ve grafikleri üretir
legacy/app_original.py       Orijinal app.py (karşılaştırma için)
docs/figures, docs/results   Üretilen grafikler ve tablolar
```

```bash
pip install -r requirements.txt pytest
```
```bash
python -m pytest -q
```
```bash
python analysis/run_experiments.py
```
```bash
streamlit run app.py
```

---

## 3. Bulunan mantık hataları ve düzeltmeler

Kanıtların tamamı: [docs/results/legacy_bug_evidence.md](docs/results/legacy_bug_evidence.md) (orijinal fonksiyonlar `legacy/app_original.py`'den doğrudan yüklenip çalıştırılmıştır).

### H1 — Kontra (tack) isimleri ters
Rüzgar kuzeyden eserken doğuya giden (ψ = 30°–150°) teknede rüzgar **iskele** (sol) bordadan gelir; bu **iskele kontra**dır. Orijinal kod aynı satırda `wind_side = "İskele (Sol)"` ile `tack = "Sancak Kontra"` döndürüyordu — kendi içinde çelişkili.

| ψ | Eski tack / rüzgar tarafı | Yeni |
|---|---|---|
| 45°, 90° | Sancak Kontra / İskele (Sol) ❌ | İskele Kontra / İskele (Sol) ✔ |
| 270° | İskele Kontra / Sancak (Sağ) ❌ | Sancak Kontra / Sancak (Sağ) ✔ |

**Düzeltme:** Rejim ve kontra adları rüzgarın geldiği bordaya göre üretiliyor (`saildrone_model.analyze_points_of_sail_360`).

### H2 — İskele/sancak sınır asimetrisi
Sancak tarafı aralıkları `(alt, üst]`, iskele tarafı da pusula açısında `(alt, üst]` yazıldığı için aynı TWA iki tarafta farklı rejime düşüyordu:

| TWA | ψ = TWA | ψ = 360 − TWA |
|---|---|---|
| 50° | Orsa, oran 0.78 | Dar Apaz, oran **0.98** |
| 80° | Dar Apaz, 0.98 | Tam Apaz, **1.30** |
| 120° | Tam Apaz, 1.30 | Geniş Apaz, **0.98** |
| 150° | Geniş Apaz, 0.98 | Pupa, **0.58** |

**Düzeltme:** Önce TWA = min(ψ, 360 − ψ) hesaplanıp tek bir bant tablosu (TWA ∈ (alt, üst]) kullanılıyor; simetri testlerle doğrulandı.

### H3 — Gösterilen hız ile simülasyon hızı farklı formüllerle hesaplanıyordu
Telemetri hızı (`trim_efficiency`) tolerans bantlı ve "fazla kasılı / fazla açık" ayrımlı bir formül kullanırken, yörünge döngüsü farklı katsayı ve üslerle **ikinci bir formül** kullanıyordu (ör. kumaş: telemetride `(d/25)^1.6`, döngüde `(d/28)^1.8`). Ayrıca döngüdeki formül d = 20° sınırında 0.08 → 0.06 sıçrıyordu.

* Saf formül farkı (aynı bom/optimum için): NACA 0012 **−%62**, NACA 0015 **−%45**, kumaş **+%110**.
* 30 s sabit seyirde ekranda gösterilen hız ile "gidilen yol / süre" arasında **%46.7'ye varan** fark.

**Düzeltme:** Tek bir `trim_efficiency()` fonksiyonu hem telemetride hem dinamikte kullanılıyor.

### H4 — Auto-Trim dönüş sırasında yeniden trim yapmıyordu
Auto-trim, bom açısını yalnız başlangıç pruvasına göre ayarlayıp tüm simülasyon boyunca donduruyordu. ψ₀ = 60°, dümen +20°, 20 s, 8 m/s: **eski 22.0 m, yeni 64.6 m** (Saildrone'un otomatik kanat kontrolü her adımda optimuma gelir).

### H5 — Orsaya kaçma (weather helm) süreksizliği
Orsaya kaçma momenti yalnızca |δ| < 2° iken uygulanıyordu; 0.1°'lik dümen farkı 60 s sonunda pruvayı **14°** değiştiriyordu (δ = 1.9° → 120.8°, δ = 2.0° → 135.0°). Yeni modelde moment her zaman var ve dümen onu karşılar: 127.4° → 128.3° (sürekli).

### H6 — İleri yön ataleti yoktu: tack fiziksel olarak imkânsızdı
Hız her adımda anında polar değerine eşitleniyordu. Pruva ölü açıya girdiği anda v = 0, dönüş hızı (v/L)·sin δ = 0 olduğundan tekne **sonsuza dek kilitleniyordu**; hiçbir rüzgar hızında rüzgar üstünden kontra değiştirilemiyordu (ψ₀ = 40°, δ = −35°: 3, 5, 8, 12 m/s'nin hepsinde son pruva 22°–29°, kilit).
**Düzeltme:** Birinci derece surge dinamiği (τ = 3 s) eklendi; artık 8 ve 12 m/s'de tekne taşıdığı hızla ölü açıyı geçebiliyor, 3–5 m/s'de geçemiyor (bkz. §5.5).

### H7 — Yanıltıcı yörünge etiketi
Grafikteki `Rota: <son pruva> | <başlangıç hızı>` etiketi son pruvayı **başlangıç** hızıyla yan yana gösteriyordu. Artık "Son pruva | Ortalama hız (yol/süre)" gösteriliyor.

### H8 — Test edilemez yapı ve yanlış yardım metni
Fizik kodu Streamlit arayüzüyle aynı dosyadaydı (içe aktarıldığında arayüz çalışıyordu) ve aynı mantık iki kez yazılmıştı. Model `saildrone_model.py`'ye taşındı; `app.py` bu modeli kullanıyor. Pruva yardım metnindeki "270°: İskele Tam Apaz" ifadesi de "Sancak Kontra Tam Apaz" olarak düzeltildi.

---

## 4. Matematiksel modeller

### 4.1 Koordinatlar ve rüzgar

* Dünya ekseni: x = Doğu, y = Kuzey (m). Pruva ψ: pusula açısı (0° = Kuzey, saat yönünde).
* Rüzgar sabit, hızı U (m/s), **kuzeyden** eser.
* Gerçek rüzgar açısı:  **TWA(ψ) = min(ψ mod 360, 360 − ψ mod 360)** ∈ [0°, 180°]
* Rüzgar altı tarafı: s_lee = +1 (ψ ∈ (0°,180°], iskele kontra, yelken sancağa), −1 (ψ ∈ (180°,360°), sancak kontra), 0 (TWA ≤ 30°).

### 4.2 Hız poları (parçalı sabit)

Ölü açı: TWA ≤ 30° → ρ = 0. Diğer bantlar (TWA ∈ (alt, üst]):

| Bant | TWA | NACA 0012 δ_opt / ρ | NACA 0015 δ_opt / ρ | Kumaş δ_opt / ρ |
|---|---|---|---|---|
| Orsa | 30–50° | 12° / 0.78 | 14° / 0.74 | 16° / 0.62 |
| Dar apaz | 50–80° | 28° / 0.98 | 30° / 0.94 | 32° / 0.85 |
| Tam apaz | 80–120° | 42° / **1.30** | 45° / 1.25 | 46° / 1.15 |
| Geniş apaz | 120–150° | 62° / 0.98 | 64° / **1.05** | 65° / 0.92 |
| Pupa | 150–180° | 78° / 0.58 | 80° / 0.65 | 82° / **0.74** |

ρ = V_b / U (tekne hızının rüzgar hızına oranı), δ_opt = optimum bom açısı.

![Hız poları](docs/figures/polar.png)

### 4.3 Trim (yelken/kanat) verimi

d = |δ_s − δ_opt| olmak üzere:

```
         ⎧ 1 − ℓ · d / d_tol                         , d ≤ d_tol           (tam trim)
η(δ_s) = ⎨ max(η_min,u , 1 − (d / a_u)^p_u)          , δ_s < δ_opt         (fazla kasılı / stall)
         ⎩ max(η_min,o , 1 − (d / a_o)^p_o)          , δ_s > δ_opt         (fazla açık / yapraklama)
```

| Profil | d_tol | ℓ | a_u, p_u, η_min,u | a_o, p_o, η_min,o |
|---|---|---|---|---|
| NACA 0012 | 4° | 0.04 | 20°, 1.9, 0.08 | 22°, 1.9, 0.06 |
| NACA 0015 | 6° | 0.04 | 24°, 1.7, 0.12 | 26°, 1.7, 0.10 |
| Kumaş | 6° | 0.05 | 25°, 1.6, 0.15 | 30°, 1.6, 0.10 |

Auto-trim modunda δ_s ≡ δ_opt(TWA) → η = 1.

![Trim verimi](docs/figures/trim_efficiency.png)

### 4.4 Dümen sürüklemesi

δ ∈ [−35°, +35°] (mekanik sınır),  **κ(δ) = 1 − 0.22 · (|δ| / 35°)²**

### 4.5 Kararlı-durum (hedef) hız

**V\*(ψ, δ_s, δ) = U · ρ(TWA(ψ)) · η(δ_s) · κ(δ)**

### 4.6 İleri yön (surge) dinamiği — yeni

**dv/dt = (V\* − v) / τ**,  τ = 3 s

Ayrık zamanda tam üstel çözüm kullanılır (her Δt için koşulsuz kararlı):
v_{k+1} = V\* + (v_k − V\*) · e^(−Δt/τ)

### 4.7 Dönüş (yaw) dinamiği — kinematik bisiklet modeli + orsaya kaçma

```
          ⎧ (v / L) · sin δ · (180/π)  −  s_lee · ω_wh   , v > 0.2 m/s
ψ̇ [°/s] = ⎨
          ⎩ 0                                           , v ≤ 0.2 m/s   (dümen etkisiz)
```
L = 4.6 m, ω_wh = 0.35 °/s. Tam dümende dönüş yarıçapı R = L / sin 35° ≈ 8.0 m (dönüş hızı sınırına takılmadıkça).

### 4.8 Konum kinematiği

ẋ = v · sin ψ,  ẏ = v · cos ψ

### 4.9 Sayısal entegrasyon (Δt = 0.25 s, yarı-örtük Euler)

1. V\*_k = V\*(ψ_k, δ_s, δ_k) 2. v_{k+1} (4.6) 3. ψ_{k+1} = ψ_k + ψ̇(v_{k+1}, δ_k, ψ_k)·Δt 4. x, y ← x, y + v_{k+1}·(sin ψ_{k+1}, cos ψ_{k+1})·Δt

τ = 0 seçildiğinde orijinal (ataletsiz) model elde edilir.

### 4.10 Navigasyon (güdüm, manevra, dümen)

**VMG ve kontra seçimi.** Hedef kerterizi β için her kontra (side ∈ {+1, −1}) üzerinde, ölü açıya 5° güvenlik payıyla:

VMG_side = max_{TWA > 35°} U·ρ(TWA)·cos(ψ_side(TWA) − β),  ψ_{+1} = TWA, ψ_{−1} = 360° − TWA

Polar tablosundan elde edilen optimum açılar:

| Profil | Optimum rüzgar üstü TWA | VMG_up / U | Optimum rüzgar altı TWA | VMG_down / U | Tam pupa VMG / U |
|---|---|---|---|---|---|
| NACA 0012 | 30.5° | 0.672 | 150° | 0.849 | 0.580 |
| NACA 0015 | 30.5° | 0.638 | 150° | 0.909 | 0.650 |
| Kumaş | 50.5° | 0.541 | 150° | 0.797 | 0.740 |

(Navigasyonda 5° pay nedeniyle sert kanatlar fiilen TWA ≈ 35.5°'de orsa yapar.) Tam pupa hiçbir profilde VMG-optimum değildir; rüzgar altı hedeflere de zikzakla (gybe) gidilir.

**Kontra değiştirme kuralı.** Diğer kontranın VMG'si daha büyükse ve
(a) mevcut kontranın VMG'si ≤ 0 ise (hedef geride kaldı), ya da asgari bacak süresi (10 s) dolmuşken
(b) **layline**: diğer kontranın optimum rotası hedefi tutuyorsa (rüzgar üstünde hedef o rotanın rüzgar üstünde değil; rüzgar altında rüzgar altında değil), veya
(c) **koridor**: başlangıç–hedef hattından |e_ct| > W (W = 0.2·D, [50, 1000] m) uzaklaşılmış ve hâlâ uzaklaşılıyorsa
kontra değiştirilir.

**Tack fizibilitesi.** Ölü açıda V\* = 0 olduğundan hız v₀·e^(−t/τ) ile söner; tam dümende dönülebilecek açı:

**Θ_max = ∫₀^∞ (v₀ e^(−t/τ) / L) sin δ_max dt = v₀ · τ · sin δ_max / L**

Tack için Θ_max ≥ 1.3 · (2·30° + 5°) gerekir ⇒ **v₀ ≥ 3.03 m/s**. Bu analitik ön koşul sağlansa bile karar, manevranın aynı dümen yasasıyla **ileri simülasyonu** ile kesinleştirilir. Tack mümkün değilse tekne rüzgar altından ~290° dönerek kontra değiştirir; bu durumda değişim, manevra **sonrası** konumdan hedefin tutulabildiği ileri simülasyonla doğrulanınca yapılır.

**Dümen kontrolcüsü (ters kinematik).** e = ψ\* − ψ (manevra yönüne göre işaretli),

r\* = sat(k·e, ±30 °/s) + s_lee·ω_wh,  **δ = sat(asin(r\*·L / v), ±35°)**, k = 0.5 s⁻¹

Doymasız bölgede kapalı çevrim ė = −k·e olur (zaman sabiti 2 s, hızdan bağımsız); ayrık sistemde e_{k+1} = (1 − kΔt)·e_k = 0.875·e_k ⇒ kararlı. Orsaya kaçma ileri beslemeyle telafi edildiğinden kalıcı durum hatası yoktur.

**Başarı / başarısızlık ölçütleri.** Varış: ‖p − p_hedef‖ ≤ r_varış (varsayılan 10 m). Başarısızlık: süre sınırı (varsayılan 7200 s) aşımı veya ölü açıda v < 0.05 m/s ile 20 s kalma.

### 4.11 Parametreler

| Sembol | Değer | Açıklama |
|---|---|---|
| θ_ng | 30° | Ölü açı yarı genişliği |
| L | 4.6 m | Efektif gövde boyu (dönüş) |
| δ_max | 35° | Dümen sınırı |
| τ | 3 s | Surge zaman sabiti (**varsayım**, yeni) |
| ω_wh | 0.35 °/s | Orsaya kaçma |
| Δt | 0.25 s | Entegrasyon adımı |
| k | 0.5 s⁻¹ | Pruva kontrol kazancı |
| r_varış | 10 m | Varış yarıçapı |

---

## 5. Testler ve sonuçlar

### 5.1 Birim testleri (174/174 ✔)

`tests/test_model.py` — ölü açıda itki sıfır; iskele/sancak simetrisi (sınırlar dahil); kontra isimlendirmesi; trim verimi tepe değeri ve sınırları; dümen cezası; **düz seyirde yol = hız × süre**; telemetri hızı = dinamik hızı; pozitif dümen → sancağa dönüş; orsaya kaçma yönü; ölü açıda başlayan tekne hareketsiz; surge yakınsaması; auto-trim'in dönüş boyunca yeniden trim yapması.

`tests/test_navigation.py` — 72 senaryonun tamamı; dümen sınırına uyum ve ölü açıda geçirilen sürenin < %5 olması; rüzgar üstünde tack gerekliliği; hafif rüzgarda gybe ile kontra değişimi; ölü açıda başlangıcın başarısızlık olarak raporlanması; sıfır mesafe; süre sınırı; 10 km uzun menzil; 12 yönde erişilebilirlik; tack fizibilite eşiği; layline geometrisi.

### 5.2 Başlangıç → hedef senaryo matrisi (72/72 ✔)

Senaryolar (rüzgar kuzeyden, mesafeler ≈ 1–1.4 km):
S1 (0,0)→(1000,0) tam apaz · S2 (0,0)→(0,−1000) tam rüzgar altı · S3 (0,0)→(700,700) · S4 (0,0)→(0,1000) tam rüzgar üstü · S5 (0,0)→(300,1000) ölü açı içinde (017°) · S6 (500,−300)→(−400,800) ofset başlangıç.

Hedefe varış süreleri:

| Profil | U (m/s) | S1 | S2 | S3 | S4 | S5 | S6 |
|---|---|---|---|---|---|---|---|
| NACA 0012 | 1 | ✔ 762 s | ✔ 1663 s | ✔ 1394 s | ✔ 1952 s | ✔ 1892 s | ✔ 2004 s |
| NACA 0012 | 3 | ✔ 254 s | ✔ 458 s | ✔ 466 s | ✔ 653 s | ✔ 632 s | ✔ 670 s |
| NACA 0012 | 8 | ✔ 95 s | ✔ 164 s | ✔ 164 s | ✔ 211 s | ✔ 218 s | ✔ 239 s |
| NACA 0012 | 15 | ✔ 51 s | ✔ 86 s | ✔ 128 s | ✔ 118 s | ✔ 124 s | ✔ 133 s |
| NACA 0015 | 1 | ✔ 792 s | ✔ 1508 s | ✔ 1457 s | ✔ 2056 s | ✔ 1990 s | ✔ 2100 s |
| NACA 0015 | 3 | ✔ 264 s | ✔ 460 s | ✔ 488 s | ✔ 688 s | ✔ 665 s | ✔ 700 s |
| NACA 0015 | 8 | ✔ 99 s | ✔ 176 s | ✔ 173 s | ✔ 221 s | ✔ 254 s | ✔ 250 s |
| NACA 0015 | 15 | ✔ 53 s | ✔ 98 s | ✔ 114 s | ✔ 125 s | ✔ 128 s | ✔ 138 s |
| Kumaş | 1 | ✔ 861 s | ✔ 1357 s | ✔ 1668 s | ✔ 2419 s | ✔ 2312 s | ✔ 2419 s |
| Kumaş | 3 | ✔ 287 s | ✔ 453 s | ✔ 559 s | ✔ 808 s | ✔ 772 s | ✔ 809 s |
| Kumaş | 8 | ✔ 108 s | ✔ 171 s | ✔ 198 s | ✔ 258 s | ✔ 268 s | ✔ 288 s |
| Kumaş | 15 | ✔ 58 s | ✔ 92 s | ✔ 113 s | ✔ 163 s | ✔ 144 s | ✔ 160 s |

Rota verimi (düz mesafe / katedilen yol): S1 ≈ 1.0 · S2 0.87 · S3 0.72 · S4 0.63 · S5 0.64 · S6 0.76 (ortalamalar). Yol, hız, tack/gybe sayılarıyla tam tablo: [docs/results/scenario_matrix.md](docs/results/scenario_matrix.md) ve `.csv`.

Gözlemler:
* **Tam apaz en hızlı** yön; tam rüzgar üstü hedef aynı mesafe için ~2.2×, tam rüzgar altı ~1.7× daha uzun sürüyor.
* Tam rüzgar altında (S2) kumaş yelken düşük rüzgarda en hızlı (1 m/s: 1357 s vs 1508–1663 s; 3 m/s: 453 s vs 458–460 s); 8–15 m/s'de NACA 0012 öne geçiyor (164 s vs 171–176 s). Rüzgar üstünde ve apazlarda NACA 0012 en iyi; tek istisna S3 / 15 m/s (0012 hedef yakınında ek bir manevra yaptığı için 128 s, kumaş 113 s).
* 1 ve 3 m/s'de tekne hızı < 3 m/s olduğundan **tack hiç yapılamıyor**; kontra değişimleri rüzgar altından dönülerek yapılıyor ve görev yine tamamlanıyor.

![Rotalar, U = 8 m/s](docs/figures/trajectories_U8.png)

![Hafif ve kuvvetli rüzgarda tam rüzgar üstü](docs/figures/upwind_light_vs_strong.png)

![S4 zaman serileri](docs/figures/timeseries_S4_U8.png)

### 5.3 360° erişilebilirlik (144/144 ✔)

1 km yarıçaplı çember üzerinde 15°'de bir 24 hedef, 3 profil, 3 ve 8 m/s. Grafik, "etkin VMG = 1000 m / varış süresi" değerini gösterir; ölü açı yönündeki hedefler (000°, 015°, 345°) en yavaş, tam apaz yönleri en hızlıdır. Tablo: [docs/results/reachability.md](docs/results/reachability.md).

![Erişilebilirlik](docs/figures/reachability.png)

### 5.4 Uç durumlar

| Durum | Ulaştı | Süre (s) | Tack | Gybe | Sonuç |
|---|---|---|---|---|---|
| Başlangıç pruvası ölü açıda (000°), hedef Doğu 1 km, 8 m/s | ✘ | 20 | 0 | 0 | Ölü açıda takıldı — hız sıfır, dümen etkisiz |
| Başlangıç pruvası ölü açı sınırında (032°), hedef Kuzey 1 km | ✔ | 211 | 3 | 0 | — |
| Başlangıç pruvası ters yönde (180°), hedef Kuzey 1 km | ✔ | 240 | 5 | 1 | — |
| Tack yasak, varış yarıçapı 10 m, Kuzey 1 km, 8 m/s | ✘ | 7200 | 0 | 458 | Süre sınırı (hedef çevresinde döngü) |
| Tack yasak, varış yarıçapı 30 m, Kuzey 1 km, 8 m/s | ✔ | 252 | 0 | 2 | — |
| Hafif rüzgar 2 m/s (tack fiziksel olarak imkânsız), Kuzey 1 km | ✔ | 976 | 0 | 2 | — |
| Hedef başlangıçta (mesafe 0) | ✔ | 0 | 0 | 0 | — |
| Uzun menzil: Kuzey 10 km, 5 m/s, kumaş | ✔ | 4485 | 2 | 3 | — |
| Kısa süre limiti: Kuzey 1 km, 3 m/s, 300 s | ✘ | 300 | 0 | 0 | Süre sınırı |

### 5.5 Tack eşiği (açık çevrim, TWA 35° + tam dümen)

| Profil | Tack için asgari rüzgar | Tack başlangıç hızı | Analitik v_min |
|---|---|---|---|
| NACA 0012 | 5.2 m/s | 3.16 m/s | 3.03 m/s |
| NACA 0015 | 5.4 m/s | 3.12 m/s | 3.03 m/s |
| Kumaş | 6.5 m/s | 3.14 m/s | 3.03 m/s |

Simülasyonla bulunan eşik hızı analitik formülle (§4.10) %3–5 içinde uyuşuyor; fark, ölü açıya girmeden önceki (dümen sürüklemeli) yavaşlamadan kaynaklanıyor.

### 5.6 Tack yapılamıyorsa: varış yarıçapı duyarlılığı (NACA 0012, Kuzey 1 km)

| Varış yarıçapı (m) | U=3 | U=5 | U=8 | U=12 | U=15 |
|---|---|---|---|---|---|
| *Dönüş çapı 2R (m)* | *16* | *16* | *24* | *36* | *45* |
| 10 | ✔ 653 s | ✔ 520 s | ✘ | ✘ | ✘ |
| 15 | ✔ 650 s | ✔ 392 s | ✘ | ✘ | ✘ |
| 20 | ✔ 648 s | ✔ 390 s | ✔ 269 s | ✘ | ✘ |
| 25 | ✔ 646 s | ✔ 389 s | ✔ 253 s | ✘ | ✘ |
| 30 | ✔ 644 s | ✔ 388 s | ✔ 252 s | ✔ 179 s | ✘ |
| 40 | ✔ 610 s | ✔ 368 s | ✔ 250 s | ✔ 177 s | ✔ 147 s |

Kural: rüzgar altından dönüşle hedefe rüzgar üstünden yaklaşılıyorsa **r_varış ≳ 0.8 · 2R** olmalıdır; aksi halde manevranın kendisi tekneyi hedefin rüzgar altına iter ve hedef çevresinde döngü oluşur. Normal modda (tack serbest) bu durum yalnızca tekne hızı < 3 m/s iken oluşur; o hızlarda R ≈ 8 m olduğundan varsayılan 10 m yarıçap yeterlidir (72/72 sonucu bunu doğrular).

---

## 6. Sonuç: Saildrone belirtilen konumdan istenen konuma gidebilir mi?

Düzeltilmiş model ve eklenen navigasyon kontrolcüsü ile **evet**: test edilen 216 başlangıç–hedef–profil–rüzgar kombinasyonunun (72 senaryo + 144 erişilebilirlik) tamamında tekne hedefe ulaştı; hedef ölü açı içinde olsa bile zikzakla (tack veya gybe) gidebiliyor.

Ulaşılamayan durumlar ve nedenleri:

1. **Başlangıç pruvası ölü açıda (TWA ≤ 30°) ve tekne durgun** → itki ve dümen etkisi sıfır, model bu durumdan çıkış mekanizması içermiyor (gerçek teknede yelkeni ters basma / rüzgarla sürüklenme ile çıkılır).
2. **Süre sınırı** mesafe / etkin VMG'den kısaysa (ör. 1 km rüzgar üstü, 3 m/s: ≈ 650 s gerekirken 300 s).
3. **Tack mümkün değil ve varış yarıçapı dönüş çapından küçük** (§5.6).

Orijinal kodla ise **rüzgar üstündeki hiçbir hedefe kontra değiştirerek gidilemezdi** (H6: ölü açıya giren tekne kalıcı olarak kilitlenir), üstelik ekranda gösterilen hız ile gidilen yol farklı formüllerden hesaplandığı için (H3) süre tahminleri de tutarsızdı.

---

## 7. Model sınırlamaları ve öneriler

| Sınırlama | Etkisi | Öneri |
|---|---|---|
| Parçalı sabit polar (bant sınırlarında sıçrama) | VMG-optimum pruvalar hep bant kenarlarına (30.5°, 50.5°, 80.5°, 150°) düşer | Poları TWA'ya göre sürekli (spline) yapmak |
| V_b ∝ U (doygunluk yok) | 25 m/s rüzgarda 32.5 m/s (63 knot) tekne hızı — gerçekçi değil; gerçek Saildrone ≈ 2–4 knot | Hız oranını rüzgarla azalan / doyuran bir polar; reef / kanat açısı sınırlaması |
| Sürüklenme (leeway), akıntı, dalga yok | Rota yere göre değil suya göre | Leeway açısı ve akıntı vektörü eklemek |
| Sabit rüzgar yönü/hızı | Kontra stratejisi gerçek koşullarda test edilmedi | Rüzgar salınımı ve hamle (gust) modeli |
| Dönüşte yaw ataleti yok (kinematik) | Manevralar gerçekte daha yavaş | 2. derece yaw dinamiği (I_z ṙ = N_δ δ − N_r r) |
| Ölü açıda durgun tekne kurtulamaz | Uç durum 1 | "Backing / drift" modeli (rüzgarın pruvayı düşürmesi) |
| τ = 3 s varsayımdır | Tack eşiğini doğrudan belirler (v_min ∝ 1/τ) | Gerçek tekne verisiyle tanımlama (system identification) |

---

## 8. Yeniden üretme

```bash
python analysis/legacy_bug_evidence.py
```
```bash
python analysis/run_experiments.py
```
```bash
python -m pytest -q
```

Tüm grafikler `docs/figures/`, tüm tablolar `docs/results/` altına yeniden yazılır. Arayüzdeki **🎯 Hedefe Navigasyon** sekmesinde başlangıç/hedef koordinatları, rüzgar, profil, başlangıç pruvası, varış yarıçapı ve süre sınırı girilerek aynı test etkileşimli olarak çalıştırılabilir.
