# Saildrone / Yelkenli Katamaran Simülatörü — Kumaş Yelken Modeli: Matematiksel Model, Hedefe Navigasyon ve Test Raporu

**Tarih:** 21 Ekim 2026  **Kapsam:** `saildrone_model.py` (fizik modeli), `navigation.py` (güdüm ve kontrol), `app.py` + `components/nav_player` (arayüz), `tests/`, `analysis/`

---

## 1. Özet

Bu çalışmada simülatör **tek bir yelken modeline — Kumaş Yelken (Cambered / Klasik)** — indirgenmiştir. Model, aynı denklemlerle hem *Serbest Seyir* (kullanıcının yelken, dümen ve pruva açılarını elle verdiği açık çevrim) hem de *Hedefe Navigasyon* (sistemin açıları kendisinin atadığı kapalı çevrim) ekranlarında kullanılır.

| Başlık | Sonuç |
|---|---|
| Yelken modeli | Kumaş yelken: 5 bantlı hız poları + trim verimi + dümen sürüklemesi |
| Serbest seyir dinamiği | Hedef hız → 1. derece ileri (surge) dinamiği → kinematik dönüş + orsaya kaçma → konum |
| Kuvvet ve moment | Yelken itkisi, yan kuvvet, gövde direnci, dümen kuvveti (N); dümen ve yelken momentleri (N·m), Bölüm 3.1 |
| Fiziksel parametreler | 13 parametre (m, k_g, c_dd, A_s, a_CE, a_LR, L, A_d, x_d, C_Lα, C_L,max, ρ_hava, ρ_su) arayüzden değiştirilebilir; türetilmiş büyüklükler τ, λ, ω_wh simülasyona girer, Bölüm 3.2 |
| Hedefe navigasyon | Kerteriz → ölü açı kontrolü → VMG-optimum pruva → kontra (tack/gybe) kararı → auto-trim yelken açısı → ters-kinematik dümen açısı |
| Navigasyon hesap tablosu | 17 adımın formülleri ve seçilen andaki değerleri arayüzde; simülasyonla birebir aynı, Bölüm 4.3 |
| Rota | Başlangıç → istenen sayıda ara durak → hedef (çokgen, daire, yay rotaları) |
| Birim testleri | **124 / 124 geçti** (`pytest`: model 59 + navigasyon 65) |
| Senaryo matrisi | **24 / 24 hedefe ulaşıldı** (4 rüzgar hızı × 6 başlangıç–hedef çifti) |
| 360° erişilebilirlik | **48 / 48 hedefe ulaşıldı** (1 km yarıçapta 15°'de bir hedef, 3 ve 8 m/s) |
| Ara duraklı rota | Kare rota (4 bacak, 8 m/s) 406 s'de tamamlandı; 4 tack, 3 gybe |

Rüzgar her testte **kuzeyden (000°) eser ve sabittir**; testler arasında yalnızca rüzgar hızı ve başlangıç / hedef konumları değiştirilir.

---

## 2. Koordinatlar, açılar ve işaret kuralları

| Sembol | Anlamı | İşaret / aralık |
|---|---|---|
| $x, y$ | Doğu, Kuzey konumu (m) | — |
| $\psi$ | Pruva açısı (pusula) | 0° = Kuzey, saat yönünde artar, $[0°, 360°)$ |
| $U$ | Rüzgar hızı (m/s) | Kuzeyden eser |
| $\theta$ | Gerçek rüzgar açısı (TWA) | $[0°, 180°]$ |
| $\delta_s$ | Yelken bom açısı (orta hattan) | $+$: bom sancağa, $-$: iskeleye; $\lvert\delta_s\rvert \in [0°, 90°]$ |
| $\delta_r$ | Dümen açısı | $+$: pala sancağa → tekne sancağa döner ($\psi$ artar); $\lvert\delta_r\rvert \le 35°$ |
| $v$ | Teknenin ileri hızı (m/s) | $v \ge 0$ |
| $s$ | Rüzgar altı bordası | $+1$: iskele kontra (yelken sancakta), $-1$: sancak kontra, $0$: ölü açı |

Rüzgar kuzeyden geldiği için gerçek rüzgar açısı ve rüzgar altı bordası doğrudan pruvadan bulunur:

$$\theta(\psi) = \min(\psi,\; 360° - \psi), \qquad s(\psi) = \begin{cases} +1, & 0° < \psi \le 180°,\ \theta > 30° \\ -1, & 180° < \psi < 360°,\ \theta > 30° \\ 0, & \theta \le 30° \ \text{(ölü açı)} \end{cases}$$

---

## 3. Serbest Seyir: Kumaş Yelken Dinamiği (adım adım)

Serbest seyirde kullanıcı üç açıyı verir: **pruva $\psi_0$, yelken bom açısı $\delta_s$ ve dümen açısı $\delta_r$**. Bu üç açının teknenin hızına ve yönelimine etkisi aşağıdaki yedi adımla hesaplanır. Her adım bir öncekinin çıktısını kullanır.

### Adım 1 — Rüzgarın tekneye göre açısı

$\theta = \min(\psi, 360°-\psi)$. Ölü açının genişliği yalnızca pruvaya değil, yelkenin bom açısına da bağlıdır:

$$\theta_{ölü}(\delta_s) = \max\big(30°,\ \lvert\delta_s\rvert\big)$$

$\theta \le \theta_{ölü}$ ise tekne **ölü açıdadır**: kumaş yelken rüzgarı tutamaz, yapraklar, itki sıfırdır.

* **30° teknenin kendi sınırıdır (330°–030°).** Bom ne kadar kasılırsa kasılsın yelken kuvveti neredeyse tamamen yana yönelir; ileri bileşeni direnci yenemez. Auto-Trim'de bom her zaman optimum açıdadır, bu yüzden ölü açı 30°'dir.
* **Bom elle 30°'den fazla açılırsa ölü açı genişler.** Bom rüzgar yönüne kadar açıldığında ($\theta \le \lvert\delta_s\rvert$) yelken rüzgara paralel kalır ve yapraklar (Adım 3, yapraklama sınırı). Örnek: $\delta_s = 56°$ için ölü açı 304°–056° olur; $\psi = 055°$ bu sektörün içindedir.

### Adım 2 — Seyir bandı: hız oranı ve optimum bom açısı

$\theta$ hangi banda düşerse kumaş yelkenin o banttaki **hız oranı** $\rho = V_b/U$ ve **optimum bom açısı** $\delta_{opt}$ okunur:

| Bant | $\theta$ aralığı | $\delta_{opt}$ | $\rho = V_b/U$ | Yelkenin durumu |
|---|---|---|---|---|
| Ölü açı | $0°$–$30°$ (elle trimde $0°$–$\max(30°, \lvert\delta_s\rvert)$) | — | 0 | Yapraklar, itki yok |
| Orsa | $30°$–$50°$ | 16° | 0.62 | Iskota sonuna kadar kasılı |
| Dar apaz | $50°$–$80°$ | 32° | 0.85 | Biraz boşlanmış, tor artar |
| Tam apaz | $80°$–$120°$ | 46° | 1.15 | Tam dolu ve torlu (en hızlı seyir) |
| Geniş apaz | $120°$–$150°$ | 65° | 0.92 | İyice boşlanmış |
| Pupa | $150°$–$180°$ | 82° | 0.74 | Paraşüt gibi dolar, itki sürüklemeden |

![Kumaş yelken hız poları](docs/figures/polar.png)

### Adım 3 — Yelken trimi: aerodinamik verim $\eta(\delta_s)$

Bomun optimum açıdan sapması $d = \lvert\,\lvert\delta_s\rvert - \delta_{opt}\rvert$ yelkenin ürettiği itkiyi azaltır:

$$\eta(\delta_s) = \begin{cases} 1 - 0.05\,\dfrac{d}{6°}, & d \le 6° \quad \text{(tam trim)}\\[2mm] \max\!\left(0.15,\; 1 - \left(\dfrac{d}{25°}\right)^{1.6}\right), & \lvert\delta_s\rvert < \delta_{opt} \quad \text{(fazla kasılı: yelken düzleşir, akış kopar)}\\[2mm] \max\!\left(0.10,\; 1 - \left(\dfrac{d}{30°}\right)^{1.6}\right), & \lvert\delta_s\rvert > \delta_{opt} \quad \text{(fazla açık: ön kenar yapraklar)} \end{cases}$$

Fazla açık dalın ölçeği (30°) fazla kasılı dalınkinden (25°) büyüktür: kumaş yelken boşlandığında itkiyi kademeli kaybeder, fazla kasıldığında ise akış daha erken kopar. *Auto-Trim* açıkken $\lvert\delta_s\rvert = \delta_{opt}$ alınır ve $\eta = 1$ olur.

**Yapraklama sınırı.** Yukarıdaki dallar yalnızca bomun optimumdan ne kadar saptığına bakar. Yelkenin rüzgarı görüp görmediğini ise kirişin rüzgara **hücum açısı** belirler:

$$\alpha = \theta - \lvert\delta_s\rvert, \qquad \eta \leftarrow \begin{cases} 0, & \alpha \le 0 \quad \text{(bom rüzgar yönünde: yelken rüzgara paralel, bayrak gibi yapraklar)}\\[1mm] \min\!\left(\eta,\ \dfrac{\alpha}{8°}\right), & 0 < \alpha < 8° \quad \text{(orsa yakası çırpar)} \end{cases}$$

Örnek: $\psi = 055°$ ($\theta = 55°$), $\delta_s = 56°$ → $\alpha = -1°$. Bom pusulada $055° + 180° - 56° = 179°$'u, yani tam rüzgar altını gösterir. Yelken rüzgara paraleldir, $\eta = 0$ olur ve tekne ilerlemez. Sınır yalnızca bom optimumdan fazla boşlandığında devreye girer. Polar optimumu ±6° tolerans içinde her bantta $\alpha > 8°$'dir ($\theta > 30°$, $\delta_s \le 16° + 6°$), bu yüzden kalibre edilmiş trim değerleri ve auto-trim sonuçları değişmez. Bom en fazla 90° açılabildiğinden $\theta \ge 98°$ seyirlerde (geniş apaz, pupa) yelken boşlanarak yapraklatılamaz.

Hücum açısı gerçek rüzgara göre hesaplanır. Tekne dururken görünür rüzgar gerçek rüzgara eşittir. Tekne ilerledikçe görünür rüzgar pruvaya kayar (AWA < θ), bu yüzden $\alpha \le 0$ yapraklamanın en hafif (gerekli) koşuludur (bkz. Bölüm 8).

![Kumaş yelken trim verimi](docs/figures/trim_efficiency.png)

### Adım 4 — Dümenin sürükleme etkisi $\kappa(\delta_r)$

Açılan dümen palası suya karşı direnç oluşturur ve hızı düşürür:

$$\kappa(\delta_r) = 1 - c_{dd}\left(\frac{\lvert\delta_r\rvert}{35°}\right)^2, \qquad c_{dd} = 0.22:\ \ \kappa(0) = 1,\ \ \kappa(\pm35°) = 0.78$$

$c_{dd}$ dümen sürükleme katsayısıdır (varsayılan 0.22, arayüzden değiştirilebilir; Bölüm 3.2).

### Adım 5 — Aerodinamik kuvvetin gövdeye iletimi: hız

Yelkenin ürettiği itki ile gövdenin su direnci dengelendiğinde tekne bir **kararlı-durum (hedef) hızına** ulaşır. Kumaş yelken modelinde bu denge hızı polar, trim ve dümen çarpanlarının çarpımıdır:

$$V^* = \lambda\cdot U\cdot\rho(\theta)\cdot\eta(\delta_s)\cdot\kappa(\delta_r)$$

$\lambda$, yelken alanı ve gövde direncinden gelen **hız ölçeğidir**; polar varsayılan tekne için ölçüldüğünden varsayılan teknede $\lambda = 1$'dir (Bölüm 3.2).

Tekne bu hıza anında değil, kütlesi nedeniyle gecikmeyle ulaşır. İleri yön (surge) hareket denklemi $m\,\dot v = F_{itki} - F_{direnç}$, gövde direnci $R = k_g\,v$ ile birinci derece bir sisteme dönüşür:

$$\dot v = \frac{V^* - v}{\tau}, \qquad \tau = \frac{m}{k_g} = \frac{250\ \text{kg}}{83.33\ \text{N·s/m}} = 3\ \text{s}$$

Yani gövdeye iletilen **net ileri kuvvet** $F_{net} = m\,(V^*-v)/\tau$'dur: tekne hedef hızından yavaşsa yelken onu hızlandırır, hızlıysa (ör. ölü açıya girdiğinde $V^*=0$) su direnci yavaşlatır. Ayrık zamanda tam çözüm kullanılır (her $\Delta t$ için kararlı):

$$v_{k+1} = V^*_k + (v_k - V^*_k)\,e^{-\Delta t/\tau}$$

### Adım 6 — Dümen etkisi ve dönme momenti: yönelim

Dümen palası, akışa açılı durduğunda bir yan kuvvet üretir; bu kuvvet gövdenin dönme merkezine olan uzaklığı ($L$) kadar bir kolla **dönme momenti** oluşturur. Momentin etkisi, yan kaymanın ihmal edildiği kinematik (bisiklet) modeliyle doğrudan dönüş hızına çevrilir. Buna ek olarak, kumaş yelkenin basınç merkezi gövdenin yanal direnç merkezinin gerisinde kaldığından tekne kendiliğinden rüzgara doğru döner (**orsaya kaçma**, weather helm):

$$\dot\psi\ [°/\text{s}] = \underbrace{\frac{v}{L}\,\sin\delta_r\cdot\frac{180}{\pi}}_{\text{dümen momenti}} \;-\; \underbrace{s(\psi)\,\omega_{wh}}_{\text{yelken momenti}}, \qquad L = 4.6\ \text{m},\ \ \omega_{wh} = 0.35\ °/\text{s}$$

* Dümen etkisi **hızla orantılıdır**: $v \le 0.2$ m/s iken $\dot\psi = 0$ (dümen dümen tutmaz).
* Sabit dümende dönüş yarıçapı $R = L/\sin\delta_r$ (tam dümende $R = 8.0$ m).
* Orsaya kaçma rüzgar altı bordasına bağlıdır: iskele kontrada ($s=+1$) pruva azalır, sancak kontrada ($s=-1$) artar — her iki durumda da tekne rüzgara döner. Örnek: $\psi_0 = 095°$, auto-trim, $\delta_r = 0°$ → 30 s'de pruva 95° → 84.5° ($-10.5°$). Tekne önce 5.6 m güneye iner, sonra kuzeye kıvrılır.
* **Rotayı Tut.** Pruvayı sabit tutmak için dümenin orsaya kaçmayı her adımda dengelemesi gerekir ($\dot\psi = 0$):
  $$\sin\delta_{r,tut} = \frac{s(\psi)\,\omega_{wh}\,L}{v}$$
  Tam apazda bu açı +0.175°'dir; arayüzdeki 1°'lik dümen adımıyla verilemez. Bu yüzden serbest seyre "Rotayı Tut" seçeneği eklendi. Seçenek açıkken dümen girdisi kullanılmaz; $\psi_0 = 095°$'de tekne 276 m'de 24.1 m güneye, pruva doğrultusunda gider.
* Arayüz pruvanın simülasyon boyunca değişimini üç yerde gösterir: pruva girdisinin altında ("95° → 84.5°"), çizimde (başlangıç ve son pruva hattı arasındaki dönüş yayı) ve **Açı Takip Tablosu**nda (pruva–zaman grafiği; zaman, pruva, TWA, seyir, bom, dümen, hız, konum).
* Dümen ve yelken momentlerinin N·m cinsinden hesabı Bölüm 3.1'dedir.
* $L$ ve $\omega_{wh}$ varsayılan tekneye aittir; $L$ arayüzden doğrudan, $\omega_{wh}$ ise yelken–dümen geometrisinden türetilerek değişir (Bölüm 3.2).

### Adım 7 — Konum

$$\dot x = v\sin\psi, \qquad \dot y = v\cos\psi$$

### Sayısal çözüm ($\Delta t = 0.25$ s)

Her adımda sırasıyla: (1) $\theta_k$, $\rho$, $\delta_{opt}$; (2) $\eta$, $\kappa$ → $V^*_k$; (3) $v_{k+1}$; (4) $\psi_{k+1} = \psi_k + \dot\psi(v_{k+1}, \delta_r, \psi_k)\,\Delta t$; (5) $x, y \mathrel{+}= v_{k+1}(\sin\psi_{k+1}, \cos\psi_{k+1})\,\Delta t$. Auto-Trim açıksa (2)'de bom açısı her adımda yeni $\theta$'ya göre güncellenir.

### Sayısal örnek (U = 8 m/s, $\psi$ = 090°, tam apaz)

| Girdi | $\eta$ | $\kappa$ | $V^*$ (m/s) | $\dot\psi$ (°/s) | Yorum |
|---|---|---|---|---|---|
| $\delta_s = 46°$, $\delta_r = 0°$ | 1.00 | 1.00 | **9.20** | −0.35 | Tam trim; yalnızca orsaya kaçma |
| $\delta_s = 30°$, $\delta_r = 0°$ | 0.51 | 1.00 | 4.70 | −0.35 | 16° fazla kasılı: itki yarıya düşer |
| $\delta_s = 60°$, $\delta_r = 0°$ | 0.71 | 1.00 | 6.48 | −0.35 | 14° fazla açık: yapraklama |
| $\delta_s = 46°$, $\delta_r = +10°$ | 1.00 | 0.98 | 9.03 | +19.2 | $R$ = 26.5 m sancağa dönüş |
| $\delta_s = 46°$, $\delta_r = +20°$ | 1.00 | 0.93 | 8.54 | +36.0 | $R$ = 13.4 m |
| $\delta_s = 46°$, $\delta_r = +35°$ | 1.00 | 0.78 | 7.18 | +50.9 | Tam dümen, $R$ = 8.0 m |

### 3.1 Açık kuvvet ve moment hesabı (N, N·m)

Yukarıdaki adımlar hızları ve açıları verir. Aynı denklemler, tekneye etkiyen **kuvvetler (N)** ve **momentler (N·m)** cinsinden de yazılabilir. Bunun için aşağıdaki fiziksel parametreler kullanılır (≈ 5 m'lik bir yelkenli katamaran için tipik değerler; kodda `saildrone_model.Physics`, arayüzde sol kenar çubuğundan değiştirilebilir — Bölüm 3.2). Tablodaki değerler **varsayılan teknedir**; bu bölümdeki sayısal sonuçlar bu değerlerle hesaplanmıştır.

| Parametre | Sembol | Varsayılan değer |
|---|---|---|
| Tekne + ekipman kütlesi | $m$ | 250 kg |
| Gövde doğrusal direnç katsayısı | $k_g$ | 83.33 N·s/m |
| Kumaş yelken alanı | $A_s$ | 20 m² |
| Efektif gövde boyu (kinematik dönüş) | $L$ | 4.6 m |
| İki dümen palasının toplam alanı | $A_d$ | 0.16 m² |
| Dümen kaldırma eğimi (en-boy oranı 2.5) | $C_{L\alpha} = 2\pi\,\frac{AR}{AR+2}$ | 3.49 1/rad, stall: $C_{L,max} = 1.2$ |
| Dümenin ağırlık merkezine uzaklığı | $x_d$ | 2.1 m |
| Ağırlık merkezi – yelken basınç merkezi / yanal direnç merkezi (kıça doğru +) | $a_{CE}$, $a_{LR}$ | 0.20 m / 0 m |
| Yelken basınç merkezi ile yanal direnç merkezi arası kaçıklık | $e = a_{CE} - a_{LR}$ | 0.20 m (tam apazda kalibre) |
| Dümen sürükleme katsayısı | $c_{dd}$ | 0.22 |
| Hava / deniz suyu yoğunluğu | $\rho_a$, $\rho_{su}$ | 1.225 / 1025 kg/m³ |

![Kuvvet diyagramı](docs/figures/force_diagram.png)

**K1 — İleri yön kuvvet dengesi (Newton'un 2. yasası).** Adım 5'teki hız denklemi, kütleyle çarpılınca bir kuvvet dengesine dönüşür:

$$m\,\dot v = \underbrace{T}_{\text{yelken itkisi}} - \underbrace{D_d}_{\text{dümen sürüklemesi}} - \underbrace{R}_{\text{gövde direnci}}, \qquad T = k_g\,\lambda\,U\rho\,\eta, \quad D_d = T\,(1-\kappa), \quad R = k_g\,v$$

Kararlı hızda $\dot v = 0$, yani $T = D_d + R$ ve $v = V^*$ olur. Gövde direnci hızla doğrusal kabul edilmiştir: $k_g = m/\tau = 83.3$ N·s/m. Yelken itkisi yelken alanıyla orantılıdır ($T \propto A_s$); varsayılan teknede $\lambda = 1$.

**K2 — Yelken kuvvetinin bileşenleri.** Kumaş yelkenin ürettiği toplam aerodinamik kuvvet $F_N$, yaklaşık olarak bom kirişine diktir (düz levha yaklaşımı). Bom orta hatla $\delta_s$ açısı yaptığından bu kuvvet iki bileşene ayrılır:

$$T = F_N\sin\delta_s \ \ \text{(ileri itki)}, \qquad F_{yan} = F_N\cos\delta_s \ \ \text{(rüzgar altına yan kuvvet)} \ \Rightarrow\ F_N = \frac{T}{\sin\delta_s}$$

Bom ne kadar kasılıysa ($\delta_s$ küçük) aynı itki için o kadar büyük yan kuvvet oluşur; orsada yan kuvvet itkinin 3.5 katıdır. Yan kuvveti gövdelerin yanal direnci karşılar (modelde yan kayma yoktur). Kuvvetin büyüklüğünün makul olup olmadığı, görünür rüzgarla tanımlı kuvvet katsayısıyla kontrol edilir:

$$V_A = \sqrt{v^2 + U^2 + 2vU\cos\psi}, \qquad \cos(\text{AWA}) = \frac{v + U\cos\psi}{V_A}, \qquad C_N = \frac{F_N}{\tfrac12\rho_a V_A^2 A_s}$$

**K3 — Dümen kuvveti ve dönme momenti.** Akışa $\delta_r$ açısıyla duran dümen palalarının yan kuvveti ve bu kuvvetin ağırlık merkezine göre momenti:

$$Y_d = \tfrac12\,\rho_{su}\,A_d\,C_L(\delta_r)\,v^2, \quad C_L = \operatorname{sat}\!\big(C_{L\alpha}\,\delta_r,\ \pm C_{L,max}\big), \qquad N_d = Y_d\,x_d$$

Pala sancağa açıldığında ($\delta_r > 0$) su palayı iskeleye iter; kıç iskeleye, pruva sancağa döner ($N_d > 0$).

**K4 — Yelkenin dönme momenti (orsaya kaçma).** Yan kuvvet $F_{yan}$ yelken basınç merkezinde ($a_{CE}$) etkir; onu dengeleyen gövde yanal direnci yanal direnç merkezinde ($a_{LR}$) etkir. Ağırlık merkezine göre net moment kolu $e = a_{CE} - a_{LR}$'dir; basınç merkezi yanal direnç merkezinin gerisinde ($e > 0$) olduğundan tekneyi rüzgara döndüren bir moment oluşur:

$$N_y = -s\,F_{yan}\,e$$

Düz gitmek için dümenin bu momenti dengelemesi gerekir: $Y_d\,x_d = -N_y$, buradan $\delta_{r,denge} = \dfrac{s\,F_{yan}\,e}{x_d\cdot\tfrac12\rho_{su}A_d C_{L\alpha} v^2}$. Tam apazda bu 0.17° çıkar; simülasyonun kontrolcüsünün düz seyirde tuttuğu dümen açısı 0.175°'dir (Adım 7, Bölüm 4).

**K5 — Net moment ve dönüş.** Net moment $N = N_d + N_y$ tekneyi döndürür. Simülasyonda dönüş hızı kinematik modelden ($\dot\psi = \frac{v}{L}\sin\delta_r - s\,\omega_{wh}$) gelir; bu dönüşü sürdürmek için gövdelerin sağlaması gereken yanal (merkezcil) kuvvet:

$$F_c = m\,v\,\dot\psi = \frac{m v^2}{R}$$

#### Sayısal sonuçlar (U = 8 m/s)

**Seyir bantlarına göre yelken kuvvetleri** (auto-trim, $\delta_r = 0$, kararlı hız):

| ψ (°) | Seyir | δs (°) | V* (m/s) | V_A (m/s) | AWA (°) | T itki (N) | F_N (N) | F_yan (N) | C_N | N_yelken (N·m) | δr denge (°) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 35.5 | Orsa | 16 | 4.96 | 12.38 | 22 | 413 | 1500 | 1441 | 0.80 | −288 | +1.12 |
| 50.5 | Dar Apaz | 32 | 6.80 | 13.40 | 27 | 567 | 1069 | 907 | 0.49 | −181 | +0.37 |
| 90.0 | Tam Apaz | 46 | 9.20 | 12.19 | 41 | 767 | 1066 | 740 | 0.59 | −148 | +0.17 |
| 135.0 | Geniş Apaz | 65 | 7.36 | 5.91 | 73 | 613 | 677 | 286 | 1.58 | −57 | +0.10 |
| 165.0 | Pupa | 82 | 5.92 | 2.75 | 131 | 493 | 498 | 69 | 5.38 | −14 | +0.04 |

Orsa, dar apaz ve tam apazda gereken kuvvet katsayısı $C_N$ = 0.5–0.8'dir; bu, kumaş bir yelken için gerçekçi bir değerdir. Rüzgar altında görünür rüzgar çok zayıfladığından ($V_A$ = 2.7–5.9 m/s) $C_N$ fiziksel sınırı (≈ 1.3) aşar. Bu, kullanılan poların rüzgar altı hız oranlarının (0.92, 0.74) iyimser olduğunu gösterir (bkz. Bölüm 8).

**Yelken trimi** ($\psi$ = 90°, $\delta_r$ = 0):

| δs (°) | η | V* (m/s) | T itki (N) | F_N (N) | F_yan (N) |
|---|---|---|---|---|---|
| 20 | 0.15 | 1.38 | 115 | 336 | 316 |
| 30 | 0.51 | 4.70 | 391 | 783 | 678 |
| **46** | **1.00** | **9.20** | **767** | **1066** | **740** |
| 60 | 0.70 | 6.48 | 540 | 624 | 312 |
| 75 | 0.10 | 0.92 | 77 | 79 | 21 |

Optimum bom açısında (46°) itki en büyüktür. Fazla kasınca itki düşer ama yan kuvvetin itkiye oranı büyür; fazla boşlayınca her iki kuvvet de söner.

**Dümen kuvveti ve dönme momenti** ($\psi$ = 90°, $\delta_s$ = 46°):

| δr (°) | κ | v (m/s) | D_dümen (N) | C_L | Y_dümen (N) | N_dümen (N·m) | N_net (N·m) | ψ̇ (°/s) | R (m) | F_merkezcil (N) |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 1.000 | 9.20 | 0 | 0.00 | 0 | 0 | −148 | −0.35 | ∞ | 14 |
| +5 | 0.996 | 9.16 | 3 | 0.30 | 2095 | 4399 | 4251 | +9.6 | 54.7 | 383 |
| +10 | 0.982 | 9.03 | 14 | 0.61 | 4077 | 8562 | 8414 | +19.2 | 27.0 | 757 |
| +20 | 0.928 | 8.54 | 55 | 1.20 | 7175 | 15067 | 14919 | +36.0 | 13.6 | 1342 |
| +35 | 0.780 | 7.18 | 169 | 1.20 | 5067 | 10641 | 10493 | +50.9 | 8.1 | 1594 |

Dümen momenti, orsaya kaçma momentinden (−148 N·m) çok büyüktür; bu yüzden düz seyirde 0.2°'lik bir dümen yeter, manevrada ise birkaç derece tekneyi hızla döndürür. ≈ 20°'nin üzerinde dümen palası stall'a girer ($C_L$ = 1.2'de sınırlanır) ve hız da düştüğü için kuvvet azalır. Kinematik dönüş modeli bu stall'u içermez (bkz. Bölüm 8).

#### Örnek hesap (adım adım, tam apaz, δs = 46°, δr = +10°, U = 8 m/s)

1. Polar ve verim: $\rho = 1.15$, $\eta = 1$, $\kappa = 1 - 0.22\,(10/35)^2 = 0.982$ → $V^* = 8 \times 1.15 \times 1 \times 0.982 = 9.03$ m/s.
2. Yelken itkisi: $T = k_g\,\lambda\,U\rho\,\eta = 83.33 \times 1 \times 8 \times 1.15 \times 1 = 767$ N.
3. Dümen sürüklemesi: $D_d = 767 \times (1 - 0.982) = 14$ N. Gövde direnci: $R = 83.3 \times 9.03 = 753$ N. Toplam: $T - D_d - R = 0$ (kararlı hız).
4. Yelken toplam kuvveti: $F_N = 767 / \sin 46° = 1066$ N; yan kuvvet $F_{yan} = 1066 \cos 46° = 740$ N (sancağa, rüzgar altına).
5. Dümen: $C_L = 3.49 \times 0.1745 = 0.61$; $Y_d = \tfrac12 \times 1025 \times 0.16 \times 0.61 \times 9.03^2 = 4077$ N.
6. Dümen momenti: $N_d = 4077 \times 2.1 = 8562$ N·m (sancağa).
7. Yelken momenti: $N_y = -(+1) \times 740 \times 0.20 = -148$ N·m (iskeleye, rüzgara doğru).
8. Net moment: $8562 - 148 = 8414$ N·m → tekne sancağa döner: $\dot\psi = \frac{9.03}{4.6}\sin 10° \cdot \frac{180}{\pi} - 0.35 = 19.2$ °/s, $R = 27.0$ m.
9. Dönüş için gereken merkezcil kuvvet: $F_c = 250 \times 9.03 \times 0.335 = 757$ N.

### 3.2 Değiştirilebilir fiziksel parametreler ve türetilmiş büyüklükler

Arayüzün sol kenar çubuğundaki **"🔧 Tekne Fiziksel Parametreleri"** paneli, kodda `saildrone_model.Physics` sınıfında tutulan 13 parametreyi kullanıcıya açar. Bir değer değiştirildiğinde sayfa yeniden hesaplanır: serbest seyir simülasyonu, kuvvet–moment tablosu, hedefe navigasyon rotası ve navigasyon hesap tablosu (Bölüm 4.3) **aynı anda** yeni değerlerle güncellenir. "↩️ Varsayılan değerlere dön" düğmesi varsayılan tekneye döner; parametreler varsayılandan farklıyken panelde bir uyarı görünür.

| Grup | Parametre | Sembol | Varsayılan | Arayüz aralığı | Etkilediği büyüklükler |
|--------|----------------|------|--------|-------|-------------|
| Kütle ve direnç | Tekne + ekipman kütlesi | $m$ | 250 kg | 20–5000 | $\tau$, $F_c$ |
| | Gövde doğrusal direnç katsayısı | $k_g$ | 83.33 N·s/m | 5–2000 | $\tau$, $\lambda$, $\omega_{wh}$, $R$ |
| | Dümen sürükleme katsayısı | $c_{dd}$ | 0.22 | 0–0.9 | $\kappa$, $D_d$ |
| Yelken | Yelken alanı | $A_s$ | 20 m² | 1–200 | $\lambda$, $T$, $C_N$ |
| | Ağırlık merkezi – yelken basınç merkezi | $a_{CE}$ | 0.20 m | −3…3 | $e$, $N_y$, $\omega_{wh}$ |
| | Ağırlık merkezi – yanal direnç merkezi | $a_{LR}$ | 0 m | −3…3 | $e$, $N_y$, $\omega_{wh}$ |
| Gövde ve dümen | Efektif gövde boyu | $L$ | 4.6 m | 1–30 | $\dot\psi$, $R_{min}$, $v_{min}$, $\omega_{wh}$ |
| | Dümen alanı (iki pala) | $A_d$ | 0.16 m² | 0.01–5 | $Y_d$, $\omega_{wh}$ |
| | Ağırlık merkezi – dümen mesafesi | $x_d$ | 2.1 m | 0.1–15 | $N_d$, $\omega_{wh}$ |
| | Dümen kaldırma eğimi | $C_{L\alpha}$ | 3.49 1/rad | 0.5–6.28 | $C_L$, $\omega_{wh}$ |
| | Dümen stall sınırı | $C_{L,max}$ | 1.2 | 0.3–2.5 | $C_L$ |
| Akışkan | Hava yoğunluğu | $\rho_a$ | 1.225 kg/m³ | 0.9–1.5 | $C_N$ |
| | Su yoğunluğu | $\rho_{su}$ | 1025 kg/m³ | 990–1050 | $Y_d$, $\omega_{wh}$ |

Boyuna mesafeler ağırlık merkezinden ölçülür ve kıça doğru pozitiftir.

**Türetilmiş büyüklükler.** Hız poları $\rho(\theta)$ ve orsaya kaçma hızı $\omega_{wh} = 0.35$ °/s varsayılan tekne için kalibre edilmiştir. Parametreler değişince simülasyon aynı denklemleri şu türetilmiş büyüklüklerle kullanır:

$$\tau = \frac{m}{k_g}, \qquad \lambda = \frac{A_s}{A_{s,ref}}\cdot\frac{k_{g,ref}}{k_g}, \qquad e = a_{CE} - a_{LR}$$

Burada $A_{s,ref} = 20$ m² ve $k_{g,ref} = 83.33$ N·s/m, poların kalibre edildiği varsayılan teknenin değerleridir.

* **Surge zaman sabiti $\tau$:** ileri yön denklemi $m\,\dot v = T - D_d - k_g v$ doğrudan $\dot v = (V^*-v)/\tau$ verir. Kütle kararlı hızı değiştirmez, yalnızca tekneyi daha yavaş hızlandırır ve tack sırasında daha uzun süzülmesini sağlar.
* **Hız ölçeği $\lambda$:** yelken itkisi alanla orantılıdır, $T = c_T\,A_s\,U\rho\,\eta$ ($c_T = k_{g,ref}/A_{s,ref} = 4.17$ N·s/m³). Kararlı hızda $T\kappa = k_g V^*$ olduğundan $V^* = \lambda\,U\rho\,\eta\,\kappa$ elde edilir. Polar $\rho(\theta)$ böylece $\lambda$ ile ölçeklenir.
* **Orsaya kaçma hızı $\omega_{wh}$:** K4'teki yelken momenti, $\delta_{eq} = N_y / (x_d\,\tfrac12\rho_{su}A_dC_{L\alpha}v^2)$ kadarlık bir dümen sapmasına eşdeğerdir. Kinematik modelde bu sapma $(v/L)\,\delta_{eq}$ dönüş hızı üretir. Kararlı hızda $F_{yan} = T\cot\delta_s = k_g v\cot\delta_s/\kappa$ olduğundan hız sadeleşir:

$$\omega_{wh} = \frac{e\,k_g\cot\delta_s}{\kappa\,x_d\,\tfrac12\rho_{su}A_dC_{L\alpha}\,L} \;\propto\; \Phi = \frac{e\,k_g}{\rho_{su}\,A_d\,C_{L\alpha}\,x_d\,L}, \qquad \omega_{wh} = 0.35\ °/\text{s}\cdot\frac{\Phi}{\Phi_{ref}}$$

Modelde $\omega_{wh}$ seyir bandından bağımsız tek bir değer olduğu için tam apaz kalibrasyonu ($\delta_s = 46°$; formül 0.33 °/s verir) korunur ve oranla ölçeklenir. Böylece kuvvet modelindeki dengeleyici dümen açısı $\delta_{r,denge}$ ile kontrolcünün düz seyirde tuttuğu $\delta_{r,ss} = \arcsin(\omega_{wh}L/v)$ her parametre setinde uyumlu kalır. $e = 0$ olduğunda orsaya kaçma kalmaz. $e < 0$ olduğunda ise tekne rüzgar altına kaçar (lee helm).

* **Asgari dönüş yarıçapı** $R_{min} = L/\sin 35°$ ve **tack için asgari hız** $v_{min} = \dfrac{1.3\cdot65°\cdot L}{\tau\,\sin35°}$ (Bölüm 4, Adım 5) kenar çubuğunda ayrıca gösterilir.

Varsayılan değerlerde $\tau = 3$ s, $\lambda = 1$ ve $\omega_{wh} = 0.35$ °/s olur. Bu durumda raporun tüm sayısal sonuçları değişmez; simülasyon çıktısının değişiklikten önceki sürümle bit düzeyinde aynı olduğu kontrol edilmiştir.

**Duyarlılık örneği** (U = 8 m/s; her satırda yalnızca bir parametre değiştirilmiştir; S1 = Doğu 1 km tam apaz, S4 = Kuzey 1 km tam rüzgar üstü; varış yarıçapı 10 m):

| Değişiklik | τ (s) | λ | ω_wh (°/s) | V* tam apaz (m/s) | R_min (m) | Tack v_min (m/s) | S1 süre (s) | S4 süre (s) | S4 tack / gybe |
|------------|----|-----|-----|-----|-----|-----|----|-----|-----|
| Varsayılan | 3.00 | 1.000 | 0.350 | 9.20 | 8.0 | 3.94 | 108 | 258 | 4 / 0 |
| m = 500 kg | 6.00 | 1.000 | 0.350 | 9.20 | 8.0 | 1.97 | 108 | 256 | 4 / 0 |
| A_s = 30 m² | 3.00 | 1.500 | 0.350 | 13.80 | 8.0 | 3.94 | 72 | 196 | 5 / 1 |
| k_g = 120 N·s/m | 2.08 | 0.694 | 0.504 | 6.39 | 8.0 | 5.68 | 155 | ✘ (7200) | 0 / 425 |
| L = 3.0 m | 3.00 | 1.000 | 0.537 | 9.20 | 5.2 | 2.57 | 108 | 258 | 4 / 0 |
| a_CE = 0.40 m (e = 0.40 m) | 3.00 | 1.000 | 0.700 | 9.20 | 8.0 | 3.94 | 108 | 258 | 4 / 0 |
| A_d = 0.32 m² | 3.00 | 1.000 | 0.175 | 9.20 | 8.0 | 3.94 | 108 | 258 | 4 / 0 |
| c_dd = 0.50 | 3.00 | 1.000 | 0.350 | 9.20 | 8.0 | 3.94 | 108 | 260 | 4 / 0 |

* **Kütle** kararlı hızı değiştirmez. τ iki katına çıkınca tack eşiği yarıya iner.
* **Yelken alanı / gövde direnci** hızı λ ile ölçekler. A_s = 30 m²'de tam apaz hızı 13.8 m/s olur; rüzgar üstü hedefe ulaşma süresi 258 s'den 196 s'ye düşer. k_g = 120 N·s/m'de dar apaz hızı (4.72 m/s) tack eşiğinin (5.68 m/s) altına düşer. Tekne kontrayı yalnızca rüzgar altından dönerek değiştirebilir; 10 m varış yarıçapı dönüş çapından küçük kaldığı için de hedef çevresinde döner. Bu durum Bölüm 6.5'tekiyle aynıdır: varış yarıçapı 20 m yapıldığında tekne hedefe 436 s'de, 2 gybe ile ulaşır.
* **L, A_d, x_d ve e** yalnızca dönüşü ve orsaya kaçmayı etkiler. Navigasyon kontrolcüsü ileri beslemeyi yeni $\omega_{wh}$ ile yaptığından rota süresi değişmez; değişen, düz seyirde tutulan dümen açısıdır (ör. e = 0.40 m'de tam apazda 0.18° yerine 0.35°).
* **Kuvvet örneği (A_s = 30 m²,** tam apaz, δs = 46°, δr = +10°): T = 1150 N, F_N = 1599 N, F_yan = 1111 N, V* = 13.55 m/s, Y_d = 9173 N, N_d = 19 264 N·m, N_y = −222 N·m, δr,denge = +0.12°.

---

## 4. Hedefe Navigasyon: Kumaş Yelkenle Hedefe Gidiş (adım adım)

Navigasyonda üç açıyı artık **sistem atar**: hedef pruva $\psi^*$, yelken açısı $\delta_s$ ve dümen açısı $\delta_r$. Rota, başlangıç (B), istenen sayıda ara durak (1, 2, …) ve hedeften (H) oluşan bacaklara bölünür; **her bacakta aynı algoritma** çalışır ve tekne bir sonraki bacağa önceki bacağın son pruvası ve hızıyla girer. Aşağıdaki adımlar her güdüm periyodunda (1 s) tekrarlanır.

### Adım 1 — Hedefin kerterizi

Teknenin $(x, y)$ konumundan bacağın hedef noktasına $(x_h, y_h)$ pusula açısı:

$$\beta = \operatorname{atan2}(x_h - x,\; y_h - y) \quad (0° = \text{Kuzey})$$

### Adım 2 — Ölü açı kontrolü

$\theta(\beta) \le 30°$ ise hedef **ölü açının içindedir**: doğrudan hedefe dönülürse yelken yapraklar ve tekne durur. Bu durumda tekne hedefe zikzakla (kontra değiştirerek) gider. Kontrol, bir sonraki adımdaki VMG aramasında ölü açıya **5° güvenlik payı** bırakılarak otomatik olarak sağlanır (aranan pruvalar $\theta \ge 35°$).

### Adım 3 — Optimum pruva: VMG (hedefe doğru hız bileşeni)

Tekne hedefe doğru değil, **hedefe en hızlı yaklaştıran** pruvaya yönlendirilir. Bir $\psi$ pruvasının hedefe yaklaşma hızı:

$$\text{VMG}(\psi) = \underbrace{\lambda\,U\,\rho(\theta(\psi))}_{\text{teknenin hızı}}\cdot\cos(\psi - \beta)$$

($\lambda$ tüm pruvaları aynı oranda ölçeklediğinden optimum pruvayı değiştirmez, yalnızca VMG değerini değiştirir.)

Her kontra için en iyi pruva ayrı aranır ($s=+1$: $\psi = \theta$, $s=-1$: $\psi = 360° - \theta$):

$$\psi^*_{s} = \arg\max_{\theta \,\ge\, 35°} \ \text{VMG}(\psi_s(\theta))$$

**Rüzgar üstü hedef örneği ($\beta = 0°$, hedef tam kuzeyde):**

| Aday $\theta$ | Bant | $\rho$ | $\text{VMG}/U = \rho\cos\theta$ |
|---|---|---|---|
| 35.5° | Orsa | 0.62 | 0.505 |
| 40.0° | Orsa | 0.62 | 0.475 |
| **50.5°** | **Dar apaz** | **0.85** | **0.541 ← en iyi** |
| 60.0° | Dar apaz | 0.85 | 0.425 |
| 80.5° | Tam apaz | 1.15 | 0.190 |

Kumaş yelken orsada (0.62) yavaş olduğundan, rüzgara en dik açıyla (35.5°) gitmek yerine **50.5°'de dar apaz** gitmek hedefe daha hızlı yaklaştırır: $\psi^* = 050.5°$ (iskele kontra) veya $309.5°$ (sancak kontra). Benzer şekilde **rüzgar altı hedefte** ($\beta = 180°$) en iyi açı $\theta = 150°$'dir ($\text{VMG}/U = 0.92\cos30° = 0.797$); tam pupa ($0.74$) daha yavaştır, bu yüzden rüzgar altına da zikzakla (gybe ile) gidilir.

### Adım 4 — Kontra seçimi ve değiştirme kuralı

Tekne mevcut kontrasında kalır. Diğer kontranın VMG'si daha büyükse ve aşağıdakilerden biri sağlanırsa kontra değiştirilir:

1. **Hedef geride kaldı:** mevcut kontranın VMG'si $\le 0$.
2. **Layline:** diğer kontranın optimum pruvası $\psi^*_{-s}$ artık hedefi "tutuyor" — tekne o pruvaya dönerse hedefe tek bacakta ulaşır.
3. **Koridor:** tekne başlangıç–hedef hattından $W = 0.2\,D$ (50–1000 m arası) kadar uzaklaştı ve hâlâ uzaklaşıyor.

Gereksiz manevrayı önlemek için iki kontra değişimi arasında en az 10 s geçmelidir.

### Adım 5 — Manevra türü: tack mı, gybe mi?

Yeni pruvaya en kısa dönüş rüzgarın içinden (ölü açıdan) geçiyorsa bu bir **tack**'tir. Ölü açıda $V^* = 0$ olduğundan tekne yalnızca taşıdığı hızla döner; hız $v_0 e^{-t/\tau}$ ile söner. Tam dümenle dönülebilecek toplam açı:

$$\Theta_{max} = \int_0^\infty \frac{v_0\,e^{-t/\tau}}{L}\,\sin\delta_{max}\,dt = \frac{v_0\,\tau\,\sin\delta_{max}}{L}$$

Ölü açıyı ve 5° payı geçmek için gereken açı $2\cdot30° + 5° = 65°$'dir:

* Fiziksel alt sınır ($\Theta_{max} \ge 65°$): $v_0 \ge 3.03$ m/s.
* Navigasyonun karar ölçütü 1.3 güvenlik katsayısı içerir ($\Theta_{max} \ge 84.5°$): $v_0 \ge 3.94$ m/s. Bu sağlansa bile karar, manevranın ileri simülasyonuyla kesinleştirilir.
* Genel olarak $v_{min} = 1.3\cdot65°\cdot L / (\tau\sin\delta_{max})$'dır. Yani eşik kütle ($\tau = m/k_g$) ve gövde boyu ($L$) ile değişir (Bölüm 3.2).

Kumaş yelkenle bu, orsadan tack için yaklaşık **U ≥ 6.5 m/s** demektir (açık çevrim simülasyonu: 6.5 m/s'de tack başlangıç hızı 3.14 m/s). Tack mümkün değilse tekne **rüzgar altından ~290° dönerek (gybe)** kontra değiştirir; bu durumda değişim, manevra sonrasında hedefin hâlâ tutulabildiği ileri simülasyonla doğrulanınca yapılır.

### Adım 6 — Yelken açısının atanması (auto-trim)

Yelken her adımda anlık rüzgar açısına göre optimum açıya ve **rüzgar altı bordasına** açılır:

$$\delta_s = s(\psi)\cdot\delta_{opt}(\theta(\psi)) \;\Rightarrow\; \eta = 1$$

Örn. $\psi = 050.5°$ → $+32°$ (bom sancakta), $\psi = 309.5°$ → $-32°$ (bom iskelede). Tack sırasında $\theta \le 30°$ olduğundan $\delta_s = 0$ (yelken yapraklar), tack bitince bom karşı bordaya geçer.

### Adım 7 — Dümen açısının atanması (ters kinematik kontrolcü)

Pruva hatası $e = \psi^* - \psi$ (seçilen dönüş yönüne göre işaretli) bir istenen dönüş hızına, bu da Adım 6'daki dönüş denkleminin tersiyle dümen açısına çevrilir:

$$r^* = \operatorname{sat}\!\big(k\,e,\ \pm30°/\text{s}\big) + s(\psi)\,\omega_{wh}, \qquad \delta_r = \operatorname{sat}\!\left(\arcsin\frac{r^*\,L}{v},\ \pm35°\right), \qquad k = 0.5\ \text{s}^{-1}$$

* Doymasız bölgede kapalı çevrim $\dot e = -k\,e$ olur: pruva hatası 2 s zaman sabitiyle, **hızdan bağımsız** olarak söner.
* $s\,\omega_{wh}$ terimi orsaya kaçmayı önceden karşılar; bu yüzden düz seyirde dümen sıfır değil, küçük bir sabit açıdır: $\delta_{r,ss} = \arcsin(s\,\omega_{wh} L / v)$. Örn. $v = 6.8$ m/s'de $\delta_{r,ss} = +0.24°$ (iskele kontra).
* Tack/gybe sırasında hata büyük olduğundan dümen ±35° sınırına dayanır.

### Adım 8 — Varış ve başarısızlık

* **Ara durak / hedefe varış:** $\lVert p - p_h \rVert \le r_{varış}$ (varsayılan 10 m). Ara durakta tekne bir sonraki bacağın en iyi VMG kontrasını seçer.
* **Başarısızlık:** süre sınırı (tüm rota için, varsayılan 7200 s) aşılırsa veya tekne ölü açıda $v < 0.05$ m/s ile 20 s kalırsa.

### Akış özeti

```
Her güdüm periyodunda (1 s):
  β  ← hedef kerterizi                                   (Adım 1)
  ψ*₊, ψ*₋ ← her kontra için max VMG, θ ≥ 35°           (Adım 2–3)
  kontra değişsin mi? (VMG ≤ 0 | layline | koridor)      (Adım 4)
  dönüş yönü ← tack mümkünse rüzgar içinden, değilse gybe (Adım 5)
Her entegrasyon adımında (0.25 s):
  δs ← s(ψ)·δopt(θ)                                      (Adım 6)
  δr ← sat(asin(r*·L / v), ±35°)                         (Adım 7)
  v, ψ, x, y ← Bölüm 3 denklemleri
  varış / başarısızlık kontrolü                          (Adım 8)
```

### 4.1 Örnek: tam rüzgar üstü hedef (B = (0, 0) → H = (0, 1000) m, U = 8 m/s)

Hedef ölü açının tam ortasında ($\beta = 0°$). Tekne 50.5°'de dar apazla başlar, hedef yaklaştıkça layline'a göre 35.5° orsaya çıkar ve 4 tack ile 258 s'de hedefe ulaşır. Arayüzdeki **Açı Takip Tablosu**'nun bu test için ürettiği kayıt:

| # | Zaman (s) | Bacak | Olay | ψ* (°) | TWA (°) | Seyir | Kontra | δs (°) | δr manevra (°) | δr seyir (°) | Ort. hız (m/s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0–22 | B→H | Başlangıç | 50.5 | 50.5 | Dar Apaz | İskele | +32 | +0.0 | +0.2 | 6.80 |
| 2 | 22–50 | B→H | Rota düzeltme | 35.5 | 35.5 | Orsa | İskele | +16 | −4.8 | +0.2 | 5.16 |
| 3 | 50–109 | B→H | Tack | 309.5 | 50.5 | Dar Apaz | Sancak | −32 | −35.0 | −0.4 | 6.14 |
| 4 | 109–153 | B→H | Rota düzeltme | 324.5 | 35.5 | Orsa | Sancak | −16 | +4.8 | −0.2 | 5.09 |
| 5 | 153–203 | B→H | Tack | 50.5 | 50.5 | Dar Apaz | İskele | +32 | +35.0 | +0.4 | 6.02 |
| 6 | 203–229 | B→H | Rota düzeltme | 35.5 | 35.5 | Orsa | İskele | +16 | −4.8 | +0.1 | 5.18 |
| 7 | 229–245 | B→H | Tack | 279.5 | 80.5 | Tam Apaz | Sancak | −46 | −35.0 | −0.7 | 5.32 |
| 8 | 245–258 | B→H | Rota düzeltme | 309.5 | 50.5 | Dar Apaz | Sancak | −32 | +7.9 | +16.7 | 5.85 |

*δr manevra*: pruva yeni ψ\*'a oturana kadarki en büyük dümen açısı; *δr seyir*: manevra sonrası ortalama dümen açısı. Tack satırlarında dümen ±35°'ye dayanır, yelken işaret değiştirir (bom karşı bordaya geçer); düz seyirde dümen yalnızca orsaya kaçmayı karşılayan ≈ ±0.2°'dir. Son satır, hedefin hemen yakınındaki son yaklaşma manevrasıdır.

**Bu testte atanan açılarda kuvvetler ve momentler** (Bölüm 3.1 formülleri; yelken değerleri ψ*'daki kararlı hızda, dümen değerleri manevradaki en büyük dümen açısı ve bölüm ortalama hızıyla):

| # | Olay | ψ* (°) | δs (°) | T itki (N) | F_yan (N) | N_yelken (N·m) | δr maks (°) | v̄ (m/s) | Y_dümen (N) | N_dümen (N·m) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Başlangıç | 50.5 | +32 | 567 | 907 | −181 | 0.0 | 6.80 | 0 | 0 |
| 2 | Rota düzeltme | 35.5 | +16 | 413 | 1441 | −288 | −4.8 | 5.16 | −645 | −1355 |
| 3 | Tack | 309.5 | −32 | 567 | 907 | +181 | −35.0 | 6.14 | −3705 | −7780 |
| 4 | Rota düzeltme | 324.5 | −16 | 413 | 1441 | +288 | +4.8 | 5.09 | 627 | 1316 |
| 5 | Tack | 50.5 | +32 | 567 | 907 | −181 | +35.0 | 6.02 | 3562 | 7480 |
| 6 | Rota düzeltme | 35.5 | +16 | 413 | 1441 | −288 | −4.8 | 5.18 | −649 | −1363 |
| 7 | Tack | 279.5 | −46 | 767 | 740 | +148 | −35.0 | 5.32 | −2786 | −5850 |
| 8 | Rota düzeltme | 309.5 | −32 | 567 | 907 | +181 | +7.9 | 5.85 | 1351 | 2837 |

Tack'lerde dümen 5.9–7.8 kN·m'lik bir moment üretir; bu, yelkenin orsaya kaçma momentinin (≈ 150–290 N·m) 25–50 katıdır ve tekneyi ölü açının içinden geçirir. Orsaya çıkıldığında (ψ* = 35.5°) itki 567 N'den 413 N'ye düşer, yan kuvvet 907 N'den 1441 N'ye çıkar: tekne rüzgara daha dik gider ama daha yavaştır.


![S4 zaman serileri: pruva, yelken, dümen, hız](docs/figures/timeseries_S4_U8.png)

### 4.2 Örnek: ara duraklı kare rota (B → 1 → 2 → 3 → H, 600 m, U = 8 m/s)

| # | Zaman (s) | Bacak | Olay | ψ* (°) | TWA (°) | Seyir | Kontra | δs (°) | δr manevra (°) | δr seyir (°) | Ort. hız (m/s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0–64 | B→1 | Başlangıç | 90.0 | 90.0 | Tam Apaz | İskele | +46 | +0.0 | +0.2 | 9.20 |
| 2 | 64–75 | 1→2 | Ara durak 1 ✓ | 50.5 | 50.5 | Dar Apaz | İskele | +32 | −9.7 | −0.2 | 7.56 |
| 3 | 76–91 | 1→2 | Rota düzeltme | 35.5 | 35.5 | Orsa | İskele | +16 | −4.8 | +0.0 | 5.33 |
| 4 | 92–130 | 1→2 | Tack | 309.5 | 50.5 | Dar Apaz | Sancak | −32 | −35.0 | −0.4 | 5.80 |
| 5 | 130–155 | 1→2 | Rota düzeltme | 324.5 | 35.5 | Orsa | Sancak | −16 | +4.8 | −0.1 | 5.19 |
| 6 | 156–189 | 1→2 | Tack | 50.5 | 50.5 | Dar Apaz | İskele | +32 | +35.0 | +0.5 | 5.65 |
| 7 | 190–204 | 1→2 | Rota düzeltme | 35.5 | 35.5 | Orsa | İskele | +16 | −4.8 | −0.0 | 5.34 |
| 8 | 204–228 | 1→2 | Tack | 279.5 | 80.5 | Tam Apaz | Sancak | −46 | −35.0 | +12.1 | 4.98 |
| 9 | 229–299 | 2→3 | Ara durak 2 ✓ | 270.0 | 90.0 | Tam Apaz | Sancak | −46 | +28.3 | −0.1 | 8.83 |
| 10 | 299–328 | 3→H | Ara durak 3 ✓ | 150.0 | 150.0 | Geniş Apaz | İskele | +65 | −16.4 | +0.1 | 6.40 |
| 11 | 328–397 | 3→H | Rota düzeltme | 180.0 | 180.0 | Pupa | İskele | +82 | +12.0 | +0.2 | 5.92 |
| 12 | 397–406 | 3→H | Gybe | 240.0 | 120.0 | Tam Apaz | Sancak | −46 | +24.3 | +1.9 | 7.08 |

Doğuya giden ilk bacak tam apazdır (9.2 m/s, düz). Kuzeye giden ikinci bacak rüzgar üstüdür ve zikzakla (3 tack) geçilir. Batıya giden üçüncü bacak yine tam apazdır (bom bu kez iskelede, −46°). Güneye dönüşte tekne 150° geniş apaz ve pupa ile iner, hedefe gybe ile oturur.

![Kare rota](docs/figures/waypoint_square_U8.png)

### 4.3 Arayüzde hedefe navigasyon hesap tablosu

Hedefe Navigasyon ekranındaki **"🧮 Hedefe Navigasyon Hesap Tablosu"**, Serbest Seyir'deki kuvvet–moment tablosunun karşılığıdır. Adım 1–8'in formüllerini, kaydırıcıyla seçilen andaki ($t$) değerleriyle birlikte **Adım | Formül | Değer** biçiminde satır satır gösterir. Seçilen an, bacak içindeki son güdüm anına (1 s) yuvarlanır; ara durak veya hedefe varılan an yuvarlanmaz ve tabloda "Varış ✓" satırı görünür.

Değerler rotayı hesaplayan fonksiyonların aynısıyla (`navigation.guidance_breakdown` → `guidance_decision`, `_rudder_cmd`, `saildrone_model.step`) ve kenar çubuğundaki fiziksel parametrelerle (Bölüm 3.2) hesaplanır. "sim" etiketli değerler, simülasyonun kaydettiği bir sonraki adımdır. Testlerde (4 rota + özel parametreli 1 rota) 1823 güdüm anının tamamında formül sonucu ile simülasyon değeri birebir (10⁻⁹ hassasiyetle) aynı çıkmıştır.

**Örnek: 4.1 testinde ilk kontra değişimi anı ($t = 50$ s, varsayılan tekne, U = 8 m/s):**

| Adım | Formül | Değer ($t = 50$ s) |
|----------|---------------------|-------------|
| Fiziksel parametreler | $\tau = m/k_g,\ \lambda,\ \omega_{wh},\ L$ | τ = 3.00 s, λ = 1.000, ω_wh = +0.350 °/s, L = 4.60 m |
| Konum ve bacak | $\mathbf p = (x, y),\ \psi,\ v$ | B→H, (201.6, 211.1) m, ψ = 35.5°, v = 4.96 m/s |
| 1 · Hedefe uzaklık | $d = \lVert\mathbf p_h - \mathbf p\rVert$ | 814.3 m |
| 2 · Hedef kerterizi | $\beta = \operatorname{atan2}(x_h - x,\ y_h - y)$ | 345.7° |
| 3 · Ölü açı kontrolü | $\theta_\beta = \min(\beta, 360° - \beta) \le 30°$ ? | θ_β = 14.3° → hedef ölü açıda, zikzakla gidilir |
| 4 · VMG – iskele kontra ($s = +1$) | $\psi^*_+ = \arg\max_{\theta \ge 35°} \lambda U\rho(\theta)\cos(\theta - \beta)$ | ψ*₊ = 35.5°, V = 4.96 m/s, VMG = +3.20 m/s |
| 4 · VMG – sancak kontra ($s = -1$) | $\psi^*_- = \arg\max_{\theta \ge 35°} \lambda U\rho(\theta)\cos(360° - \theta - \beta)$ | ψ*₋ = 309.5°, V = 6.80 m/s, VMG = +5.49 m/s |
| 5 · Rota hattından sapma | $e_\perp = (\mathbf p - \mathbf p_B)\cdot\mathbf n,\ \ W = \operatorname{sat}(0.2D, 50, 1000)$ | e⊥ = +202 m, W = 200 m, hattan uzaklaşıyor |
| 6 · Layline | $\psi^*_{-s}$ hedefi tutuyor mu? | Hayır |
| 7 · Kontra kararı | $\text{VMG}_{-s} > \text{VMG}_s \wedge \big(\text{VMG}_s \le 0 \vee t_b \ge 10\,\text{s} \wedge (\text{layline} \vee \lvert e_\perp\rvert > W)\big)$ | **Kontra değişir: İskele → Sancak (tack)**; koşul: koridor dışı |
| 8 · Tack fizibilitesi | $\Theta_{max} = \dfrac{v\,\tau\sin\delta_{max}}{L} \ge 1.3\cdot65°$ | Θ_max = 106° ≥ 84.5° (v ≥ 3.94 m/s) → tack mümkün |
| 9 · Atanan pruva | $\psi^* = \psi^*_s$ | **309.5°** (sancak kontra, dar apaz) — sim: 309.5° |
| 10 · Yelken (auto-trim) | $\delta_s = s(\psi)\,\delta_{opt}(\theta(\psi))$ | anlık +16°; ψ*'a oturunca −32° |
| 11 · Pruva hatası | $e = \psi^* - \psi$ | −86.0° (kısa yol ölü açıdan geçer: tack) |
| 12 · İstenen dönüş hızı | $r^* = \operatorname{sat}(k\,e, \pm30°/\text{s}) + s\,\omega_{wh}$ | −30.00 + 0.35 = −29.65 °/s |
| 13 · Dümen açısı | $\delta_r = \operatorname{sat}\big(\arcsin(r^* L / v), \pm35°\big)$ | **−28.68°** — sim: −28.68° |
| 14 · Hedef hız | $V^* = \lambda U\rho(\theta)\,\kappa,\ \ \kappa = 1 - c_{dd}(\delta_r/35°)^2$ | ρ = 0.62, κ = 0.852 → V* = 4.23 m/s |
| 15 · Hız güncellemesi | $v^+ = V^* + (v - V^*)\,e^{-\Delta t/\tau}$ | 4.902 m/s — sim: 4.902 m/s |
| 16 · Dönüş | $\dot\psi = \dfrac{v^+}{L}\sin\delta_r - s\,\omega_{wh},\ \ \psi^+ = \psi + \dot\psi\,\Delta t$ | ψ̇ = −29.65 °/s → ψ⁺ = 28.09° — sim: 28.09° |
| 17 · Varış kontrolü | $d \le r_{varış}$ | 814 m > 10 m → devam |

Bu anda iskele kontrada kalmak hedefe 3.20 m/s ile yaklaştırırken sancak kontra 5.49 m/s VMG sunar. Tekne başlangıç–hedef hattından koridor yarı genişliği kadar (W = 200 m) uzaklaşmış ve hâlâ uzaklaşmaktadır, bu yüzden kontra değiştirilir. Tekne 4.96 m/s ile tack eşiğinin (3.94 m/s) üzerinde olduğundan dönüş ölü açının içinden (tack) yapılır. Dümen istenen −29.65 °/s'lik dönüş hızını verecek açıya (−28.68°) gelir; bu açıdaki dümen sürüklemesi hızı 4.96'dan 4.90 m/s'ye düşürür. Açı Takip Tablosu'ndaki 3. satır (Tack, 50–109 s) bu karardan başlar.

---

## 5. Arayüz: Hedefe Navigasyon ekranı

Serbest Seyir ekranındaki kullanım kolaylığı Hedefe Navigasyon ekranına taşınmıştır:

| Özellik | Açıklama |
|---|---|
| Kaydırıcı + kutucuk | Rüzgar hızı, başlangıç/hedef X–Y, varış yarıçapı, süre sınırı ve (isteğe bağlı) başlangıç pruvası hem kaydırıcıyla hem klavyeyle girilir; ikisi senkron kalır. |
| Sürükle-bırak | Haritadaki başlangıç (B), hedef (H) ve ara duraklar fareyle sürüklenir; bırakınca koordinat kutucukları güncellenir ve rota yeniden hesaplanır. |
| Ara duraklar | Haritada çift tıklama veya "➕ Ara durak ekle" ile eklenir, sağ tık veya "➖" ile silinir, tablodan elle düzenlenir. Hazır şablonlar: kare, üçgen, altıgen, daire (12 nokta), yay. |
| Açı Takip Tablosu | Her yeni testte sıfırlanır; oynatma ilerledikçe rota bacaklarında sistemin atadığı pruva ψ*, TWA/seyir tipi, yelken δs ve dümen δr (manevra + seyir) satır satır eklenir, etkin satır vurgulanır. CSV olarak indirilebilir. |
| Hızlandırılmış oynatma | Rota baştan sona ×1–×200 hızla oynatılır (varsayılan: görev ≈ 20–25 s'de); duraklat, baştan oynat ve zaman çubuğuyla istenen ana gidilebilir. |
| Canlı göstergeler | Zaman, anlık hız (m/s, kts), pruva ψ ve atanan ψ*, yelken δs, dümen δr, seyir tipi/TWA, aktif bacak ve kalan mesafe oynatma boyunca canlı akar. |
| Kuvvet ve moment paneli | Serbest Seyir ekranında girilen pruva, yelken ve dümen açısı için τ, λ, V*, itki, yan kuvvet, direnç, dümen kuvveti, momentler, ω_wh ve dönüş formülleriyle birlikte, kenar çubuğundaki parametrelerle canlı hesaplanır. |
| Fiziksel parametreler paneli | Sol kenar çubuğunda 13 parametre (Bölüm 3.2) ve türetilmiş büyüklükler tablosu (τ, λ, e, ω_wh, R_min, tack v_min). Değişiklik iki ekrandaki simülasyonu ve tabloları anında günceller; varsayılandan farklıysa uyarı verilir, "Varsayılan değerlere dön" düğmesiyle sıfırlanır. |
| Navigasyon hesap tablosu | Hedefe Navigasyon ekranında, kaydırıcıyla seçilen andaki 17 adımın formülleri ve değerleri; simülasyonun kaydettiği değerlerle çapraz kontrol (Bölüm 4.3). |
| Katamaran animasyonu | Haritadaki tekne ve "yakın görünüm" penceresinde yelkenin (mavi) ve dümen palalarının (kırmızı) anlık açıları, ölü açı dilimi ve atanan pruva ile birlikte dinamik olarak çizilir. |

Uygulama: `streamlit run app.py`. Harita bileşeni `components/nav_player/index.html` dosyasındadır (dış bağımlılık yoktur).

---

## 6. Testler ve sonuçlar

### 6.1 Birim testleri (124 / 124 ✔)

`tests/test_model.py` (59 test) — ölü açıda itki sıfır; iskele/sancak simetrisi; kontra adlandırması; trim verimi tepe değeri ve sınırları; dümen cezası; düz seyirde yol = hız × süre; telemetri hızı = dinamik hız; pozitif dümen → sancağa dönüş; orsaya kaçma yönü; ölü açıda başlayan tekne hareketsiz; surge yakınsaması; auto-trim'in dönüş boyunca yeniden trim yapması; **kararlı hızda kuvvet dengesi (T = D_d + R), net kuvvetin m(V\*−v)/τ olması, yelken kuvveti bileşenleri, moment işaretleri ve dengeleyici dümen açısı, ölü açıda kuvvetlerin sıfır olması;** *fiziksel parametreler:* varsayılan tekne kalibre sabitleri (τ = 3 s, λ = 1, ω_wh = 0.35 °/s) aynen üretir; hız yelken alanıyla doğru, gövde direnciyle ters orantılı; kütle kararlı hızı değiştirmeyip yalnızca τ'yu büyütür ve hızlanmayı yavaşlatır; c_dd dümen cezasını belirler; L dönüş hızını, e / x_d orsaya kaçmayı ölçekler (e = 0 → denge, e < 0 → lee helm); 4 farklı parametre setinde kinematik ω_wh kuvvet modelindeki dengeleyici dümen açısıyla uyumludur; kuvvet tablosu verilen parametreleri kullanır.

`tests/test_navigation.py` (65 test) — 24 senaryonun tamamı; dümen sınırı ve ölü açıda geçen sürenin < %5 olması; rüzgar üstünde tack gerekliliği; hafif rüzgarda gybe; ölü açıda başlangıcın başarısızlık olarak raporlanması; sıfır mesafe; süre sınırı; 10 km menzil; 12 yönde erişilebilirlik; tack fizibilitesi; layline geometrisi; **ara duraklı rotada tüm durakların ziyareti; rota süre sınırı; yelken açısının rüzgar altı bordasında olması; açı takip tablosunun tack ve ara durakları kaydetmesi;** *hesap tablosu ve parametreler:* hesap tablosunun 4 rotada (rüzgar üstü 8 ve 4 m/s, ara duraklı kare, verilen başlangıç pruvası) ve özel parametreli bir rotada her güdüm anında atanan pruvayı, dümen açısını, bir sonraki hızı ve pruvayı simülasyonla birebir üretmesi; varış anının yuvarlanmadan "varış" olarak gösterilmesi; 5 farklı parametre setinde 3 yöndeki hedefe ulaşılması; büyük yelkenle hedefe daha kısa sürede varılması.

### 6.2 Senaryo matrisi (24 / 24 ✔)

S1 (0,0)→(1000,0) tam apaz · S2 (0,0)→(0,−1000) tam rüzgar altı · S3 (0,0)→(700,700) · S4 (0,0)→(0,1000) tam rüzgar üstü · S5 (0,0)→(300,1000) ölü açı içinde (017°) · S6 (500,−300)→(−400,800).

| U (m/s) | S1 | S2 | S3 | S4 | S5 | S6 |
|---|---|---|---|---|---|---|
| 1 | ✔ 861 s | ✔ 1357 s | ✔ 1668 s | ✔ 2419 s | ✔ 2312 s | ✔ 2419 s |
| 3 | ✔ 287 s | ✔ 453 s | ✔ 559 s | ✔ 808 s | ✔ 772 s | ✔ 809 s |
| 8 | ✔ 108 s | ✔ 171 s | ✔ 198 s | ✔ 258 s | ✔ 268 s | ✔ 288 s |
| 15 | ✔ 58 s | ✔ 92 s | ✔ 113 s | ✔ 163 s | ✔ 144 s | ✔ 160 s |

* **Tam apaz en hızlı yöndür**; aynı mesafe için tam rüzgar üstü ≈ 2.4×, tam rüzgar altı ≈ 1.6× daha uzun sürer.
* 1 ve 3 m/s'de tekne hızı tack eşiğinin altında kaldığından kontra değişimleri gybe ile yapılır ve görev yine tamamlanır.
* Tam tablo (yol, rota verimi, tack/gybe sayıları): [docs/results/scenario_matrix.md](docs/results/scenario_matrix.md).

![Rotalar, U = 8 m/s](docs/figures/trajectories_U8.png)

![Hafif ve kuvvetli rüzgarda tam rüzgar üstü](docs/figures/upwind_light_vs_strong.png)

### 6.3 360° erişilebilirlik (48 / 48 ✔)

1 km yarıçaplı çember üzerinde 15°'de bir 24 hedef, 3 ve 8 m/s. Grafik "etkin VMG = 1000 m / varış süresi" değerini gösterir: ölü açı yönündeki hedefler (000°, 015°, 345°) en yavaş, 090°–120° ve 240°–270° (tam apaz) en hızlıdır. Tablo: [docs/results/reachability.md](docs/results/reachability.md).

![Erişilebilirlik](docs/figures/reachability.png)

### 6.4 Uç durumlar (U = 8 m/s, aksi belirtilmedikçe)

| Durum | Ulaştı | Süre (s) | Tack | Gybe | Sonuç |
|---|---|---|---|---|---|
| Başlangıç pruvası ölü açıda (000°), hedef Doğu 1 km | ✘ | 20 | 0 | 0 | Ölü açıda takıldı — hız sıfır, dümen etkisiz |
| Başlangıç pruvası ölü açı sınırında (032°), hedef Kuzey 1 km | ✔ | 261 | 4 | 0 | — |
| Başlangıç pruvası ters yönde (180°), hedef Kuzey 1 km | ✔ | 268 | 4 | 0 | — |
| Tack yasak, varış yarıçapı 10 m, Kuzey 1 km | ✘ | 7200 | 0 | 435 | Süre sınırı (hedef çevresinde döngü) |
| Tack yasak, varış yarıçapı 30 m, Kuzey 1 km | ✔ | 308 | 0 | 2 | — |
| Hafif rüzgar 2 m/s (tack fiziksel olarak imkânsız), Kuzey 1 km | ✔ | 1209 | 0 | 2 | — |
| Hedef başlangıçta (mesafe 0) | ✔ | 0 | 0 | 0 | — |
| Uzun menzil: Kuzey 10 km, 5 m/s | ✔ | 4485 | 2 | 3 | — |
| Kısa süre limiti: Kuzey 1 km, 3 m/s, 300 s | ✘ | 300 | 0 | 0 | Süre sınırı |

### 6.5 Tack yapılamıyorsa: varış yarıçapı duyarlılığı (Kuzey 1 km)

| Varış yarıçapı (m) | U=3 | U=5 | U=8 | U=12 | U=15 |
|---|---|---|---|---|---|
| *Dönüş çapı 2R (m)* | *16* | *16* | *19* | *28* | *36* |
| 10 | ✔ 808 s | ✔ 885 s | ✘ | ✘ | ✘ |
| 20 | ✔ 803 s | ✔ 484 s | ✔ 310 s | ✘ | ✘ |
| 30 | ✔ 768 s | ✔ 481 s | ✔ 308 s | ✔ 232 s | ✔ 289 s |
| 40 | ✔ 758 s | ✔ 456 s | ✔ 291 s | ✔ 214 s | ✔ 177 s |

Hedefe rüzgar altından dönüşle yaklaşılıyorsa varış yarıçapı dönüş çapı mertebesinde olmalıdır; aksi halde manevra tekneyi hedefin rüzgar altına iter ve hedef çevresinde döngü oluşur. Normal modda (tack serbest) bu durum yalnızca tekne hızı tack eşiğinin altındayken oluşur; o hızlarda $R \approx 8$ m olduğundan varsayılan 10 m yeterlidir.

---

## 7. Sonuç

Kumaş yelken modeli ve navigasyon kontrolcüsüyle tekne, test edilen 72 başlangıç–hedef–rüzgar kombinasyonunun (24 senaryo + 48 erişilebilirlik) tamamında ve ara duraklı rotalarda hedefe ulaşmıştır. Hedef ölü açının içinde olsa bile sistem, VMG-optimum pruvayı (rüzgar üstünde 50.5°/35.5°, rüzgar altında 150°) seçip tack veya gybe ile zikzak yapar; her an yelkeni rüzgar altı bordasında optimum açıya açar ve dümeni ters-kinematik kontrolcüyle ±35° sınırında tutar.

Ulaşılamayan durumlar: (1) başlangıç pruvası ölü açıda ve tekne durgunsa, (2) süre sınırı mesafe/etkin VMG'den kısaysa, (3) tack yapılamıyor ve varış yarıçapı dönüş çapından küçükse.

Teknenin fiziksel parametreleri artık arayüzden değiştirilebilir. Simülasyon, parametrelere bağlı türetilmiş büyüklükler (τ = m/k_g, λ, ω_wh, L) üzerinden aynı denklemlerle çalışır; varsayılan değerlerde sonuçlar değişmez. Hedefe navigasyonun her adımı arayüzdeki hesap tablosunda formülü ve sayısal değeriyle izlenebilir. Tablo, simülasyonun kendisini birebir yeniden üretir.

---

## 8. Model sınırlamaları

| Sınırlama | Etkisi | Öneri |
|---|---|---|
| Parçalı sabit polar | VMG-optimum pruvalar bant kenarlarına (35.5°, 50.5°, 80.5°, 150°) düşer | Poları $\theta$'ya göre sürekli (spline) yapmak |
| $V_b \propto U$ (doygunluk yok) | Kuvvetli rüzgarda gerçekçi olmayan hızlar | Rüzgarla doyuran polar, reef |
| Yan kayma (leeway), akıntı, dalga yok | Rota suya göredir | Leeway açısı ve akıntı vektörü |
| Yaw ataleti yok (kinematik dönüş) | Manevralar gerçekte daha yavaş | $I_z\dot r = N_\delta\,\delta_r - N_r\,r$ |
| Ölü açıda durgun tekne kurtulamaz | Uç durum 1 | Rüzgarla sürüklenme / backing modeli |
| Rüzgar altı polar hız oranları iyimser | Kuvvet kontrolünde geniş apaz/pupada $C_N$ > 1.5 (Bölüm 3.1) | Poları görünür rüzgara göre yeniden kalibre etmek |
| Yapraklama sınırı gerçek rüzgara göre ($\alpha = \theta - \lvert\delta_s\rvert$) | Kararlı hızda dar/tam apazda AWA (27°/41°, Bölüm 3.1) polar optimum bom açısından (32°/46°) küçüktür; görünür rüzgara göre yelken bu açılarda zaten yapraklama sınırında olurdu | $\delta_{opt}$'u görünür rüzgardan türetmek ($\delta_{opt} = \text{AWA} - \alpha_{opt}$) ve poları buna göre yeniden kalibre etmek |
| Fiziksel parametreler arayüzden değiştirilebilir, ancak hız poları varsayılan tekneye (A_s = 20 m², k_g = 83.33 N·s/m) kalibredir ve λ ile ölçeklenir; gövde direnci doğrusaldır | Hız A_s/k_g ile doğru orantılı artar (gerçekte karesel dirençle ≈ √(A_s/k_g)); varsayılandan büyük sapmalarda hız fazla tahmin edilir | Poları ve direnci tekne ölçüleri / deney verisiyle yeniden kalibre etmek, karesel direnç |
| ω_wh tam apazda kalibre edilip seyir bandından bağımsız alınır | Orsada (cot δs büyük) gerçek orsaya kaçma daha güçlüdür | ω_wh'yi δs ve hıza bağlamak |
| Kinematik dönüş dümen stall'unu içermez | 20°'nin üzerinde dönüş hızı fazla tahmin edilir; dümen alanı dönüş hızını değil yalnızca kuvvet/momenti ve ω_wh'yi etkiler | Dümen kuvvetine dayalı yaw dinamiği |
| $\tau = m/k_g$ (varsayılan 3 s) varsayımdır | Tack eşiğini belirler ($v_{min} \propto L/\tau$) | Gerçek tekne verisiyle tanımlama |

---

## 9. Yeniden üretme

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

Grafikler `docs/figures/`, tablolar `docs/results/` altına yeniden yazılır (açı takip tabloları: `angle_log_S4_U8.md`, `angle_log_square_U8.md`). Word sürümü bu dosyadan üretilir:

```bash
pandoc RAPOR.md -o RAPOR.docx
```

---

## Ek A — Önceki sürümde düzeltilen hatalar (özet)

Kanıtlar: [docs/results/legacy_bug_evidence.md](docs/results/legacy_bug_evidence.md) (`python analysis/legacy_bug_evidence.py`).

| # | Hata | Düzeltme |
|---|---|---|
| H1 | Kontra adları ters (doğuya giden teknede "Sancak Kontra") | Ad, rüzgarın geldiği bordadan üretiliyor |
| H2 | İskele/sancak sınır asimetrisi (aynı TWA farklı bant) | Önce $\theta$ hesaplanıp tek bant tablosu kullanılıyor |
| H3 | Ekrandaki hız ile simülasyondaki hız farklı formüllerden (≈ %47'ye varan fark) | Tek `trim_efficiency()` fonksiyonu |
| H4 | Auto-trim dönüşte yeniden trim yapmıyordu | Bom her adımda optimuma geliyor |
| H5 | Orsaya kaçma, dümen 2°'nin altındayken açılıp kapanıyordu (süreksiz) | Moment her zaman var, dümen karşılıyor |
| H6 | İleri yön ataleti yoktu: ölü açıya giren tekne kilitleniyordu, tack imkânsızdı | Surge dinamiği ($\tau$ = 3 s) |
| H7 | Yörünge etiketi son pruvayı başlangıç hızıyla gösteriyordu | "Son pruva / ortalama hız" |
| H8 | Fizik kodu arayüzle iç içeydi (test edilemez) | `saildrone_model.py`, `navigation.py` ayrıldı |
| H9 | Bom rüzgar yönüne kadar boşlansa da tekne ilerliyordu (ör. $\psi$ = 055°, $\delta_s$ = 56°: 30 s'de 41.7 m, orsaya kaçmayla pruva 045°'ye dönüyordu). Fazla açık dalın 0.10 tabanı yelkenin rüzgarı görüp görmediğine bakmıyordu | Hücum açısı $\alpha = \theta - \lvert\delta_s\rvert \le 0$ → $\eta = 0$; $0 < \alpha < 8°$ → $\eta \le \alpha/8°$ (Adım 3) |
| H10 | Ölü açı yalnızca pruvaya göre sabit 330°–030° alınıyordu; bom açısı hesaba katılmıyordu | Ölü açı $\pm\max(30°, \lvert\delta_s\rvert)$ (Adım 1); arayüzdeki kama, etiketler ve seyir tipi bom açısıyla güncelleniyor |
