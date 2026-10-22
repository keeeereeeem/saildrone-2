"""
Otonom yelkenli katamaran fizik modeli – kuvvet tabanlı (VPP) çekirdek (Streamlit'ten bağımsız, test edilebilir).
Tek yelken modeli: Kumaş Yelken (Cambered / Klasik). Üç sekme (serbest seyir, hedefe navigasyon, düz rota PID)
aynı denklemleri kullanır.

Koordinat ve işaret kuralları
-----------------------------
* Dünya ekseni: x = Doğu (m), y = Kuzey (m). Pruva ψ: pusula açısı (0° = Kuzey, saat yönünde artar).
* Rüzgar sabittir ve KUZEYDEN eser. Gerçek rüzgar açısı TWA θ = min(ψ, 360 − ψ).
* ψ ∈ (0°, 180°): rüzgar iskeleden gelir (iskele kontra), bom ve yan kayma sancağa (s = +1);
  ψ ∈ (180°, 360°): sancak kontra (s = −1). Ölü açıda s = 0.
* Dümen δr > 0: pala sancağa açılır, tekne sancağa döner (ψ artar).

Fizik (her adımda)
------------------
1. Görünür rüzgar: V_A, AWA β (teknenin kendi hızıyla).
2. Yelken: hücum açısı α = β − |δs|. Kumaş yelkenin C_L(α), C_D(α) eğrileri; α ≤ 0'da yelken rüzgara
   paralel/ters kalır ve yapraklar (kaldırma yok, yalnızca çırpınma sürüklemesi). θ ≤ 30°'de (ölü açı)
   yelken doldurulamaz. İleri itki X = q·A(C_L sin β − C_D cos β), yan kuvvet Y = q·A(C_L cos β + C_D sin β),
   q = ½ρ_hava V_A².
3. Auto-trim: bom açısı her an net itkiyi X − R_i(Y) en büyük yapan açıya getirilir (görünür rüzgara göre; katsayı
   eğrileri parçalı doğrusal olduğundan aday bomlar δs = AWA − α_k düğümleridir, kesin aranır).
4. Gövde direnci: R = ½ρ_su S C_h v² (C_h sabit: sürtünme + şekil).
5. Boyutlandırma aşaması (varsayılan kapalı, tekne ölçüleri belli olunca açılır): devrilme sınırı
   Y·h ≤ m·g·B/2 (aşarsa yelken etkin alanı f = Y_maks/Y oranında azaltılır) ve boya bağlı dalga direnci tepesi
   C_R(Fr ≈ 0.5).
6. Yan kayma: yan kuvveti salma (hançer) kaldırması + gövdelerin çapraz akış direnci karşılar:
   ½ρ_su(A_s C_Lα,s v·v_yan + A_g C_D,ç v_yan²) = |Y|; yan kayma direnci R_i = |Y|·sin λ, λ = atan(v_yan / v).
7. Dümen sürüklemesi: D_d = ½ρ_su v² A_d [C_D0 + C_L(δr)² / (π·AR·e) + stall sonrası ayrılmış akış direnci].
8. İleri hız: m·dv/dt = X − R − R_i − D_d.
9. Dönüş: T·ṙ + r = (v/L)·sin δ_e + r_yelken,  T = T'·L/v (Nomoto); δ_e = C_L(δr)/C_Lα (stall'da doyar);  r_yelken = −s·Y·e_CE / (½ρ_su A_d C_Lα x_d L·v)
   (yelken momentinin dümen kinematiğiyle aynı ölçekte dönüş hızı: orsaya kaçma). Dümen servosu |δ̇r| ≤ δ̇_maks.
10. Konum: ẋ = v·sin ψ + s·v_yan·cos ψ,  ẏ = v·cos ψ − s·v_yan·sin ψ.

Kararlı hız (polar): en iyi trimde X(v) = R(v) + R_i(v) + D_d(v) denkleminin en büyük kararlı kökü.

Boyuttan bağımsızlık: boyutlandırma seçenekleri kapalıyken bütün kuvvetler (yelken, gövde, yan kayma, dümen)
hızın karesiyle ölçeklenir. Bu yüzden kararlı hız / rüzgar oranı V/U, en iyi bom açısı ve yan kayma açısı
yalnızca alan ve katsayı oranlarına bağlıdır: rüzgar hızından, tekne boyutundan ve kütleden bağımsızdır. Kütle
yalnızca ivmelenmeyi ve süzülmeyi (tack), gövde boyu yalnızca dönüş yarıçapını ve dönüş ataletini etkiler.
Varsayılan değerler ~50 kg'lık mürettebatsız otonom katamaran için tipik değerlerdir (ölçüler henüz belli değil);
tümü arayüzden değiştirilebilir.
"""
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

# --- SABİTLER ---
NO_GO_DEG = 30.0            # Ölü açı yarı genişliği (TWA ≤ 30° -> yelken doldurulamaz)
RUDDER_MAX_DEG = 35.0       # Dümen mekanik sınırı
MIN_STEER_SPEED = 0.2       # Bu hızın altında dümen ve yelken momenti dönüş üretmez (m/s)
DT = 0.25                   # Varsayılan entegrasyon adımı (s)
G = 9.81
MS_TO_KTS = 1.94384

PROFILE = "cloth"           # Tek standart yelken: kumaş (cambered) yelken
PROFILE_KEYS = (PROFILE,)
PROFILE_NAME = "Kumaş Yelken (Cambered / Klasik)"

# TWA bantları (seyir adları): (alt, üst] derece
BANDS = [
    (30.0, 50.0, "Orsa", "Close-Hauled"),
    (50.0, 80.0, "Dar Apaz", "Close Reach"),
    (80.0, 120.0, "Tam Apaz", "Beam Reach"),
    (120.0, 150.0, "Geniş Apaz", "Broad Reach"),
    (150.0, 180.0, "Pupa", "Running"),
]
DESC = [
    "ORSA: görünür rüzgar pruvaya yakın; bom orta hatta yakın kasılır. Yan kuvvet büyük, itki küçüktür; "
    "yan kayma ve orsaya kaçma en belirgin bu seyirdedir.",
    "DAR APAZ: bom biraz boşlanır; itki artar, yan kuvvet hâlâ büyüktür.",
    "TAM APAZ: yelken en verimli seyirlerinden biri; hızlandıkça görünür rüzgar pruvaya kaydığından bom gerçek "
    "rüzgara göre beklenenden daha kasılı tutulur.",
    "GENİŞ APAZ: bom iyice açılır; yan kuvvet azalır, itki kaldırma + sürüklemeden gelir.",
    "PUPA: yelken paraşüt gibi rüzgara dik durur; itki sürüklemeden gelir ve görünür rüzgar zayıfladığı için "
    "tekne yavaşlar.",
]
NO_GO_REGIME = "Ölü Açı (No-Go Zone)"

# --- KUMAŞ YELKEN KUVVET KATSAYILARI (ana yelken, en-boy oranı ≈ 3; Marchaj tipi yaklaşık eğriler) ---
SAIL_ALPHA = np.array([0, 2, 5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90], dtype=float)
SAIL_CL = np.array([0.0, 0.25, 0.60, 1.00, 1.25, 1.40, 1.45, 1.40, 1.25, 1.10, 0.90, 0.65, 0.35, 0.0])
SAIL_CD = np.array([0.10, 0.08, 0.06, 0.08, 0.12, 0.18, 0.26, 0.36, 0.60, 0.85, 1.05, 1.20, 1.30, 1.33])
CD_FLOG = 0.10              # yapraklayan (çırpınan) kumaşın sürükleme katsayısı
SAIL_GRID = np.arange(0.0, 90.0 + 1e-9, 0.5)     # bom ızgarası (°): devrilme sınırı açıkken ek arama noktaları


def _natural_spline(xk, yk, x):
    """Doğal kübik spline (düğümlerden geçen, eğimi sürekli eğri)."""
    n, h = len(xk), np.diff(xk)
    a = np.zeros((n, n))
    r = np.zeros(n)
    a[0, 0] = a[-1, -1] = 1.0
    for i in range(1, n - 1):
        a[i, i - 1], a[i, i], a[i, i + 1] = h[i - 1], 2.0 * (h[i - 1] + h[i]), h[i]
        r[i] = 6.0 * ((yk[i + 1] - yk[i]) / h[i] - (yk[i] - yk[i - 1]) / h[i - 1])
    mm = np.linalg.solve(a, r)
    i = np.clip(np.searchsorted(xk, x) - 1, 0, n - 2)
    t, hi = x - xk[i], h[i]
    return (yk[i] + ((yk[i + 1] - yk[i]) / hi - hi * (2.0 * mm[i] + mm[i + 1]) / 6.0) * t
            + mm[i] / 2.0 * t * t + (mm[i + 1] - mm[i]) / (6.0 * hi) * t ** 3)


# Düğüm değerleri 0.5° aralıklı düzgün tabloya çevrilir: eğimi sürekli eğriler sayesinde en iyi bom pruva ve hızla
# düzgün değişir (parçalı doğrusal tabloda en iyi bom düğümden düğüme atlar).
SAIL_ALPHA_F = np.arange(0.0, 90.0 + 1e-9, 0.5)
SAIL_CL_F = np.maximum(_natural_spline(SAIL_ALPHA, SAIL_CL, SAIL_ALPHA_F), 0.0)
SAIL_CD_F = np.maximum(_natural_spline(SAIL_ALPHA, SAIL_CD, SAIL_ALPHA_F), 0.05)
RUDDER_AR, RUDDER_E, RUDDER_CD0 = 2.5, 0.9, 0.01


# --- TEKNENİN FİZİKSEL PARAMETRELERİ (arayüzden değiştirilebilir) ---
@dataclass(frozen=True)
class Physics:
    # Kütle ve gövde
    mass: float = 50.0               # tekne + ekipman kütlesi m (kg): yalnızca ivmelenme ve süzülmeyi etkiler
    hull_length: float = 2.5         # su hattı boyu L (m): dönüş kinematiği ve dönüş ataleti
    wetted_area: float = 1.5         # iki gövdenin ıslak yüzeyi S (m²)
    hull_cd: float = 0.004           # gövde direnç katsayısı C_h (sürtünme + şekil, boyutsuz): R = ½ρ_su S C_h v²
    # Yelken
    sail_area: float = 3.0           # kumaş yelken alanı A (m²)
    ce_arm: float = 0.05             # yelken basınç merkezinin ağırlık merkezine boyuna uzaklığı a_CE (m, kıça +)
    clr_arm: float = 0.0             # yanal direnç merkezinin ağırlık merkezine boyuna uzaklığı a_LR (m, kıça +)
    # Yanal direnç
    board_area: float = 0.16         # salma / hançer yanal alanı A_s (m², iki hançer)
    board_cla: float = 3.6           # hançer kaldırma eğimi C_Lα,s (1/rad)
    hull_side_area: float = 0.5      # iki gövdenin su altı yan alanı A_g (m²): çapraz akış
    cross_cd: float = 1.0            # çapraz akış direnç katsayısı C_D,ç
    # Dümen
    rudder_area: float = 0.032       # iki dümen palasının toplam alanı A_d (m²)
    rudder_cla: float = 3.49         # dümen kaldırma eğimi C_Lα (1/rad), en-boy oranı 2.5
    rudder_clmax: float = 1.2        # dümen stall sınırı C_L,maks
    rudder_arm: float = 1.1          # dümenin ağırlık merkezine uzaklığı x_d (m)
    rudder_rate: float = 60.0        # dümen servosunun en büyük açılma hızı (°/s): küçük otonom teknede elektrikli servo
    yaw_t_nd: float = 2.0            # boyutsuz Nomoto zaman sabiti T' (T = T'·L/v)
    # Akışkan
    rho_air: float = 1.225           # hava yoğunluğu (kg/m³)
    rho_water: float = 1025.0        # deniz suyu yoğunluğu (kg/m³)
    # Boyutlandırma aşaması (tekne ölçüleri belli olunca): varsayılan kapalı. Kapalıyken bütün kuvvetler hızın
    # karesiyle ölçeklenir; kararlı hız / rüzgar oranı rüzgar hızından, tekne boyutundan ve kütleden bağımsızdır.
    heel_limit: bool = False         # devrilme sınırı: Y·h ≤ m·g·B/2 (açıkken yelken etkin alanı azaltılır)
    beam: float = 1.2                # gövde merkezleri arası B (m): doğrultma momenti m·g·B/2
    heel_arm: float = 1.5            # yelken basınç merkezinin yanal direnç merkezine yüksekliği h (m)
    wave_cr: float = 0.0             # dalga direnci tepesi C_R,maks (Fr = v/√(gL) ≈ 0.5'te; ince gövde ≤ 0.003)

    @property
    def ce_offset(self):
        """e = a_CE − a_LR: yelken basınç merkezi yanal direnç merkezinin gerisinde (+) → orsaya kaçma."""
        return self.ce_arm - self.clr_arm

    @property
    def righting_moment(self):
        """Doğrultma momenti (N·m): bir gövde sudan kalkarken m·g·B/2."""
        return self.mass * G * self.beam / 2.0

    @property
    def side_force_max(self):
        """Devrilmeden taşınabilecek en büyük yelken yan kuvveti (N): m·g·B / (2h)."""
        return self.righting_moment / self.heel_arm

    @property
    def rudder_stall_deg(self):
        """Dümen stall açısı (°): C_Lα·δ = C_L,maks. Bunun ötesinde dönüş artmaz, yalnızca sürükleme artar."""
        return float(min(np.degrees(self.rudder_clmax / self.rudder_cla), RUDDER_MAX_DEG))

    @property
    def yaw_coef(self):
        """c = ½ρ_su·A_d·C_Lα·x_d·L: yelken momentini dümen kinematiği ölçeğinde dönüş hızına çevirir (N·s)."""
        return 0.5 * self.rho_water * self.rudder_area * self.rudder_cla * self.rudder_arm * self.hull_length


PHYS = Physics()


def profile_key(profile_name=PROFILE_NAME):
    """Geriye dönük uyumluluk: tek yelken modeli olduğundan her ad kumaş yelkene karşılık gelir."""
    return PROFILE


# --- AÇI YARDIMCILARI ---
def wrap360(a):
    return a % 360.0


def wrap180(a):
    """Açıyı (-180, 180] aralığına indirger."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def true_wind_angle(heading_deg):
    """Kuzey rüzgarı için TWA ∈ [0, 180]."""
    h = wrap360(heading_deg)
    return h if h <= 180.0 else 360.0 - h


def leeward_sign(heading_deg):
    """+1: bom sancağa (iskele kontra), -1: bom iskeleye (sancak kontra), 0: ölü açı."""
    h = wrap360(heading_deg)
    if true_wind_angle(h) <= NO_GO_DEG:
        return 0.0
    return 1.0 if h <= 180.0 else -1.0


def dead_zone_deg(sail_deg=None):
    """
    Ölü açı yarı genişliği (°): rüzgar pruvaya bu açıdan dar gelirse yelken itki üretemez.
    * NO_GO_DEG teknenin kendi sınırıdır (auto-trim'de ölü açı budur).
    * Bom elle |δs| açılmışsa yelken (tekne dururken) TWA ≤ |δs|'de rüzgara paralel kalır ve yapraklar;
      ölü açı max(NO_GO_DEG, |δs|)'ye genişler. Tekne hızlandıkça görünür rüzgar pruvaya kaydığından
      yapraklama daha da erken başlar (kuvvet modeli bunu görünür rüzgarla hesaplar).
    """
    return NO_GO_DEG if sail_deg is None else max(NO_GO_DEG, abs(float(sail_deg)))


def band_index(twa):
    """TWA'nın düştüğü seyir bandı; ölü açıda None."""
    if twa <= NO_GO_DEG:
        return None
    for i, (lo, hi, _, _) in enumerate(BANDS):
        if lo < twa <= hi:
            return i
    return len(BANDS) - 1


# --- KUVVETLER ---
def sail_coefficients(alpha_deg):
    """Kumaş yelkenin C_L, C_D katsayıları (α: kiriş – görünür rüzgar açısı, °). α ≤ 0: yapraklama."""
    if np.ndim(alpha_deg) == 0:                   # tek değer: hızlı yol (adım fonksiyonu)
        a = float(alpha_deg)
        if a <= 1e-9:
            return 0.0, CD_FLOG
        if a <= 90.0:
            return float(np.interp(a, SAIL_ALPHA_F, SAIL_CL_F)), float(np.interp(a, SAIL_ALPHA_F, SAIL_CD_F))
        b = min(180.0 - a, 90.0)
        return (-0.6 * float(np.interp(b, SAIL_ALPHA_F, SAIL_CL_F)),
                max(CD_FLOG, float(np.interp(b, SAIL_ALPHA_F, SAIL_CD_F))))
    a = np.asarray(alpha_deg, dtype=float)
    a = np.where(np.abs(a) < 1e-9, 0.0, a)       # bom = AWA: kayan nokta payı yapraklamayı bozmasın
    fwd = np.clip(a, 0.0, 90.0)
    back = np.clip(180.0 - a, 0.0, 90.0)          # α > 90°: rüzgar kirişe arkadan gelir
    cl = np.where(a <= 0.0, 0.0, np.where(a <= 90.0, np.interp(fwd, SAIL_ALPHA_F, SAIL_CL_F),
                                         -0.6 * np.interp(back, SAIL_ALPHA_F, SAIL_CL_F)))
    cd = np.where(a <= 0.0, CD_FLOG, np.where(a <= 90.0, np.interp(fwd, SAIL_ALPHA_F, SAIL_CD_F),
                                             np.maximum(CD_FLOG, np.interp(back, SAIL_ALPHA_F, SAIL_CD_F))))
    return cl, cd


def apparent_wind(heading_deg, speed_ms, wind_ms):
    """Görünür rüzgar hızı (m/s) ve pruvaya göre açısı AWA (°, 0–180)."""
    twa = np.radians(true_wind_angle(heading_deg))
    ax = speed_ms + wind_ms * np.cos(twa)
    ay = wind_ms * np.sin(twa)
    va = np.hypot(ax, ay)
    awa = np.degrees(np.arctan2(ay, ax))
    return va, awa


def sail_force(heading_deg, speed_ms, wind_ms, sail_deg, phys=PHYS):
    """
    Yelken kuvveti (gövde ekseninde). sail_deg: bomun orta hattan açısı |δs| (dizi olabilir).
    Dönüş: (X ileri itki N, Y rüzgar altına yan kuvvet N, AWA °, V_A m/s, α °, C_L, C_D, f devrilme çarpanı)
    """
    twa = true_wind_angle(heading_deg)
    va, awa = apparent_wind(heading_deg, speed_ms, wind_ms)
    alpha = awa - np.abs(np.asarray(sail_deg, dtype=float))
    cl, cd = sail_coefficients(alpha)
    if twa <= NO_GO_DEG:
        cl = np.zeros_like(cl)                    # ölü açı: yelken doldurulamaz
    q = 0.5 * phys.rho_air * va * va * phys.sail_area
    b = np.radians(awa)
    x = q * (cl * np.sin(b) - cd * np.cos(b))
    y = q * (cl * np.cos(b) + cd * np.sin(b))
    if phys.heel_limit:
        f = np.minimum(1.0, phys.side_force_max / np.maximum(np.abs(y), 1e-9))
    else:
        f = np.ones_like(np.asarray(y, dtype=float))
    return x * f, y * f, awa, va, alpha, cl, cd, f


def best_trim(heading_deg, speed_ms, wind_ms, phys=PHYS):
    """
    Auto-trim: net itkiyi X − R_i(Y) en büyük yapan bom açısı |δs| (°). Yan kuvvet yan kayma direnci ürettiği için
    aynı itkiyi daha az yan kuvvetle veren bom seçilir. Kesin aday arama (trim_candidates). Ölü açıda 0.
    """
    if true_wind_angle(heading_deg) <= NO_GO_DEG:
        return 0.0
    v = float(speed_ms)
    cand = trim_candidates(heading_deg, v, wind_ms, phys)[0]
    x, y = sail_force(heading_deg, v, wind_ms, cand, phys)[:2]
    return float(cand[int(np.argmax(x - induced_resistance(y, v, phys)))])


def signed_sail_angle(heading_deg, speed_ms, wind_ms, phys=PHYS):
    """Auto-trim bom açısı (işaretli): + bom sancağa, − iskeleye, ölü açıda 0."""
    return leeward_sign(heading_deg) * best_trim(heading_deg, speed_ms, wind_ms, phys)


def hull_resistance(speed_ms, phys=PHYS):
    """
    Gövde direnci R = ½ρ_su S v² (C_h + C_R(Fr)) (N). C_h sabit (sürtünme + şekil): direnç hızın karesiyle büyür,
    sonuçlar tekne boyutundan bağımsızdır. C_R: boya bağlı dalga direnci tepesi (Fr ≈ 0.5), boyutlandırma
    aşamasına kadar 0.
    """
    v = np.maximum(np.asarray(speed_ms, dtype=float), 0.0)
    cr = phys.wave_cr * np.exp(-((v / np.sqrt(G * phys.hull_length) - 0.5) / 0.25) ** 2) if phys.wave_cr else 0.0
    return 0.5 * phys.rho_water * phys.wetted_area * v * v * (phys.hull_cd + cr)


def leeway_speed(side_force, speed_ms, phys=PHYS):
    """Yan kayma hızı v_yan ≥ 0 (m/s): ½ρ_su(A_s C_Lα,s v·w + A_g C_D,ç w²) = |Y|."""
    y = np.abs(np.asarray(side_force, dtype=float))
    a = 0.5 * phys.rho_water * phys.hull_side_area * phys.cross_cd
    b = 0.5 * phys.rho_water * phys.board_area * phys.board_cla * np.maximum(np.asarray(speed_ms, dtype=float), 0.0)
    return (-b + np.sqrt(b * b + 4.0 * a * y)) / (2.0 * a)


def leeway_angle(side_force, speed_ms, phys=PHYS):
    """Yan kayma açısı λ (°)."""
    w = leeway_speed(side_force, speed_ms, phys)
    return np.degrees(np.arctan2(w, np.maximum(np.asarray(speed_ms, dtype=float), 0.0)))


def induced_resistance(side_force, speed_ms, phys=PHYS):
    """
    Yan kayma direnci R_i = L_s·sin λ (N): hançerin kaldırma kuvveti L_s = ½ρ_su A_s C_Lα,s v·v_yan gelen akışa
    diktir; λ açısıyla gelen akışta ileri harekete karşı bileşeni L_s·sin λ'dır. Gövdelerin çapraz akış
    direnci saf yanal kuvvettir (ileri bileşeni yok) – tekne dururken yana kayması ileri direnç üretmez.
    """
    v = np.maximum(np.asarray(speed_ms, dtype=float), 0.0)
    w = leeway_speed(side_force, v, phys)
    lift = 0.5 * phys.rho_water * phys.board_area * phys.board_cla * v * w
    return lift * w / np.maximum(np.hypot(v, w), 1e-9)


def rudder_lift_coef(rudder_deg, phys=PHYS):
    r = np.clip(rudder_deg, -RUDDER_MAX_DEG, RUDDER_MAX_DEG)
    return np.clip(phys.rudder_cla * np.radians(r), -phys.rudder_clmax, phys.rudder_clmax)


def rudder_drag(speed_ms, rudder_deg, phys=PHYS):
    """
    Dümen palalarının sürüklemesi D_d (N): profil + indüklenmiş direnç; stall açısının (C_L,maks / C_Lα) ötesinde
    akış ayrılır ve pala düz levha gibi frenler: C_D,ayr = 1.8·(sin²δr − sin²δ_stall).
    """
    d = np.radians(np.clip(np.abs(rudder_deg), 0.0, RUDDER_MAX_DEG))
    cl = rudder_lift_coef(rudder_deg, phys)
    d_stall = min(phys.rudder_clmax / phys.rudder_cla, np.pi / 2)
    cd = (RUDDER_CD0 + cl * cl / (np.pi * RUDDER_AR * RUDDER_E)
          + 1.8 * np.maximum(np.sin(d) ** 2 - np.sin(d_stall) ** 2, 0.0))
    return 0.5 * phys.rho_water * phys.rudder_area * np.asarray(speed_ms, dtype=float) ** 2 * cd


def total_resistance(speed_ms, side_force, rudder_deg, phys=PHYS):
    return (hull_resistance(speed_ms, phys) + induced_resistance(side_force, speed_ms, phys)
            + rudder_drag(speed_ms, rudder_deg, phys))


def sail_yaw_rate(side_force, s, speed_ms, phys=PHYS):
    """Yelken momentinin dönüş hızı karşılığı r_yelken (°/s, − : rüzgara doğru, yani orsaya kaçma)."""
    if speed_ms <= MIN_STEER_SPEED or s == 0.0:
        return 0.0
    return float(-s * np.degrees(side_force * phys.ce_offset / (phys.yaw_coef * max(speed_ms, 0.5))))


def rudder_effective_deg(rudder_deg, phys=PHYS):
    """Dönüşü belirleyen etkin dümen açısı δ_e = C_L(δr) / C_Lα (°): doğrusal bölgede δr, stall'dan sonra δ_stall."""
    return float(np.degrees(rudder_lift_coef(rudder_deg, phys) / phys.rudder_cla))


def rudder_yaw_rate(speed_ms, rudder_deg, phys=PHYS):
    """
    Dümenin kararlı dönüş hızı (°/s): (v/L)·sin δ_e. Dönüş momenti palanın kaldırma kuvvetiyle orantılı olduğundan
    stall açısının (≈ 20°) ötesinde dönüş artmaz; fazla dümen yalnızca fren olur (bkz. rudder_drag).
    """
    if speed_ms <= MIN_STEER_SPEED:
        return 0.0
    return float(np.degrees(speed_ms / phys.hull_length * np.sin(np.radians(rudder_effective_deg(rudder_deg, phys)))))


def holding_rudder(side_force, s, speed_ms, phys=PHYS):
    """Rotayı tutan dümen açısı (°): (v/L)·sin δr + r_yelken = 0. Dümen tutmayan hızda 0."""
    if speed_ms <= MIN_STEER_SPEED:
        return 0.0
    lim = np.sin(np.radians(phys.rudder_stall_deg))
    x = -np.radians(sail_yaw_rate(side_force, s, speed_ms, phys)) * phys.hull_length / speed_ms
    return float(np.degrees(np.arcsin(np.clip(x, -lim, lim))))


def coast_tau(speed_ms, phys=PHYS):
    """Etkin ileri yön zaman sabiti τ = m·v / R(v) (s): itki kesildiğinde hızın sönme ölçeği."""
    v = max(float(speed_ms), 0.3)
    return float(phys.mass * v / max(float(hull_resistance(v, phys)), 1e-6))


# --- KARARLI HIZ (VPP) ---
def trim_candidates(heading_deg, speed_ms, wind_ms, phys=PHYS):
    """
    En iyi bom adayları (°). Katsayı tablosu 0.5° aralıklı ve aralarda doğrusal olduğundan net itki α'nın parçalı
    doğrusal fonksiyonudur: en büyük değer bir tablo düğümünde (δs = AWA − α_k) ya da bom sınırında (0°, 90°)
    bulunur. Aday kümesi görünür rüzgarla birlikte kaydığından en iyi bom hız ve pruvayla sürekli değişir.
    Devrilme sınırı açıkken f·X doğrusal olmadığından 0.5°'lik bom ızgarası da eklenir. speed_ms dizi olabilir:
    dönüş (n, k) biçimindedir.
    """
    awa = np.atleast_1d(apparent_wind(heading_deg, np.atleast_1d(np.asarray(speed_ms, dtype=float)), wind_ms)[1])
    cand = np.clip(awa[:, None] - SAIL_ALPHA_F[None, 1:], 0.0, 90.0)
    extra = [np.zeros_like(awa)[:, None], np.full_like(awa, 90.0)[:, None]]
    if phys.heel_limit:
        extra.append(np.broadcast_to(SAIL_GRID, (awa.size, SAIL_GRID.size)))
    return np.concatenate([cand] + extra, axis=1)


def _net_force(heading_deg, v, wind_ms, sail_deg, rudder_deg, phys):
    """v dizisi için en iyi (sail_deg None) ya da verilen trimde net ileri kuvvet ve kullanılan bom."""
    v = np.asarray(v, dtype=float)
    if sail_deg is None:
        if true_wind_angle(heading_deg) <= NO_GO_DEG:
            sails = np.zeros_like(v)
        else:
            cand = trim_candidates(heading_deg, v, wind_ms, phys)
            x2, y2 = sail_force(heading_deg, v[:, None], wind_ms, cand, phys)[:2]
            obj = x2 - induced_resistance(y2, v[:, None], phys)
            sails = cand[np.arange(v.size), np.argmax(obj, axis=1)]
    else:
        sails = np.full_like(v, abs(float(sail_deg)))
    x, y = sail_force(heading_deg, v, wind_ms, sails, phys)[:2]
    return x - total_resistance(v, y, rudder_deg, phys), sails


@lru_cache(maxsize=200000)
def _steady(twa_key, wind_key, sail_key, rudder_key, phys):
    twa, wind = twa_key / 100.0, wind_key / 1000.0
    sail = None if sail_key is None else sail_key / 10.0
    rudder = rudder_key / 10.0
    if wind <= 0.0:
        return 0.0, 0.0
    vs = np.linspace(0.0, 2.5 * wind + 1.0, 241)
    f, sails = _net_force(twa, vs, wind, sail, rudder, phys)
    idx = np.nonzero((f[:-1] > 0.0) & (f[1:] <= 0.0))[0]
    if idx.size == 0:
        return 0.0, float(sails[0])
    lo, hi = vs[idx[-1]], vs[idx[-1] + 1]
    for _ in range(30):                           # en büyük kararlı kökü daralt
        mid = 0.5 * (lo + hi)
        fm, _ = _net_force(twa, [mid], wind, sail, rudder, phys)
        lo, hi = (mid, hi) if fm[0] > 0.0 else (lo, mid)
    v = 0.5 * (lo + hi)
    _, s = _net_force(twa, [v], wind, sail, rudder, phys)
    return float(v), float(s[0])


def steady_state(heading_deg, wind_ms, sail_deg=None, rudder_deg=0.0, phys=PHYS):
    """Kararlı hız (m/s) ve bom açısı |δs| (°): sail_deg None ise auto-trim (en iyi trim)."""
    twa = true_wind_angle(heading_deg)
    return _steady(int(round(twa * 100)), int(round(wind_ms * 1000)),
                   None if sail_deg is None else int(round(abs(float(sail_deg)) * 10)),
                   int(round(float(np.clip(rudder_deg, -RUDDER_MAX_DEG, RUDDER_MAX_DEG)) * 10)), phys)


def polar(heading_deg, wind_ms, phys=PHYS):
    """(kararlı hız m/s, optimum bom |δs| °) – auto-trim, dümen 0."""
    return steady_state(heading_deg, wind_ms, None, 0.0, phys)


def polar_speed(heading_deg, profile, wind_ms, phys=PHYS):
    """Optimum trim ve sıfır dümende kararlı hız (planlama için)."""
    return polar(heading_deg, wind_ms, phys)[0]


@lru_cache(maxsize=256)
def _polar_table(wind_key, phys, step):
    wind = wind_key / 1000.0
    twa = np.arange(0.0, 180.0 + 1e-9, step)
    v, sail, lee = [], [], []
    for t in twa:
        vt, st = polar(t, wind, phys)
        y = float(sail_force(t, vt, wind, st, phys)[1])
        v.append(vt)
        sail.append(st)
        lee.append(float(leeway_angle(y, vt, phys)) if t > NO_GO_DEG and vt > 0.0 else 0.0)
    return twa, np.array(v), np.array(sail), np.array(lee)


def polar_table(wind_ms, phys=PHYS, step=0.5):
    """TWA ızgarasında (0..180) kararlı hız (m/s), optimum bom (°) ve yan kayma açısı (°): VMG araması için."""
    return _polar_table(int(round(wind_ms * 1000)), phys, step)


def coast_distance(speed_ms, phys=PHYS, rudder_deg=0.0, wind_ms=0.0):
    """
    İtki kesildiğinde (ölü açı) hız v'den dümen eşiğine düşene kadar alınan yol (m):
    ∫ m·v / (R(v) + D_d(v, δr) + D_yelken(v)) dv. Dönerken (tack) dümenin sürüklemesi, rüzgara dönmüş teknede
    çırpınan yelkenin sürüklemesi D_yelken = ½ρ_hava A C_D,çırp (v + U)² de tekneyi yavaşlatır (pruvadan rüzgar,
    en kötü durum).
    """
    if speed_ms <= MIN_STEER_SPEED:
        return 0.0
    vs = np.linspace(MIN_STEER_SPEED, speed_ms, 80)
    drag = (hull_resistance(vs, phys) + rudder_drag(vs, rudder_deg, phys)
            + 0.5 * phys.rho_air * phys.sail_area * CD_FLOG * (vs + wind_ms) ** 2)
    integrand = phys.mass * vs / np.maximum(drag, 1e-6)
    return float(np.sum(0.5 * (integrand[1:] + integrand[:-1]) * np.diff(vs)))      # yamuk kuralı


def target_speed(heading_deg, sail_deg, rudder_deg, profile, wind_ms, auto_trim=False, phys=PHYS):
    """Kararlı-durum (hedef) tekne hızı V* (m/s): verilen bom (auto_trim'de en iyi trim) ve dümende."""
    return steady_state(heading_deg, wind_ms, None if auto_trim else sail_deg, rudder_deg, phys)[0]


def trim_status(heading_deg, sail_deg, speed_ms, wind_ms, phys=PHYS):
    """
    Bomun anlık durumu (görünür rüzgara göre). Dönüş: (verim = X/X_opt, durum metni, rozet tipi, tor miktarı).
    α ≤ 0: yapraklama; α < 8°: fazla açık (orsa yakası çırpar); α > 35°: fazla kasılı (akış kopar).
    """
    if true_wind_angle(heading_deg) <= NO_GO_DEG:
        return 0.0, "Yelken Dolmuyor (Ölü Açı)", "badge-dead", 0.0
    x, _, awa, _, alpha = sail_force(heading_deg, speed_ms, wind_ms, abs(sail_deg), phys)[:5]
    x_opt = sail_force(heading_deg, speed_ms, wind_ms, best_trim(heading_deg, speed_ms, wind_ms, phys), phys)[0]
    eff = float(np.clip(x / x_opt, 0.0, 1.0)) if x_opt > 1e-9 else 0.0
    a = float(alpha)
    if a <= 1e-6:                                 # kayan nokta payı: bom = AWA tam yapraklamadır
        return 0.0, "Rüzgara Paralel (Tam Yapraklama – İtki Yok)", "badge-dead", 0.0
    if a < 8.0:
        return eff, "Fazla Açık (Yapraklama)", "badge-warn", 0.22
    if a > 35.0 and awa < 120.0:
        return eff, "Fazla Kasılı (Akış Koptu / Stall)", "badge-warn", 0.18
    if eff >= 0.9:
        return eff, "Tam Trim (Yelken Dolu / Torlu)", "badge-full", 0.45
    return eff, "Trim Ayarlanmalı", "badge-warn", 0.3


def analyze_points_of_sail_360(heading_deg, profile_name=PROFILE_NAME, sail_deg=None, wind_ms=8.0, phys=PHYS):
    """
    Seyir durumu analizi. Dönüş: (regime, opt_sail, hız oranı V*/U, tack, wind_side, leeward_sign, desc)
    sail_deg verilirse (elle trim) ölü açı bom açısına göre genişler (dead_zone_deg); auto-trim'de None.
    """
    h = wrap360(heading_deg)
    twa = true_wind_angle(h)
    i = band_index(twa)
    if i is None:
        return (NO_GO_REGIME, 0.0, 0.0, "Kör Nokta", "Pruvadan (Kafadan)", 0.0,
                "Rüzgar doğrudan pruvadan geliyor (330°-030° arası 60° ölü açı). "
                "Yelken rüzgar tutamaz, yapraklar. İtki sıfırdır.")
    v_opt, opt_sail = polar(h, wind_ms, phys)
    ratio = v_opt / wind_ms if wind_ms > 0 else 0.0
    _, _, name_tr, name_en = BANDS[i]
    lee = 1.0 if h <= 180.0 else -1.0
    if i == len(BANDS) - 1:
        regime, tack, wind_side = f"{name_tr} Seyri ({name_en})", "Pupa", "Kıçtan (Arkadan)"
    elif lee > 0:
        regime, tack, wind_side = f"İskele Kontra {name_tr} ({name_en})", "İskele Kontra", "İskele (Sol)"
    else:
        regime, tack, wind_side = f"Sancak Kontra {name_tr} ({name_en})", "Sancak Kontra", "Sancak (Sağ)"
    dz = dead_zone_deg(sail_deg)
    if twa <= dz:
        sail = abs(float(sail_deg))
        return (NO_GO_REGIME, opt_sail, 0.0, tack, wind_side, lee,
                f"Bom {sail:.0f}° açık, rüzgar pruvaya {twa:.0f}° geliyor: yelken rüzgara paralel kalır ve yapraklar "
                f"(hücum açısı α = TWA − bom = {twa - sail:.0f}° ≤ 0), itki sıfırdır. Bu bom açısında ölü açı "
                f"{360.0 - dz:03.0f}°–{dz:03.0f}° (±{dz:.0f}°) olur. Bomu {opt_sail:.0f}°'ye kasın.")
    return regime, opt_sail, ratio, tack, wind_side, lee, DESC[i]


# --- DİNAMİK ---
@dataclass
class BoatState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0       # pruva (°)
    v: float = 0.0         # ileri hız (m/s)
    r: float = 0.0         # dönüş hızı (°/s)
    delta: float = 0.0     # gerçek dümen açısı (°)
    # son adımın kayıtları
    sail: float = 0.0      # işaretli bom açısı (°)
    drive: float = 0.0     # yelken itkisi X (N)
    side: float = 0.0      # yelken yan kuvveti Y (N)
    heel_factor: float = 1.0
    v_lat: float = 0.0     # yan kayma hızı (m/s, rüzgar altına)
    awa: float = 0.0
    rudder_cmd: float = 0.0


def step(state, sail_deg, rudder_cmd, wind_ms, auto_trim=False, dt=DT, phys=PHYS, hold_course=False, leeway=True):
    """
    Tek entegrasyon adımı. sail_deg: elle bom |δs| (auto_trim'de yok sayılır); rudder_cmd: dümen komutu (°)
    – dümen servosu hız sınırıyla izler; hold_course: komut yerine orsaya kaçmayı dengeleyen dümen.
    leeway False: yan kayma konuma yansıtılmaz (karşılaştırma için; dirençler aynı kalır).
    """
    psi, v = state.psi, state.v
    s = leeward_sign(psi)
    sail = best_trim(psi, v, wind_ms, phys) if auto_trim else abs(float(sail_deg))
    x, y, awa, va, alpha, cl, cd, f = sail_force(psi, v, wind_ms, sail, phys)
    x, y, f = float(x), float(y), float(f)
    if hold_course:
        rudder_cmd = holding_rudder(y, s, v, phys)
    rudder_cmd = float(np.clip(rudder_cmd, -RUDDER_MAX_DEG, RUDDER_MAX_DEG))
    rate = phys.rudder_rate * dt
    delta = state.delta + float(np.clip(rudder_cmd - state.delta, -rate, rate))
    # ileri hız
    force = x - float(total_resistance(v, y, delta, phys))
    v_new = max(0.0, v + dt * force / phys.mass)
    # dönüş (Nomoto) – kararlı dönüş hızı: dümen + yelken momenti
    r_ss = rudder_yaw_rate(v_new, delta, phys) + sail_yaw_rate(y, s, v_new, phys)
    t_yaw = phys.yaw_t_nd * phys.hull_length / max(v_new, 1.0)
    r = r_ss + (state.r - r_ss) * np.exp(-dt / t_yaw)
    psi_new = wrap360(psi + r * dt)
    # konum: ileri hız + yan kayma (rüzgar altına)
    w = float(leeway_speed(y, v_new, phys)) if (s != 0.0 and leeway) else 0.0
    pr = np.radians(psi_new)
    vx = v_new * np.sin(pr) + s * w * np.cos(pr)
    vy = v_new * np.cos(pr) - s * w * np.sin(pr)
    return BoatState(state.x + vx * dt, state.y + vy * dt, psi_new, v_new, float(r), delta,
                     sail=s * sail, drive=x, side=y, heel_factor=f, v_lat=w, awa=float(awa), rudder_cmd=rudder_cmd)


def initial_state(x, y, heading_deg, wind_ms, sail_deg=None, phys=PHYS, speed=None):
    """Başlangıç durumu: hız verilmezse verilen trimde (None: auto-trim) kararlı hız; dümen 0."""
    psi = wrap360(float(heading_deg))
    v = steady_state(psi, wind_ms, sail_deg, 0.0, phys)[0] if speed is None else float(speed)
    return BoatState(float(x), float(y), psi, v)


def simulate_open_loop(heading_deg, sail_deg, rudder_deg, wind_ms, sim_time_s, auto_trim=False, dt=DT,
                       x0=0.0, y0=0.0, phys=PHYS, hold_course=False):
    """
    Sabit yelken/dümen komutuyla açık çevrim simülasyon. Tekne başlangıçta (dümen 0'da) kararlı hızdadır.
    hold_course: dümen her adımda orsaya kaçmayı dengeler, pruva sabit kalır (rudder_deg yok sayılır).
    """
    s = initial_state(x0, y0, heading_deg, wind_ms, None if auto_trim else sail_deg, phys)
    rec = {k: [] for k in ("x", "y", "psi", "v", "rudder", "sail", "v_lat", "drive", "side", "heel_factor", "r")}

    def log(st):
        for k, val in (("x", st.x), ("y", st.y), ("psi", st.psi), ("v", st.v), ("rudder", st.delta),
                       ("sail", st.sail), ("v_lat", st.v_lat), ("drive", st.drive), ("side", st.side),
                       ("heel_factor", st.heel_factor), ("r", st.r)):
            rec[k].append(float(val))

    first = step(s, sail_deg, 0.0, wind_ms, auto_trim, 0.0, phys)   # dt = 0: yalnızca kuvvet kaydı
    s.sail, s.drive, s.side, s.heel_factor, s.v_lat = first.sail, first.drive, first.side, first.heel_factor, first.v_lat
    log(s)
    dist = 0.0
    for _ in range(int(round(sim_time_s / dt))):
        s = step(s, sail_deg, rudder_deg, wind_ms, auto_trim, dt, phys, hold_course)
        log(s)
        dist += np.hypot(rec["x"][-1] - rec["x"][-2], rec["y"][-1] - rec["y"][-2])
    out = {k: np.array(v) for k, v in rec.items()}
    out.update(distance=float(dist), final_heading=s.psi, v0=float(rec["v"][0]))
    return out


# --- KUVVET VE MOMENT TABLOSU (arayüz / rapor) ---
def force_moment_breakdown(heading_deg, sail_deg, rudder_deg, wind_ms, speed_ms=None, auto_trim=False, phys=PHYS):
    """
    Verilen pruva / bom / dümende tekneye etkiyen kuvvetler (N) ve momentler (N·m). speed_ms None ise
    tekne verilen trim ve dümende kararlı hızdadır (net ileri kuvvet sıfır).
    İşaretler: momentte + sancağa döndürür; yan kuvvet rüzgar altı bordasına doğrudur.
    """
    psi = wrap360(heading_deg)
    s = leeward_sign(psi)
    v_star, sail_star = steady_state(psi, wind_ms, None if auto_trim else sail_deg, rudder_deg, phys)
    v = v_star if speed_ms is None else float(speed_ms)
    sail = best_trim(psi, v, wind_ms, phys) if auto_trim else abs(float(sail_deg))
    x, y, awa, va, alpha, cl, cd, f = (float(z) for z in sail_force(psi, v, wind_ms, sail, phys))
    r_hull = float(hull_resistance(v, phys))
    r_ind = float(induced_resistance(y, v, phys))
    d_rud = float(rudder_drag(v, rudder_deg, phys))
    w = float(leeway_speed(y, v, phys)) if s != 0.0 else 0.0
    q_w = 0.5 * phys.rho_water * phys.rudder_area * v * v
    cl_r = float(rudder_lift_coef(rudder_deg, phys))
    rudder_force = q_w * cl_r
    sail_moment = -s * y * phys.ce_offset
    r_sail = sail_yaw_rate(y, s, v, phys)
    r_rud = rudder_yaw_rate(v, rudder_deg, phys)
    yaw = r_rud + r_sail
    wr = np.radians(yaw)
    q_a = 0.5 * phys.rho_air * va * va * phys.sail_area
    return dict(
        twa=true_wind_angle(psi), leeward=s, sail=sail, v_star=v_star, sail_star=sail_star, v=v,
        va=va, awa=awa, alpha=alpha, cl=cl, cd=cd, q_air=q_a, heel_factor=f,
        drive=x, side=y, sail_force=float(np.hypot(x, y)), c_total=float(np.hypot(x, y) / q_a) if q_a > 0 else 0.0,
        hull_drag=r_hull, induced_drag=r_ind, rudder_drag=d_rud, net=x - r_hull - r_ind - d_rud,
        accel=(x - r_hull - r_ind - d_rud) / phys.mass, v_lat=w,
        leeway=float(np.degrees(np.arctan2(w, max(v, 1e-9)))) if w > 0 else 0.0,
        heel_moment=abs(y) * phys.heel_arm, righting_moment=phys.righting_moment, side_force_max=phys.side_force_max,
        rudder_cl=cl_r, rudder_force=rudder_force, rudder_moment=rudder_force * phys.rudder_arm,
        sail_moment=sail_moment, net_moment=rudder_force * phys.rudder_arm + sail_moment,
        sail_yaw=r_sail, rudder_yaw=r_rud, yaw_rate=yaw,
        rudder_balance=holding_rudder(y, s, v, phys),
        turn_radius=(v / abs(wr) if abs(wr) > 1e-9 else float("inf")), centripetal=phys.mass * v * abs(wr),
        tau=coast_tau(v, phys),
    )
