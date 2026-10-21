"""
Saildrone / yelkenli katamaran fizik modeli (Streamlit'ten bağımsız, test edilebilir).
Tek yelken modeli: Kumaş Yelken (Cambered / Klasik).

Koordinat ve işaret kuralları
-----------------------------
* Dünya ekseni: x = Doğu (m), y = Kuzey (m).
* Pruva açısı psi: pusula açısı (0° = Kuzey, saat yönünde artar).
  Hız bileşenleri: x' = v·sin(psi), y' = v·cos(psi).
* Rüzgar sabittir ve KUZEYDEN eser (güneye doğru akar). Bu nedenle gerçek
  rüzgar açısı TWA = min(psi, 360 - psi).
* psi ∈ (0°, 180°): rüzgar iskele (sol) bordadan gelir -> İSKELE KONTRA,
  yelken sancağa açılır (leeward_sign = +1).
  psi ∈ (180°, 360°): rüzgar sancak (sağ) bordadan gelir -> SANCAK KONTRA,
  yelken iskeleye açılır (leeward_sign = -1).
* Dümen açısı delta > 0: pala sancağa açılır, tekne sancağa döner (psi artar).
"""
from dataclasses import dataclass

import numpy as np

# --- MODEL SABİTLERİ ---
# L_HULL, RUDDER_DRAG, WEATHER_HELM_DPS ve TAU_SURGE varsayılan tekneye (Physics()) aittir;
# kullanıcı parametreleriyle çalışan fonksiyonlar bunları phys argümanından türetir.
NO_GO_DEG = 30.0            # Ölü açı yarı genişliği (TWA <= 30° -> itki yok)
L_HULL = 4.6                # Dönüş kinematiğinde kullanılan efektif gövde boyu (m)
RUDDER_MAX_DEG = 35.0       # Dümen mekanik sınırı
RUDDER_DRAG = 0.22          # Tam dümende hız kaybı oranı
WEATHER_HELM_DPS = 0.35     # Orsaya kaçma (weather helm) eğilimi (°/s)
MIN_STEER_SPEED = 0.2       # Bu hızın altında dümen etkisizdir (m/s)
TAU_SURGE = 3.0             # İleri yön (surge) hız zaman sabiti (s)
DT = 0.25                   # Varsayılan entegrasyon adımı (s)
MS_TO_KTS = 1.94384

PROFILE = "cloth"           # Tek standart yelken: kumaş (cambered) yelken
PROFILE_KEYS = (PROFILE,)
PROFILE_NAME = "Kumaş Yelken (Cambered / Klasik)"

# TWA bantları: (alt, üst] derece
BANDS = [
    (30.0, 50.0, "Orsa", "Close-Hauled"),
    (50.0, 80.0, "Dar Apaz", "Close Reach"),
    (80.0, 120.0, "Tam Apaz", "Beam Reach"),
    (120.0, 150.0, "Geniş Apaz", "Broad Reach"),
    (150.0, 180.0, "Pupa", "Running"),
]

# Her bant için (optimum bom açısı [°], hız oranı V_b / V_w)
POLAR = {
    "cloth": [(16.0, 0.62), (32.0, 0.85), (46.0, 1.15), (65.0, 0.92), (82.0, 0.74)],
}

# Trim verim modeli parametreleri:
#   tol: tam trim toleransı (°), loss: tolerans kenarındaki kayıp,
#   under/over: (ölçek °, üs, taban) – fazla kasılı / fazla açık dalları.
TRIM = {
    "cloth": dict(tol=6.0, loss=0.05, under=(25.0, 1.6, 0.15), over=(30.0, 1.6, 0.10),
                  camber=(0.45, 0.18, 0.22), full="Tam Trim (Yelken Dolu / Torlu)",
                  under_txt="Fazla Kasılı (Düz / Stall)", over_txt="Fazla Açık (Yapraklama)"),
}

DESC = {
    "cloth": [
        "Klasik kumaş yelken orsası. Iskotalar sonuna kadar kasılı. Kumaş esnemesi nedeniyle 30° sınırında itki düşer.",
        "Kumaş yelken orsaya göre biraz daha boşlanır, tor alanı genişler ve hız artar.",
        "TAM APAZ (BEAM REACH): Kumaş yelken tam dolu ve torludur. Rüzgar hızını %15 aşar (1.15x).",
        "GENİŞ APAZ: Kumaş yelken iyice boşlanır, rüzgar cebi oluşarak itki sağlar.",
        "PUPA SEYRİ: Kumaş yelken rüzgarı paraşüt gibi doldurup torlanır; itki sürükleme (drag) kuvvetinden gelir (0.74x).",
    ],
}

NO_GO_REGIME = "Ölü Açı (No-Go Zone)"


# --- TEKNENİN FİZİKSEL PARAMETRELERİ (arayüzden değiştirilebilir) ---
# Hız poları (POLAR) ve orsaya kaçma hızı (WEATHER_HELM_DPS) varsayılan tekne için kalibre edilmiştir.
# Parametreler değişince simülasyon aynı denklemlerle şu türetilmiş büyüklükleri kullanır:
#   τ    = m / k_g                                   ileri yön (surge) zaman sabiti
#   λ    = (A_s / A_s,ref) · (k_g,ref / k_g)          hız ölçeği: V* = λ·U·ρ·η·κ  (itki ∝ A_s, direnç = k_g·v)
#   ω_wh = ω_wh,ref · Φ / Φ_ref,  Φ = e·k_g / (ρ_su·A_d·C_Lα·x_d·L)
#          (yelken momentine eşdeğer dümen açısının kinematik dönüş hızı; bkz. force_moment_breakdown)
#   L    : kinematik dönüşte efektif gövde boyu,  c_dd : tam dümende hız kaybı oranı
@dataclass(frozen=True)
class Physics:
    mass: float = 250.0              # tekne + ekipman kütlesi m (kg) – varsayım (≈ 5 m'lik katamaran)
    hull_drag: float = 250.0 / 3.0   # gövde doğrusal direnç katsayısı k_g (N·s/m): R = k_g·v
    sail_area: float = 20.0          # kumaş yelken alanı A_s (m²) – varsayım
    hull_length: float = L_HULL      # efektif gövde boyu L (m): ψ̇ = v·sin δr / L
    rudder_area: float = 0.16        # iki dümen palasının toplam alanı A_d (m²) – varsayım
    rudder_cla: float = 3.49         # dümen kaldırma eğimi C_Lα (1/rad), en-boy oranı 2.5: 2π·AR/(AR+2)
    rudder_clmax: float = 1.2        # dümen stall sınırı (C_L,max)
    rudder_arm: float = 2.1          # dümen palasının ağırlık merkezine uzaklığı x_d (m)
    ce_arm: float = 0.20             # yelken basınç merkezinin ağırlık merkezine boyuna uzaklığı a_CE (m, kıça doğru +)
    clr_arm: float = 0.0             # yanal direnç merkezinin ağırlık merkezine boyuna uzaklığı a_LR (m, kıça doğru +)
    rudder_drag: float = RUDDER_DRAG # dümen sürükleme katsayısı c_dd: κ = 1 − c_dd·(δr/δmax)²
    rho_air: float = 1.225           # hava yoğunluğu (kg/m³)
    rho_water: float = 1025.0        # deniz suyu yoğunluğu (kg/m³)

    @property
    def ce_offset(self):
        """e = a_CE − a_LR: yelken basınç merkezi yanal direnç merkezinin ne kadar gerisinde (m), tam apazda 0.20 m'ye kalibre."""
        return self.ce_arm - self.clr_arm

    @property
    def tau(self):
        """İleri yön zaman sabiti τ = m / k_g (s)."""
        return self.mass / self.hull_drag

    @property
    def speed_scale(self):
        """Hız ölçeği λ: varsayılan teknede 1."""
        return (self.sail_area / _REF.sail_area) * (_REF.hull_drag / self.hull_drag)

    @property
    def helm_factor(self):
        """Φ = e·k_g / (ρ_su·A_d·C_Lα·x_d·L): orsaya kaçma hızıyla orantılı geometri/kuvvet çarpanı."""
        return (self.ce_offset * self.hull_drag
                / (self.rho_water * self.rudder_area * self.rudder_cla * self.rudder_arm * self.hull_length))

    @property
    def weather_helm(self):
        """Orsaya kaçma hızı ω_wh (°/s)."""
        return WEATHER_HELM_DPS * (self.helm_factor / _REF.helm_factor)


_REF = Physics()
PHYS = _REF


def profile_key(profile_name=PROFILE_NAME):
    """Geriye dönük uyumluluk: tek yelken modeli olduğundan her ad kumaş yelkene karşılık gelir."""
    return PROFILE


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
    """+1: yelken sancağa (iskele kontra), -1: yelken iskeleye (sancak kontra), 0: ölü açı."""
    h = wrap360(heading_deg)
    if true_wind_angle(h) <= NO_GO_DEG:
        return 0.0
    return 1.0 if h <= 180.0 else -1.0


def band_index(twa):
    """TWA'nın düştüğü bant indeksi; ölü açıda None."""
    if twa <= NO_GO_DEG:
        return None
    for i, (lo, hi, _, _) in enumerate(BANDS):
        if lo < twa <= hi:
            return i
    return len(BANDS) - 1


def polar_point(heading_deg, profile=PROFILE):
    """(optimum bom açısı, hız oranı) – ölü açıda (0, 0)."""
    i = band_index(true_wind_angle(heading_deg))
    if i is None:
        return 0.0, 0.0
    return POLAR[profile][i]


def signed_sail_angle(heading_deg, profile=PROFILE):
    """Auto-trim bom açısı (işaretli): + yelken sancağa, - iskeleye, ölü açıda 0."""
    return leeward_sign(heading_deg) * polar_point(heading_deg, profile)[0]


def analyze_points_of_sail_360(heading_deg, profile_name=PROFILE_NAME):
    """
    Seyir durumu analizi. Dönüş: (regime, opt_sail, speed_ratio, tack, wind_side, leeward_sign, desc)
    """
    profile = profile_key(profile_name)
    h = wrap360(heading_deg)
    twa = true_wind_angle(h)
    i = band_index(twa)
    if i is None:
        return (NO_GO_REGIME, 0.0, 0.0, "Kör Nokta", "Pruvadan (Kafadan)", 0.0,
                "Rüzgar doğrudan pruvadan geliyor (330°-030° arası 60° ölü açı). "
                "Yelken rüzgar tutamaz, yapraklar. İtki sıfırdır.")

    opt_sail, ratio = POLAR[profile][i]
    _, _, name_tr, name_en = BANDS[i]
    lee = 1.0 if h <= 180.0 else -1.0
    if i == len(BANDS) - 1:
        regime = f"{name_tr} Seyri ({name_en})"
        tack = "Pupa"
        wind_side = "Kıçtan (Arkadan)"
    elif lee > 0:
        regime = f"İskele Kontra {name_tr} ({name_en})"
        tack = "İskele Kontra"
        wind_side = "İskele (Sol)"
    else:
        regime = f"Sancak Kontra {name_tr} ({name_en})"
        tack = "Sancak Kontra"
        wind_side = "Sancak (Sağ)"
    return regime, opt_sail, ratio, tack, wind_side, lee, DESC[profile][i]


def trim_efficiency(sail_deg, opt_deg, profile=PROFILE):
    """
    Bom açısı ile optimum açı farkına göre aerodinamik trim verimi.
    Dönüş: (eta, durum metni, rozet tipi, tor miktarı)
    """
    p = TRIM[profile]
    sail = abs(sail_deg)
    d = abs(sail - opt_deg)
    if d <= p["tol"]:
        return 1.0 - (d / p["tol"]) * p["loss"], p["full"], "badge-full", p["camber"][0]
    if sail < opt_deg:
        scale, expo, floor = p["under"]
        return max(floor, 1.0 - (d / scale) ** expo), p["under_txt"], "badge-warn", p["camber"][1]
    scale, expo, floor = p["over"]
    return max(floor, 1.0 - (d / scale) ** expo), p["over_txt"], "badge-warn", p["camber"][2]


def rudder_penalty(rudder_deg, phys=PHYS):
    r = np.clip(rudder_deg, -RUDDER_MAX_DEG, RUDDER_MAX_DEG)
    return 1.0 - (abs(r) / RUDDER_MAX_DEG) ** 2 * phys.rudder_drag


def target_speed(heading_deg, sail_deg, rudder_deg, profile, wind_ms, auto_trim=False, phys=PHYS):
    """Kararlı-durum (hedef) tekne hızı V* = λ·U·ρ·η·κ (m/s)."""
    opt, ratio = polar_point(heading_deg, profile)
    if ratio == 0.0:
        return 0.0
    eta = 1.0 if auto_trim else trim_efficiency(sail_deg, opt, profile)[0]
    return wind_ms * ratio * eta * rudder_penalty(rudder_deg, phys) * phys.speed_scale


def polar_speed(heading_deg, profile, wind_ms, phys=PHYS):
    """Optimum trim ve sıfır dümende hız (planlama için)."""
    return wind_ms * polar_point(heading_deg, profile)[1] * phys.speed_scale


def yaw_rate_dps(speed_ms, rudder_deg, heading_deg, phys=PHYS):
    """Dönüş hızı (°/s): kinematik bisiklet modeli + orsaya kaçma eğilimi."""
    if speed_ms <= MIN_STEER_SPEED:
        return 0.0
    r = np.clip(rudder_deg, -RUDDER_MAX_DEG, RUDDER_MAX_DEG)
    rate = np.degrees(speed_ms / phys.hull_length * np.sin(np.radians(r)))
    return rate - leeward_sign(heading_deg) * phys.weather_helm


@dataclass
class BoatState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0   # pruva (°)
    v: float = 0.0     # ileri hız (m/s)


def step(state, sail_deg, rudder_deg, profile, wind_ms, auto_trim=False, dt=DT, tau=None, phys=PHYS):
    """Tek entegrasyon adımı (yarı-örtük Euler; surge için tam üstel çözüm). tau None -> phys.tau."""
    tau = phys.tau if tau is None else tau
    v_star = target_speed(state.psi, sail_deg, rudder_deg, profile, wind_ms, auto_trim, phys)
    if tau > 0.0:
        v = v_star + (state.v - v_star) * np.exp(-dt / tau)
    else:
        v = v_star
    psi = wrap360(state.psi + yaw_rate_dps(v, rudder_deg, state.psi, phys) * dt)
    rad = np.radians(psi)
    return BoatState(state.x + v * np.sin(rad) * dt, state.y + v * np.cos(rad) * dt, psi, v)


def simulate_open_loop(heading_deg, sail_deg, rudder_deg, profile, wind_ms, sim_time_s,
                       auto_trim=False, dt=DT, tau=None, x0=0.0, y0=0.0, phys=PHYS):
    """
    Sabit yelken/dümen girdisiyle açık çevrim simülasyon.
    Tekne başlangıçta kararlı-durum hızında kabul edilir.
    """
    s = BoatState(x0, y0, wrap360(float(heading_deg)), 0.0)
    s.v = target_speed(s.psi, sail_deg, rudder_deg, profile, wind_ms, auto_trim, phys)
    xs, ys, psis, vs = [s.x], [s.y], [s.psi], [s.v]
    dist = 0.0
    for _ in range(int(round(sim_time_s / dt))):
        s = step(s, sail_deg, rudder_deg, profile, wind_ms, auto_trim, dt, tau, phys)
        xs.append(s.x)
        ys.append(s.y)
        psis.append(s.psi)
        vs.append(s.v)
        dist += s.v * dt
    return dict(x=np.array(xs), y=np.array(ys), psi=np.array(psis), v=np.array(vs),
                distance=dist, final_heading=s.psi, v0=vs[0])


# --- KUVVET VE MOMENT HESABI (fiziksel yorum) ---
# Simülasyon hız poları ve kinematik dönüşle çalışır. Aşağıdaki fonksiyon aynı denklemlerden,
# belirtilen fiziksel parametrelerle eşdeğer kuvvetleri (N) ve momentleri (N·m) hesaplar:
#   ileri yön:  m·dv/dt = T_yelken − D_dümen − R_gövde,  R_gövde = k_g·v, τ = m/k_g  ->  dv/dt = (V* − v)/τ
#   yelken:     T = k_g·λ·U·ρ·η (∝ A_s); toplam kuvvet bom kirişine dik (düz levha yaklaşımı):
#               T = F_N·sin δs, F_yan = F_N·cos δs
#   dümen:      Y = ½·ρ_su·A_d·C_L(δr)·v²,  N_dümen = Y·x_d
#   yelken momenti (orsaya kaçma): N_yelken = −s·F_yan·e  (basınç merkezi yanal direnç merkezinin gerisinde)
def apparent_wind(heading_deg, speed_ms, wind_ms):
    """Görünür rüzgar hızı (m/s) ve pruvaya göre açısı AWA (°). Kuzey rüzgarı + teknenin kendi hareketi."""
    c = np.cos(np.radians(heading_deg))
    va = float(np.sqrt(speed_ms ** 2 + wind_ms ** 2 + 2.0 * speed_ms * wind_ms * c))
    if va < 1e-9:
        return 0.0, 0.0
    return va, float(np.degrees(np.arccos(np.clip((speed_ms + wind_ms * c) / va, -1.0, 1.0))))


def force_moment_breakdown(heading_deg, sail_deg, rudder_deg, wind_ms, speed_ms=None, auto_trim=False,
                           profile=PROFILE, phys=PHYS, tau=None):
    """
    Verilen pruva / yelken / dümen açısında tekneye etkiyen kuvvetler (N) ve momentler (N·m).
    speed_ms None ise tekne kararlı hızda (v = V*) kabul edilir; o zaman net ileri kuvvet sıfırdır.
    tau None ise τ = m / k_g (phys); verilirse gövde direnç katsayısı k_g = m / τ alınır.
    İşaretler: momentte + sancağa (ψ artar) döndürür; yan kuvvet rüzgar altı bordaya doğrudur.
    """
    psi = wrap360(heading_deg)
    opt, ratio = polar_point(psi, profile)
    s = leeward_sign(psi)
    sail = opt if auto_trim else abs(sail_deg)
    eta = 1.0 if (auto_trim or ratio == 0.0) else trim_efficiency(sail, opt, profile)[0]
    kappa = rudder_penalty(rudder_deg, phys)
    lam = phys.speed_scale
    v_star = wind_ms * ratio * eta * kappa * lam
    v = v_star if speed_ms is None else float(speed_ms)
    if tau is None:
        k, tau = phys.hull_drag, phys.tau                 # doğrusal direnç katsayısı (N·s/m)
    else:
        k = phys.mass / tau

    # İleri yön kuvvet dengesi
    drive = k * lam * wind_ms * ratio * eta               # yelkenin ileri itkisi (∝ A_s)
    rudder_drag = drive * (1.0 - kappa)                   # dümen palasının sürüklemesi
    hull_drag = k * v                                     # gövde direnci
    net = drive - rudder_drag - hull_drag                 # = m·(V* − v)/τ

    # Yelken kuvveti: kirişe dik toplam kuvvet -> ileri ve yan bileşen
    sin_s = np.sin(np.radians(max(sail, 5.0)))
    normal = drive / sin_s if drive > 0 else 0.0
    side = normal * np.cos(np.radians(sail))
    va, awa = apparent_wind(psi, v, wind_ms)
    q_air = 0.5 * phys.rho_air * va ** 2
    c_n = normal / (q_air * phys.sail_area) if q_air > 0 else 0.0

    # Dümen kuvveti ve momenti
    r = float(np.clip(rudder_deg, -RUDDER_MAX_DEG, RUDDER_MAX_DEG))
    q_water = 0.5 * phys.rho_water * phys.rudder_area * v ** 2
    cl = float(np.clip(phys.rudder_cla * np.radians(r), -phys.rudder_clmax, phys.rudder_clmax))
    rudder_force = q_water * cl
    rudder_moment = rudder_force * phys.rudder_arm
    sail_moment = -s * side * phys.ce_offset              # orsaya kaçma: tekneyi rüzgara döndürür
    balance = (np.degrees(-sail_moment / (phys.rudder_arm * q_water * phys.rudder_cla))
               if q_water > 0 else 0.0)                   # yelken momentini dengeleyen dümen açısı

    # Dönüş (kinematik model) ve gereken merkezcil kuvvet
    yaw = yaw_rate_dps(v, r, psi, phys)
    w = np.radians(yaw)
    out = dict(
        tau=tau, hull_coef=k, speed_scale=lam, weather_helm=phys.weather_helm, ratio=ratio,
        twa=true_wind_angle(psi), leeward=s, sail=sail, opt=opt, eta=eta, kappa=kappa,
        v_star=v_star, v=v, drive=drive, rudder_drag=rudder_drag, hull_drag=hull_drag, net=net,
        accel=net / phys.mass, normal=normal, side=side, va=va, awa=awa, c_n=c_n,
        rudder_cl=cl, rudder_force=rudder_force, rudder_moment=rudder_moment, sail_moment=sail_moment,
        net_moment=rudder_moment + sail_moment, rudder_balance=float(balance),
        yaw_rate=yaw, turn_radius=(v / abs(w) if abs(w) > 1e-9 else float("inf")),
        centripetal=phys.mass * v * abs(w),
    )
    return {key: float(val) for key, val in out.items()}
