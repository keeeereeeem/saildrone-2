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


def rudder_penalty(rudder_deg):
    r = np.clip(rudder_deg, -RUDDER_MAX_DEG, RUDDER_MAX_DEG)
    return 1.0 - (abs(r) / RUDDER_MAX_DEG) ** 2 * RUDDER_DRAG


def target_speed(heading_deg, sail_deg, rudder_deg, profile, wind_ms, auto_trim=False):
    """Kararlı-durum (hedef) tekne hızı V* (m/s)."""
    opt, ratio = polar_point(heading_deg, profile)
    if ratio == 0.0:
        return 0.0
    eta = 1.0 if auto_trim else trim_efficiency(sail_deg, opt, profile)[0]
    return wind_ms * ratio * eta * rudder_penalty(rudder_deg)


def polar_speed(heading_deg, profile, wind_ms):
    """Optimum trim ve sıfır dümende hız (planlama için)."""
    return wind_ms * polar_point(heading_deg, profile)[1]


def yaw_rate_dps(speed_ms, rudder_deg, heading_deg):
    """Dönüş hızı (°/s): kinematik bisiklet modeli + orsaya kaçma eğilimi."""
    if speed_ms <= MIN_STEER_SPEED:
        return 0.0
    r = np.clip(rudder_deg, -RUDDER_MAX_DEG, RUDDER_MAX_DEG)
    rate = np.degrees(speed_ms / L_HULL * np.sin(np.radians(r)))
    return rate - leeward_sign(heading_deg) * WEATHER_HELM_DPS


@dataclass
class BoatState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0   # pruva (°)
    v: float = 0.0     # ileri hız (m/s)


def step(state, sail_deg, rudder_deg, profile, wind_ms, auto_trim=False, dt=DT, tau=TAU_SURGE):
    """Tek entegrasyon adımı (yarı-örtük Euler; surge için tam üstel çözüm)."""
    v_star = target_speed(state.psi, sail_deg, rudder_deg, profile, wind_ms, auto_trim)
    if tau > 0.0:
        v = v_star + (state.v - v_star) * np.exp(-dt / tau)
    else:
        v = v_star
    psi = wrap360(state.psi + yaw_rate_dps(v, rudder_deg, state.psi) * dt)
    rad = np.radians(psi)
    return BoatState(state.x + v * np.sin(rad) * dt, state.y + v * np.cos(rad) * dt, psi, v)


def simulate_open_loop(heading_deg, sail_deg, rudder_deg, profile, wind_ms, sim_time_s,
                       auto_trim=False, dt=DT, tau=TAU_SURGE, x0=0.0, y0=0.0):
    """
    Sabit yelken/dümen girdisiyle açık çevrim simülasyon.
    Tekne başlangıçta kararlı-durum hızında kabul edilir.
    """
    s = BoatState(x0, y0, wrap360(float(heading_deg)), 0.0)
    s.v = target_speed(s.psi, sail_deg, rudder_deg, profile, wind_ms, auto_trim)
    xs, ys, psis, vs = [s.x], [s.y], [s.psi], [s.v]
    dist = 0.0
    for _ in range(int(round(sim_time_s / dt))):
        s = step(s, sail_deg, rudder_deg, profile, wind_ms, auto_trim, dt, tau)
        xs.append(s.x)
        ys.append(s.y)
        psis.append(s.psi)
        vs.append(s.v)
        dist += s.v * dt
    return dict(x=np.array(xs), y=np.array(ys), psi=np.array(psis), v=np.array(vs),
                distance=dist, final_heading=s.psi, v0=vs[0])
