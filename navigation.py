"""
Noktadan noktaya otonom navigasyon (güdüm + dümen kontrolü) – saildrone_model üzerine kurulur.

Güdüm:   Her güdüm periyodunda iki kontra (iskele/sancak) için hedefe doğru hız bileşenini
         (VMG) maksimize eden pruva aranır. Tekne mevcut kontrada kalır; diğer kontranın
         VMG'si daha yüksekse ve
           (a) mevcut kontranın VMG'si pozitif değilse (hedef geride kaldı), ya da
               asgari bacak süresi dolmuşken
           (b) hedef diğer kontranın layline'ına gelmişse veya
           (c) tekne başlangıç-hedef hattından koridor yarı genişliğinden fazla
               uzaklaşmış ve hâlâ uzaklaşıyorsa
         kontra değiştirir.
Manevra: Yeni pruvaya giden en kısa dönüş ölü açıdan geçiyorsa (tack), mevcut hızla ölü
         açının aşılıp aşılamayacağı kestirilir; aşılamıyorsa tekne rüzgar altından döner
         ("kuyruk çevirme" / gybe-around). Bu durumda kontra değişimi, manevra ileriye
         doğru simüle edilerek manevra SONRASINDA hedefin tutulabileceği garanti edilince
         yapılır (aksi halde hedef çevresinde sonsuz döngü oluşur).
Dümen:   İstenen dönüş hızı r* = k·e (+ orsaya kaçma ileri beslemesi); kinematik modelin
         tersinden delta = asin(r*·L / v), ±35° ile sınırlı.
"""
from dataclasses import dataclass, field

import numpy as np

import saildrone_model as m


@dataclass
class NavConfig:
    arrival_radius: float = 10.0      # Varış yarıçapı (m)
    max_time: float = 7200.0          # Görev süre sınırı (s)
    dt: float = m.DT
    guidance_period: float = 1.0      # Güdüm güncelleme periyodu (s)
    no_go_margin: float = 5.0         # Ölü açı sınırına güvenlik payı (°)
    corridor: float = None            # Koridor yarı genişliği (m); None -> 0.2·D, [50, 1000]
    layline_tol: float = 1.0          # Layline açı toleransı (°)
    min_leg_time: float = 10.0        # İki kontra değişimi arası asgari süre (s)
    heading_gain: float = 0.5         # Pruva hatası -> dönüş hızı kazancı (1/s)
    max_turn_rate: float = 30.0       # İstenen azami dönüş hızı (°/s)
    tack_safety: float = 1.3          # Tack fizibilite güvenlik katsayısı
    allow_tack: bool = True           # False: kontra yalnızca rüzgar altından değiştirilir
    stuck_time: float = 20.0          # Ölü açıda bu süre ~sıfır hızda kalırsa başarısız
    tau: float = m.TAU_SURGE


@dataclass
class NavResult:
    reached: bool
    reason: str
    time: float
    path_length: float
    straight_distance: float
    final_distance: float
    n_tacks: int
    n_gybes: int
    t: np.ndarray = field(repr=False)
    x: np.ndarray = field(repr=False)
    y: np.ndarray = field(repr=False)
    psi: np.ndarray = field(repr=False)
    v: np.ndarray = field(repr=False)
    rudder: np.ndarray = field(repr=False)
    psi_des: np.ndarray = field(repr=False)


def bearing_deg(x0, y0, x1, y1):
    """(x0,y0)'dan (x1,y1)'e pusula kerterizi."""
    return m.wrap360(np.degrees(np.arctan2(x1 - x0, y1 - y0)))


_TWA_GRID = np.arange(0.0, 180.0 + 1e-9, 0.5)
_RATIO_CACHE = {p: np.array([m.polar_point(t, p)[1] for t in _TWA_GRID]) for p in m.PROFILE_KEYS}


def best_heading_per_side(profile, wind_ms, bearing, margin):
    """
    İki kontra için VMG-optimum pruva: {side: (pruva, VMG, hız)}.
    side +1: psi ∈ (0,180] (iskele kontra), side -1: psi ∈ [180,360) (sancak kontra).
    """
    ok = _TWA_GRID > m.NO_GO_DEG + margin
    twa = _TWA_GRID[ok]
    spd = wind_ms * _RATIO_CACHE[profile][ok]
    out = {}
    for side in (1, -1):
        hdg = twa if side == 1 else 360.0 - twa
        vmg = spd * np.cos(np.radians(hdg - bearing))
        k = int(np.argmax(vmg))
        out[side] = (m.wrap360(hdg[k]), float(vmg[k]), float(spd[k]))
    return out


def on_layline(h_o, side_o, bearing, tol):
    """
    Diğer kontrada (side_o) VMG-optimum rota h_o ile hedef "tutuluyor" mu?
    Rüzgar üstü (TWA<90): hedef h_o rota çizgisinin rüzgar üstünde DEĞİLSE layline'a gelinmiştir.
    Rüzgar altı (TWA>=90): hedef h_o rota çizgisinin rüzgar altında DEĞİLSE layline'a gelinmiştir.
    side +1 (iskele kontra) için rüzgar üstü taraf soldur, side -1 için sağdır.
    """
    off = m.wrap180(bearing - h_o)
    if abs(off) >= 90.0:
        return False
    windward_off = side_o * off          # < 0: hedef rotanın rüzgar üstünde
    if m.true_wind_angle(h_o) < 90.0:
        return windward_off >= -tol
    return windward_off <= tol


def _side_of(psi):
    return 1 if m.wrap360(psi) <= 180.0 else -1


def _crosses_no_go(psi, err):
    """psi'den err kadar (işaretli) dönüş ölü açıdan (kuzeyden) geçer mi?"""
    return np.floor((psi + err) / 360.0) != np.floor(psi / 360.0) or m.true_wind_angle(psi) <= m.NO_GO_DEG


def tack_feasible(psi, v, cfg):
    """
    Ölü açıya girildiğinde hedef hız 0 olur ve hız v·exp(-t/tau) ile söner. Tam dümende
    dönülebilecek açı ≈ v·tau·sin(delta_max)/L (rad). Gereken açı ≈ ölü açı genişliği + pay.
    """
    capacity = np.degrees(v * cfg.tau * np.sin(np.radians(m.RUDDER_MAX_DEG)) / m.L_HULL)
    need = 2.0 * m.NO_GO_DEG + cfg.no_go_margin
    if m.true_wind_angle(psi) <= m.NO_GO_DEG:
        need = m.NO_GO_DEG + cfg.no_go_margin + m.true_wind_angle(psi)
    return capacity >= cfg.tack_safety * need


def _turn_direction(s, psi_des, profile, wind_ms, cfg):
    """
    İstenen pruvaya dönüş yönü (+1 sancağa / -1 iskeleye) ve bunun bir tack olup olmadığı.
    Kısa yol ölü açıdan geçiyorsa tack, aynı dümen yasasıyla ileri simüle edilir; tekne ölü
    açıyı geçip hedef pruvaya oturamıyorsa rüzgar altından dönülür.
    """
    err = m.wrap180(psi_des - s.psi)
    d = 1 if err > 0 else -1
    twa = m.true_wind_angle(s.psi)
    if twa <= m.NO_GO_DEG:
        # Zaten ölü açının içinde: hız sönmeden en yakın sınırdan çık
        # (psi ∈ (0,30] -> sancağa, psi ∈ [330,360) -> iskeleye; psi=0 -> kısa yol).
        if twa == 0.0:
            return d, True
        return (1 if m.wrap360(s.psi) <= 180.0 else -1), True
    if _crosses_no_go(s.psi, err):
        if cfg.allow_tack and tack_feasible(s.psi, s.v, cfg):
            p = _predict_maneuver(s, psi_des, d, profile, wind_ms, cfg, horizon=60.0)
            if abs(m.wrap180(psi_des - p.psi)) < 5.0 and m.true_wind_angle(p.psi) > m.NO_GO_DEG:
                return d, True
        return -d, False            # tack mümkün değil -> rüzgar altından dön
    return d, False


def _rudder_cmd(psi, v, err, cfg):
    r_des = np.clip(cfg.heading_gain * err, -cfg.max_turn_rate, cfg.max_turn_rate)
    r_des += m.leeward_sign(psi) * m.WEATHER_HELM_DPS        # orsaya kaçma ileri beslemesi
    if v <= m.MIN_STEER_SPEED:
        return float(np.sign(err) * m.RUDDER_MAX_DEG)
    lim = np.sin(np.radians(m.RUDDER_MAX_DEG))
    return float(np.degrees(np.arcsin(np.clip(np.radians(r_des) * m.L_HULL / v, -lim, lim))))


def _signed_error(psi, psi_des, turn_dir):
    err = m.wrap180(psi_des - psi)
    if turn_dir != 0 and np.sign(err) != turn_dir and abs(err) > 1.0:
        err += 360.0 * turn_dir
    return err


def _predict_maneuver(s, psi_des, turn_dir, profile, wind_ms, cfg, horizon=90.0):
    """Manevrayı (psi_des'e dönüş) aynı dümen yasasıyla ileri simüle eder; son durumu döndürür."""
    p = m.BoatState(s.x, s.y, s.psi, s.v)
    for _ in range(int(horizon / cfg.dt)):
        err = _signed_error(p.psi, psi_des, turn_dir)
        if abs(err) < 5.0:
            break
        p = m.step(p, 0.0, _rudder_cmd(p.psi, p.v, err, cfg), profile, wind_ms, True, cfg.dt, cfg.tau)
    return p


def navigate(start, target, profile, wind_ms, initial_heading=None, cfg=None):
    """
    start, target: (x, y) metre. profile: '0012' | '0015' | 'cloth' (veya arayüz adı).
    initial_heading: None ise tekne hedefe en iyi VMG pruvasıyla ve kararlı hızda başlar.
    """
    cfg = cfg or NavConfig()
    if profile not in m.PROFILE_KEYS:
        profile = m.profile_key(profile)
    sx, sy = start
    tx, ty = target

    d0 = float(np.hypot(tx - sx, ty - sy))
    corridor = cfg.corridor if cfg.corridor is not None else float(np.clip(0.2 * d0, 50.0, 1000.0))
    nx, ny = ((ty - sy) / d0, -(tx - sx) / d0) if d0 > 0 else (1.0, 0.0)   # rota hattı normali

    opts = best_heading_per_side(profile, wind_ms, bearing_deg(sx, sy, tx, ty), cfg.no_go_margin)
    if initial_heading is None:
        side = max(opts, key=lambda sd: opts[sd][1])
        psi0 = opts[side][0]
    else:
        psi0 = m.wrap360(float(initial_heading))
        side = _side_of(psi0)

    s = m.BoatState(sx, sy, psi0, 0.0)
    s.v = m.target_speed(psi0, 0.0, 0.0, profile, wind_ms, auto_trim=True)

    psi_des = psi0
    T, X, Y, P, V, R, D = [0.0], [s.x], [s.y], [s.psi], [s.v], [0.0], [psi0]

    def log(t, rudder):
        T.append(t)
        X.append(s.x)
        Y.append(s.y)
        P.append(s.psi)
        V.append(s.v)
        R.append(rudder)
        D.append(psi_des)

    path = 0.0
    n_tacks = n_gybes = 0
    turn_dir, turn_for = 0, psi0      # taahhüt edilmiş dönüş yönü ve ait olduğu hedef pruva
    t_switch = -1e9
    stuck = 0.0
    reached, reason = False, "Süre sınırı aşıldı"
    prev_side = _side_of(psi0) if m.true_wind_angle(psi0) > m.NO_GO_DEG else 0
    g_every = max(1, int(round(cfg.guidance_period / cfg.dt)))

    for k in range(int(round(cfg.max_time / cfg.dt))):
        t = k * cfg.dt
        dist = np.hypot(tx - s.x, ty - s.y)
        if dist <= cfg.arrival_radius:
            reached, reason = True, "Hedefe ulaşıldı"
            break

        # --- GÜDÜM ---
        if k % g_every == 0:
            b = bearing_deg(s.x, s.y, tx, ty)
            opts = best_heading_per_side(profile, wind_ms, b, cfg.no_go_margin)
            other = -side
            cur_vmg, oth_vmg = opts[side][1], opts[other][1]
            h = np.radians(opts[side][0])
            xtrack = (s.x - sx) * nx + (s.y - sy) * ny
            moving_out = xtrack * (np.sin(h) * nx + np.cos(h) * ny) > 0.0
            leg_done = (t - t_switch) >= cfg.min_leg_time
            want = oth_vmg > cur_vmg and (
                cur_vmg <= 0.0
                or (leg_done and on_layline(opts[other][0], other, b, cfg.layline_tol))
                or (leg_done and abs(xtrack) > corridor and moving_out))
            if want:
                h_new = opts[other][0]
                d_new, is_tack = _turn_direction(s, h_new, profile, wind_ms, cfg)
                upwind = m.true_wind_angle(s.psi) < 90.0 and m.true_wind_angle(h_new) < 90.0
                if upwind and not is_tack:
                    # Rüzgar altından dönüş: manevra sonrası konumdan hedef tutulabiliyor mu?
                    p = _predict_maneuver(s, h_new, d_new, profile, wind_ms, cfg)
                    b2 = bearing_deg(p.x, p.y, tx, ty)
                    o2 = best_heading_per_side(profile, wind_ms, b2, cfg.no_go_margin)
                    want = (np.hypot(tx - p.x, ty - p.y) <= cfg.arrival_radius
                            or on_layline(o2[other][0], other, b2, cfg.layline_tol))
            if want:
                side = other
                t_switch = t
            psi_des = opts[side][0]

        # --- MANEVRA YÖNÜ ---
        in_irons = m.true_wind_angle(s.psi) <= m.NO_GO_DEG
        if abs(m.wrap180(psi_des - s.psi)) < 10.0 or (abs(m.wrap180(psi_des - turn_for)) > 5.0 and not in_irons):
            turn_dir = 0                  # manevra bitti ya da hedef pruva değişti -> yeniden karar
                                          # (ölü açı içindeyken, yani tack ortasında, yön korunur)
        if turn_dir == 0 and abs(m.wrap180(psi_des - s.psi)) > 20.0:
            turn_dir, _ = _turn_direction(s, psi_des, profile, wind_ms, cfg)
            turn_for = psi_des
        err = _signed_error(s.psi, psi_des, turn_dir)

        # --- DÜMEN + DİNAMİK ---
        rudder = _rudder_cmd(s.psi, s.v, err, cfg)
        prev_psi = s.psi
        s = m.step(s, 0.0, rudder, profile, wind_ms, auto_trim=True, dt=cfg.dt, tau=cfg.tau)
        path += s.v * cfg.dt

        # Kontra değişimi sayımı (ölü açı ve tam pupa ±5° dışında taraf değiştiyse)
        if m.NO_GO_DEG < m.true_wind_angle(s.psi) < 175.0:
            cur_side = _side_of(s.psi)
            if prev_side and cur_side != prev_side:
                if m.true_wind_angle(prev_psi) <= 90.0 or m.true_wind_angle(s.psi) <= 90.0:
                    n_tacks += 1
                else:
                    n_gybes += 1
            prev_side = cur_side

        log(t + cfg.dt, rudder)

        # Ölü açıda takılma tespiti
        if m.true_wind_angle(s.psi) <= m.NO_GO_DEG and s.v < 0.05:
            stuck += cfg.dt
            if stuck >= cfg.stuck_time:
                reason = "Ölü açıda takıldı (in irons) – hız sıfır, dümen etkisiz"
                break
        else:
            stuck = 0.0

    return NavResult(reached, reason, T[-1], path, d0, float(np.hypot(tx - s.x, ty - s.y)),
                     n_tacks, n_gybes, np.array(T), np.array(X), np.array(Y), np.array(P),
                     np.array(V), np.array(R), np.array(D))
