"""
A → B düz rota takibi: kaskad PID ile çizgi tutma (Streamlit'ten bağımsız, test edilebilir).

Tekne A'dan B'ye çizilen doğrunun üzerinde kalmaya çalışır. Çizgiden sapma (rota sapması, XTE) iki katlı
(kaskad) bir PID kontrolcüyle düzeltilir:

    dış döngü:  e (m)            --PID-->  Δψ (°)   hedef pruva ψ* = β + Δψ + ψ_İB   (ölü açıya girmeyecek şekilde sınırlı)
    iç döngü:   ε = ψ* − ψ (°)   --PID-->  δr,komut = δ_İB + PID (°) --dümen servosu (hız sınırlı)--> δr

* İleri besleme (İB): ψ_İB = −s·λ (yan kayma kadar rüzgara pay), δ_İB = orsaya kaçmayı dengeleyen dümen.
  PID yalnızca kalan hatayı düzeltir.
* Hıza göre kazanç ölçekleme: Kd = Kd_ref·sat(v_ref/v, 1, 6). Dümen etkisi hızla orantılı olduğundan
  (T'σ² + (1 + (v/L)·Kd)σ + Kp = 0, σ: boyutsuz zaman) sönüm oranı ancak böyle hızdan bağımsız kalır.
* Koşullu entegrasyon (anti-windup): I terimleri yalnızca hata küçükken (|e| < 10 m, |ε| < 10°) birikir.
* Yelken auto-trim'dedir; dümen rüzgar altına doyduğu hâlde tekne rüzgardan uzaklaşamıyorsa
  (orsaya kaçma dümeni yeniyorsa) yelken boşaltılır.

Bu bölümün dinamiği diğer sekmelerden daha ayrıntılıdır (kararlı durumda aynı denklemlere iner):
* Dönüş ataleti (birinci derece Nomoto modeli):  T·ṙ + r = r_ss,  T = T'·L / v
  r_ss = (v/L)·sin δr  +  r_yelken,  r_yelken = −s·F_yan·e / (½·ρ_su·A_d·C_Lα·x_d·L·v)
  (yelken momentinin, dümen kinematiğiyle aynı ölçekte dönüş hızına çevrilmiş hâli: orsaya kaçma
  yelkenin yan kuvvetiyle orantılıdır; tam apazda ≈ 0.35 °/s, yelken boşaltılınca azalır)
* Dümen servosu: |δ̇r| ≤ dümen hızı (°/s), |δr| ≤ 35°.
* Yan kayma (leeway): yelkenin yan kuvvetini salma + gövdelerin yanal direnci karşılar. Modelin ileri yön
  direnci doğrusal olduğundan (R = k_g·v) yanal direnç de doğrusaldır: F_yan = k_yan·v_yan, k_yan = n_yan·k_g
  -> v_yan = F_yan / k_yan (rüzgar altına). Kararlı hızda yan kayma açısı rüzgar hızından bağımsızdır:
  tan λ = cot δs / n_yan (n_yan = 50: orsada ≈ 4°, tam apazda ≈ 1.1°, pupada ≈ 0.2°).
* İleri hız ve yelken itkisi saildrone_model ile aynıdır (polar, trim verimi, dümen sürüklemesi, τ = m/k_g).

Koordinatlar ve işaretler saildrone_model ile aynıdır (x = Doğu, y = Kuzey, pusula açısı, δr > 0 sancağa).
Rota sapması e > 0: tekne A→B yönüne göre çizginin sağında (sancak tarafında).
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np

import saildrone_model as m

DEAD_ZONE_MARGIN = 5.0      # hedef pruva ölü açı sınırına bu kadar yaklaşabilir (TWA ≥ 35°)
SETTLE_BAND = 2.0           # |e| bu bandın içinde kalınca rota "oturmuş" sayılır (m)
STOP_SPEED = 0.05           # bu hızın altında 5 s kalan tekne "durdu" sayılır (m/s)
I_ZONE_XTE = 10.0           # dış döngü I terimi yalnızca |e| bu değerden küçükken birikir (m)
I_ZONE_HDG = 10.0           # iç döngü I terimi yalnızca |ε| bu değerden küçükken birikir (°)
I_MAX_HDG = 10.0            # iç döngü I katkısı sınırı (°)


@dataclass(frozen=True)
class LineConfig:
    # Varsayılan kazançlar: 12 senaryoluk (iki kontra, orsa–pupa, 3–15 m/s rüzgar, ±25–30 m sapma, ±25° pruva
    # hatası) ızgara taramasında aşım + RMS sapma + dümen kullanımı + oturma süresi toplamını en aza indiren değerler.
    # Dış döngü: rota sapması e (m) -> pruva düzeltmesi Δψ (°)
    kp_xte: float = 3.0             # °/m
    ki_xte: float = 0.02            # °/(m·s)
    kd_xte: float = 4.0             # °/(m/s)  (sapma hızı ė = hızın çizgi normali bileşeni, GPS'ten)
    max_intercept: float = 30.0     # |Δψ| sınırı (°): çizgiye en fazla bu açıyla yaklaşılır
    # İç döngü: pruva hatası ε (°) -> dümen komutu δr (°)
    kp_hdg: float = 1.0             # °/°
    ki_hdg: float = 0.05            # 1/s
    kd_hdg: float = 1.5             # s (dönüş hızı geri beslemesi: −Kd·r)
    # Hıza göre kazanç ölçekleme (gain scheduling): dümen etkisi hızla orantılı olduğundan türev kazançları
    # Kd = Kd_ref·sat(v_ref / v, 1, 6) ile büyütülür; böylece pruva döngüsünün sönüm oranı hızdan bağımsız kalır.
    gain_schedule: bool = True
    schedule_outer: bool = True     # dış döngünün D terimi de ölçeklenir (ė = v·sin(ψ − β) de hızla orantılı)
    v_ref: float = 8.0              # kazançların ayarlandığı referans hız (m/s)
    # İleri besleme (feedforward): kontrolcü yelken kuvvetinden orsaya kaçmayı ve yan kayma açısını hesaplar;
    # dümene orsaya kaçmayı dengeleyen açı (Rotayı Tut ile aynı), hedef pruvaya yan kayma kadar rüzgar payı eklenir.
    # PID yalnızca kalan hatayı düzeltir (yavaş seyirde P teriminin kalıcı pruva hatası tekneyi ölü açıya sokmaz).
    feedforward: bool = True
    # Dinamik
    yaw_t_nd: float = 2.0           # boyutsuz Nomoto zaman sabiti T' (T = T'·L/v) – varsayım
    rudder_rate: float = 10.0       # dümen servosu en büyük açılma hızı (°/s) – varsayım
    # Yan kayma
    leeway: bool = True
    lat_ratio: float = 50.0         # yanal / ileri direnç oranı n_yan = k_yan / k_g – orsada ≈ 4° yan kaymaya kalibre
    # Başlangıç sapması
    start_offset: float = 0.0       # A'nın çizgiye dik yöndeki kaydırması (m, + çizginin sağı)
    start_heading_err: float = 0.0  # başlangıç pruvasının çizgi yönünden farkı (°, + sancağa)
    # Yelken
    depower: bool = True            # dümen orsaya kaçmayı yenemezse yelkeni boşalt
    depower_rate: float = 1.0       # dümen tam açıkken bu dönüş hızına (°/s, rüzgar altına) ulaşılamıyorsa boşalt
    depower_time: float = 2.0       # tam boşaltma süresi (s)
    recover_time: float = 4.0       # yeniden kasma süresi (s)
    # Sayısal çözüm ve görev
    dt: float = 0.1
    max_time: Optional[float] = None    # None: 3·L/V* + 120 s
    allow_dead_zone: bool = False       # True: hedef pruva ölü açıya girebilir (rota ölü açıdaysa "izin ver")


def line_frame(a, b):
    """A→B doğrusu: (uzunluk L, kerteriz β, birim yön u, sağ normal n)."""
    dx, dy = float(b[0]) - float(a[0]), float(b[1]) - float(a[1])
    length = float(np.hypot(dx, dy))
    if length < 1e-9:
        raise ValueError("A ve B aynı nokta olamaz")
    u = np.array([dx, dy]) / length
    n = np.array([u[1], -u[0]])                     # sağ normal = (cos β, −sin β)
    bearing = m.wrap360(round(float(np.degrees(np.arctan2(dx, dy))), 9))   # ölü açı sınırında kayan nokta payı
    return length, bearing, u, n


def line_status(a, b):
    """Rotanın rüzgara göre durumu: (kerteriz β, TWA_β, 'dead' | 'edge' | 'ok')."""
    _, bearing, _, _ = line_frame(a, b)
    twa = m.true_wind_angle(bearing)
    if twa <= m.NO_GO_DEG:
        return bearing, twa, "dead"
    if twa < m.NO_GO_DEG + DEAD_ZONE_MARGIN:
        return bearing, twa, "edge"
    return bearing, twa, "ok"


def clamp_heading_to_sailable(psi_des, bearing):
    """Hedef pruvayı, çizginin kontrasında ölü açı sınırının (TWA ≥ 35°) dışında tutar."""
    lim = m.NO_GO_DEG + DEAD_ZONE_MARGIN
    if m.true_wind_angle(psi_des) >= lim:
        return psi_des
    return lim if m.wrap360(bearing) <= 180.0 else 360.0 - lim


def sail_state(psi, depower, phys=m.PHYS):
    """Auto-trim + boşaltma: (işaretli bom açısı, mutlak bom, η, polar oranı, TWA, rüzgar altı bordası s)."""
    twa = m.true_wind_angle(psi)
    s = m.leeward_sign(psi)
    opt, ratio = m.polar_point(psi)
    if ratio == 0.0:
        return 0.0, 0.0, 0.0, 0.0, twa, s
    ease_max = max(opt, min(90.0, twa - 4.0))       # en fazla α ≈ 4°'ye kadar boşaltılır (η ≤ 0.5)
    sail = opt + depower * (ease_max - opt)
    eta = m.trim_efficiency(sail, opt, m.PROFILE, twa)[0]
    return s * sail, sail, eta, ratio, twa, s


def sail_forces(wind_ms, ratio, eta, sail, phys=m.PHYS):
    """Yelken itkisi T ve yan kuvvet F_yan (N) – saildrone_model.force_moment_breakdown ile aynı denklemler."""
    drive = phys.hull_drag * phys.speed_scale * wind_ms * ratio * eta
    if drive <= 0.0:
        return 0.0, 0.0
    normal = drive / np.sin(np.radians(max(sail, 5.0)))
    return drive, normal * np.cos(np.radians(sail))


def leeway_speed(side_force, cfg, phys=m.PHYS):
    """Yan kayma hızı v_yan (m/s, rüzgar altına): F_yan = k_yan·v_yan, k_yan = n_yan·k_g."""
    if not cfg.leeway or side_force <= 0.0:
        return 0.0
    return float(side_force / (cfg.lat_ratio * phys.hull_drag))


def holding_rudder(r_sail, speed, phys=m.PHYS):
    """Yelken momentinin dönüşünü dengeleyen dümen açısı (°): (v/L)·sin δ + r_yelken = 0."""
    if speed <= m.MIN_STEER_SPEED:
        return 0.0
    lim = np.sin(np.radians(m.RUDDER_MAX_DEG))
    return float(np.degrees(np.arcsin(np.clip(-np.radians(r_sail) * phys.hull_length / speed, -lim, lim))))


def sail_yaw_rate(side_force, s, speed, phys=m.PHYS):
    """Yelken momentinin dönüş hızı karşılığı (°/s, − : rüzgara doğru); dümen kinematiğiyle aynı ölçek."""
    if speed <= m.MIN_STEER_SPEED:
        return 0.0
    denom = 0.5 * phys.rho_water * phys.rudder_area * phys.rudder_cla * phys.rudder_arm * phys.hull_length * speed
    return float(-s * np.degrees(side_force * phys.ce_offset / denom))


@dataclass
class LineResult:
    a: tuple
    b: tuple
    bearing: float
    length: float
    status: str                 # "reached" | "stopped" | "timeout"
    reason: str
    data: dict                  # zaman serileri (numpy dizileri)
    metrics: dict


def simulate_line(a, b, wind_ms, cfg=LineConfig(), phys=m.PHYS):
    """A'dan B'ye düz rota takibi simülasyonu (kaskad PID + ataletli dönüş + yan kayma)."""
    length, bearing, u, n = line_frame(a, b)
    dt = cfg.dt
    a = np.asarray(a, dtype=float)
    p = a + cfg.start_offset * n
    psi = m.wrap360(bearing + cfg.start_heading_err)
    if cfg.allow_dead_zone and m.true_wind_angle(psi) <= m.NO_GO_DEG:
        psi = clamp_heading_to_sailable(psi, bearing)     # ölü açıdaki rota: tekne en yakın seyir pruvasından başlar
    _, sail0, eta0, ratio0, _, _ = sail_state(psi, 0.0, phys)
    v = wind_ms * ratio0 * eta0 * phys.speed_scale          # başlangıçta kararlı hız (diğer sekmelerle aynı)
    r, delta, depower = 0.0, 0.0, 0.0
    i_xte, i_hdg = 0.0, 0.0
    tau = phys.tau
    v_line = m.polar_speed(bearing, m.PROFILE, wind_ms, phys)
    t_max = cfg.max_time if cfg.max_time is not None else 3.0 * length / max(v_line, 0.5) + 120.0

    keys = ("t x y psi psi_des r v v_lat cog e along delta delta_cmd sail depower twa eta "
            "p_xte i_xte d_xte p_hdg i_hdg d_hdg drive side hull_drag rudder_force rudder_moment sail_moment "
            "r_rudder r_sail leeway_angle ff_hdg crab").split()
    rec = {k: [] for k in keys}
    t, stopped_for, status, reason = 0.0, 0.0, "timeout", "süre sınırı doldu"

    while True:
        rel = p - a
        e = float(rel @ n)
        along = float(rel @ u)
        signed_sail, sail, eta, ratio, twa, s = sail_state(psi, depower, phys)
        drive, side = sail_forces(wind_ms, ratio, eta, sail, phys)
        v_lat = leeway_speed(side, cfg, phys)
        lat_dir = np.array([np.cos(np.radians(psi)), -np.sin(np.radians(psi))])   # sancak yönü
        vel = v * np.array([np.sin(np.radians(psi)), np.cos(np.radians(psi))]) + s * v_lat * lat_dir
        e_dot = float(vel @ n)

        # --- Dış döngü: rota sapması -> pruva düzeltmesi ---
        g = float(np.clip(cfg.v_ref / max(v, 0.5), 1.0, 6.0)) if cfg.gain_schedule else 1.0
        p_x, d_x = cfg.kp_xte * e, cfg.kd_xte * (g if cfg.schedule_outer else 1.0) * e_dot
        u_x = p_x + cfg.ki_xte * i_xte + d_x
        sat_x = abs(u_x) > cfg.max_intercept
        # koşullu entegrasyon (anti-windup): büyük düzeltmeyi P ve D yapar, I yalnızca kalan küçük hatayı toplar
        if abs(e) < I_ZONE_XTE and (not sat_x or np.sign(e) != np.sign(u_x)):
            i_xte += e * dt
            if cfg.ki_xte > 0.0:
                i_xte = float(np.clip(i_xte, -0.5 * cfg.max_intercept / cfg.ki_xte, 0.5 * cfg.max_intercept / cfg.ki_xte))
        i_x = cfg.ki_xte * i_xte
        crab = -s * float(np.degrees(np.arctan2(v_lat, max(v, 0.5)))) if cfg.feedforward else 0.0
        d_psi = -float(np.clip(p_x + i_x + d_x, -cfg.max_intercept, cfg.max_intercept)) + crab
        psi_des = m.wrap360(bearing + d_psi)
        if not cfg.allow_dead_zone:
            psi_des = clamp_heading_to_sailable(psi_des, bearing)

        # --- İç döngü: pruva hatası -> dümen komutu ---
        eps = m.wrap180(psi_des - psi)
        p_h, d_h = cfg.kp_hdg * eps, -cfg.kd_hdg * g * r
        r_sail_now = sail_yaw_rate(side, s, v, phys)
        ff_h = holding_rudder(r_sail_now, v, phys) if cfg.feedforward else 0.0
        u_h = ff_h + p_h + cfg.ki_hdg * i_hdg + d_h
        sat_h = abs(u_h) > m.RUDDER_MAX_DEG
        if abs(eps) < I_ZONE_HDG and (not sat_h or np.sign(eps) != np.sign(u_h)):
            i_hdg += eps * dt
            if cfg.ki_hdg > 0.0:
                i_hdg = float(np.clip(i_hdg, -I_MAX_HDG / cfg.ki_hdg, I_MAX_HDG / cfg.ki_hdg))
        i_h = cfg.ki_hdg * i_hdg
        u_h = ff_h + p_h + i_h + d_h
        delta_cmd = float(np.clip(u_h, -m.RUDDER_MAX_DEG, m.RUDDER_MAX_DEG))

        # --- Yelken boşaltma: dümen rüzgar altına doydu ve tekne yine de rüzgardan uzaklaşamıyor
        #     (orsaya kaçma dümeni yeniyor). Yalnızca doyma yetmez: boşaltmak hızı, dolayısıyla dümen
        #     etkisini de düşürür; dümen dönüşü sağlayabiliyorsa yelken dolu kalır. ---
        if (cfg.depower and s != 0.0 and abs(u_h) > m.RUDDER_MAX_DEG and np.sign(u_h) == s
                and s * delta > 0.95 * m.RUDDER_MAX_DEG and s * r < cfg.depower_rate):
            depower = min(1.0, depower + dt / cfg.depower_time)
        else:
            depower = max(0.0, depower - dt / cfg.recover_time)

        # --- Kuvvetler ve momentler (kayıt) ---
        q_w = 0.5 * phys.rho_water * phys.rudder_area * v * v
        cl = float(np.clip(phys.rudder_cla * np.radians(delta), -phys.rudder_clmax, phys.rudder_clmax))
        rudder_force = q_w * cl
        r_rud = float(np.degrees(v / phys.hull_length * np.sin(np.radians(delta)))) if v > m.MIN_STEER_SPEED else 0.0
        r_sail = sail_yaw_rate(side, s, v, phys)
        for k, val in (("t", t), ("x", p[0]), ("y", p[1]), ("psi", psi), ("psi_des", psi_des), ("r", r), ("v", v),
                       ("v_lat", v_lat), ("cog", m.wrap360(float(np.degrees(np.arctan2(vel[0], vel[1]))))),
                       ("e", e), ("along", along), ("delta", delta), ("delta_cmd", delta_cmd), ("sail", signed_sail),
                       ("depower", depower), ("twa", twa), ("eta", eta), ("p_xte", p_x), ("i_xte", i_x), ("d_xte", d_x),
                       ("p_hdg", p_h), ("i_hdg", i_h), ("d_hdg", d_h), ("drive", drive), ("side", side),
                       ("hull_drag", phys.hull_drag * v), ("rudder_force", rudder_force),
                       ("rudder_moment", rudder_force * phys.rudder_arm), ("sail_moment", -s * side * phys.ce_offset),
                       ("r_rudder", r_rud), ("r_sail", r_sail),
                       ("leeway_angle", float(np.degrees(np.arctan2(v_lat, max(v, 1e-6)))) if v_lat > 0 else 0.0)):
            rec[k].append(float(val))
        rec["ff_hdg"].append(ff_h)
        rec["crab"].append(crab)

        if along >= length:
            status, reason = "reached", "B'nin hizasına ulaşıldı"
            break
        stopped_for = stopped_for + dt if v < STOP_SPEED else 0.0
        if stopped_for >= 5.0:
            status, reason = "stopped", "tekne durdu (yelken rüzgarı tutmuyor)"
            break
        if t >= t_max:
            break

        # --- Bir adım ilerlet ---
        delta += float(np.clip(delta_cmd - delta, -cfg.rudder_rate * dt, cfg.rudder_rate * dt))
        signed_sail, sail, eta, ratio, twa, s = sail_state(psi, depower, phys)
        drive, side = sail_forces(wind_ms, ratio, eta, sail, phys)
        v_star = wind_ms * ratio * eta * m.rudder_penalty(delta, phys) * phys.speed_scale
        v = v_star + (v - v_star) * np.exp(-dt / tau) if tau > 0.0 else v_star
        if v > m.MIN_STEER_SPEED:
            r_ss = float(np.degrees(v / phys.hull_length * np.sin(np.radians(delta)))) + sail_yaw_rate(side, s, v, phys)
        else:
            r_ss = 0.0
        t_yaw = cfg.yaw_t_nd * phys.hull_length / max(v, 1.0)
        r = r_ss + (r - r_ss) * np.exp(-dt / t_yaw)
        psi = m.wrap360(psi + r * dt)
        v_lat = leeway_speed(side, cfg, phys)
        lat_dir = np.array([np.cos(np.radians(psi)), -np.sin(np.radians(psi))])
        vel = v * np.array([np.sin(np.radians(psi)), np.cos(np.radians(psi))]) + s * v_lat * lat_dir
        p = p + vel * dt
        t += dt

    data = {k: np.array(vals) for k, vals in rec.items()}
    return LineResult(a=tuple(a), b=(float(b[0]), float(b[1])), bearing=bearing, length=length,
                      status=status, reason=reason, data=data, metrics=line_metrics(data, cfg))


def line_metrics(d, cfg):
    """Rota tutma başarımı: en büyük / RMS sapma, aşım, oturma süresi, dümen kullanımı, varış süresi."""
    e, t = d["e"], d["t"]
    e0 = float(e[0])
    crossed = np.nonzero(np.sign(e) != np.sign(e0))[0] if abs(e0) > SETTLE_BAND else np.array([], dtype=int)
    overshoot = float(np.max(np.abs(e[crossed[0]:]))) if crossed.size else 0.0
    outside = np.nonzero(np.abs(e) > SETTLE_BAND)[0]
    settle = 0.0 if outside.size == 0 else (float(t[outside[-1] + 1]) if outside[-1] + 1 < len(t) else None)
    half = len(e) // 2
    return dict(
        time=float(t[-1]),
        max_abs_e=float(np.max(np.abs(e))),
        overshoot=overshoot,
        settle_time=settle,
        rms_e_second_half=float(np.sqrt(np.mean(e[half:] ** 2))) if len(e) else 0.0,
        final_e=float(e[-1]),
        rudder_travel=float(np.sum(np.abs(np.diff(d["delta"])))),
        max_abs_rudder=float(np.max(np.abs(d["delta"]))),
        mean_speed=float(np.mean(d["v"])),
        max_leeway=float(np.max(d["leeway_angle"][d["v"] > 1.0], initial=0.0)),   # tekne ilerlerken
        depower_time=float(np.sum(d["depower"] > 0.01) * cfg.dt),
    )
