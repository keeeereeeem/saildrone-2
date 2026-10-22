"""
A → B düz rota takibi: kaskad PID ile çizgi tutma (Streamlit'ten bağımsız, test edilebilir).

Tekne A'dan B'ye çizilen doğrunun üzerinde kalmaya çalışır. Çizgiden sapma (rota sapması, XTE) iki katlı
(kaskad) bir PID kontrolcüyle düzeltilir:

    dış döngü:  e (m)            --PID-->  Δψ (°)   hedef pruva ψ* = β + Δψ + ψ_İB   (ölü açıya girmeyecek şekilde sınırlı)
    iç döngü:   ε = ψ* − ψ (°)   --PID-->  δr,komut = δ_İB + PID (°) --dümen servosu (hız sınırlı)--> δr

* İleri besleme (İB): ψ_İB = −s·λ (yan kayma kadar rüzgara pay), δ_İB = orsaya kaçmayı dengeleyen dümen.
  İkisi de kontrolcünün o anki yelken yan kuvvetinden hesaplanır; PID yalnızca kalan hatayı düzeltir.
* Hıza göre kazanç ölçekleme: g = sat(v_ref/v, 0.2, 6); iç döngü Kp, Kd × g, dış döngü Kd × max(g, 1). Dümen
  etkisi hızla orantılı olduğundan ((v/L)·δ) pruva döngüsünün dinamiği ancak böyle hızdan bağımsız kalır; hızlı
  teknede ölçeklenmemiş türev kazancı dümeni titretir.
* Koşullu entegrasyon (anti-windup): I terimleri yalnızca hata küçükken (|e| < 10 m, |ε| < 10°) birikir.
* Yelken auto-trim'dedir (devrilme sınırı dahil); dümen rüzgar altına doyduğu hâlde tekne rüzgardan
  uzaklaşamıyorsa (orsaya kaçma dümeni yeniyorsa) bom en iyi açıdan yapraklama sınırına doğru boşaltılır.

Dinamik diğer sekmelerle aynıdır (saildrone_model.step): görünür rüzgarda yelken kuvvetleri, devrilme sınırı,
gövde + yan kayma + dümen direnciyle ileri hız, Nomoto dönüş ataleti, dümen servosu hız sınırı, yan kayma.

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
G_MIN, G_MAX = 0.2, 6.0     # hıza göre kazanç çarpanı g = v_ref / v sınırları
EASE_ALPHA = 2.0            # boşaltmada bom, hücum açısı bu değere inene kadar açılır (yapraklama sınırı, °)


@dataclass(frozen=True)
class LineConfig:
    # Varsayılan kazançlar: 50 kg referans tekne için senaryo ızgarasında (iki kontra, orsa–pupa, 3–15 m/s rüzgar,
    # ±30 m sapma, ±25° pruva hatası) aşım + RMS sapma + dümen kullanımı + oturma süresi toplamını en aza indiren değerler.
    # Dış döngü: rota sapması e (m) -> pruva düzeltmesi Δψ (°)
    kp_xte: float = 5.0             # °/m
    ki_xte: float = 0.02            # °/(m·s)
    kd_xte: float = 8.0             # °/(m/s)  (sapma hızı ė = hızın çizgi normali bileşeni, GPS'ten)
    max_intercept: float = 30.0     # |Δψ| sınırı (°): çizgiye en fazla bu açıyla yaklaşılır
    # İç döngü: pruva hatası ε (°) -> dümen komutu δr (°)
    kp_hdg: float = 1.0             # °/°
    ki_hdg: float = 0.05            # 1/s
    kd_hdg: float = 1.5             # s (dönüş hızı geri beslemesi: −Kd·r)
    # Hıza göre kazanç ölçekleme (gain scheduling): dümen etkisi hızla orantılı olduğundan iç döngü kazançları
    # g = sat(v_ref / v, 0.2, 6) ile çarpılır; böylece pruva döngüsünün dinamiği hızdan bağımsız kalır.
    gain_schedule: bool = True
    schedule_outer: bool = True     # dış döngünün D terimi de ölçeklenir (ė = v·sin(ψ − β) de hızla orantılı)
    v_ref: float = 5.0              # kazançların ayarlandığı referans hız (m/s)
    # İleri besleme (feedforward): kontrolcü yelken kuvvetinden orsaya kaçmayı ve yan kayma açısını hesaplar;
    # dümene orsaya kaçmayı dengeleyen açı (Rotayı Tut ile aynı), hedef pruvaya yan kayma kadar rüzgar payı eklenir.
    # PID yalnızca kalan hatayı düzeltir (yavaş seyirde P teriminin kalıcı pruva hatası tekneyi ölü açıya sokmaz).
    feedforward: bool = True
    # Yan kayma (fiziği saildrone_model'dedir; kapatılırsa konuma yansıtılmaz)
    leeway: bool = True
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


def eased_sail(psi, v, wind_ms, depower, phys=m.PHYS):
    """
    Auto-trim + boşaltma: (|δs| bom, en iyi bom). depower = 0: itkiyi en büyük yapan bom; 1: bom, hücum açısı
    EASE_ALPHA'ya inene kadar açılır (yelken yapraklama sınırında, yan kuvvet ve orsaya kaçma en aza iner).
    """
    opt = m.best_trim(psi, v, wind_ms, phys)
    if depower <= 0.0 or m.true_wind_angle(psi) <= m.NO_GO_DEG:
        return opt, opt
    awa = float(m.apparent_wind(psi, v, wind_ms)[1])
    ease_max = float(np.clip(awa - EASE_ALPHA, opt, 90.0))
    return opt + depower * (ease_max - opt), opt


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
    """A'dan B'ye düz rota takibi simülasyonu (kaskad PID + saildrone_model.step dinamiği)."""
    length, bearing, u, n = line_frame(a, b)
    dt = cfg.dt
    a = np.asarray(a, dtype=float)
    p0 = a + cfg.start_offset * n
    psi0 = m.wrap360(bearing + cfg.start_heading_err)
    if cfg.allow_dead_zone and m.true_wind_angle(psi0) <= m.NO_GO_DEG:
        psi0 = clamp_heading_to_sailable(psi0, bearing)   # ölü açıdaki rota: tekne en yakın seyir pruvasından başlar
    st = m.initial_state(p0[0], p0[1], psi0, wind_ms, None, phys)   # başlangıçta kararlı hız (diğer sekmelerle aynı)
    depower = 0.0
    i_xte, i_hdg = 0.0, 0.0
    v_line = m.polar_speed(bearing, m.PROFILE, wind_ms, phys)
    t_max = cfg.max_time if cfg.max_time is not None else 3.0 * length / max(v_line, 0.5) + 120.0

    keys = ("t x y psi psi_des r v v_lat cog e along delta delta_cmd sail sail_opt depower twa awa heel_factor "
            "p_xte i_xte d_xte p_hdg i_hdg d_hdg drive side hull_drag induced_drag rudder_drag rudder_force "
            "rudder_moment sail_moment r_rudder r_sail leeway_angle ff_hdg crab").split()
    rec = {k: [] for k in keys}
    t, stopped_for, status, reason = 0.0, 0.0, "timeout", "süre sınırı doldu"

    while True:
        psi, v, r, delta = st.psi, st.v, st.r, st.delta
        p = np.array([st.x, st.y])
        rel = p - a
        e = float(rel @ n)
        along = float(rel @ u)
        s = m.leeward_sign(psi)
        twa = m.true_wind_angle(psi)
        # Yelken (auto-trim + boşaltma) ve kuvvetleri – adım fonksiyonu aynı bomla aynı kuvvetleri hesaplar
        sail, sail_opt = eased_sail(psi, v, wind_ms, depower, phys)
        drive, side, awa, _va, _al, _cl, _cd, heel = (float(z) for z in m.sail_force(psi, v, wind_ms, sail, phys))
        v_lat = float(m.leeway_speed(side, v, phys)) if (s != 0.0 and cfg.leeway) else 0.0
        lat_dir = np.array([np.cos(np.radians(psi)), -np.sin(np.radians(psi))])   # sancak yönü
        vel = v * np.array([np.sin(np.radians(psi)), np.cos(np.radians(psi))]) + s * v_lat * lat_dir
        e_dot = float(vel @ n)

        # --- Dış döngü: rota sapması -> pruva düzeltmesi ---
        g = float(np.clip(cfg.v_ref / max(v, 0.5), G_MIN, G_MAX)) if cfg.gain_schedule else 1.0
        p_x, d_x = cfg.kp_xte * e, cfg.kd_xte * (max(g, 1.0) if cfg.schedule_outer else 1.0) * e_dot
        u_x = p_x + cfg.ki_xte * i_xte + d_x
        sat_x = abs(u_x) > cfg.max_intercept
        # koşullu entegrasyon (anti-windup): büyük düzeltmeyi P ve D yapar, I yalnızca kalan küçük hatayı toplar
        if abs(e) < I_ZONE_XTE and (not sat_x or np.sign(e) != np.sign(u_x)):
            i_xte += e * dt
            if cfg.ki_xte > 0.0:
                i_xte = float(np.clip(i_xte, -0.5 * cfg.max_intercept / cfg.ki_xte, 0.5 * cfg.max_intercept / cfg.ki_xte))
        i_x = cfg.ki_xte * i_xte
        lee_deg = float(np.degrees(np.arctan2(v_lat, max(v, 0.5))))
        crab = -s * lee_deg if cfg.feedforward else 0.0
        d_psi = -float(np.clip(p_x + i_x + d_x, -cfg.max_intercept, cfg.max_intercept)) + crab
        psi_des = m.wrap360(bearing + d_psi)
        if not cfg.allow_dead_zone:
            psi_des = clamp_heading_to_sailable(psi_des, bearing)

        # --- İç döngü: pruva hatası -> dümen komutu ---
        eps = m.wrap180(psi_des - psi)
        p_h, d_h = cfg.kp_hdg * g * eps, -cfg.kd_hdg * g * r
        ff_h = m.holding_rudder(side, s, v, phys) if cfg.feedforward else 0.0
        u_h = ff_h + p_h + cfg.ki_hdg * i_hdg + d_h
        d_max = phys.rudder_stall_deg          # stall'dan büyük dümen dönüşü artırmaz, yalnızca frenler
        sat_h = abs(u_h) > d_max
        if abs(eps) < I_ZONE_HDG and (not sat_h or np.sign(eps) != np.sign(u_h)):
            i_hdg += eps * dt
            if cfg.ki_hdg > 0.0:
                i_hdg = float(np.clip(i_hdg, -I_MAX_HDG / cfg.ki_hdg, I_MAX_HDG / cfg.ki_hdg))
        i_h = cfg.ki_hdg * i_hdg
        u_h = ff_h + p_h + i_h + d_h
        delta_cmd = float(np.clip(u_h, -d_max, d_max))

        # --- Yelken boşaltma: dümen rüzgar altına doydu ve tekne yine de rüzgardan uzaklaşamıyor
        #     (orsaya kaçma dümeni yeniyor). Yalnızca doyma yetmez: boşaltmak hızı, dolayısıyla dümen
        #     etkisini de düşürür; dümen dönüşü sağlayabiliyorsa yelken dolu kalır. ---
        if (cfg.depower and s != 0.0 and abs(u_h) > d_max and np.sign(u_h) == s
                and s * delta > 0.95 * d_max and s * r < cfg.depower_rate):
            depower = min(1.0, depower + dt / cfg.depower_time)
        else:
            depower = max(0.0, depower - dt / cfg.recover_time)

        # --- Kuvvetler ve momentler (kayıt) ---
        rudder_force = 0.5 * phys.rho_water * phys.rudder_area * v * v * float(m.rudder_lift_coef(delta, phys))
        for k, val in (("t", t), ("x", p[0]), ("y", p[1]), ("psi", psi), ("psi_des", psi_des), ("r", r), ("v", v),
                       ("v_lat", v_lat), ("cog", m.wrap360(float(np.degrees(np.arctan2(vel[0], vel[1]))))),
                       ("e", e), ("along", along), ("delta", delta), ("delta_cmd", delta_cmd), ("sail", s * sail),
                       ("sail_opt", s * sail_opt), ("depower", depower), ("twa", twa), ("awa", awa),
                       ("heel_factor", heel), ("p_xte", p_x), ("i_xte", i_x), ("d_xte", d_x),
                       ("p_hdg", p_h), ("i_hdg", i_h), ("d_hdg", d_h), ("drive", drive), ("side", side),
                       ("hull_drag", float(m.hull_resistance(v, phys))),
                       ("induced_drag", float(m.induced_resistance(side, v, phys))),
                       ("rudder_drag", float(m.rudder_drag(v, delta, phys))), ("rudder_force", rudder_force),
                       ("rudder_moment", rudder_force * phys.rudder_arm), ("sail_moment", -s * side * phys.ce_offset),
                       ("r_rudder", m.rudder_yaw_rate(v, delta, phys)), ("r_sail", m.sail_yaw_rate(side, s, v, phys)),
                       ("leeway_angle", lee_deg if v_lat > 0.0 and v > 0.0 else 0.0),
                       ("ff_hdg", ff_h), ("crab", crab)):
            rec[k].append(float(val))

        if along >= length:
            status, reason = "reached", "B'nin hizasına ulaşıldı"
            break
        stopped_for = stopped_for + dt if v < STOP_SPEED else 0.0
        if stopped_for >= 5.0:
            status, reason = "stopped", "tekne durdu (yelken rüzgarı tutmuyor)"
            break
        if t >= t_max:
            break

        # --- Bir adım ilerlet (diğer sekmelerle aynı adım fonksiyonu) ---
        st = m.step(st, sail, delta_cmd, wind_ms, auto_trim=False, dt=dt, phys=phys, leeway=cfg.leeway)
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
        min_heel_factor=float(np.min(d["heel_factor"])),
    )
