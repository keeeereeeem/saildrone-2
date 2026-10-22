"""
Noktadan noktaya otonom navigasyon (güdüm + dümen kontrolü) – saildrone_model (VPP) üzerine kurulur.

Güdüm:   Her güdüm periyodunda iki kontra (iskele/sancak) için hedefe doğru hız bileşenini
         (VMG) maksimize eden pruva aranır. Hız, o rüzgarda VPP'den (kuvvet dengesi) okunur; gidiş yönü
         yan kayma kadar rüzgar altına kaydığından VMG = V·cos(ψ + s·λ − β). Tekne mevcut kontrada kalır;
         diğer kontranın VMG'si daha yüksekse ve
           (a) mevcut kontranın VMG'si pozitif değilse (hedef geride kaldı), ya da
               asgari bacak süresi dolmuşken
           (b) hedef diğer kontranın layline'ına gelmişse veya
           (c) tekne başlangıç-hedef hattından koridor yarı genişliğinden fazla
               uzaklaşmış ve hâlâ uzaklaşıyorsa
         kontra değiştirir.
Manevra: Yeni pruvaya giden en kısa dönüş ölü açıdan geçiyorsa (tack), mevcut hızla ölü
         açının aşılıp aşılamayacağı kestirilir (süzülme mesafesi); aşılamıyorsa tekne rüzgar altından döner
         ("kuyruk çevirme" / gybe-around). Kontra değişimi, manevra ileriye doğru simüle edilerek
         manevra SONRASINDA hedefin tutulabileceği garanti edilince yapılır.
Dümen:   İstenen dönüş hızı r* = k·e − r_yelken (orsaya kaçma ileri beslemesi, yelken yan kuvvetinden);
         kinematik modelin tersinden δ = asin(r*·L / v), dümen stall açısıyla (≈ 20°) sınırlı; dümen tutmayan
         hızda dümen ortalanır (pala yalnızca fren olur). Dümen servosu ve dönüş ataleti
         modelin adım fonksiyonundadır.
Yelken:  Auto-trim: bom her adımda görünür rüzgara göre itkiyi en büyük yapan açıya, rüzgar altı bordaya açılır.
Rota:    navigate_route() başlangıç → ara duraklar → hedef bacaklarını sırayla koşar; tekne her
         bacağa bir önceki bacağın son durumuyla (pruva, hız, dönüş hızı, dümen) girer.
"""
from dataclasses import dataclass, field, replace
from functools import lru_cache

import numpy as np

import saildrone_model as m


@dataclass
class NavConfig:
    arrival_radius: float = 10.0      # Varış yarıçapı (m)
    max_time: float = 7200.0          # Görev süre sınırı (s)
    dt: float = m.DT
    guidance_period: float = 1.0      # Güdüm güncelleme periyodu (s)
    predict_every: int = 5            # Rüzgar altından dönüşün ileri simülasyonu her bu kadar güdüm periyodunda bir
    no_go_margin: float = 5.0         # Ölü açı sınırına güvenlik payı (°)
    corridor: float = None            # Koridor yarı genişliği (m); None -> 0.2·D, [50, 1000]
    layline_tol: float = 1.0          # Layline açı toleransı (°)
    min_leg_time: float = 10.0        # İki kontra değişimi arası asgari süre (s)
    heading_gain: float = 0.5         # Pruva hatası -> dönüş hızı kazancı (1/s)
    heading_lead: float = 0.6         # Pruva öngörüsü (s): ε − τ·r (dümen servosu ve dönüş ataleti gecikmesi için)
    los_gain: float = 3.0             # Son yaklaşmada kerteriz dönüş hızı ileri beslemesi N (oransal seyrüsefer)
    homing_factor: float = 5.0        # Hedefe bu kadar dönüş yarıçapı yaklaşınca (yön ölü açıda değilse) doğrudan hedefe dön
    reach_tolerance: float = 0.05     # Doğrudan rota en iyi rotanın (polar zarfı) en çok bu oranda yavaşındaysa düz gidilir
    max_turn_rate: float = 30.0       # İstenen azami dönüş hızı (°/s)
    tack_safety: float = 1.15         # Tack fizibilite güvenlik katsayısı (süzülme kestirimi zaten en kötü durum rüzgar frenini içerir)
    allow_tack: bool = True           # False: kontra yalnızca rüzgar altından değiştirilir
    stuck_time: float = 20.0          # Ölü açıda bu süre ~sıfır hızda kalırsa başarısız
    phys: m.Physics = m.PHYS          # Teknenin fiziksel parametreleri


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
    rudder: np.ndarray = field(repr=False)          # gerçek dümen açısı (°)
    psi_des: np.ndarray = field(repr=False)
    sail: np.ndarray = field(default=None, repr=False)        # işaretli bom açısı (°), + sancağa
    rudder_cmd: np.ndarray = field(default=None, repr=False)  # dümen komutu (°)
    r: np.ndarray = field(default=None, repr=False)           # dönüş hızı (°/s)
    side: np.ndarray = field(default=None, repr=False)        # yelken yan kuvveti (N)
    v_lat: np.ndarray = field(default=None, repr=False)       # yan kayma hızı (m/s)
    heel_factor: np.ndarray = field(default=None, repr=False) # devrilme sınırı çarpanı (1: tam yelken)


def bearing_deg(x0, y0, x1, y1):
    """(x0,y0)'dan (x1,y1)'e pusula kerterizi."""
    return m.wrap360(np.degrees(np.arctan2(x1 - x0, y1 - y0)))


@lru_cache(maxsize=64)
def _polar_hull(wind_ms, margin, phys):
    """
    Hız polarının (gidiş yönü COG = ψ + s·λ uzayında, iki kontra) destek fonksiyonu: her normal yönü n için
    H(n) = max_i V_i·cos(COG_i − n) ve bu maksimumu veren nokta. Dışbükey zarf bu fonksiyondan okunur.
    """
    twa_all, spd_all, _, lee_all = m.polar_table(wind_ms, phys)
    ok = twa_all > m.NO_GO_DEG + margin
    twa, spd, lee = twa_all[ok], spd_all[ok], lee_all[ok]
    hdg = np.concatenate([twa, 360.0 - twa])
    cog = np.concatenate([twa + lee, 360.0 - (twa + lee)])
    spd = np.concatenate([spd, spd])
    side = np.concatenate([np.ones_like(twa), -np.ones_like(twa)])
    nrm = np.radians(np.arange(0.0, 360.0, 0.25))
    proj = spd[None, :] * np.cos(np.radians(cog)[None, :] - nrm[:, None])
    return hdg, cog, spd, side, nrm, proj.max(axis=1), proj.argmax(axis=1)


def hull_course(wind_ms, bearing, margin, phys=m.PHYS):
    """
    Sabit rüzgarda kerteriz β yönünde en kısa süreli rota (poların dışbükey zarfı):
      ρ(β) = min_n H(n) / cos(n − β)  – zarfın β yönündeki yarıçapı, en iyi rotayla hedefe yaklaşma hızı.
    Kerterize doğrudan giden pruvanın hızı ρ'ya eşitse düz gitmek en hızlısıdır; değilse rota, β ışınının
    kestiği zarf kenarının iki ucundaki pruvalarla iki bacakta gidilir (aynı ya da zıt kontra).
    Döner: dict(rho, direct=(pruva, hız) ya da None, legs=[(pruva, hız, kontra), (pruva, hız, kontra)]).
    """
    hdg, cog, spd, side, nrm, H, arg = _polar_hull(float(wind_ms), float(margin), phys)
    c = np.cos(nrm - np.radians(bearing))
    rho_n = np.where(c > 1e-3, H / np.maximum(c, 1e-3), np.inf)
    j = int(np.argmin(rho_n))
    rho = float(rho_n[j])
    off = (cog - bearing + 180.0) % 360.0 - 180.0
    k = int(np.argmin(np.abs(off)))
    direct = (m.wrap360(hdg[k]), float(spd[k])) if abs(off[k]) < 1.0 else None
    cand = sorted({int(arg[(j + d) % len(nrm)]) for d in (-2, -1, 0, 1, 2)}, key=lambda q: off[q])
    legs = [(m.wrap360(hdg[q]), float(spd[q]), int(side[q])) for q in (cand[0], cand[-1])]
    return dict(rho=rho, direct=direct, legs=legs)


def best_heading_per_side(profile, wind_ms, bearing, margin, phys=m.PHYS, direct_tol=0.0, psi_now=None):
    """
    İki kontra için hedefe gidiş pruvası: {side: (pruva, VMG, hız)}. VMG gidiş yönüyle (yan kayma dahil) hesaplanır.
    side +1: psi ∈ (0,180] (iskele kontra), side -1: psi ∈ [180,360) (sancak kontra).
    Varsayılan: her kontrada anlık VMG'yi en büyük yapan pruva (orsa ve pupa seyrinde tack/gybe bacakları).
    direct_tol > 0 ise rota poların dışbükey zarfına göre düzeltilir (hull_course):
      * kerterize doğrudan giden pruvanın hızı ≥ (1 − direct_tol)·ρ(β) ise o kontrada doğrudan hedefe gidilir;
      * en iyi rotanın iki bacağı da aynı kontradaysa (kuvvetli rüzgarda tam apazdan hızlı geniş apaz gibi)
        o kontrada bacak uçlarından mevcut pruvaya yakın olanı tutulur – kerteriz öbür bacağın yönüne gelince
        doğrudan gidiş en iyi hale gelir. Anlık VMG pruvası ise kerteriz döndükçe tekneyi sarmal bir takip
        eğrisiyle hedefin rüzgar altına düşürür (düz rotadan yavaş, sonda gereksiz tack).
    """
    twa_all, spd_all, _, lee_all = m.polar_table(wind_ms, phys)
    ok = twa_all > m.NO_GO_DEG + margin
    twa, spd, lee = twa_all[ok], spd_all[ok], lee_all[ok]
    out = {}
    for side in (1, -1):
        hdg = twa if side == 1 else 360.0 - twa
        vmg = spd * np.cos(np.radians(hdg + side * lee - bearing))
        k = int(np.argmax(vmg))
        out[side] = (m.wrap360(hdg[k]), float(vmg[k]), float(spd[k]))
    if direct_tol > 0.0:
        hc = hull_course(wind_ms, bearing, margin, phys)
        legs = hc["legs"]
        if hc["direct"] is not None and hc["direct"][1] >= (1.0 - direct_tol) * hc["rho"]:
            h, v = hc["direct"]
            out[_side_of(h)] = (h, v, v)
        elif legs[0][2] == legs[1][2]:
            ref = bearing if psi_now is None else psi_now
            h, v, sd = min(legs, key=lambda L: abs(m.wrap180(L[0] - ref)))
            out[sd] = (h, hc["rho"], v)
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


def turn_radius(v, cfg):
    """En küçük dönüş yarıçapı (m): istenen dönüş hızı sınırı ya da tam dümenin kinematik dönüşü."""
    if v <= m.MIN_STEER_SPEED:
        return 0.0
    w = min(np.radians(cfg.max_turn_rate), v / cfg.phys.hull_length * np.sin(np.radians(cfg.phys.rudder_stall_deg)))
    return float(v / w)


def tack_capacity(v, phys=m.PHYS, wind_ms=0.0):
    """
    Ölü açıda dönülebilecek açı (°): Θ_maks = sin δ_stall · s_süzülme / L. Süzülmeyi gövde direnci, stall
    açısındaki dümenin sürüklemesi ve (rüzgar hızı verilirse) çırpınan yelkenin sürüklemesi yavaşlatır.
    """
    d = phys.rudder_stall_deg
    return float(np.degrees(np.sin(np.radians(d)) * m.coast_distance(v, phys, d, wind_ms) / phys.hull_length))


def tack_need(psi, cfg):
    """Tack için dönülmesi gereken açı (°, güvenlik katsayısı dahil)."""
    need = 2.0 * m.NO_GO_DEG + cfg.no_go_margin
    if m.true_wind_angle(psi) <= m.NO_GO_DEG:
        need = m.NO_GO_DEG + cfg.no_go_margin + m.true_wind_angle(psi)
    return cfg.tack_safety * need


def tack_feasible(psi, v, cfg, wind_ms=0.0):
    """
    Ölü açıda yelken itki üretmez; tekne v hızından dümen eşiğine kadar s = ∫ m·v / R(v) dv yol süzülür.
    Tam dümende dönülebilecek açı ≈ sin(δ_maks)·s / L. Gereken açı ≈ ölü açı genişliği + pay.
    """
    return tack_capacity(v, cfg.phys, wind_ms) >= tack_need(psi, cfg)


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
        if cfg.allow_tack and tack_feasible(s.psi, s.v, cfg, wind_ms):
            p = _predict_maneuver(s, psi_des, d, profile, wind_ms, cfg, horizon=60.0)
            if abs(m.wrap180(psi_des - p.psi)) < 5.0 and m.true_wind_angle(p.psi) > m.NO_GO_DEG:
                return d, True
        return -d, False            # tack mümkün değil -> rüzgar altından dön
    return d, False


def rudder_terms(s, err, cfg, los=0.0):
    """
    Dümen yasasının ara değerleri: (r_fb, r_ff, r*, δ komutu). r_ff orsaya kaçmayı (yelken momenti) dengeler;
    son yaklaşmada kerteriz dönüş hızı N·dβ/dt de eklenir (los).
    """
    r_fb = float(np.clip(cfg.heading_gain * (err - cfg.heading_lead * s.r), -cfg.max_turn_rate, cfg.max_turn_rate))
    r_ff = -m.sail_yaw_rate(s.side, m.leeward_sign(s.psi), s.v, cfg.phys) + cfg.los_gain * los
    r_des = r_fb + r_ff
    if s.v <= m.MIN_STEER_SPEED:
        # dümen tutmayan hızda dümen ortalanır: pala yalnızca fren olur, yelken önce tekneyi hızlandırmalı
        # (ölü açıda itki yoktur; pala, tekne süzülürse dönüşü başlatacak yöne tutulur)
        in_irons = m.true_wind_angle(s.psi) <= m.NO_GO_DEG
        return r_fb, r_ff, r_des, float(np.sign(err) * cfg.phys.rudder_stall_deg) if in_irons else 0.0
    lim = np.sin(np.radians(cfg.phys.rudder_stall_deg))
    rud = float(np.degrees(np.arcsin(np.clip(np.radians(r_des) * cfg.phys.hull_length / s.v, -lim, lim))))
    return r_fb, r_ff, r_des, rud


def _rudder_cmd(s, err, cfg, los=0.0):
    return rudder_terms(s, err, cfg, los)[3]


def _signed_error(psi, psi_des, turn_dir):
    err = m.wrap180(psi_des - psi)
    if turn_dir != 0 and np.sign(err) != turn_dir and abs(err) > 1.0:
        err += 360.0 * turn_dir
    return err


def _predict_maneuver(s, psi_des, turn_dir, profile, wind_ms, cfg, horizon=90.0):
    """Manevrayı (psi_des'e dönüş) aynı dümen yasasıyla ileri simüle eder; son durumu döndürür."""
    p = replace(s)
    for _ in range(int(horizon / cfg.dt)):
        err = _signed_error(p.psi, psi_des, turn_dir)
        if abs(err) < 5.0:
            break
        p = m.step(p, 0.0, _rudder_cmd(p, err, cfg), wind_ms, True, cfg.dt, cfg.phys)
    return p


def leg_geometry(start, target, cfg):
    """Bacak uzunluğu D, koridor yarı genişliği W ve rota hattı birim normali n."""
    sx, sy = start
    tx, ty = target
    d0 = float(np.hypot(tx - sx, ty - sy))
    corridor = cfg.corridor if cfg.corridor is not None else float(np.clip(0.2 * d0, 50.0, 1000.0))
    normal = ((ty - sy) / d0, -(tx - sx) / d0) if d0 > 0 else (1.0, 0.0)
    return d0, corridor, normal


def guidance_decision(s, side, leg_done, start, target, profile, wind_ms, cfg, predict_now=True):
    """
    Bir güdüm periyodunun kararı: kerteriz, iki kontranın VMG-optimum pruvaları, kontra değişim
    koşulları ve atanan pruva ψ*. navigate() ile hesap tablosu (guidance_breakdown) aynı fonksiyonu kullanır.
    s: anlık durum, side: mevcut kontra (+1 iskele / -1 sancak), leg_done: asgari bacak süresi doldu mu,
    predict_now: rüzgar altından dönüşün ileri simülasyonu bu periyotta yapılsın mı (her predict_every periyotta bir).
    """
    sx, sy = start
    tx, ty = target
    _, corridor, (nx, ny) = leg_geometry(start, target, cfg)
    b = bearing_deg(s.x, s.y, tx, ty)
    opts = best_heading_per_side(profile, wind_ms, b, cfg.no_go_margin, cfg.phys, cfg.reach_tolerance, s.psi)
    other = -side
    cur_vmg, oth_vmg = opts[side][1], opts[other][1]
    h = np.radians(opts[side][0])
    xtrack = (s.x - sx) * nx + (s.y - sy) * ny
    moving_out = xtrack * (np.sin(h) * nx + np.cos(h) * ny) > 0.0
    behind = cur_vmg <= 0.0
    layline = on_layline(opts[other][0], other, b, cfg.layline_tol)
    out_of_corridor = abs(xtrack) > corridor and moving_out
    want = oth_vmg > cur_vmg and (behind or (leg_done and layline) or (leg_done and out_of_corridor))
    turn_dir, is_tack, after_ok = 0, False, None
    if want:
        h_new = opts[other][0]
        turn_dir, is_tack = _turn_direction(s, h_new, profile, wind_ms, cfg)
        upwind = m.true_wind_angle(s.psi) < 90.0 and m.true_wind_angle(h_new) < 90.0
        if upwind and not is_tack and not predict_now:
            want = False                  # ileri simülasyon bir sonraki tahmin periyodunda
        elif upwind and not is_tack:
            # Rüzgar altından dönüş: manevra sonrası konumdan hedef tutulabiliyor mu?
            p = _predict_maneuver(s, h_new, turn_dir, profile, wind_ms, cfg)
            b2 = bearing_deg(p.x, p.y, tx, ty)
            o2 = best_heading_per_side(profile, wind_ms, b2, cfg.no_go_margin, cfg.phys, cfg.reach_tolerance, p.psi)
            after_ok = bool(np.hypot(tx - p.x, ty - p.y) <= cfg.arrival_radius
                            or on_layline(o2[other][0], other, b2, cfg.layline_tol))
            want = after_ok
    new_side = other if want else side
    psi_des = opts[new_side][0]
    term = terminal_guidance(s, target, wind_ms, cfg)
    if term["active"]:
        psi_des = term["psi_des"]
        new_side = _side_of(psi_des) if 0.0 < m.true_wind_angle(psi_des) < 180.0 else new_side
    return dict(bearing=b, opts=opts, side=side, other=other, cur_vmg=cur_vmg, oth_vmg=oth_vmg,
                xtrack=float(xtrack), corridor=corridor, moving_out=bool(moving_out), behind=bool(behind),
                layline=bool(layline), out_of_corridor=bool(out_of_corridor), leg_done=bool(leg_done),
                switch=bool(want), turn_dir=turn_dir, is_tack=bool(is_tack), after_ok=after_ok,
                new_side=new_side, psi_des=psi_des, homing=term["homing"], shoot=term["shoot"],
                inside_turn=term["inside"], los=term["los"], dist=term["dist"])


def los_rate(s, target):
    """Görüş hattı (kerteriz) dönüş hızı dβ/dt (°/s): yere göre hız vektörüyle (yan kayma dahil)."""
    dx, dy = target[0] - s.x, target[1] - s.y
    d2 = dx * dx + dy * dy
    if d2 < 1e-6:
        return 0.0
    p = np.radians(s.psi)
    lee = m.leeward_sign(s.psi) * s.v_lat
    vx = s.v * np.sin(p) + lee * np.cos(p)
    vy = s.v * np.cos(p) - lee * np.sin(p)
    return float(np.degrees((dx * vy - dy * vx) / d2))


def terminal_guidance(s, target, wind_ms, cfg):
    """
    Son yaklaşma. Hedef birkaç dönüş yarıçapı içindeyse ve yönü yelkenle gidilebilirse doğrudan hedefe dönülür
    (homing): pruva yan kayma kadar rüzgar üstüne tutulur (gidiş yönü COG = ψ + s·λ hedefe baksın) ve kerterizin
    dönüş hızı dβ/dt dümene ileri beslenir (oransal seyrüsefer) – hızlı teknede kerteriz hızla döndüğünden yalnız
    pruva hatasıyla tekne hedefin yanından geçer. Hedef, ona doğru dönülecek en küçük dönüş çemberinin içindeyse
    tekne dönerek hedefi tutamaz: önce düz gidilir, hedef çemberden çıkınca dönülür. Hedef ölü açıdaysa ama tekne
    stall dümeninde hedefe kadar süzülebiliyorsa (çırpınan yelkenin rüzgar freni dahil) doğrudan hedefe süzülür.
    Güdüm periyodu beklenmeden her adımda yenilenir.
    """
    tx, ty = target
    b = bearing_deg(s.x, s.y, tx, ty)
    dist = float(np.hypot(tx - s.x, ty - s.y))
    lam = float(np.clip(np.degrees(np.arctan2(s.v_lat, max(s.v, 0.5))), 0.0, 15.0))
    aim = m.wrap360(b - _side_of(b) * lam) if m.true_wind_angle(b) < 175.0 else b
    r_turn = turn_radius(s.v, cfg)
    homing = bool(dist < cfg.homing_factor * max(r_turn, cfg.arrival_radius)
                  and m.true_wind_angle(aim) > m.NO_GO_DEG + cfg.no_go_margin)
    shoot = bool(not homing and abs(m.wrap180(b - s.psi)) < 60.0
                 and dist < m.coast_distance(s.v, cfg.phys, cfg.phys.rudder_stall_deg, wind_ms) / cfg.tack_safety)
    inside = False
    if homing and r_turn > 0.0 and m.true_wind_angle(s.psi) > m.NO_GO_DEG + cfg.no_go_margin:
        d = 1.0 if m.wrap180(aim - s.psi) > 0 else -1.0
        cx = s.x + d * r_turn * np.cos(np.radians(s.psi))
        cy = s.y - d * r_turn * np.sin(np.radians(s.psi))
        inside = bool(np.hypot(tx - cx, ty - cy) < r_turn and abs(m.wrap180(aim - s.psi)) > 10.0)
    active = homing or shoot
    psi_des = s.psi if inside else (aim if homing else b)
    los = los_rate(s, target) if active and not inside else 0.0
    return dict(active=active, homing=homing, shoot=shoot, inside=inside, psi_des=psi_des, los=los,
                dist=dist, bearing=b)


def navigate(start, target, profile, wind_ms, initial_heading=None, cfg=None, initial_speed=None,
             best_side=False, initial_state=None):
    """
    start, target: (x, y) metre. profile: 'cloth' (tek yelken modeli).
    initial_heading: None ise tekne hedefe en iyi VMG pruvasıyla başlar.
    initial_speed: None ise tekne başlangıç pruvasındaki kararlı hızla başlar.
    best_side: True ise başlangıç pruvası verilse de en iyi VMG kontrası seçilir (ara durak dönüşü).
    initial_state: önceki bacaktan devreden durum (pruva, hız, dönüş hızı, dümen); konum start'tır.
    """
    cfg = cfg or NavConfig()
    if profile not in m.PROFILE_KEYS:
        profile = m.profile_key(profile)
    sx, sy = start
    tx, ty = target
    d0 = leg_geometry(start, target, cfg)[0]

    opts = best_heading_per_side(profile, wind_ms, bearing_deg(sx, sy, tx, ty), cfg.no_go_margin, cfg.phys, cfg.reach_tolerance)
    if initial_state is not None:
        psi0 = m.wrap360(float(initial_state.psi))
        side = max(opts, key=lambda sd: opts[sd][1]) if best_side else _side_of(psi0)
        s = replace(initial_state, x=float(sx), y=float(sy))
    else:
        if initial_heading is None:
            side = max(opts, key=lambda sd: opts[sd][1])
            psi0 = opts[side][0]
        else:
            psi0 = m.wrap360(float(initial_heading))
            side = max(opts, key=lambda sd: opts[sd][1]) if best_side else _side_of(psi0)
        s = m.initial_state(sx, sy, psi0, wind_ms, None, cfg.phys, initial_speed)
        s = m.step(s, 0.0, 0.0, wind_ms, True, 0.0, cfg.phys)       # dt = 0: kuvvet kaydı (bom, yan kuvvet)

    psi_des = psi0
    rec = dict(T=[0.0], X=[s.x], Y=[s.y], P=[s.psi], V=[s.v], R=[s.delta], RC=[s.delta], D=[psi0],
               S=[s.sail], YR=[s.r], SD=[s.side], W=[s.v_lat], F=[s.heel_factor])

    def log(t, cmd):
        for k, val in (("T", t), ("X", s.x), ("Y", s.y), ("P", s.psi), ("V", s.v), ("R", s.delta), ("RC", cmd),
                       ("D", psi_des), ("S", s.sail), ("YR", s.r), ("SD", s.side), ("W", s.v_lat),
                       ("F", s.heel_factor)):
            rec[k].append(val)

    path = 0.0
    n_tacks = n_gybes = 0
    turn_dir, turn_for = 0, psi0      # taahhüt edilmiş dönüş yönü ve ait olduğu hedef pruva
    t_switch = -1e9
    terminal, los = False, 0.0
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
            g = guidance_decision(s, side, (t - t_switch) >= cfg.min_leg_time, start, target,
                                  profile, wind_ms, cfg, (k // g_every) % cfg.predict_every == 0)
            if g["switch"]:
                t_switch = t
            side = g["new_side"]          # kontra değişimi ya da son yaklaşmada hedefin bulunduğu kontra
            psi_des = g["psi_des"]
            terminal, los = g["homing"] or g["shoot"], g["los"]
        elif terminal:
            tg = terminal_guidance(s, target, wind_ms, cfg)    # son yaklaşma her adımda yenilenir
            if tg["active"]:
                psi_des, los = tg["psi_des"], tg["los"]

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
        cmd = _rudder_cmd(s, err, cfg, los)
        prev_psi, px, py = s.psi, s.x, s.y
        s = m.step(s, 0.0, cmd, wind_ms, auto_trim=True, dt=cfg.dt, phys=cfg.phys)
        path += float(np.hypot(s.x - px, s.y - py))

        # Kontra değişimi sayımı (ölü açı ve tam pupa ±5° dışında taraf değiştiyse)
        if m.NO_GO_DEG < m.true_wind_angle(s.psi) < 175.0:
            cur_side = _side_of(s.psi)
            if prev_side and cur_side != prev_side:
                if m.true_wind_angle(prev_psi) <= 90.0 or m.true_wind_angle(s.psi) <= 90.0:
                    n_tacks += 1
                else:
                    n_gybes += 1
            prev_side = cur_side

        log(t + cfg.dt, cmd)

        # Ölü açıda takılma tespiti
        if m.true_wind_angle(s.psi) <= m.NO_GO_DEG and s.v < 0.05:
            stuck += cfg.dt
            if stuck >= cfg.stuck_time:
                reason = "Ölü açıda takıldı (in irons) – hız sıfır, dümen etkisiz"
                break
        else:
            stuck = 0.0

    A = {k: np.array(v) for k, v in rec.items()}
    res = NavResult(reached, reason, float(A["T"][-1]), path, d0, float(np.hypot(tx - s.x, ty - s.y)),
                    n_tacks, n_gybes, A["T"], A["X"], A["Y"], A["P"], A["V"], A["R"], A["D"],
                    sail=A["S"], rudder_cmd=A["RC"], r=A["YR"], side=A["SD"], v_lat=A["W"], heel_factor=A["F"])
    res.final_state = s
    return res


@dataclass
class RouteResult(NavResult):
    """Çok bacaklı rota sonucu: NavResult alanları + bacak indeksi, bacak özetleri, rüzgar ve tekne."""
    leg: np.ndarray = field(default=None, repr=False)    # örneğin ait olduğu bacak (0'dan)
    points: list = field(default_factory=list, repr=False)
    legs: list = field(default_factory=list, repr=False)
    wind: float = 8.0
    phys: m.Physics = m.PHYS


ROUTE_ARRAYS = ("t", "x", "y", "psi", "v", "rudder", "psi_des", "sail", "rudder_cmd", "r", "side", "v_lat",
                "heel_factor")


def navigate_route(points, wind_ms, initial_heading=None, cfg=None, profile=m.PROFILE):
    """
    points: [başlangıç, ara durak 1, ..., hedef] – (x, y) metre. Her bacak navigate() ile koşulur;
    süre sınırı (cfg.max_time) tüm rota içindir. Bir bacak başarısız olursa rota orada biter.
    """
    cfg = cfg or NavConfig()
    pts = [(float(x), float(y)) for x, y in points]
    acc = {k: [] for k in ROUTE_ARRAYS}
    G = []
    legs = []
    t0 = path = 0.0
    n_tacks = n_gybes = 0
    state = None
    reached, reason = True, "Hedefe ulaşıldı"
    for i in range(len(pts) - 1):
        leg_cfg = replace(cfg, max_time=max(cfg.max_time - t0, cfg.dt))
        r = navigate(pts[i], pts[i + 1], profile, wind_ms, initial_heading=initial_heading if i == 0 else None,
                     cfg=leg_cfg, best_side=i > 0, initial_state=state)
        k = 0 if i == 0 else 1                      # bacak başlangıç örneği önceki bacağın sonudur
        for name in ROUTE_ARRAYS:
            arr = getattr(r, name)
            acc[name] += list(arr[k:] + t0) if name == "t" else list(arr[k:])
        G += [i] * (len(r.t) - k)
        legs.append(dict(leg=i, start=pts[i], end=pts[i + 1], reached=r.reached, time=r.time,
                         path=r.path_length, tacks=r.n_tacks, gybes=r.n_gybes))
        t0 += r.time
        path += r.path_length
        n_tacks += r.n_tacks
        n_gybes += r.n_gybes
        state = r.final_state
        if not r.reached:
            reached = False
            reason = r.reason if len(pts) == 2 else f"{leg_name(i, len(pts))} bacağı: {r.reason}"
            break
    straight = float(sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])))
    A = {k: np.array(v) for k, v in acc.items()}
    final = float(np.hypot(pts[-1][0] - A["x"][-1], pts[-1][1] - A["y"][-1]))
    return RouteResult(reached, reason, t0, path, straight, final, n_tacks, n_gybes,
                       A["t"], A["x"], A["y"], A["psi"], A["v"], A["rudder"], A["psi_des"],
                       sail=A["sail"], rudder_cmd=A["rudder_cmd"], r=A["r"], side=A["side"], v_lat=A["v_lat"],
                       heel_factor=A["heel_factor"], leg=np.array(G), points=pts, legs=legs,
                       wind=float(wind_ms), phys=cfg.phys)


def leg_name(i, n_points):
    """Bacak adı: B→1, 1→2, ..., k→H (B: başlangıç, H: hedef)."""
    a = "B" if i == 0 else str(i)
    b = "H" if i == n_points - 2 else str(i + 1)
    return f"{a}→{b}"


def state_at(res, k):
    """Rota kaydının k. örneğindeki tam durum (dümen yasası ve adım fonksiyonu için)."""
    return m.BoatState(float(res.x[k]), float(res.y[k]), float(res.psi[k]), float(res.v[k]), float(res.r[k]),
                       float(res.rudder[k]), sail=float(res.sail[k]), side=float(res.side[k]),
                       v_lat=float(res.v_lat[k]), heel_factor=float(res.heel_factor[k]))


def guidance_breakdown(res, k, wind_ms, cfg=None, profile=m.PROFILE):
    """
    Hedefe navigasyon hesap tablosu: rotanın k. örneğinde (bacak içindeki son güdüm anına yuvarlanır)
    güdüm, yelken, dümen ve dinamik denklemlerinin ara değerleri. navigate() ile aynı fonksiyonlar
    kullanılır; sim_* alanları simülasyonun kaydettiği bir sonraki adımdır (çapraz kontrol).
    res: navigate_route() sonucu; cfg: rotayı hesaplayan NavConfig.
    """
    cfg = cfg or NavConfig()
    n = len(res.t)
    k = int(np.clip(k, 0, n - 1))
    leg = int(res.leg[k])
    first = int(np.argmax(res.leg == leg))
    b0 = first if leg == 0 else first - 1          # bacağın başlangıç durumu (önceki bacağın son örneği)
    start, target = res.points[leg], res.points[leg + 1]
    tx, ty = target
    g_every = max(1, int(round(cfg.guidance_period / cfg.dt)))
    if np.hypot(tx - res.x[k], ty - res.y[k]) > cfg.arrival_radius:      # varış anı yuvarlanmaz
        k = b0 + ((k - b0) // g_every) * g_every
    s = state_at(res, k)
    if k == b0 and leg > 0:
        s = replace(s, x=float(start[0]), y=float(start[1]))
    phys = cfg.phys
    t_loc = (k - b0) * cfg.dt

    # Güdümden önceki kontra ve son kontra değişimi (psi_des[i], i-1. adımın güdüm kararıdır).
    # Tam pupa (ψ* = 180°) iki kontraya da aittir; orada kontra değişmez, önceki kontra korunur.
    if leg > 0:
        o0 = best_heading_per_side(profile, wind_ms, bearing_deg(start[0], start[1], tx, ty),
                                   cfg.no_go_margin, phys, cfg.reach_tolerance)
        side = max(o0, key=lambda sd: o0[sd][1])
    else:
        side = _side_of(res.psi_des[0])
    t_switch = -1e9
    for i in range(b0 + 1, k + 1):
        if m.true_wind_angle(res.psi_des[i]) < 180.0 and _side_of(res.psi_des[i]) != side:
            side = -side
            t_switch = (i - 1 - b0) * cfg.dt
    leg_done = (t_loc - t_switch) >= cfg.min_leg_time

    dist = float(np.hypot(tx - s.x, ty - s.y))
    f = m.force_moment_breakdown(s.psi, abs(s.sail), s.delta, wind_ms, speed_ms=s.v, phys=phys)
    out = dict(k=k, t=float(res.t[k]), leg=leg, start=start, target=target, x=s.x, y=s.y, psi=s.psi, v=s.v,
               dist=dist, arrived=dist <= cfg.arrival_radius, t_leg=t_loc, t_switch=t_switch,
               d0=leg_geometry(start, target, cfg)[0], L=phys.hull_length, twa=m.true_wind_angle(s.psi),
               sail=float(s.sail), forces=f)
    nxt = k + 1 if k + 1 < n else None
    if out["arrived"]:
        return out

    g = guidance_decision(s, side, leg_done, start, target, profile, wind_ms, cfg,
                          ((k - b0) // g_every) % cfg.predict_every == 0)
    out.update(g)
    psi_des = g["psi_des"]
    out["twa_bearing"] = m.true_wind_angle(g["bearing"])
    out["hull"] = hull_course(wind_ms, g["bearing"], cfg.no_go_margin, phys)
    out["twa_des"] = m.true_wind_angle(psi_des)
    v_des, sail_des = m.polar(psi_des, wind_ms, phys)
    out["v_des"], out["sail_des"] = v_des, m.leeward_sign(psi_des) * sail_des

    # Tack fizibilitesi: Θ_maks = sin δ_maks · s_süzülme / L
    out["coast"] = m.coast_distance(s.v, phys, phys.rudder_stall_deg, wind_ms)
    out["tack_capacity"] = tack_capacity(s.v, phys, wind_ms)
    out["tack_need"] = tack_need(s.psi, cfg)
    out["tack_ok"] = bool(tack_feasible(s.psi, s.v, cfg, wind_ms))

    # Dümen: dönüş yönü (taahhüt edilen uzun yol dahil) simülasyonun dümen komutunun işaretinden okunur
    err = m.wrap180(psi_des - s.psi)
    sim_cmd = float(res.rudder_cmd[nxt]) if nxt is not None else None
    long_way = sim_cmd is not None and abs(err) > 10.0 and sim_cmd * err < 0.0
    if long_way:
        err = _signed_error(s.psi, psi_des, int(np.sign(sim_cmd)))
    r_fb, r_ff, r_des, rudder = rudder_terms(s, err, cfg, g["los"])
    s1 = m.step(s, 0.0, rudder, wind_ms, True, cfg.dt, phys)
    out.update(err=float(err), long_way=bool(long_way), r_fb=r_fb, r_ff=r_ff, r_des=r_des,
               rudder=rudder, rudder_saturated=abs(rudder) >= phys.rudder_stall_deg - 1e-9,
               delta_next=float(s1.delta), v_next=float(s1.v), psi_next=float(s1.psi), r_next=float(s1.r),
               sail_next=float(s1.sail), sim_rudder=sim_cmd,
               sim_psi_des=float(res.psi_des[nxt]) if nxt is not None else None,
               sim_v=float(res.v[nxt]) if nxt is not None else None,
               sim_psi=float(res.psi[nxt]) if nxt is not None else None)
    return out


def _maneuver_type(psi_seq):
    """Gerçekleşen pruva dizisi kuzeyden (rüzgarın geldiği yön) geçtiyse Tack, güneyden geçtiyse Gybe."""
    u = np.degrees(np.unwrap(np.radians(np.asarray(psi_seq, dtype=float))))
    lo, hi = u.min(), u.max()
    if np.floor(hi / 360.0) != np.floor(lo / 360.0):
        return "Tack"
    if np.floor((hi - 180.0) / 360.0) != np.floor((lo - 180.0) / 360.0):
        return "Gybe"
    return None


def angle_log(res, split_deg=10.0, min_duration=4.0):
    """
    Açı takip tablosu: rota, sistemin atadığı hedef pruvanın (ψ*) sabit kaldığı bölümlere ayrılır.
    Yeni satır: bacak değişimi, kontra değişimi (ψ* rüzgarın öbür tarafına geçti) ya da ψ*'ın
    bölüm başındaki değerinden split_deg'den fazla sapması; min_duration'dan kısa bölümler bir
    öncekine katılır. Her satır, bölümde atanan pruvayı, sabit seyirde auto-trim'in atadığı bom açısını,
    manevra sırasındaki en büyük dümen açısını ve manevra sonrası sabit seyirdeki ortalama dümen açısını verir.
    """
    n = len(res.t)
    if n < 2:
        return []
    starts = [0]
    for k in range(1, n):
        a = starts[-1]
        if (res.leg[k] != res.leg[a] or _side_of(res.psi_des[k]) != _side_of(res.psi_des[a])
                or abs(m.wrap180(res.psi_des[k] - res.psi_des[a])) > split_deg):
            starts.append(k)
    kept = [0]
    for j in range(1, len(starts)):
        end = starts[j + 1] if j + 1 < len(starts) else n - 1
        if res.leg[starts[j]] != res.leg[kept[-1]] or res.t[end] - res.t[starts[j]] >= min_duration:
            kept.append(starts[j])
    n_points = len(res.points)
    rows = []
    for j, a in enumerate(kept):
        last = j + 1 == len(kept)
        b = n if last else kept[j + 1]                # bölüm örnekleri: [a, b)
        psi_des = float(res.psi_des[min(a + 1, n - 1)])
        # Manevra penceresi: pruva atanan pruvaya 5° yaklaşana kadar
        err = np.abs([m.wrap180(psi_des - p) for p in res.psi[a:b]])
        settled = np.nonzero(err < 5.0)[0]
        mv_end = a + (int(settled[0]) if settled.size else b - a - 1)
        maneuver = _maneuver_type(res.psi[max(a - 1, 0):mv_end + 1]) if mv_end > a else None
        if maneuver is None and a > 0 and res.sail[a - 1] * m.leeward_sign(psi_des) < 0:
            maneuver = "Gybe"                         # tam pupadan bom karşı bordaya geçti
        if j == 0:
            event = "Başlangıç"
        elif res.leg[a] != res.leg[kept[j - 1]]:
            event = f"Ara durak {res.leg[a]} ✓"
        else:
            event = maneuver or "Rota düzeltme"
        rud_mv = res.rudder[a:mv_end + 1]
        k_max = int(np.argmax(np.abs(rud_mv)))
        steady = slice(mv_end, b) if b - mv_end > 1 else slice(b - 1, b)
        psi_st = np.radians(res.psi[steady])
        i_band = m.band_index(m.true_wind_angle(psi_des))
        rows.append(dict(
            no=j + 1,
            t0=float(res.t[a]), t1=float(res.t[b - 1]),
            leg=leg_name(int(res.leg[a]), n_points),
            event=event,
            maneuver=maneuver or "—",
            psi_des=psi_des,
            twa=float(m.true_wind_angle(psi_des)),
            regime=m.BANDS[i_band][2] if i_band is not None else "Ölü açı",
            tack="İskele" if _side_of(psi_des) == 1 else "Sancak",
            sail=float(np.mean(res.sail[steady])),
            rudder_max=float(rud_mv[k_max]),
            rudder_steady=float(np.mean(res.rudder[steady])),
            psi_mean=float(m.wrap360(np.degrees(np.arctan2(np.mean(np.sin(psi_st)), np.mean(np.cos(psi_st)))))),
            v_mean=float(np.mean(res.v[a:b])),
            leeway=float(np.mean(np.degrees(np.arctan2(res.v_lat[steady], np.maximum(res.v[steady], 1e-6))))),
        ))
    return rows
