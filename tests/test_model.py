import numpy as np
import pytest

import saildrone_model as m
from conftest import BOATS, scaled

P = m.PHYS


# --- Ölü açı, simetri, adlandırma ---
@pytest.mark.parametrize("h", [0, 10, 30, 330, 345, 359.9, 360])
def test_no_go_zone_has_zero_thrust(h):
    regime, opt, ratio, tack, _, lee, _ = m.analyze_points_of_sail_360(h, "Kumaş Yelken")
    assert regime == m.NO_GO_REGIME
    assert ratio == 0.0 and lee == 0.0
    for sail in (0, 10, 40, 90):
        assert m.target_speed(h, sail, 0, "cloth", 8.0) == 0.0
        assert float(m.sail_force(h, 0.0, 8.0, sail)[0]) <= 0.0           # kaldırma yok: yalnızca sürükleme


@pytest.mark.parametrize("twa", [30.5, 40, 50, 80, 90, 120, 135, 150, 165, 179])
def test_port_starboard_symmetry(twa):
    """TWA aynı olan iki kontra aynı hızı, bomu ve kuvvetleri verir."""
    assert m.polar(twa, 8.0) == m.polar(360 - twa, 8.0)
    a = m.sail_force(twa, 3.0, 8.0, 30.0)
    b = m.sail_force(360 - twa, 3.0, 8.0, 30.0)
    assert np.allclose([float(x) for x in a], [float(x) for x in b])


@pytest.mark.parametrize("h,tack,lee", [(45, "İskele Kontra", 1.0), (90, "İskele Kontra", 1.0),
                                        (270, "Sancak Kontra", -1.0), (315, "Sancak Kontra", -1.0)])
def test_tack_naming_matches_wind_side(h, tack, lee):
    """Kuzey rüzgarında doğuya giden teknede rüzgar iskeleden gelir -> İskele Kontra."""
    _, _, _, t, side, l, _ = m.analyze_points_of_sail_360(h, "Kumaş")
    assert t == tack and l == lee
    assert ("İskele" in side) == (lee > 0)


# --- Görünür rüzgar ve yelken katsayıları ---
def test_apparent_wind_geometry():
    va, awa = m.apparent_wind(90.0, 8.0, 8.0)                          # tam apaz, v = U
    assert va == pytest.approx(8.0 * np.sqrt(2)) and awa == pytest.approx(45.0)
    va, awa = m.apparent_wind(180.0, 3.0, 8.0)                         # tam pupa
    assert va == pytest.approx(5.0) and awa == pytest.approx(180.0)
    va, awa = m.apparent_wind(40.0, 0.0, 8.0)                          # duran tekne: gerçek rüzgar
    assert va == pytest.approx(8.0) and awa == pytest.approx(40.0)
    for h in (40, 90, 135):                                            # hızlandıkça rüzgar pruvaya kayar
        assert m.apparent_wind(h, 4.0, 8.0)[1] < m.true_wind_angle(h)


def test_sail_coefficients_luff_and_stall():
    cl, cd = m.sail_coefficients(np.array([-10.0, 0.0]))
    assert np.all(cl == 0.0) and np.all(cd == m.CD_FLOG)               # α ≤ 0: yapraklama
    a = np.arange(0.5, 90.0, 0.5)
    cl, cd = m.sail_coefficients(a)
    assert np.all(cl > 0) and np.all(cd > 0)
    assert a[np.argmax(cl)] == pytest.approx(25.0, abs=1.0)            # kumaş yelkende stall ≈ 25°
    assert m.sail_coefficients(90.0)[1] > 1.2                           # rüzgara dik yelken: düz levha


# --- Yapraklama: bom rüzgar yönüne kadar açılırsa ---
@pytest.mark.parametrize("h,sail", [(55, 55), (55, 56), (55, 90), (305, 56), (305, -56), (40, 45), (85, 90)])
def test_sail_eased_to_wind_direction_luffs(h, sail):
    """Bom rüzgar yönüne kadar boşlanınca (α = AWA − bom ≤ 0) yelken rüzgara paraleldir: itki yok, tekne kıpırdamaz."""
    eff, txt, badge, camber = m.trim_status(h, sail, 0.0, 8.0)
    assert eff == 0.0 and badge == "badge-dead" and camber == 0.0
    assert m.target_speed(h, sail, 0, "cloth", 8.0) == 0.0
    r = m.simulate_open_loop(h, sail, 0, 8.0, 30.0)
    assert np.all(r["v"] == 0.0) and r["final_heading"] == h          # ileri hız yok, pruva değişmez
    # çırpınan yelkenin sürüklemesi tekneyi yalnızca rüzgar altına (güneye) sürükler: yavaş, yana doğru
    assert r["y"][-1] <= 0.0 and r["distance"] < 0.5 * 30.0
    f = m.force_moment_breakdown(h, sail, 0, 8.0)
    assert f["drive"] <= 0.0 and f["cl"] == 0.0


def test_dead_zone_widens_with_boom_angle():
    assert m.dead_zone_deg() == m.NO_GO_DEG                       # auto-trim: teknenin kendi sınırı
    assert m.dead_zone_deg(0) == m.dead_zone_deg(22) == m.NO_GO_DEG
    assert m.dead_zone_deg(56) == m.dead_zone_deg(-56) == 56.0
    assert m.dead_zone_deg(90) == 90.0


def test_dead_zone_matches_zero_thrust():
    """Elle trimde ölü açının içinde (TWA ≤ max(30°, |bom|)) tekne hiç hız alamaz; sınırdan uzakta hız alır."""
    for h in np.arange(0.0, 360.0, 3.0):
        twa = m.true_wind_angle(h)
        for sail in np.arange(0.0, 90.01, 6.0):
            v = m.target_speed(h, sail, 0, "cloth", 8.0)
            dz = m.dead_zone_deg(sail)
            if twa <= dz:
                assert v == 0.0, (h, sail)
                assert m.analyze_points_of_sail_360(h, sail_deg=sail, wind_ms=8.0)[0] == m.NO_GO_REGIME
            elif twa >= dz + 8.0 and twa - sail <= 75.0:
                # (bom orta hatta ve rüzgar dikten gelirse yelken düz levha gibi yalnızca yana iter: itki yok)
                assert v > 0.0, (h, sail)


@pytest.mark.parametrize("h,lee,tack", [(55, 1.0, "İskele Kontra"), (305, -1.0, "Sancak Kontra")])
def test_boom_dead_zone_keeps_tack_and_trim_hint(h, lee, tack):
    """Bom yüzünden ölü açıda: hız oranı 0, ama kontra / rüzgar altı bordası ve kasılacak optimum açı korunur."""
    regime, opt, ratio, t, _, l, desc = m.analyze_points_of_sail_360(h, sail_deg=56, wind_ms=8.0)
    assert regime == m.NO_GO_REGIME and ratio == 0.0
    assert opt == m.polar(h, 8.0)[1] and t == tack and l == lee
    assert "304°–056°" in desc
    assert m.analyze_points_of_sail_360(h, sail_deg=20, wind_ms=8.0)[0] != m.NO_GO_REGIME
    assert m.analyze_points_of_sail_360(h, wind_ms=8.0)[0] != m.NO_GO_REGIME


# --- Auto-trim ve kararlı hız (VPP) ---
@pytest.mark.parametrize("h", [40, 60, 90, 120, 150, 180])
@pytest.mark.parametrize("v", [0.0, 2.0, 5.0])
def test_best_trim_maximises_net_drive(h, v):
    """Auto-trim net itkiyi X − R_i(Y) en büyük yapar: hiçbir bom açısı (0.1° ızgara) daha iyi değildir."""
    def net(sail):
        x, y = m.sail_force(h, v, 8.0, sail)[:2]
        return x - m.induced_resistance(y, v)
    best = m.best_trim(h, v, 8.0)
    assert float(net(best)) >= float(np.max(net(np.arange(0.0, 90.01, 0.1)))) - 1e-3   # (R_i doğrusal değil: 1 mN payı)


@pytest.mark.parametrize("h,sail,rud", [(90, 33, 0), (90, 25, 15), (50, 20, -10), (150, 70, 5), (270, 33, 20)])
def test_force_balance_at_steady_speed(h, sail, rud):
    """Kararlı hızda yelken itkisi = gövde + yan kayma + dümen direnci (net ileri kuvvet sıfır)."""
    f = m.force_moment_breakdown(h, sail, rud, 8.0)
    assert f["v"] > 0.0
    assert f["net"] == pytest.approx(0.0, abs=1e-3)
    assert f["drive"] == pytest.approx(f["hull_drag"] + f["induced_drag"] + f["rudder_drag"], abs=1e-3)


@pytest.mark.parametrize("boat", BOATS)
def test_steady_speed_is_stable_and_reached_from_rest(boat):
    """Durgun tekne auto-trim'de VPP hızına yakınsar (kararlı kök) – her tekne boyutunda ve kütlesinde."""
    phys = BOATS[boat]
    for h in (45, 90, 135, 180):
        s = m.BoatState(0, 0, float(h), 0.0)
        for _ in range(int(240 / m.DT)):
            s = m.step(s, 0, 0, 8.0, auto_trim=True, phys=phys, hold_course=True)
        assert s.v == pytest.approx(m.polar(h, 8.0, phys)[0], rel=0.02), h


def test_net_force_gives_acceleration():
    f = m.force_moment_breakdown(90, 33, 0, 8.0, speed_ms=2.0)
    assert f["net"] > 0 and f["accel"] == pytest.approx(f["net"] / P.mass)
    g = m.force_moment_breakdown(90, 33, 0, 8.0, speed_ms=f["v_star"] + 2.0)
    assert g["net"] < 0                                                # kararlı hızın üstünde yavaşlar


def test_downwind_speed_below_wind_speed():
    """Tam pupada tekne rüzgardan hızlı gidemez (görünür rüzgar sıfıra iner)."""
    for U in (2.0, 5.0, 8.0, 15.0):
        v = m.polar(180, U)[0]
        assert 0.0 < v < U


def test_polar_shape():
    """Orsada yavaş, tam apaz en hızlı, pupada yavaş; rüzgar arkaya geldikçe bom açılır."""
    v = {t: m.polar(t, 8.0)[0] for t in (40, 90, 120, 180)}
    assert v[40] < v[90] and v[180] < v[120]
    assert 0.4 * 8.0 < v[90] < 1.5 * 8.0
    assert m.polar(90, 8.0)[1] < m.polar(150, 8.0)[1]


def test_polar_is_independent_of_wind_size_and_mass():
    """Boyutlandırma seçenekleri kapalıyken bütün kuvvetler hızın karesiyle ölçeklenir: V/U, en iyi bom ve yan kayma
    açısı rüzgar hızından, tekne boyutundan (geometrik ölçek) ve kütleden bağımsızdır."""
    ref = [(m.polar(t, 8.0)[0] / 8.0, m.polar(t, 8.0)[1]) for t in (40, 60, 90, 135, 180)]
    for U in (2.0, 15.0):
        for phys in (P, scaled(0.5), scaled(3.0), m.Physics(mass=200.0)):
            got = [(m.polar(t, U, phys)[0] / U, m.polar(t, U, phys)[1]) for t in (40, 60, 90, 135, 180)]
            assert np.allclose(got, ref, rtol=2e-3, atol=0.51), (U, phys)
    _, _, _, lee = m.polar_table(3.0)
    _, _, _, lee2 = m.polar_table(15.0, scaled(2.0))
    assert np.allclose(lee, lee2, atol=0.05)


def test_polar_table_matches_polar():
    twa, v, sail, lee = m.polar_table(8.0)
    for t in (45.0, 90.0, 150.0):
        i = int(np.argmin(np.abs(twa - t)))
        assert (v[i], sail[i]) == m.polar(t, 8.0)
        assert lee[i] > 0.0
    assert np.all(v[twa <= m.NO_GO_DEG] == 0.0)


# --- Boyutlandırma aşaması seçenekleri (varsayılan kapalı) ---
HEEL = m.Physics(heel_limit=True)


def test_sizing_options_off_by_default():
    assert not P.heel_limit and P.wave_cr == 0.0
    f = m.force_moment_breakdown(90, 0, 0, 20.0, auto_trim=True)
    assert f["heel_factor"] == 1.0                                    # kuvvetli rüzgarda da yelken azaltılmaz


def test_heel_limit_caps_side_force():
    for h in (40, 60, 90, 120):
        for v in (0.0, 4.0, 8.0):
            y = m.sail_force(h, v, 20.0, np.arange(0.0, 90.01, 5.0), HEEL)[1]
            assert np.all(np.abs(y) <= HEEL.side_force_max + 1e-9)
    f = m.force_moment_breakdown(90, 0, 0, 15.0, auto_trim=True, phys=HEEL)
    assert f["heel_factor"] < 1.0 and f["heel_moment"] == pytest.approx(f["righting_moment"], rel=1e-6)
    assert m.force_moment_breakdown(90, 0, 0, 3.0, auto_trim=True, phys=HEEL)["heel_factor"] == 1.0
    assert HEEL.righting_moment == pytest.approx(HEEL.mass * m.G * HEEL.beam / 2)


def test_righting_moment_limits_strong_wind_speed():
    """Devrilme sınırı açıkken kuvvetli rüzgarda hızı doğrultma momenti belirler: geniş tekne daha çok yelken taşır."""
    base = m.polar(90, 15.0, HEEL)[0]
    assert base < m.polar(90, 15.0)[0]
    assert m.polar(90, 15.0, m.Physics(heel_limit=True, beam=1.8))[0] > base
    assert m.polar(90, 15.0, m.Physics(heel_limit=True, heel_arm=1.0))[0] > base


def test_wave_drag_hump_option():
    hump = m.Physics(wave_cr=0.003)
    v_hump = 0.5 * np.sqrt(m.G * hump.hull_length)                   # Fr = 0.5
    assert m.hull_resistance(v_hump, hump) > m.hull_resistance(v_hump)
    v = np.linspace(0.0, 20.0, 401)
    assert np.all(np.diff(m.hull_resistance(v, hump)) > 0)            # direnç yine tekdüze artar


# --- Dirençler ve yan kayma ---
def test_hull_resistance_quadratic():
    v = np.linspace(0.0, 20.0, 401)
    r = m.hull_resistance(v)
    assert r[0] == 0.0 and np.all(np.diff(r) > 0)
    assert m.hull_resistance(6.0) == pytest.approx(4 * m.hull_resistance(3.0))
    assert m.hull_resistance(4.0) == pytest.approx(0.5 * P.rho_water * P.wetted_area * P.hull_cd * 16.0)


def test_leeway_solves_lateral_balance():
    for y, v in ((50.0, 3.0), (196.0, 6.0), (100.0, 0.0)):
        w = float(m.leeway_speed(y, v))
        lift = 0.5 * P.rho_water * P.board_area * P.board_cla * v * w
        cross = 0.5 * P.rho_water * P.hull_side_area * P.cross_cd * w * w
        assert lift + cross == pytest.approx(y)
    assert m.leeway_angle(100.0, 6.0) < m.leeway_angle(100.0, 3.0)     # hız arttıkça yan kayma açısı küçülür
    assert m.leeway_angle(100.0, 4.0, m.Physics(board_area=0.08)) > m.leeway_angle(100.0, 4.0)


def test_induced_resistance_zero_at_rest():
    """Yan kayma direnci hançer kaldırmasından gelir: tekne dururken ileri direnç yoktur (kalkış mümkün)."""
    assert float(m.induced_resistance(150.0, 0.0)) == 0.0
    assert float(m.induced_resistance(150.0, 4.0)) > 0.0


@pytest.mark.parametrize("boat", BOATS)
@pytest.mark.parametrize("h,sign", [(90, -1.0), (270, -1.0), (45, -1.0), (135, -1.0)])
def test_leeway_pushes_boat_downwind(boat, h, sign):
    """Yan kayma tekneyi rüzgar altına (kuzey rüzgarında güneye) iter."""
    on = m.simulate_open_loop(h, 0, 0, 8.0, 20.0, auto_trim=True, phys=BOATS[boat], hold_course=True)
    off_track = on["y"][-1] - on["distance"] * np.cos(np.radians(h))
    assert sign * off_track > 0.0


def test_rudder_drag_grows_with_angle():
    """Stall'dan sonra kaldırma artmaz ama ayrılmış akış direnci artar: tam dümen fren gibi çalışır."""
    d = [float(m.rudder_drag(5.0, a)) for a in (0, 10, 20, 25, 35)]
    assert all(a < b for a, b in zip(d, d[1:]))
    assert m.rudder_lift_coef(25.0) == m.rudder_lift_coef(35.0) == P.rudder_clmax
    assert float(m.rudder_drag(5.0, 35)) == float(m.rudder_drag(5.0, -35))


# --- Dönüş ---
def test_positive_rudder_turns_to_starboard():
    r = m.simulate_open_loop(90, 33, 15, 8.0, 2.0)
    assert m.wrap180(r["final_heading"] - 90) > 0
    r = m.simulate_open_loop(90, 33, -15, 8.0, 2.0)
    assert m.wrap180(r["final_heading"] - 90) < 0


def test_rudder_servo_rate_limit_and_nomoto_lag():
    s = m.initial_state(0, 0, 90, 8.0)
    s1 = m.step(s, 33, 30, 8.0, dt=0.1)
    assert s1.delta == pytest.approx(P.rudder_rate * 0.1)              # servo hız sınırı
    r_ss = m.rudder_yaw_rate(s1.v, s1.delta) + m.sail_yaw_rate(s1.side, 1.0, s1.v)
    assert 0 < s1.r < r_ss                                             # dönüş hızı gecikmeli yükselir


@pytest.mark.parametrize("boat", BOATS)
@pytest.mark.parametrize("h", [60, 120, 240, 300])
def test_weather_helm_rounds_up_into_wind(boat, h):
    r = m.simulate_open_loop(h, 0, 0, 8.0, 10.0, auto_trim=True, phys=BOATS[boat])
    assert m.true_wind_angle(r["final_heading"]) < m.true_wind_angle(h)


def test_balanced_and_lee_helm():
    bal = m.simulate_open_loop(90, 0, 0, 8.0, 10.0, auto_trim=True, phys=m.Physics(ce_arm=0.0))
    assert bal["final_heading"] == pytest.approx(90.0, abs=1e-9)
    lee = m.simulate_open_loop(90, 0, 0, 8.0, 10.0, auto_trim=True, phys=m.Physics(ce_arm=-0.1))
    assert m.true_wind_angle(lee["final_heading"]) > 90.0              # rüzgar altına kaçar


def test_holding_rudder_cancels_weather_helm():
    f = m.force_moment_breakdown(90, 33, 0, 8.0)
    d = f["rudder_balance"]
    assert d > 0
    assert m.rudder_yaw_rate(f["v"], d) + f["sail_yaw"] == pytest.approx(0.0, abs=1e-12)
    g = m.force_moment_breakdown(270, 33, 0, 8.0)
    assert g["rudder_balance"] == pytest.approx(-d)
    assert m.holding_rudder(f["side"], 1.0, 0.1) == 0.0                # dümen tutmayan hızda


@pytest.mark.parametrize("boat", BOATS)
@pytest.mark.parametrize("h", [40, 95, 150, 210, 265, 320])
def test_hold_course_keeps_heading(boat, h):
    phys = BOATS[boat]
    free = m.simulate_open_loop(h, 0, 0, 8.0, 30.0, auto_trim=True, phys=phys)
    held = m.simulate_open_loop(h, 0, 0, 8.0, 30.0, auto_trim=True, phys=phys, hold_course=True)
    assert m.true_wind_angle(free["final_heading"]) < m.true_wind_angle(h)   # dümen 0°: orsaya kaçar
    assert held["final_heading"] == pytest.approx(h, abs=0.05)
    assert np.all(np.sign(held["rudder"][1:]) == m.leeward_sign(h))


def test_start_in_irons_never_moves():
    r = m.simulate_open_loop(0, 40, 30, 10.0, 60.0)
    assert r["distance"] == 0.0 and r["final_heading"] == 0.0


def test_auto_trim_retrims_when_heading_changes():
    """Auto-trim dönüş boyunca bomu yeniler; sabit bom yeni pruvada verimsiz kalır."""
    auto = m.simulate_open_loop(60, 0, 20, 8.0, 20.0, auto_trim=True)
    fixed = m.simulate_open_loop(60, 24, 20, 8.0, 20.0, auto_trim=False)
    assert auto["distance"] > fixed["distance"]


def test_moment_signs():
    port = m.force_moment_breakdown(90, 33, 0, 8.0)                  # iskele kontra
    stbd = m.force_moment_breakdown(270, 33, 0, 8.0)                 # sancak kontra
    assert port["sail_moment"] < 0 < stbd["sail_moment"]            # her iki kontrada rüzgara döndürür
    turn = m.force_moment_breakdown(90, 33, 20, 8.0)
    assert turn["rudder_moment"] > 0 and turn["net_moment"] > 0 and turn["yaw_rate"] > 0
    assert turn["centripetal"] == pytest.approx(P.mass * turn["v"] ** 2 / turn["turn_radius"])


def test_no_force_in_irons():
    f = m.force_moment_breakdown(10, 40, 0, 8.0)
    assert f["drive"] <= 0.0 and f["v"] == 0.0 and f["cl"] == 0.0


# --- Fiziksel parametreler ---
@pytest.mark.parametrize("boat", BOATS)
def test_tack_through_wind_with_momentum(boat):
    """Orsadan (45°) stall dümeniyle ölü açı momentumla geçilir: her boyutta ve kütlede (3–20 m/s)."""
    phys = BOATS[boat]
    for U in (3.0, 8.0, 20.0):
        r = m.simulate_open_loop(45, 0, -phys.rudder_stall_deg, U, 40.0, auto_trim=True, phys=phys)
        assert any(180 < p < 330 for p in r["psi"]), U


def test_sail_area_and_mass_effects():
    assert m.polar(90, 3.0, m.Physics(sail_area=4.5))[0] > m.polar(90, 3.0)[0]   # yelken / direnç oranı büyür
    s0 = m.BoatState(0.0, 0.0, 90.0, 0.0)
    light = m.step(s0, 0, 0, 8.0, auto_trim=True, phys=m.Physics(mass=30.0))
    heavy = m.step(s0, 0, 0, 8.0, auto_trim=True, phys=m.Physics(mass=120.0))
    assert light.v > heavy.v > 0.0                                     # ağır tekne yavaş ivmelenir
    assert m.coast_distance(4.0, m.Physics(mass=100.0)) > m.coast_distance(4.0)
    assert m.coast_distance(6.0) > m.coast_distance(3.0) > 0.0


def test_hull_length_scales_turn_rate():
    r_short = m.rudder_yaw_rate(4.0, 10.0, m.Physics(hull_length=1.25))
    assert r_short == pytest.approx(2 * m.rudder_yaw_rate(4.0, 10.0))


def test_force_breakdown_uses_physics():
    phys = m.Physics(sail_area=4.0, mass=70.0, wetted_area=2.0)
    f = m.force_moment_breakdown(90, 33, 0, 6.0, phys=phys)
    assert f["v"] == pytest.approx(m.target_speed(90, 33, 0, "cloth", 6.0, phys=phys))
    assert f["hull_drag"] == pytest.approx(float(m.hull_resistance(f["v"], phys)))
    assert f["side_force_max"] == pytest.approx(phys.side_force_max)


def test_centerline_boom_in_beam_wind_gives_no_drive():
    """Bom orta hatta, rüzgar tam yandan: yelken rüzgara dik düz levha, kuvvet yalnızca yana – ileri itki yok."""
    x, y = (float(z) for z in m.sail_force(90.0, 0.0, 8.0, 0.0)[:2])
    assert x == pytest.approx(0.0, abs=1e-9) and y > 0.0
    assert m.target_speed(90, 0, 0, "cloth", 8.0) == 0.0
