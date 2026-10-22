import numpy as np
import pytest

import saildrone_model as m

PROFILES = m.PROFILE_KEYS


@pytest.mark.parametrize("h", [0, 10, 30, 330, 345, 359.9, 360])
def test_no_go_zone_has_zero_thrust(h):
    regime, opt, ratio, tack, _, lee, _ = m.analyze_points_of_sail_360(h, "Kumaş Yelken")
    assert regime == m.NO_GO_REGIME
    assert ratio == 0.0 and lee == 0.0
    assert m.target_speed(h, 40, 0, "cloth", 8.0) == 0.0


@pytest.mark.parametrize("profile", PROFILES)
@pytest.mark.parametrize("twa", [30.5, 40, 50, 50.5, 80, 90, 120, 135, 150, 165, 179])
def test_port_starboard_symmetry(profile, twa):
    """TWA aynı olan iki kontra aynı polar değerini vermeli (sınırlarda dahil)."""
    assert m.polar_point(twa, profile) == m.polar_point(360 - twa, profile)


@pytest.mark.parametrize("h,tack,lee", [(45, "İskele Kontra", 1.0), (90, "İskele Kontra", 1.0),
                                        (270, "Sancak Kontra", -1.0), (315, "Sancak Kontra", -1.0)])
def test_tack_naming_matches_wind_side(h, tack, lee):
    """Kuzey rüzgarında doğuya giden teknede rüzgar iskeleden gelir -> İskele Kontra."""
    _, _, _, t, side, l, _ = m.analyze_points_of_sail_360(h, "Kumaş")
    assert t == tack and l == lee
    assert ("İskele" in side) == (lee > 0)


@pytest.mark.parametrize("profile", PROFILES)
def test_trim_efficiency_peak_and_bounds(profile):
    for opt, _ in m.POLAR[profile]:
        assert m.trim_efficiency(opt, opt, profile)[0] == 1.0
        for s in np.arange(0, 91, 1.0):
            eta = m.trim_efficiency(s, opt, profile)[0]
            assert 0.0 < eta <= 1.0


@pytest.mark.parametrize("h,sail", [(55, 55), (55, 56), (55, 90), (305, 56), (305, -56), (40, 45), (85, 90)])
def test_sail_eased_to_wind_direction_luffs(h, sail):
    """Bom rüzgar yönüne kadar boşlanınca (TWA − bom ≤ 0) yelken rüzgara paraleldir: itki yok, tekne kıpırdamaz."""
    eta, txt, badge, camber = m.trim_efficiency(sail, m.polar_point(h)[0], "cloth", m.true_wind_angle(h))
    assert eta == 0.0 and badge == "badge-dead" and camber == 0.0
    assert m.target_speed(h, sail, 0, "cloth", 8.0) == 0.0
    r = m.simulate_open_loop(h, sail, 0, "cloth", 8.0, 30.0)
    assert r["distance"] == 0.0 and r["final_heading"] == h
    f = m.force_moment_breakdown(h, sail, 0, 8.0)
    assert f["drive"] == 0.0 and f["side"] == 0.0


def test_partial_luff_limits_efficiency():
    """0 < α < LUFF_DEG: orsa yakası çırpar, verim α / LUFF_DEG ile sınırlı ve bom boşlandıkça düşer."""
    twa, opt = 55.0, m.polar_point(55)[0]
    etas = [m.trim_efficiency(s, opt, "cloth", twa)[0] for s in np.arange(twa - m.LUFF_DEG, twa + 0.01, 0.5)]
    assert all(e <= (twa - s) / m.LUFF_DEG + 1e-12
               for e, s in zip(etas, np.arange(twa - m.LUFF_DEG, twa + 0.01, 0.5)))
    assert all(a >= b for a, b in zip(etas, etas[1:])) and etas[-1] == 0.0


@pytest.mark.parametrize("twa", np.arange(30.5, 180.01, 0.5))
def test_luff_limit_leaves_calibrated_trim_unchanged(twa):
    """Polar optimumu ± tolerans içinde (ve fazla kasılı tarafta) yapraklama sınırı hiç devreye girmez."""
    opt, tol = m.polar_point(twa)[0], m.TRIM["cloth"]["tol"]
    for s in np.arange(0.0, min(opt + tol, 90.0) + 0.01, 0.5):
        assert m.trim_efficiency(s, opt, "cloth", twa) == m.trim_efficiency(s, opt, "cloth")


def test_downwind_sail_cannot_be_eased_into_luff():
    """Bom en fazla 90° açılır: TWA ≥ 90° + LUFF_DEG seyirlerde yelken boşlayarak yapraklatılamaz."""
    for twa in np.arange(90.0 + m.LUFF_DEG, 180.01, 1.0):
        opt = m.polar_point(twa)[0]
        assert m.trim_efficiency(90, opt, "cloth", twa) == m.trim_efficiency(90, opt, "cloth")


def test_dead_zone_widens_with_boom_angle():
    assert m.dead_zone_deg() == m.NO_GO_DEG                       # auto-trim: teknenin kendi sınırı
    assert m.dead_zone_deg(0) == m.dead_zone_deg(22) == m.NO_GO_DEG
    assert m.dead_zone_deg(56) == m.dead_zone_deg(-56) == 56.0
    assert m.dead_zone_deg(90) == 90.0


def test_dead_zone_matches_zero_thrust():
    """Elle trimde tekne tam olarak ölü açının içindeyken (TWA ≤ max(30°, |bom|)) itkisiz kalır."""
    for h in np.arange(0.0, 360.0, 1.0):
        for sail in np.arange(0.0, 90.01, 2.0):
            in_dead = m.true_wind_angle(h) <= m.dead_zone_deg(sail)
            assert (m.target_speed(h, sail, 0, "cloth", 8.0) == 0.0) == in_dead
            assert (m.analyze_points_of_sail_360(h, sail_deg=sail)[0] == m.NO_GO_REGIME) == in_dead


@pytest.mark.parametrize("h,lee,tack", [(55, 1.0, "İskele Kontra"), (305, -1.0, "Sancak Kontra")])
def test_boom_dead_zone_keeps_tack_and_trim_hint(h, lee, tack):
    """Bom yüzünden ölü açıda: hız oranı 0, ama kontra / rüzgar altı bordası ve kasılacak optimum açı korunur."""
    regime, opt, ratio, t, _, l, desc = m.analyze_points_of_sail_360(h, sail_deg=56)
    assert regime == m.NO_GO_REGIME and ratio == 0.0
    assert opt == 32.0 and t == tack and l == lee
    assert "304°–056°" in desc
    # aynı pruvada bom kasılınca ölü açıdan çıkar; auto-trim (sail_deg None) pruvaya göre değerlendirilir
    assert m.analyze_points_of_sail_360(h, sail_deg=40)[0] != m.NO_GO_REGIME
    assert m.analyze_points_of_sail_360(h)[0] != m.NO_GO_REGIME


def test_rudder_penalty():
    assert m.rudder_penalty(0) == 1.0
    assert m.rudder_penalty(35) == pytest.approx(0.78)
    assert m.rudder_penalty(-35) == m.rudder_penalty(35)
    assert m.rudder_penalty(90) == m.rudder_penalty(35)   # mekanik sınır


@pytest.mark.parametrize("profile", PROFILES)
@pytest.mark.parametrize("h", [45, 90, 135, 180, 225, 270, 315])
def test_straight_run_distance_equals_speed_times_time(profile, h):
    """Dümen 0, auto-trim: gidilen yol = gösterilen hız × süre (eski kodda tutarsızdı)."""
    v = m.target_speed(h, 0, 0, profile, 8.0, auto_trim=True)
    r = m.simulate_open_loop(h, 0, 0, profile, 8.0, 30.0, auto_trim=True)
    assert r["v0"] == pytest.approx(v)
    # orsaya kaçma pruvayı ~10° değiştirebilir -> bant değişimi dışında hız sabittir
    assert r["distance"] == pytest.approx(v * 30.0, rel=0.35)


def test_manual_trim_speed_matches_telemetry_formula():
    """Açık çevrimdeki hız ile anlık telemetri hızı aynı verim fonksiyonunu kullanmalı."""
    for sail in [0, 20, 38, 42, 46, 60, 85, 90]:
        v_tel = 8.0 * 1.15 * m.trim_efficiency(sail, 46.0, "cloth", twa=90.0)[0]
        r = m.simulate_open_loop(90, sail, 0, "cloth", 8.0, 1.0, tau=0.0)
        assert r["v0"] == pytest.approx(v_tel)


def test_positive_rudder_turns_to_starboard():
    r = m.simulate_open_loop(90, 46, 15, "cloth", 8.0, 2.0)
    assert m.wrap180(r["final_heading"] - 90) > 0
    r = m.simulate_open_loop(90, 46, -15, "cloth", 8.0, 2.0)
    assert m.wrap180(r["final_heading"] - 90) < 0


@pytest.mark.parametrize("h", [60, 120, 240, 300])
def test_weather_helm_rounds_up_into_wind(h):
    r = m.simulate_open_loop(h, 0, 0, "cloth", 8.0, 10.0, auto_trim=True)
    assert m.true_wind_angle(r["final_heading"]) < m.true_wind_angle(h)


def test_holding_rudder_cancels_weather_helm():
    """Rotayı tut: dümen orsaya kaçmayı tam dengeler; kuvvet modelindeki denge dümeniyle aynı mertebede."""
    f = m.force_moment_breakdown(90, 46, 0, 8.0)
    r = m.holding_rudder_deg(f["v"], 90)
    assert r == pytest.approx(f["rudder_balance"], rel=0.1)
    assert m.yaw_rate_dps(f["v"], r, 90) == pytest.approx(0.0, abs=1e-12)
    assert m.holding_rudder_deg(f["v"], 270) == pytest.approx(-r)
    assert m.holding_rudder_deg(0.1, 90) == 0.0                       # dümen tutmayan hızda dönüş yok


@pytest.mark.parametrize("h", [40, 95, 150, 210, 265, 320])
def test_hold_course_keeps_heading(h):
    free = m.simulate_open_loop(h, 0, 0, "cloth", 8.0, 30.0, auto_trim=True)
    held = m.simulate_open_loop(h, 0, 0, "cloth", 8.0, 30.0, auto_trim=True, hold_course=True)
    assert m.true_wind_angle(free["final_heading"]) < m.true_wind_angle(h)   # dümen 0°: orsaya kaçar
    assert held["final_heading"] == pytest.approx(h, abs=1e-9)
    assert np.all(np.sign(held["rudder"]) == m.leeward_sign(h))
    assert held["distance"] == pytest.approx(held["v0"] * 30.0, rel=1e-3)


def test_hold_course_follows_heading_line():
    """Pruva 095°'de tutulunca tekne doğunun 5° altına, yani hafifçe güneye gider."""
    held = m.simulate_open_loop(95, 0, 0, "cloth", 8.0, 30.0, auto_trim=True, hold_course=True)
    assert held["y"][-1] == pytest.approx(held["distance"] * np.cos(np.radians(95)), rel=1e-9)
    assert held["y"][-1] < -20.0


def test_open_loop_reports_rudder_used():
    r = m.simulate_open_loop(90, 46, 12, "cloth", 8.0, 5.0)
    assert len(r["rudder"]) == len(r["psi"]) and np.all(r["rudder"] == 12.0)


def test_start_in_irons_never_moves():
    r = m.simulate_open_loop(0, 40, 30, "cloth", 10.0, 60.0)
    assert r["distance"] == 0.0 and r["final_heading"] == 0.0


def test_surge_lag_converges_to_target_speed():
    s = m.BoatState(0, 0, 90.0, 0.0)
    for _ in range(200):
        s = m.step(s, 0, 0, "cloth", 8.0, auto_trim=True, tau=3.0)
    assert s.v == pytest.approx(m.target_speed(s.psi, 0, 0, "cloth", 8.0, True), rel=1e-3)


def test_auto_trim_retrims_when_heading_changes():
    """Auto-trim'de dönüş boyunca verim 1 kalmalı (eski kod başlangıç açısında donduruyordu)."""
    auto = m.simulate_open_loop(60, 0, 20, "cloth", 8.0, 20.0, auto_trim=True)
    fixed = m.simulate_open_loop(60, 32, 20, "cloth", 8.0, 20.0, auto_trim=False)
    assert auto["distance"] > fixed["distance"]


@pytest.mark.parametrize("h,sail,rud", [(90, 46, 0), (90, 30, 15), (50.5, 32, -10), (150, 65, 5), (270, 46, 20)])
def test_force_balance_at_steady_speed(h, sail, rud):
    """Kararlı hızda yelken itkisi = dümen sürüklemesi + gövde direnci (net ileri kuvvet sıfır)."""
    f = m.force_moment_breakdown(h, sail, rud, 8.0)
    assert f["net"] == pytest.approx(0.0, abs=1e-9)
    assert f["drive"] == pytest.approx(f["rudder_drag"] + f["hull_drag"])
    assert f["v"] == pytest.approx(m.target_speed(h, sail, rud, "cloth", 8.0))


def test_net_force_gives_surge_equation():
    """Hız V*'dan düşükse net kuvvet m·(V*−v)/τ kadar ileri yönlüdür."""
    f = m.force_moment_breakdown(90, 46, 0, 8.0, speed_ms=4.0)
    assert f["net"] == pytest.approx(m.PHYS.mass * (f["v_star"] - 4.0) / m.TAU_SURGE)
    assert f["accel"] == pytest.approx((f["v_star"] - 4.0) / m.TAU_SURGE)


def test_sail_force_components():
    f = m.force_moment_breakdown(90, 46, 0, 8.0)
    assert f["drive"] == pytest.approx(f["normal"] * np.sin(np.radians(46)))
    assert f["side"] == pytest.approx(f["normal"] * np.cos(np.radians(46)))
    assert f["va"] == pytest.approx(np.hypot(8.0, f["v"]))           # tam apazda görünür rüzgar


def test_moment_signs_and_balance():
    port = m.force_moment_breakdown(90, 46, 0, 8.0)                  # iskele kontra
    stbd = m.force_moment_breakdown(270, 46, 0, 8.0)                 # sancak kontra
    assert port["sail_moment"] < 0 < stbd["sail_moment"]            # her iki kontrada rüzgara döndürür
    assert port["rudder_balance"] > 0 > stbd["rudder_balance"]
    # dengeleyici dümen açısı simülasyondaki kalıcı dümen açısıyla aynı mertebede (kalibrasyon noktası)
    dss = np.degrees(np.arcsin(np.radians(m.WEATHER_HELM_DPS) * m.L_HULL / port["v"]))
    assert port["rudder_balance"] == pytest.approx(dss, rel=0.1)
    turn = m.force_moment_breakdown(90, 46, 20, 8.0)
    assert turn["rudder_moment"] > 0 and turn["net_moment"] > 0 and turn["yaw_rate"] > 0
    assert turn["centripetal"] == pytest.approx(m.PHYS.mass * turn["v"] ** 2 / turn["turn_radius"])


def test_no_force_in_irons():
    f = m.force_moment_breakdown(10, 40, 0, 8.0)
    assert f["drive"] == 0.0 and f["normal"] == 0.0 and f["side"] == 0.0 and f["v"] == 0.0


# --- Kullanıcı tarafından değiştirilebilen fiziksel parametreler ---
def test_default_physics_reproduces_calibrated_constants():
    P = m.Physics()
    assert P == m.PHYS
    assert P.tau == m.TAU_SURGE
    assert P.speed_scale == 1.0
    assert P.weather_helm == m.WEATHER_HELM_DPS
    assert P.hull_length == m.L_HULL and P.rudder_drag == m.RUDDER_DRAG
    assert P.ce_offset == pytest.approx(0.20)


def test_sail_area_and_hull_drag_scale_speed():
    base = m.target_speed(90, 46, 0, "cloth", 8.0)
    assert m.target_speed(90, 46, 0, "cloth", 8.0, phys=m.Physics(sail_area=30.0)) == pytest.approx(1.5 * base)
    assert m.target_speed(90, 46, 0, "cloth", 8.0, phys=m.Physics(hull_drag=2 * m.PHYS.hull_drag)) == pytest.approx(base / 2)
    # kütle kararlı hızı değiştirmez, yalnızca zaman sabitini büyütür
    heavy = m.Physics(mass=500.0)
    assert m.target_speed(90, 46, 0, "cloth", 8.0, phys=heavy) == pytest.approx(base)
    assert heavy.tau == pytest.approx(2 * m.TAU_SURGE)


def test_mass_slows_surge_response():
    s0 = m.BoatState(0.0, 0.0, 90.0, 0.0)
    light = m.step(s0, 0, 0, "cloth", 8.0, auto_trim=True, phys=m.Physics(mass=125.0))
    heavy = m.step(s0, 0, 0, "cloth", 8.0, auto_trim=True, phys=m.Physics(mass=500.0))
    assert light.v > heavy.v > 0.0


def test_rudder_drag_coefficient():
    assert m.rudder_penalty(35, m.Physics(rudder_drag=0.5)) == pytest.approx(0.5)
    assert m.rudder_penalty(35, m.Physics(rudder_drag=0.0)) == 1.0


def test_hull_length_and_weather_helm_scale_yaw():
    v, rud = 6.0, 10.0
    r_short = m.yaw_rate_dps(v, rud, 0.0, m.Physics(hull_length=2.3))   # ölü açıda: orsaya kaçma yok
    r_base = m.yaw_rate_dps(v, rud, 0.0)
    assert r_short == pytest.approx(2 * r_base)
    assert m.Physics(ce_arm=0.0).weather_helm == 0.0                     # CE = CLR: denge
    assert m.Physics(ce_arm=0.40).weather_helm == pytest.approx(2 * m.WEATHER_HELM_DPS)
    assert m.Physics(ce_arm=-0.1).weather_helm < 0.0                     # rüzgar altına kaçma (lee helm)
    assert m.Physics(rudder_arm=4.2).weather_helm == pytest.approx(m.WEATHER_HELM_DPS / 2)


@pytest.mark.parametrize("phys", [m.Physics(ce_arm=0.35), m.Physics(rudder_area=0.3, rudder_arm=1.5),
                                  m.Physics(hull_drag=120.0, hull_length=6.0), m.Physics(sail_area=28.0, mass=400.0)])
def test_weather_helm_consistent_with_force_balance(phys):
    """Kinematik orsaya kaçma, her parametre setinde kuvvet modelindeki dengeleyici dümen açısıyla uyumludur."""
    f = m.force_moment_breakdown(90, 46, 0, 8.0, phys=phys)
    dss = np.degrees(np.arcsin(np.radians(phys.weather_helm) * phys.hull_length / f["v"]))
    assert f["rudder_balance"] == pytest.approx(dss, rel=0.1)


def test_force_breakdown_uses_physics():
    phys = m.Physics(sail_area=30.0, mass=400.0, hull_drag=100.0)
    f = m.force_moment_breakdown(90, 46, 0, 8.0, phys=phys)
    assert f["v"] == pytest.approx(m.target_speed(90, 46, 0, "cloth", 8.0, phys=phys))
    assert f["drive"] == pytest.approx(f["hull_drag"]) and f["hull_drag"] == pytest.approx(100.0 * f["v"])
    g = m.force_moment_breakdown(90, 46, 0, 8.0, speed_ms=2.0, phys=phys)
    assert g["accel"] == pytest.approx((g["v_star"] - 2.0) / phys.tau)
