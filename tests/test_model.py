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
    for sail in [0, 20, 38, 42, 46, 60, 90]:
        v_tel = 8.0 * 1.15 * m.trim_efficiency(sail, 46.0, "cloth")[0]
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
