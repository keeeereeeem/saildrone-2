import numpy as np
import pytest

import navigation as nav
import saildrone_model as m

SCENARIOS = [
    ((0, 0), (1000, 0)),        # tam apaz
    ((0, 0), (0, -1000)),       # tam rüzgar altı
    ((0, 0), (700, 700)),       # kuzeydoğu
    ((0, 0), (0, 1000)),        # tam rüzgar üstü (tack gerekir)
    ((0, 0), (300, 1000)),      # ölü açı içinde
    ((500, -300), (-400, 800)), # ofset başlangıç
]


@pytest.mark.parametrize("profile", m.PROFILE_KEYS)
@pytest.mark.parametrize("wind", [1.0, 3.0, 8.0, 15.0])
@pytest.mark.parametrize("start,target", SCENARIOS)
def test_reaches_target(profile, wind, start, target):
    r = nav.navigate(start, target, profile, wind)
    assert r.reached, r.reason
    assert r.final_distance <= nav.NavConfig().arrival_radius
    assert r.path_length >= r.straight_distance - nav.NavConfig().arrival_radius


def test_path_respects_no_go_and_rudder_limits():
    r = nav.navigate((0, 0), (0, 1000), "0012", 8.0)
    assert np.all(np.abs(r.rudder) <= m.RUDDER_MAX_DEG + 1e-9)
    # Tekne ölü açıda yalnızca tack sırasında kısa süre bulunabilir
    in_irons = np.array([m.true_wind_angle(p) <= m.NO_GO_DEG for p in r.psi])
    assert in_irons.mean() < 0.05
    assert r.n_tacks >= 1


def test_upwind_requires_tacking_beam_reach_does_not():
    up = nav.navigate((0, 0), (0, 1000), "0012", 8.0)
    beam = nav.navigate((0, 0), (1000, 0), "0012", 8.0)
    assert up.n_tacks + up.n_gybes >= 1
    assert beam.n_tacks == 0 and beam.n_gybes == 0
    assert up.time > beam.time


def test_light_wind_changes_tack_by_gybing():
    """2 m/s'de tack fiziksel olarak imkânsız: tekne kontrayı rüzgar altından değiştirmeli."""
    r = nav.navigate((0, 0), (0, 1000), "0012", 2.0)
    assert r.reached
    assert r.n_tacks == 0 and r.n_gybes >= 1


def test_start_in_irons_is_reported_as_failure():
    r = nav.navigate((0, 0), (1000, 0), "0012", 8.0, initial_heading=0.0)
    assert not r.reached
    assert "Ölü açı" in r.reason


def test_target_at_start():
    r = nav.navigate((5, 5), (5, 5), "cloth", 8.0)
    assert r.reached and r.time == 0.0


def test_time_limit_failure():
    r = nav.navigate((0, 0), (0, 1000), "0012", 3.0, cfg=nav.NavConfig(max_time=300))
    assert not r.reached and r.time == pytest.approx(300)


def test_long_range_mission():
    r = nav.navigate((0, 0), (0, 10000), "cloth", 5.0, cfg=nav.NavConfig(max_time=6 * 3600))
    assert r.reached


@pytest.mark.parametrize("bearing", range(0, 360, 30))
def test_reachability_all_bearings(bearing):
    tg = (1000 * np.sin(np.radians(bearing)), 1000 * np.cos(np.radians(bearing)))
    r = nav.navigate((0, 0), tg, "0015", 8.0)
    assert r.reached


def test_tack_feasibility_threshold():
    cfg = nav.NavConfig()
    assert not nav.tack_feasible(35.0, 1.0, cfg)
    assert nav.tack_feasible(35.0, 8.0, cfg)


@pytest.mark.parametrize("h_o,side,b,expected", [
    (324.5, -1, 350.0, False),   # hedef sancak kontra rotasının rüzgar üstünde
    (324.5, -1, 320.0, True),    # layline geçildi
    (210.0, -1, 180.0, False),   # rüzgar altı: hedef rotanın rüzgar altında
    (210.0, -1, 212.0, True),
])
def test_layline_geometry(h_o, side, b, expected):
    assert nav.on_layline(h_o, side, b, 1.0) is expected
