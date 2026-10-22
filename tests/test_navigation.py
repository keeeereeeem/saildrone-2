import numpy as np
import pytest

import navigation as nav
import saildrone_model as m
from conftest import BOATS

SCENARIOS = [
    ((0, 0), (1000, 0)),        # tam apaz
    ((0, 0), (0, -1000)),       # tam rüzgar altı
    ((0, 0), (700, 700)),       # kuzeydoğu
    ((0, 0), (0, 1000)),        # tam rüzgar üstü (tack gerekir)
    ((0, 0), (300, 1000)),      # ölü açı içinde
    ((500, -300), (-400, 800)), # ofset başlangıç
]


@pytest.mark.parametrize("wind", [1.0, 3.0, 8.0, 15.0])
@pytest.mark.parametrize("start,target", SCENARIOS)
def test_reaches_target(wind, start, target):
    r = nav.navigate(start, target, "cloth", wind)
    assert r.reached, r.reason
    assert r.final_distance <= nav.NavConfig().arrival_radius
    assert r.path_length >= r.straight_distance - nav.NavConfig().arrival_radius


def test_path_respects_no_go_and_rudder_limits():
    r = nav.navigate((0, 0), (0, 1000), "cloth", 8.0)
    assert np.all(np.abs(r.rudder) <= m.PHYS.rudder_stall_deg + 1e-9)    # kontrolcü stall'ın ötesine gitmez
    in_irons = np.array([m.true_wind_angle(p) <= m.NO_GO_DEG for p in r.psi])
    assert in_irons.mean() < 0.05                                         # ölü açıda yalnızca tack sırasında
    assert r.n_tacks + r.n_gybes >= 1


def test_upwind_requires_tacking_beam_reach_does_not():
    up = nav.navigate((0, 0), (0, 1000), "cloth", 8.0)
    beam = nav.navigate((0, 0), (1000, 0), "cloth", 8.0)
    assert up.n_tacks + up.n_gybes >= 1
    assert beam.n_tacks == 0 and beam.n_gybes == 0
    assert up.time > beam.time


@pytest.mark.parametrize("wind", [8.0, 15.0, 20.0])
def test_reach_is_sailed_straight(wind):
    """Kerteriz yelkenle gidilebiliyorsa düz gidilir: anlık VMG'yi kovalayan sarmal (takip eğrisi) yok."""
    r = nav.navigate((0, 0), (1000, 0), "cloth", wind)
    assert r.reached and r.n_tacks == 0 and r.n_gybes == 0
    assert r.path_length < 1.02 * r.straight_distance
    assert np.max(np.abs(r.y)) < 25.0


def test_light_wind_changes_tack_by_gybing():
    """1 m/s'de süzülme mesafesi ölü açıyı geçmeye yetmez: tekne kontrayı rüzgar altından değiştirmeli."""
    r = nav.navigate((0, 0), (0, 1000), "cloth", 1.0)
    assert r.reached
    assert r.n_tacks == 0 and r.n_gybes >= 1


def test_start_in_irons_is_reported_as_failure():
    r = nav.navigate((0, 0), (1000, 0), "cloth", 8.0, initial_heading=0.0)
    assert not r.reached
    assert "Ölü açı" in r.reason


def test_target_at_start():
    r = nav.navigate((5, 5), (5, 5), "cloth", 8.0)
    assert r.reached and r.time == 0.0


def test_time_limit_failure():
    r = nav.navigate((0, 0), (0, 1000), "cloth", 3.0, cfg=nav.NavConfig(max_time=300))
    assert not r.reached and r.time == pytest.approx(300)


def test_long_range_mission():
    r = nav.navigate((0, 0), (0, 10000), "cloth", 5.0, cfg=nav.NavConfig(max_time=6 * 3600))
    assert r.reached


@pytest.mark.parametrize("bearing", range(0, 360, 30))
def test_reachability_all_bearings(bearing):
    tg = (1000 * np.sin(np.radians(bearing)), 1000 * np.cos(np.radians(bearing)))
    r = nav.navigate((0, 0), tg, "cloth", 8.0)
    assert r.reached


def test_tack_feasibility_uses_coasting_with_rudder_and_sail_drag():
    cfg = nav.NavConfig()
    assert not nav.tack_feasible(45.0, 0.5, cfg)
    assert nav.tack_feasible(45.0, 3.0, cfg)
    assert not nav.tack_feasible(45.0, 3.0, cfg, wind_ms=5.0)               # çırpınan yelkenin rüzgar freni
    d = m.PHYS.rudder_stall_deg
    assert m.coast_distance(3.0, m.PHYS, d) < m.coast_distance(3.0)          # dönen dümen tekneyi frenler
    assert m.coast_distance(3.0, m.PHYS, d, 8.0) < m.coast_distance(3.0, m.PHYS, d)
    assert nav.tack_capacity(3.0, m.PHYS, 8.0) == pytest.approx(
        np.degrees(np.sin(np.radians(d)) * m.coast_distance(3.0, m.PHYS, d, 8.0) / m.PHYS.hull_length))


@pytest.mark.parametrize("h_o,side,b,expected", [
    (324.5, -1, 350.0, False),   # hedef sancak kontra rotasının rüzgar üstünde
    (324.5, -1, 320.0, True),    # layline geçildi
    (210.0, -1, 180.0, False),   # rüzgar altı: hedef rotanın rüzgar altında
    (210.0, -1, 212.0, True),
])
def test_layline_geometry(h_o, side, b, expected):
    assert nav.on_layline(h_o, side, b, 1.0) is expected


@pytest.mark.parametrize("wind", [3.0, 8.0, 15.0])
def test_hull_course(wind):
    """Polar zarfı: zarf hızı ρ hiçbir pruvanın doğrudan hızından küçük olamaz; rüzgar üstü iki kontrada gidilir."""
    up = nav.hull_course(wind, 0.0, 5.0)
    assert up["direct"] is None and up["legs"][0][2] != up["legs"][1][2]
    assert m.true_wind_angle(up["legs"][0][0]) == pytest.approx(m.true_wind_angle(up["legs"][1][0]), abs=0.6)
    for b in (60.0, 90.0, 120.0, 150.0, 180.0):
        hc = nav.hull_course(wind, b, 5.0)
        assert hc["rho"] >= 0.99 * hc["direct"][1]           # (ızgara çözünürlüğü payı)
    beam = nav.hull_course(wind, 90.0, 5.0)
    assert beam["direct"][1] >= 0.99 * beam["rho"]


def test_route_with_waypoints_visits_all_points():
    pts = [(0, 0), (500, 500), (0, 1000), (-500, 500), (0, 0)]
    r = nav.navigate_route(pts, 8.0)
    assert r.reached, r.reason
    assert [lg["reached"] for lg in r.legs] == [True] * 4
    for i, p in enumerate(pts[1:]):
        k = np.nonzero(r.leg == i)[0][-1]                      # bacağın son örneği
        assert np.hypot(r.x[k] - p[0], r.y[k] - p[1]) <= nav.NavConfig().arrival_radius + 1.0
    assert np.all(np.diff(r.t) > 0)                            # bacaklar arası zaman sürekli


def test_route_without_waypoints_equals_single_leg():
    a = nav.navigate_route([(0, 0), (0, 1000)], 8.0)
    b = nav.navigate((0, 0), (0, 1000), "cloth", 8.0)
    assert a.time == b.time and a.n_tacks == b.n_tacks


def test_route_time_limit_applies_to_whole_route():
    r = nav.navigate_route([(0, 0), (1000, 0), (1000, 1000)], 3.0, cfg=nav.NavConfig(max_time=400))
    assert not r.reached and r.time <= 400 + 1e-9


def test_sail_angle_is_on_leeward_side():
    r = nav.navigate_route([(0, 0), (0, 1000)], 8.0)
    lee = np.array([m.leeward_sign(p) for p in r.psi])
    agree = (np.sign(r.sail) == lee) | (r.sail == 0.0)
    assert agree.mean() > 0.99                                 # (tack anında bir örnek gecikebilir)
    assert np.all(np.abs(r.sail) <= 90.0)


def test_angle_log_records_tacks_and_waypoints():
    r = nav.navigate_route([(0, 0), (0, 1000)], 8.0)
    rows = nav.angle_log(r)
    assert rows[0]["event"] == "Başlangıç"
    assert sum(row["event"] == "Tack" for row in rows) >= 1
    for row in rows:
        assert abs(row["rudder_max"]) <= m.RUDDER_MAX_DEG + 1e-9
        assert row["twa"] > m.NO_GO_DEG                       # atanan pruva hiçbir zaman ölü açıda değil
        assert np.sign(row["sail"]) == m.leeward_sign(row["psi_des"])
        assert row["leeway"] >= 0.0
    r2 = nav.navigate_route([(0, 0), (800, 0), (800, -800)], 8.0)
    events = [row["event"] for row in nav.angle_log(r2)]
    assert "Ara durak 1 ✓" in events


# --- Hesap tablosu (guidance_breakdown) ve fiziksel parametreler ---
def _check_breakdown_matches_simulation(points, wind, cfg, initial_heading=None):
    r = nav.navigate_route(points, wind, initial_heading=initial_heading, cfg=cfg)
    checked = 0
    for k in range(len(r.t) - 1):
        b = nav.guidance_breakdown(r, k, wind, cfg)
        if b["arrived"] or b["k"] != k:
            continue
        checked += 1
        assert m.wrap180(b["psi_des"] - b["sim_psi_des"]) == pytest.approx(0.0, abs=1e-9)
        assert b["rudder"] == pytest.approx(b["sim_rudder"], abs=1e-9)
        assert b["v_next"] == pytest.approx(b["sim_v"], abs=1e-9)
        assert m.wrap180(b["psi_next"] - b["sim_psi"]) == pytest.approx(0.0, abs=1e-9)
    return r, checked


@pytest.mark.parametrize("points,wind,heading", [
    ([(0, 0), (0, 1000)], 8.0, None),                       # rüzgar üstü, tack'ler
    ([(0, 0), (0, 1000)], 1.0, None),                       # hafif rüzgar, rüzgar altından dönüş
    ([(0, 0), (500, 0), (500, 500), (0, 500), (0, 0)], 8.0, None),   # ara duraklı kare
    ([(0, 0), (700, 700)], 6.0, 300.0),                     # verilen başlangıç pruvası
])
def test_breakdown_reproduces_every_guidance_step(points, wind, heading):
    _, checked = _check_breakdown_matches_simulation(points, wind, nav.NavConfig(), heading)
    assert checked > 20


def test_breakdown_with_custom_physics():
    cfg = nav.NavConfig(phys=m.Physics(mass=80.0, sail_area=4.5, hull_length=3.0, ce_arm=0.2))
    r, checked = _check_breakdown_matches_simulation([(0, 0), (200, 900)], 6.0, cfg)
    assert r.reached and checked > 20


def test_breakdown_arrival_and_vmg_fields():
    r = nav.navigate_route([(0, 0), (0, 1000)], 8.0)
    b0 = nav.guidance_breakdown(r, 0, 8.0)
    assert not b0["arrived"] and b0["bearing"] == pytest.approx(0.0) and b0["twa_bearing"] == 0.0
    assert b0["opts"][1][1] == pytest.approx(b0["opts"][-1][1])          # simetrik rüzgar üstü
    assert b0["hull"]["direct"] is None
    f = b0["forces"]
    assert f["drive"] > 0 and f["side"] > f["drive"]                    # orsada yan kuvvet itkiden büyük
    end = nav.guidance_breakdown(r, len(r.t) - 1, 8.0)
    assert end["arrived"] and end["dist"] <= nav.NavConfig().arrival_radius


@pytest.mark.parametrize("phys", [m.Physics(mass=80.0), m.Physics(sail_area=5.0), m.Physics(hull_length=3.5),
                                  m.Physics(ce_arm=0.3), m.Physics(rudder_area=0.02), m.Physics(board_area=0.08)])
def test_navigation_reaches_target_with_custom_physics(phys):
    for target in [(1000, 0), (0, 1000), (-600, -600)]:
        r = nav.navigate((0, 0), target, "cloth", 8.0, cfg=nav.NavConfig(phys=phys))
        assert r.reached, (phys, target, r.reason)


@pytest.mark.parametrize("boat", BOATS)
def test_navigation_any_boat_size_and_mass(boat):
    """Tekne ölçüleri henüz belli değil: güdüm her boyutta ve kütlede hedefe varmalı (rüzgar üstü, apaz, rüzgar altı)."""
    cfg = nav.NavConfig(phys=BOATS[boat])
    for U in (3.0, 15.0):
        for target in [(1000, 0), (0, 1000), (-600, -600), (300, 1000)]:
            r = nav.navigate((0, 0), target, "cloth", U, cfg=cfg)
            assert r.reached, (boat, U, target, r.reason)
            assert r.n_tacks + r.n_gybes <= 6


def test_reach_time_scales_inversely_with_wind():
    """Kararlı hız rüzgarla orantılı (V/U sabit): tam apaz 1 km'de süre × rüzgar hızı sabittir."""
    tu = [nav.navigate((0, 0), (1000, 0), "cloth", U).time * U for U in (3.0, 8.0, 15.0)]
    assert max(tu) / min(tu) < 1.03


@pytest.mark.parametrize("wind", [8.0, 15.0, 20.0])
def test_terminal_guidance_does_not_orbit_target(wind):
    """Hızlı teknede son yaklaşma (kerteriz dönüş hızı ileri beslemesi + dönüş çemberi kontrolü): hedef çevresinde
    dönmeden varılır."""
    for target in [(300, 1000), (700, 700), (-400, 800)]:
        r = nav.navigate((0, 0), target, "cloth", wind)
        assert r.reached and r.n_tacks + r.n_gybes <= 4, (wind, target, r.n_tacks, r.n_gybes)


def test_los_rate_sign():
    """Kuzeye giden teknenin sağ önündeki hedef geriye kayar (kerteriz artar), sol öndeki hedefin kerterizi azalır."""
    s = m.BoatState(0.0, 0.0, 10.0, 5.0)
    assert nav.los_rate(s, (50.0, 50.0)) > 0.0
    assert nav.los_rate(s, (-50.0, 50.0)) < 0.0
    assert nav.los_rate(m.BoatState(0.0, 0.0, 0.0, 5.0), (0.0, 100.0)) == pytest.approx(0.0)


def test_bigger_sail_arrives_sooner_in_light_wind():
    base = nav.navigate((0, 0), (1000, 0), "cloth", 3.0)
    big = nav.navigate((0, 0), (1000, 0), "cloth", 3.0, cfg=nav.NavConfig(phys=m.Physics(sail_area=4.5)))
    assert big.time < base.time
