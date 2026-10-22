from dataclasses import replace

import numpy as np
import pytest

import line_follow as lf
import saildrone_model as m


def end(bearing, length=600.0):
    return (length * np.sin(np.radians(bearing)), length * np.cos(np.radians(bearing)))


def run(bearing, wind=8.0, phys=m.PHYS, **kw):
    return lf.simulate_line((0.0, 0.0), end(bearing), wind, replace(lf.LineConfig(), **kw), phys)


def test_line_frame_and_cross_track_sign():
    length, bearing, u, n = lf.line_frame((0, 0), (0, 100))
    assert length == pytest.approx(100.0) and bearing == pytest.approx(0.0)
    assert n == pytest.approx([1.0, 0.0])                       # kuzeye giden çizginin sağı = doğu
    _, bearing, _, n = lf.line_frame((10, 10), (110, 10))
    assert bearing == pytest.approx(90.0) and n == pytest.approx([0.0, -1.0])


@pytest.mark.parametrize("bearing,status", [(0, "dead"), (10, "dead"), (350, "dead"), (32, "edge"), (330, "dead"),
                                            (36, "ok"), (90, "ok"), (180, "ok")])
def test_line_status(bearing, status):
    assert lf.line_status((0, 0), end(bearing))[2] == status


@pytest.mark.parametrize("bearing", [45, 90, 135, 170, 225, 300])
def test_holds_line_without_disturbance(bearing):
    r = run(bearing, leeway=False)
    assert r.status == "reached"
    assert np.max(np.abs(r.data["e"])) < 0.05
    assert np.max(np.abs(np.array([m.wrap180(p - bearing) for p in r.data["psi"]]))) < 0.5


@pytest.mark.parametrize("bearing,expected", [(40, 4.0), (90, 1.1), (135, 0.53), (170, 0.16)])
def test_leeway_angle_matches_point_of_sail_and_not_wind_speed(bearing, expected):
    for wind in (3.0, 8.0, 15.0):
        r = run(bearing, wind=wind)
        assert r.metrics["max_leeway"] == pytest.approx(expected, abs=0.1)


def test_leeway_is_compensated_by_crabbing():
    """Yan kaymaya rağmen tekne çizgide kalır: pruva rüzgara doğru, gidiş (COG) çizgi boyunca."""
    r = run(60)
    d, half = r.data, len(r.data["t"]) // 2
    assert np.max(np.abs(d["e"][half:])) < 0.3
    assert np.mean(d["psi"][half:]) < 60.0 - 1.0                # iskele kontra: pruva rüzgara (küçük açıya) döner
    assert np.mean(d["cog"][half:]) == pytest.approx(60.0, abs=0.3)


def test_port_starboard_mirror():
    a = run(60, start_offset=30.0)
    b = run(300, start_offset=-30.0)
    assert a.status == b.status == "reached"
    assert a.data["e"] == pytest.approx(-b.data["e"], abs=1e-6)
    assert a.data["delta"] == pytest.approx(-b.data["delta"], abs=1e-6)


@pytest.mark.parametrize("wind", [3.0, 8.0, 15.0])
@pytest.mark.parametrize("bearing", [60, 90, 135, 170, 250])
@pytest.mark.parametrize("off,err", [(30.0, 0.0), (-30.0, 0.0), (0.0, 20.0), (0.0, -20.0)])
def test_recovers_from_start_offset(wind, bearing, off, err):
    if (bearing, off) == (60, 30.0) and wind < 8.0:
        pytest.skip("orsaya yakın, rüzgar altında başlangıç: yalnızca yavaş dönüş beklenir (ayrı test)")
    r = run(bearing, wind=wind, start_offset=off, start_heading_err=err)
    mt = r.metrics
    assert r.status == "reached"
    assert mt["overshoot"] < 3.0
    assert mt["settle_time"] is not None
    assert mt["rms_e_second_half"] < 1.0
    assert mt["max_abs_rudder"] <= m.RUDDER_MAX_DEG


def test_close_hauled_leeward_start_returns_slowly_but_never_enters_dead_zone():
    """Orsada rüzgar altında başlayan tekne ancak (35° sınırı − yan kayma) açısıyla dönebilir: yavaş ama güvenli."""
    r = run(40, start_offset=30.0)
    assert r.status == "reached"
    assert abs(r.data["e"][-1]) < abs(r.data["e"][0])
    assert np.min(r.data["twa"]) >= m.NO_GO_DEG


@pytest.mark.parametrize("bearing", [40, 60, 300, 320])
def test_target_heading_never_in_dead_zone(bearing):
    for off in (-30.0, 30.0):
        r = run(bearing, start_offset=off)
        twa_des = np.array([m.true_wind_angle(p) for p in r.data["psi_des"]])
        assert np.min(twa_des) >= m.NO_GO_DEG + lf.DEAD_ZONE_MARGIN - 1e-9


def test_start_in_irons_stops():
    r = run(40, start_heading_err=-20.0)              # pruva 020°: ölü açı
    assert r.status == "stopped" and r.data["v"][0] == 0.0


def test_allowed_dead_zone_line_starts_sailable_then_stops():
    r = run(10, allow_dead_zone=True)
    assert r.status == "stopped"
    assert r.data["psi"][0] == pytest.approx(m.NO_GO_DEG + lf.DEAD_ZONE_MARGIN)
    assert r.data["v"][0] > 1.0
    assert m.true_wind_angle(r.data["psi"][-1]) <= m.NO_GO_DEG


def test_rudder_rate_and_limit():
    cfg = lf.LineConfig()
    r = run(90, start_heading_err=-45.0)
    assert np.max(np.abs(np.diff(r.data["delta"]))) <= cfg.rudder_rate * cfg.dt + 1e-9
    assert np.max(np.abs(r.data["delta"])) <= m.RUDDER_MAX_DEG + 1e-9


def test_force_based_weather_helm_matches_model_calibration():
    """Yelken yan kuvvetinden hesaplanan orsaya kaçma, tam apazda modelin 0.35 °/s kalibrasyonuyla aynı mertebede."""
    f = m.force_moment_breakdown(90, 0, 0, 8.0, auto_trim=True)
    r_sail = lf.sail_yaw_rate(f["side"], 1.0, f["v"])
    assert r_sail == pytest.approx(-m.WEATHER_HELM_DPS, rel=0.1)
    assert lf.holding_rudder(r_sail, f["v"]) == pytest.approx(f["rudder_balance"], rel=0.1)


@pytest.mark.parametrize("heading", [40, 60, 90, 135, 170])
def test_sail_forces_match_force_breakdown(heading):
    f = m.force_moment_breakdown(heading, 0, 0, 8.0, auto_trim=True)
    _, sail, eta, ratio, _, _ = lf.sail_state(heading, 0.0)
    drive, side = lf.sail_forces(8.0, ratio, eta, sail)
    assert drive == pytest.approx(f["drive"]) and side == pytest.approx(f["side"])


def test_depower_only_when_weather_helm_beats_rudder():
    assert run(90, start_heading_err=-45.0).metrics["depower_time"] == 0.0     # dümen yetiyor: yelken dolu
    strong = m.Physics(ce_arm=1.5)                                             # güçlü orsaya kaçma, hafif rüzgar
    on = run(60, wind=3.0, phys=strong, start_heading_err=-10.0)
    off = run(60, wind=3.0, phys=strong, start_heading_err=-10.0, depower=False)
    assert on.metrics["depower_time"] > 0.0
    assert on.status == "reached" and off.status == "stopped"


def test_low_wind_turn_toward_wind_does_not_wind_up_into_irons():
    r = run(40, wind=3.0, start_heading_err=20.0)
    assert r.status == "reached"
    assert np.min(r.data["twa"]) >= m.NO_GO_DEG
