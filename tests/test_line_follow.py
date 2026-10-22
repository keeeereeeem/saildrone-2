from dataclasses import replace

import numpy as np
import pytest

import line_follow as lf
import saildrone_model as m
from conftest import BOATS


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


def test_leeway_largest_close_hauled_and_independent_of_wind():
    """Yan kayma açısı orsada en büyük; kuvvetler hızın karesiyle ölçeklendiğinden rüzgar hızından bağımsızdır."""
    lee = {b: [run(b, wind=w).metrics["max_leeway"] for w in (3.0, 8.0, 15.0)] for b in (40, 90, 170)}
    assert lee[40][0] > lee[90][0] > lee[170][0]
    for b in lee:
        assert max(lee[b]) - min(lee[b]) < 0.15
    assert 0.5 < lee[40][1] < 6.0                                 # orsada birkaç derece


def test_leeway_is_compensated_by_crabbing():
    """Yan kaymaya rağmen tekne çizgide kalır: pruva rüzgara doğru, gidiş (COG) çizgi boyunca."""
    r = run(60)
    d, half = r.data, len(r.data["t"]) // 2
    assert np.max(np.abs(d["e"][half:])) < 0.3
    assert np.mean(d["psi"][half:]) < 60.0 - 0.5                # iskele kontra: pruva rüzgara (küçük açıya) döner
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
    r = run(bearing, wind=wind, start_offset=off, start_heading_err=err)
    mt = r.metrics
    assert r.status == "reached"
    assert mt["overshoot"] < 3.0
    assert mt["settle_time"] is not None
    assert mt["rms_e_second_half"] < 1.0
    assert mt["max_abs_rudder"] <= m.PHYS.rudder_stall_deg + 1e-9


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


def test_rudder_rate_and_stall_limit():
    """Dümen servosu hız sınırlı; kontrolcü dümeni stall açısının ötesine götürmez (orada yalnızca fren olur)."""
    cfg = lf.LineConfig()
    r = run(90, start_heading_err=-45.0)
    assert np.max(np.abs(np.diff(r.data["delta"]))) <= m.PHYS.rudder_rate * cfg.dt + 1e-9
    assert np.max(np.abs(r.data["delta"])) == pytest.approx(m.PHYS.rudder_stall_deg)


def test_same_dynamics_as_shared_step():
    """Düz rota simülasyonu, kayıtlı komutlarla diğer sekmelerin adım fonksiyonunu adım adım tekrarlar."""
    cfg = lf.LineConfig(start_offset=20.0)
    r = lf.simulate_line((0.0, 0.0), end(70), 8.0, cfg)
    d = r.data
    s = m.BoatState(d["x"][0], d["y"][0], d["psi"][0], d["v"][0], d["r"][0], d["delta"][0])
    for k in range(200):
        s = m.step(s, abs(d["sail"][k]), d["delta_cmd"][k], 8.0, dt=cfg.dt)
        assert (s.x, s.y, s.psi, s.v) == pytest.approx((d["x"][k + 1], d["y"][k + 1], d["psi"][k + 1], d["v"][k + 1]))


@pytest.mark.parametrize("heading", [40, 60, 90, 135, 170])
def test_recorded_forces_match_force_breakdown(heading):
    r = lf.simulate_line((0.0, 0.0), end(heading), 8.0, lf.LineConfig(feedforward=False))
    d = r.data
    f = m.force_moment_breakdown(heading, 0, d["delta"][0], 8.0, speed_ms=d["v"][0], auto_trim=True)
    assert d["drive"][0] == pytest.approx(f["drive"]) and d["side"][0] == pytest.approx(f["side"])
    assert d["heel_factor"][0] == pytest.approx(f["heel_factor"])
    assert d["leeway_angle"][0] == pytest.approx(f["leeway"])


def test_feedforward_uses_holding_rudder():
    r = run(90)
    d = r.data
    assert d["ff_hdg"][0] == pytest.approx(m.holding_rudder(d["side"][0], 1.0, d["v"][0]))
    assert d["ff_hdg"][0] > 0.0                                       # iskele kontrada orsaya kaçmayı dengeler


def test_depower_only_when_weather_helm_beats_rudder():
    assert run(90, start_heading_err=-45.0).metrics["depower_time"] == 0.0     # dümen yetiyor: yelken dolu
    strong = m.Physics(ce_arm=2.0)                                             # güçlü orsaya kaçma (denge dümeni ≈ 14°)
    on = run(60, wind=3.0, phys=strong, start_heading_err=-10.0)
    off = run(60, wind=3.0, phys=strong, start_heading_err=-10.0, depower=False)
    assert on.metrics["depower_time"] > 0.0
    assert on.status == "reached" and off.status == "stopped"


def test_low_wind_turn_toward_wind_does_not_wind_up_into_irons():
    r = run(40, wind=3.0, start_heading_err=20.0)
    assert r.status == "reached"
    assert np.min(r.data["twa"]) >= m.NO_GO_DEG


def test_strong_wind_heel_limit_reported():
    """Boyutlandırma aşaması seçeneği: devrilme sınırı açıkken yelken azaltılır ve kayda geçer."""
    heel = m.Physics(heel_limit=True)
    r = run(90, wind=15.0, phys=heel)
    assert r.status == "reached"
    assert r.metrics["min_heel_factor"] < 1.0
    assert np.all(np.abs(r.data["side"]) <= heel.side_force_max + 1e-9)
    assert run(90, wind=15.0).metrics["min_heel_factor"] == 1.0           # varsayılan: kapalı


@pytest.mark.parametrize("boat", BOATS)
def test_line_follow_any_boat_size_and_mass(boat):
    """Tekne ölçüleri henüz belli değil: kontrolcü her boyutta ve kütlede çizgiyi tutmalı."""
    phys = BOATS[boat]
    for brg, wind, off, err in [(45, 8.0, 30.0, 0.0), (60, 3.0, 30.0, 25.0), (90, 15.0, -30.0, 0.0),
                                (150, 8.0, -30.0, 20.0), (250, 15.0, 30.0, -20.0)]:
        r = run(brg, wind=wind, phys=phys, start_offset=off, start_heading_err=err)
        mt = r.metrics
        assert r.status == "reached", (boat, brg, wind)
        assert mt["overshoot"] < 0.5 * phys.hull_length + 1.0
        assert mt["rms_e_second_half"] < 0.5
        assert mt["rudder_travel"] < 400.0                               # dümen titreşimi yok
