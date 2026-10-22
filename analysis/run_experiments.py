"""
Rapor deneyleri (kuvvet tabanlı VPP modeli, boyuttan bağımsız; varsayılan değerler ~50 kg otonom katamaran):
hız poları, yelken trimi, kuvvet tabloları, navigasyon senaryo matrisi, rota çizimleri, ara duraklı rota ve açı
takip tablosu, erişilebilirlik (ETA) analizi, tack analizi, düz rota (PID) başarımı, tekne boyutu / kütle
bağımsızlığı.

Çıktılar: docs/figures/*.png, docs/results/*.md|csv
Çalıştırma:  python analysis/run_experiments.py
"""
import csv
import os
import sys
from dataclasses import replace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import line_follow as lf  # noqa: E402
import navigation as nav  # noqa: E402
import saildrone_model as m  # noqa: E402

sys.path.insert(0, os.path.join(ROOT, "tests"))
from conftest import BOATS  # noqa: E402

FIG = os.path.join(ROOT, "docs", "figures")
RES = os.path.join(ROOT, "docs", "results")
os.makedirs(FIG, exist_ok=True)
os.makedirs(RES, exist_ok=True)

P = m.PHYS
WINDS = [1.0, 3.0, 8.0, 15.0, 20.0]
POLAR_WINDS = [(3.0, "#86b6f6"), (8.0, "#2a78d6"), (15.0, "#0b3d91")]     # tek renk tonu, açıktan koyuya
SCENARIOS = [
    ("S1 Tam apaz (Doğu)", (0, 0), (1000, 0)),
    ("S2 Rüzgar altı (Güney)", (0, 0), (0, -1000)),
    ("S3 Kuzeydoğu 45°", (0, 0), (700, 700)),
    ("S4 Tam rüzgar üstü (Kuzey)", (0, 0), (0, 1000)),
    ("S5 Ölü açı içi (017°)", (0, 0), (300, 1000)),
    ("S6 Ofset başlangıç (KB)", (500, -300), (-400, 800)),
]


def _clean(ax):
    ax.grid(alpha=0.3)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def fig_polar():
    """VPP hız poları V/U: rüzgar hızından ve tekne boyutundan bağımsız tek eğri. Kesikli: boyutlandırma aşaması
    seçeneği (devrilme sınırı) açıkken 50 kg örnek tekne – kuvvetli rüzgarda hız sınırlanır."""
    fig, ax = plt.subplots(subplot_kw=dict(projection="polar"), figsize=(6.6, 6.6))
    twa, v, sail, lee = m.polar_table(8.0)
    ok = twa > m.NO_GO_DEG
    th = np.radians(twa[ok])
    ax.plot(th, v[ok] / 8.0, color="#2a78d6", lw=2.4, label="V/U (her rüzgar ve tekne boyutu)")
    ax.plot(-th, v[ok] / 8.0, color="#2a78d6", lw=2.4)
    heel = m.Physics(heel_limit=True)
    for U, color in ((8.0, "#eb6834"), (15.0, "#b91c1c")):
        twa_h, v_h, _, _ = m.polar_table(U, heel)
        ax.plot(np.radians(twa_h[ok]), v_h[ok] / U, color=color, lw=1.4, ls="--",
                label=f"devrilme sınırı açık, U = {U:.0f} m/s (boyutlandırma)")
        ax.plot(-np.radians(twa_h[ok]), v_h[ok] / U, color=color, lw=1.4, ls="--")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.fill_between(np.radians(np.linspace(-30, 30, 50)), 0, 1.2, color="#ef4444", alpha=0.12)
    ax.set_rlim(0, 1.2)
    ax.set_rlabel_position(200)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.15, -0.14), fontsize=8, frameon=False)
    ax.set_title("VPP hız poları – kararlı hız / rüzgar hızı V/U (rüzgar kuzeyden)", fontsize=10, pad=16)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "polar.png"), dpi=130)
    plt.close(fig)


def polar_table_md():
    """Rüzgardan bağımsız polar: V/U, en iyi bom, AWA, yan kayma; devrilme sınırı açıkken 8 / 15 m/s'deki V/U."""
    heel = m.Physics(heel_limit=True)
    rows = ["| TWA (°) | Seyir | V/U | En iyi bom (°) | AWA (°) | Yan kayma λ (°) | V/U, devrilme sınırı U=8 | V/U, devrilme sınırı U=15 |",
            "|---|---|---|---|---|---|---|---|"]
    for t in (35, 40, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180):
        v, sail = m.polar(t, 8.0)
        x, y, awa = (float(z) for z in m.sail_force(t, v, 8.0, sail)[:3])
        lam = float(m.leeway_angle(y, v))
        h8, h15 = m.polar(t, 8.0, heel)[0] / 8.0, m.polar(t, 15.0, heel)[0] / 15.0
        rows.append(f"| {t} | {m.BANDS[m.band_index(t)][2]} | {v / 8.0:.3f} | {sail:.1f} | {awa:.1f} | {lam:.2f} | "
                    f"{h8:.3f} | {h15:.3f} |")
    md = "\n".join(rows) + "\n"
    with open(os.path.join(RES, "polar.md"), "w", encoding="utf-8") as fh:
        fh.write(md)
    return md


def fig_trim():
    """İtki X(δs) üç seyirde (kararlı hızda görünür rüzgar): en iyi bomda tepe, bom AWA'ya açılınca itki sıfır."""
    U = 8.0
    fig, ax = plt.subplots(1, 1, figsize=(7.4, 4.2))
    sails = np.linspace(0, 90, 361)
    curves = [(45.0, "Orsa", "#2a78d6"), (90.0, "Tam apaz", "#eb6834"), (150.0, "Geniş apaz / pupa", "#1baf7a")]
    for twa, band, color in curves:
        v, opt = m.polar(twa, U)
        x = m.sail_force(twa, v, U, sails)[0]
        awa = float(m.apparent_wind(twa, v, U)[1])
        ax.plot(sails, x, color=color, lw=2, label=f"θ = {twa:.0f}° ({band}), v = {v:.1f} m/s, AWA = {awa:.0f}°")
        ax.plot([opt], [float(m.sail_force(twa, v, U, opt)[0])], "o", color=color, ms=8, mec="white", mew=2, zorder=5)
        if awa <= 90.0:
            ax.axvline(awa, color=color, lw=1, ls=":")
    ax.axhline(0.0, color="#52514e", lw=0.8)
    ax.text(1, ax.get_ylim()[1] * 0.92, "• en iyi bom (auto-trim)   ┆ bom = AWA → yapraklama", fontsize=8, color="#52514e")
    ax.set_xlim(0, 90)
    ax.set_xlabel("Bom açısı δs (°)")
    ax.set_ylabel("Yelken itkisi X (N)")
    ax.set_title(f"Yelken itkisi bom açısına göre (U = {U:.0f} m/s, kararlı hızdaki görünür rüzgar)", fontsize=10)
    ax.legend(loc="lower right", fontsize=8, frameon=False)
    _clean(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "sail_trim.png"), dpi=130)
    plt.close(fig)


def vmg_table():
    rows = ["| U (m/s) | Rüzgar üstü en iyi TWA | VMG_up (m/s) | Rüzgar altı en iyi TWA | VMG_down (m/s) | Tam pupa V (m/s) |",
            "|---|---|---|---|---|---|"]
    for U in (1.0, 3.0, 8.0, 15.0, 20.0):
        twa, v, _, lee = m.polar_table(U)
        ok = twa > m.NO_GO_DEG + 5.0
        cog = np.radians(twa[ok] + lee[ok])
        up, dn = v[ok] * np.cos(cog), -v[ok] * np.cos(cog)
        iu, idn = int(np.argmax(up)), int(np.argmax(dn))
        rows.append(f"| {U:.0f} | {twa[ok][iu]:.1f}° | {up[iu]:.2f} | {twa[ok][idn]:.1f}° | {dn[idn]:.2f} | {v[-1]:.2f} |")
    return "\n".join(rows)


def tack_threshold():
    """Açık çevrim tack: orsa (TWA 45°) kararlı hızından stall dümeniyle ölü açı momentumla geçiliyor mu
    (her tekne, 2–20 m/s) ve güdümün süzülme kestirimi (tack kapasitesi) ile karşılaştırma."""
    cfg = nav.NavConfig()
    winds = (2.0, 5.0, 8.0, 15.0, 20.0)
    rows = ["| Tekne | " + " | ".join(f"U={U:.0f}: geçti? / kapasite" for U in winds) + " |", "|---|" + "---|" * len(winds)]
    for name, phys in BOATS.items():
        cells = []
        for U in winds:
            r = m.simulate_open_loop(45, 0, -phys.rudder_stall_deg, U, 40.0, auto_trim=True, phys=phys)
            ok = any(180 < p < 330 for p in r["psi"])
            cap = nav.tack_capacity(r["v"][0], phys, U)
            cells.append(f"{'✔' if ok else '✘'} / {cap:.0f}°")
        rows.append(f"| {name} | " + " | ".join(cells) + " |")
    rows.append(f"\nGüdümün tack koşulu: kapasite ≥ {nav.tack_need(45.0, cfg):.1f}° "
                f"(ölü açı 60° + 5° pay, × {cfg.tack_safety}). Kapasite, çırpınan yelkenin en kötü durum rüzgar frenini "
                "ve stall dümeninin sürüklemesini içerir; karar ayrıca manevranın ileri simülasyonuyla doğrulanır.")
    return "\n".join(rows), None


def boat_size_table():
    """Tekne boyutu ve kütle bağımsızlığı: aynı senaryolar 5 teknede (geometrik ölçek ve kütle)."""
    md = ["| Tekne | m (kg) | L (m) | V/U tam apaz | S1 tam apaz 8 m/s: süre (s) | S4 rüzgar üstü 8 m/s: süre / tack / gybe | "
          "S4 15 m/s: süre / tack / gybe | PID 090°, 15 m/s, 30 m sapma: aşım / RMS (m) |", "|---|---|---|---|---|---|---|---|"]
    for name, phys in BOATS.items():
        cfg = nav.NavConfig(phys=phys)
        s1 = nav.navigate((0, 0), (1000, 0), "cloth", 8.0, cfg=cfg)
        s4 = nav.navigate((0, 0), (0, 1000), "cloth", 8.0, cfg=cfg)
        s4b = nav.navigate((0, 0), (0, 1000), "cloth", 15.0, cfg=cfg)
        ln = lf.simulate_line((0, 0), (600, 0), 15.0, replace(lf.LineConfig(), start_offset=30.0), phys)
        md.append(f"| {name} | {phys.mass:.0f} | {phys.hull_length:.2f} | {m.polar(90, 8.0, phys)[0] / 8.0:.3f} | "
                  f"{s1.time:.0f} | {s4.time:.0f} / {s4.n_tacks} / {s4.n_gybes} | {s4b.time:.0f} / {s4b.n_tacks} / "
                  f"{s4b.n_gybes} | {ln.metrics['overshoot']:.2f} / {ln.metrics['rms_e_second_half']:.2f} |")
    out = "\n".join(md) + "\n"
    with open(os.path.join(RES, "boat_size.md"), "w", encoding="utf-8") as fh:
        fh.write(out)
    return out


def scenario_matrix():
    rows = []
    for U in WINDS:
        for sname, st, tg in SCENARIOS:
            r = nav.navigate(st, tg, "cloth", U)
            vmg = r.straight_distance / r.time if r.reached and r.time > 0 else 0.0
            rows.append(dict(ruzgar=U, senaryo=sname, ulasti="EVET" if r.reached else "HAYIR",
                             sure_s=round(r.time, 1), yol_m=round(r.path_length, 1),
                             duz_mesafe_m=round(r.straight_distance, 1),
                             rota_verimi=round(r.straight_distance / r.path_length, 3) if r.path_length else 0,
                             ort_vmg=round(vmg, 2), tack=r.n_tacks, gybe=r.n_gybes, sonuc=r.reason))
    with open(os.path.join(RES, "scenario_matrix.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    md = ["| U (m/s) | Senaryo | Ulaştı | Süre (s) | Yol (m) | Düz mesafe (m) | Rota verimi | Ort. VMG (m/s) | Tack | Gybe |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        md.append(f"| {r['ruzgar']:.0f} | {r['senaryo']} | {r['ulasti']} | {r['sure_s']:.0f} | "
                  f"{r['yol_m']:.0f} | {r['duz_mesafe_m']:.0f} | {r['rota_verimi']:.2f} | {r['ort_vmg']:.2f} | "
                  f"{r['tack']} | {r['gybe']} |")
    with open(os.path.join(RES, "scenario_matrix.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    return rows


def edge_cases():
    cases = []

    def add(name, **kw):
        r = nav.navigate(**kw)
        cases.append(f"| {name} | {'EVET' if r.reached else 'HAYIR'} | {r.time:.0f} | {r.n_tacks} | {r.n_gybes} | {r.reason} |")

    add("Başlangıç pruvası ölü açıda (000°), hedef Doğu 1 km, 8 m/s",
        start=(0, 0), target=(1000, 0), profile="cloth", wind_ms=8.0, initial_heading=0.0)
    add("Başlangıç pruvası ölü açı sınırında (032°), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=8.0, initial_heading=32.0)
    add("Başlangıç pruvası ters yönde (180°), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=8.0, initial_heading=180.0)
    add("Tack yasak (yalnızca gybe), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=8.0, cfg=nav.NavConfig(allow_tack=False))
    add("Hafif rüzgar 1 m/s, hedef Kuzey 1 km",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=1.0)
    add("Kuvvetli rüzgar 20 m/s, hedef Kuzey 1 km",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=20.0)
    add("Hedef başlangıçta (mesafe 0)", start=(0, 0), target=(0, 0), profile="cloth", wind_ms=8.0)
    add("Uzun menzil: Kuzey 10 km, 5 m/s", start=(0, 0), target=(0, 10000), profile="cloth", wind_ms=5.0,
        cfg=nav.NavConfig(max_time=6 * 3600))
    add("Kısa süre limiti: Kuzey 1 km, 3 m/s, 300 s", start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=3.0,
        cfg=nav.NavConfig(max_time=300))
    md = ["| Durum | Ulaştı | Süre (s) | Tack | Gybe | Sonuç |", "|---|---|---|---|---|---|"] + cases
    with open(os.path.join(RES, "edge_cases.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    return "\n".join(md)


def gybe_only_sweep():
    """Tack yasakken (yalnız rüzgar altından dönüş) varış yarıçapı – rüzgar duyarlılığı."""
    radii = (5, 10, 20, 30)
    winds = (3.0, 5.0, 8.0, 12.0, 15.0)
    cfg0 = nav.NavConfig()
    lines = ["| Varış yarıçapı (m) | " + " | ".join(f"U={U:.0f} m/s" for U in winds) + " |",
             "|---|" + "---|" * len(winds)]
    diam = [2 * nav.turn_radius(m.polar(45, U)[0], cfg0) for U in winds]
    lines.append("| *Orsada dönüş çapı 2R (m)* | " + " | ".join(f"*{d:.0f}*" for d in diam) + " |")
    for rad in radii:
        cells = []
        for U in winds:
            r = nav.navigate((0, 0), (0, 1000), "cloth", U,
                             cfg=nav.NavConfig(allow_tack=False, arrival_radius=rad, max_time=3000))
            cells.append(f"✔ {r.time:.0f} s" if r.reached else "✘")
        lines.append(f"| {rad} | " + " | ".join(cells) + " |")
    md = "\n".join(lines)
    with open(os.path.join(RES, "gybe_only_sweep.md"), "w", encoding="utf-8") as f:
        f.write(md + "\n")
    return md


def _wind_arrow(ax):
    ax.annotate("", xy=(0.93, 0.80), xytext=(0.93, 0.95), xycoords="axes fraction",
                arrowprops=dict(arrowstyle="->", color="#0369a1", lw=2))


def fig_trajectories():
    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    for ax, (sname, st, tg) in zip(axes.ravel(), SCENARIOS):
        for U, color in ((3.0, "#86b6f6"), (8.0, "#2a78d6"), (15.0, "#0b3d91")):
            r = nav.navigate(st, tg, "cloth", U)
            ax.plot(r.x, r.y, color=color, lw=1.5, label=f"U={U:.0f}: {r.time:.0f} s, tack={r.n_tacks}, gybe={r.n_gybes}")
        ax.plot(*st, "ko", ms=6)
        ax.add_patch(plt.Circle(tg, 10, color="#16a34a", fill=False, lw=1.5))
        ax.plot(*tg, "*", color="#16a34a", ms=14)
        ax.set_title(sname, fontsize=10)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="best")
        _wind_arrow(ax)
    fig.suptitle("Navigasyon rotaları – 50 kg referans tekne, rüzgar kuzeyden (mavi ok)", fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "trajectories_U8.png"), dpi=120)
    plt.close(fig)

    # Hafif rüzgar: tack yerine rüzgar altından dönüş
    fig, axes = plt.subplots(1, 2, figsize=(11, 6))
    for ax, U in zip(axes, (1.0, 8.0)):
        r = nav.navigate((0, 0), (0, 1000), "cloth", U)
        sc = ax.scatter(r.x, r.y, c=r.v, s=3, cmap="viridis")
        fig.colorbar(sc, ax=ax, label="Hız (m/s)")
        ax.plot(0, 0, "ko")
        ax.plot(0, 1000, "*", color="#16a34a", ms=14)
        ax.set_title(f"Tam rüzgar üstü, U = {U:.0f} m/s\n{r.time:.0f} s, tack={r.n_tacks}, gybe={r.n_gybes}")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "upwind_light_vs_strong.png"), dpi=120)
    plt.close(fig)

    # Zaman serisi: pruva, yelken, dümen, hız, devrilme payı
    r = nav.navigate_route([(0, 0), (0, 1000)], 8.0)
    fig, axes = plt.subplots(5, 1, figsize=(10, 9.5), sharex=True)
    axes[0].plot(r.t, r.psi, label="ψ (gerçekleşen)")
    axes[0].plot(r.t, r.psi_des, "--", label="ψ* (atanan)")
    axes[0].set_ylabel("Pruva (°)")
    axes[0].legend(fontsize=8)
    axes[1].plot(r.t, r.sail, color="#2563eb")
    axes[1].set_ylabel("Bom δs (°)\n+ sancağa")
    axes[2].plot(r.t, r.rudder, color="#dc2626", label="gerçek")
    axes[2].plot(r.t, r.rudder_cmd, color="#dc2626", lw=0.8, alpha=0.5, label="komut")
    axes[2].set_ylabel("Dümen δr (°)")
    axes[2].legend(fontsize=8)
    axes[3].plot(r.t, r.v, color="#16a34a")
    axes[3].set_ylabel("Hız (m/s)")
    axes[4].plot(r.t, np.degrees(np.arctan2(r.v_lat, np.maximum(r.v, 0.3))), color="#7c3aed")
    axes[4].set_ylabel("Yan kayma λ (°)")
    axes[4].set_xlabel("Zaman (s)")
    for a in axes:
        a.grid(alpha=0.3)
    fig.suptitle("S4 tam rüzgar üstü – U = 8 m/s: pruva, yelken, dümen, hız ve yan kayma")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "timeseries_S4_U8.png"), dpi=120)
    plt.close(fig)


def _angle_rows_md(rows):
    md = ["| # | Zaman (s) | Bacak | Olay | ψ* (°) | TWA (°) | Seyir | Kontra | δs (°) | δr manevra (°) | δr seyir (°) | Ort. hız (m/s) | λ (°) |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for w in rows:
        md.append(f"| {w['no']} | {w['t0']:.0f}–{w['t1']:.0f} | {w['leg']} | {w['event']} | {w['psi_des']:.1f} | "
                  f"{w['twa']:.1f} | {w['regime']} | {w['tack']} | {w['sail']:+.0f} | {w['rudder_max']:+.1f} | "
                  f"{w['rudder_steady']:+.1f} | {w['v_mean']:.2f} | {w['leeway']:.1f} |")
    return "\n".join(md) + "\n"


def waypoint_route():
    """Ara duraklı (kare) rota ve açı takip tabloları."""
    pts = [(0, 0), (600, 0), (600, 600), (0, 600), (0, 0)]
    r = nav.navigate_route(pts, 8.0)
    rows = nav.angle_log(r)
    fig, ax = plt.subplots(figsize=(7, 7))
    sc = ax.scatter(r.x, r.y, c=r.v, s=3, cmap="viridis")
    fig.colorbar(sc, ax=ax, label="Hız (m/s)", shrink=0.8)
    px, py = zip(*pts)
    ax.plot(px, py, ":", color="#64748b")
    for i, (x, y) in enumerate(pts[1:-1], 1):
        ax.plot(x, y, "s", color="#7c3aed", ms=8)
        ax.text(x + 20, y + 20, str(i), color="#7c3aed", fontweight="bold")
    ax.plot(*pts[0], "ko", ms=7)
    ax.plot(*pts[-1], "*", color="#16a34a", ms=14)
    for row in rows:
        if row["event"] != "Rota düzeltme":
            k = int(np.searchsorted(r.t, row["t0"]))
            ax.annotate(f"#{row['no']}", (r.x[k], r.y[k]), fontsize=7, color="#b91c1c")
    ax.set_title(f"Kare rota B→1→2→3→H (U = 8 m/s): {r.time:.0f} s, tack={r.n_tacks}, gybe={r.n_gybes}", fontsize=10)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    _wind_arrow(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "waypoint_square_U8.png"), dpi=120)
    plt.close(fig)
    with open(os.path.join(RES, "angle_log_square_U8.md"), "w", encoding="utf-8") as f:
        f.write(_angle_rows_md(rows))
    with open(os.path.join(RES, "angle_log_S4_U8.md"), "w", encoding="utf-8") as f:
        f.write(_angle_rows_md(nav.angle_log(nav.navigate_route([(0, 0), (0, 1000)], 8.0))))
    return r


def reachability():
    bearings = np.arange(0, 360, 15)
    R = 1000.0
    winds = ((3.0, "#86b6f6"), (8.0, "#2a78d6"), (15.0, "#0b3d91"))
    data = {}
    for U, _ in winds:
        eta = []
        for b in bearings:
            tg = (R * np.sin(np.radians(b)), R * np.cos(np.radians(b)))
            r = nav.navigate((0, 0), tg, "cloth", U)
            eta.append(r.time if r.reached else np.nan)
        data[U] = np.array(eta)
    fig, ax = plt.subplots(subplot_kw=dict(projection="polar"), figsize=(6.6, 6.6))
    th = np.radians(np.append(bearings, 360))
    for U, color in winds:
        e = data[U]
        ax.plot(th, R / np.append(e, e[0]), color=color, lw=2, marker="o", ms=3, label=f"U = {U:.0f} m/s")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_title("Etkin VMG = 1000 m / varış süresi (m/s)", pad=16)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.15, -0.12), fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "reachability.png"), dpi=120)
    plt.close(fig)

    lines = ["| Kerteriz (°) | " + " | ".join(f"U={U:.0f} m/s ETA (s)" for U, _ in winds) + " |",
             "|---|" + "---|" * len(winds)]
    for i, b in enumerate(bearings):
        lines.append(f"| {b:03d} | " + " | ".join(
            (f"{data[U][i]:.0f}" if np.isfinite(data[U][i]) else "—") for U, _ in winds) + " |")
    total = sum(np.isfinite(v).sum() for v in data.values())
    with open(os.path.join(RES, "reachability.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return total, len(bearings) * len(winds)


def force_tables():
    """Kuvvet ve moment tabloları (U = 8 m/s, kararlı hız) ve kuvvet diyagramı."""
    U = 8.0
    out = [f"Parametreler: m = {P.mass:.0f} kg, L = {P.hull_length:.2f} m, A = {P.sail_area:.1f} m², S = {P.wetted_area:.2f} m², "
           f"C_h = {P.hull_cd:.4f}, A_s = {P.board_area:.2f} m², A_d = {P.rudder_area:.3f} m², e = {P.ce_offset:+.2f} m, "
           f"U = {U:.0f} m/s (kuvvetler U² ile, hızlar U ile ölçeklenir; açılar rüzgardan bağımsızdır)\n"]
    out += ["### Seyirlere göre kuvvetler (auto-trim, δr = 0, kararlı hız)\n",
            "| ψ (°) | Seyir | δs (°) | V* (m/s) | V_A (m/s) | AWA (°) | α (°) | C_L | C_D | X (N) | Y (N) | R (N) | R_i (N) | λ (°) | N_yelken (N·m) | δr denge (°) |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for h in (40.0, 60.0, 90.0, 120.0, 150.0, 180.0):
        f = m.force_moment_breakdown(h, 0, 0, U, auto_trim=True)
        name = m.BANDS[m.band_index(f["twa"])][2]
        out.append(f"| {h:.0f} | {name} | {f['sail']:.1f} | {f['v']:.2f} | {f['va']:.2f} | {f['awa']:.1f} | {f['alpha']:.1f} | "
                   f"{f['cl']:.2f} | {f['cd']:.2f} | {f['drive']:.1f} | {f['side']:.1f} | "
                   f"{f['hull_drag']:.1f} | {f['induced_drag']:.2f} | {f['leeway']:.2f} | {f['sail_moment']:.2f} | "
                   f"{f['rudder_balance']:+.2f} |")
    out += ["\n### Bom açısına göre (ψ = 90°, δr = 0, her bomda kararlı hız)\n",
            "| δs (°) | V* (m/s) | AWA (°) | α (°) | X (N) | Y (N) | Durum |", "|---|---|---|---|---|---|---|"]
    for sail in (10, 18, 26, 35, 45, 60):
        f = m.force_moment_breakdown(90, sail, 0, U)
        txt = m.trim_status(90, sail, f["v"], U)[1]
        out.append(f"| {sail} | {f['v']:.2f} | {f['awa']:.1f} | {f['alpha']:.1f} | {f['drive']:.1f} | {f['side']:.1f} | "
                   f"{txt} |")
    out += [f"\n### Dümen (ψ = 90°, auto-trim; stall açısı {P.rudder_stall_deg:.1f}°)\n",
            "| δr (°) | v (m/s) | D_dümen (N) | C_L | Y_dümen (N) | N_dümen (N·m) | N_net (N·m) | ψ̇ (°/s) | R (m) | F_merkezcil (N) |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for rud in (0, 5, 10, 20, 35):
        f = m.force_moment_breakdown(90, 0, rud, U, auto_trim=True)
        R = "∞" if not np.isfinite(f["turn_radius"]) or f["turn_radius"] > 1e3 else f"{f['turn_radius']:.1f}"
        out.append(f"| {rud:+d} | {f['v']:.2f} | {f['rudder_drag']:.1f} | {f['rudder_cl']:.2f} | {f['rudder_force']:.0f} | "
                   f"{f['rudder_moment']:.0f} | {f['net_moment']:.0f} | {f['yaw_rate']:+.1f} | {R} | {f['centripetal']:.0f} |")
    md = "\n".join(out) + "\n"
    with open(os.path.join(RES, "forces.md"), "w", encoding="utf-8") as fh:
        fh.write(md)

    # Kuvvet diyagramı: tam apaz, auto-trim, δr = +10° (tekne doğuya gidiyor; gövde ekseninde, pruva yukarı)
    rud = 10.0
    f = m.force_moment_breakdown(90, 0, rud, U, auto_trim=True)
    fig, ax = plt.subplots(figsize=(7.4, 7.4))
    for cx in (-0.6, 0.6):
        ax.add_patch(plt.Polygon([(cx, 1.25), (cx + .12, 0.75), (cx + .12, -0.95), (cx + .06, -1.25), (cx - .06, -1.25),
                                  (cx - .12, -0.95), (cx - .12, 0.75)], color="#1e293b"))
    for by in (0.6, -0.7):
        ax.plot([-0.6, 0.6], [by, by], color="#475569", lw=4)
    d = np.radians(f["sail"])
    mast = np.array([0.0, 0.3])
    boom = mast + 1.2 * np.array([np.sin(d), -np.cos(d)])
    ax.plot(*zip(mast, boom), color="#2563eb", lw=4)
    ce = mast + 0.5 * (boom - mast)
    sc = 2.0 / max(abs(f["side"]), abs(f["drive"]), 1.0)

    def arrow(p, vec, color, text, off=(0.08, 0.08)):
        ax.annotate("", xy=(p[0] + vec[0], p[1] + vec[1]), xytext=p,
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=2.4, mutation_scale=16))
        ax.text(p[0] + vec[0] + off[0], p[1] + vec[1] + off[1], text, color=color, fontsize=9, fontweight="bold")

    arrow(ce, (0, f["drive"] * sc), "#16a34a", f"X = {f['drive']:.0f} N", off=(-1.1, -0.3))
    arrow(ce, (f["side"] * sc, 0), "#d97706", f"Y = {f['side']:.0f} N", off=(-0.9, -0.3))
    arrow(ce, (f["side"] * sc, f["drive"] * sc), "#7c3aed", f"F_yelken = {f['sail_force']:.0f} N", off=(0.05, 0.05))
    res = f["hull_drag"] + f["induced_drag"] + f["rudder_drag"]
    arrow((0, -1.4), (0, -res * sc), "#64748b", f"R + R_i + D_d = {res:.0f} N", off=(0.1, 0))
    for cx in (-0.6, 0.6):
        hinge = np.array([cx, -1.25])
        r = np.radians(rud)
        ax.plot(*zip(hinge, hinge + 0.4 * np.array([np.sin(r), -np.cos(r)])), color="#dc2626", lw=4)
    arrow((0.6, -1.9), (-f["rudder_force"] * sc * 0.25, 0), "#dc2626",
          f"Y_dümen = {f['rudder_force']:.0f} N (ok ¼ ölçek)\nN_dümen = Y·x_d = {f['rudder_moment']:.0f} N·m", off=(-2.6, -0.6))
    for wy in (0.2, 0.6, 1.0):                               # rüzgar kuzeyden = iskele bordadan (gövde ekseninde soldan)
        ax.annotate("", xy=(-1.5, wy), xytext=(-2.8, wy), arrowprops=dict(arrowstyle="->", color="#0284c7", lw=2.2))
    ax.text(-2.15, 1.35, f"gerçek rüzgar U = {U:.0f} m/s\n(iskele bordadan, TWA 90°)\nAWA = {f['awa']:.0f}°",
            ha="center", fontsize=8, color="#0369a1")
    ax.text(0, 1.6, "PRUVA (ψ = 090°)", ha="center", fontsize=9, fontweight="bold")
    ax.set_title(f"Kuvvet diyagramı – tam apaz, auto-trim δs = {f['sail']:.0f}°, δr = +{rud:.0f}°, v = {f['v']:.2f} m/s\n"
                 f"Net moment {f['net_moment']:.0f} N·m → ψ̇ = {f['yaw_rate']:.1f} °/s, R = {f['turn_radius']:.1f} m",
                 fontsize=10)
    ax.set_xlim(-3.2, 3.2)
    ax.set_ylim(-3.6, 2.2)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "force_diagram.png"), dpi=130)
    plt.close(fig)
    return md


LINE_SCEN = [  # (kerteriz, rüzgar, başlangıç sapması, pruva hatası)
    (45, 8, 30, 0), (315, 8, -30, 0), (60, 3, 30, 25), (300, 15, -30, -25),
    (90, 8, 30, 0), (270, 15, 30, 25), (120, 3, -30, 0), (240, 8, 30, -25),
    (150, 15, 30, 0), (210, 3, -30, 25), (170, 8, -30, 0), (190, 15, 30, -25),
]


def line_follow_table():
    """Düz rota (kaskad PID) başarımı: kazanç ayarında kullanılan 12 senaryo, 600 m çizgi."""
    md = ["| Kerteriz (°) | U (m/s) | Sapma (m) | Pruva hatası (°) | Sonuç | Süre (s) | Aşım (m) | Oturma (s) | RMS son yarı (m) | Dümen hareketi (°) | En büyük λ (°) |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for brg, U, off, herr in LINE_SCEN:
        b = (600 * np.sin(np.radians(brg)), 600 * np.cos(np.radians(brg)))
        r = lf.simulate_line((0, 0), b, float(U), replace(lf.LineConfig(), start_offset=off, start_heading_err=herr))
        mt = r.metrics
        st = "–" if mt["settle_time"] is None else f"{mt['settle_time']:.0f}"
        md.append(f"| {brg:03d} | {U} | {off:+d} | {herr:+d} | {'B ✔' if r.status == 'reached' else r.status} | "
                  f"{mt['time']:.0f} | {mt['overshoot']:.2f} | {st} | {mt['rms_e_second_half']:.2f} | "
                  f"{mt['rudder_travel']:.0f} | {mt['max_leeway']:.1f} |")
    out = "\n".join(md) + "\n"
    with open(os.path.join(RES, "line_follow.md"), "w", encoding="utf-8") as fh:
        fh.write(out)
    return out


def main():
    fig_polar()
    pm = polar_table_md()
    fig_trim()
    vt = vmg_table()
    tt, _ = tack_threshold()
    rows = scenario_matrix()
    ec = edge_cases()
    gs = gybe_only_sweep()
    fig_trajectories()
    waypoint_route()
    force_tables()
    lt = line_follow_table()
    bs = boat_size_table()
    ok, n = reachability()
    with open(os.path.join(RES, "summary.md"), "w", encoding="utf-8") as f:
        f.write("### Hız poları\n\n" + pm + "\n### VMG optimumları\n\n" + vt + "\n\n### Tack (açık çevrim)\n\n" + tt
                + "\n\n### Uç durumlar\n\n" + ec
                + "\n\n### Tack yasak – varış yarıçapı duyarlılığı (Kuzey 1 km)\n\n" + gs + "\n")
        f.write(f"\n### Senaryo matrisi: {sum(r['ulasti'] == 'EVET' for r in rows)}/{len(rows)} başarılı\n")
        f.write(f"\n### Erişilebilirlik: {ok}/{n} hedef başarılı\n")
        f.write("\n### Düz rota (PID)\n\n" + lt)
        f.write("\n### Tekne boyutu ve kütle bağımsızlığı\n\n" + bs)
    print(open(os.path.join(RES, "summary.md"), encoding="utf-8").read())


if __name__ == "__main__":
    main()
