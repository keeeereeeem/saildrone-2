"""
Rapor deneyleri: polar/verim grafikleri, navigasyon senaryo matrisi, rota çizimleri,
erişilebilirlik (ETA) analizi ve tack eşik analizi.

Çıktılar: docs/figures/*.png, docs/results/*.md|csv
Çalıştırma:  python analysis/run_experiments.py
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import navigation as nav  # noqa: E402
import saildrone_model as m  # noqa: E402

FIG = os.path.join(ROOT, "docs", "figures")
RES = os.path.join(ROOT, "docs", "results")
os.makedirs(FIG, exist_ok=True)
os.makedirs(RES, exist_ok=True)

PROFILES = {"0012": "NACA 0012", "0015": "NACA 0015", "cloth": "Kumaş Yelken"}
COLORS = {"0012": "#0284c7", "0015": "#4338ca", "cloth": "#d97706"}
WINDS = [1.0, 3.0, 8.0, 15.0]
SCENARIOS = [
    ("S1 Tam apaz (Doğu)", (0, 0), (1000, 0)),
    ("S2 Rüzgar altı (Güney)", (0, 0), (0, -1000)),
    ("S3 Kuzeydoğu 45°", (0, 0), (700, 700)),
    ("S4 Tam rüzgar üstü (Kuzey)", (0, 0), (0, 1000)),
    ("S5 Ölü açı içi (017°)", (0, 0), (300, 1000)),
    ("S6 Ofset başlangıç (KB)", (500, -300), (-400, 800)),
]


def fig_polar():
    twa = np.linspace(0, 180, 721)
    fig, ax = plt.subplots(subplot_kw=dict(projection="polar"), figsize=(6.4, 6.4))
    for k, name in PROFILES.items():
        r = [m.polar_point(t, k)[1] for t in twa]
        th = np.radians(twa)
        ax.plot(th, r, color=COLORS[k], lw=2, label=name)
        ax.plot(-th, r, color=COLORS[k], lw=2)
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.fill_between(np.radians(np.linspace(-30, 30, 50)), 0, 1.4, color="#ef4444", alpha=0.15)
    ax.set_rlim(0, 1.4)
    ax.set_title("Hız poları  V_b / V_w  (rüzgar kuzeyden)", pad=18)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.1, -0.1), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "polar.png"), dpi=130)
    plt.close(fig)


def fig_trim():
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.4), sharey=True)
    d = np.linspace(-60, 60, 481)
    for ax, (k, name) in zip(axes, PROFILES.items()):
        opt = 45.0
        eta = [m.trim_efficiency(opt + x, opt, k)[0] for x in d]
        ax.plot(d, eta, color=COLORS[k], lw=2)
        tol = m.TRIM[k]["tol"]
        ax.axvspan(-tol, tol, color="#22c55e", alpha=0.15)
        ax.set_title(name)
        ax.set_xlabel("δ_s − δ_opt (°)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("Trim verimi η")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "trim_efficiency.png"), dpi=130)
    plt.close(fig)


def vmg_table():
    rows = ["| Profil | Optimum rüzgar üstü TWA | VMG_up / V_w | Optimum rüzgar altı TWA | VMG_down / V_w | Tam pupa VMG / V_w |",
            "|---|---|---|---|---|---|"]
    twa = np.arange(30.5, 180.01, 0.5)
    for k, name in PROFILES.items():
        r = np.array([m.polar_point(t, k)[1] for t in twa])
        up = r * np.cos(np.radians(twa))
        dn = -r * np.cos(np.radians(twa))
        iu, idn = int(np.argmax(up)), int(np.argmax(dn))
        rows.append(f"| {name} | {twa[iu]:.1f}° | {up[iu]:.3f} | {twa[idn]:.1f}° | {dn[idn]:.3f} | {r[-1]:.3f} |")
    return "\n".join(rows)


def tack_threshold():
    """Açık çevrim: close-hauled (TWA 35°) + tam dümen ile ölü açıyı geçebilmek için gereken asgari rüzgar."""
    rows = ["| Profil | Asgari rüzgar (m/s) | Tack başlangıç hızı (m/s) | Teorik v_min (m/s) |", "|---|---|---|---|"]
    v_theory = np.radians(2 * m.NO_GO_DEG + 5) * m.L_HULL / (m.TAU_SURGE * np.sin(np.radians(m.RUDDER_MAX_DEG)))
    out = {}
    for k, name in PROFILES.items():
        found = None
        for U in np.arange(0.5, 15.01, 0.1):
            r = m.simulate_open_loop(35, 0, -35, k, U, 40.0, auto_trim=True)
            # 35°'den sola dönerek 325°'nin ötesine (TWA>30, sancak kontra) geçti mi?
            passed = any(180 < p < 330 for p in r["psi"])
            if passed:
                found = U
                break
        v0 = m.target_speed(35, 0, -35, k, found, True) if found else float("nan")
        out[k] = found
        rows.append(f"| {name} | {found:.1f} | {v0:.2f} | {v_theory:.2f} |")
    return "\n".join(rows), out


def scenario_matrix():
    rows = []
    for k, name in PROFILES.items():
        for U in WINDS:
            for sname, st, tg in SCENARIOS:
                r = nav.navigate(st, tg, k, U)
                vmg = r.straight_distance / r.time if r.reached and r.time > 0 else 0.0
                rows.append(dict(profil=name, ruzgar=U, senaryo=sname, ulasti="EVET" if r.reached else "HAYIR",
                                 sure_s=round(r.time, 1), yol_m=round(r.path_length, 1),
                                 duz_mesafe_m=round(r.straight_distance, 1),
                                 rota_verimi=round(r.straight_distance / r.path_length, 3) if r.path_length else 0,
                                 ort_vmg=round(vmg, 2), tack=r.n_tacks, gybe=r.n_gybes, sonuc=r.reason))
    with open(os.path.join(RES, "scenario_matrix.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    md = ["| Profil | U (m/s) | Senaryo | Ulaştı | Süre (s) | Yol (m) | Düz mesafe (m) | Rota verimi | Ort. VMG (m/s) | Tack | Gybe |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        md.append(f"| {r['profil']} | {r['ruzgar']:.0f} | {r['senaryo']} | {r['ulasti']} | {r['sure_s']:.0f} | "
                  f"{r['yol_m']:.0f} | {r['duz_mesafe_m']:.0f} | {r['rota_verimi']:.2f} | {r['ort_vmg']:.2f} | "
                  f"{r['tack']} | {r['gybe']} |")
    with open(os.path.join(RES, "scenario_matrix.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    return rows


def edge_cases():
    cases = []
    cfg = nav.NavConfig()

    def add(name, **kw):
        r = nav.navigate(**kw)
        cases.append(f"| {name} | {'EVET' if r.reached else 'HAYIR'} | {r.time:.0f} | {r.n_tacks} | {r.n_gybes} | {r.reason} |")

    add("Başlangıç pruvası ölü açıda (000°), hedef Doğu 1 km, 8 m/s",
        start=(0, 0), target=(1000, 0), profile="0012", wind_ms=8.0, initial_heading=0.0)
    add("Başlangıç pruvası ölü açı sınırında (032°), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="0012", wind_ms=8.0, initial_heading=32.0)
    add("Başlangıç pruvası ters yönde (180°), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="0012", wind_ms=8.0, initial_heading=180.0)
    add("Tack yasak (yalnızca gybe), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="0012", wind_ms=8.0, cfg=nav.NavConfig(allow_tack=False))
    add("Tack yasak, varış yarıçapı 30 m, hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="0012", wind_ms=8.0,
        cfg=nav.NavConfig(allow_tack=False, arrival_radius=30.0))
    add("Hafif rüzgar 2 m/s (tack fiziksel olarak imkânsız), hedef Kuzey 1 km",
        start=(0, 0), target=(0, 1000), profile="0012", wind_ms=2.0)
    add("Hedef başlangıçta (mesafe 0)", start=(0, 0), target=(0, 0), profile="0012", wind_ms=8.0)
    add("Uzun menzil: Kuzey 10 km, 5 m/s, kumaş", start=(0, 0), target=(0, 10000), profile="cloth", wind_ms=5.0,
        cfg=nav.NavConfig(max_time=6 * 3600))
    add("Kısa süre limiti: Kuzey 1 km, 3 m/s, 300 s", start=(0, 0), target=(0, 1000), profile="0012", wind_ms=3.0,
        cfg=nav.NavConfig(max_time=300))
    md = ["| Durum | Ulaştı | Süre (s) | Tack | Gybe | Sonuç |", "|---|---|---|---|---|---|"] + cases
    with open(os.path.join(RES, "edge_cases.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    return "\n".join(md)


def gybe_only_sweep():
    """Tack yasakken (yalnız rüzgar altından dönüş) varış yarıçapı – rüzgar duyarlılığı."""
    radii = (10, 15, 20, 25, 30, 40)
    winds = (3.0, 5.0, 8.0, 12.0, 15.0)
    cfg0 = nav.NavConfig()
    lines = ["| Varış yarıçapı (m) | " + " | ".join(f"U={U:.0f} m/s" for U in winds) + " |",
             "|---|" + "---|" * len(winds)]
    diam = []
    for U in winds:
        v = U * m.POLAR["0012"][0][1]
        w = min(np.radians(cfg0.max_turn_rate), v / m.L_HULL * np.sin(np.radians(m.RUDDER_MAX_DEG)))
        diam.append(2 * v / w)
    lines.append("| *Dönüş çapı 2R (m)* | " + " | ".join(f"*{d:.0f}*" for d in diam) + " |")
    for rad in radii:
        cells = []
        for U in winds:
            r = nav.navigate((0, 0), (0, 1000), "0012", U,
                             cfg=nav.NavConfig(allow_tack=False, arrival_radius=rad, max_time=3000))
            cells.append(f"✔ {r.time:.0f} s" if r.reached else "✘")
        lines.append(f"| {rad} | " + " | ".join(cells) + " |")
    md = "\n".join(lines)
    with open(os.path.join(RES, "gybe_only_sweep.md"), "w", encoding="utf-8") as f:
        f.write(md + "\n")
    return md


def fig_trajectories():
    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    for ax, (sname, st, tg) in zip(axes.ravel(), SCENARIOS):
        for k, name in PROFILES.items():
            r = nav.navigate(st, tg, k, 8.0)
            ax.plot(r.x, r.y, color=COLORS[k], lw=1.6, label=f"{name}: {r.time:.0f} s")
        ax.plot(*st, "ko", ms=6)
        ax.add_patch(plt.Circle(tg, 10, color="#16a34a", fill=False, lw=1.5))
        ax.plot(*tg, "*", color="#16a34a", ms=14)
        ax.set_title(sname + "  (U = 8 m/s)", fontsize=10)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="best")
        ax.annotate("", xy=(0.93, 0.80), xytext=(0.93, 0.95), xycoords="axes fraction",
                    arrowprops=dict(arrowstyle="->", color="#0369a1", lw=2))
    fig.suptitle("Navigasyon rotaları – rüzgar kuzeyden (mavi ok)", fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "trajectories_U8.png"), dpi=120)
    plt.close(fig)

    # Hafif rüzgar: tack yerine gybe ile kontra değiştirme
    fig, axes = plt.subplots(1, 2, figsize=(11, 6))
    for ax, U in zip(axes, (1.0, 8.0)):
        r = nav.navigate((0, 0), (0, 1000), "0012", U)
        sc = ax.scatter(r.x, r.y, c=r.v, s=3, cmap="viridis")
        fig.colorbar(sc, ax=ax, label="Hız (m/s)")
        ax.plot(0, 0, "ko")
        ax.plot(0, 1000, "*", color="#16a34a", ms=14)
        ax.set_title(f"NACA 0012, tam rüzgar üstü, U={U:.0f} m/s\n{r.time:.0f} s, tack={r.n_tacks}, gybe={r.n_gybes}")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "upwind_light_vs_strong.png"), dpi=120)
    plt.close(fig)

    # Zaman serisi
    r = nav.navigate((0, 0), (0, 1000), "0012", 8.0)
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(r.t, r.psi, label="ψ")
    axes[0].plot(r.t, r.psi_des, "--", label="ψ*")
    axes[0].set_ylabel("Pruva (°)")
    axes[0].legend(fontsize=8)
    axes[1].plot(r.t, r.v, color="#16a34a")
    axes[1].set_ylabel("Hız (m/s)")
    axes[2].plot(r.t, r.rudder, color="#dc2626")
    axes[2].set_ylabel("Dümen (°)")
    axes[2].set_xlabel("Zaman (s)")
    for a in axes:
        a.grid(alpha=0.3)
    fig.suptitle("S4 tam rüzgar üstü – NACA 0012 – U = 8 m/s zaman serileri")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "timeseries_S4_U8.png"), dpi=120)
    plt.close(fig)


def reachability():
    bearings = np.arange(0, 360, 15)
    R = 1000.0
    data = {}
    for k in PROFILES:
        for U in (3.0, 8.0):
            eta = []
            for b in bearings:
                tg = (R * np.sin(np.radians(b)), R * np.cos(np.radians(b)))
                r = nav.navigate((0, 0), tg, k, U)
                eta.append(r.time if r.reached else np.nan)
            data[(k, U)] = np.array(eta)
    fig, axes = plt.subplots(1, 2, subplot_kw=dict(projection="polar"), figsize=(12, 6))
    th = np.radians(np.append(bearings, 360))
    for ax, U in zip(axes, (3.0, 8.0)):
        for k, name in PROFILES.items():
            e = data[(k, U)]
            ax.plot(th, R / np.append(e, e[0]), color=COLORS[k], lw=2, marker="o", ms=3, label=name)
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_title(f"Etkin VMG = 1000 m / ETA  (U = {U:.0f} m/s)", pad=16)
        ax.legend(loc="lower left", bbox_to_anchor=(-0.15, -0.12), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "reachability.png"), dpi=120)
    plt.close(fig)

    lines = ["| Kerteriz (°) | " + " | ".join(f"{PROFILES[k]} U={U:.0f}" for k in PROFILES for U in (3.0, 8.0)) + " |",
             "|---|" + "---|" * 6]
    for i, b in enumerate(bearings):
        lines.append(f"| {b:03d} | " + " | ".join(
            (f"{data[(k, U)][i]:.0f}" if np.isfinite(data[(k, U)][i]) else "—") for k in PROFILES for U in (3.0, 8.0)) + " |")
    total = sum(np.isfinite(v).sum() for v in data.values())
    with open(os.path.join(RES, "reachability.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return total, len(bearings) * len(data)


def main():
    fig_polar()
    fig_trim()
    vt = vmg_table()
    tt, _ = tack_threshold()
    rows = scenario_matrix()
    ec = edge_cases()
    gs = gybe_only_sweep()
    fig_trajectories()
    ok, n = reachability()
    with open(os.path.join(RES, "summary.md"), "w", encoding="utf-8") as f:
        f.write("### VMG optimumları\n\n" + vt + "\n\n### Tack eşiği\n\n" + tt + "\n\n### Uç durumlar\n\n" + ec
                + "\n\n### Tack yasak – varış yarıçapı duyarlılığı (NACA 0012, Kuzey 1 km)\n\n" + gs + "\n")
        f.write(f"\n### Senaryo matrisi: {sum(r['ulasti'] == 'EVET' for r in rows)}/{len(rows)} başarılı\n")
        f.write(f"\n### Erişilebilirlik: {ok}/{n} hedef başarılı\n")
    print(open(os.path.join(RES, "summary.md"), encoding="utf-8").read())


if __name__ == "__main__":
    main()
