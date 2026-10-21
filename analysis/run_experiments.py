"""
Rapor deneyleri (tek yelken modeli: kumaş yelken): polar/verim grafikleri, navigasyon senaryo
matrisi, rota çizimleri, ara duraklı rota ve açı takip tablosu, erişilebilirlik (ETA) analizi,
tack eşik analizi.

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

PROFILES = {"cloth": "Kumaş Yelken"}
COLORS = {"cloth": "#d97706"}
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
    for (lo, hi, name, _), (opt, ratio) in zip(m.BANDS, m.POLAR["cloth"]):
        mid = np.radians((lo + hi) / 2)
        ax.text(mid, ratio + 0.15, f"{name}\n{ratio:.2f}× / δ={opt:.0f}°", fontsize=7, ha="center")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.fill_between(np.radians(np.linspace(-30, 30, 50)), 0, 1.4, color="#ef4444", alpha=0.15)
    ax.set_rlim(0, 1.4)
    ax.set_rlabel_position(200)
    ax.set_title("Kumaş yelken hız poları  V_b / U  (rüzgar kuzeyden)", pad=18)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "polar.png"), dpi=130)
    plt.close(fig)


def fig_trim():
    fig, ax0 = plt.subplots(1, 1, figsize=(6.4, 3.6))
    axes = [ax0]
    d = np.linspace(-60, 60, 481)
    for ax, (k, name) in zip(axes, PROFILES.items()):
        opt = 45.0
        eta = [m.trim_efficiency(opt + x, opt, k)[0] for x in d]
        ax.plot(d, eta, color=COLORS[k], lw=2)
        tol = m.TRIM[k]["tol"]
        ax.axvspan(-tol, tol, color="#22c55e", alpha=0.15)
        ax.set_title(name + " – trim verimi")
        ax.text(-55, 0.2, "fazla kasılı\n(düz / stall)", fontsize=8)
        ax.text(25, 0.2, "fazla açık\n(yapraklama)", fontsize=8)
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
        start=(0, 0), target=(1000, 0), profile="cloth", wind_ms=8.0, initial_heading=0.0)
    add("Başlangıç pruvası ölü açı sınırında (032°), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=8.0, initial_heading=32.0)
    add("Başlangıç pruvası ters yönde (180°), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=8.0, initial_heading=180.0)
    add("Tack yasak (yalnızca gybe), hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=8.0, cfg=nav.NavConfig(allow_tack=False))
    add("Tack yasak, varış yarıçapı 30 m, hedef Kuzey 1 km, 8 m/s",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=8.0,
        cfg=nav.NavConfig(allow_tack=False, arrival_radius=30.0))
    add("Hafif rüzgar 2 m/s (tack fiziksel olarak imkânsız), hedef Kuzey 1 km",
        start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=2.0)
    add("Hedef başlangıçta (mesafe 0)", start=(0, 0), target=(0, 0), profile="cloth", wind_ms=8.0)
    add("Uzun menzil: Kuzey 10 km, 5 m/s, kumaş", start=(0, 0), target=(0, 10000), profile="cloth", wind_ms=5.0,
        cfg=nav.NavConfig(max_time=6 * 3600))
    add("Kısa süre limiti: Kuzey 1 km, 3 m/s, 300 s", start=(0, 0), target=(0, 1000), profile="cloth", wind_ms=3.0,
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
        v = U * m.POLAR["cloth"][0][1]
        w = min(np.radians(cfg0.max_turn_rate), v / m.L_HULL * np.sin(np.radians(m.RUDDER_MAX_DEG)))
        diam.append(2 * v / w)
    lines.append("| *Dönüş çapı 2R (m)* | " + " | ".join(f"*{d:.0f}*" for d in diam) + " |")
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


def fig_trajectories():
    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    for ax, (sname, st, tg) in zip(axes.ravel(), SCENARIOS):
        for k, name in PROFILES.items():
            r = nav.navigate(st, tg, k, 8.0)
            ax.plot(r.x, r.y, color=COLORS[k], lw=1.6, label=f"{r.time:.0f} s, tack={r.n_tacks}, gybe={r.n_gybes}")
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
        r = nav.navigate((0, 0), (0, 1000), "cloth", U)
        sc = ax.scatter(r.x, r.y, c=r.v, s=3, cmap="viridis")
        fig.colorbar(sc, ax=ax, label="Hız (m/s)")
        ax.plot(0, 0, "ko")
        ax.plot(0, 1000, "*", color="#16a34a", ms=14)
        ax.set_title(f"Kumaş yelken, tam rüzgar üstü, U={U:.0f} m/s\n{r.time:.0f} s, tack={r.n_tacks}, gybe={r.n_gybes}")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "upwind_light_vs_strong.png"), dpi=120)
    plt.close(fig)

    # Zaman serisi: pruva, yelken, dümen, hız
    r = nav.navigate_route([(0, 0), (0, 1000)], 8.0)
    fig, axes = plt.subplots(4, 1, figsize=(10, 8.5), sharex=True)
    axes[0].plot(r.t, r.psi, label="ψ (gerçekleşen)")
    axes[0].plot(r.t, r.psi_des, "--", label="ψ* (atanan)")
    axes[0].set_ylabel("Pruva (°)")
    axes[0].legend(fontsize=8)
    axes[1].plot(r.t, r.sail, color="#2563eb")
    axes[1].set_ylabel("Yelken δ_s (°)\n+ sancağa")
    axes[2].plot(r.t, r.rudder, color="#dc2626")
    axes[2].set_ylabel("Dümen δ_r (°)")
    axes[3].plot(r.t, r.v, color="#16a34a")
    axes[3].set_ylabel("Hız (m/s)")
    axes[3].set_xlabel("Zaman (s)")
    for a in axes:
        a.grid(alpha=0.3)
    fig.suptitle("S4 tam rüzgar üstü – kumaş yelken – U = 8 m/s: pruva, yelken, dümen ve hız")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "timeseries_S4_U8.png"), dpi=120)
    plt.close(fig)


def _angle_rows_md(rows):
    md = ["| # | Zaman (s) | Bacak | Olay | ψ* (°) | TWA (°) | Seyir | Kontra | δ_s (°) | δ_r manevra (°) | δ_r seyir (°) | Ort. hız (m/s) |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for w in rows:
        md.append(f"| {w['no']} | {w['t0']:.0f}–{w['t1']:.0f} | {w['leg']} | {w['event']} | {w['psi_des']:.1f} | "
                  f"{w['twa']:.1f} | {w['regime']} | {w['tack']} | {w['sail']:+.0f} | {w['rudder_max']:+.1f} | "
                  f"{w['rudder_steady']:+.1f} | {w['v_mean']:.2f} |")
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
    ax.annotate("", xy=(0.93, 0.80), xytext=(0.93, 0.95), xycoords="axes fraction",
                arrowprops=dict(arrowstyle="->", color="#0369a1", lw=2))
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "waypoint_square_U8.png"), dpi=120)
    plt.close(fig)
    with open(os.path.join(RES, "angle_log_square_U8.md"), "w", encoding="utf-8") as f:
        f.write(_angle_rows_md(rows))
    with open(os.path.join(RES, "angle_log_S4_U8.md"), "w", encoding="utf-8") as f:
        f.write(_angle_rows_md(nav.angle_log(nav.navigate_route([(0, 0), (0, 1000)], 8.0))))


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
             "|---|" + "---|" * len(data)]
    for i, b in enumerate(bearings):
        lines.append(f"| {b:03d} | " + " | ".join(
            (f"{data[(k, U)][i]:.0f}" if np.isfinite(data[(k, U)][i]) else "—") for k in PROFILES for U in (3.0, 8.0)) + " |")
    total = sum(np.isfinite(v).sum() for v in data.values())
    with open(os.path.join(RES, "reachability.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return total, len(bearings) * len(data)


def force_tables():
    """Kuvvet ve moment tabloları (U = 8 m/s, kararlı hız) ve kuvvet diyagramı."""
    U = 8.0
    P = m.PHYS
    out = [f"Parametreler: m = {P.mass:.0f} kg, A_yelken = {P.sail_area:.0f} m², A_dümen = {P.rudder_area:.2f} m², "
           f"C_Lα = {P.rudder_cla:.2f} 1/rad, x_d = {P.rudder_arm:.1f} m, e = {P.ce_offset:.2f} m, τ = {m.TAU_SURGE:.0f} s, U = {U:.0f} m/s\n"]
    out += ["### Seyir bantlarına göre yelken kuvvetleri (auto-trim, δr = 0)\n",
            "| ψ (°) | Seyir | δs (°) | V* (m/s) | V_A (m/s) | AWA (°) | T itki (N) | F_N (N) | F_yan (N) | C_N | N_yelken (N·m) | δr denge (°) |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for h in (35.5, 50.5, 90.0, 135.0, 165.0):
        f = m.force_moment_breakdown(h, 0, 0, U, auto_trim=True)
        name = m.BANDS[m.band_index(f["twa"])][2]
        out.append(f"| {h:.1f} | {name} | {f['sail']:.0f} | {f['v']:.2f} | {f['va']:.2f} | {f['awa']:.0f} | {f['drive']:.0f} | "
                   f"{f['normal']:.0f} | {f['side']:.0f} | {f['c_n']:.2f} | {f['sail_moment']:.0f} | {f['rudder_balance']:+.2f} |")
    out += ["\n### Yelken trimi (ψ = 90°, δr = 0)\n",
            "| δs (°) | η | V* (m/s) | T itki (N) | F_N (N) | F_yan (N) |", "|---|---|---|---|---|---|"]
    for sail in (20, 30, 46, 60, 75):
        f = m.force_moment_breakdown(90, sail, 0, U)
        out.append(f"| {sail} | {f['eta']:.2f} | {f['v']:.2f} | {f['drive']:.0f} | {f['normal']:.0f} | {f['side']:.0f} |")
    out += ["\n### Dümen kuvveti ve dönme momenti (ψ = 90°, δs = 46°)\n",
            "| δr (°) | κ | v (m/s) | D_dümen (N) | C_L | Y_dümen (N) | N_dümen (N·m) | N_net (N·m) | ψ̇ (°/s) | R (m) | F_merkezcil (N) |",
            "|---|---|---|---|---|---|---|---|---|---|---|"]
    for rud in (0, 5, 10, 20, 35):
        f = m.force_moment_breakdown(90, 46, rud, U)
        R = "∞" if not np.isfinite(f["turn_radius"]) or f["turn_radius"] > 1e3 else f"{f['turn_radius']:.1f}"
        out.append(f"| {rud:+d} | {f['kappa']:.3f} | {f['v']:.2f} | {f['rudder_drag']:.0f} | {f['rudder_cl']:.2f} | {f['rudder_force']:.0f} | "
                   f"{f['rudder_moment']:.0f} | {f['net_moment']:.0f} | {f['yaw_rate']:+.1f} | {R} | {f['centripetal']:.0f} |")
    md = "\n".join(out) + "\n"
    with open(os.path.join(RES, "forces.md"), "w", encoding="utf-8") as fh:
        fh.write(md)

    # Kuvvet diyagramı: tam apaz, δs = 46°, δr = +10° (tekne doğuya gidiyor; çizim gövde eksenlerinde, pruva yukarı)
    f = m.force_moment_breakdown(90, 46, 10, U)
    fig, ax = plt.subplots(figsize=(7.2, 7.2))
    for cx in (-1.35, 1.35):
        ax.add_patch(plt.Polygon([(cx, 2.2), (cx + .22, 1.3), (cx + .22, -1.5), (cx + .12, -2.1), (cx - .12, -2.1),
                                  (cx - .22, -1.5), (cx - .22, 1.3)], color="#1e293b"))
    for by in (1.3, -1.4):
        ax.plot([-1.35, 1.35], [by, by], color="#475569", lw=4)
    d = np.radians(46.0)
    mast = np.array([0.0, 0.2])
    boom = mast + 2.85 * np.array([np.sin(d), -np.cos(d)])
    ax.plot(*zip(mast, boom), color="#2563eb", lw=4)
    ce = mast + 0.5 * (boom - mast)
    sc = 2.2 / f["normal"]                                     # ok ölçeği (m / N)

    def arrow(p, vec, color, text, off=(0.1, 0.1)):
        ax.annotate("", xy=(p[0] + vec[0], p[1] + vec[1]), xytext=p,
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=2.4, mutation_scale=16))
        ax.text(p[0] + vec[0] + off[0], p[1] + vec[1] + off[1], text, color=color, fontsize=9, fontweight="bold")

    n_dir = np.array([np.cos(d), np.sin(d)])                  # kirişe dik, rüzgar altına (sancak) ve ileri
    arrow(ce, f["normal"] * sc * n_dir, "#7c3aed", f"F_N = {f['normal']:.0f} N")
    arrow(ce, (0, f["drive"] * sc), "#16a34a", f"T = F_N·sin δs = {f['drive']:.0f} N", off=(-2.6, 0.15))
    arrow(ce, (f["side"] * sc, 0), "#d97706", f"F_yan = F_N·cos δs = {f['side']:.0f} N", off=(-0.6, -0.35))
    arrow((0, -2.3), (0, -f["hull_drag"] * sc), "#64748b", f"R_gövde = {f['hull_drag']:.0f} N", off=(0.15, 0))
    for cx in (-1.35, 1.35):
        hinge = np.array([cx, -2.1])
        r = np.radians(10.0)
        ax.plot(*zip(hinge, hinge + 0.95 * np.array([np.sin(r), -np.cos(r)])), color="#dc2626", lw=4)
    arrow((1.35, -2.6), (-f["rudder_force"] * sc * 0.25, 0), "#dc2626",
          f"Y_dümen = {f['rudder_force']:.0f} N (ok ¼ ölçek)\nN_dümen = Y·x_d = {f['rudder_moment']:.0f} N·m", off=(-3.2, -0.75))
    for wy in (0.6, 1.2, 1.8):                               # rüzgar kuzeyden = iskele bordadan (gövde ekseninde soldan)
        ax.annotate("", xy=(-2.6, wy), xytext=(-4.2, wy), arrowprops=dict(arrowstyle="->", color="#0284c7", lw=2.2))
    ax.text(-3.4, 2.2,f"gerçek rüzgar U = {U:.0f} m/s\n(iskele bordadan, TWA 90°)", ha="center", fontsize=8, color="#0369a1")
    ax.text(0, 2.5, "PRUVA (ψ = 090°)", ha="center", fontsize=9, fontweight="bold")
    ax.set_title(f"Kuvvet diyagramı – tam apaz, δs = 46°, δr = +10°, v = {f['v']:.2f} m/s\n"
                 f"Net moment {f['net_moment']:.0f} N·m → ψ̇ = {f['yaw_rate']:.1f} °/s, R = {f['turn_radius']:.1f} m", fontsize=10)
    ax.set_xlim(-4.6, 4.8)
    ax.set_ylim(-4.0, 3.0)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "force_diagram.png"), dpi=130)
    plt.close(fig)
    return md


def main():
    fig_polar()
    fig_trim()
    vt = vmg_table()
    tt, _ = tack_threshold()
    rows = scenario_matrix()
    ec = edge_cases()
    gs = gybe_only_sweep()
    fig_trajectories()
    waypoint_route()
    force_tables()
    ok, n = reachability()
    with open(os.path.join(RES, "summary.md"), "w", encoding="utf-8") as f:
        f.write("### VMG optimumları\n\n" + vt + "\n\n### Tack eşiği\n\n" + tt + "\n\n### Uç durumlar\n\n" + ec
                + "\n\n### Tack yasak – varış yarıçapı duyarlılığı (kumaş yelken, Kuzey 1 km)\n\n" + gs + "\n")
        f.write(f"\n### Senaryo matrisi: {sum(r['ulasti'] == 'EVET' for r in rows)}/{len(rows)} başarılı\n")
        f.write(f"\n### Erişilebilirlik: {ok}/{n} hedef başarılı\n")
    print(open(os.path.join(RES, "summary.md"), encoding="utf-8").read())


if __name__ == "__main__":
    main()
