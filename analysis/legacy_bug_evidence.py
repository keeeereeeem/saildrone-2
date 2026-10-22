"""
Orijinal app.py'deki (legacy/app_original.py) mantık hatalarını sayısal olarak gösterir
ve güncel kuvvet tabanlı modelle (saildrone_model.py, 50 kg referans tekne) karşılaştırır. Eski kod 250 kg'lık
farklı bir tekneyi temsil ettiğinden hız değerleri doğrudan karşılaştırılamaz; tablolar hataların niteliğini gösterir.

Çalıştırma:  python analysis/legacy_bug_evidence.py
"""
import ast
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Wedge
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import saildrone_model as m  # noqa: E402


def load_legacy():
    """Orijinal dosyadan yalnızca iki fonksiyonu (Streamlit arayüzü olmadan) yükler."""
    src = open(os.path.join(ROOT, "legacy", "app_original.py"), encoding="utf-8").read()
    tree = ast.parse(src)
    funcs = [n for n in tree.body if isinstance(n, ast.FunctionDef)
             and n.name in ("analyze_points_of_sail_360", "simulate_catamaran")]
    ns = dict(np=np, plt=plt, patches=patches, Wedge=Wedge)
    exec(compile(ast.Module(body=funcs, type_ignores=[]), "legacy", "exec"), ns)
    return ns["analyze_points_of_sail_360"], ns["simulate_catamaran"]


def legacy_run(sim, **kw):
    fig, tel = sim(**kw)
    plt.close(fig)
    return tel


def main(out=sys.stdout):
    analyze, sim = load_legacy()
    PC = "Kumaş Yelken"
    p = lambda *a: print(*a, file=out)

    p("## H1 – Kontra (tack) isimlendirmesi")
    for h in (45, 90, 270):
        _, _, _, tack, side, lee, _ = analyze(h, PC)
        _, _, _, tack2, side2, _, _ = m.analyze_points_of_sail_360(h, PC)
        p(f"- psi={h:3d}°: eski -> tack='{tack}', rüzgar tarafı='{side}' | yeni -> '{tack2}', '{side2}'")

    p("\n## H2 – İskele/sancak sınır asimetrisi (aynı TWA, farklı rejim)")
    for twa in (50, 80, 120, 150):
        a = analyze(twa, PC)
        b = analyze(360 - twa, PC)
        p(f"- TWA={twa}°: psi={twa}° -> {a[0]} (oran {a[2]}) | psi={360-twa}° -> {b[0]} (oran {b[2]})")

    p("\n## H3 – Gösterilen hız ile gidilen yol tutarsızlığı (dümen 0, 30 s, 8 m/s)")
    p("| Profil | psi | Bom | Eski V (m/s) | Eski yol/30 s (m/s) | Fark % | Yeni V | Yeni yol/30 s |")
    p("|---|---|---|---|---|---|---|---|")
    worst = 0.0
    for prof, key in ((PC, "cloth"),):
        for h in (90, 135):
            opt = m.polar(h, 8.0)[1]
            for ds in (-30, -10, -5, 5, 10, 25):
                sail = max(0, min(90, opt + ds))
                tel = legacy_run(sim, heading_deg=h, sail_input_deg=sail, rudder_deg=0, profile_name=prof,
                                 wind_speed_ms=8.0, sim_time_s=30.0)
                v_avg = tel["distance_m"] / 30.0
                diff = 100 * (v_avg - tel["speed_ms"]) / max(tel["speed_ms"], 1e-9)
                new = m.simulate_open_loop(h, sail, 0, 8.0, 30.0)
                worst = max(worst, abs(diff))
                if ds in (-10, 5, 25):
                    p(f"| {key} | {h} | {sail:.0f} | {tel['speed_ms']:.2f} | {v_avg:.2f} | {diff:+.1f} | "
                      f"{new['v0']:.2f} | {new['distance']/30:.2f} |")
    p(f"\nEn kötü fark (eski): {worst:.1f} %")

    p("\n## H4 – Auto-trim dönüş sırasında yeniden trim yapmıyor (psi0=60°, dümen +20°, 20 s, 8 m/s)")
    tel = legacy_run(sim, heading_deg=60, sail_input_deg=0, rudder_deg=20, profile_name=PC,
                     wind_speed_ms=8.0, sim_time_s=20.0, auto_trim=True)
    new = m.simulate_open_loop(60, 0, 20, 8.0, 20.0, auto_trim=True)
    p(f"- Eski: yol={tel['distance_m']:.1f} m (bom başlangıç açısında donmuş) | Yeni: yol={new['distance']:.1f} m (her adımda optimum)")

    p("\n## H5 – Orsaya kaçma (weather helm) süreksizliği (psi0=90°, 60 s, 8 m/s)")
    for rud in (0.0, 1.9, 2.0):
        tel = legacy_run(sim, heading_deg=90, sail_input_deg=46, rudder_deg=rud, profile_name=PC,
                         wind_speed_ms=8.0, sim_time_s=60.0)
        new = m.simulate_open_loop(90, 33, rud, 8.0, 60.0)
        p(f"- dümen={rud:+.1f}°: eski son pruva={tel['final_cog_deg']:.1f}° | yeni son pruva={new['final_heading']:.1f}°")

    p("\n## H6 – Atalet yok: ölü açıdan geçiş (tack) imkânsız (psi0=40°, dümen -35°, 30 s)")
    for U in (3.0, 5.0, 8.0, 12.0):
        tel = legacy_run(sim, heading_deg=40, sail_input_deg=16, rudder_deg=-35, profile_name=PC,
                         wind_speed_ms=U, sim_time_s=30.0)
        new = m.simulate_open_loop(40, 12, -35, U, 30.0, auto_trim=True)
        stuck_new = m.true_wind_angle(new["final_heading"]) <= m.NO_GO_DEG and new["v"][-1] < 0.05
        p(f"- U={U:4.1f} m/s: eski son pruva={tel['final_cog_deg']:.1f}° (ölü açıda kilit) | "
          f"yeni son pruva={new['final_heading']:.1f}°, "
          f"{'ölü açıda kaldı' if stuck_new else 'ölü açıyı geçti'}")


if __name__ == "__main__":
    path = os.path.join(ROOT, "docs", "results", "legacy_bug_evidence.md")
    with open(path, "w", encoding="utf-8") as f:
        main(f)
    print(open(path, encoding="utf-8").read())
