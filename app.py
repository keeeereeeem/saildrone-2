import streamlit as st
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Wedge

import navigation as nav
import saildrone_model as sm

# --- SAYFA VE ARAYÜZ YAPILANDIRMASI ---
st.set_page_config(
    page_title="Yelkenli Katamaran ve Saildrone Simülatörü",
    page_icon="⛵",
    layout="wide"
)

# Profesyonel denizcilik ve kokpit teması CSS'i
st.markdown("""
<style>
    .block-container {
        padding-top: 0.7rem !important;
        padding-bottom: 0.5rem !important;
        padding-left: 1.8rem !important;
        padding-right: 1.8rem !important;
        max-width: 100% !important;
    }
    h2, h3 {
        margin-top: 0rem !important;
        margin-bottom: 0.35rem !important;
        font-weight: 600 !important;
    }
    .metric-card {
        background: #f8fafc;
        border-radius: 8px;
        padding: 8px 10px;
        border: 1px solid #e2e8f0;
        text-align: center;
        box-shadow: 0 1px 2px rgba(0,0,0,0.04);
    }
    .metric-val {
        font-size: 1.25rem;
        font-weight: 700;
        color: #0284c7;
    }
    .metric-lbl {
        font-size: 0.72rem;
        font-weight: 600;
        color: #64748b;
        letter-spacing: 0.4px;
    }
    .badge-full {
        background-color: #dcfce7;
        color: #15803d;
        padding: 2px 7px;
        border-radius: 4px;
        font-weight: 600;
        font-size: 0.76rem;
    }
    .badge-warn {
        background-color: #fef9c3;
        color: #a16207;
        padding: 2px 7px;
        border-radius: 4px;
        font-weight: 600;
        font-size: 0.76rem;
    }
    .badge-dead {
        background-color: #fee2e2;
        color: #b91c1c;
        padding: 2px 7px;
        border-radius: 4px;
        font-weight: 600;
        font-size: 0.76rem;
    }
    .polar-table {
        width: 100%;
        border-collapse: collapse;
        font-size: 0.8rem;
        margin-top: 6px;
    }
    .polar-table th, .polar-table td {
        border: 1px solid #e2e8f0;
        padding: 5px 8px;
        text-align: center;
    }
    .polar-table th {
        background-color: #f1f5f9;
        font-weight: 600;
        color: #334155;
    }
</style>
""", unsafe_allow_html=True)

# --- SENKRON GİRDİ BİLEŞENİ (SLIDER + KLAVYEDEN GİRİŞ) ---
def sync_input(label, key_prefix, min_val, max_val, default_val, step=1, help_text=""):
    slider_key = f"{key_prefix}_slider"
    num_key = f"{key_prefix}_num"

    if slider_key not in st.session_state:
        st.session_state[slider_key] = default_val
    if num_key not in st.session_state:
        st.session_state[num_key] = default_val

    def on_slider():
        st.session_state[num_key] = st.session_state[slider_key]

    def on_num():
        val = int(st.session_state[num_key])
        val = max(min_val, min(max_val, val))
        st.session_state[num_key] = val
        st.session_state[slider_key] = val

    st.markdown(f"<span style='font-weight: 600; font-size: 0.86rem;'>{label}</span>", unsafe_allow_html=True)
    c1, c2 = st.columns([3, 1.2], gap="small")
    with c1:
        st.slider(
            label=label, min_value=min_val, max_value=max_val, step=step,
            key=slider_key, on_change=on_slider, label_visibility="collapsed", help=help_text
        )
    with c2:
        st.number_input(
            label=label, min_value=min_val, max_value=max_val, step=step,
            key=num_key, on_change=on_num, label_visibility="collapsed"
        )
    return st.session_state[slider_key]

# --- 360 DERECE PUSULA VE AERODİNAMİK SEYİR POLARLARI ---
# Seyir analizi, trim verimi ve hareket denklemleri saildrone_model.py içindedir.
analyze_points_of_sail_360 = sm.analyze_points_of_sail_360

# --- SİMÜLASYON VE ÇİZİM ÇEKİRDEĞİ ---
def simulate_catamaran(heading_deg, sail_input_deg, rudder_deg, profile_name, wind_speed_ms, sim_time_s, auto_trim=False):
    """
    330°-030° Ölü Açı sektörü, NACA 0012 / NACA 0015 / Kumaş Yelken fiziksel polarları,
    ters işleyen dümen kinematiği ve merkezi katamaran görselleştirmesi.
    """
    profile = sm.profile_key(profile_name)
    is_12 = profile == "0012"
    is_cloth = profile == "cloth"

    # 1. BAŞLANGIÇ SEYİR DURUMU (360 Derece Analiz):
    regime, opt_sail, max_speed_ratio, tack, wind_side, leeward_sign, desc = sm.analyze_points_of_sail_360(heading_deg, profile_name)
    is_in_irons = (regime == sm.NO_GO_REGIME)

    # Otomatik trim seçeneği veya kullanıcının ayarı
    if auto_trim and not is_in_irons:
        sail_boom_angle = opt_sail
    else:
        sail_boom_angle = abs(sail_input_deg)

    # 2. YELKEN / KANAT TRİM VE AERODİNAMİK VERİM DEĞERLENDİRMESİ:
    if is_in_irons:
        trim_efficiency, trim_status, badge_type, camber_amount = 0.0, "Yelken Dolmuyor (Yapraklıyor)", "badge-dead", 0.0
    else:
        trim_efficiency, trim_status, badge_type, camber_amount = sm.trim_efficiency(sail_boom_angle, opt_sail, profile)

    # 3. DÜMEN VE FRENLEME ETKİSİ:
    rudder_clamped = float(np.clip(rudder_deg, -sm.RUDDER_MAX_DEG, sm.RUDDER_MAX_DEG))

    # Anlık başlangıç hızı (simülasyon döngüsüyle aynı fonksiyon)
    base_speed_ms = sm.target_speed(heading_deg, sail_boom_angle, rudder_clamped, profile, wind_speed_ms, auto_trim)
    base_speed_kts = base_speed_ms * sm.MS_TO_KTS

    # 4. SİMÜLASYON YOLU (surge ataleti + kinematik dönüş + orsaya kaçma):
    dt = sm.DT
    sim = sm.simulate_open_loop(heading_deg, sail_boom_angle, rudder_clamped, profile, wind_speed_ms,
                                sim_time_s, auto_trim=auto_trim, dt=dt)
    traj_x = list(sim["x"])
    traj_y = list(sim["y"])
    total_distance_m = sim["distance"]
    final_cog_deg = sim["final_heading"]

    # --- MATPLOTLIB ÇİZİMİ ---
    fig, ax = plt.subplots(figsize=(7.8, 7.8), dpi=110)
    fig.patch.set_facecolor('#ffffff')
    ax.set_facecolor('#f8fafc')

    # 1. ÖLÜ AÇI (NO-GO ZONE) 330° - 030° SEKTÖRÜ (60 DERECE):
    # Matematiksel açılar: 030° pusula = 60° mat, 330° pusula = 120° mat
    no_go_wedge = Wedge((0, 0), 8.5, 60, 120, facecolor='#fee2e2', edgecolor='#ef4444', 
                        alpha=0.32, linestyle='--', lw=1.2, zorder=1)
    ax.add_patch(no_go_wedge)
    ax.text(0, 5.5, "ÖLÜ AÇI (NO-GO ZONE)\n330° - 030° (Rüzgara Karşı)", 
            ha='center', va='center', color='#b91c1c', fontweight='bold', fontsize=8.2, zorder=2)

    # 2. Dairesel Pusula Kılavuzları (2m, 4m, 6m, 8m)
    for r in [2, 4, 6, 8]:
        circle = plt.Circle((0, 0), r, color='#e2e8f0', fill=False, linestyle=':', linewidth=0.8, zorder=1)
        ax.add_patch(circle)

    # 3. Pusula Yön İmleri
    ax.text(0, 8.4, "000° KUZEY", ha='center', va='bottom', fontsize=8.8, fontweight='bold', color='#1e293b')
    ax.text(8.4, 0, "090° DOĞU", ha='left', va='center', fontsize=8.8, fontweight='bold', color='#1e293b')
    ax.text(0, -8.4, "180° GÜNEY", ha='center', va='top', fontsize=8.8, fontweight='bold', color='#1e293b')
    ax.text(-8.4, 0, "270° BATI", ha='right', va='center', fontsize=8.8, fontweight='bold', color='#1e293b')

    # Ölü açı sınırları (030° ve 330°)
    ax.text(4.3, 7.3, "030°", ha='left', va='bottom', fontsize=8.2, fontweight='bold', color='#dc2626')
    ax.text(-4.3, 7.3, "330°", ha='right', va='bottom', fontsize=8.2, fontweight='bold', color='#dc2626')
    ax.text(6.0, -6.0, "135°", ha='center', va='center', fontsize=7.8, color='#64748b')
    ax.text(-6.0, -6.0, "225°", ha='center', va='center', fontsize=7.8, color='#64748b')

    # 4. SABİT RÜZGAR AKIŞ OKLARI (Yukarıdan Aşağıya ↓↓↓)
    for wx in np.linspace(-6.8, 6.8, 7):
        ax.annotate(
            '', xy=(wx, 6.2), xytext=(wx, 8.2),
            arrowprops=dict(arrowstyle="->", color='#0284c7', lw=1.8, alpha=0.75),
            zorder=3
        )
    ax.text(0, 7.4, f"SABİT RÜZGAR: {wind_speed_ms:.1f} m/s ({wind_speed_ms*1.944:.1f} kts) ↓↓↓", 
            ha='center', va='center', fontsize=8.5, fontweight='bold', color='#0369a1',
            bbox=dict(boxstyle='round,pad=0.25', facecolor='#e0f2fe', edgecolor='#38bdf8', alpha=0.95),
            zorder=4)

    # 5. ROTASYON VE KATAMARAN GÖVDESİ:
    init_psi_rad = np.radians(heading_deg)
    cos_p = np.cos(init_psi_rad)
    sin_p = np.sin(init_psi_rad)

    def rot(lx, ly):
        xw = lx * cos_p + ly * sin_p
        yw = -lx * sin_p + ly * cos_p
        return xw, yw

    hull_half_beam = 1.35
    def create_hull(cx):
        pts_local = [
            (cx, 2.2),             # Pruva sivri burun
            (cx + 0.22, 1.3),      # Omuzluk dış
            (cx + 0.22, -1.5),     # Bordası dış
            (cx + 0.12, -2.1),     # Kıç dış
            (cx - 0.12, -2.1),     # Kıç iç
            (cx - 0.22, -1.5),     # Bordası iç
            (cx - 0.22, 1.3),      # Omuzluk iç
        ]
        pts_w = [rot(px, py) for px, py in pts_local]
        return patches.Polygon(pts_w, closed=True, facecolor='#1e293b', edgecolor='#0f172a', lw=1.5, zorder=6)

    ax.add_patch(create_hull(-hull_half_beam))
    ax.add_patch(create_hull(hull_half_beam))

    # Kirişler ve Trambolin
    for beam_y in [1.3, -1.4]:
        p1 = rot(-hull_half_beam, beam_y)
        p2 = rot(hull_half_beam, beam_y)
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#475569', lw=3.8, solid_capstyle='round', zorder=5)

    tramp_local = [
        (-hull_half_beam + 0.25, 1.25),
        (hull_half_beam - 0.25, 1.25),
        (hull_half_beam - 0.25, -1.35),
        (-hull_half_beam + 0.25, -1.35)
    ]
    ax.add_patch(patches.Polygon([rot(px, py) for px, py in tramp_local], closed=True, 
                                 facecolor='#cbd5e1', edgecolor='#94a3b8', linestyle='--', lw=1, alpha=0.5, zorder=4))

    # Direk (Mast)
    mast_w = rot(0, 0.2)
    ax.plot(mast_w[0], mast_w[1], 'o', color='#f59e0b', markersize=8, markeredgecolor='#b45309', zorder=10)

    # 6. DÜMENLER (TERS İŞLEYEN DENİZCİ DÜMENİ):
    # Kullanıcı +rudder girdiğinde dümenci yekeyi sola basar, pala sağa açılır (+), tekne sağa döner!
    rudder_len = 0.95
    rudder_rad_local = np.radians(rudder_clamped)

    for hull_lx in [-hull_half_beam, hull_half_beam]:
        hinge_w = rot(hull_lx, -2.1)
        tip_lx = hull_lx + rudder_len * np.sin(rudder_rad_local)
        tip_ly = -2.1 - rudder_len * np.cos(rudder_rad_local)
        tip_w = rot(tip_lx, tip_ly)
        ax.plot([hinge_w[0], tip_w[0]], [hinge_w[1], tip_w[1]], color='#dc2626', lw=3.8, solid_capstyle='round', zorder=8)
        ax.plot(hinge_w[0], hinge_w[1], 'o', color='#991b1b', markersize=4, zorder=9)

    # 7. YELKEN / KANAT GEOMETRİSİ:
    boom_len = 2.85
    actual_sail_rot = leeward_sign * np.radians(sail_boom_angle)
    u_chord = np.array([np.sin(actual_sail_rot), -np.cos(actual_sail_rot)])
    u_norm = np.array([np.cos(actual_sail_rot), np.sin(actual_sail_rot)])

    tip_lx = boom_len * u_chord[0]
    tip_ly = 0.2 + boom_len * u_chord[1]
    tip_w = rot(tip_lx, tip_ly)

    if is_in_irons:
        if is_cloth:
            num_w = 20
            wt = np.linspace(0, 1, num_w)
            wave_lx = wt * tip_lx + 0.15 * np.sin(4 * np.pi * wt)
            wave_ly = 0.2 + wt * (tip_ly - 0.2)
            wave_pts = [rot(lx, ly) for lx, ly in zip(wave_lx, wave_ly)]
            ax.plot([p[0] for p in wave_pts], [p[1] for p in wave_pts], color='#ef4444', linestyle='--', lw=2.2, zorder=9)
            ax.text(tip_w[0], tip_w[1] - 0.3, "Yapraklıyor\n(Dolmuyor)", fontsize=7.8, color='#b91c1c', 
                    fontweight='bold', ha='center', zorder=11)
        else:
            # Kanat rüzgar içine firar eder (feathers / nötr kanat)
            ax.plot([mast_w[0], tip_w[0]], [mast_w[1], tip_w[1]], color='#ef4444', linestyle='--', lw=2.2, zorder=9)
            ax.text(tip_w[0], tip_w[1] - 0.3, "Kanat Nötr\n(Taşıma Yok)", fontsize=7.8, color='#b91c1c', 
                    fontweight='bold', ha='center', zorder=11)
    else:
        if is_cloth:
            # Kumaş yelken: Torlu eğri
            s_t = np.linspace(0, 1, 25)
            b_dx = tip_lx - 0.0
            b_dy = tip_ly - 0.2
            b_len = np.hypot(b_dx, b_dy)
            norm_x = -b_dy / b_len * leeward_sign
            norm_y =  b_dx / b_len * leeward_sign

            camber_disp = camber_amount * np.sin(np.pi * s_t)
            curve_lx = s_t * b_dx + camber_disp * norm_x
            curve_ly = 0.2 + s_t * b_dy + camber_disp * norm_y

            curve_w = [rot(lx, ly) for lx, ly in zip(curve_lx, curve_ly)]
            cx_w = [p[0] for p in curve_w]
            cy_w = [p[1] for p in curve_w]

            ax.plot([mast_w[0], tip_w[0]], [mast_w[1], tip_w[1]], color='#475569', lw=2.2, zorder=8)
            ax.plot(cx_w, cy_w, color='#2563eb', lw=3.8, solid_capstyle='round', zorder=9)
            fill_poly_w = [mast_w] + curve_w + [tip_w]
            ax.add_patch(patches.Polygon(fill_poly_w, closed=True, facecolor='#2563eb', alpha=0.22, zorder=7))
        else:
            # NACA 0012 veya NACA 0015 Rijit Kanat Profili
            t_ratio = 0.12 if is_12 else 0.15
            n_pts = 40
            x_norm = np.linspace(0, 1, n_pts)
            yt = 5.0 * t_ratio * (
                0.2969 * np.sqrt(x_norm) - 0.1260 * x_norm - 0.3516 * (x_norm**2) + 0.2843 * (x_norm**3) - 0.1036 * (x_norm**4)
            ) * boom_len

            upper_lx = [x_i * boom_len * u_chord[0] + y_i * u_norm[0] for x_i, y_i in zip(x_norm, yt)]
            upper_ly = [0.2 + x_i * boom_len * u_chord[1] + y_i * u_norm[1] for x_i, y_i in zip(x_norm, yt)]
            lower_lx = [x_i * boom_len * u_chord[0] - y_i * u_norm[0] for x_i, y_i in zip(x_norm[::-1], yt[::-1])]
            lower_ly = [0.2 + x_i * boom_len * u_chord[1] - y_i * u_norm[1] for x_i, y_i in zip(x_norm[::-1], yt[::-1])]

            poly_lx = upper_lx + lower_lx
            poly_ly = upper_ly + lower_ly
            poly_w = [rot(lx, ly) for lx, ly in zip(poly_lx, poly_ly)]

            foil_color = '#0284c7' if is_12 else '#4338ca'
            foil_edge = '#0369a1' if is_12 else '#312e81'

            ax.add_patch(patches.Polygon(poly_w, closed=True, facecolor=foil_color, edgecolor=foil_edge, lw=2.0, alpha=0.6, zorder=9))
            ax.plot([mast_w[0], tip_w[0]], [mast_w[1], tip_w[1]], color='#0f172a', lw=1.8, linestyle='-', zorder=10)

            # Aerodinamik kaburga / istasyon çizgileri (Ribs)
            for frac in [0.25, 0.50, 0.75]:
                idx = int(frac * (n_pts - 1))
                p_up = rot(upper_lx[idx], upper_ly[idx])
                p_dn = rot(lower_lx[-(idx+1)], lower_ly[-(idx+1)])
                ax.plot([p_up[0], p_dn[0]], [p_up[1], p_dn[1]], color='#0f172a', lw=1.0, alpha=0.7, zorder=10)

    # 8. TEKNENİN GİTTİĞİ DOĞRULTU VE SİMÜLASYON YOLU:
    if total_distance_m > 1.0 and not is_in_irons:
        max_dist_to_show = max(total_distance_m, 1.0)
        scale_factor = min(1.0, 7.2 / max_dist_to_show)
        scaled_tx = [x * scale_factor for x in traj_x]
        scaled_ty = [y * scale_factor for y in traj_y]

        # Rota eğrisi / çizgisi
        ax.plot(scaled_tx, scaled_ty, color='#10b981', lw=3.8, linestyle='-', zorder=12)
        
        # 5 saniyelik zaman işaretçileri
        step_5s = max(1, int(5.0 / dt))
        for idx in range(step_5s, len(scaled_tx), step_5s):
            ax.plot(scaled_tx[idx], scaled_ty[idx], 'o', color='#059669', markersize=3.8, zorder=13)

        # Varış noktası oku ve etiketi
        fx, fy = scaled_tx[-1], scaled_ty[-1]
        ax.annotate(
            '', xy=(fx, fy), xytext=(scaled_tx[-2], scaled_ty[-2]),
            arrowprops=dict(arrowstyle="-|>", color='#10b981', lw=3.8, mutation_scale=22),
            zorder=14
        )
        label_x = fx * 1.15
        label_y = fy * 1.15
        ax.text(label_x, label_y, 
                f"GİDİLEN YOL: {total_distance_m:.1f} m\nSon pruva: {final_cog_deg:.0f}° | Ort. {total_distance_m / sim_time_s * sm.MS_TO_KTS:.1f} kts\n({sim_time_s:.0f} sn)",
                fontsize=8.0, fontweight='bold', color='#065f46', ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.25', facecolor='#d1fae5', edgecolor='#10b981', alpha=0.95),
                zorder=15)
    else:
        ax.text(0, 0, "[!] ÖLÜ AÇI / İTKİ YOK\nYelken Dolmuyor (Kör Nokta)", color='#b91c1c', fontsize=8.5, fontweight='bold',
                ha='center', va='center', bbox=dict(boxstyle='round,pad=0.35', facecolor='#fee2e2', edgecolor='#ef4444'),
                zorder=15)

    # Pruva Referans Çizgisi (Kırmızı pruva hattı)
    bow_ref = rot(0, 4.4)
    ax.plot([0, bow_ref[0]], [0, bow_ref[1]], color='#ef4444', linestyle='-', lw=1.5, zorder=5, label='Pruva Hattı')

    ax.set_xlim(-9, 9)
    ax.set_ylim(-9, 9)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.5, color='#cbd5e1')
    ax.set_xlabel("Doğu / Batı Mesafesi (m)", fontsize=8.5, color='#475569')
    ax.set_ylabel("Kuzey / Güney Mesafesi (m)", fontsize=8.5, color='#475569')
    ax.tick_params(colors='#64748b', labelsize=8)
    for spine in ax.spines.values():
        spine.set_color('#cbd5e1')

    plt.tight_layout()

    # Dümen yönü açıklaması
    if rudder_clamped > 0:
        rudder_info = f"Sağa Dönüş (+{rudder_clamped:.0f}° Sancak)"
    elif rudder_clamped < 0:
        rudder_info = f"Sola Dönüş ({rudder_clamped:.0f}° İskele)"
    else:
        rudder_info = "Düz Rota (0°)"

    telemetry = {
        "speed_kts": base_speed_kts,
        "speed_ms": base_speed_ms,
        "distance_m": total_distance_m,
        "regime": regime,
        "regime_desc": desc,
        "opt_sail": opt_sail,
        "actual_sail": sail_boom_angle,
        "heading_deg": heading_deg,
        "tack": tack,
        "wind_side": wind_side,
        "trim_status": trim_status,
        "trim_eff": trim_efficiency * 100.0,
        "badge_type": badge_type,
        "final_cog_deg": final_cog_deg,
        "rudder_info": rudder_info,
        "rudder_deg": rudder_clamped,
        "profile_name": profile_name
    }

    return fig, telemetry

# --- STREAMLIT ARAYÜZÜ ---
st.markdown("## ⛵ Otonom Yelkenli Katamaran ve Saildrone Simülatörü")
st.caption("360° Pusula Seyirleri • 330°-030° Ölü Açı • NACA 0012 / 0015 / Kumaş Yelken Aerodinamik Polarları • Realist Dümen")

tab_free, tab_nav = st.tabs(["⛵ Serbest Seyir", "🎯 Hedefe Navigasyon"])

with tab_free:
    col_ctrl, col_graph = st.columns([1.1, 2.0], gap="large")

    with col_ctrl:
        st.markdown("### ⚙️ Kontrol Paneli")
    
        # 1. Yelken Profili
        naca_profile = st.selectbox(
            "Yelken / Kanat Profili",
            ["NACA 0012 (Sert Kanat / İnce Simetrik)", "NACA 0015 (Sert Kanat / Kalın Profil)", "Kumaş Yelken (Cambered / Klasik)"],
            index=0,
            help="NACA 0012: İnce simetrik (%12), orsa ve tam apazda en hızlı (1.30x). NACA 0015: Kalın profil (%15), geniş apazda zirve (1.05x). Kumaş Yelken: Esnek torlu, pupa seyrinde en yüksek itki (0.74x)."
        )

        # 2. Tekne Yönü (0 - 360°)
        boat_heading_val = sync_input(
            label="🧭 Tekne Yönü (Pruva Açısı: 0° - 360°)",
            key_prefix="heading",
            min_val=0,
            max_val=360,
            default_val=90,
            step=1,
            help_text="330°-030°: Ölü Açı (60° No-Go), 030°-050°: Orsa, 050°-080°: Dar Apaz, 090°: Tam Apaz (En Hızlı), 135°: Geniş Apaz, 180°: Pupa, 270°: Sancak Kontra Tam Apaz."
        )

        # 3. Yelken Açısı ve Otomatik Trim
        col_s1, col_s2 = st.columns([2.5, 1.5])
        with col_s1:
            sail_angle_val = sync_input(
                label="📐 Yelken Bom Açısı (0° - 90°)",
                key_prefix="sail",
                min_val=0,
                max_val=90,
                default_val=42 if "0012" in naca_profile else (45 if "0015" in naca_profile else 46),
                step=1,
                help_text="Iskota/Kanat trimi: Bomun orta hattan açısı (0°: Kasılı, 12°-16°: Orsa, 42°-46°: Tam Apaz, 62°-65°: Geniş Apaz, 78°-82°: Pupa)."
            )
        with col_s2:
            st.markdown("<div style='height:22px;'></div>", unsafe_allow_html=True)
            auto_trim_active = st.checkbox("🤖 Auto-Trim", value=False, help="Saildrone otomatik trim modu: Seçili kanat profiline ve anlık rüzgar açısına göre en ideal trim açısına otomatik ayarlar.")

        # 4. Dümen Açısı (Realist ±35°)
        rudder_angle_val = sync_input(
            label="🕹️ Dümen Açısı (-35° İskele / +35° Sancak)",
            key_prefix="rudder",
            min_val=-35,
            max_val=35,
            default_val=0,
            step=1,
            help_text="Pozitif (+): Yeke sola basılır, pala sağa açılır, tekne sağa (sancağa) döner. Negatif (-): Pala sola açılır, tekne sola döner."
        )

        # 5. Rüzgar Hızı
        wind_speed_val = sync_input(
            label="💨 Sabit Rüzgar Hızı (m/s)",
            key_prefix="wind",
            min_val=1,
            max_val=25,
            default_val=8,
            step=1,
            help_text="Sabit kuzey rüzgarı (8 m/s ≈ 15.5 knot)."
        )

        # 6. Simülasyon Süresi
        sim_time_val = sync_input(
            label="⏱️ Simülasyon Süresi (sn)",
            key_prefix="sim_time",
            min_val=5,
            max_val=60,
            default_val=30,
            step=1,
            help_text="Teknenin denizde seyir ve dönüş yapacağı toplam süre."
        )

        st.markdown("---")
        st.info("💡 **Girdi Kolaylığı:** Değerleri ister kaydırıcıyla sürükleyin, ister yanındaki kutucuğa tıklayıp klavyeden doğrudan yazın.")

        with st.expander("📚 Profil Karşılaştırma & Aerodinamik Tablosu"):
            st.markdown("""
            <table class="polar-table">
                <thead>
                    <tr>
                        <th>Profil</th>
                        <th>Orsa (35°)</th>
                        <th>Tam Apaz (90°)</th>
                        <th>Geniş Apaz (135°)</th>
                        <th>Pupa (180°)</th>
                    </tr>
                </thead>
                <tbody>
                    <tr>
                        <td><b>NACA 0012</b></td>
                        <td style="color:#16a34a;"><b>0.78x (Zirve)</b></td>
                        <td style="color:#16a34a;"><b>1.30x (Zirve)</b></td>
                        <td>0.98x</td>
                        <td>0.58x</td>
                    </tr>
                    <tr>
                        <td><b>NACA 0015</b></td>
                        <td>0.74x</td>
                        <td>1.25x</td>
                        <td style="color:#16a34a;"><b>1.05x (Zirve)</b></td>
                        <td>0.65x</td>
                    </tr>
                    <tr>
                        <td><b>Kumaş Yelken</b></td>
                        <td>0.62x</td>
                        <td>1.15x</td>
                        <td>0.92x</td>
                        <td style="color:#16a34a;"><b>0.74x (Zirve)</b></td>
                    </tr>
                </tbody>
            </table>
            <div style="font-size:0.75rem; color:#64748b; margin-top:5px;">
            * Ölü Açı: 330° - 030° (60° No-Go Zone). Tüm hızlar rüzgar hızının katı (V_b / V_w) olarak hesaplanır.
            </div>
            """, unsafe_allow_html=True)

    with col_graph:
        st.markdown("### 📊 Katamaran Görselleştirmesi ve Gidilen Yol")
    
        fig, tel = simulate_catamaran(
            heading_deg=boat_heading_val,
            sail_input_deg=sail_angle_val,
            rudder_deg=rudder_angle_val,
            profile_name=naca_profile,
            wind_speed_ms=float(wind_speed_val),
            sim_time_s=float(sim_time_val),
            auto_trim=auto_trim_active
        )

        st.pyplot(fig, width='stretch')

        c1, c2, c3, c4 = st.columns(4)

        with c1:
            st.markdown(f"""
            <div class="metric-card">
                <div class="metric-lbl">TEKNE HIZI</div>
                <div class="metric-val">{tel['speed_kts']:.1f} kts</div>
                <div style="font-size:0.75rem; color:#64748b;">{tel['speed_ms']:.1f} m/s</div>
            </div>
            """, unsafe_allow_html=True)

        with c2:
            st.markdown(f"""
            <div class="metric-card">
                <div class="metric-lbl">GİDİLEN MESAFE</div>
                <div class="metric-val">{tel['distance_m']:.1f} m</div>
                <div style="font-size:0.75rem; color:#64748b;">{sim_time_val} sn sürede</div>
            </div>
            """, unsafe_allow_html=True)

        with c3:
            st.markdown(f"""
            <div class="metric-card">
                <div class="metric-lbl">360° SEYİR TİPİ</div>
                <div style="font-size:0.92rem; font-weight:700; color:#0f172a; margin-top:3px;">{tel['regime']}</div>
                <div style="font-size:0.72rem; color:#64748b;">Pruva: {tel['heading_deg']}° | {tel['wind_side']}</div>
            </div>
            """, unsafe_allow_html=True)

        with c4:
            badge_html = f'<span class="{tel["badge_type"]}">{tel["trim_status"]}</span>'
            st.markdown(f"""
            <div class="metric-card">
                <div class="metric-lbl">TRİM VE DÜMEN DURUMU</div>
                <div style="margin-top:2px;">{badge_html}</div>
                <div style="font-size:0.72rem; color:#64748b; margin-top:2px;">{tel['rudder_info']}</div>
            </div>
            """, unsafe_allow_html=True)

        st.markdown(f"**ℹ️ Seyir & Aerodinamik Durumu:** {tel['regime_desc']}")


# --- HEDEFE NAVİGASYON SEKMESİ ---
def draw_navigation(res, start, target, arrival_radius):
    fig, ax = plt.subplots(figsize=(7.8, 7.0), dpi=110)
    sc = ax.scatter(res.x, res.y, c=res.v, s=4, cmap="viridis", zorder=3)
    fig.colorbar(sc, ax=ax, label="Tekne hızı (m/s)", shrink=0.8)
    ax.plot(*start, "o", color="#0f172a", ms=8, zorder=4, label="Başlangıç")
    ax.plot(*target, "*", color="#16a34a", ms=16, zorder=4, label="Hedef")
    ax.add_patch(plt.Circle(target, arrival_radius, color="#16a34a", fill=False, lw=1.5, zorder=4))
    ax.plot([start[0], target[0]], [start[1], target[1]], ":", color="#64748b", lw=1, zorder=2)
    ax.annotate("", xy=(0.92, 0.80), xytext=(0.92, 0.95), xycoords="axes fraction",
                arrowprops=dict(arrowstyle="->", color="#0369a1", lw=2.5))
    ax.text(0.92, 0.97, "Rüzgar", transform=ax.transAxes, ha="center", fontsize=8, color="#0369a1")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.set_xlabel("Doğu (m)")
    ax.set_ylabel("Kuzey (m)")
    ax.legend(loc="lower left", fontsize=8)
    plt.tight_layout()
    return fig


with tab_nav:
    nc1, nc2 = st.columns([1.1, 2.0], gap="large")
    with nc1:
        st.markdown("### 🎯 Görev Tanımı")
        nav_profile = st.selectbox(
            "Yelken / Kanat Profili ",
            ["NACA 0012 (Sert Kanat / İnce Simetrik)", "NACA 0015 (Sert Kanat / Kalın Profil)", "Kumaş Yelken (Cambered / Klasik)"],
            index=0, key="nav_profile")
        nav_wind = st.number_input("💨 Rüzgar hızı (m/s, kuzeyden)", 0.5, 25.0, 8.0, 0.5, key="nav_wind")
        a1, a2 = st.columns(2)
        sx = a1.number_input("Başlangıç X / Doğu (m)", -20000.0, 20000.0, 0.0, 50.0, key="nav_sx")
        sy = a2.number_input("Başlangıç Y / Kuzey (m)", -20000.0, 20000.0, 0.0, 50.0, key="nav_sy")
        b1, b2 = st.columns(2)
        tx = b1.number_input("Hedef X / Doğu (m)", -20000.0, 20000.0, 0.0, 50.0, key="nav_tx")
        ty = b2.number_input("Hedef Y / Kuzey (m)", -20000.0, 20000.0, 1000.0, 50.0, key="nav_ty")
        auto_h = st.checkbox("Başlangıç pruvasını otomatik seç (en iyi VMG)", value=True, key="nav_auto_h")
        init_h = None if auto_h else st.number_input("Başlangıç pruvası (°)", 0.0, 359.9, 90.0, 1.0, key="nav_h0")
        c1_, c2_ = st.columns(2)
        arr_r = c1_.number_input("Varış yarıçapı (m)", 2.0, 500.0, 10.0, 1.0, key="nav_r")
        max_t = c2_.number_input("Süre sınırı (s)", 60.0, 86400.0, 7200.0, 60.0, key="nav_T")
        allow_tack = st.checkbox("Tack'e izin ver (ölü açıdan geçiş)", value=True, key="nav_tack")

    with nc2:
        st.markdown("### 🗺️ Rota ve Sonuç")
        cfg = nav.NavConfig(arrival_radius=float(arr_r), max_time=float(max_t), allow_tack=allow_tack)
        res = nav.navigate((sx, sy), (tx, ty), nav_profile, float(nav_wind), initial_heading=init_h, cfg=cfg)
        st.pyplot(draw_navigation(res, (sx, sy), (tx, ty), float(arr_r)), width='stretch')
        if res.reached:
            st.success(f"✅ Hedefe ulaşıldı: {res.time:.0f} s ({res.time / 60:.1f} dk)")
        else:
            st.error(f"❌ Hedefe ulaşılamadı: {res.reason} — kalan mesafe {res.final_distance:.0f} m")
        m1, m2, m3, m4 = st.columns(4)
        m1.metric("Düz mesafe", f"{res.straight_distance:.0f} m")
        m2.metric("Katedilen yol", f"{res.path_length:.0f} m")
        m3.metric("Ort. VMG", f"{(res.straight_distance - res.final_distance) / max(res.time, 1e-9):.2f} m/s")
        m4.metric("Tack / Gybe", f"{res.n_tacks} / {res.n_gybes}")
