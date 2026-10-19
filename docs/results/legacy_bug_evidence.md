## H1 – Kontra (tack) isimlendirmesi
- psi= 45°: eski -> tack='Sancak Kontra', rüzgar tarafı='İskele (Sol)' | yeni -> 'İskele Kontra', 'İskele (Sol)'
- psi= 90°: eski -> tack='Sancak Kontra', rüzgar tarafı='İskele (Sol)' | yeni -> 'İskele Kontra', 'İskele (Sol)'
- psi=270°: eski -> tack='İskele Kontra', rüzgar tarafı='Sancak (Sağ)' | yeni -> 'Sancak Kontra', 'Sancak (Sağ)'

## H2 – İskele/sancak sınır asimetrisi (aynı TWA, farklı rejim)
- TWA=50°: psi=50° -> Sancak Orsa (Close-Hauled) (oran 0.78) | psi=310° -> İskele Dar Apaz (Close Reach) (oran 0.98)
- TWA=80°: psi=80° -> Sancak Dar Apaz (Close Reach) (oran 0.98) | psi=280° -> İskele Tam Apaz (Beam Reach) (oran 1.3)
- TWA=120°: psi=120° -> Sancak Tam Apaz (Beam Reach) (oran 1.3) | psi=240° -> İskele Geniş Apaz (Broad Reach) (oran 0.98)
- TWA=150°: psi=150° -> Sancak Geniş Apaz (Broad Reach) (oran 0.98) | psi=210° -> Pupa Seyri (Running) (oran 0.58)

## H3 – Gösterilen hız ile gidilen yol tutarsızlığı (dümen 0, 30 s, 8 m/s)
| Profil | psi | Bom | Eski V (m/s) | Eski yol/30 s (m/s) | Fark % | Yeni V | Yeni yol/30 s |
|---|---|---|---|---|---|---|---|
| 0012 | 90 | 32 | 7.61 | 7.61 | -0.1 | 7.61 | 7.61 |
| 0012 | 90 | 47 | 9.78 | 9.28 | -5.1 | 9.78 | 9.45 |
| 0012 | 90 | 67 | 0.62 | 0.62 | -1.0 | 0.62 | 0.62 |
| 0012 | 135 | 52 | 5.74 | 5.74 | -0.0 | 5.74 | 5.74 |
| 0012 | 135 | 67 | 7.37 | 7.28 | -1.3 | 7.37 | 7.37 |
| 0012 | 135 | 87 | 0.47 | 0.47 | -0.0 | 0.47 | 0.47 |
| 0015 | 90 | 35 | 7.74 | 7.71 | -0.4 | 7.74 | 7.72 |
| 0015 | 90 | 50 | 9.67 | 9.00 | -6.9 | 9.67 | 9.38 |
| 0015 | 90 | 70 | 1.00 | 0.79 | -20.8 | 1.00 | 0.99 |
| 0015 | 135 | 54 | 6.50 | 6.50 | -0.0 | 6.50 | 6.50 |
| 0015 | 135 | 69 | 8.12 | 7.82 | -3.7 | 8.12 | 8.12 |
| 0015 | 135 | 89 | 0.84 | 0.67 | -20.0 | 0.84 | 0.84 |
| cloth | 90 | 36 | 7.08 | 7.71 | +9.0 | 7.08 | 7.06 |
| cloth | 90 | 51 | 8.82 | 8.56 | -2.9 | 8.82 | 8.60 |
| cloth | 90 | 71 | 2.33 | 1.65 | -29.1 | 2.33 | 2.26 |
| cloth | 135 | 55 | 5.66 | 6.21 | +9.6 | 5.66 | 5.66 |
| cloth | 135 | 70 | 7.05 | 7.03 | -0.3 | 7.05 | 7.05 |
| cloth | 135 | 90 | 1.86 | 1.36 | -27.1 | 1.86 | 1.86 |

En kötü fark (eski): 46.7 %

## H4 – Auto-trim dönüş sırasında yeniden trim yapmıyor (psi0=60°, dümen +20°, 20 s, 8 m/s)
- Eski: yol=22.0 m (bom 28°'de donmuş) | Yeni: yol=64.6 m (her adımda optimum)

## H5 – Orsaya kaçma (weather helm) süreksizliği (psi0=90°, 60 s, 8 m/s)
- dümen=+0.0°: eski son pruva=69.0° | yeni son pruva=69.0°
- dümen=+1.9°: eski son pruva=120.8° | yeni son pruva=127.4°
- dümen=+2.0°: eski son pruva=135.0° | yeni son pruva=128.3°

## H6 – Atalet yok: ölü açıdan geçiş (tack) imkânsız (psi0=40°, dümen -35°, 30 s)
- U= 3.0 m/s: eski son pruva=27.0° (ölü açıda kilit) | yeni son pruva=356.7°, ölü açıda kaldı
- U= 5.0 m/s: eski son pruva=29.1° (ölü açıda kilit) | yeni son pruva=330.8°, ölü açıda kaldı
- U= 8.0 m/s: eski son pruva=22.6° (ölü açıda kilit) | yeni son pruva=137.2°, ölü açıyı geçti
- U=12.0 m/s: eski son pruva=27.0° (ölü açıda kilit) | yeni son pruva=293.5°, ölü açıyı geçti
