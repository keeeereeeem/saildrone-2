### Hız poları

| TWA (°) | Seyir | V/U | En iyi bom (°) | AWA (°) | Yan kayma λ (°) | V/U, devrilme sınırı U=8 | V/U, devrilme sınırı U=15 |
|---|---|---|---|---|---|---|---|
| 35 | Orsa | 0.643 | 5.4 | 21.4 | 2.55 | 0.534 | 0.321 |
| 40 | Orsa | 0.719 | 6.4 | 23.4 | 2.22 | 0.581 | 0.350 |
| 45 | Orsa | 0.784 | 7.9 | 25.4 | 1.95 | 0.624 | 0.377 |
| 60 | Dar Apaz | 0.925 | 12.3 | 31.3 | 1.43 | 0.736 | 0.453 |
| 75 | Dar Apaz | 0.989 | 17.8 | 37.8 | 1.09 | 0.835 | 0.529 |
| 90 | Tam Apaz | 0.978 | 24.1 | 45.6 | 0.82 | 0.918 | 0.610 |
| 105 | Tam Apaz | 0.903 | 33.8 | 56.3 | 0.57 | 0.903 | 0.698 |
| 120 | Tam Apaz | 0.782 | 48.5 | 72.0 | 0.32 | 0.782 | 0.770 |
| 135 | Geniş Apaz | 0.647 | 69.8 | 94.8 | 0.06 | 0.647 | 0.647 |
| 150 | Geniş Apaz | 0.533 | 75.2 | 123.7 | 0.02 | 0.533 | 0.533 |
| 165 | Pupa | 0.483 | 84.3 | 151.8 | 0.04 | 0.483 | 0.483 |
| 180 | Pupa | 0.465 | 90.0 | 180.0 | 0.00 | 0.465 | 0.465 |

### VMG optimumları

| U (m/s) | Rüzgar üstü en iyi TWA | VMG_up (m/s) | Rüzgar altı en iyi TWA | VMG_down (m/s) | Tam pupa V (m/s) |
|---|---|---|---|---|---|
| 1 | 43.0° | 0.54 | 164.5° | 0.47 | 0.46 |
| 3 | 43.0° | 1.61 | 164.5° | 1.40 | 1.39 |
| 8 | 43.0° | 4.29 | 164.5° | 3.73 | 3.72 |
| 15 | 43.0° | 8.04 | 164.5° | 7.00 | 6.97 |
| 20 | 43.0° | 10.73 | 164.5° | 9.33 | 9.30 |

### Tack (açık çevrim)

| Tekne | U=2: geçti? / kapasite | U=5: geçti? / kapasite | U=8: geçti? / kapasite | U=15: geçti? / kapasite | U=20: geçti? / kapasite |
|---|---|---|---|---|---|
| küçük (×0.75) | ✔ / 73° | ✔ / 79° | ✔ / 80° | ✔ / 80° | ✔ / 80° |
| referans | ✔ / 73° | ✔ / 79° | ✔ / 80° | ✔ / 80° | ✔ / 80° |
| büyük (×2) | ✔ / 73° | ✔ / 79° | ✔ / 80° | ✔ / 80° | ✔ / 80° |
| hafif (25 kg) | ✘ / 36° | ✔ / 40° | ✔ / 40° | ✔ / 40° | ✔ / 40° |
| ağır (150 kg) | ✔ / 218° | ✔ / 237° | ✔ / 240° | ✔ / 241° | ✔ / 241° |

Güdümün tack koşulu: kapasite ≥ 74.8° (ölü açı 60° + 5° pay, × 1.15). Kapasite, çırpınan yelkenin en kötü durum rüzgar frenini ve stall dümeninin sürüklemesini içerir; karar ayrıca manevranın ileri simülasyonuyla doğrulanır.

### Uç durumlar

| Durum | Ulaştı | Süre (s) | Tack | Gybe | Sonuç |
|---|---|---|---|---|---|
| Başlangıç pruvası ölü açıda (000°), hedef Doğu 1 km, 8 m/s | HAYIR | 20 | 0 | 0 | Ölü açıda takıldı (in irons) – hız sıfır, dümen etkisiz |
| Başlangıç pruvası ölü açı sınırında (032°), hedef Kuzey 1 km, 8 m/s | EVET | 265 | 0 | 2 | Hedefe ulaşıldı |
| Başlangıç pruvası ters yönde (180°), hedef Kuzey 1 km, 8 m/s | EVET | 260 | 0 | 1 | Hedefe ulaşıldı |
| Tack yasak (yalnızca gybe), hedef Kuzey 1 km, 8 m/s | EVET | 268 | 0 | 2 | Hedefe ulaşıldı |
| Hafif rüzgar 1 m/s, hedef Kuzey 1 km | EVET | 2308 | 0 | 4 | Hedefe ulaşıldı |
| Kuvvetli rüzgar 20 m/s, hedef Kuzey 1 km | EVET | 107 | 0 | 1 | Hedefe ulaşıldı |
| Hedef başlangıçta (mesafe 0) | EVET | 0 | 0 | 0 | Hedefe ulaşıldı |
| Uzun menzil: Kuzey 10 km, 5 m/s | EVET | 3917 | 1 | 3 | Hedefe ulaşıldı |
| Kısa süre limiti: Kuzey 1 km, 3 m/s, 300 s | HAYIR | 300 | 0 | 0 | Süre sınırı aşıldı |

### Tack yasak – varış yarıçapı duyarlılığı (Kuzey 1 km)

| Varış yarıçapı (m) | U=3 m/s | U=5 m/s | U=8 m/s | U=12 m/s | U=15 m/s |
|---|---|---|---|---|---|
| *Orsada dönüş çapı 2R (m)* | *15* | *15* | *24* | *36* | *45* |
| 5 | ✔ 705 s | ✔ 421 s | ✔ 269 s | ✔ 171 s | ✔ 139 s |
| 10 | ✔ 702 s | ✔ 419 s | ✔ 268 s | ✔ 170 s | ✔ 138 s |
| 20 | ✔ 697 s | ✔ 416 s | ✔ 266 s | ✔ 169 s | ✔ 138 s |
| 30 | ✔ 691 s | ✔ 414 s | ✔ 265 s | ✔ 168 s | ✔ 137 s |

### Senaryo matrisi: 30/30 başarılı

### Erişilebilirlik: 72/72 hedef başarılı

### Düz rota (PID)

| Kerteriz (°) | U (m/s) | Sapma (m) | Pruva hatası (°) | Sonuç | Süre (s) | Aşım (m) | Oturma (s) | RMS son yarı (m) | Dümen hareketi (°) | En büyük λ (°) |
|---|---|---|---|---|---|---|---|---|---|---|
| 045 | 8 | +30 | +0 | B ✔ | 106 | 0.13 | 42 | 0.11 | 15 | 2.6 |
| 315 | 8 | -30 | +0 | B ✔ | 106 | 0.13 | 42 | 0.11 | 15 | 2.6 |
| 060 | 3 | +30 | +25 | B ✔ | 234 | 0.22 | 47 | 0.20 | 56 | 2.6 |
| 300 | 15 | -30 | -25 | B ✔ | 46 | 0.11 | 12 | 0.06 | 31 | 2.6 |
| 090 | 8 | +30 | +0 | B ✔ | 78 | 0.07 | 10 | 0.07 | 30 | 1.5 |
| 270 | 15 | +30 | +25 | B ✔ | 42 | 0.05 | 9 | 0.05 | 28 | 1.3 |
| 120 | 3 | -30 | +0 | B ✔ | 270 | 0.28 | 42 | 0.23 | 65 | 0.3 |
| 240 | 8 | +30 | -25 | B ✔ | 101 | 0.11 | 14 | 0.10 | 20 | 0.3 |
| 150 | 15 | +30 | +0 | B ✔ | 73 | 0.06 | 8 | 0.06 | 28 | 0.3 |
| 210 | 3 | -30 | +25 | B ✔ | 366 | 0.23 | 30 | 0.15 | 37 | 0.3 |
| 170 | 8 | -30 | +0 | B ✔ | 160 | 0.12 | 19 | 0.12 | 48 | 0.1 |
| 190 | 15 | +30 | -25 | B ✔ | 85 | 0.07 | 9 | 0.07 | 13 | 0.1 |

### Tekne boyutu ve kütle bağımsızlığı

| Tekne | m (kg) | L (m) | V/U tam apaz | S1 tam apaz 8 m/s: süre (s) | S4 rüzgar üstü 8 m/s: süre / tack / gybe | S4 15 m/s: süre / tack / gybe | PID 090°, 15 m/s, 30 m sapma: aşım / RMS (m) |
|---|---|---|---|---|---|---|---|
| küçük (×0.75) | 21 | 1.88 | 0.978 | 126 | 262 / 1 / 1 | 139 / 0 / 1 | 0.06 / 0.05 |
| referans | 50 | 2.50 | 0.978 | 126 | 256 / 1 / 1 | 138 / 0 / 1 | 0.06 / 0.05 |
| büyük (×2) | 400 | 5.00 | 0.978 | 126 | 260 / 0 / 1 | 138 / 0 / 1 | 0.07 / 0.04 |
| hafif (25 kg) | 25 | 2.50 | 0.978 | 126 | 265 / 0 / 2 | 139 / 0 / 1 | 0.06 / 0.05 |
| ağır (150 kg) | 150 | 2.50 | 0.978 | 126 | 244 / 3 / 0 | 133 / 3 / 0 | 0.06 / 0.05 |
