### VMG optimumları

| Profil | Optimum rüzgar üstü TWA | VMG_up / V_w | Optimum rüzgar altı TWA | VMG_down / V_w | Tam pupa VMG / V_w |
|---|---|---|---|---|---|
| NACA 0012 | 30.5° | 0.672 | 150.0° | 0.849 | 0.580 |
| NACA 0015 | 30.5° | 0.638 | 150.0° | 0.909 | 0.650 |
| Kumaş Yelken | 50.5° | 0.541 | 150.0° | 0.797 | 0.740 |

### Tack eşiği

| Profil | Asgari rüzgar (m/s) | Tack başlangıç hızı (m/s) | Teorik v_min (m/s) |
|---|---|---|---|
| NACA 0012 | 5.2 | 3.16 | 3.03 |
| NACA 0015 | 5.4 | 3.12 | 3.03 |
| Kumaş Yelken | 6.5 | 3.14 | 3.03 |

### Uç durumlar

| Durum | Ulaştı | Süre (s) | Tack | Gybe | Sonuç |
|---|---|---|---|---|---|
| Başlangıç pruvası ölü açıda (000°), hedef Doğu 1 km, 8 m/s | HAYIR | 20 | 0 | 0 | Ölü açıda takıldı (in irons) – hız sıfır, dümen etkisiz |
| Başlangıç pruvası ölü açı sınırında (032°), hedef Kuzey 1 km, 8 m/s | EVET | 211 | 3 | 0 | Hedefe ulaşıldı |
| Başlangıç pruvası ters yönde (180°), hedef Kuzey 1 km, 8 m/s | EVET | 240 | 5 | 1 | Hedefe ulaşıldı |
| Tack yasak (yalnızca gybe), hedef Kuzey 1 km, 8 m/s | HAYIR | 7200 | 0 | 458 | Süre sınırı aşıldı |
| Tack yasak, varış yarıçapı 30 m, hedef Kuzey 1 km, 8 m/s | EVET | 252 | 0 | 2 | Hedefe ulaşıldı |
| Hafif rüzgar 2 m/s (tack fiziksel olarak imkânsız), hedef Kuzey 1 km | EVET | 976 | 0 | 2 | Hedefe ulaşıldı |
| Hedef başlangıçta (mesafe 0) | EVET | 0 | 0 | 0 | Hedefe ulaşıldı |
| Uzun menzil: Kuzey 10 km, 5 m/s, kumaş | EVET | 4485 | 2 | 3 | Hedefe ulaşıldı |
| Kısa süre limiti: Kuzey 1 km, 3 m/s, 300 s | HAYIR | 300 | 0 | 0 | Süre sınırı aşıldı |

### Tack yasak – varış yarıçapı duyarlılığı (NACA 0012, Kuzey 1 km)

| Varış yarıçapı (m) | U=3 m/s | U=5 m/s | U=8 m/s | U=12 m/s | U=15 m/s |
|---|---|---|---|---|---|
| *Dönüş çapı 2R (m)* | *16* | *16* | *24* | *36* | *45* |
| 10 | ✔ 653 s | ✔ 520 s | ✘ | ✘ | ✘ |
| 15 | ✔ 650 s | ✔ 392 s | ✘ | ✘ | ✘ |
| 20 | ✔ 648 s | ✔ 390 s | ✔ 269 s | ✘ | ✘ |
| 25 | ✔ 646 s | ✔ 389 s | ✔ 253 s | ✘ | ✘ |
| 30 | ✔ 644 s | ✔ 388 s | ✔ 252 s | ✔ 179 s | ✘ |
| 40 | ✔ 610 s | ✔ 368 s | ✔ 250 s | ✔ 177 s | ✔ 147 s |

### Senaryo matrisi: 72/72 başarılı

### Erişilebilirlik: 144/144 hedef başarılı
