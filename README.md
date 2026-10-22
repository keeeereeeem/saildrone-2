# Saildrone / Yelkenli Katamaran Simülatörü

Canlı uygulama: https://keeeereeeem-saildrone-2-app-2uzfnl.streamlit.app/

Kaynak kod: https://github.com/keeeereeeem/saildrone-2

```bash
pip install -r requirements.txt
streamlit run app.py
```

* Otonom yelkenli katamaran (≈ 50 kg, ölçüler henüz belli değil) – üç ekran tek bir **kuvvet tabanlı (VPP)** fizik modelini kullanır: görünür rüzgarda kumaş yelken kuvvetleri, gövde + yan kayma + dümen direnci, dümen stall'u, Nomoto dönüş ataleti, orsaya kaçma
* Model boyuttan bağımsızdır: hız / rüzgar oranı, bom ve yan kayma açısı rüzgar hızından, tekne boyutundan ve kütleden bağımsızdır; boyuta bağlı fizik (devrilme sınırı, dalga direnci) kenar çubuğunda "Boyutlandırma" altında, varsayılan kapalı
* `saildrone_model.py` – fizik modeli, `navigation.py` – başlangıç → ara duraklar → hedef otonom navigasyon ve açı takip kaydı, `line_follow.py` – A→B düz rota kaskad PID
* `components/nav_player/`, `components/line_player/` – harita, oynatma, açı takip tablosu, PID ve kuvvet panelleri
* Testler: `pip install pytest` ve ardından `python -m pytest -q` (testler beş farklı tekne boyutu / kütlesiyle çalışır)
* Matematiksel model, navigasyon algoritması ve test sonuçları: [RAPOR.md](RAPOR.md) (Word: `RAPOR.docx`)
