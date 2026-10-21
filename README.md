# Saildrone / Yelkenli Katamaran Simülatörü

Canlı uygulama: https://keeeereeeem-saildrone-2-app-2uzfnl.streamlit.app/

Kaynak kod: https://github.com/keeeereeeem/saildrone-2

```bash
pip install -r requirements.txt
streamlit run app.py
```

* Tek yelken modeli: **Kumaş Yelken (Cambered / Klasik)**
* `saildrone_model.py` – fizik modeli, `navigation.py` – başlangıç → ara duraklar → hedef otonom navigasyon ve açı takip kaydı
* `components/nav_player/` – Hedefe Navigasyon haritası (sürükle-bırak, ara durak, hızlandırılmış oynatma, açı takip tablosu)
* Testler: `pip install pytest` ve ardından `python -m pytest -q`
* Matematiksel model, navigasyon algoritması ve test sonuçları: [RAPOR.md](RAPOR.md) (Word: `RAPOR.docx`)
