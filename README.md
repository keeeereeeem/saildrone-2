# Saildrone / Yelkenli Katamaran Simülatörü

Canlı demo: https://saildronecatamaransim.streamlit.app/

```bash
pip install -r requirements.txt
streamlit run app.py
```

* `saildrone_model.py` – fizik modeli, `navigation.py` – başlangıç → hedef otonom navigasyon
* Testler: `pip install pytest` ve ardından `python -m pytest -q`
* Hata analizi, matematiksel modeller ve navigasyon test sonuçları: [RAPOR.md](RAPOR.md)
