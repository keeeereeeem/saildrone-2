"""
Ortak test tekneleri. Tekne ölçüleri henüz belli olmadığından testler tek bir tekneye bağlı kalmaz: aynı testler
geometrik olarak küçültülmüş / büyütülmüş ve farklı kütleli teknelerde de geçmelidir (boyutlandırma sonraki aşama).
"""
from dataclasses import replace

import saildrone_model as m

P = m.PHYS


def scaled(k, phys=P, **kw):
    """Geometrik ölçekleme: uzunluklar ×k, alanlar ×k², kütle ×k³ (aynı biçimde küçük / büyük tekne)."""
    return replace(phys, mass=phys.mass * k ** 3, hull_length=phys.hull_length * k,
                   wetted_area=phys.wetted_area * k * k, sail_area=phys.sail_area * k * k,
                   board_area=phys.board_area * k * k, hull_side_area=phys.hull_side_area * k * k,
                   rudder_area=phys.rudder_area * k * k, rudder_arm=phys.rudder_arm * k,
                   ce_arm=phys.ce_arm * k, clr_arm=phys.clr_arm * k, beam=phys.beam * k, heel_arm=phys.heel_arm * k, **kw)


BOATS = {
    "küçük (×0.75)": scaled(0.75),
    "referans": P,
    "büyük (×2)": scaled(2.0),
    "hafif (25 kg)": replace(P, mass=25.0),
    "ağır (150 kg)": replace(P, mass=150.0),
}
