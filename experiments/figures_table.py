"""
Sonuc tablosu gorseli (LinkedIn Article gibi tablo desteklemeyen yerler icin).
Sayilar results/canonical_table.csv'den okunur.

Kullanim (proje kokunden):
    python experiments/figures_table.py

Cikti: results/figures/sonuc_tablosu.png
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV = os.path.join(ROOT, "results", "canonical_table.csv")
OUT = os.path.join(ROOT, "results", "figures", "sonuc_tablosu.png")
FLIGHT_LABEL = {"2026": "2026", "oturum_3": "oturum_3", "2024": "2024 (yalnızca XY)"}


def main():
    rows = {}
    with open(CSV, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows[(r["flight"], r["front"])] = r
    header = ["uçuş", "ön uç", "yön hatası (°)", "alt yol RPE (%)", "şekil ATE (m)", "konum RMSE (m)"]
    body = []
    for f in ["2026", "oturum_3", "2024"]:
        for front, name in [("ORB", "ORB"), ("SP", "SuperPoint+LG")]:
            r = rows[(f, front)]
            body.append([FLIGHT_LABEL[f], name, f"{float(r['yon']):.1f}",
                         f"{float(r['rpe']):.1f}", f"{float(r['sekil']):.1f}", f"{float(r['konum']):.1f}"])

    fig, ax = plt.subplots(figsize=(11, 4.2))
    ax.axis("off")
    table = ax.table(cellText=body, colLabels=header, loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1, 1.6)
    for (row, col), cell in table.get_celld().items():
        if row == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#e8eef7")
        if row > 0 and row % 2 == 0:
            cell.set_facecolor("#f7f7f7")
    ax.set_title("Seçilen ayarlarla sonuçlar (yön hatası ana ölçüt; konum RMSE ikincil)", fontsize=12, pad=12)
    fig.tight_layout()
    fig.savefig(OUT, dpi=150)
    plt.close(fig)
    print("Yazildi:", OUT)


if __name__ == "__main__":
    main()
