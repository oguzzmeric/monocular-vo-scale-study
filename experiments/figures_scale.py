"""
Olcek orani (s) grafigi: ucus x on uc. Sayilar results/canonical_table.csv'den okunur
(GT gerektirmez; tabloyla birebir ayni kaynak).

Kullanim (proje kokunden):
    python experiments/figures_scale.py

Cikti: results/figures/olcek_cubuk.png
Not: s = 1 ise tahmin edilen ölçek GT ile uyumlu demektir; cizgi bunu gosterir.
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV = os.path.join(ROOT, "results", "canonical_table.csv")
OUT = os.path.join(ROOT, "results", "figures", "olcek_cubuk.png")
FLIGHTS = ["2026", "oturum_3", "2024"]


def main():
    rows = {}
    with open(CSV, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows[(r["flight"], r["front"])] = float(r["olcek"])

    fig, ax = plt.subplots(figsize=(8, 4.5))
    width = 0.38
    x = np.arange(len(FLIGHTS))
    for j, front in enumerate(["ORB", "SP"]):
        vals = [rows[(f, front)] for f in FLIGHTS]
        label = "ORB" if front == "ORB" else "SuperPoint+LG"
        bars = ax.bar(x + (j - 0.5) * width, vals, width, label=label)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.02, f"{v:.2f}", ha="center", fontsize=9)
    ax.axhline(1.0, color="gray", lw=1, ls="--")
    ax.set_xticks(x)
    ax.set_xticklabels(FLIGHTS)
    ax.set_ylabel("ölçek oranı s [-]")
    ax.set_title("Sim(3) ölçek oranı, seçilen ayarlar (1 = GT ile uyum)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT, dpi=130)
    plt.close(fig)
    print("Yazildi:", OUT)


if __name__ == "__main__":
    main()
