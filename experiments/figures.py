"""
Yayin gorselleri: tum ucuslarda ORB ve SuperPoint+LG karsilastirmasi.
Kanonik degerlendirme fonksiyonlarini (evaluate_canonical) kullanir; sayilar tabloyla birebir ayni.

Kullanim (proje kokunden):
    python experiments/figures.py

Cikti (results/figures/):
    overlay_<ucus>.png   : tum yorunge GT'ye Sim(3) ile hizali; ORB ve SP yan yana (sekil)
    sekil_cubuk.png      : ucus x on uc sekil hatasi
    yon_cubuk.png        : ucus x on uc yon hatasi (medyan)
Not: sekil hizalamasi GT'yi tum yorunge uzerinden kullanir (ideal olcum); gorsellerde bu belirtilir.
"""
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import evaluate_canonical as ev  # noqa: E402

ROOT = ev.ROOT
OUT = os.path.join(ROOT, "results", "figures")
FLIGHTS = ["2026", "oturum_3", "2024"]


def trajectory(steps, gt):
    fidx = [int(s["frame_name"].rsplit(".", 1)[0].split("_")[-1]) for s in steps]
    stride = fidx[1] - fidx[0] if len(fidx) > 1 else 5
    first = fidx[0] - stride
    fi = np.array([first] + fidx)
    G = np.array([gt[f] - gt[first] for f in fi])
    P = ev.integrate(steps)
    s, R, t = ev.umeyama(P, G)
    H = (s * (R @ P.T)).T + t
    return G, H


def main():
    os.makedirs(OUT, exist_ok=True)
    results = {}
    for flight, front, gt_path, pkl, key in ev.RUNS:
        gt = ev.load_gt(gt_path)
        steps = ev.load_steps(pkl, key)
        results[(flight, front)] = (ev.evaluate(steps, gt), trajectory(steps, gt))

    # 1) ucus basina overlay (ORB ve SP yan yana)
    for flight in FLIGHTS:
        fig, axes = plt.subplots(1, 2, figsize=(11, 6), sharey=True)
        for ax, front in zip(axes, ["ORB", "SP"]):
            res, (G, H) = results[(flight, front)]
            ax.plot(G[:, 0], G[:, 1], "k-", lw=2.5, label="GT")
            ax.plot(H[:, 0], H[:, 1], "-", color="tab:red", lw=1.2, label="tahmin (hizali)")
            name = "ORB" if front == "ORB" else "SuperPoint+LG"
            ax.set_title(f"{name}\nşekil hatası {res['sekil']:.1f} m", fontsize=11)
            ax.set_aspect("equal")
            ax.set_xlabel("x [m]")
            ax.legend(fontsize=8, loc="best")
        axes[0].set_ylabel("y [m]")
        fig.suptitle(f"{flight}: yörünge şekli (GT'ye Sim(3) hizalı, ideal ölçüm)", fontsize=12)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"overlay_{flight}.png"), dpi=130)
        plt.close(fig)

    # 2) sekil cubuk grafigi
    fig, ax = plt.subplots(figsize=(8, 4.5))
    width = 0.38
    x = np.arange(len(FLIGHTS))
    for j, front in enumerate(["ORB", "SP"]):
        vals = [results[(f, front)][0]["sekil"] for f in FLIGHTS]
        bars = ax.bar(x + (j - 0.5) * width, vals, width, label="ORB" if front == "ORB" else "SuperPoint+LG")
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 1, f"{v:.1f}", ha="center", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(FLIGHTS)
    ax.set_ylabel("şekil hatası [m] (düşük = iyi)")
    ax.set_title("Şekil hatası, seçilen ayarlar (GT hizalı, ideal ölçüm)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "sekil_cubuk.png"), dpi=130)
    plt.close(fig)

    # 3) yon cubuk grafigi
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for j, front in enumerate(["ORB", "SP"]):
        vals = [results[(f, front)][0]["yon"] for f in FLIGHTS]
        bars = ax.bar(x + (j - 0.5) * width, vals, width, label="ORB" if front == "ORB" else "SuperPoint+LG")
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.3, f"{v:.1f}°", ha="center", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(FLIGHTS)
    ax.set_ylabel("yön hatası, medyan [°]")
    ax.set_title("Yön hatası (hizalanmış yörünge hız yönü, GT'ye göre)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "yon_cubuk.png"), dpi=130)
    plt.close(fig)

    print("Yazildi:", ", ".join(sorted(os.listdir(OUT))))


if __name__ == "__main__":
    main()
