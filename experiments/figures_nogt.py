"""
GT gosterilmeden yorunge figurleri (guvenlik onlemi): yalnizca tahmin cizilir, GT cizgisi/noktasi yoktur.
Her panelin basliginda ve altinda dogruluk oranlari (sekil ATE, alt yol RPE, yon, gecerli) yazilir.
Tahmin, GT'ye Sim(3) ile hizalidir (sekil olcumu); bu bilgi figur altina yazilir.
Cikti: results/figures_nogt/*.png
"""
import os
import pickle
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import evaluate_canonical as ev  # noqa: E402

OUT = os.path.join(ev.ROOT, "results", "figures_nogt")


def estimate_aligned(steps, gt):
    fidx = [int(s["frame_name"].rsplit(".", 1)[0].split("_")[-1]) for s in steps]
    stride = fidx[1] - fidx[0] if len(fidx) > 1 else 5
    first = fidx[0] - stride
    fi = np.array([first] + fidx)
    G = np.array([gt[f] - gt[first] for f in fi])  # GT yalnizca hizalama icin
    P = ev.integrate(steps)
    s, R, t = ev.umeyama(P, G)
    return (s * (R @ P.T)).T + t, ev.evaluate(steps, gt)


def main():
    os.makedirs(OUT, exist_ok=True)
    runs = [("2026", "data/ground-truth.csv", "data/grid_2026_orb.pkl", (0.75, 0.45), "ORB"),
            ("2026", "data/ground-truth.csv", "data/grid_2026_sp.pkl", (0.1, 0.3), "SuperPoint+LG"),
            ("oturum_3", "data_2025_oturum_3/THYZ_2025_Oturum_3_Translation.csv", "data/grid_oturum3_orb.pkl", (0.75, 0.45), "ORB"),
            ("oturum_3", "data_2025_oturum_3/THYZ_2025_Oturum_3_Translation.csv", "data/grid_oturum3_sp.pkl", (0.1, 0.3), "SuperPoint+LG"),
            ("2024", "data_2024/ground-truth.csv", "data/grid_2024_orb.pkl", (0.75, 0.45), "ORB"),
            ("2024", "data_2024/ground-truth.csv", "data/grid_2024_sp.pkl", (0.1, 0.3), "SuperPoint+LG")]
    for flight in ["2026", "oturum_3", "2024"]:
        fig, axes = plt.subplots(1, 2, figsize=(11, 6.2))
        for ax, (fl, gtp, pkl, key, name) in zip(axes, [r for r in runs if r[0] == flight]):
            gt = ev.load_gt(gtp)
            steps = pickle.load(open(os.path.join(ev.ROOT, pkl), "rb"))[key]
            H, r = estimate_aligned(steps, gt)
            ax.plot(H[:, 0], H[:, 1], color="#2b6cb0" if name == "ORB" else "#c53030", lw=1.6)
            ax.set_aspect("equal")
            ax.set_title(name, fontsize=12)
            ax.set_xlabel("x [m]")
            ax.text(0.02, 0.02,
                    f"şekil ATE {r['sekil']:.1f} m\nalt yol RPE {r['rpe']:.1f} %\n"
                    f"yön (medyan) {r['yon']:.1f}°\ngeçerli {100*r['gecerli']:.0f} %",
                    transform=ax.transAxes, fontsize=9, va="bottom",
                    bbox=dict(boxstyle="round", fc="white", ec="#ccc", alpha=0.9))
        axes[0].set_ylabel("y [m]")
        fig.suptitle(f"{flight}: tahmin edilen yörünge (GT gösterilmiyor; ölçüm GT'ye Sim(3) hizalı)", fontsize=11)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"tahmin_{flight}.png"), dpi=130)
        plt.close(fig)
    print("yazildi:", sorted(os.listdir(OUT)))


if __name__ == "__main__":
    main()
