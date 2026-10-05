"""
Canlandirma verisi: 2026 icin ORB ve SuperPoint+LG kayitli adim ciktilarindan iki yorunge uretir
ve tek bir HTML sayfasina gomer (experiments/replay_2026.html).

Iki gorunum:
  sekil    : tum yorunge GT'ye Sim(3) ile hizali (ideal, sekil olcumu ile ayni)
  uretim   : warmup (kare < 450) Sim(3) ile hizali, GT yok (uretim tarzi cikti)
Not: Bu bir kayitli ciktinin canlandirmasidir; sistem bu sayfada canli calismaz.

Kullanim (proje kokunden): python experiments/replay_export.py
"""
import json
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import evaluate_canonical as ev  # noqa: E402

ROOT = ev.ROOT
OUT_DIR = os.path.join(ROOT, "results", "replay")
WARM = 450


def trajectories(steps, gt):
    fidx = [int(s["frame_name"].rsplit(".", 1)[0].split("_")[-1]) for s in steps]
    stride = fidx[1] - fidx[0] if len(fidx) > 1 else 5
    first = fidx[0] - stride
    fi = np.array([first] + fidx)
    G = np.array([gt[f] - gt[first] for f in fi])
    P = ev.integrate(steps)
    s, R, t = ev.umeyama(P, G)
    H_shape = (s * (R @ P.T)).T + t
    w = fi < WARM
    s2, R2, t2 = ev.umeyama(P[w], G[w])
    H_prod = (s2 * (R2 @ P.T)).T + t2
    return fi, G, H_shape, H_prod


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    gt = ev.load_gt("data/ground-truth.csv")
    out = {"flight": "2026", "warm_frame": WARM, "series": {}}
    for front, pkl, key in [("ORB", "data/grid_2026_orb.pkl", (0.75, 0.45)),
                            ("SuperPoint+LG", "data/grid_2026_sp.pkl", (0.1, 0.3))]:
        steps = pickle.load(open(os.path.join(ROOT, pkl), "rb"))[key]
        fi, G, Hs, Hp = trajectories(steps, gt)
        out["series"][front] = {
            "shape": np.round(Hs[:, :2], 2).tolist(),
            "prod": np.round(Hp[:, :2], 2).tolist(),
            "err_shape": np.round(np.linalg.norm(Hs - G, axis=1), 2).tolist(),
            "err_prod": np.round(np.linalg.norm(Hp - G, axis=1), 2).tolist(),
        }
        if "gt" not in out:
            out["gt"] = np.round(G[:, :2], 2).tolist()
            out["frames"] = [int(f) for f in fi]
    data_path = os.path.join(OUT_DIR, "replay_2026.json")
    with open(data_path, "w", encoding="utf-8") as fh:
        json.dump(out, fh)
    print("yazildi:", data_path)


if __name__ == "__main__":
    main()
