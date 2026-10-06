"""
Oturum_3 kayitli cikti canlandirmasi (40 sn video). Sol: kamera kareleri ve sayilar; sag: doner 3B harita.
3B harita yalnizca yatay duzlemi (x-y) gosterir: tahmin z=0 oldugu icin GT irtifasi cizilmez (yaniltici olmasin).
Tahmin SuperPoint+LG (secilen ayar, E-only), GT'ye Sim(3) hizali (sekil olcumu). Kayitli cikti, canli calisma degil.
Cikti: results/video/oturum3_3d.mp4 (git disi)
Kullanim (proje kokunden): python experiments/make_video_oturum3_3d.py
"""
import os
import pickle
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import evaluate_canonical as ev  # noqa: E402

ROOT = ev.ROOT
FRAMES = os.path.join(ROOT, "data_2025_oturum_3", "THYZ_2025_Oturum_3")
GT_CSV = "data_2025_oturum_3/THYZ_2025_Oturum_3_Translation.csv"
PKL = "data/grid_oturum3_sp.pkl"
KEY = (0.1, 0.3)
OUT_DIR = os.path.join(ROOT, "results", "video")
DURATION = 40.0
W, H = 1280, 720
CAM_W, CAM_H = 640, 360


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    gt = ev.load_gt(GT_CSV)
    steps = pickle.load(open(os.path.join(ROOT, PKL), "rb"))[KEY]
    fidx = [int(s["frame_name"].rsplit(".", 1)[0].split("_")[-1]) for s in steps]
    first = fidx[0] - (fidx[1] - fidx[0])
    fi = np.array([first] + fidx)
    G = np.array([gt[f] - gt[first] for f in fi])[:, :2]
    P = ev.integrate(steps)
    s, R, t = ev.umeyama(np.c_[P[:, :2], np.zeros(len(P))], np.c_[G, np.zeros(len(G))])
    Hh = ((s * (R @ np.c_[P[:, :2], np.zeros(len(P))].T)).T + t)[:, :2]
    err = np.linalg.norm(Hh - G, axis=1)
    n = len(steps)
    fps = n / DURATION

    lim = np.vstack([G, Hh])
    c = lim.mean(0)
    r = np.max(np.abs(lim - c)) * 1.15

    fig = plt.figure(figsize=(6.4, 7.2), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=-0.08, right=1.08, bottom=-0.1, top=1.0)
    ax.set_box_aspect((1, 1, 0.02))
    ax.dist = 6.2
    out_path = os.path.join(OUT_DIR, "oturum3_3d.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
    for k in range(n):
        ax.cla()
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(-0.5, 0.5)
        ax.set_axis_off()
        ax.view_init(elev=35, azim=(30 + 0.35 * k) % 360)
        ax.plot(G[: k + 2, 0], G[: k + 2, 1], np.zeros(k + 2), color="black", lw=2.2)
        ax.plot(Hh[: k + 2, 0], Hh[: k + 2, 1], np.zeros(k + 2), color="#d03030", lw=2.2)
        ax.scatter([G[k + 1, 0]], [G[k + 1, 1]], [0], color="black", s=40)
        ax.scatter([Hh[k + 1, 0]], [Hh[k + 1, 1]], [0], color="#d03030", s=40)
        ax.set_title("3B harita (x-y duzlemi, dondurulen gorunum)", fontsize=10)
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        right = cv2.cvtColor(np.ascontiguousarray(buf), cv2.COLOR_RGB2BGR)
        right = cv2.resize(right, (W - CAM_W, H))

        canvas = np.full((H, W, 3), 255, np.uint8)
        img = cv2.imread(os.path.join(FRAMES, steps[k]["frame_name"]))
        if img is not None:
            canvas[0:CAM_H, 0:CAM_W] = cv2.resize(img, (CAM_W, CAM_H))
        canvas[:, CAM_W:] = right
        cv2.putText(canvas, "Kamera (kayitli kare)", (10, CAM_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        stats = [
            f"Kare: {fi[k + 1]}   (kayitli cikti, canli degil)",
            "Tahmin: SuperPoint+LG, yalnizca E",
            f"Anlik sekil hatasi: {err[k + 1]:.1f} m",
            "Siyah: GT   Kirmizi: tahmin (GT'ye Sim(3) hizali)",
        ]
        for j, txt in enumerate(stats):
            cv2.putText(canvas, txt, (10, CAM_H + 40 + 30 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2)
        writer.write(canvas)
    writer.release()
    plt.close(fig)
    print(f"yazildi: {out_path} | kare: {n} | fps: {fps:.2f} | sure: {n/fps:.1f} sn")


if __name__ == "__main__":
    main()
