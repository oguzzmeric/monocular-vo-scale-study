"""
VIO ilk adim: EuRoC V1_01_easy sensor verisinin tutarliligini dogrular.
Kontroller:
  - kare (cam0) ve IMU (imu0) ornekleme hizlari ve zaman araliklari
  - GT (state_groundtruth_estimate0) ornekleme hizi
  - kamera-IMU donusumu T_BS (sensor.yaml) ve donme/oteleme buyuklugu
  - duran kisimda ivmeolcer ortalamasi: yercekimi buyuklugu ~9.81 m/s^2 olmali
Kullanim (proje kokunden): python experiments/vio/check_euroc.py
"""
import csv
import glob
import os

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SEQ = os.path.join(ROOT, "data", "euroc", "mav0")


def load_csv(path):
    rows = []
    with open(path) as fh:
        for r in csv.reader(fh):
            if r and not r[0].startswith("#"):
                rows.append(r)
    return rows


def rate(ts_ns):
    d = np.diff(ts_ns) * 1e-9
    return 1.0 / np.median(d), d.min(), d.max()


def main():
    imu = np.array([[float(x) for x in r] for r in load_csv(os.path.join(SEQ, "imu0", "data.csv"))])
    gt = np.array([[float(x) for x in r[:8]] for r in load_csv(os.path.join(SEQ, "state_groundtruth_estimate0", "data.csv"))])
    cam_ts = np.array(sorted(int(os.path.basename(f).split(".")[0]) for f in glob.glob(os.path.join(SEQ, "cam0", "data", "*.png"))))

    print(f"kare sayisi: {len(cam_ts)}")
    fr, fmin, fmax = rate(cam_ts)
    print(f"kamera hizi: {fr:.2f} Hz (aralik {fmin*1e3:.1f}–{fmax*1e3:.1f} ms)")
    ir, imin, imax = rate(imu[:, 0].astype(np.int64))
    print(f"IMU hizi:    {ir:.1f} Hz (aralik {imin*1e3:.2f}–{imax*1e3:.2f} ms)")
    gr, gmin, gmax = rate(gt[:, 0].astype(np.int64))
    print(f"GT hizi:     {gr:.1f} Hz (aralik {gmin*1e3:.2f}–{gmax*1e3:.2f} ms)")
    print(f"zaman ortusmesi: kamera {cam_ts[0]} – {cam_ts[-1]}; IMU {int(imu[0,0])} – {int(imu[-1,0])}")

    with open(os.path.join(SEQ, "cam0", "sensor.yaml")) as fh:
        cs = yaml.safe_load(fh.read().replace("%YAML:1.0", ""))
    T = np.array(cs["T_BS"]["data"]).reshape(4, 4)
    R = T[:3, :3]
    print(f"T_BS (kamera->govde): oteleme = {np.round(T[:3,3], 3)} m; rotasyon ortogonal: {np.allclose(R @ R.T, np.eye(3), atol=1e-6)}, det={np.linalg.det(R):+.3f}")

    # Duran kisim: ilk 2 saniye hizi ~0 kabul; ivmeolcer ortalamasi yercekimi buyuklugu olmali
    t0 = imu[0, 0]
    still = imu[(imu[:, 0] - t0) < 2e9]
    acc = still[:, 4:7]
    print(f"ivmeolcer (ilk 2 s) ortalama |a| = {np.linalg.norm(acc.mean(0)):.3f} m/s^2 (beklenen ~9.81)")
    gyro_bias = still[:, 1:4].mean(0)
    print(f"jiroskop ortalama (ilk 2 s): {np.round(gyro_bias, 4)} rad/s")


if __name__ == "__main__":
    main()
