"""
EuRoC V1_01_easy: monoküler adım tabanı ve ölçek değişkenliği (ön test, IMU öncesi).

Ne yapar:
  1) cam0 görüntüleri undistort edilir (sensor.yaml: pinhole + radtan).
  2) ORB + BF(Hamming) + Lowe 0.75; E matrisi (RANSAC) ve recoverPose ile birim
     öteleme u ve R çıkarılır.
  3) GT (Vicon) konumları görüntü zamanlarına en yakın örnekle eşleştirilir.
  4) İki ölçüt:
     - sekil: tüm yörünge GT'ye Sim(3) ile hizalanır; kalan RMS (ideal ölçüm).
     - olcek_adim: her adımda GT adım uzunluğu / |u| = "ideal ölçek"; değişkenliği (log-std).
Not: Bu bağımsız bir taban ölçümdür (proje hattı değil); amaç EuRoC'ta sorunun ölçeği ve şekli nasıl gösterdiğini görmek.

Kullanım (proje kökünden):  python experiments/euroc_scale_baseline.py
"""
import csv
import glob
import os
import sys

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEQ = os.path.join(ROOT, "data", "euroc", "mav0")
STRIDE = 2  # 20 Hz -> 10 Hz


def load_cam():
    with open(os.path.join(SEQ, "cam0", "sensor.yaml")) as fh:
        s = yaml.safe_load(fh.read().replace("%YAML:1.0", ""))
    fu, fv, cu, cv_ = s["intrinsics"]
    K = np.array([[fu, 0, cu], [0, fv, cv_], [0, 0, 1.0]])
    D = np.array(s["distortion_coefficients"], dtype=np.float64)
    return K, D


def load_gt():
    ts, P = [], []
    with open(os.path.join(SEQ, "state_groundtruth_estimate0", "data.csv")) as fh:
        for r in csv.reader(fh):
            if not r or r[0].startswith("#"):
                continue
            ts.append(int(r[0]))
            P.append([float(r[1]), float(r[2]), float(r[3])])
    return np.array(ts), np.array(P)


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    U, D, Vt = np.linalg.svd(dc.T @ sc / len(src))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / ((sc ** 2).sum() / len(src))
    return s, R, mu_d - s * R @ mu_s


def main():
    K, D = load_cam()
    files = sorted(glob.glob(os.path.join(SEQ, "cam0", "data", "*.png")))
    files = files[::STRIDE]
    img_ts = np.array([int(os.path.basename(f).split(".")[0]) for f in files])
    gt_ts, gt_P = load_gt()

    orb = cv2.ORB_create(2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)

    prev = None
    U, Rs, step_gt, valid = [], [], [], []
    for i, f in enumerate(files):
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        g = cv2.undistort(g, K, D)
        kp, des = orb.detectAndCompute(g, None)
        cur = (kp, des)
        ok = False
        if prev is not None and prev[1] is not None and des is not None:
            knn = bf.knnMatch(prev[1], des, k=2)
            good = [m for p in knn if len(p) == 2 for m, n in [p] if m.distance < 0.75 * n.distance]
            if len(good) >= 30:
                a = np.float32([prev[0][m.queryIdx].pt for m in good])
                b = np.float32([kp[m.trainIdx].pt for m in good])
                E, mask = cv2.findEssentialMat(a, b, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                if E is not None and E.shape == (3, 3):
                    n_in, R, t, _ = cv2.recoverPose(E, a, b, K, mask=mask)
                    if n_in >= 20:
                        ok = True
                        U.append(t.ravel())
                        Rs.append(R)
                        j = np.argmin(np.abs(gt_ts - img_ts[i]))
                        k = np.argmin(np.abs(gt_ts - img_ts[i - 1]))
                        step_gt.append(np.linalg.norm(gt_P[j] - gt_P[k]))
                        valid.append(True)
        if not ok:
            U.append(np.zeros(3)); Rs.append(np.eye(3)); step_gt.append(0.0); valid.append(False)
        prev = cur

    U = np.array(U); step_gt = np.array(step_gt); valid = np.array(valid)
    # yorunge: uretimdeki birlesim (t_world += R_world t_local; R_world *= R_local)
    p = np.zeros(3); Rw = np.eye(3); P = [p.copy()]
    for k in range(1, len(U)):  # k=0 ilk kare: adim yok
        if valid[k]:
            p = p + Rw @ U[k]
            Rw = Rw @ Rs[k]
        P.append(p.copy())
    P = np.array(P)  # uzunluk = kare sayisi
    # GT, goruntu zamanlarina gore
    idx = [np.argmin(np.abs(gt_ts - t)) for t in img_ts]
    G = gt_P[idx]
    G = G - G[0]
    sc, R, t = umeyama(P, G)
    H = (sc * (R @ P.T)).T + t
    sekil = float(np.sqrt(np.mean(np.sum((H - G) ** 2, axis=1))))

    m = valid & (step_gt > 0.01)
    m[0] = False
    ratio = step_gt[m]  # |u| = 1 oldugu icin GT adim = ideal olcek
    logstd = float(np.std(np.log(ratio)))
    print(f"kareler={len(files)} gecerli_adim={valid.sum()}/{len(U)}")
    print(f"sekil hatasi (Sim3, tum yorunge) = {sekil:.3f} m   (yol = {step_gt.sum():.2f} m, %{100*sekil/step_gt.sum():.1f})")
    print(f"ideal olcek (GT adim, |u|=1): medyan={np.median(ratio):.4f} m  log-std={logstd:.3f}  n={len(ratio)}")
    print(f"olcek s (Sim3 yorunge)={sc:.3f}")


if __name__ == "__main__":
    main()
