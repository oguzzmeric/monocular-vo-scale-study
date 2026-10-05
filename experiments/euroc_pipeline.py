"""
EuRoC V1_01_easy: PROJE HATTI ile monoküler adım ve şekil değerlendirmesi.

Ön uç ve poz tahmini projenin kendi modülleridir (FeatureExtractor, Matcher, MotionEstimator);
yalnızca veri yükleyici EuRoC'a uyarlanmıştır. Ölçek bu testte kullanılmaz; birim adım
(u, R) kaydedilir ve şekil, GT'ye Sim(3) hizalamasıyla ölçülür (canonical ölçütle aynı).

Kullanım (proje kökünden):
    python experiments/euroc_pipeline.py superpoint     # SuperPoint+LG, E-only (mimari karar)
    python experiments/euroc_pipeline.py orb            # ORB, aynı pipeline (Lowe 0.75)
"""
import csv
import glob
import os
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from utils.data_loader import DataLoader  # noqa: E402
from utils.camera_calibration import CameraCalibration  # noqa: E402
from core.feature_extractor import FeatureExtractor  # noqa: E402
from core.matcher import Matcher  # noqa: E402
from core.motion_estimator import MotionEstimator  # noqa: E402

SEQ = os.path.join(ROOT, "data", "euroc", "mav0")
STRIDE = 2  # 20 Hz -> 10 Hz


class EurocLoader(DataLoader):
    """DataLoader'in EuRoC uyarlaması: kare listesi, görüntü okuma, kalibrasyon, yapılandırma."""

    def __init__(self, cfg: dict, frame_paths: list):
        # Temel __init__ config dosyası ister; bilerek atlanır.
        self.config = cfg
        self._frame_list_override = frame_paths

    @property
    def frame_list(self):
        return self._frame_list_override

    @property
    def total_frames(self):
        return len(self._frame_list_override)

    def load_frame(self, frame_path):
        img = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"görüntü okunamadı: {frame_path}")
        return img

    def get_detections(self, frame_name):
        return []

    def get_ground_truth(self, frame_name):
        return None

    def get_camera_config(self):
        return self.config["camera_rgb"]

    def get_feature_config(self):
        return self.config["features"]

    def get_semantic_config(self):
        return self.config["semantic"]

    def get_hybrid_config(self):
        return self.config["hybrid"]


def build_config(front):
    base = "config_superpoint_lightglue_test.yaml" if front == "superpoint" else "config.yaml"
    with open(os.path.join(ROOT, base), encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    with open(os.path.join(SEQ, "cam0", "sensor.yaml")) as fh:
        s = yaml.safe_load(fh.read().replace("%YAML:1.0", ""))
    fu, fv, cu, cv_ = s["intrinsics"]
    d = list(s["distortion_coefficients"]) + [0.0]  # 4 -> 5 katsayi (k3 = 0)
    cfg["camera_rgb"] = dict(fx=fu, fy=fv, cx=cu, cy=cv_, distortion_coefficients=d,
                             original_width=752, original_height=480)
    cfg["features"]["homography_score_ratio_threshold"] = 10.0  # yalniz E (mimari karar)
    if front == "orb":
        cfg["features"]["detector_type"] = "orb"
        cfg["features"]["matcher_type"] = "classical"
        cfg["features"]["descriptor_norm"] = "hamming"
        cfg["features"]["lowe_ratio"] = 0.75
    cfg["semantic"] = cfg.get("semantic", {})
    cfg["hybrid"] = cfg.get("hybrid", {})
    return cfg


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
    front = sys.argv[1] if len(sys.argv) > 1 else "superpoint"
    cfg = build_config(front)
    files = sorted(glob.glob(os.path.join(SEQ, "cam0", "data", "*.png")))[::STRIDE]
    L = EurocLoader(cfg, [Path(os.path.abspath(f)) for f in files])
    C = CameraCalibration(L)
    E = FeatureExtractor(L, C)
    M = Matcher(L)
    Es = MotionEstimator(L, C)

    steps = []
    prev = None
    names = []
    for idx, name, frame in L.frame_generator():
        names.append(name)
        cur = E.extract(frame, name)
        if prev is not None:
            mr = M.match(prev, cur)
            pose = Es.estimate(mr)
            valid = bool(pose.is_valid)
            steps.append(dict(frame_name=name, valid=valid,
                              u=(None if not valid else pose.t.ravel().copy()),
                              R=(None if not valid else pose.R.copy()),
                              scale=1.0, mode="unit", n_inl=int(pose.inlier_count),
                              matrix_type=str(pose.matrix_type)))
        prev = cur
    pickle.dump(steps, open(os.path.join(ROOT, "data", f"euroc_{front}_steps.pkl"), "wb"))

    # Olcum: sekil (GT Sim3) ve adim basina ideal olcek (GT adim / |u|)
    gt_ts, gt_P = load_gt()
    img_ts = np.array([int(n.split(".")[0]) for n in names])
    G = gt_P[[np.argmin(np.abs(gt_ts - t)) for t in img_ts]]
    G = G - G[0]
    p = np.zeros(3); Rw = np.eye(3); P = [p.copy()]
    for k, s in enumerate(steps, start=1):
        if s["valid"]:
            p = p + Rw @ s["u"]
            Rw = Rw @ s["R"]
        P.append(p.copy())
    P = np.array(P)
    sc, R, t = umeyama(P, G)
    H = (sc * (R @ P.T)).T + t
    sekil = float(np.sqrt(np.mean(np.sum((H - G) ** 2, axis=1))))
    yol = float(np.linalg.norm(np.diff(G, axis=0), axis=1).sum())
    ideal = np.array([np.linalg.norm(G[k] - G[k - 1]) for k, s in enumerate(steps, start=1) if s["valid"]])
    ideal = ideal[ideal > 0.01]
    print(f"[{front}] kare={len(files)} gecerli_adim={sum(s['valid'] for s in steps)}/{len(steps)}")
    print(f"  sekil (Sim3, tum yorunge) = {sekil:.3f} m  yol={yol:.1f} m  %{100*sekil/yol:.1f}")
    print(f"  ideal olcek (GT adim/|u|): medyan={np.median(ideal):.4f} m  log-std={np.std(np.log(ideal)):.3f}")
    print(f"  Sim3 olcek s = {sc:.4f}")


if __name__ == "__main__":
    main()
