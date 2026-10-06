"""
refine_trajectory.py
=====================
Uretim pipeline'ini calistirip, PoseGraph.refine_with_persistent_map ile
GTSAM tabanli kalici-harita duzeltmesini uygular, sonucu uretimle
karsilastirir.

SADECE WSL/Linux'ta calisir -- GTSAM'in Windows'ta PyPI wheel'i yok.
Windows'ta normal main.py akisi bu dosyayi hic import etmez, gtsam'a
bagimli degildir.

Kullanim (WSL):
    source venv/bin/activate
    python3 refine_trajectory.py
"""

import logging
import sys
from pathlib import Path

import numpy as np

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logging.getLogger("core.matcher").setLevel(logging.WARNING)
logging.getLogger("core.motion_estimator").setLevel(logging.WARNING)

from utils.data_loader import DataLoader
from utils.camera_calibration import CameraCalibration
from core.feature_extractor import FeatureExtractor
from core.matcher import Matcher
from core.motion_estimator import MotionEstimator
from core.scale_recovery import ScaleRecovery
from core.pose_graph import PoseGraph

config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
print(f"config: {config_path}")

loader = DataLoader(config_path)
cam = CameraCalibration(loader)
extractor = FeatureExtractor(loader, cam)
matcher = Matcher(loader)
estimator = MotionEstimator(loader, cam)
scale_rec = ScaleRecovery(loader, cam)
pose_graph = PoseGraph(loader)

prev_features = None
for idx, name, frame in loader.frame_generator():
    curr = extractor.extract(frame, name)
    if prev_features is not None:
        mr = matcher.match(prev_features, curr)
        pose = estimator.estimate(mr)
        sr = scale_rec.recover(pose, name, match_result=mr)
        pose_graph.update(pose, sr, name)
    prev_features = curr

print(f"uretim pipeline tamamlandi: {len(pose_graph.trajectory)} kare islendi")

refined = pose_graph.refine_with_persistent_map(cam, extractor)
print(f"persistent-map BA tamamlandi.")

refined_lc = pose_graph.refine_with_loop_closure(cam, extractor, matcher, estimator)
print(f"loop closure + PGO tamamlandi.")

tag = Path(config_path).stem  # ör. "config" -> "config", "config_2024" -> "config_2024"
prod_path = f"data/trajectory_output_{tag}.csv"
ba_path = f"data/trajectory_output_persistent_ba_{tag}.csv"
lc_path = f"data/trajectory_output_loop_closure_{tag}.csv"
pose_graph.save_trajectory(prod_path)
pose_graph.save_trajectory(ba_path, trajectory=refined)
pose_graph.save_trajectory(lc_path, trajectory=refined_lc)
print(f"Kaydedildi: {prod_path}, {ba_path}, {lc_path}")


force_2d = bool(loader.config.get("evaluation", {}).get("force_2d", False))


def mean_error(points):
    errs = []
    for p in points:
        if not p.is_valid:
            continue
        gt = loader.get_ground_truth(p.frame_name)
        if gt is None:
            continue
        est_pos = p.position.copy()
        gt_pos = gt.as_vector()
        if force_2d:
            # GT'de Z yoksa (hep 0), BA'nin serbestce urettigi Z tahminini
            # de kiyaslamadan cikar -- yoksa var olmayan bir Z farkindan
            # metrik sisiyor (bkz. main.py'deki ayni sorun, force_2d fix'i)
            est_pos = est_pos.copy()
            est_pos[2] = 0.0
            gt_pos = gt_pos.copy()
            gt_pos[2] = 0.0
        errs.append(np.linalg.norm(est_pos - gt_pos))
    return (float(np.mean(errs)) if errs else float("nan")), len(errs)


prod_err, n1 = mean_error(pose_graph.trajectory)
ref_err, n2 = mean_error(refined)
lc_err, n3 = mean_error(refined_lc)

print()
print("=" * 60)
print("  HIZALANMAMIS mean (yarisma metrigi) -- ucu de PoseGraph'in")
print("  KENDI Sim(3) raporlama mekanizmasiyla hesaplandi")
print("=" * 60)
print(f"  URETIM (duzeltmesiz)           n={n1:4d}  : {prod_err:.2f} m")
print(f"  PERSISTENT-MAP BA              n={n2:4d}  : {ref_err:.2f} m")
print(f"  BA + LOOP CLOSURE              n={n3:4d}  : {lc_err:.2f} m")
