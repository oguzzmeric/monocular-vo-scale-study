"""
Uzun-bazli (long-baseline) track + GTSAM motion-only BA.

bacheck_gtsam.py 6 kareli pencerede, ardisik ORB eslesmelerini
zincirleyerek track kuruyordu -- bu yontem uzun pencerede (15-30 kare)
CALISMAZ: her adimda bagimsiz yeniden eslesme+RANSAC gerektigi icin
hayatta kalma orani carpimsal dusuyor (olcum: ~%68/adim -> 20 karede
~%0.02 -- pratikte sifir track).

Bunun yerine KLT (Lucas-Kanade optik akis) kullaniyoruz: bir noktayi
pencerenin ILK karesinde bulup, sonraki her karede yeniden eslestirmeye
CALISMADAN, goruntu gradyanlariyla DOGRUDAN takip ediyoruz. Ileri-geri
(forward-backward) tutarlilik kontroluyle sessizce kayan noktalar
eleniyor.

Ayni GTSAM motion-only BA cozucusu (bacheck_gtsam.py'den, degismedi --
tek degisken track uretim yontemi ve pencere buyuklugu) tekrar
kullaniliyor -- boylece "uzun track yardimci oluyor mu" sorusuna tek
degiskenli bir cevap aliyoruz.
"""

import csv
import logging
import numpy as np
import cv2
import gtsam
from gtsam import Pose3, Rot3, Point3, Cal3_S2
from gtsam.symbol_shorthand import X, L

logging.basicConfig(level=logging.WARNING)

from utils.data_loader import DataLoader
from utils.camera_calibration import CameraCalibration
from core.feature_extractor import FeatureExtractor
from core.matcher import Matcher
from core.motion_estimator import MotionEstimator
from core.scale_recovery import ScaleRecovery
from core.pose_graph import PoseGraph

MIN_WINDOW = 15     # SABIT 15 kare -- adaptif esik denemeleri (130/200px) ikisi de
MAX_WINDOW = 15     # bundan daha kotu cikti, en iyi olculen sonuc bu sabit deger.
DISPLACEMENT_STOP_PX = 1e9   # etkisiz birakildi
MIN_ALIVE_TRACKS = 0         # etkisiz birakildi
REPLENISH_THRESHOLD = 200    # aktif track sayisi bunun altina dusunce yeni kose ekle
REPLENISH_EXCLUDE_RADIUS = 15  # yeni kose, var olan bir track'e bu kadar yakin olmasin
MAX_TRACKS_PER_WINDOW = 150
PARALLAX_COS_THRESHOLD = 0.99998
PIXEL_NOISE_SIGMA = 1.5
ANCHOR_SIGMA = 1e-6
KLT_WIN = (63, 63)
KLT_MAX_LEVEL = 6
KLT_FB_THRESHOLD = 1.5   # ileri-geri piksel hatasi esigi

loader = DataLoader("config.yaml")
cam = CameraCalibration(loader)
extractor = FeatureExtractor(loader, cam)
matcher = Matcher(loader)
estimator = MotionEstimator(loader, cam)
scale_rec = ScaleRecovery(loader, cam)
pose_graph = PoseGraph(loader)

K = cam.K
F_FOCAL = float(np.sqrt(cam.fx * cam.fy))
gtsam_K = Cal3_S2(cam.fx, cam.fy, 0.0, cam.cx, cam.cy)

# --------------------------------------------------------------------
# 1) uretim pipeline -- her adimin R_local/t_local'i
# --------------------------------------------------------------------
steps = []
prev_features = None

for idx, name, frame in loader.frame_generator():
    curr = extractor.extract(frame, name)
    if prev_features is not None:
        mr = matcher.match(prev_features, curr)
        pose = estimator.estimate(mr)
        sr = scale_rec.recover(pose, name, match_result=mr)
        tp = pose_graph.update(pose, sr, name)
        steps.append(dict(
            frame_name=name,
            R_local=(pose.R.copy() if pose.is_valid else np.eye(3)),
            t_local=(sr.t_scaled.flatten().copy() if (pose.is_valid and sr.is_valid) else np.zeros(3)),
        ))
    prev_features = curr

print(f"toplam adim = {len(steps)}")


def chain(local_rotations, local_translations):
    N = len(local_rotations)
    R_world = [np.eye(3)]
    t_world = [np.zeros(3)]
    for i in range(N):
        R_world.append(R_world[-1] @ local_rotations[i])
        t_world.append(t_world[-1] + R_world[-2] @ local_translations[i])
    return R_world, t_world


R_local_all = [s["R_local"] for s in steps]
t_local_all = [s["t_local"] for s in steps]
R_world_prod, t_world_prod = chain(R_local_all, t_local_all)


def clean_gray_and_mask(frame_bgr, frame_name):
    clean = cam.undistort(frame_bgr)
    gray = cv2.cvtColor(clean, cv2.COLOR_BGR2GRAY)
    detections = loader.get_detections(frame_name)
    mask = extractor._build_semantic_mask(gray.shape, detections)
    return gray, mask


def build_long_tracks(frame_paths, frame_names):
    """
    KLT ile pencerenin ILK karesinden baslar. Aktif track sayisi
    REPLENISH_THRESHOLD'un altina dusunce, o karede YENI kose noktalari
    bulup ekler (var olan track'lere yakin olmayacak sekilde) -- boylece
    pencere ortasinda "besleme" yapilir, sadece basta bulunan koselere
    bagli kalinmaz.

    Her track {'start': pencere-ici baslangic kare indeksi,
               'obs': [(x,y), ...] -- start'tan itibaren ardisik gozlemler}
    seklinde -- boylece pencerenin ortasinda baslayan bir track de
    GTSAM'e dogru kareye bagli bir faktor olarak eklenebilir.

    Doner: (kullanilan_kare_sayisi, tamamlanmis_track_listesi)
    """
    gray0, mask0 = clean_gray_and_mask(loader.load_frame(frame_paths[0]), frame_names[0])
    pts0 = cv2.goodFeaturesToTrack(
        gray0, maxCorners=400, qualityLevel=0.01, minDistance=12, mask=mask0)
    if pts0 is None or len(pts0) == 0:
        return 1, []

    lk_params = dict(winSize=KLT_WIN, maxLevel=KLT_MAX_LEVEL,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    active = [{"start": 0, "obs": [tuple(p[0])]} for p in pts0]
    finished = []

    gray_prev = gray0
    used = 0
    max_i = min(MAX_WINDOW, len(frame_paths)) - 1

    for i in range(1, max_i + 1):
        gray_curr, mask_curr = clean_gray_and_mask(loader.load_frame(frame_paths[i]), frame_names[i])
        used = i

        if active:
            pts_in = np.array([a["obs"][-1] for a in active], dtype=np.float32).reshape(-1, 1, 2)
            pts_curr, st_fwd, _ = cv2.calcOpticalFlowPyrLK(gray_prev, gray_curr, pts_in, None, **lk_params)
            pts_back, st_bwd, _ = cv2.calcOpticalFlowPyrLK(gray_curr, gray_prev, pts_curr, None, **lk_params)
            fb_err = np.linalg.norm((pts_in - pts_back).reshape(-1, 2), axis=1)
            ok = (st_fwd.flatten() == 1) & (st_bwd.flatten() == 1) & (fb_err < KLT_FB_THRESHOLD)

            new_active = []
            for j, a in enumerate(active):
                if ok[j]:
                    a["obs"].append(tuple(pts_curr[j, 0, :]))
                    new_active.append(a)
                elif len(a["obs"]) >= 2:
                    finished.append(a)
            active = new_active

        gray_prev = gray_curr

        # besleme: aktif track azalinca bu karede yeni kose ekle
        if len(active) < REPLENISH_THRESHOLD:
            excl_mask = mask_curr.copy()
            for a in active:
                x, y = a["obs"][-1]
                cv2.circle(excl_mask, (int(round(x)), int(round(y))), REPLENISH_EXCLUDE_RADIUS, 0, -1)
            n_needed = 400 - len(active)
            if n_needed > 0:
                new_pts = cv2.goodFeaturesToTrack(
                    gray_curr, maxCorners=n_needed, qualityLevel=0.01, minDistance=12, mask=excl_mask)
                if new_pts is not None:
                    for p in new_pts:
                        active.append({"start": i, "obs": [tuple(p[0])]})

        if not active and i < max_i:
            break

    for a in active:
        if len(a["obs"]) >= 2:
            finished.append(a)

    n_frames_used = used + 1
    return n_frames_used, finished


def triangulate_multiview(Ps, pts):
    rows = []
    for P, pt in zip(Ps, pts):
        rows.append(pt[0] * P[2] - P[0])
        rows.append(pt[1] * P[2] - P[1])
    A = np.array(rows)
    _, _, Vt = np.linalg.svd(A)
    Xh = Vt[-1]
    if abs(Xh[3]) < 1e-9:
        return None
    return Xh[:3] / Xh[3]


def triangulate(P1, P2, pt1, pt2):
    A = np.array([
        pt1[0] * P1[2] - P1[0], pt1[1] * P1[2] - P1[1],
        pt2[0] * P2[2] - P2[0], pt2[1] * P2[2] - P2[1],
    ])
    _, _, Vt = np.linalg.svd(A)
    Xh = Vt[-1]
    if abs(Xh[3]) < 1e-9:
        return None
    return Xh[:3] / Xh[3]


def projection_matrix(R_w, t_w):
    Rt = R_w.T
    return K @ np.hstack([Rt, -Rt @ t_w.reshape(3, 1)])


def parallax_ok(track, R_init, t_init):
    """track kendi (start..start+len-1) araligindaki ilk/son gozlemle
    paralaksini kontrol eder -- pencerenin globalinden degil, kendi
    yasadigi araliktan."""
    s = track["start"]
    e = s + len(track["obs"]) - 1
    R_first, t_first = R_init[s], t_init[s]
    R_last, t_last = R_init[e], t_init[e]
    P_first = projection_matrix(R_first, t_first)
    P_last = projection_matrix(R_last, t_last)
    Xw = triangulate(P_first, P_last, np.array(track["obs"][0]), np.array(track["obs"][-1]))
    if Xw is None:
        return False
    ray1, ray2 = Xw - t_first, Xw - t_last
    d1, d2 = np.linalg.norm(ray1), np.linalg.norm(ray2)
    if d1 < 1e-9 or d2 < 1e-9:
        return False
    cos_parallax = float(np.dot(ray1, ray2) / (d1 * d2))
    return cos_parallax <= PARALLAX_COS_THRESHOLD


def solve_window(R_init, t_init, tracks, confidence):
    n = len(R_init) - 1
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()
    pixel_noise = gtsam.noiseModel.Isotropic.Sigma(2, PIXEL_NOISE_SIGMA)
    anchor_noise = gtsam.noiseModel.Isotropic.Sigma(6, ANCHOR_SIGMA)

    for i in range(n + 1):
        pose_i = Pose3(Rot3(R_init[i]), Point3(t_init[i]))
        initial.insert(X(i), pose_i)
        if i == 0:
            graph.add(gtsam.PriorFactorPose3(X(i), pose_i, anchor_noise))
        else:
            rot_sigma = PIXEL_NOISE_SIGMA / (F_FOCAL * np.sqrt(max(confidence, 1)))
            step_len = float(np.linalg.norm(t_init[i] - t_init[i - 1]))
            trans_sigma = max(step_len * rot_sigma, 1e-4)
            sigmas = np.array([rot_sigma] * 3 + [trans_sigma] * 3)
            graph.add(gtsam.PriorFactorPose3(
                X(i), pose_i, gtsam.noiseModel.Diagonal.Sigmas(sigmas)))

    Ps_init = [projection_matrix(R_init[i], t_init[i]) for i in range(n + 1)]
    n_landmarks = 0
    for tr in tracks:
        s = tr["start"]
        obs = [np.array(p) for p in tr["obs"]]
        Ps_sub = Ps_init[s:s + len(obs)]
        Xw = triangulate_multiview(Ps_sub, obs)
        if Xw is None:
            continue
        initial.insert(L(n_landmarks), Point3(Xw))
        for k, pt in enumerate(obs):
            graph.add(gtsam.GenericProjectionFactorCal3_S2(
                pt.astype(np.float64), pixel_noise, X(s + k), L(n_landmarks), gtsam_K))
        n_landmarks += 1

    if n_landmarks == 0:
        return R_init, t_init

    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(50)
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()

    R_out, t_out = [], []
    for i in range(n + 1):
        p = result.atPose3(X(i))
        R_out.append(p.rotation().matrix())
        t_out.append(p.translation())
    return R_out, t_out


# --------------------------------------------------------------------
# 2) uzun pencereler -- KLT track + GTSAM BA
# --------------------------------------------------------------------
rng = np.random.default_rng(0)
R_local_corr = [r.copy() for r in R_local_all]
t_local_corr = [t.copy() for t in t_local_all]
N = len(steps)

n_windows_used = 0
n_tracks_total = 0
window_sizes = []
R0, t0 = np.eye(3), np.zeros(3)

start = 0
while start < N - 1:
    max_span = min(MAX_WINDOW - 1, N - 1 - start)  # kalan adim sayisiyla sinirli
    frame_paths = loader.frame_list[start:start + max_span + 2]
    frame_names = [p.name for p in frame_paths]

    n_frames_used, tracks = build_long_tracks(frame_paths, frame_names)
    n = n_frames_used - 1  # lokal adim (hop) sayisi
    end = start + n
    window_sizes.append(n_frames_used)

    if n <= 0:
        start += 1
        t0 = t0 + R0 @ t_local_all[start - 1]
        R0 = R0 @ R_local_all[start - 1]
        continue

    R_init = [R0.copy()]
    t_init = [t0.copy()]
    for i in range(n):
        t_init.append(t_init[-1] + R_init[-1] @ t_local_all[start + i])
        R_init.append(R_init[-1] @ R_local_all[start + i])

    min_track_len = max(2, (n_frames_used + 1) // 2)  # pencerenin en az yarisi
    tracks = [tr for tr in tracks if len(tr["obs"]) >= min_track_len and parallax_ok(tr, R_init, t_init)]

    if not tracks:
        print(f"  pencere [{start}:{end}] (uzunluk={n_frames_used}) -- track yok, atlandi")
        R0, t0 = R_init[-1], t_init[-1]
        start = end
        continue

    if len(tracks) > MAX_TRACKS_PER_WINDOW:
        idx = rng.choice(len(tracks), MAX_TRACKS_PER_WINDOW, replace=False)
        tracks = [tracks[i] for i in idx]

    n_windows_used += 1
    n_tracks_total += len(tracks)
    print(f"  pencere [{start}:{end}] (uzunluk={n_frames_used}) -- {len(tracks)} track hayatta kaldi")

    R_out, t_out = solve_window(R_init, t_init, tracks, len(tracks))

    for i in range(n):
        R_local_corr[start + i] = R_out[i].T @ R_out[i + 1]
        t_local_corr[start + i] = R_out[i].T @ (t_out[i + 1] - t_out[i])

    R0, t0 = R_out[-1], t_out[-1]
    start = end

print(f"pencere uzunluklari -- medyan={np.median(window_sizes):.0f}  "
      f"min={min(window_sizes)}  max={max(window_sizes)}")

print(f"BA uygulanan pencere = {n_windows_used}   toplam track = {n_tracks_total}   "
      f"(ortalama {n_tracks_total / max(n_windows_used,1):.1f}/pencere)")

R_world_ba, t_world_ba = chain(R_local_corr, t_local_corr)

# --------------------------------------------------------------------
# 3) degerlendirme
# --------------------------------------------------------------------
gt = {}
with open("data/ground-truth.csv", encoding="utf-8") as fh:
    for r in csv.DictReader(fh):
        z = r.get("translation_z", "") or 0.0
        gt[r["frame_numbers"].strip()] = np.array(
            [float(r["translation_x"]), float(r["translation_y"]), float(z)])

names = [s["frame_name"] for s in steps]
stems = [n.rsplit(".", 1)[0] for n in names]

prod_pos = np.array(t_world_prod[1:])
ba_pos = np.array(t_world_ba[1:])
mask = np.array([s in gt for s in stems])
gt_arr = np.array([gt[s] for s in np.array(stems)[mask]])
prod_arr = prod_pos[mask]
ba_arr = ba_pos[mask]


def umeyama(P, Q):
    P = P.T; Q = Q.T
    n = P.shape[1]
    mu_p = P.mean(axis=1, keepdims=True)
    mu_q = Q.mean(axis=1, keepdims=True)
    Pc = P - mu_p; Qc = Q - mu_q
    W = Qc @ Pc.T / n
    U, D, Vt = np.linalg.svd(W)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_p = (Pc ** 2).sum() / n
    s = np.trace(np.diag(D) @ S) / var_p if var_p > 1e-12 else 1.0
    t = mu_q - s * R @ mu_p
    return s, R, t


def report(label, est):
    s, R, t = umeyama(est, gt_arr)
    est_al = (s * R @ est.T + t).T
    err_al = np.linalg.norm(gt_arr - est_al, axis=1)
    err_raw = np.linalg.norm(gt_arr - est, axis=1)
    print(f"--- {label} ---")
    print(f"  Sim(3) ATE (rmse)                   : {np.sqrt((err_al**2).mean()):7.2f} m   olcek={s:.4f}")
    print(f"  HIZALANMAMIS mean (yarisma metrigi) : {err_raw.mean():7.2f} m")
    print()


print("=" * 66)
print(f"  n={len(gt_arr)} kare, uyarlanabilir pencere (medyan={np.median(window_sizes):.0f}), GTSAM")
print("=" * 66)
report("URETIM (duzeltmesiz)", prod_arr)
report("BA (GTSAM, KLT uzun-bazli track)", ba_arr)
