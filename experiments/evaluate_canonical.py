"""
Kanonik degerlendirme: tek komutla, uc ucus x iki on uc icin ayni tabloyu uretir.

Kullanim (proje kokunden):
    python experiments/evaluate_canonical.py

Girdi : Adim kayitlari (pickle). Her kayit bir adim listesidir; her adim icin
        frame_name, valid, u (birim oteleme), R (rotasyon), scale.
        Varsayilan kayitlar data/ altindadir (bkz. RUNS).
Cikti : results/canonical_table.md ve results/canonical_table.csv

Metrikler (harita.md §12.2 ve §13.3 ile uyumlu):
    sekil        : tum yorunge GT'ye Sim(3) ile hizalanir (ideal olcum). Ana metrik.
    konum_rmse   : yalnizca warmup (kare < 450) Sim(3) ile hizalanan ham konum,
                   otonom karelerde (kare >= 450) 3D RMSE. Ikincil metrik.
    yon_med      : hizalanmis yorungenin hiz yonu ile GT hiz yonu farki (medyan, derece).
    olcek_s      : tum yorunge Sim(3) olcegi (olcek hatasinin gostergesi; demo disi).
    gecerli      : gecerli poz orani.

2024 verisinde GT z yoktur; metrikler XY duzleminde hesaplanir (GT z = 0).
"""
import csv
import os
import pickle
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WARM_FRAME = 450  # resmi kalibrasyon penceresi (kare)

# Kosu tanimlari: (etiket, GT csv, adim kaydi pkl, anahtar)
# Anahtar: grid kayitlarinda (ayar, H/E esigi) ciftidir; duz adim kaydi ise None.
GT_2026 = "data/ground-truth.csv"
GT_OTURUM3 = "data_2025_oturum_3/THYZ_2025_Oturum_3_Translation.csv"
GT_2024 = "data_2024/ground-truth.csv"
RUNS = [
    ("2026", "ORB", GT_2026, "data/grid_2026_orb.pkl", (0.75, 0.45)),
    ("2026", "SP", GT_2026, "data/grid_2026_sp.pkl", (0.1, 0.3)),
    ("oturum_3", "ORB", GT_OTURUM3, "data/grid_oturum3_orb.pkl", (0.75, 0.45)),
    ("oturum_3", "SP", GT_OTURUM3, "data/grid_oturum3_sp.pkl", (0.1, 0.3)),
    ("2024", "ORB", GT_2024, "data/grid_2024_orb.pkl", (0.75, 0.45)),
    ("2024", "SP", GT_2024, "data/grid_2024_sp.pkl", (0.1, 0.3)),
]


def load_gt(path):
    import csv as _csv
    gt = {}
    with open(os.path.join(ROOT, path), encoding="utf-8") as fh:
        for r in _csv.DictReader(fh):
            idx = int(r["frame_numbers"].strip().split("_")[-1])
            z = float(r.get("translation_z") or 0.0)
            gt[idx] = np.array([float(r["translation_x"]), float(r["translation_y"]), z])
    return gt


def umeyama(src, dst):
    """Sim(3): dst ~ s * R * src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    U, D, Vt = np.linalg.svd(dc.T @ sc / len(src))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / ((sc ** 2).sum() / len(src))
    return s, R, mu_d - s * R @ mu_s


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def integrate(steps):
    """Uretimle ayni birlesim: t_world += R_world * t_local; R_world *= R_local; z sifirlanir."""
    p = np.zeros(3)
    Rw = np.eye(3)
    out = [p.copy()]
    for s in steps:
        if s["valid"]:
            u = np.array([s["u"][0], s["u"][1], 0.0])
            u = u / max(np.linalg.norm(u[:2]), 1e-9)
            p = p + Rw @ (s["scale"] * u)
            Rw = Rw @ s["R"]
            p[2] = 0.0
        out.append(p.copy())
    return np.array(out)


RPE_LENGTHS = [100, 200, 300, 400, 500, 600, 700, 800]  # metre (KITTI tarzi)


def rpe_percent(H, G):
    """
    KITTI tarzi alt yol hatasi (%): Sim(3) ile hizali yorunge H ve GT G uzerinde,
    her uzunluk L icin tum baslangic karelerinde GT yolu L metreye ulasan alt yollar secilir;
    alt yol ote hatasi ||(H_son-H_bas)-(G_son-G_bas)|| / L. Tum alt yollarin ortalamasi (%).
    """
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(G, axis=0), axis=1))]
    errs = []
    for L in RPE_LENGTHS:
        for i in range(0, len(G), 10):
            j = np.searchsorted(seg, seg[i] + L)
            if j >= len(G):
                break
            dG = G[j] - G[i]
            dH = H[j] - H[i]
            errs.append(np.linalg.norm(dH - dG) / L)
    if not errs:
        return float("nan")
    return 100.0 * float(np.mean(errs))


def evaluate(steps, gt):
    fidx = [int(s["frame_name"].rsplit(".", 1)[0].split("_")[-1]) for s in steps]
    stride = fidx[1] - fidx[0] if len(fidx) > 1 else 5
    first = fidx[0] - stride
    fi = np.array([first] + fidx)
    G = np.array([gt[f] - gt[first] for f in fi])
    P = integrate(steps)

    # sekil: tum yorunge Sim(3) (ideal)
    s_all, R_all, t_all = umeyama(P, G)
    H = (s_all * (R_all @ P.T)).T + t_all
    sekil = float(np.sqrt(np.mean(np.sum((H - G) ** 2, axis=1))))

    # konum: warmup Sim(3), otonomda 3D RMSE
    w = fi < WARM_FRAME
    s_w, R_w, t_w = umeyama(P[w], G[w])
    Hw = (s_w * (R_w @ P.T)).T + t_w
    ev = fi >= WARM_FRAME
    konum = float(np.sqrt(np.mean(np.sum((Hw[ev] - G[ev]) ** 2, axis=1))))

    # yon: hizalanmis hiz yonu farki (hareketli adimlar)
    dG = np.diff(G[:, :2], axis=0)
    dH = np.diff(H[:, :2], axis=0)
    mv = np.linalg.norm(dG, axis=1) > 0.3
    hg = np.arctan2(dG[:, 1], dG[:, 0])
    hh = np.arctan2(dH[:, 1], dH[:, 0])
    yon = float(np.median(np.abs(np.degrees(wrap(hh - hg)))[mv]))

    gecerli = float(np.mean([s["valid"] for s in steps]))
    rpe = rpe_percent(H, G)
    return dict(sekil=sekil, konum=konum, yon=yon, olcek=float(s_all), gecerli=gecerli, rpe=rpe)


def load_steps(path, key):
    obj = pickle.load(open(os.path.join(ROOT, path), "rb"))
    return obj[key] if key is not None else obj


def main():
    os.chdir(ROOT)
    rows = []
    for flight, front, gt_path, pkl, key in RUNS:
        gt = load_gt(gt_path)
        steps = load_steps(pkl, key)
        r = evaluate(steps, gt)
        r.update(flight=flight, front=front)
        rows.append(r)
        print(f"{flight:9s} {front:3s} sekil={r['sekil']:6.1f}  yon={r['yon']:5.1f}  "
              f"rpe={r['rpe']:5.2f}%  konum={r['konum']:6.1f}  olcek={r['olcek']:.2f}  gecerli={r['gecerli']:.2f}")

    os.makedirs("results", exist_ok=True)
    with open("results/canonical_table.csv", "w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=["flight", "front", "sekil", "rpe", "yon", "konum", "olcek", "gecerli"])
        wr.writeheader()
        for r in rows:
            wr.writerow({k: (f"{r[k]:.3f}" if isinstance(r[k], float) else r[k]) for k in wr.fieldnames})

    lines = [
        "# Kanonik sonuc tablosu",
        "",
        "| uçuş | ön uç | şekil ATE (m, ana) | alt yol RPE (%) | yön (° med.) | konum RMSE (m, ikincil) | ölçek s | geçerli |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(f"| {r['flight']} | {r['front']} | {r['sekil']:.1f} | {r['rpe']:.2f} | {r['yon']:.1f} | "
                     f"{r['konum']:.1f} | {r['olcek']:.2f} | {r['gecerli']:.2f} |")
    lines += [
        "",
        "Notlar: Şekil ATE, GT ile tüm yörünge üzerinden Sim(3) hizalanarak ölçülür (ideal ölçüm; literatürdeki standart ATE-Sim3).",
        "Alt yol RPE (%), KITTI tarzı: 100–800 m alt yollarda Sim(3) hizalı yörüngenin göreli ötelemesi hatası, yol uzunluğuna oranla.",
        "Konum RMSE, yalnızca warmup (kare < 450) hizalamasıyla ve üretim ölçeğiyle hesaplanır.",
        "2024'te GT z yoktur; metrikler XY düzleminde hesaplanır, diğer uçuşlarla mutlak karşılaştırma yapılmaz.",
        "Ölçek ayrı bir sorundur ve demo kapsamı dışındadır (harita.md §13.13).",
    ]
    with open("results/canonical_table.md", "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\nYazildi: results/canonical_table.md, results/canonical_table.csv")


if __name__ == "__main__":
    sys.exit(main())
