"""
core/feature_extractor.py
==========================
Feature extraction ve semantic maskeleme modülü.
ORB ile keypoint tespiti ve descriptor hesabı yapar.
Dinamik objeler semantic mask ile engellenir.

Phase 2'de bu modül SuperPoint ile swap edilir.
Interface değişmez — odometry.py bu değişimden habersiz kalır.
"""

import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from utils.camera_calibration import CameraCalibration, CameraCalibrationError
from utils.data_loader import DataLoader, DataLoaderError, Detection

logger = logging.getLogger(__name__)

# 3 Ekim: SuperPoint, torch+lightglue gerektiriyor. ORB-only kullanim
# (varsayilan, config.yaml) bu bagimliliklara hic ihtiyac duymamali --
# ayni GTSAM'in pose_graph.py'de lazy-import edilmesi gibi, burada da
# korumali (guarded) import yapiyoruz.
try:
    import torch
    from lightglue import SuperPoint as _LightGlueSuperPoint
    from lightglue.utils import numpy_image_to_torch
    _SUPERPOINT_AVAILABLE = True
except ImportError:
    torch = None
    _LightGlueSuperPoint = None
    numpy_image_to_torch = None
    _SUPERPOINT_AVAILABLE = False


class FeatureExtractorError(Exception):
    """FeatureExtractor'a özgü hata sınıfı."""
    pass


@dataclass
class FrameFeatures:
    """
    Tek bir frame'in feature extraction çıktısını temsil eder.

    Attributes:
        frame_name : Frame dosya adı
        keypoints  : ORB keypoint listesi
        descriptors: ORB binary descriptor matrisi (N x 32, uint8)
        mask       : Semantic mask (H x W, uint8) — 255: statik, 0: dinamik
        clean_frame: Undistorted BGR frame
    """
    frame_name: str
    keypoints: List[cv2.KeyPoint]
    descriptors: Optional[np.ndarray]
    mask: np.ndarray
    clean_frame: np.ndarray

    @property
    def keypoint_count(self) -> int:
        return len(self.keypoints)

    @property
    def has_descriptors(self) -> bool:
        return self.descriptors is not None and len(self.descriptors) > 0

    def __repr__(self) -> str:
        return (
            f"FrameFeatures(frame={self.frame_name}, "
            f"keypoints={self.keypoint_count}, "
            f"has_descriptors={self.has_descriptors})"
        )


class FeatureExtractor:
    """
    ORB tabanlı feature extraction ve semantic maskeleme.

    Pipeline'daki yeri:
        1. Frame undistort  → CameraCalibration
        2. Semantic mask    → YOLO detections → dinamik objeler engellenir
        3. ORB detect       → FAST keypoints
        4. ORB describe     → rBRIEF descriptors

    Phase 2 swap notu:
        SuperPoint ile değiştirildiğinde bu sınıf kaldırılır,
        aynı extract() interface'ini sunan SuperPointExtractor yazılır.
        odometry.py'da hiçbir değişiklik gerekmez.
    """

    def __init__(
        self,
        data_loader: DataLoader,
        camera_calibration: CameraCalibration,
    ) -> None:
        """
        FeatureExtractor'ı başlatır.

        Args:
            data_loader        : Başlatılmış DataLoader instance'ı.
            camera_calibration : Başlatılmış CameraCalibration instance'ı.

        Raises:
            FeatureExtractorError: Config'de eksik parametre varsa.
        """
        logger.info("[FeatureExtractor] Başlatılıyor...")

        self._loader = data_loader
        self._cam = camera_calibration

        feat_cfg = data_loader.get_feature_config()
        sem_cfg = data_loader.get_semantic_config()

        self._max_features = int(feat_cfg["max_features"])
        self._spatial_distribution = bool(feat_cfg.get("spatial_distribution", False))
        self._spatial_oversample = int(feat_cfg.get("spatial_distribution_oversample", 3))
        self._detector_type = str(feat_cfg.get("detector_type", "orb")).lower()

        self._orb = None
        self._superpoint = None
        self._sp_device = None

        if self._detector_type == "orb":
            self._orb = self._build_orb(feat_cfg)
        elif self._detector_type == "superpoint":
            self._superpoint, self._sp_device = self._build_superpoint(feat_cfg)
        else:
            raise FeatureExtractorError(
                f"Bilinmeyen 'detector_type': {self._detector_type!r} "
                f"(gecerli degerler: 'orb', 'superpoint')"
            )

        self._dynamic_classes = self._parse_dynamic_classes(sem_cfg)

        logger.info(
            "[FeatureExtractor] Kuruldu — detector_type=%s, max_features=%d, "
            "spatial_distribution=%s, dynamic_classes=%s",
            self._detector_type, self._max_features, self._spatial_distribution,
            self._dynamic_classes,
        )

    # ------------------------------------------------------------------
    # Initialization helpers
    # ------------------------------------------------------------------

    def _build_orb(self, feat_cfg: dict) -> cv2.ORB:
        """
        Config'den ORB detector oluşturur.

        spatial_distribution aktifse, ORB'u max_features'in
        spatial_distribution_oversample katı kadar keypoint bulacak
        sekilde kurar -- extract() sonra bunlari _spatial_bucket ile
        max_features'e (ama uzamsal olarak dagilmis sekilde) indirger.

        Args:
            feat_cfg: config.yaml features bloğu.

        Returns:
            cv2.ORB instance'ı.

        Raises:
            FeatureExtractorError: Eksik veya geçersiz parametre varsa.
        """
        required = ["max_features", "scale_factor", "n_levels"]
        missing = [k for k in required if k not in feat_cfg]
        if missing:
            raise FeatureExtractorError(
                f"features config'inde eksik anahtarlar: {missing}"
            )

        n_features = int(feat_cfg["max_features"])
        if self._spatial_distribution:
            n_features *= max(1, self._spatial_oversample)

        try:
            return cv2.ORB_create(
                nfeatures=n_features,
                scaleFactor=float(feat_cfg["scale_factor"]),
                nlevels=int(feat_cfg["n_levels"]),
            )
        except Exception as e:
            raise FeatureExtractorError(f"ORB oluşturulamadı: {e}") from e

    def _build_superpoint(self, feat_cfg: dict) -> Tuple["torch.nn.Module", str]:
        """
        3 Ekim: Config'den SuperPoint extractor olusturur (lightglue paketi
        uzerinden). Agirliklar ilk calistirmada torch.hub ile otomatik
        indirilip ~/.cache/torch/hub/checkpoints altinda saklaniyor --
        ayri bir weights/ klasoru gerekmiyor.

        Args:
            feat_cfg: config.yaml features blogu.

        Returns:
            (superpoint_model, device_str) tuple'i.

        Raises:
            FeatureExtractorError: torch/lightglue kurulu degilse veya
                model olusturulamazsa.
        """
        if not _SUPERPOINT_AVAILABLE:
            raise FeatureExtractorError(
                "detector_type='superpoint' secildi ama 'torch'/'lightglue' "
                "kurulu degil. Kurulum: pip install torch (CUDA'li) + "
                "pip install git+https://github.com/cvg/LightGlue.git"
            )

        device_cfg = str(feat_cfg.get("device", "auto")).lower()
        if device_cfg == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            device = device_cfg

        try:
            model = _LightGlueSuperPoint(
                max_num_keypoints=self._max_features,
            ).eval().to(device)
        except Exception as e:
            raise FeatureExtractorError(f"SuperPoint olusturulamadi: {e}") from e

        logger.info(
            "[FeatureExtractor] SuperPoint kuruldu — device=%s, max_num_keypoints=%d",
            device, self._max_features,
        )
        return model, device

    def _detect_superpoint(
        self,
        gray: np.ndarray,
        mask: np.ndarray,
    ) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]:
        """
        SuperPoint ile keypoint+descriptor cikarir, ORB ile ayni
        FrameFeatures formatina (List[cv2.KeyPoint], NxD float32
        descriptor array) donusturur -- boylece matcher.py/
        motion_estimator.py'de hicbir degisiklik gerekmez.

        Semantic mask, SuperPoint'in kendi API'sinde bir parametre
        olmadigi icin, cikarim SONRASI post-hoc filtre olarak uygulanir
        (dinamik bolgeye dusen keypoint'ler atilir).

        Args:
            gray: Undistorted gri tonlamali goruntu (H x W, uint8).
            mask: Semantic mask (H x W, uint8) — 255: statik, 0: dinamik.

        Returns:
            (keypoints, descriptors) tuple'i.
        """
        img_tensor = numpy_image_to_torch(gray).to(self._sp_device)
        with torch.no_grad():
            feats = self._superpoint.extract(img_tensor)

        kps = feats["keypoints"][0].cpu().numpy()
        scores = feats["keypoint_scores"][0].cpu().numpy()
        desc = feats["descriptors"][0].cpu().numpy().astype(np.float32)

        if len(kps) == 0:
            return [], None

        # Semantic mask'i post-hoc uygula
        h, w = mask.shape
        xs = np.clip(np.round(kps[:, 0]).astype(int), 0, w - 1)
        ys = np.clip(np.round(kps[:, 1]).astype(int), 0, h - 1)
        keep = mask[ys, xs] != 0

        kps, scores, desc = kps[keep], scores[keep], desc[keep]

        keypoints = [
            cv2.KeyPoint(x=float(x), y=float(y), size=8.0, response=float(s))
            for (x, y), s in zip(kps, scores)
        ]
        descriptors = np.ascontiguousarray(desc) if len(desc) > 0 else None
        return keypoints, descriptors

    def _spatial_bucket(
        self,
        keypoints: List[cv2.KeyPoint],
        descriptors: Optional[np.ndarray],
        img_w: int,
        img_h: int,
    ) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]:
        """
        29 Eylul: ORB-SLAM2'nin quadtree'sinin basitlestirilmis bir
        esdegeri -- goruntuyu ~max_features hucrelik bir izgaraya boler,
        her hucrede en guclu (response) TEK keypoint'i tutar. Amac,
        keypoint'lerin bir bolgede kumelenip E/H tahminini o bolgenin
        yerel geometrisine asiri bagimli kilmasini engellemek (olcum:
        quadtree_check.py, kumelenmis adimlarda yon hatasi 5x daha
        yuksek).
        """
        if not keypoints:
            return keypoints, descriptors

        cell_size = max(1.0, float(np.sqrt(img_w * img_h / max(self._max_features, 1))))
        n_cols = max(1, int(np.ceil(img_w / cell_size)))

        best: dict = {}
        for i, kp in enumerate(keypoints):
            col = min(int(kp.pt[0] / cell_size), n_cols - 1)
            row = int(kp.pt[1] / cell_size)
            key = row * n_cols + col
            prev = best.get(key)
            if prev is None or kp.response > prev[0]:
                best[key] = (kp.response, i)

        keep_idx = sorted(idx for _, idx in best.values())
        kept_kps = [keypoints[i] for i in keep_idx]
        kept_desc = descriptors[keep_idx] if descriptors is not None and len(descriptors) > 0 else descriptors
        return kept_kps, kept_desc

    @staticmethod
    def _parse_dynamic_classes(sem_cfg: dict) -> set:
        """
        Config'den dinamik class ID setini parse eder.

        Args:
            sem_cfg: config.yaml semantic bloğu.

        Returns:
            int set — maskelenecek class ID'leri.

        Raises:
            FeatureExtractorError: dynamic_classes eksikse.
        """
        if "dynamic_classes" not in sem_cfg:
            raise FeatureExtractorError(
                "semantic config'inde 'dynamic_classes' anahtarı bulunamadı."
            )
        return set(int(c) for c in sem_cfg["dynamic_classes"])

    # ------------------------------------------------------------------
    # Semantic Mask
    # ------------------------------------------------------------------

    def _build_semantic_mask(
        self,
        frame_shape: Tuple[int, int],
        detections: List[Detection],
    ) -> np.ndarray:
        """
        YOLO tespitlerinden semantic mask üretir.

        Mask mantığı:
            255 (beyaz) → statik bölge — ORB buradan feature çıkarır
            0   (siyah) → dinamik obje  — ORB bu bölgeyi yok sayar

        Sadece config'deki dynamic_classes'a ait bbox'lar maskelenir.
        Diğer class'lar (UAP, UAI, MAKİNE) maskelenmez.

        Args:
            frame_shape: (height, width) tuple.
            detections : Bu frame'e ait Detection listesi.

        Returns:
            uint8 mask array, shape=(H, W).
        """
        mask = np.ones(frame_shape, dtype=np.uint8) * 255

        masked_count = 0
        for det in detections:
            if det.class_id not in self._dynamic_classes:
                continue

            bbox = det.bbox
            if len(bbox) != 4:
                logger.warning(
                    "[FeatureExtractor] Geçersiz bbox formatı, atlanıyor: %s", bbox
                )
                continue

            x1, y1, x2, y2 = map(int, bbox)

            # Sınır kontrolü — bbox frame dışına taşabilir
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(frame_shape[1], x2)
            y2 = min(frame_shape[0], y2)

            if x2 <= x1 or y2 <= y1:
                logger.warning(
                    "[FeatureExtractor] Dejenere bbox, atlanıyor: [%d,%d,%d,%d]",
                    x1, y1, x2, y2,
                )
                continue

            cv2.rectangle(mask, (x1, y1), (x2, y2), 0, -1)
            masked_count += 1

        logger.debug(
            "[FeatureExtractor] Semantic mask: %d/%d tespit maskelendi.",
            masked_count, len(detections),
        )
        return mask

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def extract(self, frame: np.ndarray, frame_name: str) -> FrameFeatures:
        """
        Tek bir frame üzerinde tam feature extraction pipeline'ını çalıştırır.

        Adımlar:
            1. Undistort
            2. Grayscale dönüşümü
            3. Semantic mask üretimi
            4. ORB detect + compute (mask uygulanmış)

        Args:
            frame      : Raw BGR frame (diskten okunmuş).
            frame_name : Frame dosya adı (detections lookup için).

        Returns:
            FrameFeatures dataclass instance'ı.

        Raises:
            FeatureExtractorError: Kritik bir hata oluşursa.
        """
        try:
            # 1. Undistort
            clean_frame = self._cam.undistort(frame)

            # 2. Grayscale
            gray = cv2.cvtColor(clean_frame, cv2.COLOR_BGR2GRAY)

            # 3. Semantic mask
            detections = self._loader.get_detections(frame_name)
            mask = self._build_semantic_mask(gray.shape, detections)

            # 4. Detect + compute (detector_type'a gore ORB ya da SuperPoint)
            if self._detector_type == "orb":
                keypoints, descriptors = self._orb.detectAndCompute(gray, mask)
            else:
                keypoints, descriptors = self._detect_superpoint(gray, mask)

            if keypoints is None:
                keypoints = []

            if self._spatial_distribution and keypoints:
                keypoints, descriptors = self._spatial_bucket(
                    keypoints, descriptors, gray.shape[1], gray.shape[0]
                )

            logger.debug(
                "[FeatureExtractor] %s → %d keypoint bulundu.",
                frame_name, len(keypoints),
            )

            return FrameFeatures(
                frame_name=frame_name,
                keypoints=keypoints,
                descriptors=descriptors,
                mask=mask,
                clean_frame=clean_frame,
            )

        except CameraCalibrationError as e:
            raise FeatureExtractorError(
                f"Undistort hatası ({frame_name}): {e}"
            ) from e
        except cv2.error as e:
            raise FeatureExtractorError(
                f"OpenCV hatası ({frame_name}): {e}"
            ) from e

    def extract_from_loader(self, frame_name: str) -> FrameFeatures:
        """
        Frame adından otomatik olarak frame'i yükler ve extract() çağırır.

        Args:
            frame_name: Frame dosya adı.

        Returns:
            FrameFeatures instance'ı.
        """
        frame_path = next(
            (p for p in self._loader.frame_list if p.name == frame_name), None
        )
        if frame_path is None:
            raise FeatureExtractorError(
                f"Frame bulunamadı: {frame_name}"
            )
        frame = self._loader.load_frame(frame_path)
        return self.extract(frame, frame_name)

    def __repr__(self) -> str:
        return (
            f"FeatureExtractor("
            f"dynamic_classes={self._dynamic_classes})"
        )


# ------------------------------------------------------------------
# Standalone test
# ------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    config_path = __import__("pathlib").Path(__file__).resolve().parent.parent / "config.yaml"

    try:
        loader = DataLoader(str(config_path))
        cam = CameraCalibration(loader)
        extractor = FeatureExtractor(loader, cam)

        # İlk 3 frame üzerinde test
        print("\n--- Feature Extraction Testi ---")
        for idx, name, frame in loader.frame_generator():
            features = extractor.extract(frame, name)
            print(f"[{idx}] {features}")

            # Mask istatistikleri
            total_pixels = features.mask.size
            masked_pixels = int(np.sum(features.mask == 0))
            mask_ratio = masked_pixels / total_pixels * 100
            print(f"     Maskelenen alan: {mask_ratio:.2f}%")

            if idx >= 2:
                break

        # Debug görsel — ilk frame
        print("\n--- Debug Görsel Üretiliyor ---")
        first_name = loader.frame_list[0].name
        features = extractor.extract_from_loader(first_name)

        debug_img = cv2.drawKeypoints(
            features.clean_frame,
            features.keypoints,
            None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        # Mask'i görsel olarak üst üste koy
        mask_colored = cv2.cvtColor(features.mask, cv2.COLOR_GRAY2BGR)
        mask_colored[features.mask == 0] = [0, 0, 80]  # Maskelenen alan koyu kırmızı

        cv2.imwrite("debug_feature_extraction.jpg", debug_img)
        cv2.imwrite("debug_semantic_mask.jpg", mask_colored)
        print(f"  debug_feature_extraction.jpg → {features.keypoint_count} keypoint")
        print(f"  debug_semantic_mask.jpg → maskeleme görsel")

    except (DataLoaderError, CameraCalibrationError, FeatureExtractorError) as e:
        logger.error("Hata: %s", e)
        sys.exit(1)