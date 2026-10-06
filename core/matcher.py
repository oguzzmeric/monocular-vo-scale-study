"""
core/matcher.py
================
Feature matching modülü.
BF Matcher + Hamming Distance + Lowe's Ratio Test ile
ardışık iki frame arasındaki keypoint eşleşmelerini üretir.

Phase 2'de bu modül LightGlue ile swap edilir.
Interface değişmez — motion_estimator.py bu değişimden habersiz kalır.
"""

import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from core.feature_extractor import FrameFeatures, FeatureExtractorError
from utils.data_loader import DataLoader

logger = logging.getLogger(__name__)

# 3 Ekim: LightGlue de SuperPoint gibi torch+lightglue gerektiriyor --
# ayni guarded-import deseni (bkz. feature_extractor.py).
try:
    import torch
    from lightglue import LightGlue as _LightGlueMatcher
    _LIGHTGLUE_AVAILABLE = True
except ImportError:
    torch = None
    _LightGlueMatcher = None
    _LIGHTGLUE_AVAILABLE = False


class MatcherError(Exception):
    """Matcher'a özgü hata sınıfı."""
    pass


def validate_lightglue_conf(depth_confidence: float, width_confidence: float) -> None:
    """
    5 Ekim guvenlik kilidi: 2024 verisinde depth_confidence=0.99 ve width_confidence=-1
    birlikte kullanildiginda pozlar felakete suruklendi (163-280 m).
    Bu kombinasyon ve width_confidence<0 (genislik budamasi kapali) reddedilir.

    Raises:
        MatcherError: yasakli kombinasyon.
    """
    if width_confidence < 0:
        raise MatcherError(
            f"lg_width_confidence={width_confidence} (<0) yasak: genislik budamasi "
            "kapali; 2024'te felakete yol acti."
        )
    if depth_confidence >= 0.99:
        raise MatcherError(
            f"lg_depth_confidence={depth_confidence} (>=0.99) yasak: erken cikis "
            "neredeyse hic olmuyor; 2024'te felakete yol acti."
        )


@dataclass
class MatchResult:
    """
    İki frame arasındaki matching sonucunu temsil eder.

    Attributes:
        frame_name_prev : Önceki frame adı
        frame_name_curr : Mevcut frame adı
        pts_prev        : Önceki frame'deki eşleşen nokta koordinatları (N x 2, float32)
        pts_curr        : Mevcut frame'deki eşleşen nokta koordinatları (N x 2, float32)
        matches         : Filtrelenmiş DMatch listesi
        match_count     : Toplam eşleşme sayısı
    """
    frame_name_prev: str
    frame_name_curr: str
    pts_prev: np.ndarray
    pts_curr: np.ndarray
    matches: List[cv2.DMatch]

    @property
    def match_count(self) -> int:
        return len(self.matches)

    @property
    def has_enough_matches(self) -> bool:
        """Motion estimation için minimum 8 nokta gerekir (8-point algorithm)."""
        return self.match_count >= 8

    def __repr__(self) -> str:
        return (
            f"MatchResult("
            f"prev={self.frame_name_prev}, "
            f"curr={self.frame_name_curr}, "
            f"matches={self.match_count}, "
            f"enough={self.has_enough_matches})"
        )


class Matcher:
    """
    BF Matcher + Lowe's Ratio Test tabanlı feature matcher.

    Matching pipeline:
        1. BF Matcher ile kNN eşleştirme (k=2)
        2. Lowe's Ratio Test → kaba outlier eleme
        3. Koordinat array'lerine dönüştürme

    Phase 2 swap notu:
        LightGlue ile değiştirildiğinde bu sınıf kaldırılır,
        aynı match() interface'ini sunan LightGlueMatcher yazılır.
        motion_estimator.py'da hiçbir değişiklik gerekmez.
    """

    def __init__(self, data_loader: DataLoader) -> None:
        """
        Matcher'ı başlatır.

        Args:
            data_loader: Başlatılmış DataLoader instance'ı.

        Raises:
            MatcherError: Config'de eksik parametre varsa.
        """
        logger.info("[Matcher] Başlatılıyor...")

        feat_cfg = data_loader.get_feature_config()
        self._matcher_type = str(feat_cfg.get("matcher_type", "classical")).lower()
        self._lightglue = None
        self._lg_device = None
        self._lowe_ratio = None
        self._descriptor_norm = None
        self._bf_matcher = None

        if self._matcher_type == "classical":
            self._lowe_ratio = self._parse_lowe_ratio(feat_cfg)
            self._descriptor_norm = str(feat_cfg.get("descriptor_norm", "hamming")).lower()
            self._bf_matcher = self._build_bf_matcher(self._descriptor_norm)
            logger.info(
                "[Matcher] BF Matcher kuruldu — lowe_ratio=%.2f, descriptor_norm=%s",
                self._lowe_ratio, self._descriptor_norm,
            )
        elif self._matcher_type == "lightglue":
            self._lightglue, self._lg_device = self._build_lightglue(feat_cfg)
            logger.info(
                "[Matcher] LightGlue kuruldu — device=%s", self._lg_device,
            )
        else:
            raise MatcherError(
                f"Bilinmeyen 'matcher_type': {self._matcher_type!r} "
                f"(gecerli degerler: 'classical', 'lightglue')"
            )

    # ------------------------------------------------------------------
    # Initialization helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_lowe_ratio(feat_cfg: dict) -> float:
        """
        Config'den Lowe's Ratio Test eşiğini parse eder.

        Args:
            feat_cfg: config.yaml features bloğu.

        Returns:
            float — lowe ratio eşiği.

        Raises:
            MatcherError: Eksik veya geçersiz değer varsa.
        """
        if "lowe_ratio" not in feat_cfg:
            raise MatcherError(
                "features config'inde 'lowe_ratio' anahtarı bulunamadı."
            )

        ratio = float(feat_cfg["lowe_ratio"])
        if not 0.0 < ratio < 1.0:
            raise MatcherError(
                f"lowe_ratio 0 ile 1 arasında olmalı, alınan: {ratio}"
            )
        return ratio

    @staticmethod
    def _build_bf_matcher(descriptor_norm: str) -> cv2.BFMatcher:
        """
        Config'deki descriptor_norm'a gore BF Matcher olusturur.

        ORB ikili descriptor -> Hamming Distance (varsayilan).
        SuperPoint ondalikli (float) descriptor -> L2 mesafe (kosinus
        benzerligiyle orantili, cunku SuperPoint tanimlayicilari zaten
        L2-normalize edilmis durumda).
        crossCheck=False → kNN (k=2) ile Lowe's Ratio Test uygulanabilir.

        Args:
            descriptor_norm: "hamming" ya da "l2".

        Returns:
            cv2.BFMatcher instance'ı.

        Raises:
            MatcherError: Bilinmeyen norm degeri verilirse.
        """
        norm_map = {"hamming": cv2.NORM_HAMMING, "l2": cv2.NORM_L2}
        if descriptor_norm not in norm_map:
            raise MatcherError(
                f"Bilinmeyen descriptor_norm: {descriptor_norm!r} "
                f"(gecerli degerler: 'hamming', 'l2')"
            )
        return cv2.BFMatcher(norm_map[descriptor_norm], crossCheck=False)

    def _build_lightglue(self, feat_cfg: dict) -> Tuple["torch.nn.Module", str]:
        """
        3 Ekim: LightGlue eslestirici olusturur (lightglue paketi).
        Agirliklar SuperPoint gibi torch.hub ile otomatik indirilir.

        Args:
            feat_cfg: config.yaml features blogu.

        Returns:
            (lightglue_model, device_str) tuple'i.

        Raises:
            MatcherError: torch/lightglue kurulu degilse.
        """
        if not _LIGHTGLUE_AVAILABLE:
            raise MatcherError(
                "matcher_type='lightglue' secildi ama 'torch'/'lightglue' "
                "kurulu degil."
            )

        device_cfg = str(feat_cfg.get("device", "auto")).lower()
        if device_cfg == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            device = device_cfg

        # 3 Ekim: dogruluga etki eden 3 ic parametre config'ten okunabilir
        # (varsayilanlari LightGlue'nun kendi varsayilanlariyla ayni --
        # hicbir sey belirtilmezse davranis degismez).
        lg_conf = {
            "filter_threshold": float(feat_cfg.get("lg_filter_threshold", 0.1)),
            "depth_confidence": float(feat_cfg.get("lg_depth_confidence", 0.95)),
            "width_confidence": float(feat_cfg.get("lg_width_confidence", 0.99)),
        }

        validate_lightglue_conf(lg_conf["depth_confidence"], lg_conf["width_confidence"])

        try:
            model = _LightGlueMatcher(features="superpoint", **lg_conf).eval().to(device)
        except Exception as e:
            raise MatcherError(f"LightGlue olusturulamadi: {e}") from e

        return model, device

    def _match_lightglue(
        self,
        features_prev: FrameFeatures,
        features_curr: FrameFeatures,
    ) -> "MatchResult":
        """
        LightGlue ile esleme yapar. Girdi olarak keypoints+descriptors+
        image_size gerekiyor -- bunlari MEVCUT FrameFeatures alanlarindan
        (keypoints, descriptors, clean_frame.shape) yeniden kuruyoruz,
        FrameFeatures'a YENI bir alan EKLEMEDEN (arayuz degismiyor).
        """
        if not features_prev.has_descriptors:
            raise MatcherError(
                f"Önceki frame'de descriptor yok: {features_prev.frame_name}"
            )
        if not features_curr.has_descriptors:
            raise MatcherError(
                f"Mevcut frame'de descriptor yok: {features_curr.frame_name}"
            )

        h0, w0 = features_prev.clean_frame.shape[:2]
        h1, w1 = features_curr.clean_frame.shape[:2]

        kp0 = torch.tensor(
            [kp.pt for kp in features_prev.keypoints], dtype=torch.float32, device=self._lg_device
        )[None]
        kp1 = torch.tensor(
            [kp.pt for kp in features_curr.keypoints], dtype=torch.float32, device=self._lg_device
        )[None]
        desc0 = torch.from_numpy(features_prev.descriptors).to(self._lg_device)[None]
        desc1 = torch.from_numpy(features_curr.descriptors).to(self._lg_device)[None]
        size0 = torch.tensor([[w0, h0]], dtype=torch.float32, device=self._lg_device)
        size1 = torch.tensor([[w1, h1]], dtype=torch.float32, device=self._lg_device)

        with torch.no_grad():
            out = self._lightglue({
                "image0": {"keypoints": kp0, "descriptors": desc0, "image_size": size0},
                "image1": {"keypoints": kp1, "descriptors": desc1, "image_size": size1},
            })

        matches = out["matches"][0].cpu().numpy()
        scores = out["scores"][0].cpu().numpy()

        if len(matches) == 0:
            pts_prev = np.empty((0, 2), dtype=np.float32)
            pts_curr = np.empty((0, 2), dtype=np.float32)
            dmatches: List[cv2.DMatch] = []
        else:
            pts_prev = np.float32([features_prev.keypoints[i].pt for i in matches[:, 0]])
            pts_curr = np.float32([features_curr.keypoints[j].pt for j in matches[:, 1]])
            dmatches = [
                cv2.DMatch(int(i), int(j), float(1.0 - s))
                for (i, j), s in zip(matches, scores)
            ]

        logger.debug(
            "[Matcher] %s → %s: LightGlue %d eşleşme buldu.",
            features_prev.frame_name, features_curr.frame_name, len(dmatches),
        )

        result = MatchResult(
            frame_name_prev=features_prev.frame_name,
            frame_name_curr=features_curr.frame_name,
            pts_prev=pts_prev,
            pts_curr=pts_curr,
            matches=dmatches,
        )

        if not result.has_enough_matches:
            logger.warning(
                "[Matcher] Yetersiz eşleşme: %d (minimum 8 gerekli).",
                result.match_count,
            )

        return result

    # ------------------------------------------------------------------
    # Lowe's Ratio Test
    # ------------------------------------------------------------------

    def _apply_lowe_ratio_test(
        self,
        knn_matches: List[Tuple[cv2.DMatch, cv2.DMatch]],
    ) -> List[cv2.DMatch]:
        """
        Lowe's Ratio Test uygular.

        Matematiksel temel (Lowe 2004):
            En iyi eşleşme mesafesi d1, ikinci en iyi d2 ise:
            d1 / d2 < ratio → güvenilir eşleşme (inlier)
            d1 / d2 >= ratio → belirsiz eşleşme (outlier, atılır)

        Sezgisel açıklama:
            Eğer bir keypoint'in en iyi eşleşmesi ile ikinci en iyi eşleşmesi
            birbirine çok yakınsa, eşleşme belirsizdir — iki aday da olabilir.
            Ratio test bu belirsizliği eler.

        Args:
            knn_matches: cv2.BFMatcher.knnMatch(k=2) çıktısı.

        Returns:
            Ratio test'i geçen DMatch listesi.
        """
        good_matches = []
        for match_pair in knn_matches:
            if len(match_pair) != 2:
                continue
            m, n = match_pair
            if m.distance < self._lowe_ratio * n.distance:
                good_matches.append(m)
        return good_matches

    # ------------------------------------------------------------------
    # Koordinat dönüşümü
    # ------------------------------------------------------------------

    @staticmethod
    def _matches_to_points(
        matches: List[cv2.DMatch],
        kp_prev: List[cv2.KeyPoint],
        kp_curr: List[cv2.KeyPoint],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        DMatch listesini piksel koordinat array'lerine dönüştürür.

        motion_estimator.py'daki findHomography ve findEssentialMat
        doğrudan bu array'leri kullanır.

        Args:
            matches : Filtrelenmiş DMatch listesi.
            kp_prev : Önceki frame keypoint listesi.
            kp_curr : Mevcut frame keypoint listesi.

        Returns:
            (pts_prev, pts_curr): Her biri (N x 2) float32 array.
        """
        pts_prev = np.float32(
            [kp_prev[m.queryIdx].pt for m in matches]
        ).reshape(-1, 2)

        pts_curr = np.float32(
            [kp_curr[m.trainIdx].pt for m in matches]
        ).reshape(-1, 2)

        return pts_prev, pts_curr

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def match(
        self,
        features_prev: FrameFeatures,
        features_curr: FrameFeatures,
    ) -> MatchResult:
        """
        İki frame arasında feature matching yapar.

        Adımlar:
            1. Descriptor validasyonu
            2. BF kNN eşleştirme (k=2)
            3. Lowe's Ratio Test
            4. Koordinat array'lerine dönüştürme

        Args:
            features_prev: Önceki frame'in FrameFeatures'ı.
            features_curr: Mevcut frame'in FrameFeatures'ı.

        Returns:
            MatchResult instance'ı.

        Raises:
            MatcherError: Descriptor yoksa veya matching başarısız olursa.
        """
        if self._matcher_type == "lightglue":
            return self._match_lightglue(features_prev, features_curr)

        # Descriptor validasyonu
        if not features_prev.has_descriptors:
            raise MatcherError(
                f"Önceki frame'de descriptor yok: {features_prev.frame_name}"
            )
        if not features_curr.has_descriptors:
            raise MatcherError(
                f"Mevcut frame'de descriptor yok: {features_curr.frame_name}"
            )

        try:
            # 1. kNN eşleştirme (k=2) — Lowe's Ratio Test için 2 aday gerekir
            knn_matches = self._bf_matcher.knnMatch(
                features_prev.descriptors,
                features_curr.descriptors,
                k=2,
            )

            # 2. Lowe's Ratio Test
            good_matches = self._apply_lowe_ratio_test(knn_matches)

            logger.debug(
                "[Matcher] %s → %s: %d/%d eşleşme Lowe's Ratio Test'i geçti.",
                features_prev.frame_name,
                features_curr.frame_name,
                len(good_matches),
                len(knn_matches),
            )

            # 3. Koordinat array'lerine dönüştür
            if len(good_matches) == 0:
                pts_prev = np.empty((0, 2), dtype=np.float32)
                pts_curr = np.empty((0, 2), dtype=np.float32)
            else:
                pts_prev, pts_curr = self._matches_to_points(
                    good_matches,
                    features_prev.keypoints,
                    features_curr.keypoints,
                )

            result = MatchResult(
                frame_name_prev=features_prev.frame_name,
                frame_name_curr=features_curr.frame_name,
                pts_prev=pts_prev,
                pts_curr=pts_curr,
                matches=good_matches,
            )

            if not result.has_enough_matches:
                logger.warning(
                    "[Matcher] Yetersiz eşleşme: %d (minimum 8 gerekli). "
                    "Deep features fallback tetiklenebilir.",
                    result.match_count,
                )

            return result

        except cv2.error as e:
            raise MatcherError(
                f"OpenCV matching hatası "
                f"({features_prev.frame_name} → {features_curr.frame_name}): {e}"
            ) from e

    def __repr__(self) -> str:
        if self._matcher_type == "lightglue":
            return f"Matcher(type=lightglue, device={self._lg_device})"
        return f"Matcher(type=classical, lowe_ratio={self._lowe_ratio:.2f})"


# ------------------------------------------------------------------
# Standalone test
# ------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    from pathlib import Path

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    config_path = Path(__file__).resolve().parent.parent / "config.yaml"

    try:
        from utils.data_loader import DataLoader, DataLoaderError
        from utils.camera_calibration import CameraCalibration, CameraCalibrationError
        from core.feature_extractor import FeatureExtractor

        loader = DataLoader(str(config_path))
        cam = CameraCalibration(loader)
        extractor = FeatureExtractor(loader, cam)
        matcher = Matcher(loader)

        print(f"\n{matcher}")
        print("\n--- Matching Testi (ilk 5 frame çifti) ---")

        prev_features = None
        match_counts = []

        for idx, name, frame in loader.frame_generator():
            curr_features = extractor.extract(frame, name)

            if prev_features is not None:
                result = matcher.match(prev_features, curr_features)
                match_counts.append(result.match_count)
                print(f"  {result}")

            prev_features = curr_features

            if idx >= 5:
                break

        if match_counts:
            print(f"\nİstatistikler:")
            print(f"  Ortalama eşleşme : {np.mean(match_counts):.1f}")
            print(f"  Min eşleşme      : {min(match_counts)}")
            print(f"  Max eşleşme      : {max(match_counts)}")

        # Debug görsel — ilk frame çifti
        print("\n--- Debug Görsel Üretiliyor ---")
        frame_list = loader.frame_list

        f0 = extractor.extract(loader.load_frame(frame_list[0]), frame_list[0].name)
        f1 = extractor.extract(loader.load_frame(frame_list[1]), frame_list[1].name)
        result = matcher.match(f0, f1)

        debug_img = cv2.drawMatches(
            f0.clean_frame, f0.keypoints,
            f1.clean_frame, f1.keypoints,
            result.matches[:50],  # ilk 50 eşleşme
            None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
        )
        cv2.imwrite("debug_matches.jpg", debug_img)
        print(f"  debug_matches.jpg → {min(50, result.match_count)} eşleşme görseli")

    except Exception as e:
        logger.error("Hata: %s", e)
        sys.exit(1)