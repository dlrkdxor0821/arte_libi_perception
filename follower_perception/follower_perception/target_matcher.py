import numpy as np

from .color_hist import hsv_hist, hist_similarity
from .profile import save_profile, load_profile
from .constants import (
    REID_THRESHOLD, HSV_THRESHOLD, VERIFY_FRAMES,
    CALIBRATION_ADD_THRESHOLD, MAX_GALLERY_SIZE,
)


def reid_backend_name(reid):
    """Best-effort backend label for compatibility checks."""
    return getattr(reid, "_backend", reid.__class__.__name__)


class TargetMatcher:
    """Identifies the owner among tracked candidates via ReID + HSV dual gate,
    locking a safe_id after VERIFY_FRAMES and expanding an online gallery.

    hsv_threshold: HSV correlation floor. None disables the HSV gate (ReID only)
    — useful when a single-photo HSV template is fragile under new lighting.
    """

    def __init__(self, reid, hsv_threshold=HSV_THRESHOLD):
        self.reid = reid
        self.hsv_threshold = hsv_threshold
        self.template_reid = None
        self.template_hsv = None
        self.gallery = []
        self.safe_id = None
        self._verify = {}      # track_id -> consecutive pass count
        self.last_reid_sim = None
        self.last_hsv_sim = None

    @property
    def is_registered(self):
        return self.template_reid is not None

    def register(self, roi_bgr):
        self.template_reid = self.reid.extract(roi_bgr)
        self.template_hsv = hsv_hist(roi_bgr)
        self.gallery = [self.template_reid]
        self.safe_id = None
        self._verify.clear()

    def reset(self):
        self.template_reid = None
        self.template_hsv = None
        self.gallery = []
        self.safe_id = None
        self._verify.clear()

    def save(self, dir, *, crop_bgr, meta):
        if not self.is_registered:
            raise ValueError("cannot save: matcher is not registered")
        full_meta = dict(meta)
        full_meta["reid_backend"] = reid_backend_name(self.reid)
        full_meta["feat_dim"] = int(getattr(self.reid, "feat_dim", len(self.template_reid)))
        save_profile(
            dir,
            crop_bgr=crop_bgr,
            reid_vec=self.template_reid,
            hsv_vec=self.template_hsv,
            gallery=self.gallery,
            meta=full_meta,
        )

    def load(self, dir, *, strict=False):
        data = load_profile(dir)
        meta = data["meta"]
        cur_backend = reid_backend_name(self.reid)
        cur_dim = int(getattr(self.reid, "feat_dim", len(data["reid"])))
        matched_backend = (meta.get("reid_backend") == cur_backend
                           and int(meta.get("feat_dim", -1)) == cur_dim)
        self.template_hsv = np.asarray(data["hsv"], dtype=np.float32)
        if matched_backend:
            self.template_reid = np.asarray(data["reid"], dtype=np.float32)
            gal = data["gallery"]
            self.gallery = [np.asarray(g, dtype=np.float32) for g in gal] \
                if len(gal) else [self.template_reid]
        else:
            if strict:
                raise ValueError(
                    f"profile backend {meta.get('reid_backend')}"
                    f"({meta.get('feat_dim')}) != current {cur_backend}({cur_dim})")
            crop = data["crop"]
            if crop is None or not getattr(crop, "size", 0):
                raise ValueError("backend mismatch and no usable crop to re-extract")
            # Portability: re-extract the embedding with the current backend.
            print(f"[TargetMatcher] backend mismatch "
                  f"({meta.get('reid_backend')} -> {cur_backend}); "
                  f"re-extracting embedding from crop.jpg")
            self.template_reid = self.reid.extract(crop)
            self.gallery = [self.template_reid]
        self.safe_id = None
        self._verify.clear()

    def match(self, cands, frame):
        if not self.is_registered:
            return None
        # Fast path: known owner id present this frame.
        if self.safe_id is not None:
            for c in cands:
                if c.track_id == self.safe_id:
                    return self.safe_id
            return None
        # Evaluate candidates against the dual gate.
        matched = None
        for c in cands:
            roi = self._crop(frame, c.bbox)
            reid_vec = self.reid.extract(roi)
            hsv_vec = hsv_hist(roi)
            reid_sim = max(self.reid.similarity(g, reid_vec) for g in self.gallery)
            hsv_sim = hist_similarity(self.template_hsv, hsv_vec)
            self.last_reid_sim = reid_sim
            self.last_hsv_sim = hsv_sim
            hsv_ok = self.hsv_threshold is None or hsv_sim >= self.hsv_threshold
            if reid_sim >= REID_THRESHOLD and hsv_ok:
                cnt = self._verify.get(c.track_id, 0) + 1
                self._verify[c.track_id] = cnt
                matched = c.track_id
                if cnt >= VERIFY_FRAMES:
                    self.safe_id = c.track_id
                    return c.track_id
            else:
                self._verify[c.track_id] = 0
        return matched

    def calibrate(self, owner_roi):
        reid_vec = self.reid.extract(owner_roi)
        best = max((self.reid.similarity(g, reid_vec) for g in self.gallery),
                   default=0.0)
        if best < CALIBRATION_ADD_THRESHOLD and len(self.gallery) < MAX_GALLERY_SIZE:
            self.gallery.append(reid_vec)

    @staticmethod
    def _crop(frame, bbox):
        x1, y1, x2, y2 = (int(round(v)) for v in bbox)
        x1 = max(0, x1)
        y1 = max(0, y1)
        roi = frame[y1:y2, x1:x2]
        return roi if roi.size else frame
