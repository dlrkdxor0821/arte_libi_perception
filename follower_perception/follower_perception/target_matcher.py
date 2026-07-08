from .color_hist import hsv_hist, hist_similarity
from .constants import (
    REID_THRESHOLD, HSV_THRESHOLD, VERIFY_FRAMES,
    CALIBRATION_ADD_THRESHOLD, MAX_GALLERY_SIZE,
)


class TargetMatcher:
    """Identifies the owner among tracked candidates via ReID + HSV dual gate,
    locking a safe_id after VERIFY_FRAMES and expanding an online gallery."""

    def __init__(self, reid):
        self.reid = reid
        self.template_reid = None
        self.template_hsv = None
        self.gallery = []
        self.safe_id = None
        self._verify = {}      # track_id -> consecutive pass count

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
            if reid_sim >= REID_THRESHOLD and hsv_sim >= HSV_THRESHOLD:
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
