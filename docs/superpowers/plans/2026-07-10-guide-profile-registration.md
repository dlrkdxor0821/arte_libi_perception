# Guide Profile Registration & Persistent ReID Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 사진 1장으로 사람을 등록해 프로필(ReID+HSV+crop)을 디스크에 저장하고, 이후 영상에서 그 사람이 계속 ReID 되는지 실제 파이프라인으로 검증하는 기능을 `follower_perception`에 추가한다.

**Architecture:** 기존 `TargetMatcher`/`FollowerPerception`(ReID+HSV 이중게이트)에 최소 추가 — 프로필 직렬화(`profile.py`), matcher `save/load`(백엔드 불일치 시 crop 재추출), 사진1장 등록(`register_from_image`), 검증 CLI(`scripts/register_and_track.py`). 매칭/스무딩/coasting 로직은 그대로 재사용한다.

**Tech Stack:** Python 3.10+, numpy, opencv-python (설치됨), ultralytics/torch(실행 시에만, 지연 임포트). pytest. 단위 테스트는 ReID `backend='colour'`로 하드웨어/실모델 없이 돈다.

**Design doc:** [Spec 3 — 가이드 프로필 등록](../specs/2026-07-10-guide-profile-registration-design.md).

## Global Constraints

- **커밋 금지** — 에이전트는 절대 `git commit` 하지 않는다. 각 태스크 끝은 **체크포인트**이며 사용자가 검토 후 직접 커밋한다. 계획의 "Checkpoint" 스텝은 사용자용 커밋 제안일 뿐이다.
- **순수 파이썬 코어** — `follower_perception/` 안에 `rclpy`/ROS/네트워크 임포트 금지. `ultralytics`/`torch`는 `Detector`/실 ReID 백엔드 안에서만 **지연 임포트**.
- **이식성 1급** — 경로는 `__file__` 기준 상대해석(cwd 무관), 저장물에 절대경로 금지, `crop.jpg`가 진실원본(다른 백엔드/머신에서 재추출 가능), device 자동+override.
- **결정성** — 단위 테스트에 벽시계(wall-clock) 사용 금지. 타임스탬프 등 비결정 값은 인자로 주입한다. ReID는 `backend='colour'`(모델 다운로드 불필요).
- **파라미터는 참고용** — 임계값은 `constants.py`. HSV 완화는 인스턴스 필드/CLI 플래그로만 조정, 상수 기본값은 불변.
- **테스트 실행** — `follower_perception/` 패키지 루트에서 `python3 -m pytest`.

---

### Task 1: 모델 vendoring + 이식 가능한 가중치 경로 + person 클래스 헬퍼

**Files:**
- Create: `follower_perception/weights/best.pt` (복사본, 코드 아님)
- Modify: `follower_perception/follower_perception/detector.py`
- Test: `follower_perception/tests/test_detector.py` (기존 파일에 추가)

**Interfaces:**
- Produces: `default_weights_path() -> str` (env `FOLLOWER_WEIGHTS` → 패키지상대 `weights/best.pt` 존재 시 그 경로 → 아니면 `'yolo11n.pt'`); `is_person_class0(names: dict) -> bool`; `Detector(weights=None, ...)` — `weights=None`이면 `default_weights_path()` 사용.

- [ ] **Step 1: 모델 파일 복사**

```bash
mkdir -p follower_perception/weights
cp /home/ane/personal_repo/labeling_sam3/model/web-person/train/weights/best.pt \
   follower_perception/weights/best.pt
ls -la follower_perception/weights/best.pt   # ~5.2M 확인
```
> 이 파일은 커밋하지 않는다(사용자 판단). git 추적을 원치 않으면 사용자가 `.gitignore`에 `follower_perception/weights/`를 추가할 수 있다.

- [ ] **Step 2: 실패하는 테스트 작성**

`follower_perception/tests/test_detector.py` 끝에 추가:
```python
from follower_perception.detector import default_weights_path, is_person_class0


def test_default_weights_prefers_env(monkeypatch):
    monkeypatch.setenv("FOLLOWER_WEIGHTS", "/custom/x.pt")
    assert default_weights_path() == "/custom/x.pt"


def test_default_weights_falls_back_when_absent(monkeypatch):
    monkeypatch.delenv("FOLLOWER_WEIGHTS", raising=False)
    monkeypatch.setattr("follower_perception.detector.os.path.exists", lambda p: False)
    assert default_weights_path() == "yolo11n.pt"


def test_default_weights_uses_package_relative_when_present(monkeypatch):
    monkeypatch.delenv("FOLLOWER_WEIGHTS", raising=False)
    monkeypatch.setattr("follower_perception.detector.os.path.exists", lambda p: True)
    got = default_weights_path()
    assert got.endswith("weights/best.pt")
    assert os.path.isabs(got)


def test_is_person_class0():
    assert is_person_class0({0: "person"}) is True
    assert is_person_class0({0: "Person", 1: "car"}) is True
    assert is_person_class0({0: "car"}) is False
    assert is_person_class0({}) is False
```
파일 상단에 `import os` 가 없으면 추가한다.

- [ ] **Step 3: 실패 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_detector.py -v`
Expected: FAIL — `ImportError: cannot import name 'default_weights_path'`

- [ ] **Step 4: 구현**

`follower_perception/follower_perception/detector.py` 를 다음으로 수정 (상단 import + 함수 추가, `__init__` 기본값 변경):
```python
import os

from .detection import TrackedBox
from .constants import MIN_CONFIDENCE


def default_weights_path():
    """Resolve YOLO weights portably: env override → package-relative
    weights/best.pt if present → stock yolo11n.pt (auto-download)."""
    env = os.environ.get("FOLLOWER_WEIGHTS")
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(here, "..", "weights", "best.pt"))
    if os.path.exists(candidate):
        return candidate
    return "yolo11n.pt"


def is_person_class0(names):
    """True if class index 0 maps to 'person' (case-insensitive)."""
    if not isinstance(names, dict):
        return False
    return str(names.get(0, "")).lower() == "person"


class Detector:
    """YOLO11n detection + built-in ByteTrack. Person (class 0) only."""

    def __init__(self, weights=None, conf=MIN_CONFIDENCE,
                 tracker_cfg='bytetrack.yaml', device=None):
        from ultralytics import YOLO
        self.model = YOLO(weights or default_weights_path())
        self.conf = conf
        self.tracker_cfg = tracker_cfg
        self.device = device

    def detect(self, frame):
        results = self.model.track(
            frame, persist=True, conf=self.conf, classes=[0],
            tracker=self.tracker_cfg, verbose=False, device=self.device,
        )
        if not results:
            return []
        return self._to_tracked_boxes(results[0])

    @staticmethod
    def _to_tracked_boxes(result):
        boxes = getattr(result, 'boxes', None)
        if boxes is None or getattr(boxes, 'id', None) is None:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        ids = boxes.id.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        out = []
        for (x1, y1, x2, y2), tid, conf in zip(xyxy, ids, confs):
            x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
            w, h = x2 - x1, y2 - y1
            out.append(TrackedBox(
                bbox=(x1, y1, x2, y2),
                cx=(x1 + x2) / 2.0,
                cy=(y1 + y2) / 2.0,
                area=w * h,
                track_id=int(tid),
                confidence=float(conf),
            ))
        return out
```

- [ ] **Step 5: 통과 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_detector.py -v`
Expected: PASS (기존 3개 + 신규 4개)

- [ ] **Step 6: Checkpoint (사용자 커밋)**

전체 스위트 확인 후 사용자가 커밋:
```bash
cd follower_perception && python3 -m pytest -q   # 여전히 green
# (사용자) git add follower_perception/follower_perception/detector.py \
#                  follower_perception/tests/test_detector.py
# (사용자) git commit -m "feat(perception): portable weights path + person-class helper"
# best.pt 추적 여부는 사용자 판단
```

---

### Task 2: 프로필 직렬화 (`profile.py`)

**Files:**
- Create: `follower_perception/follower_perception/profile.py`
- Test: `follower_perception/tests/test_profile.py`

**Interfaces:**
- Consumes: numpy, cv2, json, os.
- Produces: `save_profile(dir, *, crop_bgr, reid_vec, hsv_vec, gallery, meta) -> None` (dir 생성, `crop.jpg`+`features.npz`(reid,hsv,gallery)+`meta.json` 기록); `load_profile(dir) -> dict` (`{"crop","reid","hsv","gallery","meta"}`; crop 없거나 npz/meta 부재 시 `FileNotFoundError`).

- [ ] **Step 1: 실패하는 테스트 작성**

`follower_perception/tests/test_profile.py`:
```python
import numpy as np
import pytest
from follower_perception.profile import save_profile, load_profile


def _crop(color_bgr=(0, 0, 255), h=40, w=20):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color_bgr
    return img


def test_round_trip_preserves_arrays_and_meta(tmp_path):
    d = str(tmp_path / "p1")
    reid = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    hsv = np.arange(48, dtype=np.float32)
    gallery = [reid, reid * 0.5]
    meta = {"name": "visitor1", "reid_backend": "colour", "feat_dim": 3}
    save_profile(d, crop_bgr=_crop(), reid_vec=reid, hsv_vec=hsv,
                 gallery=gallery, meta=meta)

    got = load_profile(d)
    assert got["meta"]["name"] == "visitor1"
    assert got["meta"]["feat_dim"] == 3
    np.testing.assert_allclose(got["reid"], reid, rtol=1e-6)
    np.testing.assert_allclose(got["hsv"], hsv, rtol=1e-6)
    assert got["gallery"].shape == (2, 3)
    assert got["crop"].shape == (40, 20, 3)


def test_load_missing_dir_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_profile(str(tmp_path / "nope"))
```

- [ ] **Step 2: 실패 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_profile.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.profile'`

- [ ] **Step 3: 구현**

`follower_perception/follower_perception/profile.py`:
```python
"""Self-contained on-disk person profile: crop.jpg + features.npz + meta.json.

A profile folder is portable — copy it anywhere and it still loads. No absolute
paths are stored. `crop.jpg` is the source of truth so backend-specific
embeddings can be re-extracted on a different machine/backend.
"""
import json
import os

import cv2
import numpy as np

CROP_NAME = "crop.jpg"
FEATURES_NAME = "features.npz"
META_NAME = "meta.json"


def save_profile(dir, *, crop_bgr, reid_vec, hsv_vec, gallery, meta):
    os.makedirs(dir, exist_ok=True)
    cv2.imwrite(os.path.join(dir, CROP_NAME), crop_bgr)
    gallery_arr = np.stack([np.asarray(g, dtype=np.float32) for g in gallery]) \
        if len(gallery) else np.zeros((0, len(reid_vec)), dtype=np.float32)
    np.savez(
        os.path.join(dir, FEATURES_NAME),
        reid=np.asarray(reid_vec, dtype=np.float32),
        hsv=np.asarray(hsv_vec, dtype=np.float32),
        gallery=gallery_arr,
    )
    with open(os.path.join(dir, META_NAME), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)


def load_profile(dir):
    crop_path = os.path.join(dir, CROP_NAME)
    feats_path = os.path.join(dir, FEATURES_NAME)
    meta_path = os.path.join(dir, META_NAME)
    for p in (crop_path, feats_path, meta_path):
        if not os.path.exists(p):
            raise FileNotFoundError(f"profile file missing: {p}")
    crop = cv2.imread(crop_path)
    if crop is None:
        raise FileNotFoundError(f"unreadable crop: {crop_path}")
    feats = np.load(feats_path)
    with open(meta_path, encoding="utf-8") as f:
        meta = json.load(f)
    return {
        "crop": crop,
        "reid": feats["reid"],
        "hsv": feats["hsv"],
        "gallery": feats["gallery"],
        "meta": meta,
    }
```

- [ ] **Step 4: 통과 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_profile.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Checkpoint (사용자 커밋)**

```bash
# (사용자) git add follower_perception/follower_perception/profile.py \
#                  follower_perception/tests/test_profile.py
# (사용자) git commit -m "feat(perception): portable profile serialization (crop+features+meta)"
```

---

### Task 3: `TargetMatcher` HSV 게이트 완화 (인스턴스 필드)

**Files:**
- Modify: `follower_perception/follower_perception/target_matcher.py`
- Test: `follower_perception/tests/test_target_matcher.py` (기존 파일에 추가)

**Interfaces:**
- Consumes: 기존 constants.
- Produces: `TargetMatcher(reid, hsv_threshold=HSV_THRESHOLD)` — `match()`가 `self.hsv_threshold` 사용, `None`이면 HSV 게이트 무시(ReID 단독). 진단용 `self.last_reid_sim`/`self.last_hsv_sim`(마지막 평가 후보값, 초기 `None`).

- [ ] **Step 1: 실패하는 테스트 작성**

`follower_perception/tests/test_target_matcher.py` 끝에 추가. HSV만 격리하기 위해 ReID를 항상 통과시키는 스텁을 쓴다:
```python
class _AlwaysSimReID:
    """ReID stub: extract returns a constant vector, similarity always 1.0.
    Isolates the HSV gate from ReID."""
    feat_dim = 3

    def extract(self, roi_bgr):
        return np.ones(3, dtype=np.float32)

    def similarity(self, a, b):
        return 1.0


def _box_full(tid, w=64, h=64):
    return TrackedBox(bbox=(0, 0, w, h), cx=w / 2, cy=h / 2, area=w * h,
                      track_id=tid, confidence=0.9)


def _frame_bgr(color, w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color
    return img


def test_default_hsv_gate_rejects_different_color():
    m = TargetMatcher(_AlwaysSimReID())        # default hsv_threshold
    m.register(_frame_bgr((0, 0, 255)))        # red template
    assert m.match([_box_full(1)], _frame_bgr((255, 0, 0))) is None  # blue -> HSV fails


def test_hsv_none_matches_despite_different_color():
    m = TargetMatcher(_AlwaysSimReID(), hsv_threshold=None)
    m.register(_frame_bgr((0, 0, 255)))        # red template
    # ReID always passes, HSV ignored -> blue candidate still matches
    assert m.match([_box_full(1)], _frame_bgr((255, 0, 0))) == 1


def test_last_sims_recorded_after_match():
    m = TargetMatcher(_AlwaysSimReID())
    m.register(_frame_bgr((0, 0, 255)))
    m.match([_box_full(1)], _frame_bgr((0, 0, 255)))
    assert m.last_reid_sim is not None
    assert m.last_hsv_sim is not None
```
(파일에 `import numpy as np`, `from follower_perception.detection import TrackedBox`, `from follower_perception.target_matcher import TargetMatcher` 가 이미 있으면 재선언 불필요.)

- [ ] **Step 2: 실패 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_target_matcher.py -v`
Expected: FAIL — `TypeError: __init__() got an unexpected keyword argument 'hsv_threshold'`

- [ ] **Step 3: 구현**

`follower_perception/follower_perception/target_matcher.py` 의 import, `__init__`, `match` 를 수정:
```python
from .color_hist import hsv_hist, hist_similarity
from .constants import (
    REID_THRESHOLD, HSV_THRESHOLD, VERIFY_FRAMES,
    CALIBRATION_ADD_THRESHOLD, MAX_GALLERY_SIZE,
)


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
```
그리고 `match` 의 게이트 판정 부분을 다음으로 교체(중간 루프):
```python
    def match(self, cands, frame):
        if not self.is_registered:
            return None
        # Fast path: known owner id present this frame.
        if self.safe_id is not None:
            for c in cands:
                if c.track_id == self.safe_id:
                    return self.safe_id
            return None
        # Evaluate candidates against the (optionally HSV-relaxed) gate.
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
```
`calibrate`, `_crop`, `register`, `reset`, `is_registered` 는 그대로. (단 `reset`은 `last_reid_sim/last_hsv_sim`을 굳이 지우지 않아도 되지만, 원하면 `None`으로 리셋 추가 가능 — 기능상 무관.)

- [ ] **Step 4: 통과 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_target_matcher.py -v`
Expected: PASS (기존 6개 + 신규 3개). 기본 임계값 불변이라 기존 테스트도 green.

- [ ] **Step 5: Checkpoint (사용자 커밋)**

```bash
# (사용자) git add follower_perception/follower_perception/target_matcher.py \
#                  follower_perception/tests/test_target_matcher.py
# (사용자) git commit -m "feat(perception): relaxable HSV gate (instance hsv_threshold) + last sims"
```

---

### Task 4: `TargetMatcher` save/load + cross-backend 재추출

**Files:**
- Modify: `follower_perception/follower_perception/target_matcher.py`
- Test: `follower_perception/tests/test_target_matcher.py` (추가)

**Interfaces:**
- Consumes: `profile.save_profile`, `profile.load_profile`, `color_hist.hsv_hist`, `reid.extract`, `reid.feat_dim`, `reid.__class__` 백엔드 식별.
- Produces: `TargetMatcher.save(dir, *, crop_bgr, meta)` — meta에 `reid_backend`/`feat_dim` 주입 후 template/gallery 기록; `TargetMatcher.load(dir, *, strict=False)` — 복원. backend/feat_dim 불일치 시 crop에서 재추출(경고), `strict=True`면 `ValueError`, crop 없으면 `ValueError`. `reid_backend_name(reid) -> str` 헬퍼.

- [ ] **Step 1: 실패하는 테스트 작성**

`follower_perception/tests/test_target_matcher.py` 끝에 추가:
```python
import json as _json
import os as _os
from follower_perception.reid_engine import ReIDEngine


def _red_frame(w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (0, 0, 255)
    return img


def test_save_then_load_matches_same_color(tmp_path):
    d = str(tmp_path / "prof")
    m = TargetMatcher(ReIDEngine(backend="colour"))
    m.register(_red_frame())
    m.save(d, crop_bgr=_red_frame(), meta={"name": "v1"})

    m2 = TargetMatcher(ReIDEngine(backend="colour"))
    m2.load(d)
    assert m2.is_registered is True
    assert m2.match([_box_full(1)], _red_frame()) == 1


def test_backend_mismatch_reextracts_from_crop(tmp_path):
    d = str(tmp_path / "prof")
    m = TargetMatcher(ReIDEngine(backend="colour"))
    m.register(_red_frame())
    m.save(d, crop_bgr=_red_frame(), meta={"name": "v1"})
    # Forge a mismatching backend/feat_dim in meta.
    mp = _os.path.join(d, "meta.json")
    meta = _json.load(open(mp))
    meta["reid_backend"] = "osnet"
    meta["feat_dim"] = 512
    _json.dump(meta, open(mp, "w"))

    m2 = TargetMatcher(ReIDEngine(backend="colour"))
    m2.load(d)                       # re-extracts from crop, no raise
    assert m2.is_registered is True
    assert m2.reid.similarity(m2.template_reid, m2.template_reid) == 1.0


def test_strict_load_raises_on_mismatch(tmp_path):
    d = str(tmp_path / "prof")
    m = TargetMatcher(ReIDEngine(backend="colour"))
    m.register(_red_frame())
    m.save(d, crop_bgr=_red_frame(), meta={"name": "v1"})
    mp = _os.path.join(d, "meta.json")
    meta = _json.load(open(mp)); meta["feat_dim"] = 999
    _json.dump(meta, open(mp, "w"))
    m2 = TargetMatcher(ReIDEngine(backend="colour"))
    with pytest.raises(ValueError):
        m2.load(d, strict=True)


def test_load_raises_when_crop_missing_and_mismatch(tmp_path):
    d = str(tmp_path / "prof")
    m = TargetMatcher(ReIDEngine(backend="colour"))
    m.register(_red_frame())
    m.save(d, crop_bgr=_red_frame(), meta={"name": "v1"})
    mp = _os.path.join(d, "meta.json")
    meta = _json.load(open(mp)); meta["feat_dim"] = 999
    _json.dump(meta, open(mp, "w"))
    _os.remove(_os.path.join(d, "crop.jpg"))
    m2 = TargetMatcher(ReIDEngine(backend="colour"))
    with pytest.raises((ValueError, FileNotFoundError)):
        m2.load(d)
```
(파일 상단에 `import pytest` 가 없으면 추가.)

- [ ] **Step 2: 실패 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_target_matcher.py -k "save or load or backend or strict or crop" -v`
Expected: FAIL — `AttributeError: 'TargetMatcher' object has no attribute 'save'`

- [ ] **Step 3: 구현**

`follower_perception/follower_perception/target_matcher.py` 상단 import에 profile 헬퍼 추가하고, 클래스에 `save`/`load` + 모듈함수 `reid_backend_name` 추가:
```python
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
```
클래스 내부(예: `reset` 아래)에 추가:
```python
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
```
> 참고: `ReIDEngine`은 이미 `self._backend`('colour'/'osnet'/'mobilenet')와 `self.feat_dim`을 노출한다(Spec 1). `reid_backend_name`이 이를 사용.

- [ ] **Step 4: 통과 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_target_matcher.py -v`
Expected: PASS (신규 4개 포함 전부)

- [ ] **Step 5: Checkpoint (사용자 커밋)**

```bash
# (사용자) git add follower_perception/follower_perception/target_matcher.py \
#                  follower_perception/tests/test_target_matcher.py
# (사용자) git commit -m "feat(perception): TargetMatcher save/load with cross-backend re-extraction"
```

---

### Task 5: `FollowerPerception.register_from_image` + `save_profile`

**Files:**
- Modify: `follower_perception/follower_perception/pipeline.py`
- Test: `follower_perception/tests/test_pipeline.py` (추가)

**Interfaces:**
- Consumes: `detector.detect`, `_pick_central`, `TargetMatcher._crop`, `matcher.register`, `matcher.save`.
- Produces: `FollowerPerception.register_from_image(image_bgr) -> TrackedBox | None` (한 프레임에서 중앙 사람 등록, 3프레임 안정 우회; 성공 시 `self._last_crop`/`self._last_bbox` 저장); `FollowerPerception.save_profile(dir, *, name, source_image=None, registered_at=None) -> None`.

- [ ] **Step 1: 실패하는 테스트 작성**

`follower_perception/tests/test_pipeline.py` 끝에 추가:
```python
def test_register_from_image_registers_central_person():
    p = _perception([])   # detector script unused; we set it below
    from follower_perception.mocks import MockDetector
    p.detector = MockDetector([[_full_box(1)]])
    red = _frame((0, 0, 255))
    box = p.register_from_image(red)
    assert box is not None
    assert box.track_id == 1
    assert p.matcher.is_registered is True


def test_register_from_image_no_person_returns_none():
    from follower_perception.mocks import MockDetector
    p = _perception([])
    p.detector = MockDetector([[]])
    assert p.register_from_image(_frame((0, 0, 255))) is None
    assert p.matcher.is_registered is False


def test_save_profile_writes_folder(tmp_path):
    from follower_perception.mocks import MockDetector
    p = _perception([])
    p.detector = MockDetector([[_full_box(1)]])
    p.register_from_image(_frame((0, 0, 255)))
    d = str(tmp_path / "v1")
    p.save_profile(d, name="v1", source_image="x.jpg", registered_at="2026-07-10T00:00:00")
    import os
    assert os.path.exists(os.path.join(d, "crop.jpg"))
    assert os.path.exists(os.path.join(d, "meta.json"))
```
> `_perception([])`는 기존 헬퍼(`FollowerPerception(detector=MockDetector([]), reid=ReIDEngine('colour'))`)를 재사용한다. 위 테스트는 `p.detector`를 원하는 스크립트로 교체한다.

- [ ] **Step 2: 실패 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_pipeline.py -k register_from_image -v`
Expected: FAIL — `AttributeError: 'FollowerPerception' object has no attribute 'register_from_image'`

- [ ] **Step 3: 구현**

`follower_perception/follower_perception/pipeline.py` 의 `__init__`에 상태 2개 추가하고, 등록 메서드 아래에 두 메서드 추가:
```python
    def __init__(self, detector=None, reid=None):
        self.detector = detector if detector is not None else Detector()
        self.reid = reid if reid is not None else ReIDEngine()
        self.matcher = TargetMatcher(self.reid)
        self.smoother = BBoxSmoother()
        self._last_owner = None       # last TrackedBox seen as owner
        self._miss = 0
        self._frame_count = 0
        self._reg_id = None
        self._reg_streak = 0
        self._last_crop = None        # crop of the most recent registration
        self._last_bbox = None
```
그리고 `_pick_central` 아래에:
```python
    def register_from_image(self, image_bgr):
        """Register the central person from a SINGLE image (bypasses the
        3-frame stability requirement). Returns the chosen TrackedBox or None."""
        cands = self.detector.detect(image_bgr)
        target = self._pick_central(cands, image_bgr)
        if target is None:
            return None
        roi = TargetMatcher._crop(image_bgr, target.bbox)
        self.matcher.register(roi)
        self.smoother.reset()
        self._last_owner = None
        self._miss = 0
        self._reg_id = None
        self._reg_streak = 0
        self._last_crop = roi
        self._last_bbox = list(target.bbox)
        return target

    def save_profile(self, dir, *, name, source_image=None, registered_at=None):
        if self._last_crop is None:
            raise ValueError("no registered crop to save; call register_from_image first")
        meta = {
            "name": name,
            "registered_at": registered_at,
            "source_image": source_image,
            "bbox": self._last_bbox,
        }
        self.matcher.save(dir, crop_bgr=self._last_crop, meta=meta)
```

- [ ] **Step 4: 통과 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_pipeline.py -v`
Expected: PASS (기존 + 신규 3개)

- [ ] **Step 5: Checkpoint (사용자 커밋)**

```bash
# (사용자) git add follower_perception/follower_perception/pipeline.py \
#                  follower_perception/tests/test_pipeline.py
# (사용자) git commit -m "feat(perception): single-image registration + save_profile on pipeline"
```

---

### Task 6: `load_profile` + `track_frames`/`summarize` + CLI 스크립트

**Files:**
- Modify: `follower_perception/follower_perception/pipeline.py` (`load_profile` 추가)
- Create: `follower_perception/follower_perception/tracking_report.py` (`track_frames`, `summarize`)
- Create: `follower_perception/scripts/register_and_track.py` (CLI)
- Test: `follower_perception/tests/test_tracking_report.py`
- Test: `follower_perception/tests/test_pipeline.py` (`load_profile` 라운드트립 1개 추가)

**Interfaces:**
- Consumes: `FollowerPerception.run/get_latest`, `matcher.load`, `matcher.last_reid_sim/last_hsv_sim`.
- Produces: `FollowerPerception.load_profile(dir, *, strict=False) -> None`; `track_frames(frames, perception, *, log=None) -> dict`; `summarize(records) -> dict` (`records`는 `track_frames` 내부 프레임 기록 리스트). CLI: `register`/`track` 서브커맨드.

- [ ] **Step 1: 실패하는 테스트 작성**

`follower_perception/tests/test_tracking_report.py`:
```python
import numpy as np
from follower_perception.reid_engine import ReIDEngine
from follower_perception.mocks import MockDetector
from follower_perception.pipeline import FollowerPerception
from follower_perception.detection import TrackedBox
from follower_perception.tracking_report import track_frames, summarize
from follower_perception import constants


def _frame(color=(0, 0, 255), w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color
    return img


def _full_box(tid, w=64, h=64):
    return TrackedBox(bbox=(0, 0, w, h), cx=w / 2, cy=h / 2, area=w * h,
                      track_id=tid, confidence=0.9)


def test_track_frames_owner_held_every_frame():
    n = 6
    p = FollowerPerception(detector=MockDetector([[_full_box(1)]] * n),
                           reid=ReIDEngine(backend="colour"))
    p.register_from_image(_frame((0, 0, 255)))
    p.detector = MockDetector([[_full_box(1)]] * n)   # reset script for tracking
    frames = [_frame((0, 0, 255)) for _ in range(n)]
    summary = track_frames(frames, p)
    assert summary["frames"] == n
    assert summary["owner_hold_ratio"] == 1.0
    assert summary["max_miss_streak"] == 0


def test_track_frames_reports_misses():
    n = 4
    p = FollowerPerception(detector=MockDetector([[_full_box(1)]]),
                           reid=ReIDEngine(backend="colour"))
    p.register_from_image(_frame((0, 0, 255)))
    # Owner never appears in tracking -> no owner frames, misses accumulate.
    p.detector = MockDetector([[]] * n)
    frames = [_frame((0, 0, 255)) for _ in range(n)]
    summary = track_frames(frames, p)
    assert summary["owner_frames"] == 0
    assert summary["owner_hold_ratio"] == 0.0
    assert summary["max_miss_streak"] == n


def test_summarize_counts_predicted_and_streaks():
    records = [
        {"owner": True, "predicted": False},
        {"owner": True, "predicted": True},
        {"owner": False, "predicted": False},
        {"owner": False, "predicted": False},
        {"owner": True, "predicted": False},
    ]
    s = summarize(records)
    assert s["frames"] == 5
    assert s["owner_frames"] == 3
    assert s["predicted_frames"] == 1
    assert s["max_miss_streak"] == 2
    assert abs(s["owner_hold_ratio"] - 0.6) < 1e-9
```

- [ ] **Step 2: 실패 확인**

Run: `cd follower_perception && python3 -m pytest tests/test_tracking_report.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.tracking_report'`

- [ ] **Step 3: 구현 — `tracking_report.py`**

`follower_perception/follower_perception/tracking_report.py`:
```python
"""Turn a stream of frames into a persistent-ReID report. Pure w.r.t. IO:
`track_frames` takes any iterable of BGR frames and a perception object, so it
is unit-testable with a MockDetector and no video codec."""


def summarize(records):
    total = len(records)
    owner = sum(1 for r in records if r["owner"])
    predicted = sum(1 for r in records if r["owner"] and r["predicted"])
    max_miss = streak = 0
    for r in records:
        if r["owner"]:
            streak = 0
        else:
            streak += 1
            max_miss = max(max_miss, streak)
    return {
        "frames": total,
        "owner_frames": owner,
        "owner_hold_ratio": (owner / total) if total else 0.0,
        "predicted_frames": predicted,
        "max_miss_streak": max_miss,
    }


def track_frames(frames, perception, *, log=None):
    records = []
    for i, frame in enumerate(frames):
        perception.run(frame)
        det = perception.get_latest()
        is_owner = det is not None and det.is_owner
        is_pred = bool(det.is_predicted) if is_owner else False
        records.append({"owner": is_owner, "predicted": is_pred})
        if log is not None:
            if is_owner:
                rs = perception.matcher.last_reid_sim
                hs = perception.matcher.last_hsv_sim
                rs = f"{rs:.2f}" if rs is not None else "n/a"
                hs = f"{hs:.2f}" if hs is not None else "n/a"
                log(f"frame {i}: owner track_id={det.track_id} "
                    f"reid_sim={rs} hsv_sim={hs} predicted={is_pred}")
            else:
                log(f"frame {i}: owner not found")
    return summarize(records)
```

- [ ] **Step 4: 구현 — `FollowerPerception.load_profile`**

`follower_perception/follower_perception/pipeline.py` 의 `save_profile` 아래에 추가:
```python
    def load_profile(self, dir, *, strict=False):
        """Load a saved profile into the matcher so tracking can resume without
        re-registration. Cross-backend safe (re-extracts from crop)."""
        self.matcher.load(dir, strict=strict)
        self.smoother.reset()
        self._last_owner = None
        self._miss = 0
        self._frame_count = 0
```
그리고 `follower_perception/tests/test_pipeline.py` 에 라운드트립 1개 추가:
```python
def test_save_then_load_profile_round_trip(tmp_path):
    from follower_perception.mocks import MockDetector
    p = _perception([])
    p.detector = MockDetector([[_full_box(1)]])
    p.register_from_image(_frame((0, 0, 255)))
    d = str(tmp_path / "v1")
    p.save_profile(d, name="v1")

    p2 = _perception([])
    p2.load_profile(d)
    assert p2.matcher.is_registered is True
```

- [ ] **Step 5: 구현 — CLI `scripts/register_and_track.py`**

`follower_perception/scripts/register_and_track.py`:
```python
#!/usr/bin/env python3
"""Register a person from ONE photo, then verify persistent ReID on a video.

Run from the follower_perception/ package root:
    python scripts/register_and_track.py register --image v.jpg --profile profiles/v1
    python scripts/register_and_track.py track    --video walk.mp4 --profile profiles/v1
"""
import argparse
import os
import sys

# make the package importable when run as a script from the package root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402


def _build_perception(device=None, hsv_threshold="default"):
    try:
        from follower_perception.detector import Detector
        from follower_perception.reid_engine import ReIDEngine
        from follower_perception.pipeline import FollowerPerception
        from follower_perception import constants
    except ImportError as e:  # pragma: no cover - env-dependent
        print(f"[error] missing dependency ({e}). Run this in an env with "
              f"ultralytics+torch installed (e.g. your project venv).")
        raise SystemExit(2)
    reid = ReIDEngine(device=device)
    p = FollowerPerception(detector=Detector(device=device), reid=reid)
    if hsv_threshold == "none":
        p.matcher.hsv_threshold = None
    elif hsv_threshold != "default":
        p.matcher.hsv_threshold = float(hsv_threshold)
    return p


def cmd_register(args):
    from follower_perception.detector import is_person_class0
    img = cv2.imread(args.image)
    if img is None:
        print(f"[error] cannot read image: {args.image}"); return 2
    p = _build_perception(device=args.device)
    names = getattr(p.detector.model, "names", None)
    if names is not None and not is_person_class0(names):
        print(f"[warn] class 0 is not 'person' (names={names}); detections may be empty")
    box = p.register_from_image(img)
    if box is None:
        print("[error] no person found in the image"); return 1
    from datetime import datetime
    p.save_profile(args.profile, name=args.name or os.path.basename(args.profile),
                   source_image=os.path.basename(args.image),
                   registered_at=datetime.now().isoformat(timespec="seconds"))
    print(f"[ok] registered track_id={box.track_id}; profile saved to {args.profile}")
    return 0


def _iter_video(path):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"[error] cannot open video: {path}"); raise SystemExit(2)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield frame
    finally:
        cap.release()


def cmd_track(args):
    from follower_perception.tracking_report import track_frames
    hsv = "none" if args.no_hsv else (
        str(args.hsv_threshold) if args.hsv_threshold is not None else "default")
    p = _build_perception(device=args.device, hsv_threshold=hsv)
    try:
        p.load_profile(args.profile, strict=args.strict)
    except (FileNotFoundError, ValueError) as e:
        print(f"[error] load profile failed: {e}"); return 2

    writer = {"w": None}
    frames = _iter_video(args.video)
    if args.out:
        frames = _tee_annotated(frames, p, args.out, writer)
    summary = track_frames(frames, p, log=print)
    if writer["w"] is not None:
        writer["w"].release()
    print("---- summary ----")
    print(f"frames={summary['frames']} owner={summary['owner_frames']} "
          f"hold_ratio={summary['owner_hold_ratio']:.2%} "
          f"predicted={summary['predicted_frames']} "
          f"max_miss_streak={summary['max_miss_streak']}")
    return 0


def _tee_annotated(frames, perception, out_path, writer):
    """Yield frames unchanged, but also write an annotated mp4 using the
    perception's latest detection (called AFTER perception.run inside
    track_frames is not possible, so we draw the previous frame's box)."""
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    for frame in frames:
        det = perception.get_latest()
        vis = frame.copy()
        if det is not None and det.is_owner:
            x1, y1, x2, y2 = (int(v) for v in det.bbox)
            color = (0, 165, 255) if det.is_predicted else (0, 255, 0)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        if writer["w"] is None:
            h, w = vis.shape[:2]
            writer["w"] = cv2.VideoWriter(out_path, fourcc, 20.0, (w, h))
        writer["w"].write(vis)
        yield frame


def main():
    ap = argparse.ArgumentParser(description="Register from photo, verify ReID on video")
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("register")
    r.add_argument("--image", required=True)
    r.add_argument("--profile", required=True)
    r.add_argument("--name", default=None)
    r.add_argument("--device", default=None)
    r.set_defaults(func=cmd_register)

    t = sub.add_parser("track")
    t.add_argument("--video", required=True)
    t.add_argument("--profile", required=True)
    t.add_argument("--out", default=None)
    t.add_argument("--device", default=None)
    t.add_argument("--strict", action="store_true")
    g = t.add_mutually_exclusive_group()
    g.add_argument("--hsv-threshold", dest="hsv_threshold", type=float, default=None)
    g.add_argument("--no-hsv", dest="no_hsv", action="store_true")
    t.set_defaults(func=cmd_track)

    args = ap.parse_args()
    raise SystemExit(args.func(args))


if __name__ == "__main__":
    main()
```
> `_tee_annotated`는 `perception.get_latest()`가 직전 프레임 결과를 반영하므로 박스가 1프레임 지연되어 그려진다(시각 확인용으로 충분). 정밀 오버레이가 필요하면 후속 개선 대상.

- [ ] **Step 6: 통과 확인 (단위 테스트)**

Run: `cd follower_perception && python3 -m pytest tests/test_tracking_report.py tests/test_pipeline.py -v`
Expected: PASS (tracking_report 3개 + pipeline 라운드트립 포함)

- [ ] **Step 7: 전체 스위트 확인**

Run: `cd follower_perception && python3 -m pytest -q`
Expected: PASS (기존 30 + 신규 전부). CLI 스크립트는 import 부작용 없음(‱`if __name__`) 이라 수집에 영향 없음.

- [ ] **Step 8: (옵션) 실모델 스모크 — 환경 의존, 수동**

ultralytics/torch 설치된 환경에서:
```bash
cd follower_perception
python scripts/register_and_track.py register --image <사람사진.jpg> --profile profiles/v1
python scripts/register_and_track.py track    --video <영상.mp4> --profile profiles/v1 --out out.mp4
# 기대: register 성공 로그 + profiles/v1/{crop.jpg,features.npz,meta.json},
#       track 요약의 hold_ratio 가 높게(같은 사람 계속 매칭) 나오고 out.mp4에 박스가 보임
```

- [ ] **Step 9: Checkpoint (사용자 커밋)**

```bash
# (사용자) git add follower_perception/follower_perception/tracking_report.py \
#                  follower_perception/follower_perception/pipeline.py \
#                  follower_perception/scripts/register_and_track.py \
#                  follower_perception/tests/test_tracking_report.py \
#                  follower_perception/tests/test_pipeline.py
# (사용자) git commit -m "feat(perception): register+track CLI, tracking report, load_profile"
```

---

## Self-Review

- **Spec 커버리지:**
  - §3 정본 모델/경로 이식성 → Task 1 (`default_weights_path`, best.pt 복사).
  - §4.1 profile.py → Task 2.
  - §3/§4.2 HSV 완화 → Task 3 (`hsv_threshold`).
  - §3/§4.2/§6 백엔드 이식성(재추출·strict) → Task 4 (`save`/`load`).
  - §4.3 사진1장 등록 → Task 5 (`register_from_image`).
  - §4.4 CLI register/track + 요약 → Task 6 (`tracking_report`, `register_and_track.py`, `load_profile`).
  - §5 데이터계약(파일명만·backend/feat_dim) → Task 2+4.
  - §6 에러처리(사람없음/불일치/미설치/영상열기) → Task 5(None)·4(ValueError)·6(CLI SystemExit).
  - §7 이식성 원칙 → Task 1(경로)·2(자기완결)·4(재추출)·6(지연임포트·device).
  - §8 테스트(round-trip/save-load/cross-backend/HSV/register_from_image) → Task 2~6 테스트.
- **Placeholder 스캔:** 없음 — 모든 스텝에 실제 코드/명령. (죽은 코드였던 CLI `_sink`는 본문에서 제거 완료.)
- **타입 일관성:** `TrackedBox`/`Detection` 필드 불변. `save(dir,*,crop_bgr,meta)`/`load(dir,*,strict)`/`register_from_image(image)->TrackedBox|None`/`save_profile(dir,*,name,...)`/`load_profile(dir,*,strict)`/`track_frames(frames,perception,*,log)`/`summarize(records)` 신규 시그니처가 태스크 간 일치. `reid._backend`/`feat_dim`는 Spec 1 `ReIDEngine` 실제 필드.
