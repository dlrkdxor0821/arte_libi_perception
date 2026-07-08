# Follower Perception Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the `follower_perception` package — a pure-Python vision pipeline that turns camera frames into a single owner `Detection` (detect → track → identify → smooth), plus a thin UDP/TCP server adapter.

**Architecture:** Pure Python core (no ROS, no network) exposing `FollowerPerception.register/run/get_latest/reset`, wrapped by a thin injectable `AiServer` adapter. YOLO11n + built-in ByteTrack for detection/tracking; ReID (OSNet→MobileNet→colour fallback) + HSV histogram dual-gate for owner identification; α-β smoother for coasting.

**Tech Stack:** Python 3.10+, ultralytics (YOLO11n), opencv-python, numpy, torch (optional torchreid). pytest for tests.

**Design docs:** [Spec 1 — perception](../specs/2026-07-08-follower-perception-design.md).

## Global Constraints

- **Pure Python core** — no `rclpy`, no ROS imports anywhere in `follower_perception/`. ROS lives only in Spec 2.
- **Public contract is `Detection`** — `get_latest()` returns `Detection | None`; nothing else leaks pipeline internals.
- **Parameters are reference-only** — all thresholds live in `constants.py`; values are tuning starting points, not requirements.
- **Person class only** — YOLO detects COCO class 0 (person). Stock `yolo11n.pt` (auto-downloaded) is the starting weight.
- **Determinism in tests** — no wall-clock in unit tests; `dt` is passed explicitly. Use `backend='colour'` for ReID in unit tests so no model download is needed.
- **Test framework** — pytest, run from the `follower_perception/` package root (editable install).

---

### Task 1: Package scaffolding + Detection contract + constants

**Files:**
- Create: `follower_perception/pyproject.toml`
- Create: `follower_perception/requirements.txt`
- Create: `follower_perception/bytetrack.yaml`
- Create: `follower_perception/follower_perception/__init__.py`
- Create: `follower_perception/follower_perception/detection.py`
- Create: `follower_perception/follower_perception/constants.py`
- Create: `follower_perception/tests/__init__.py`
- Test: `follower_perception/tests/test_detection.py`

**Interfaces:**
- Produces: `Detection(cx, cy, area, bbox, track_id, is_owner, confidence, is_predicted)` and `TrackedBox(bbox, cx, cy, area, track_id, confidence)` dataclasses; `constants` module with reference thresholds.

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_detection.py`:
```python
from follower_perception.detection import Detection, TrackedBox


def test_detection_fields():
    d = Detection(cx=1.0, cy=2.0, area=100.0, bbox=(0, 0, 10, 10),
                  track_id=7, is_owner=True, confidence=0.9, is_predicted=False)
    assert d.cx == 1.0
    assert d.bbox == (0, 0, 10, 10)
    assert d.track_id == 7
    assert d.is_owner is True
    assert d.is_predicted is False


def test_tracked_box_fields():
    t = TrackedBox(bbox=(0, 0, 10, 20), cx=5.0, cy=10.0, area=200.0,
                   track_id=3, confidence=0.8)
    assert t.area == 200.0
    assert t.track_id == 3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && pip install -e . && python -m pytest tests/test_detection.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.detection'`

- [ ] **Step 3: Write the scaffolding and implementation**

`follower_perception/pyproject.toml`:
```toml
[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[project]
name = "follower_perception"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = [
    "numpy",
    "opencv-python",
    "ultralytics",
]

[tool.setuptools.packages.find]
include = ["follower_perception*"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

`follower_perception/requirements.txt`:
```
numpy
opencv-python
ultralytics
torch
# optional, improves ReID quality; falls back to torchvision/colour if absent:
# torchreid
```

`follower_perception/bytetrack.yaml`:
```yaml
tracker_type: bytetrack
track_high_thresh: 0.5
track_low_thresh: 0.1
new_track_thresh: 0.6
track_buffer: 30      # coasting: frames a lost track_id is kept alive
match_thresh: 0.8
fuse_score: true
```

`follower_perception/follower_perception/__init__.py`:
```python
```

`follower_perception/follower_perception/detection.py`:
```python
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Detection:
    """The only public output of perception. Consumed by the control layer."""
    cx: float
    cy: float
    area: float
    bbox: Tuple[float, float, float, float]
    track_id: int
    is_owner: bool
    confidence: float
    is_predicted: bool


@dataclass
class TrackedBox:
    """Internal per-frame detection with a ByteTrack id. Not public."""
    bbox: Tuple[float, float, float, float]
    cx: float
    cy: float
    area: float
    track_id: int
    confidence: float
```

`follower_perception/follower_perception/constants.py`:
```python
# All values are reference starting points for tuning — not hard requirements.

# Detection
MIN_CONFIDENCE = 0.42          # YOLO person confidence floor

# Owner identification (dual gate)
REID_THRESHOLD = 0.48          # cosine similarity floor
HSV_THRESHOLD = 0.38           # histogram correlation floor ([0,1])
VERIFY_FRAMES = 5              # consecutive passes to lock safe_id

# Online gallery
CALIBRATION_INTERVAL = 30      # frames between gallery-append attempts
CALIBRATION_ADD_THRESHOLD = 0.94  # append only if best gallery sim < this
MAX_GALLERY_SIZE = 50

# Registration
REGISTRATION_STABLE_FRAMES = 3    # consecutive frames the central target must persist
REGISTRATION_MIN_AREA_RATIO = 0.01  # min bbox area / frame area to register

# Smoothing / coasting
SMOOTHER_ALPHA = 0.45
SMOOTHER_BETA = 0.15
FRAME_DT = 0.05                # nominal seconds per frame (20 FPS)
PREDICT_DT = 0.05             # latency-compensation lookahead
COAST_LIMIT = 10              # max consecutive missed frames still output (predicted)

# HSV histogram
HSV_BINS = 16                 # per channel; total 48-d (H+S+V)
```

`follower_perception/tests/__init__.py`:
```python
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_detection.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_perception/pyproject.toml follower_perception/requirements.txt \
        follower_perception/bytetrack.yaml follower_perception/follower_perception/ \
        follower_perception/tests/
git commit -m "feat(perception): package scaffolding + Detection contract + constants"
```

---

### Task 2: HSV color histogram

**Files:**
- Create: `follower_perception/follower_perception/color_hist.py`
- Test: `follower_perception/tests/test_color_hist.py`

**Interfaces:**
- Consumes: `constants.HSV_BINS`
- Produces: `hsv_hist(roi_bgr) -> np.ndarray` (48-d, sums to ~1); `hist_similarity(a, b) -> float` (in [0,1], 1.0 for identical).

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_color_hist.py`:
```python
import numpy as np
from follower_perception.color_hist import hsv_hist, hist_similarity


def _solid(color_bgr, size=(40, 40)):
    img = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    img[:] = color_bgr
    return img


def test_hist_shape_and_normalization():
    h = hsv_hist(_solid((0, 0, 255)))  # red
    assert h.shape == (48,)
    assert abs(float(h.sum()) - 1.0) < 1e-3


def test_identical_similarity_is_one():
    red = _solid((0, 0, 255))
    assert hist_similarity(hsv_hist(red), hsv_hist(red)) > 0.99


def test_different_colors_low_similarity():
    red = hsv_hist(_solid((0, 0, 255)))
    blue = hsv_hist(_solid((255, 0, 0)))
    assert hist_similarity(red, blue) < 0.5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && python -m pytest tests/test_color_hist.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.color_hist'`

- [ ] **Step 3: Write minimal implementation**

`follower_perception/follower_perception/color_hist.py`:
```python
import cv2
import numpy as np

from .constants import HSV_BINS


def hsv_hist(roi_bgr) -> np.ndarray:
    """48-d normalized HSV histogram (HSV_BINS per channel, concatenated)."""
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0], None, [HSV_BINS], [0, 180])
    s = cv2.calcHist([hsv], [1], None, [HSV_BINS], [0, 256])
    v = cv2.calcHist([hsv], [2], None, [HSV_BINS], [0, 256])
    hist = np.concatenate([h, s, v]).flatten().astype(np.float32)
    total = float(hist.sum())
    if total > 0:
        hist /= total
    return hist


def hist_similarity(a, b) -> float:
    """Pearson correlation of two histograms mapped to [0, 1]."""
    a = np.asarray(a, dtype=np.float32)
    b = np.asarray(b, dtype=np.float32)
    corr = cv2.compareHist(a, b, cv2.HISTCMP_CORREL)
    return max(0.0, min(1.0, (corr + 1.0) / 2.0))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_color_hist.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_perception/follower_perception/color_hist.py follower_perception/tests/test_color_hist.py
git commit -m "feat(perception): HSV color histogram + similarity"
```

---

### Task 3: Bbox smoother (α-β filter, coasting)

**Files:**
- Create: `follower_perception/follower_perception/bbox_smoother.py`
- Test: `follower_perception/tests/test_bbox_smoother.py`

**Interfaces:**
- Consumes: `constants.SMOOTHER_ALPHA`, `constants.SMOOTHER_BETA`
- Produces: `BBoxSmoother.update(cx, cy, area, dt)`, `.predict(dt) -> (cx, cy, area) | None`, `.reset()`.

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_bbox_smoother.py`:
```python
from follower_perception.bbox_smoother import BBoxSmoother


def test_first_update_sets_state():
    s = BBoxSmoother()
    s.update(100.0, 50.0, 400.0, dt=0.05)
    cx, cy, area = s.predict(0.0)
    assert abs(cx - 100.0) < 1e-6
    assert abs(area - 400.0) < 1e-6


def test_predict_none_before_any_update():
    assert BBoxSmoother().predict(0.05) is None


def test_extrapolates_constant_velocity():
    s = BBoxSmoother()
    # cx moves +10 per step at dt=1.0; feed several steps to build velocity
    for i in range(10):
        s.update(100.0 + 10.0 * i, 0.0, 400.0, dt=1.0)
    cx_next, _, _ = s.predict(1.0)
    # last measurement was 190; one more step should land near 200
    assert 197.0 < cx_next < 203.0


def test_reset_clears_state():
    s = BBoxSmoother()
    s.update(1.0, 2.0, 3.0, dt=0.05)
    s.reset()
    assert s.predict(0.05) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && python -m pytest tests/test_bbox_smoother.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.bbox_smoother'`

- [ ] **Step 3: Write minimal implementation**

`follower_perception/follower_perception/bbox_smoother.py`:
```python
import numpy as np

from .constants import SMOOTHER_ALPHA, SMOOTHER_BETA


class BBoxSmoother:
    """Alpha-beta filter over [cx, cy, area]; predicts during detection gaps."""

    def __init__(self, alpha=SMOOTHER_ALPHA, beta=SMOOTHER_BETA):
        self.alpha = alpha
        self.beta = beta
        self.state = None            # np.array([cx, cy, area])
        self.velocity = np.zeros(3)

    def update(self, cx, cy, area, dt):
        z = np.array([cx, cy, area], dtype=float)
        if self.state is None or dt <= 0:
            self.state = z
            self.velocity = np.zeros(3)
            return
        pred = self.state + self.velocity * dt
        residual = z - pred
        self.state = pred + self.alpha * residual
        self.velocity = self.velocity + (self.beta / dt) * residual

    def predict(self, dt):
        if self.state is None:
            return None
        p = self.state + self.velocity * dt
        return float(p[0]), float(p[1]), float(p[2])

    def reset(self):
        self.state = None
        self.velocity = np.zeros(3)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_bbox_smoother.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_perception/follower_perception/bbox_smoother.py follower_perception/tests/test_bbox_smoother.py
git commit -m "feat(perception): alpha-beta bbox smoother for coasting"
```

---

### Task 4: ReID engine (OSNet → MobileNet → colour fallback)

**Files:**
- Create: `follower_perception/follower_perception/reid_engine.py`
- Test: `follower_perception/tests/test_reid_engine.py`

**Interfaces:**
- Produces: `ReIDEngine(device=None, backend='auto')`, `.extract(roi_bgr) -> np.ndarray` (L2-normalized), `.similarity(a, b) -> float` (cosine), `.feat_dim`.

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_reid_engine.py`:
```python
import numpy as np
from follower_perception.reid_engine import ReIDEngine


def _solid(color_bgr):
    img = np.zeros((64, 32, 3), dtype=np.uint8)
    img[:] = color_bgr
    return img


def test_colour_backend_normalized_vector():
    eng = ReIDEngine(backend='colour')
    v = eng.extract(_solid((0, 0, 255)))
    assert v.shape == (6,)
    assert abs(float(np.linalg.norm(v)) - 1.0) < 1e-5


def test_self_similarity_is_one():
    eng = ReIDEngine(backend='colour')
    v = eng.extract(_solid((0, 0, 255)))
    assert abs(eng.similarity(v, v) - 1.0) < 1e-5


def test_different_colours_less_similar_than_self():
    eng = ReIDEngine(backend='colour')
    red = eng.extract(_solid((0, 0, 255)))
    blue = eng.extract(_solid((255, 0, 0)))
    assert eng.similarity(red, blue) < eng.similarity(red, red)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && python -m pytest tests/test_reid_engine.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.reid_engine'`

- [ ] **Step 3: Write minimal implementation**

`follower_perception/follower_perception/reid_engine.py`:
```python
import cv2
import numpy as np


class ReIDEngine:
    """Appearance embedding. Tries OSNet, then torchvision MobileNetV3,
    then a 6-float colour-stats fallback. All outputs are L2-normalized."""

    def __init__(self, device=None, backend='auto'):
        self._backend = None
        self._model = None
        self._device = None
        self._tf = None
        self.feat_dim = None
        if backend == 'colour':
            self._init_colour()
        else:
            self._init_auto(device)

    def _init_colour(self):
        self._backend = 'colour'
        self.feat_dim = 6

    def _init_auto(self, device):
        try:
            import torch
            from torchreid.utils import FeatureExtractor
            dev = device or ('cuda' if torch.cuda.is_available() else 'cpu')
            self._model = FeatureExtractor(model_name='osnet_x0_25',
                                           model_path='', device=str(dev))
            self._backend = 'osnet'
            self.feat_dim = 512
            return
        except Exception:
            pass
        try:
            import torch
            import torchvision
            from torchvision import transforms
            dev = device or ('cuda' if torch.cuda.is_available() else 'cpu')
            weights = torchvision.models.MobileNet_V3_Small_Weights.DEFAULT
            model = torchvision.models.mobilenet_v3_small(weights=weights)
            model.classifier = torch.nn.Identity()
            model.eval().to(dev)
            self._model = model
            self._device = dev
            self._tf = transforms.Compose([
                transforms.ToPILImage(),
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406],
                                     [0.229, 0.224, 0.225]),
            ])
            self._backend = 'mobilenet'
            self.feat_dim = 576
            return
        except Exception:
            self._init_colour()

    def extract(self, roi_bgr) -> np.ndarray:
        if self._backend == 'colour':
            vec = self._colour_stats(roi_bgr)
        elif self._backend == 'osnet':
            rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)
            vec = np.asarray(self._model(rgb).cpu().numpy()).flatten()
        else:  # mobilenet
            import torch
            rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)
            tensor = self._tf(rgb).unsqueeze(0).to(self._device)
            with torch.no_grad():
                vec = self._model(tensor).cpu().numpy().flatten()
        return self._normalize(vec)

    @staticmethod
    def _colour_stats(roi_bgr) -> np.ndarray:
        flat = roi_bgr.reshape(-1, 3).astype(np.float32)
        return np.concatenate([flat.mean(axis=0), flat.std(axis=0)]).astype(np.float32)

    @staticmethod
    def _normalize(vec) -> np.ndarray:
        vec = np.asarray(vec, dtype=np.float32)
        norm = float(np.linalg.norm(vec))
        return vec / norm if norm > 0 else vec

    def similarity(self, a, b) -> float:
        a = np.asarray(a, dtype=np.float32)
        b = np.asarray(b, dtype=np.float32)
        return float(np.dot(a, b))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_reid_engine.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_perception/follower_perception/reid_engine.py follower_perception/tests/test_reid_engine.py
git commit -m "feat(perception): ReID engine with OSNet/MobileNet/colour cascade"
```

---

### Task 5: Detector (YOLO11n + built-in ByteTrack)

**Files:**
- Create: `follower_perception/follower_perception/detector.py`
- Test: `follower_perception/tests/test_detector.py`

**Interfaces:**
- Consumes: `detection.TrackedBox`, `constants.MIN_CONFIDENCE`
- Produces: `Detector(weights='yolo11n.pt', conf=MIN_CONFIDENCE, tracker_cfg='bytetrack.yaml', device=None)`, `.detect(frame) -> list[TrackedBox]`; static `Detector._to_tracked_boxes(result) -> list[TrackedBox]`.

**Note:** Parsing is split into a static `_to_tracked_boxes` so it is unit-testable without loading the model. A real end-to-end model run is an optional smoke test.

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_detector.py`:
```python
import numpy as np
from follower_perception.detector import Detector


class _Arr:
    def __init__(self, data):
        self._data = np.asarray(data)

    def cpu(self):
        return self

    def numpy(self):
        return self._data


class _Boxes:
    def __init__(self, xyxy, ids, confs):
        self.xyxy = _Arr(xyxy)
        self.id = _Arr(ids) if ids is not None else None
        self.conf = _Arr(confs)


class _Result:
    def __init__(self, boxes):
        self.boxes = boxes


def test_parse_returns_tracked_boxes():
    result = _Result(_Boxes(
        xyxy=[[10, 20, 30, 60]],   # w=20, h=40 -> area 800, cx 20, cy 40
        ids=[5],
        confs=[0.9],
    ))
    out = Detector._to_tracked_boxes(result)
    assert len(out) == 1
    tb = out[0]
    assert tb.track_id == 5
    assert tb.cx == 20.0
    assert tb.cy == 40.0
    assert tb.area == 800.0
    assert tb.bbox == (10.0, 20.0, 30.0, 60.0)


def test_parse_no_ids_returns_empty():
    result = _Result(_Boxes(xyxy=[[0, 0, 1, 1]], ids=None, confs=[0.5]))
    assert Detector._to_tracked_boxes(result) == []


def test_parse_none_boxes_returns_empty():
    assert Detector._to_tracked_boxes(_Result(None)) == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && python -m pytest tests/test_detector.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.detector'`

- [ ] **Step 3: Write minimal implementation**

`follower_perception/follower_perception/detector.py`:
```python
from .detection import TrackedBox
from .constants import MIN_CONFIDENCE


class Detector:
    """YOLO11n detection + built-in ByteTrack. Person (class 0) only."""

    def __init__(self, weights='yolo11n.pt', conf=MIN_CONFIDENCE,
                 tracker_cfg='bytetrack.yaml', device=None):
        from ultralytics import YOLO
        self.model = YOLO(weights)
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

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_detector.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_perception/follower_perception/detector.py follower_perception/tests/test_detector.py
git commit -m "feat(perception): YOLO11n + ByteTrack detector with testable parsing"
```

---

### Task 6: MockDetector (test double)

**Files:**
- Create: `follower_perception/follower_perception/mocks.py`
- Test: `follower_perception/tests/test_mocks.py`

**Interfaces:**
- Consumes: `detection.TrackedBox`
- Produces: `MockDetector(script: list[list[TrackedBox]])`, `.detect(frame) -> list[TrackedBox]` (returns the next scripted list per call, then `[]`).

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_mocks.py`:
```python
from follower_perception.detection import TrackedBox
from follower_perception.mocks import MockDetector


def _box(tid):
    return TrackedBox(bbox=(0, 0, 10, 10), cx=5.0, cy=5.0, area=100.0,
                      track_id=tid, confidence=0.9)


def test_returns_scripted_lists_in_order():
    det = MockDetector([[_box(1)], [], [_box(2)]])
    assert det.detect(None)[0].track_id == 1
    assert det.detect(None) == []
    assert det.detect(None)[0].track_id == 2


def test_returns_empty_after_script_exhausted():
    det = MockDetector([[_box(1)]])
    det.detect(None)
    assert det.detect(None) == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && python -m pytest tests/test_mocks.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.mocks'`

- [ ] **Step 3: Write minimal implementation**

`follower_perception/follower_perception/mocks.py`:
```python
class MockDetector:
    """Test double for Detector. Yields scripted TrackedBox lists per frame."""

    def __init__(self, script):
        self.script = list(script)
        self.i = 0

    def detect(self, frame):
        out = self.script[self.i] if self.i < len(self.script) else []
        self.i += 1
        return out

    def reset(self):
        self.i = 0
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_mocks.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_perception/follower_perception/mocks.py follower_perception/tests/test_mocks.py
git commit -m "test(perception): MockDetector test double"
```

---

### Task 7: Target matcher (dual-gate + online gallery)

**Files:**
- Create: `follower_perception/follower_perception/target_matcher.py`
- Test: `follower_perception/tests/test_target_matcher.py`

**Interfaces:**
- Consumes: `ReIDEngine`, `color_hist.hsv_hist`, `color_hist.hist_similarity`, `detection.TrackedBox`, constants (`REID_THRESHOLD`, `HSV_THRESHOLD`, `VERIFY_FRAMES`, `CALIBRATION_ADD_THRESHOLD`, `MAX_GALLERY_SIZE`).
- Produces: `TargetMatcher(reid)`, `.register(roi_bgr)`, `.match(cands, frame) -> int | None`, `.calibrate(owner_roi)`, `.is_registered` (property), `.safe_id`, `.reset()`. Cropping helper: `TargetMatcher._crop(frame, bbox) -> roi`.

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_target_matcher.py`:
```python
import numpy as np
from follower_perception.detection import TrackedBox
from follower_perception.reid_engine import ReIDEngine
from follower_perception.target_matcher import TargetMatcher
from follower_perception import constants


def _frame(color_bgr, w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color_bgr
    return img


def _full_box(tid, w=64, h=64):
    return TrackedBox(bbox=(0, 0, w, h), cx=w / 2, cy=h / 2, area=w * h,
                      track_id=tid, confidence=0.9)


def _matcher():
    return TargetMatcher(ReIDEngine(backend='colour'))


def test_not_registered_returns_none():
    m = _matcher()
    assert m.is_registered is False
    assert m.match([_full_box(1)], _frame((0, 0, 255))) is None


def test_matches_registered_colour_owner():
    m = _matcher()
    m.register(_frame((0, 0, 255)))          # red owner
    assert m.is_registered is True
    assert m.match([_full_box(1)], _frame((0, 0, 255))) == 1


def test_rejects_different_colour():
    m = _matcher()
    m.register(_frame((0, 0, 255)))          # red owner
    assert m.match([_full_box(9)], _frame((255, 0, 0))) is None  # blue candidate


def test_locks_safe_id_after_verify_frames():
    m = _matcher()
    m.register(_frame((0, 0, 255)))
    red = _frame((0, 0, 255))
    for _ in range(constants.VERIFY_FRAMES):
        m.match([_full_box(1)], red)
    assert m.safe_id == 1


def test_calibrate_grows_gallery_for_novel_view():
    m = _matcher()
    m.register(_frame((0, 0, 255)))
    start = len(m.gallery)
    m.calibrate(_frame((0, 40, 200)))        # different enough -> append
    assert len(m.gallery) == start + 1


def test_reset_clears_registration():
    m = _matcher()
    m.register(_frame((0, 0, 255)))
    m.reset()
    assert m.is_registered is False
    assert m.safe_id is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && python -m pytest tests/test_target_matcher.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.target_matcher'`

- [ ] **Step 3: Write minimal implementation**

`follower_perception/follower_perception/target_matcher.py`:
```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_target_matcher.py -v`
Expected: PASS (6 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_perception/follower_perception/target_matcher.py follower_perception/tests/test_target_matcher.py
git commit -m "feat(perception): dual-gate target matcher with online gallery"
```

---

### Task 8: Pipeline facade (FollowerPerception)

**Files:**
- Create: `follower_perception/follower_perception/pipeline.py`
- Test: `follower_perception/tests/test_pipeline.py`

**Interfaces:**
- Consumes: `Detector`/`MockDetector` (`.detect`), `ReIDEngine`, `TargetMatcher`, `BBoxSmoother`, `detection.Detection`, constants (`FRAME_DT`, `PREDICT_DT`, `COAST_LIMIT`, `CALIBRATION_INTERVAL`, `REGISTRATION_STABLE_FRAMES`, `REGISTRATION_MIN_AREA_RATIO`).
- Produces: `FollowerPerception(detector=None, reid=None)`, `.register(frame) -> bool`, `.run(frame) -> None`, `.get_latest() -> Detection | None`, `.reset()`.

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_pipeline.py`:
```python
import numpy as np
from follower_perception.detection import TrackedBox
from follower_perception.reid_engine import ReIDEngine
from follower_perception.mocks import MockDetector
from follower_perception.pipeline import FollowerPerception
from follower_perception import constants


def _frame(color_bgr, w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color_bgr
    return img


def _full_box(tid, w=64, h=64):
    return TrackedBox(bbox=(0, 0, w, h), cx=w / 2, cy=h / 2, area=w * h,
                      track_id=tid, confidence=0.9)


def _perception(script):
    return FollowerPerception(detector=MockDetector(script),
                              reid=ReIDEngine(backend='colour'))


def test_register_requires_stable_frames():
    script = [[_full_box(1)]] * constants.REGISTRATION_STABLE_FRAMES
    p = _perception(script)
    red = _frame((0, 0, 255))
    results = [p.register(red) for _ in range(constants.REGISTRATION_STABLE_FRAMES)]
    assert results[-1] is True            # confirmed on the last stable frame
    assert results[0] is False            # not yet stable on frame 1


def test_run_then_get_latest_returns_owner():
    n = constants.REGISTRATION_STABLE_FRAMES
    script = [[_full_box(1)]] * (n + 1)
    p = _perception(script)
    red = _frame((0, 0, 255))
    for _ in range(n):
        p.register(red)
    p.run(red)
    det = p.get_latest()
    assert det is not None
    assert det.is_owner is True
    assert det.is_predicted is False
    assert det.track_id == 1


def test_coasting_then_none():
    n = constants.REGISTRATION_STABLE_FRAMES
    # n stable frames to register, 1 run with owner, then misses (empty lists)
    script = [[_full_box(1)]] * (n + 1) + [[]] * (constants.COAST_LIMIT + 2)
    p = _perception(script)
    red = _frame((0, 0, 255))
    for _ in range(n):
        p.register(red)
    p.run(red)                            # owner seen
    assert p.get_latest().is_predicted is False
    p.run(red)                            # first miss
    assert p.get_latest().is_predicted is True   # coasting
    for _ in range(constants.COAST_LIMIT + 1):
        p.run(red)
    assert p.get_latest() is None         # beyond coast limit


def test_reset_clears_everything():
    n = constants.REGISTRATION_STABLE_FRAMES
    p = _perception([[_full_box(1)]] * (n + 1))
    red = _frame((0, 0, 255))
    for _ in range(n):
        p.register(red)
    p.reset()
    assert p.get_latest() is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && python -m pytest tests/test_pipeline.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.pipeline'`

- [ ] **Step 3: Write minimal implementation**

`follower_perception/follower_perception/pipeline.py`:
```python
from .detection import Detection
from .detector import Detector
from .reid_engine import ReIDEngine
from .target_matcher import TargetMatcher
from .bbox_smoother import BBoxSmoother
from .constants import (
    FRAME_DT, PREDICT_DT, COAST_LIMIT, CALIBRATION_INTERVAL,
    REGISTRATION_STABLE_FRAMES, REGISTRATION_MIN_AREA_RATIO,
)


class FollowerPerception:
    """Facade: frame -> Detection. Detect -> track -> identify -> smooth."""

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

    # ---- registration -------------------------------------------------
    def register(self, frame):
        cands = self.detector.detect(frame)
        target = self._pick_central(cands, frame)
        if target is None:
            self._reg_id = None
            self._reg_streak = 0
            return False
        if target.track_id == self._reg_id:
            self._reg_streak += 1
        else:
            self._reg_id = target.track_id
            self._reg_streak = 1
        if self._reg_streak >= REGISTRATION_STABLE_FRAMES:
            roi = TargetMatcher._crop(frame, target.bbox)
            self.matcher.register(roi)
            self.smoother.reset()
            self._last_owner = None
            self._miss = 0
            self._reg_id = None
            self._reg_streak = 0
            return True
        return False

    def _pick_central(self, cands, frame):
        if not cands:
            return None
        h, w = frame.shape[:2]
        frame_area = float(w * h)
        cx0 = w / 2.0
        viable = [c for c in cands
                  if c.area / frame_area >= REGISTRATION_MIN_AREA_RATIO]
        if not viable:
            return None
        # nearest to center; larger area breaks ties
        return min(viable, key=lambda c: (abs(c.cx - cx0), -c.area))

    # ---- runtime ------------------------------------------------------
    def run(self, frame):
        self._frame_count += 1
        cands = self.detector.detect(frame)
        owner_id = self.matcher.match(cands, frame)
        if owner_id is not None:
            owner = next(c for c in cands if c.track_id == owner_id)
            self.smoother.update(owner.cx, owner.cy, owner.area, FRAME_DT)
            self._last_owner = owner
            self._miss = 0
            if self._frame_count % CALIBRATION_INTERVAL == 0:
                self.matcher.calibrate(TargetMatcher._crop(frame, owner.bbox))
        else:
            self._miss += 1

    def get_latest(self):
        if self._last_owner is None:
            return None
        if self._miss == 0:
            pred = self.smoother.predict(PREDICT_DT)
            is_pred = False
        elif self._miss <= COAST_LIMIT:
            pred = self.smoother.predict(self._miss * FRAME_DT)
            is_pred = True
        else:
            return None
        if pred is None:
            return None
        cx, cy, area = pred
        return Detection(
            cx=cx, cy=cy, area=area, bbox=self._last_owner.bbox,
            track_id=self._last_owner.track_id, is_owner=True,
            confidence=self._last_owner.confidence, is_predicted=is_pred,
        )

    def reset(self):
        self.matcher.reset()
        self.smoother.reset()
        self._last_owner = None
        self._miss = 0
        self._frame_count = 0
        self._reg_id = None
        self._reg_streak = 0
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_pipeline.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add follower_perception/follower_perception/pipeline.py follower_perception/tests/test_pipeline.py
git commit -m "feat(perception): FollowerPerception pipeline facade"
```

---

### Task 9: Server adapter (AiServer, injectable transports)

**Files:**
- Create: `follower_perception/follower_perception/ai_server.py`
- Test: `follower_perception/tests/test_ai_server.py`

**Interfaces:**
- Consumes: `FollowerPerception` (`.register/.run/.get_latest/.reset`), `detection.Detection`.
- Produces: `AiServer(frame_source, result_sink, command_source, make_perception=FollowerPerception)`, `.process_once() -> None`; `detection_to_dict(det) -> dict | None`. Transport contracts: `frame_source.next() -> (source_id, frame) | None`; `command_source.poll() -> list[dict]` where dict is `{"cmd": "register"|"reset", "source": <id>}`; `result_sink.send(source_id, dict_or_none)`.

**Note:** Concrete UDP/TCP transports wrap these small interfaces and are exercised by integration/real tests, not this unit test. The adapter logic (per-source routing, register/reset, send) is what we test here.

- [ ] **Step 1: Write the failing test**

`follower_perception/tests/test_ai_server.py`:
```python
import numpy as np
from follower_perception.detection import TrackedBox
from follower_perception.reid_engine import ReIDEngine
from follower_perception.mocks import MockDetector
from follower_perception.pipeline import FollowerPerception
from follower_perception.ai_server import AiServer, detection_to_dict
from follower_perception import constants


def _frame(color_bgr, w=64, h=64):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = color_bgr
    return img


def _full_box(tid, w=64, h=64):
    return TrackedBox(bbox=(0, 0, w, h), cx=w / 2, cy=h / 2, area=w * h,
                      track_id=tid, confidence=0.9)


class _FrameSource:
    def __init__(self, items):
        self.items = list(items)
        self.i = 0

    def next(self):
        if self.i >= len(self.items):
            return None
        item = self.items[self.i]
        self.i += 1
        return item


class _CommandSource:
    def __init__(self, per_call):
        self.per_call = list(per_call)   # list of list[dict]
        self.i = 0

    def poll(self):
        out = self.per_call[self.i] if self.i < len(self.per_call) else []
        self.i += 1
        return out


class _ResultSink:
    def __init__(self):
        self.sent = []

    def send(self, source_id, payload):
        self.sent.append((source_id, payload))


def test_detection_to_dict_none():
    assert detection_to_dict(None) is None


def test_register_then_track_emits_owner_detection():
    red = _frame((0, 0, 255))
    n = constants.REGISTRATION_STABLE_FRAMES
    # colour ReID perception fed a MockDetector that always sees owner id 1
    script = [[_full_box(1)]] * (n + 5)

    def make_perception():
        return FollowerPerception(detector=MockDetector(script),
                                  reid=ReIDEngine(backend='colour'))

    # n frames with a 'register' command, then a plain tracking frame
    frames = [("drive", red)] * (n + 1)
    commands = [[{"cmd": "register", "source": "drive"}]] * n + [[]]
    sink = _ResultSink()
    server = AiServer(_FrameSource(frames), sink, _CommandSource(commands),
                      make_perception=make_perception)
    for _ in range(n + 1):
        server.process_once()

    src, payload = sink.sent[-1]
    assert src == "drive"
    assert payload is not None
    assert payload["is_owner"] is True
    assert payload["track_id"] == 1


def test_two_sources_are_independent():
    red = _frame((0, 0, 255))

    def make_perception():
        return FollowerPerception(detector=MockDetector([[]] * 10),
                                  reid=ReIDEngine(backend='colour'))

    frames = [("drive", red), ("handy", red)]
    sink = _ResultSink()
    server = AiServer(_FrameSource(frames), sink, _CommandSource([[], []]),
                      make_perception=make_perception)
    server.process_once()
    server.process_once()
    sources = {s for s, _ in sink.sent}
    assert sources == {"drive", "handy"}
    # no registration -> no owner -> None payloads
    assert all(payload is None for _, payload in sink.sent)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd follower_perception && python -m pytest tests/test_ai_server.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'follower_perception.ai_server'`

- [ ] **Step 3: Write minimal implementation**

`follower_perception/follower_perception/ai_server.py`:
```python
from .pipeline import FollowerPerception


def detection_to_dict(det):
    if det is None:
        return None
    return {
        "cx": det.cx, "cy": det.cy, "area": det.area, "bbox": list(det.bbox),
        "track_id": det.track_id, "is_owner": det.is_owner,
        "confidence": det.confidence, "is_predicted": det.is_predicted,
    }


class AiServer:
    """Thin adapter around per-source FollowerPerception instances.

    Transports are injected. Each is a tiny object:
      frame_source.next()   -> (source_id, frame) or None
      command_source.poll() -> list[{"cmd","source"}]
      result_sink.send(source_id, payload_dict_or_none)
    """

    def __init__(self, frame_source, result_sink, command_source,
                 make_perception=FollowerPerception):
        self._frames = frame_source
        self._sink = result_sink
        self._commands = command_source
        self._make = make_perception
        self._perceptions = {}
        self._last_frame = {}

    def _perc(self, source_id):
        if source_id not in self._perceptions:
            self._perceptions[source_id] = self._make()
        return self._perceptions[source_id]

    def process_once(self):
        # 1. Apply pending commands (register/reset) from ABA.
        for cmd in self._commands.poll():
            source = cmd.get("source")
            perc = self._perc(source)
            if cmd.get("cmd") == "register" and source in self._last_frame:
                perc.register(self._last_frame[source])
            elif cmd.get("cmd") == "reset":
                perc.reset()
        # 2. Process one incoming frame.
        item = self._frames.next()
        if item is None:
            return
        source_id, frame = item
        self._last_frame[source_id] = frame
        perc = self._perc(source_id)
        perc.run(frame)
        self._sink.send(source_id, detection_to_dict(perc.get_latest()))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd follower_perception && python -m pytest tests/test_ai_server.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Run the full suite + commit**

Run: `cd follower_perception && python -m pytest -v`
Expected: PASS (all tasks' tests green)

```bash
git add follower_perception/follower_perception/ai_server.py follower_perception/tests/test_ai_server.py
git commit -m "feat(perception): injectable UDP/TCP server adapter"
```

---

## Deferred (not in this plan)

- Concrete UDP `FrameReceiver` and TCP `AbaChannel` wire implementations (JPEG framing, socket handling) — pinned when the robot-side Image Sender protocol is fixed (integration/real testing, Spec 2 boundary).
- Real end-to-end model smoke test (`Detector` loading `yolo11n.pt` on a real person image) — optional, environment-dependent.

## Self-Review

- **Spec coverage:** detector (Task 5), ByteTrack via `bytetrack.yaml` (Task 1) + `model.track` (Task 5), ReID (Task 4), HSV (Task 2), dual-gate matcher + online gallery (Task 7), α-β smoother/coasting (Task 3, 8), registration central+stable (Task 8), Detection contract (Task 1), MockDetector (Task 6), adapter UDP/TCP + 2 sources (Task 9). All Spec 1 sections covered. Concrete socket transports intentionally deferred (documented above).
- **Placeholder scan:** none — every step has runnable code/commands.
- **Type consistency:** `TrackedBox`/`Detection` fields consistent across tasks; `Detector._to_tracked_boxes`, `TargetMatcher._crop`, `ReIDEngine.extract/similarity`, `BBoxSmoother.update(...,dt)/predict(dt)`, `FollowerPerception.register/run/get_latest/reset`, `AiServer.process_once`/`detection_to_dict` all referenced consistently.
