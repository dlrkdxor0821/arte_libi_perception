# 가이드 프로필 등록 & 지속 ReID 설계 (Spec 3)

- **날짜:** 2026-07-10
- **상태:** 설계 확정 대기 (사용자 리뷰 전)
- **범위:** `follower_perception` 만. 사진 1장 등록 → 프로필 디스크 저장 → 영상에서 지속 ReID 검증.
- **참조:** [Spec 1 — perception](2026-07-08-follower-perception-design.md) (`TargetMatcher`, `FollowerPerception`, `ReIDEngine`)
- **비고:** 커밋은 사용자가 직접 수행(에이전트는 커밋하지 않음).

---

## 1. 컨텍스트 & 목표

라이브러리 **가이드 로봇** 시나리오. 방문자를 **한 번 촬영(사진 1장)** 하여 등록하고, 이후 **영상**으로 그 사람을 계속 ReID 하며 추적한다. (사서/직원은 이미 DB에 등록되어 있으나, 결국 "저장된 프로필 ↔ 실시간 ReID 매칭" 구조로 동일하므로 하나의 메커니즘으로 다룬다.)

> **목표:** `사진 1장 → 사람 프로필(ReID+HSV+crop) 디스크 저장`, 그리고 `저장된 프로필 → 영상 프레임에서 동일 인물이 계속 ReID 되는지` 를 실제 파이프라인 코드 경로로 검증한다.

### 현재 격차 (Spec 1 대비)
`TargetMatcher.register(roi)` + `match()` 는 ReID+HSV 이중게이트로 등록/식별을 이미 수행한다. 그러나:
1. **디스크 영속성 없음** — 템플릿이 메모리에만 존재. 프로세스 종료 시 소멸.
2. **사진 1장 등록 경로 없음** — `FollowerPerception.register()` 는 `REGISTRATION_STABLE_FRAMES`(3) 연속 프레임을 요구(영상 등록용). 단일 이미지 경로가 없다.

이 두 격차를 메우는 것이 본 스펙의 전부다. (매칭/스무딩/coasting 로직은 그대로 재사용.)

---

## 2. 아키텍처

```
[등록] photo.jpg ─► Detector.detect ─► 중앙/최대 사람 선택 ─► crop
                     └─► ReIDEngine.extract + hsv_hist ─► Profile ─► 💾 profiles/<name>/
[추적] video.mp4 ─► (프레임 루프) Detector.detect+track ─► TargetMatcher.match(저장 프로필)
                     └─► BBoxSmoother ─► Detection(is_owner, reid_sim, ...) ─► 콘솔/annotated mp4
```

- **순수 파이썬 유지** — ROS/네트워크 의존성 추가 없음 (Spec 1 제약 유지).
- **실제 경로 검증** — 검증 스크립트는 `Detector` → `TargetMatcher` → `BBoxSmoother` 를 그대로 사용. 별도 매칭 로직 복제 안 함.
- **검출은 로컬 ultralytics `.pt` 직행** — `Detector.detect(frame)`(detector.py:16)이 이미 `이미지→검출` 추상화라 별도 어댑터 불필요. NCNN·서버추론(TCP)은 이 repo에 없음(그건 `pinky_perception`/`shoppinkki` 얘기). 네트워크 의존성 0.

---

## 3. 결정 사항

| 항목 | 결정 |
|---|---|
| 접근법 | 기존 `TargetMatcher`/`FollowerPerception` 에 최소 추가 (save/load + 사진1장 등록). 별도 데모용 로직 복제 안 함. |
| **정본 모델** | 방금 학습한 **person 검출 `best.pt`** (web-person). `~/personal_repo/labeling_sam3/.../best.pt` → **`follower_perception/weights/best.pt`** 로 복사. `Detector` 기본 가중치를 이 경로로. |
| 배제된 모델 | shoppinkki `doll-recog-model.pt`(인형 검출·타 repo), pinky NCNN(성능비교·타 repo) — 본 스펙 미사용. |
| 모델 git 추적 | 에이전트는 커밋하지 않음. git 추적 여부는 사용자 결정. (`.gitignore` 추가는 사용자 판단.) |
| **HSV 게이트(track)** | track 시 **ReID 우선, HSV 완화 가능**. `--hsv-threshold <v>`(기본 constants) / `--no-hsv`(HSV 게이트 무시). 단일사진 HSV의 조명 취약성 대응(우려 1). register/기본 동작은 이중게이트 유지. |
| **gallery 영속화** | track은 **read-only** — `calibrate()`로 세션 중 갤러리가 커져도 **디스크 재저장 안 함**(우려 3). |
| 프로필 단위 | 사람 1명 = 폴더 1개 (DB 레코드 유사, 다인 확장 여지). |
| 프로필 포맷 | `crop.jpg` + `features.npz`(reid, hsv, gallery) + `meta.json`(name, 등록시각, model, bbox, backend, feat_dim). |
| 등록 대상 선택 | 사진 내 사람 후보 중 **화면 중앙 최근접, 동률 시 최대 면적** (Spec 1 `_pick_central` 규칙 재사용). |
| **백엔드 이식성** | `crop.jpg` = **진실원본**. load 시 프로필 backend/feat_dim ≠ 현재 엔진이면 **에러 대신 crop에서 ReID 재추출**(경고 로그). HSV는 백엔드 무관이라 항상 재사용. `--strict` 지정 시에만 `ValueError`. → GPU(OSNet 512d) 저장분을 CPU(colour 6d) 머신에서도 로드 가능. |
| **경로 이식성** | `Detector` 기본 가중치는 `__file__` 기준 **패키지 상대경로**로 해석(`weights/best.pt`) → cwd 무관. `--weights`/env `FOLLOWER_WEIGHTS` 로 override. meta.json엔 절대경로 대신 파일명만 기록. |
| **device 이식성** | ReID/YOLO device는 **auto(cuda 있으면 cuda, 없으면 cpu)** + `--device` override. torch/ultralytics 미설치 환경은 명확한 안내 후 종료(단위 테스트는 colour로 무관). |

---

## 4. 컴포넌트 설계

### 4.1 Profile 직렬화 (`profile.py`, 신규)
순수 함수/데이터. `TargetMatcher` 내부구조를 파일로 옮기는 얇은 계층.

- `save_profile(dir, *, crop_bgr, reid_vec, hsv_vec, gallery, meta) -> None`
  - `dir/crop.jpg` (cv2.imwrite), `dir/features.npz` (np.savez: reid, hsv, gallery 스택), `dir/meta.json`.
- `load_profile(dir) -> {crop, reid, hsv, gallery, meta}`
  - 파일 부재/손상 시 명확한 예외.

### 4.2 `TargetMatcher` 확장 (기존 파일)
- `save(dir, *, crop_bgr, name)` — 현재 `template_reid/template_hsv/gallery` + 메타를 `save_profile` 로 기록.
- `load(dir, *, strict=False)` — `load_profile` 로 상태 복원.
  - 프로필 `backend`/`feat_dim` == 현재 `reid` 엔진이면 저장된 임베딩 그대로 사용(빠른 경로).
  - **불일치 시(이식성): `crop.jpg` 에서 `reid.extract` 로 재추출**하여 `template_reid`/`gallery` 재구성 + 경고 로그. HSV(`template_hsv`)는 백엔드 무관이라 저장분 그대로. crop 없거나 손상이면 그때 `ValueError`.
  - `strict=True` 이면 재추출 대신 불일치 즉시 `ValueError`.
- 불변식: load 후 `is_registered == True`, `safe_id is None`(track_id 는 새 영상마다 다름 → 재확립).
- **HSV 완화(우려 1)**: `match()` 가 인스턴스 `hsv_threshold`(생성자 인자, 기본 `constants.HSV_THRESHOLD`)를 사용하도록. `None` 이면 HSV 게이트 무시하고 ReID 단독 판정. 하드코딩 상수 참조 → 인스턴스 필드로 승격(기본값 동일하므로 기존 동작·테스트 불변).

### 4.3 사진 1장 등록 경로 (`FollowerPerception` 메서드)
- `FollowerPerception.register_from_image(image_bgr) -> TrackedBox | None`
  - `detector.detect(image)` → `_pick_central` → crop → `matcher.register(roi)`. 3프레임 안정 요구 우회.
  - 사람 미검출 시 `None` (스크립트가 사용자에게 "사람 없음" 보고).

### 4.4 검증 CLI (`follower_perception/scripts/register_and_track.py`, 신규)
`follower_perception/` 패키지 루트에서 실행(테스트와 동일한 import 경로 규약).
```bash
cd follower_perception
# 등록: 사진 1장 → 프로필 저장
python scripts/register_and_track.py register --image visitor.jpg --profile profiles/visitor1 [--name 홍길동]

# 추적: 영상에서 지속 ReID 검증
python scripts/register_and_track.py track --video walk.mp4 --profile profiles/visitor1 \
       [--out annotated.mp4] [--hsv-threshold 0.2 | --no-hsv]
```
- `--hsv-threshold <v>`: track의 HSV 게이트를 낮춤(기본은 프로필/constants). `--no-hsv`: HSV 무시, ReID 단독(우려 1 — 조명 변화 대응). 두 플래그는 `TargetMatcher(hsv_threshold=...)`로 전달.
- **register**: 이미지 로드 → `register_from_image` → 성공 시 `matcher.save(...)` → crop/meta 경로 출력. 실패 시 비정상 종료코드 + 사유.
- **track**: 프로필 load → 비디오 프레임 루프 → `FollowerPerception.run` → `get_latest`. 프레임별 로그(`frame i: owner track_id=3 reid_sim=0.71 hsv_sim=0.58 predicted=False` / `owner not found`). 종료 시 **요약**: 전체 프레임 중 주인 유지 비율, 예측(coasting) 프레임 수, 최대 연속 놓침. `--out` 지정 시 bbox 그린 mp4 저장.

---

## 5. 데이터 계약

`meta.json`:
```json
{
  "name": "visitor1",
  "registered_at": "2026-07-10T12:00:00",
  "model_weights": "best.pt",
  "reid_backend": "mobilenet",
  "feat_dim": 576,
  "bbox": [x1, y1, x2, y2],
  "source_image": "visitor.jpg"
}
```
- **절대경로 금지**(이식성): `model_weights`/`source_image`는 파일명만. 프로필 폴더는 자기완결이라 다른 머신에 복사만 하면 됨.
- `features.npz`: `reid`(feat_dim,), `hsv`(48,), `gallery`(N, feat_dim). `reid`/`gallery`는 backend 종속(불일치 시 crop 재추출), `hsv`는 backend 무관.

---

## 6. 에러 처리

| 상황 | 처리 |
|---|---|
| 등록 사진에 사람 없음 | `register_from_image` → None, 스크립트가 비정상 종료 + 안내. |
| 커스텀 모델 클래스가 person=0 아님 | 등록 시 `model.names` 확인, class 0 이 사람이 아니면 경고/에러(검출 0 방지). |
| load 시 백엔드/차원 불일치 | 기본: `crop.jpg`에서 재추출 + 경고("mobilenet(576)로 저장됨 → colour(6)로 재추출"). `--strict`면 `ValueError`. crop 없으면 `ValueError`. |
| 프로필 파일 부재/손상 | `load_profile` 명확한 예외, 스크립트가 경로 안내. |
| torch/ultralytics 미설치 | `Detector`/실백엔드 생성 시 `ImportError` 를 잡아 "이 명령은 ultralytics 설치된 환경에서 실행하세요(예: <venv>)" 안내 후 종료. |
| 영상 열기 실패 | cv2.VideoCapture 실패 감지 → 종료 + 경로 안내. |

---

## 7. 환경 이식성 (Portability) — 1급 원칙

**목표: 어느 환경(GPU/CPU, 다른 머신, 다른 venv, 다른 cwd, 다른 repo로 이식)에서도 그대로 동작.**

1. **crop.jpg = 진실원본** — 프로필은 백엔드 종속 임베딩만이 아니라 **원본 crop 이미지**를 항상 보관. 다른 백엔드/머신에서 로드하면 crop에서 재추출 → 임베딩이 이식성을 잃지 않음. (HSV는 백엔드 무관.)
2. **경로는 코드 기준 상대해석** — 모델·기본 자원은 `__file__` 기준으로 찾고(cwd 무관), 저장물엔 절대경로를 남기지 않음. override는 인자/env.
3. **의존성은 지연 임포트 + 우아한 폴백** — `ultralytics`/`torch`는 `Detector` 안에서만 지연 임포트(순수코어·단위테스트는 미설치여도 동작). ReID는 OSNet→MobileNet→colour 자동 폴백(Spec 1 유지). 미설치 시 죽지 않고 안내.
4. **device 자동 + override** — cuda 있으면 cuda, 없으면 cpu. `--device`로 강제 가능.
5. **자기완결 프로필 폴더** — 프로필은 폴더 하나로 복사만 하면 다른 프로젝트/머신에서 그대로 사용(외부 참조 없음). ROS/네트워크 의존 없음(순수 파이썬).
6. **결정성 보존** — dt 명시 주입(벽시계 미사용) 규약 유지 → 어느 머신에서도 테스트 재현.

> 이식성 검증: "colour(6d)로 저장한 프로필을 그대로 로드해도, 그리고 (실백엔드 환경에서) 다른 backend로 로드해도 crop 재추출로 매칭이 살아있다"를 단위 테스트로 못박음(§8).

---

## 8. 테스트 전략

기존 방식대로 pytest, 하드웨어/실모델 없이 도는 단위 테스트 우선(ReID `backend='colour'`).

- `test_profile.py`: `save_profile`→`load_profile` 왕복(round-trip)으로 배열/메타 보존.
- `test_target_matcher.py`(확장): `register`→`save`→새 matcher `load`→`match` 가 같은 색 후보를 매칭.
- **이식성**: 프로필 meta의 backend/feat_dim을 다르게 위조 → `load`가 crop에서 재추출해 여전히 매칭(경고). crop 삭제 시 `ValueError`. `strict=True` 시 재추출 없이 `ValueError`.
- `hsv_threshold=None` / 낮은 값 → HSV가 깨져도(다른 색) ReID로 매칭 유지되는 경로.
- `register_from_image`: `MockDetector` 로 사람 있음/없음 두 경로.
- 경로 이식성: cwd를 바꿔도 `Detector` 기본 가중치가 패키지 상대경로로 해석되는지(존재하면).
- **실모델 스모크(옵션, 환경 의존)**: 실제 `best.pt` + 샘플 사진/영상 → register→track. CI 아님, 로컬 검증용.

---

## 9. YAGNI / 범위 밖

- 다인(多人) DB / `ProfileStore` — 지금은 폴더 규약만. 여러 사람 관리 로직은 나중.
- **track 중 gallery 디스크 재저장 — 안 함(우려 3).** 세션 메모리 한정. 온라인 갱신 영속화는 나중.
- UDP/TCP wire, ROS 연동 — Spec 1/2 경계 유지, 본 스펙 무관. (검출은 로컬 `.pt` 직행이므로 NCNN/서버추론 어댑터 불필요.)
- 실시간 웹캠 GUI — headless 세션이므로 콘솔+annotated mp4로 대체.
- 재학습/라벨 export — 이번 목적 아님.

---

## 10. 완료 판정

1. `best.pt` 가 repo 내부 경로로 복사되고 `Detector` 가 **cwd 무관**하게 그 경로로 동작.
2. `register --image` → `profiles/<name>/{crop.jpg,features.npz,meta.json}` 생성.
3. `track --video` → 프레임별 ReID 로그 + "주인 유지 비율" 요약 출력.
4. 단위 테스트(profile round-trip, save/load match, **cross-backend 재추출**, HSV 완화, register_from_image) 통과.
5. **이식성**: colour로 저장한 프로필을 backend 위조 후 로드해도 crop 재추출로 매칭 유지.
6. (옵션) 실 `best.pt`+샘플로 register→track 스모크가 "같은 사람 계속 매칭"을 보임.
