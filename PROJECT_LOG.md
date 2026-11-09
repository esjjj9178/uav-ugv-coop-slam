# UAV–UGV 협동 SLAM 시뮬레이션: 전체 진행 기록

목표: 드론(UAV)과 차량(UGV)이 함께 출발해 각자 RealSense D435로 RGB-D SLAM을 하고, 지붕(캐노피) 위·아래처럼
서로 못 보는 영역을 나눠 맵핑한 뒤 두 맵을 하나로 정합한다.

환경: Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic(gz sim 8.14), `ros-humble-ros-gzharmonic`, RTAB-Map(Humble 패키지).
워크스페이스: `~/ros2_ws`. 패키지 3개:

| 패키지 | 내용 |
|---|---|
| `couv_sim_world` | 월드 `canopy_world` (텍스처 생성, 배치 정의 하나로 SDF·배치도·README 표 생성) |
| `couv_sim_robots` | 차량·드론 모델, D435 매크로, 브리지, 미션, 녹화 |
| `couv_map_merge` | 로봇별 RTAB-Map, ArUco 검출, 맵 정합·평가, RViz 재생 |

---

## 1단계 – 월드 (`couv_sim_world`)

- 20 × 20 m 바닥(5 m 타일 16장, 타일마다 다른 텍스처), 높이 3 m 외벽.
- 캐노피: 8 × 6 m 평지붕(아랫면 z 3.0, 윗면 3.2), 기둥 6개, 옆면 개방.
- 지붕 아래 장애물 2개(상자 더미, 드럼통; 차량만 봄), 지붕 위 2개(실외기, 물탱크; 드론만 봄),
  공용 랜드마크 4개(상자·원기둥·L자·계단; 둘 다 봄 → 정합 겹침 영역), 출발/도착 판.
- 시각 오도메트리용으로 모든 면에 반복 없는 텍스처(38장, numpy+Pillow 생성).
- 좌표·크기는 `scripts/layout.py` 하나에서 SDF, `docs/layout.png`, README 좌표표를 함께 생성.
- 검증: `gz sdf -k` 통과(월드는 `SDF_PATH` 필요), colcon build, headless 실행 오류 0,
  GUI·카메라 렌더에서 텍스처 정상(스크린샷 `couv_sim_world/docs/`).

## 2단계 – 로봇·센서·미션·녹화 (`couv_sim_robots`)

- Leo Rover 공식 시뮬레이터는 Humble+Harmonic 미지원(humble 브랜치는 Fortress/Garden) → 같은 크기의
  4륜 diff-drive 차량을 직접 제작. 윗면에 0.5 × 0.5 m 착륙판.
- 드론: S500급(대각 0.5 m, 1.5 kg) X-쿼드, Gazebo 내장 MulticopterMotorModel + VelocityControl
  (`/drone/cmd_vel`). 비행 제어 부분은 별도 xacro로 분리(나중에 PX4로 교체).
- D435 공용 매크로: 640×480 30 Hz, 화각 69.4°, 깊이 0.2–10 m, 노이즈 σ 1 cm, 내부 IMU 200 Hz, optical frame.
  드론은 하방(기본 90°, `drone_cam_pitch_deg`), IMU 200 Hz, 기압계, 하방 1빔 거리센서.
- 모든 수치는 `config/frames.yaml` 한 곳. **T0**(착륙판 위 드론 위치, 차량 base_link 기준
  x 0, y 0, z 0.3425 m, yaw 0)도 여기서 정의 → 스폰과 정합 초기값에 사용.
- 정답 포즈는 토픽(`/<robot>/ground_truth/odom`)으로만, RViz용은 별도 `gt/` TF 트리.
- 검증 결과: 깊이 오차 중앙값 0.8 mm(기둥 6.630 m 기대 / 6.633 m 측정), 카메라 30.3 Hz, IMU 200 Hz,
  미션(일자 경로) 56 s, 두 번 반복해 동일, 드론 기울기 ≤ 3.1°, 착륙판 위 미끄러짐 0 mm.
- 녹화는 압축 이미지 기본(원본은 시뮬 1초에 130 MB). 디스크 부족으로 이후 녹화는 모두 삭제.

## 3단계 – 첫 정합 시도: 일자 경로, 드론 90° (`couv_map_merge`)

1. 녹화 bag으로 로봇별 RTAB-Map(시각 오도메트리 + SLAM) → 차량 맵, 드론 맵.
   - 차량 SLAM 오차(ATE) 1.9 cm, 드론 13.6 cm(끝에서 y −27, z +22 cm 누적).
2. 정합(드론 맵 → 차량 맵):
   - 그대로 겹치면 59.9 cm 어긋남.
   - **T0**(“착륙판 위에서 같이 출발”)만 적용 → 26.1 cm.
   - **+ 거리센서 상승량**(드론 VO는 착륙판이 근거리 한계보다 가까워 조금 뜬 뒤 시작 → 그만큼 보정) → **6.6 cm, 0.34°**.
   - **ICP**(가까운 점끼리 짝지어 맞추기): 오히려 13.6 cm, 3.2°로 악화.
     원인: ① 겹치는 부분 대부분이 평평한 바닥이라 x·y·회전을 못 잡음,
     ② 드론 맵 끝이 휘어 있어 끝을 맞추느라 원점이 틀어짐.
     → 이동량이 T0 불확실성(10 cm, 2°)을 넘으면 기각하는 규칙으로 버림(정답 미사용).
   - 정답 포즈는 오차 채점에만 사용.
3. RViz 재생: 차량 맵 | 정합 맵(주황/파랑, 단계별 애니메이션) | 드론 맵.

## 4단계 – ArUco + ㄹ자 경로 + 착륙 (방식 1–4 비교 완료 2026-10-28, 다음: 공동 최적화)

### 목표
- 출발: 정지한 차량 위에서 드론이 ArUco(착륙판, ID 7, 0.30 m)를 촬영 → 실제로 측정한 T0.
- ㄹ자 경로: 왼쪽 아래 (-8,-6.5) → 오른쪽 위 (8,6.5). 줄 3개(y = -6.5, 0(캐노피 아래/위), +6.5).
- 차량이 먼저 도착 → 드론이 내려와 ArUco 촬영 → 착륙.
- 드론 카메라 45°. 녹화 없음(실시간 SLAM). 결과는 이미지 파일로만 저장.
- 비교: A T0만, B ArUco 출발, C ArUco 출발+도착(누적오차를 경로 길이 비율로 분배), D C+ICP,
  그리고 3단계의 “T0 + 거리센서, ICP 없음”(일자 경로).

### 완료된 것
- 착륙판 텍스처에 ArUco 넣음(`couv_sim_robots/couv_sim_robots/gen_pad_mesh.py`, 설정 `frames.yaml` `ugv.aruco`),
  텍스처 비율 문제(마커 6% 찌그러짐) 수정.
- `sim.launch.py`에 `ugv_spawn_x/y/yaw` 인자 추가.
- 미션: `couv_sim_robots/config/mission_serpentine.yaml`, `couv_sim_robots/mission_aruco.py`
  (상태: takeoff → aruco_start → climb → cruise → wait_ugv → approach → aruco_end → land → complete).
  위치 제어에 적분항 추가(속도 제어기의 정상상태 오차 13 cm 제거).
- `couv_map_merge`:
  - `aruco_detector.py`: 드론 영상에서 마커 검출 → `/drone/aruco/marker_pose`, 촬영 장면 이미지 저장.
    테스트: 출발 호버링(마커까지 1.67 m)에서 4초간 29회 검출 성공.
  - `launch/slam_live.launch.py`: 시뮬과 함께 로봇별 RTAB-Map 실시간 실행 + 작은 로그 bag
    (정답 포즈, 거리센서, 미션 상태, ArUco, VO 오도메트리. 영상 없음).
    시각 오도메트리는 `/<robot>/vo_odom` (`/ugv/odom`은 휠 오도메트리와 충돌하므로).
  - `run_live.sh`: 시뮬 + SLAM + 미션 실행 → 끝나면 지도/포즈/노드별 이미지 내보내기.
  - `watch_live.sh` + `rviz/live.rviz`: 실행 중에 Gazebo GUI와 RViz(두 지도 실시간)를 붙여 보기.
  - `compare_aruco.py`: 방법 A~D와 이전 결과 비교, `results.yaml`과 그림 01~05 생성. **아직 실행 전(미검증).**
- 녹화 bag 전부 삭제. 3단계 결과 재현용 정답/거리센서만 `maps/couv_20261027_200102_pitch90/aux_bag`(5 MB)에 보관.

### 2026-10-28 세션 2: 수정한 것
- **DB 용량**: `config/slam.yaml`에서 `Rtabmap/DetectionRate 1`(메시지 시각=시뮬 시간 기준), `Mem/NotLinkedNodesKept false`,
  `Mem/SaveDepth16Format true`. → 전체 실행 DB 합계 약 157 MB(이전 추정 수 GB), 결과 폴더 365 MB.
- **차량이 heading ±90° 근처에서 제자리 회전을 못 함** (첫 본 실행이 (8,0) 근처를 맴돌다 실패한 원인):
  바퀴 마찰 `mu1 1.0 / mu2 0.6`(비등방)인데 dartsim이 `fdir1`을 무시 → 마찰 방향이 월드에 고정되어 heading 0°에선
  옆 마찰 0.6(회전 가능), 90°에선 1.0(회전 0). `fdir1`을 넣어도 동일. → `ugv.urdf.xacro` 바퀴 마찰을 **등방 0.6**으로.
  시험: 모든 heading에서 명령 0.35 rad/s → 실제 0.23 rad/s, 직진 0.4 m/s 정상.
- **`run_live.sh` 종료 처리**: launch도 무시된 SIGINT를 물려받아 `kill -INT`로 안 꺼지고 스크립트가 멈춤 →
  `set -m`(작업 제어) + launch에 SIGTERM, sim launch 종료 대기, 남은 bridge/robot_state_publisher 정리,
  `aux_bag/metadata.yaml` 없으면 `ros2 bag reindex`.
- `compare_aruco.py`: 노드별 calib yaml(ROS camera_info 형식)을 yaml로 읽도록 수정, ICP 기각 시 그림 02에서 D를 점선으로.

### 4단계 결과 (`maps/serpentine_aruco`, 그림·수치 `src/couv_map_merge/docs/serpentine/`)
실행: 차량 경로 전체 완주(t=221 s 도착), 드론 출발/도착 ArUco 촬영 완료, **착륙 시작(t=241 s) 직후 노트북 절전**
(17:30:59, NetworkManager sleep) → 깨어난 뒤 DDS/gz 통신이 복구되지 않아 착륙은 미완료. 지도·로그는 그 시점까지 온전함
(내보낸 노드 drone 160 / ugv 161, 실행 중 WM 161/162). **긴 실행 전에는 절전 끄기**.

ArUco: 출발 24회, 도착 23회 검출(거리 1.67 / 1.59 m). 차량→드론 상대 포즈 오차(중앙값) **3.9 / 3.3 cm**
(예상 1–2 cm보다 큼, 카메라 y·z 방향 산포 약 5 cm: 45° 시선에서 PnP 깊이/기울기 불확실성).

SLAM 자체 오차(각 지도 원점 기준, 정답 비교):

| | RMSE | 최대 | 비고 |
|---|---|---|---|
| 차량 지도 | 27.0 cm | 62.0 cm | 3단계 일자 경로 1.9 cm. 첫 90° 제자리 회전 전후로 11 → 22 cm, 끝 59 cm (z도 +32 cm) |
| 드론 지도 | 9.3 cm | 22.2 cm | 3단계 13.6 cm(최대 35) |

정합 오차(드론 노드 위치를 차량 지도에 놓았을 때 정답 대비, `results.yaml`):

| 방법 | 중앙값 | 최대 | 시작 | 끝 | 각도 중앙값 |
|---|---|---|---|---|---|
| 3단계: 일자, T0 + 거리센서, ICP 없음 | 12.9 cm | 38.2 | 6.6 | 38.2 | 1.46° |
| A: T0만 | 7.8 cm | 22.7 | 0.5 | 19.6 | 0.58° |
| B: ArUco 출발 | **7.6 cm** | **20.1** | 3.2 | 16.9 | 0.52° |
| C: ArUco 출발+도착(누적오차 경로길이 분배) | 20.4 cm | 61.8 | 3.2 | 61.6 | 0.71° |
| D: C + ICP | = C (ICP 보정 15.7 cm > 10 cm → 기각) | | | | |

해석:
- A ≈ B: 시뮬에서는 T0가 정확(스폰 값 그대로)해서 시작 0.5 cm. ArUco 1회 측정(3 cm)이 이를 이기지 못함.
  실제 로봇(착륙 위치가 매번 다름)에서는 B가 의미 있음.
- **C가 나빠진 이유는 구현 오류가 아니라 가정 문제**: C는 "차량 지도가 기준(정확), 드론 지도가 흐른다"고 보고
  도착 시점에 드론 지도를 차량 지도에 맞춤. 이번엔 반대로 차량 지도(끝 59 cm)가 드론 지도(끝 20 cm)보다 훨씬 많이 흘러
  드론 지도가 차량 지도의 오차를 그대로 따라감(C 끝 오차 61.6 cm ≈ 차량 지도 끝 오차 59 cm).
  도착 ArUco는 두 지도의 **상대** 누적오차만 알려 주고 어느 쪽이 흘렀는지는 모름.
- 그림 04(A vs C)에서 두 지도는 육안상 둘 다 잘 겹침 → 겹침 영역의 국소 일관성은 이 "정답 대비 전역 오차"와 다른 문제.
- 그림: `01_paths.png`, `02_error_along_flight.png`, `03_error_summary.png`, `04_merged_maps_A_vs_C.png`,
  `05_aruco_start.png`, `05_aruco_end.png`.

- RViz 재생: `ros2 launch couv_map_merge aruco_viz.launch.py run:=~/ros2_ws/maps/serpentine_aruco` (`aruco_viz.py`, `rviz/aruco.rviz`). 가운데에 A→B→C→D→정답 배치를 4초씩 반복, 주황/파랑 = SLAM 경로, 흰색/회색 가는 선 = 드론/차량 정답 경로(차량 지도 원점 기준). `compare_aruco.py`의 계산은 `solve()`로 분리해 공유.

### 2026-10-28 세션 2 후반: 번호별 방식, 구간 분석, 방식 4 (RGB-D 특징점)
- 방식 번호: 1 T0만, 2 T0 + ICP, 3 ArUco 출발+도착 + ICP, 4 T0 + ArUco 도착 + RGB-D 특징점(ICP 없음).
  `compare_aruco.py` → `results.yaml`의 `numbered_methods`, `segments_serpentine`, `overlap_consistency`.
- 새 지표 **국소 오차**(`local_node_errors`): 근처 차량 지도 기준(차량 지도 자체 드리프트와 분리).
  방식 1–4 국소 중앙값 20.9 / 9.6 / 11.6 / **7.7** cm, 전역 중앙값 **7.8** / 28.2 / 16.9 / 20.6 cm.
- 방식 4 (`feature_anchor.py`): 364쌍 → 62 채택 → 기준점 10, 매칭 정확도 중앙값 2.2 cm. 캐노피 구간(21.8–52.3 m) 공백.
- RViz `aruco_viz.launch.py` 기본이 4패널 비교 + 빨강(겹침 어긋남 > 10 cm), `mode:=cycle`은 이전 순환.
- 주의: `pkill -f`/`pgrep -f` 패턴이 같은 셸 명령줄의 다른 문자열(launch 명령 등)에도 걸려 셸이 죽음 →
  종료와 실행을 별도 명령으로.

### 지표 정의와 "정답" (세션 2에서 정리한 개념)
- **정답** = Gazebo가 아는 실제 세계. ① 로봇 실제 위치 `/<robot>/ground_truth/odom`(50 Hz, `aux_bag`),
  ② 실제 물체 모양·위치 `couv_sim_world/scripts/layout.py`(SDF). 정답은 **채점에만** 사용.
- **전역 오차** (`node_errors`): 드론 노드 i의 추정 `T_i·P_i` vs 정답 `W_u⁻¹·GT_d(t_i)`,
  `W_u = GT_u(t₀)·P_u,₀⁻¹`(차량 지도 첫 노드가 정확하다고 가정). 차량 지도 자체 누적오차도 포함됨.
  지도 점이 노드에 붙어 있으므로 노드 오차로 지도 오차를 대신한 것(지도 점↔실제 표면 직접 비교는 아직 안 함).
- **오차율** = 끝 오차 ÷ 드론 정답 경로 길이(3단계 19.5 m, 4단계 73.5 m). 중앙값 오차율은 전체 길이로 나눈 값
  (노드별 이동거리 기준 드리프트율이 아님).
- **국소 오차** (`local_node_errors`): 공간상 가장 가까운 차량 노드 j 기준 `P_u,j·GT_u(t_j)⁻¹·GT_d(t_i)`와 비교.
  "두 지도가 서로 잘 겹치나". 차량 지도 자체 드리프트와 분리됨.
- **겹침 어긋남(빨강)** (`overlap_error`): 정답 없이 지도끼리. 0.5 m 이내 겹침에서 상대 표면까지 거리 > 10 cm 비율
  (평면은 점-평면). ICP가 최소화하는 양이라 ICP에 유리, 평면 위 미끄러짐은 못 잡음.
- 목표에 따라 다름: 실제 좌표(측량, GPS 결합) → 전역 / 합친 지도로 주행·장애물 판단 → 국소. **둘 다 → 공동 최적화.**

### 현재 결론 (4단계, 방식 1–4)
| 방식 | 전역 중앙값/끝 | 국소 중앙값/최대/끝 | 빨강 |
|---|---|---|---|
| 1 T0만 | **7.8** / 19.6 cm | 20.9 / 62.8 / 62.8 | 42 % |
| 2 T0 + ICP(강제) | 28.2 / 44.7 | 9.6 / 34.5 / 15.4 | **16 %** |
| 3 ArUco 출발+도착 + ICP(강제) | 16.9 / 59.8 | 11.6 / 25.0 / **3.3** | 19 % |
| 4 T0 + ArUco + RGB-D 특징점 | 20.6 / 61.6 | **7.7 / 18.4** / 3.9 | 22 % |
- ICP가 나쁜 이유: 휜 지도를 강체 하나로 맞춤, 공통 관측의 95 %가 바닥·벽(평면 → x·y·yaw 미끄러짐),
  기준인 차량 지도 자체가 59 cm 흐름, 초기값(T0 0.5 cm)이 ICP보다 정확.
- 3·4가 전역에서 나쁜 이유: 상대 측정으로 드론 지도를 "차량 지도"에 맞추므로 차량 지도 누적오차를 따라감.
- 공통 관측 실측(정답 배치, 10 cm 격자 66,846칸): 바닥 55 %, 외벽 39 %, 랜드마크 2.6 %, 기둥 2.4 %.
  랜드마크 C(L자, 4면, 889칸)·D(계단, 4면, 545칸) 유효, A(상자, 사실상 한 면)·B(원기둥, yaw 불가) 약함.
- 방식 4 특징점: 364쌍 → 62 채택 → 기준점 10, 정확도 중앙값 2.2 cm / 90 % 5.3 cm. 캐노피 구간(경로 21.8–52.3 m) 기준점 없음.

### 남은 일 (다음 세션) — 공동 최적화 (방식 5)
목표: 두 로봇 경로를 하나의 포즈 그래프로 묶어 **두 지도를 함께** 고쳐 전역·국소 오차를 동시에 줄이기.

**그래프**
- 노드: 차량 161 + 드론 160 포즈(6자유도). 원점: 차량 첫 노드 고정.
- 연결(측정 = 연결, 가중치 Ω = 공분산⁻¹):
  - 차량 / 드론 오도메트리: 연속 노드 간 상대 이동(RTAB-Map 결과)
  - T0: 출발 시 차량 ↔ 드론
  - ArUco 출발 / 도착: 측정 시점 차량 ↔ 드론 (깊이 방향 산포가 커서 비등방 가중치)
  - RGB-D 특징점 62개: 드론 노드 i ↔ 차량 노드 j (`feature_anchor.match_anchors`, 강건 커널 Huber/Cauchy)
  - 추가: 차량 바닥 조건(높이 일정, roll/pitch 0 → 차량 z 누적오차 +32 cm 제거 기대), 드론 고도(기압계/거리센서)
- 비용 min Σ eᵀΩe, e = log(Z⁻¹·X_i⁻¹·X_j). Gauss-Newton / LM, GTSAM 또는 g2o (설치 여부 확인부터).

**캘리브레이션 (가중치 정하기, 결과를 좌우)**
1. 오도메트리 노이즈 모델 σ = a·이동거리 + b·회전각 을 로봇별로 추정. 사실: 차량은 회전마다 8–10 cm,
   드론은 경로 대비 약 0.27 %(차량 약 1 %). 방법: 정답 있는 짧은 주행으로 1회 측정(센서 캘리브레이션 성격),
   또는 정답 없이 ArUco 출발–도착 차이로 추정. RTAB-Map 연결 공분산이 실제 드리프트와 맞는지 먼저 비교(미확인).
2. 측정 노이즈: T0 시뮬 ~1 cm(실제는 수 cm), ArUco 3–4 cm(깊이 방향 ~5 cm), 특징점 중앙값 2.2 / 90 % 5.3 cm.
3. 기하 캘리브레이션(실제 로봇 필수): 카메라 장착 위치, 마커 위치, 핸드–아이. 시뮬은 URDF 값이라 정확.

**지도 재생성과 평가**
- 최적화 포즈로 노드별 RGB-D 점군 재배치(노드 이미지 `maps/serpentine_aruco/*_rgb|depth|calib` 사용) → 합치기.
- RViz `aruco_viz` 비교에 5번째 패널 추가.
- 지표 3개 + 새 지표: **지도 점 ↔ 월드 실제 표면 거리**(`layout.py` 기하로 직접 계산).

**예상 (추정, 검증 필요)**
- 국소: 기준점 구간은 수 cm 유지, 4번 이상.
- 전역: 상대 측정만이라 두 지도가 같이 틀린 부분은 못 고침. 가중치가 맞으면 덜 흐른 드론 쪽으로 쏠림 →
  끝 오차 대략 √(59²·20²/(59²+20²)) ≈ 19 cm 수준(지금 4번 61.6 cm), 바닥 조건으로 더 감소 가능.
- 캐노피 구간은 오도메트리 보간 수준 오차 남음. 가중치가 틀리면 오히려 악화 → 캘리브레이션 검증 필수.

**그 밖에**
- 차량 VO가 제자리 회전에서 크게 흐름 → 바퀴 오도메트리 + IMU 융합(`robot_localization`) 또는 회전 속도 낮추기 / 큰 반경 회전.
  (재실행 필요, 약 15분, 절전 끌 것)
- 착륙 단계 재확인(이번 실행은 착륙 직전 절전으로 중단).
- 데이터: `maps/serpentine_aruco`(365 MB)는 재분석에 필요해 보관. DB(157 MB)는 미사용이라 지워도 됨.

### 다음 세션 시작 방법
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run couv_map_merge compare_aruco.py --run ~/ros2_ws/maps/serpentine_aruco \
    --baseline ~/ros2_ws/maps/couv_20261027_200102_pitch90 --docs ~/ros2_ws/src/couv_map_merge/docs/serpentine  # 약 1분
ros2 launch couv_map_merge aruco_viz.launch.py      # RViz 1 | 2 | 3 | 4 (+ 빨강)
python3 -c "import gtsam" || pip install gtsam      # 공동 최적화 준비
```
코드 위치: `src/couv_map_merge/couv_map_merge/compare_aruco.py`(`solve()`, 지표, 구간 분석),
`feature_anchor.py`(특징점 매칭), `aruco_viz.py`(RViz), `registration.py`(입출력·ICP).

### 주의 (이번 세션에서 겪은 것)
- 비대화형 셸의 백그라운드 프로세스는 SIGINT를 무시 → `ros2 run` 노드는 SIGTERM으로 종료.
  (이전 테스트의 미션 노드가 남아 서로 다른 명령을 내던 문제가 있었음)
- `pkill -f` 패턴이 자기 셸 명령줄에도 걸릴 수 있음.
- 시뮬은 한 번에 하나만 실행(`/clock` 두 개 → 시간 역행 경고).
- 노트북 절전 시 실행 중인 ROS/gz 통신이 끊기고 복구 안 됨(세션 2). 긴 실행 전 절전/화면잠금 해제.
- `ros2 launch`를 `kill -9`하면 자식(bridge, bag record)이 고아로 남음 → 다음 실행 전에 `ps`로 확인.

### 3단계까지의 기준 수치 (비교용)
- 일자 경로, 드론 90°, T0 + 거리센서, ICP 없음: 정합 오차(지도 원점) 6.6 cm, 0.34°.
  드론 SLAM ATE 13.6 cm(최대 35 cm).
- 예상(4단계): ArUco 1회 → 출발점 1~2 cm, 2회 + ㄹ자 → 전체 2~5 cm.

---
