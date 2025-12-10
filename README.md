# sensor_diagnostic

## 개요

`sensor_diagnostic` 패키지는 자율주행 시스템에 사용되는 주요 센서(LiDAR, Camera, IMU, GNSS)의 상태를 실시간으로 모니터링하고 진단하는 기능을 제공한다.  
ROS 2의 `diagnostic_updater`를 기반으로 구현되어 있으며, 각 센서의 **연결 상태**, **데이터 수신 주기**, **데이터 유효성**, **물리적 가림 현상** 등을 검사하여 `diagnostic_msgs/DiagnosticStatus` 메시지로 발행한다.

---

## 의존성 (Dependencies)

이 패키지는 다음 패키지들에 의존한다.

- `rclcpp`
- `diagnostic_updater`
- `diagnostic_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `image_transport`
- `cv_bridge`
- `skyautonet_msgs` (GNSS Heading 메시지용)

---

## 노드 및 기능 설명

패키지는 센서 유형별로 **4개의 독립적인 노드**를 제공한다.

---

### 1. `lidar_diagnostic_node`

LiDAR 센서의 연결 상태와 포인트 클라우드 데이터의 무결성을 진단한다.

#### Subscribed Topics

- `/sensing/lidar/{sensor_name}/pointcloud`  
  (`sensor_msgs/msg/PointCloud2`)

#### 진단 항목

- **Timeout**  
  - 포인트 클라우드 수신 지연 여부 확인  
  - 타겟 IP에 대한 Ping 응답 지연 시간(Latency) 확인
- **Point Distribution**  
  - NaN/Inf 포인트 포함 여부  
  - 모든 좌표가 0인 경우 감지  
  - 특정 섹터(Sector)의 포인트 부재 여부 검사
- **Ring**  
  - 특정 채널(Ring) 범위의 포인트 개수 체크  
  - 해당 범위의 평균 강도(Intensity)가 임계값 이상인지 확인  
  - (벨로다인/로보센스 계열 등 `ring` 필드가 있는 경우 유효)

#### 주요 파라미터

- `sensor_name` : 센서 이름 (토픽 경로 구성에 사용)
- `target_ip` : 센서 하드웨어 IP 주소 (Ping 테스트용)
- `cloud_timeout_sec` : 데이터 수신 타임아웃 시간
- `sector_divisions` : 포인트 분포 확인을 위한 시야각 분할 수
- `min_points_per_sector` : 분할 영역당 최소 포인트 수
- `ring_upper_start`, `ring_upper_end` : 진단할 상단 Ring 범위

---

### 2. `imu_diagnostic_node`

IMU 센서의 연결 및 데이터 이상 유무를 진단한다.

#### Subscribed Topics

- `/imu_raw` (또는 파라미터로 지정된 토픽)  
  (`sensor_msgs/msg/Imu`)

#### 진단 항목

- **Device**  
  - 장치 파일(`/dev/ttyUSB*` 등)의 접근 가능 여부 확인
- **Timeout**  
  - 데이터 수신 중단 여부 확인
- **Frequency**  
  - 데이터 수신 주기가 설정된 최소 주파수 이상인지 검사
- **Data**  
  - 가속도, 각속도, 쿼터니언 값에 NaN 또는 Inf 포함 여부 검사
- **Outlier**  
  - 가속도 및 각속도의 크기가 비정상적으로 큰지 확인  
  - 쿼터니언 정규화가 올바른지 검사
- **Stuck**  
  - 센서 값이 변동 없이 고정(Freeze)되어 있는지 확인

#### 주요 파라미터

- `device_path` : 장치 파일 경로
- `min_expected_frequency` : 기대 최소 수신 주파수(Hz)
- `accel_threshold` : 가속도 이상치 임계값
- `gyro_threshold` : 각속도 이상치 임계값

---

### 3. `camera_diagnostic_node`

카메라의 프레임 수신 상태와 이미지가 가려졌는지 여부를 진단한다.

#### Subscribed Topics

- `/sensing/camera/{sensor_name}/image_raw`  
  (`sensor_msgs/msg/Image`)

#### 진단 항목

- **Timeout**  
  - 이미지 수신 중단 여부 확인
- **FrameRate**  
  - 현재 수신 FPS가 기대치보다 낮은지 검사
- **Blocked**  
  - 이미지의 평균 밝기, 표준편차, 색상 히스토그램을 분석  
  - 렌즈가 가려졌거나 시야가 차단되었는지 판단

#### 주요 파라미터

- `sensor_name` : 카메라 이름
- `image_transport` : 이미지 전송 방식 (`raw`, `compressed` 등)
- `expected_fps` : 기대 FPS
- `brightness_dark_threshold` : 어두움 판단 임계값
- `brightness_bright_threshold` : 밝음(포화) 판단 임계값

---

### 4. `gnss_diagnostic_node`

GNSS 수신 상태와 위치/헤딩 데이터의 튀는 현상(Jump)을 진단한다.

#### Subscribed Topics

- `/sensing/gnss/fix` (파라미터로 변경 가능)  
  (`sensor_msgs/msg/NavSatFix`)
- `/sensing/gnss/heading` (파라미터로 변경 가능)  
  (`skyautonet_msgs/msg/Gphdt`)

#### 진단 항목

- **Device**  
  - 장치 파일 접근 가능 여부 확인
- **Timeout**  
  - 데이터 수신 지연 여부 확인
- **Fix**  
  - GNSS 상태(status)가 정상인지 확인
- **Data**  
  - 위도, 경도, 고도 값 유효성 검사
- **Jump**  
  - 이전 위치 대비 이동 속도가 비정상적으로 빠른 경우  
    → 위치 점프(Jump)로 간주
- **Heading**  
  - 헤딩 값의 급격한 변화 감지

#### 주요 파라미터

- `max_jump_speed_kmph` : 위치 점프 판단을 위한 최대 속도 임계값
- `heading_jump_threshold_deg` : 헤딩 점프 판단 임계값(도)

---

## 빌드 및 실행

### 빌드 (Build)

`colcon`을 사용하여 패키지를 빌드한다.

```bash
colcon build --packages-select sensor_diagnostic
```
## 실행 (Launch)

제공된 런치 파일을 사용하여 **모든 진단 노드**를 한 번에 실행할 수 있다.  
필요에 따라 `sensor_diagnostic.launch.xml` 파일을 수정하여 파라미터를 조정한다.

```bash
ros2 launch sensor_diagnostic sensor_diagnostic.launch.xml
```
