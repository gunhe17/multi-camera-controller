**Resource Class 검증 항목**
System Resource

Memory: 사용 가능한 RAM 용량 (최소 8GB 권장)
Storage: Output directory 디스크 여유 공간
CPU: 사용률 및 온도 상태
Process: 기존 실행 중인 capture process cleanup

Windows COM/MediaFoundation

COM Library: CoInitialize 상태 확인
MF Platform: MFStartup/MFShutdown 테스트
Permission: 카메라 접근 권한 확인

External Dependencies

FFmpeg: 경로 및 실행 가능 여부
Driver: USB/Camera driver 상태


**Camera Class 검증 항목**
Device Detection

Enumeration: MFEnumDeviceSources로 감지되는 device 수
Index Validation: Config의 camera_indices 유효성
Friendly Name: 각 camera 식별 정보

Capability Test

Resolution: 요청한 해상도 지원 여부 (1280x720, 1920x1080)
Frame Rate: 30fps 지원 확인
Pixel Format: MJPG, YUV420 등 format 지원
Connection: 실제 stream 시작/중지 테스트

Performance Check

Latency: Frame capture 지연 시간 측정
Stability: 5초간 연속 capture 안정성
Sync: Multi-camera timestamp 동기화 오차


**Tobii Class 검증 항목**
SDK/Library

Tobii SDK: 라이브러리 로드 및 초기화
License: SDK license 유효성
Version: 호환되는 version 확인

Hardware Connection

Device Detection: Eye tracker 연결 상태
Calibration: 기존 calibration 유효성
Tracking Quality: Eye tracking 정확도 테스트

Data Stream

Gaze Data: Real-time gaze coordinate 수신
Timestamp: Camera와 동기화 가능한 timestamp
Sample Rate: 설정된 sampling rate 달성 여부


**Arducam Class 검증 항목**
SDK Initialization

Library: ArducamCamera.hpp 라이브러리 로드
Parameter: Default Param 객체 생성
Open/Close: Camera.open() 성공 여부

Configuration Test

Resolution: CameraConfig height/width 설정
Format: 지원되는 image format 확인
Log Level: Trace, Debug 등 log 출력 테스트

Stream Validation

Init: camera.init() 성공
Capture: 실제 frame capture 테스트
Error Handling: lastErrorMessage() 동작 확인


**통합 검증 Flow**
Pre-Check

Process Kill: 기존 capture process 종료
Resource Clear: Buffer, handle cleanup
Permission: UAC, camera access 권한

Validation Sequence

Resource → Camera → Tobii → Arducam 순서
각 단계별 PASS/FAIL 상태 리포트
Critical Error 발생 시 즉시 중단

Post-Validation

Configuration: 최적 설정값 제안
Cleanup: 테스트 중 생성된 임시 파일 삭제
Report: 전체 검증 결과 summary