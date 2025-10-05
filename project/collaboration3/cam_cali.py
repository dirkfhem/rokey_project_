import cv2

cap = cv2.VideoCapture(0)

# 모든 카메라 속성 설정
cap.set(cv2.CAP_PROP_POS_MSEC, 0)                                   # 재생 위치 (밀리초)
cap.set(cv2.CAP_PROP_POS_FRAMES, 0)                                 # 프레임 인덱스
cap.set(cv2.CAP_PROP_POS_AVI_RATIO, 0)                              # 비디오 파일 상대 위치 (0~1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)                             # 프레임 너비
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)                             # 프레임 높이
cap.set(cv2.CAP_PROP_FPS, 30)                                       # 초당 프레임 수
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))       # 코덱
cap.set(cv2.CAP_PROP_FRAME_COUNT, 0)                                # 총 프레임 수 (읽기 전용)
cap.set(cv2.CAP_PROP_FORMAT, 0)                                     # 프레임 포맷 (YUV, RGB 등)
cap.set(cv2.CAP_PROP_MODE, 0)                                       # 백엔드 모드 (V4L2, DSHOW 등)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)                               # 밝기
cap.set(cv2.CAP_PROP_CONTRAST, 0.5)                                 # 대비
cap.set(cv2.CAP_PROP_SATURATION, 0.5)                               # 채도
cap.set(cv2.CAP_PROP_HUE, 0.5)                                      # 색조
cap.set(cv2.CAP_PROP_GAIN, 0.5)                                     # 게인
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)                              # 자동 노출 (0: 수동, 1: 자동)
cap.set(cv2.CAP_PROP_EXPOSURE, -4)                                  # 노출
cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)                                # RGB 변환 여부 (0: 비활성, 1: 활성)
cap.set(cv2.CAP_PROP_RECTIFICATION, 0)                              # 스테레오 카메라 보정
cap.set(cv2.CAP_PROP_MONOCHROME, 0)                                 # 흑백 모드 (0: 비활성, 1: 활성)
cap.set(cv2.CAP_PROP_SHARPNESS, 0.5)                                # 선명도
cap.set(cv2.CAP_PROP_GAMMA, 0.5)                                    # 감마
cap.set(cv2.CAP_PROP_TEMPERATURE, 4000)                             # 색온도
cap.set(cv2.CAP_PROP_TRIGGER, 0)                                    # 트리거 모드
cap.set(cv2.CAP_PROP_TRIGGER_DELAY, 0)                              # 트리거 지연
cap.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 4000)                     # 화이트 밸런스 (빨간색)
cap.set(cv2.CAP_PROP_ZOOM, 1)                                       # 줌
cap.set(cv2.CAP_PROP_FOCUS, 50)                                     # 초점
cap.set(cv2.CAP_PROP_GUID, 0)                                       # 카메라 GUID
cap.set(cv2.CAP_PROP_ISO_SPEED, 100)                                # ISO 감도
cap.set(cv2.CAP_PROP_BACKLIGHT, 0)                                  # 백라이트 보정
cap.set(cv2.CAP_PROP_PAN, 0)                                        # 팬
cap.set(cv2.CAP_PROP_TILT, 0)                                       # 틸트
cap.set(cv2.CAP_PROP_ROLL, 0)                                       # 롤
cap.set(cv2.CAP_PROP_IRIS, 0)                                       # 조리개
cap.set(cv2.CAP_PROP_SETTINGS, 0)                                   # 카메라 설정 창 표시
cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)                                 # 버퍼 크기
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)                                  # 자동 초점 (0: 수동, 1: 자동)
cap.set(cv2.CAP_PROP_SAR_NUM, 1)                                    # 샘플 종횡비 (Numerator)
cap.set(cv2.CAP_PROP_SAR_DEN, 1)                                    # 샘플 종횡비 (Denominator)
cap.set(cv2.CAP_PROP_BACKEND, cv2.CAP_ANY)                          # 백엔드 (ANY, V4L2, DSHOW 등)
cap.set(cv2.CAP_PROP_CHANNEL, 0)                                    # 카메라 채널
cap.set(cv2.CAP_PROP_AUTO_WB, 0)                                    # 자동 화이트 밸런스
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4000)                          # 화이트 밸런스 온도
cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 4000)                    # 화이트 밸런스 (파란색)
cap.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 4000)                     # 화이트 밸런스 (빨간색)
cap.set(cv2.CAP_PROP_CODEC_PIXEL_FORMAT, 0)                         # 코덱 픽셀 포맷
cap.set(cv2.CAP_PROP_BITRATE, 1000000)                              # 비트레이트
cap.set(cv2.CAP_PROP_ORIENTATION_META, 0)                           # 오리엔테이션 메타데이터
cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)                           # 자동 회전
cap.set(cv2.CAP_PROP_HW_ACCELERATION, cv2.VIDEO_ACCELERATION_ANY)   # 하드웨어 가속
cap.set(cv2.CAP_PROP_HW_DEVICE, 0)                                  # 하드웨어 디바이스 인덱스
cap.set(cv2.CAP_PROP_HW_ACCELERATION_USE_OPENCL, 0)                 # OpenCL 사용
cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 1000)                       # 오픈 타임아웃 (밀리초)
cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)                       # 읽기 타임아웃 (밀리초)
cap.set(cv2.CAP_PROP_STREAM_OPEN_TIME_USEC, 0)                      # 스트림 오픈 시간 (마이크로초)
cap.set(cv2.CAP_PROP_VIDEO_STREAM, 0)                               # 비디오 스트림 인덱스
cap.set(cv2.CAP_PROP_VIDEO_TOTAL_CHANNELS, 0)                       # 비디오 채널 수
cap.set(cv2.CAP_PROP_EXPOSUREPROGRAM, 0)                            # 노출 프로그램
cap.set(cv2.CAP_PROP_VIEWFINDER, 0)                                 # 뷰파인더 모드
cap.set(cv2.CAP_PROP_LRFHASKEY, 0)                                  # LRF 키 여부
cap.set(cv2.CAP_PROP_XI_TIMEOUT, 1000)                              # XI 카메라 타임아웃

# 리소스 해제
cap.release()