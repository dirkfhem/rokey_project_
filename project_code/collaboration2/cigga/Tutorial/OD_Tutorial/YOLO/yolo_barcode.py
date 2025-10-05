import cv2
import numpy as np
from pylibdmtx.pylibdmtx import decode as dmtx_decode
from pyzbar.pyzbar import decode as zbar_decode
import time
from datetime import datetime
from ultralytics import YOLO

class BarcodeScanner:
    def __init__(self, model_path="yolo11n.pt", output_file="scanned_barcodes.txt", barcode_class_id=0, camera_index=0):
        self.output_file = output_file
        self.barcode_class_id = barcode_class_id
        self.model = YOLO(model_path)  # YOLOv11n 모델 로드
        self.cap = None
        self.scanned_barcodes = set()  # 중복 방지를 위한 세트
        self.target_size = 640  # 목표 입력 크기 (640x640)
        self.camera_index = camera_index
    
    def start_camera(self):
        """카메라 시작"""
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)  # V4L2 백엔드 사용
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 720p 해상도
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            print(f"카메라 인덱스 {self.camera_index} 열기 실패. 사용 가능한 카메라 인덱스 확인 중...")
            for i in range(5):  # 0~4번 인덱스 시도
                self.cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                if self.cap.isOpened():
                    print(f"카메라 인덱스 {i}로 연결 성공")
                    self.camera_index = i
                    break
            if not self.cap.isOpened():
                raise Exception("사용 가능한 카메라를 찾을 수 없습니다.")
    
    def stop_camera(self):
        """카메라 종료"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
    
    def save_barcode(self, barcode_data, barcode_type="DATAMATRIX"):
        """바코드 데이터를 파일에 저장"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.output_file, "a") as f:
            f.write(f"{timestamp} | Type: {barcode_type} | Data: {barcode_data}\n")
    
    def pad_image(self, image):
        """이미지를 640x640으로 패딩 (검은색)"""
        h, w = image.shape[:2]
        target_size = self.target_size
        
        # 정사각형으로 만들기 위해 패딩 계산
        if h > w:
            pad_x = (h - w) // 2
            pad_y = 0
            new_w = h
            new_h = h
        else:
            pad_x = 0
            pad_y = (w - h) // 2
            new_w = w
            new_h = w
        
        # 이미지를 target_size로 리사이즈
        if new_h > target_size:
            ratio = target_size / new_h
            new_w = int(new_w * ratio)
            new_h = int(new_h * ratio)
            image = cv2.resize(image, (new_w, new_h))
        
        # 검은색 패딩 추가
        padded_image = np.zeros((target_size, target_size, 3), dtype=np.uint8)
        pad_x_offset = (target_size - new_w) // 2
        pad_y_offset = (target_size - new_h) // 2
        padded_image[pad_y_offset:pad_y_offset+new_h, pad_x_offset:pad_x_offset+new_w] = image
        
        return padded_image, pad_x_offset, pad_y_offset, new_w, new_h
    
    def get_yolo_bbox(self, frame):
        """YOLOv11n으로 바코드 바운딩 박스 얻기"""
        try:
            # 프레임을 640x640으로 패딩
            padded_frame, pad_x_offset, pad_y_offset, orig_w, orig_h = self.pad_image(frame)
            print(f"패딩된 프레임 크기: {padded_frame.shape}, 오프셋: ({pad_x_offset}, {pad_y_offset}), 원본 크기: ({orig_w}, {orig_h})")
            
            # YOLO 예측
            results = self.model.predict(padded_frame, conf=0.5, classes=[self.barcode_class_id], verbose=True)
            if not results or len(results) == 0:
                print("YOLO: 결과 없음")
                return None
            
            # 첫 번째 결과에서 바운딩 박스 확인
            boxes = results[0].boxes
            if boxes is None or len(boxes) == 0:
                print("YOLO: 바운딩 박스 없음")
                return None
            
            # 바운딩 박스 데이터 디버깅
            print(f"YOLO boxes 데이터: {boxes.xywh}")
            
            # 첫 번째 바운딩 박스 추출
            if len(boxes.xywh) == 0 or boxes.xywh.shape[0] < 1:
                print("YOLO: 유효한 바운딩 박스 데이터 없음")
                return None
            
            box = boxes.xywh[0].cpu().numpy()
            if box.size < 4:
                print(f"YOLO: 바운딩 박스 데이터 크기 부족: {box}")
                return None
            
            # xywh를 원본 프레임 좌표로 변환
            scale_x = frame.shape[1] / orig_w
            scale_y = frame.shape[0] / orig_h
            x = int((box[0] - box[2] / 2 - pad_x_offset) * scale_x)
            y = int((box[1] - box[3] / 2 - pad_y_offset) * scale_y)
            w = int(box[2] * scale_x)
            h = int(box[3] * scale_y)
            
            # 바운딩 박스 유효성 검사
            if w <= 0 or h <= 0 or x < 0 or y < 0:
                print(f"YOLO: 유효하지 않은 바운딩 박스: x={x}, y={y}, w={w}, h={h}")
                return None
            
            print(f"YOLO: 바운딩 박스 감지됨: x={x}, y={y}, w={w}, h={h}")
            return (x, y, w, h)
        
        except Exception as e:
            print(f"YOLO 예측 중 오류: {e}")
            return None
    
    def process_frame(self, frame, bbox):
        """프레임에서 바코드 처리"""
        try:
            if bbox is None:
                return frame, []
            
            x, y, w, h = bbox
            # 프레임 크기 내에서 ROI 유효성 검사
            h_frame, w_frame = frame.shape[:2]
            x = max(0, x)

            y = max(0, y)
            w = min(w, w_frame - x)
            h = min(h, h_frame - y)
            
            if w <= 0 or h <= 0:
                print(f"ROI 유효하지 않음: x={x}, y={y}, w={w}, h={h}")
                return frame, []
            
            # 바코드 영역 자르기 (여유空間 최소화)
            x, y, w, h = x-5, y-5, w+10, h+10
            x = max(0, x)
            y = max(0, y)
            w = min(w, w_frame - x)
            h = min(h, h_frame - y)
            
            barcode_roi = frame[y:y+h, x:x+w]
            
            # 이미지 전처리: 대비 조정
            barcode_roi = cv2.convertScaleAbs(barcode_roi, alpha=1.5, beta=50)
            
            # ROI 이미지 저장 (디버깅용)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"roi_{timestamp}.jpg", barcode_roi)
            print(f"ROI 저장됨: roi_{timestamp}.jpg, 크기: {barcode_roi.shape}")
            
            # DataMatrix 디코딩
            barcodes = dmtx_decode(barcode_roi)
            if not barcodes:
                print("pylibdmtx: 바코드 디코딩 실패, pyzbar 시도")
                barcodes = zbar_decode(barcode_roi)
            
            for barcode in barcodes:
                barcode_data = barcode.data.decode('utf-8')
                barcode_type = barcode.type if hasattr(barcode, 'type') else "DATAMATRIX"
                
                # 중복 방지
                if barcode_data not in self.scanned_barcodes:
                    self.scanned_barcodes.add(barcode_data)
                    self.save_barcode(barcode_data, barcode_type)
                    print(f"새 바코드 저장: {barcode_data} ({barcode_type})")
                
                # 바코드 영역 표시
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, barcode_data, (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            return frame, barcodes
        except Exception as e:
            print(f"프레임 처리 중 오류: {e}")
            return frame, []
    
    def run(self):
        """카메라로 바코드 스캔 실행"""
        self.start_camera()
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("프레임을 읽을 수 없습니다.")
                    break
                
                # 프레임 크기 디버깅
                print(f"원본 프레임 크기: {frame.shape}")
                
                # YOLOv11n으로 바코드 바운딩 박스 얻기
                bbox = self.get_yolo_bbox(frame)
                frame, barcodes = self.process_frame(frame, bbox)
                
                # 프레임 표시
                cv2.imshow('Barcode Scanner', frame)
                
                # 'q' 키로 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # 프레임 속도 제어
                time.sleep(0.2)  # 200ms 대기 (5fps)
                
        finally:
            self.stop_camera()

# 사용 예시
if __name__ == "__main__":
    scanner = BarcodeScanner(model_path="yolo11n.pt", output_file="scanned_barcodes.txt", barcode_class_id=0, camera_index=0)
    scanner.run()