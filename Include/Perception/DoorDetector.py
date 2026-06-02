# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DoorDetector.py
# Project Name : HealthcareRobotPyRT
# Description  : Door detector for guide/delivery arrival checking
# -------------------------------------------------------------------------------------------------------------------- #

import os
import cv2
import numpy as np

from ultralytics import YOLO

from Commons import *


class CDoorDetector:
    """
    문 검출기

    역할:
        - YOLO door model을 이용해서 frame 안의 문을 검출
        - 검출된 door bbox를 반환
        - 필요하면 door crop 이미지를 반환
    """

    def __init__(
            self,
            yolo_model_path: str = None,
            door_conf: float = 0.85,
    ):

        if yolo_model_path is None:
            root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
            yolo_model_path = os.path.join(root_path, "Data", "Models", "door_yolov8n.pt")

        self._yolo_model_path = yolo_model_path
        self._door_conf = door_conf # 문 검출 confidence threshold

        self._door_model = None # YOLO 모델 객체

        self.load_model()

    # ==============================================================================================================
    # Load Model
    # ==============================================================================================================

    def load_model(self):
        """
        YOLO door detector 로드
        """

        if not os.path.exists(self._yolo_model_path):
            raise FileNotFoundError(
                "Door YOLO model not found: %s" % self._yolo_model_path
            )

        self._door_model = YOLO(self._yolo_model_path)

        # warm-up
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        _ = self._door_model.predict(
            dummy,
            conf=self._door_conf,
            verbose=False
        )

        write_log("Door detector loaded: %s" % self._yolo_model_path, self)

    # ==============================================================================================================
    # Utils
    # ==============================================================================================================

    @staticmethod
    def clamp_box(x1, y1, x2, y2, width, height):
        """
        bbox가 이미지 밖으로 나가지 않도록 제한
        """

        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(width, x2)
        y2 = min(height, y2)

        return x1, y1, x2, y2

    @staticmethod
    def crop_door(image_bgr: np.ndarray, bbox):
        """
        door bbox 영역 crop
        """

        x1, y1, x2, y2 = bbox
        return image_bgr[y1:y2, x1:x2]

    # ==============================================================================================================
    # Detect
    # ==============================================================================================================

    def detect(self, image_bgr: np.ndarray):
        """
        frame에서 문을 검출한다.

        반환:
            [
                {
                    "bbox": [x1, y1, x2, y2],
                    "confidence": conf,
                    "crop": door_crop
                },
                ...
            ]
        """

        detected_doors = []

        if image_bgr is None:
            return detected_doors

        if image_bgr.size == 0: # 이미지가 비어있는 경우 빈 결과 반환
            return detected_doors

        height, width = image_bgr.shape[:2] # 이미지 크기 가져오기

        results = self._door_model.predict(
            image_bgr,
            conf=self._door_conf,
            verbose=False # 로그 출력하지 않도록 설정
        )

        for r in results:
            if r.boxes is None:
                continue

            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                # b.xyxy : 검출된 객체의 bbox 좌표 (x1, y1, x2, y2) 형태로 반환, b.xyxy[0]은 첫 번째 검출된 객체의 bbox 좌표를 가져옴, .tolist()는 텐서를 리스트로 변환, map(int, ...)는 좌표 값을 정수로 변환하여 x1, y1, x2, y2에 할당
                x1, y1, x2, y2 = self.clamp_box(
                    x1, y1, x2, y2,
                    width,
                    height
                )

                conf = 0.0
                if b.conf is not None:
                    conf = float(b.conf[0]) # [0.92] 이렇게 텐서로 반환되므로 첫 번째 요소를 가져와서 float로 변환하여 confidence 값으로 사용

                bbox = [x1, y1, x2, y2]
                crop = self.crop_door(image_bgr, bbox)

                detected_doors.append({ # 검출된 문 정보를 결과 리스트에 추가, 각 문 정보는 bbox, confidence, crop 이미지를 포함하는 딕셔너리 형태
                    "bbox": bbox,
                    "confidence": conf,
                    "crop": crop,
                })

        return detected_doors

    # ==============================================================================================================
    # Draw
    # ==============================================================================================================

    def draw_result(self, image_bgr: np.ndarray, doors: list):
        """
        검출된 문 bbox를 화면에 표시
        """

        for door in doors:
            x1, y1, x2, y2 = door["bbox"]
            conf = door.get("confidence", 0.0)

            cv2.rectangle(
                image_bgr,
                (x1, y1),
                (x2, y2),
                (0, 255, 255),
                2
            )

            cv2.putText(
                image_bgr,
                "Door %.2f" % conf,
                (x1, y1 - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

        return image_bgr

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_yolo_model_path(self): # YOLO 모델 경로 반환
        return self._yolo_model_path

    def get_door_conf(self): # 문 검출 confidence threshold 반환
        return self._door_conf