# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : PatientDetector.py
# Project Name : HealthcareRobotPyRT
# Description  : Patient detection pipeline: person -> gown -> marker
# -------------------------------------------------------------------------------------------------------------------- #

import os
import cv2
import numpy as np

from ultralytics import YOLO

from Commons import *
from Perception.GownClassifier import CGownClassifier
from Perception.MarkerDetector import CMarkerDetector


class CPatientDetector:
    """
    환자 검출기

    역할:
        1. YOLO로 사람 검출
        2. 사람 상체 crop으로 환자복 판별
        3. 환자복이면 ArUco 마커 검출
        4. marker_id, center_x, bbox 등을 반환
    """

    def __init__(
            self,
            yolo_model_path: str = None,
            person_conf: float = 0.5,
    ):

        if yolo_model_path is None:
            root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
            yolo_model_path = os.path.join(root_path, "Data", "Models", "yolov8n.pt")

        self._yolo_model_path = yolo_model_path
        self._person_conf = person_conf

        self._person_model = None
        self._gown_classifier = CGownClassifier()
        self._marker_detector = CMarkerDetector()

        self.load_model()

    # ==============================================================================================================
    # Load Model
    # ==============================================================================================================

    def load_model(self):
        """
        YOLO person detector 로드
        """

        if not os.path.exists(self._yolo_model_path):
            raise FileNotFoundError(
                "YOLO model not found: %s" % self._yolo_model_path
            )

        self._person_model = YOLO(self._yolo_model_path)

        # warm-up
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        _ = self._person_model.predict(
            dummy,
            classes=[0],
            conf=self._person_conf,
            verbose=False
        )

        write_log("YOLO person detector loaded: %s" % self._yolo_model_path, self)

    # ==============================================================================================================
    # Utils
    # ==============================================================================================================

    @staticmethod
    def clamp_box(x1, y1, x2, y2, width, height): # bbox 좌표를 이미지 범위 안으로 제한하는 함수
        """
        bbox가 이미지 밖으로 나가지 않도록 제한
        """

        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(width, x2)
        y2 = min(height, y2)

        return x1, y1, x2, y2

    @staticmethod
    def get_torso_roi(image_bgr, bbox): # 사람 bbox에서 상체 영역만 crop하는 함수, image_bgr은 원본 이미지, bbox는 사람의 bounding box 좌표 (x1, y1, x2, y2) 형태
        """
        사람 bbox에서 상체 영역만 crop

        bbox:
            [x1, y1, x2, y2]
        """

        x1, y1, x2, y2 = bbox

        person_height = y2 - y1 # 사람 bbox의 높이 계산
        torso_y2 = y1 + int(person_height * 0.65)

        tx1 = x1
        ty1 = y1
        tx2 = x2
        ty2 = max(y1 + 1, torso_y2)

        torso = image_bgr[ty1:ty2, tx1:tx2]

        return torso # 상체 영역을 원본 이미지에서 crop하여 반환

    @staticmethod
    def get_person_roi(image_bgr, bbox): # 사람 전체 bbox 영역을 crop하는 함수, image_bgr은 원본 이미지, bbox는 사람의 bounding box 좌표 (x1, y1, x2, y2) 형태
        """
        사람 전체 ROI crop
        """

        x1, y1, x2, y2 = bbox
        return image_bgr[y1:y2, x1:x2] # 사람 전체 영역을 원본 이미지에서 crop하여 반환

    # ==============================================================================================================
    # Detect
    # ==============================================================================================================

    def detect(self, image_bgr: np.ndarray): # 환자 검출
        """
        frame에서 환자를 검출한다.

        반환:
            [
                {
                    "bbox": [x1, y1, x2, y2],
                    "is_gown": True,
                    "marker_id": 12,
                    "marker_center": [cx, cy],
                    "marker_corners": corners
                },
                ...
            ]
        """

        detected_patients = []

        if image_bgr is None:
            return detected_patients

        if image_bgr.size == 0:
            return detected_patients

        height, width = image_bgr.shape[:2]

        # ----------------------------------------------------------------------------------------------------------
        # 1. Person detection
        # ----------------------------------------------------------------------------------------------------------

        results = self._person_model.predict(
            image_bgr,
            classes=[0], # COCO dataset에서 사람 클래스는 0번
            conf=self._person_conf,
            verbose=False
        )

        for r in results:
            if r.boxes is None:
                continue

            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                x1, y1, x2, y2 = self.clamp_box(
                    x1, y1, x2, y2,
                    width, height
                )

                bbox = [x1, y1, x2, y2]

                # --------------------------------------------------------------------------------------------------
                # 2. Gown classification
                # --------------------------------------------------------------------------------------------------

                torso = self.get_torso_roi(image_bgr, bbox)
                is_gown = self._gown_classifier.is_gown(torso)

                if not is_gown:
                    continue

                # --------------------------------------------------------------------------------------------------
                # 3. Marker detection
                # --------------------------------------------------------------------------------------------------

                person_roi = self.get_person_roi(image_bgr, bbox)

                scale_up = 1.0
                if max(person_roi.shape[:2]) < 420:
                    person_roi = cv2.resize(
                        person_roi,
                        None,
                        fx=1.5,
                        fy=1.5,
                        interpolation=cv2.INTER_NEAREST
                    )
                    scale_up = 1.5

                markers = self._marker_detector.detect(person_roi)

                for marker in markers:
                    marker_frame = self._marker_detector.roi_to_frame_coord( # ROI 좌표계에서 원본 이미지 좌표계로 변환
                        marker,
                        x1,
                        y1,
                        scale_up
                    )

                    detected_patients.append({
                        "bbox": bbox,
                        "is_gown": True,
                        "marker_id": marker_frame["id"],
                        "marker_center": marker_frame["center"],
                        "marker_corners": marker_frame["corners"],
                    })

        return detected_patients

    # ==============================================================================================================
    # Draw
    # ==============================================================================================================

    def draw_result(self, image_bgr: np.ndarray, patients: list):
        """
        검출 결과를 화면에 표시
        """

        for patient in patients:
            x1, y1, x2, y2 = patient["bbox"]

            cv2.rectangle(
                image_bgr,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2
            )

            cv2.putText(
                image_bgr,
                "patient(gown)",
                (x1, y1 - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                2
            )

            marker_info = {
                "id": patient["marker_id"],
                "center": patient["marker_center"],
                "corners": patient["marker_corners"],
            }

            self._marker_detector.draw_marker(image_bgr, marker_info)

            cx, cy = patient["marker_center"]

            cv2.putText(
                image_bgr,
                "ArUco: %s" % str(patient["marker_id"]),
                (int(cx) + 10, int(cy) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )

        return image_bgr

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_yolo_model_path(self): # YOLO 모델 경로 반환
        return self._yolo_model_path