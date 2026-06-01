# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MarkerDetector.py
# Project Name : HealthcareRobotPyRT
# Description  : ArUco marker detector for patient identification
# -------------------------------------------------------------------------------------------------------------------- #

import cv2
import numpy as np

from Commons import *


class CMarkerDetector:
    """
    ArUco 마커 검출기

    입력:
        - 사람 ROI 이미지(BGR)

    출력:
        - 검출된 ArUco marker 정보 리스트
    """

    def __init__(self):

        self._aruco_dict = cv2.aruco.getPredefinedDictionary( # ArUco 마커 종류 설정, 4x4 격자에 50개 마커가 있는 사전 사용
            cv2.aruco.DICT_4X4_50
        )

        self._aruco_params = cv2.aruco.DetectorParameters() # ArUco 검출 매개변수 초기화, 기본값 사용

        self._detector = cv2.aruco.ArucoDetector( # ArUco 검출기 객체 생성, 지정한 사전과 매개변수 사용
            self._aruco_dict,
            self._aruco_params
        )

    # ==============================================================================================================
    # Detect
    # ==============================================================================================================

    def detect(self, roi_bgr: np.ndarray): # ArUco 마커 검출
        """
        ROI 이미지에서 ArUco 마커를 검출한다.

        반환:
            [
                {
                    "id": marker_id,
                    "corners": corners,
                    "center": center_xy
                },
                ...
            ]
        """
        # roi_bgr : 사람 ROI 이미지 (BGR, OpenCV format)

        result = []

        if roi_bgr is None:
            return result

        if roi_bgr.size == 0:
            return result

        try:
            corners, ids, _ = self._detector.detectMarkers(roi_bgr) # ArUco 마커 검출, corners는 각 마커의 4개 코너 좌표, ids는 각 마커의 ID

            if ids is None:
                return result

            for i, marker_id in enumerate(ids.flatten()): # ids.flatten()는 2D 배열을 1D 배열로 변환, 각 마커 ID에 대해 반복, i : 인덱스, marker_id : 마커 ID
                marker_corners = corners[i].astype(int).reshape(-1, 2) # 각 마커의 코너 좌표를 정수형으로 변환하고 (4, 2) 형태로 재구성, corners[i]는 i번째 마커의 4개 코너 좌표 (1, 4, 2) 형태이므로 reshape(-1, 2)를 통해 (4, 2) 형태로 변환

                center_xy = marker_corners.mean(axis=0).astype(int) # 마커의 중심 좌표 계산

                result.append({ # 검출된 마커 정보를 결과 리스트에 추가, 각 마커 정보는 ID, 코너 좌표, 중심 좌표를 포함하는 딕셔너리 형태
                    "id": int(marker_id),
                    "corners": marker_corners,
                    "center": center_xy,
                })

        except Exception:
            ErrorHandler().report()

        return result # 검출된 마커 정보 리스트 반환

    # ==============================================================================================================
    # Coordinate Transform
    # ==============================================================================================================

    def roi_to_frame_coord( # ROI 좌표계(사람 좌표를 구하고 마커 좌표를 구하는데 사람 ROI 안에서 좌표를 구한 것임)를 원본 프레임 좌표계로 변환
            self,
            marker_info: dict, # 마커 정보 딕셔너리 (ID, 코너 좌표, 중심 좌표 포함)
            roi_x1: int, # ROI의 원본 프레임에서의 왼쪽 상단 x좌표
            roi_y1: int, # ROI의 원본 프레임에서의 왼쪽 상단 y좌표
            scale_up: float = 1.0 # ROI 이미지가 원본 프레임에서 축소된 경우, 축소된 비율로 좌표를 다시 원본 프레임 크기로 확장하는 데 사용 (예: scale_up=2.0이면 ROI 이미지가 원본 프레임의 절반 크기이므로 좌표를 2배로 확장)
    ):
        """
        ROI 좌표계의 마커 좌표를 원본 frame 좌표계로 변환한다.
        """

        corners = marker_info["corners"] # ROI 좌표계에서의 마커 코너 좌표 (4, 2) 형태

        frame_corners = (corners / scale_up).astype(int) + np.array([roi_x1, roi_y1]) # ROI 좌표계를 원본 프레임 좌표계로 변환, 먼저 scale_up으로 좌표를 확장한 후, ROI의 왼쪽 상단 좌표(roi_x1, roi_y1)를 더하여 원본 프레임에서의 절대 좌표로 변환

        frame_center = frame_corners.mean(axis=0).astype(int) # ROI 좌표계에서의 마커 중심 좌표를 원본 프레임 좌표계로 변환, 먼저 frame_corners에서 중심 좌표를 계산한 후, 정수형으로 변환

        return { # 변환된 마커 정보를 딕셔너리 형태로 반환, ID는 그대로 유지, 코너 좌표와 중심 좌표는 원본 프레임 좌표계로 변환된 값으로 업데이트
            "id": marker_info["id"],
            "corners": frame_corners,
            "center": frame_center,
        }

    # ==============================================================================================================
    # Draw
    # ==============================================================================================================

    @staticmethod
    def draw_marker( # 이미지 위에 마커 테두리와 중심점을 표시
            image_bgr: np.ndarray,
            marker_info: dict,
            color=(0, 255, 255), # 마커 테두리와 중심점 색상 (BGR, 여기서는 노란색)
            thickness: int = 2
    ):
        """
        이미지 위에 마커 테두리와 중심점을 표시한다.
        """

        corners = marker_info.get("corners", None)

        if corners is None:
            return image_bgr

        if len(corners) >= 4:
            cv2.polylines(
                image_bgr,
                [corners],
                True,
                color,
                thickness
            )

        center = marker_info.get("center", None)

        if center is not None:
            cv2.circle(
                image_bgr,
                tuple(center),
                4,
                color,
                -1
            )

        return image_bgr