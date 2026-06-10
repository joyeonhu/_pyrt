# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ROSFollowNode.py
# Project Name : HealthcareRobotPyRT
# Description  : Patient follow logic node
# -------------------------------------------------------------------------------------------------------------------- #

import cv2

from Commons import *
from HealthcareRobot.HealthcareMessage import *

from Perception.PatientDetector import CPatientDetector
from HealthcareRobot.PatientDB import CPatientDB
from Control.APFController import CAPFController


class CROSFollowNode:
    """
    환자 추적 로직

    역할:
        1. frame에서 환자 검출
        2. marker_id 기반 환자 정보 조회
        3. depth 기반 실제 거리 계산
        4. APF 기반 cmd_vel 계산
        5. 계산된 linear_x, angular_z 반환

    주의:
        이 클래스는 더 이상 /cmd_vel을 직접 publish하지 않는다.
        실제 Stella B2 구동은 ControlCore -> MPStellaB2 경로로 수행한다.
    """

    def __init__(self):

        self._patient_detector = CPatientDetector()

        self._patient_db = CPatientDB()

        self._apf_controller = CAPFController()

        self._last_patient_id = None
        self._last_real_distance_cm = None
        self._last_cmd_vel = {
            KEY_LINEAR_X: 0.0,
            KEY_ANGULAR_Z: 0.0,
        }

        write_log("ROSFollowNode initialized.", self)

    # ==============================================================================================================
    # Process
    # ==============================================================================================================

    def process(
            self,
            frame_bgr,
            depth_map
    ):
        """
        frame 처리 후 follow용 cmd_vel 계산

        반환:
            linear_x, angular_z
        """

        if frame_bgr is None:
            return self._return_stop("FOLLOW_NO_FRAME")

        patients = self._patient_detector.detect(frame_bgr)

        if len(patients) == 0:
            return self._return_stop("FOLLOW_NO_PATIENT")

        # ----------------------------------------------------------------------------------------------------------
        # 첫 번째 환자 사용
        # ----------------------------------------------------------------------------------------------------------

        patient = patients[0]

        marker_id = patient["marker_id"]

        center_x, center_y = patient["marker_center"]

        self._last_patient_id = marker_id

        # ----------------------------------------------------------------------------------------------------------
        # Depth
        # ----------------------------------------------------------------------------------------------------------

        depth_m = None

        if depth_map is not None:
            try:
                depth_m = float(depth_map[int(center_y), int(center_x)])

                if depth_m <= 0:
                    depth_m = None

            except Exception:
                depth_m = None

        if depth_m is None:
            return self._return_stop("FOLLOW_NO_DEPTH")

        # ----------------------------------------------------------------------------------------------------------
        # Real Distance
        # ----------------------------------------------------------------------------------------------------------

        depth_cm = depth_m * 100.0

        real_distance_cm = self._patient_db.calculate_range(
            marker_id,
            depth_cm
        )

        if real_distance_cm is None:
            return self._return_stop("FOLLOW_NO_REAL_DISTANCE")

        self._last_real_distance_cm = real_distance_cm

        real_distance_m = real_distance_cm / 100.0

        # ----------------------------------------------------------------------------------------------------------
        # APF
        # ----------------------------------------------------------------------------------------------------------

        linear_x, angular_z = self._apf_controller.calculate_cmd_vel(
            center_x,
            real_distance_m
        )

        self._last_cmd_vel = {
            KEY_LINEAR_X: linear_x,
            KEY_ANGULAR_Z: angular_z,
        }

        # ----------------------------------------------------------------------------------------------------------
        # Log
        # ----------------------------------------------------------------------------------------------------------

        patient_name = self._patient_db.get_patient_name(marker_id)

        write_log(
            "FOLLOW | patient=%s | dist=%.2f m | linear=%.2f | angular=%.2f"
            % (
                patient_name,
                real_distance_m,
                linear_x,
                angular_z
            ),
            self
        )

        return linear_x, angular_z

    # ==============================================================================================================
    # Stop Return
    # ==============================================================================================================

    def _return_stop(self, reason: str = ""):
        """
        follow 실패/정지 상황에서 0 velocity 반환
        """

        self._last_cmd_vel = {
            KEY_LINEAR_X: 0.0,
            KEY_ANGULAR_Z: 0.0,
        }

        if reason:
            write_log(
                "FOLLOW STOP | reason=%s" % str(reason),
                self
            )

        return 0.0, 0.0

    # ==============================================================================================================
    # Draw
    # ==============================================================================================================

    def draw_result(
            self,
            frame_bgr,
            depth_map
    ):
        """
        detection 결과 시각화
        """

        if frame_bgr is None:
            return frame_bgr

        patients = self._patient_detector.detect(frame_bgr)

        frame_bgr = self._patient_detector.draw_result(
            frame_bgr,
            patients
        )

        for patient in patients:

            marker_id = patient["marker_id"]

            center_x, center_y = patient["marker_center"]

            patient_name = self._patient_db.get_patient_name(
                marker_id
            )

            cv2.putText(
                frame_bgr,
                "Name: %s" % patient_name,
                (int(center_x) + 10, int(center_y) + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

            if depth_map is not None:

                try:
                    depth_m = float(
                        depth_map[int(center_y), int(center_x)]
                    )

                    depth_cm = depth_m * 100.0

                    real_distance_cm = self._patient_db.calculate_range(
                        marker_id,
                        depth_cm
                    )

                    if real_distance_cm is not None:

                        cv2.putText(
                            frame_bgr,
                            "REAL_DISTANCE = %.2f cm"
                            % real_distance_cm,
                            (int(center_x) + 10, int(center_y) + 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (255, 255, 255),
                            2
                        )

                except Exception:
                    pass

        return frame_bgr

    # ==============================================================================================================
    # Stop
    # ==============================================================================================================

    def stop(self):
        """
        follow 중지

        실제 로봇 정지는 ControlCore -> MPStellaB2에서 처리한다.
        여기서는 내부 상태만 0 velocity로 정리한다.
        """

        self._return_stop("FOLLOW_STOP_REQUESTED")

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_last_patient_id(self):
        return self._last_patient_id

    def get_last_real_distance_cm(self):
        return self._last_real_distance_cm

    def get_last_cmd_vel(self):
        return self._last_cmd_vel