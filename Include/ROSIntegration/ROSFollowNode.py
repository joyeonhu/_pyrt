# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ROSFollowNode.py
# Project Name : HealthcareRobotPyRT
# Description  : ROS2 patient follow node
# -------------------------------------------------------------------------------------------------------------------- #

import cv2

from Commons import *

from Perception.PatientDetector import CPatientDetector
from HealthcareRobot.PatientDB import CPatientDB
from Control.APFController import CAPFController
from ROSIntegration.ROSCmdVelNode import CROSCmdVelNode


class CROSFollowNode:
    """
    환자 추적 노드

    역할:
        1. frame에서 환자 검출
        2. marker_id 기반 환자 정보 조회
        3. depth 기반 실제 거리 계산
        4. APF 기반 cmd_vel 계산
        5. /cmd_vel publish
    """

    def __init__(self):

        self._patient_detector = CPatientDetector()

        self._patient_db = CPatientDB()

        self._apf_controller = CAPFController()

        self._cmd_vel_node = CROSCmdVelNode()

        self._last_patient_id = None # 마지막으로 추적한 환자 ID, 디버깅 및 평가용

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
        frame 처리 후 follow 수행
        """

        if frame_bgr is None:
            return 0.0, 0.0

        patients = self._patient_detector.detect(frame_bgr)

        if len(patients) == 0:
            self._cmd_vel_node.stop_robot()
            return 0.0, 0.0

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

        depth_m = None # depth 값 저장할 변수

        if depth_map is not None:

            try:
                depth_m = float(depth_map[int(center_y), int(center_x)])

                if depth_m <= 0:
                    depth_m = None

            except Exception:
                depth_m = None

        if depth_m is None:
            self._cmd_vel_node.stop_robot()
            return 0.0, 0.0

        # ----------------------------------------------------------------------------------------------------------
        # Real Distance
        # ----------------------------------------------------------------------------------------------------------

        depth_cm = depth_m * 100.0

        real_distance_cm = self._patient_db.calculate_range( # 수평 거리 계산
            marker_id,
            depth_cm
        )

        if real_distance_cm is None:
            self._cmd_vel_node.stop_robot()
            return 0.0, 0.0

        real_distance_m = real_distance_cm / 100.0

        # ----------------------------------------------------------------------------------------------------------
        # APF
        # ----------------------------------------------------------------------------------------------------------

        linear_x, angular_z = self._apf_controller.calculate_cmd_vel(
            center_x,
            real_distance_m
        )

        # ----------------------------------------------------------------------------------------------------------
        # cmd_vel publish
        # ----------------------------------------------------------------------------------------------------------

        self._cmd_vel_node.publish_cmd_vel(
            linear_x,
            angular_z
        )

        # ----------------------------------------------------------------------------------------------------------
        # distance publish
        # ----------------------------------------------------------------------------------------------------------

        self._cmd_vel_node.publish_target_distance(
            real_distance_cm
        )

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
        """

        self._cmd_vel_node.stop_robot()

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_last_patient_id(self): # 디버깅 및 평가용으로 마지막으로 추적한 환자 ID 반환
        return self._last_patient_id