# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : FollowManager.py
# Project Name : HealthcareRobotPyRT
# Description  : Follow mode manager
# -------------------------------------------------------------------------------------------------------------------- #

from Commons import *

from ROSIntegration.ROSHealthcareNode import CROSHealthcareNode


class CFollowManager:
    """
    Follow 기능 관리자

    역할:
        1. follow 시작/중지 관리
        2. follow 상태 관리
        3. ROSHealthcareNode follow 기능 호출
    """

    def __init__(self):

        self._ros_node = CROSHealthcareNode() # ROS 통합 노드

        self._is_following = False # 현재 follow 중인지 여부

        self._target_patient_id = None # 현재 추적 대상 patient_id, None이면 추적 대상 없음

        write_log("FollowManager initialized.", self)

    # ==============================================================================================================
    # Start / Stop
    # ==============================================================================================================

    def start_follow( # follow 시작 함수
            self,
            patient_id: str = None # 추적할 환자 ID
    ):
        """
        follow 시작
        """

        self._is_following = True # follow 상태 True로 설정

        self._target_patient_id = patient_id # 추적 대상 patient_id 설정

        write_log(
            "Follow started | patient_id=%s"
            % str(patient_id),
            self
        )

    def stop_follow(self): # follow 중지 함수
        """
        follow 종료
        """

        self._ros_node.stop_follow() # ROSHealthcareNode의 follow 기능 중지 호출

        self._is_following = False # follow 상태 False로 설정

        self._target_patient_id = None # 추적 대상 patient_id 초기화

        write_log("Follow stopped.", self)

    # ==============================================================================================================
    # Process
    # ==============================================================================================================

    def process( # follow 처리 함수
            self,
            frame_bgr,
            depth_map
    ):
        """
        follow 처리
        """

        if not self._is_following: # follow 중이 아니면 처리하지 않고 종료
            return

        self._ros_node.process_follow( # ROSHealthcareNode의 follow 처리 호출, 내부에서 환자 검출 -> 거리 계산 -> APF -> /cmd_vel publish
            frame_bgr,
            depth_map
        )

    # ==============================================================================================================
    # Draw
    # ==============================================================================================================

    def draw_result( # follow 결과 시각화 함수
            self,
            frame_bgr,
            depth_map
    ):
        """
        follow 시각화 결과 반환
        """

        if not self._is_following: # follow 중이 아니면 시각화하지 않고 원본 프레임 반환
            return frame_bgr

        return self._ros_node.draw_follow_result( # ROSHealthcareNode의 follow 시각화 결과 반환 호출, 내부에서 환자 검출 결과를 프레임에 그려서 반환
            frame_bgr,
            depth_map
        )

    # ==============================================================================================================
    # State
    # ==============================================================================================================

    def is_following(self):
        """
        현재 follow 중인지 반환
        """

        return self._is_following

    def has_target(self):
        """
        target patient가 있는지 반환
        """

        return self._target_patient_id is not None

    def get_target_patient_id(self):
        """
        현재 추적 대상 patient_id 반환
        """

        return self._target_patient_id

    # ==============================================================================================================
    # Reset
    # ==============================================================================================================

    def reset(self): # follow 상태 초기화 함수, MPHealthcareROS에서 시스템 초기화 또는 리셋 시 호출됨

        self.stop_follow() # follow 중지하여 상태 초기화, 내부에서 /cmd_vel을 0으로 설정하여 로봇 정지

        self._is_following = False # follow 상태 False로 설정

        self._target_patient_id = None # 추적 대상 patient_id 초기화

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_ros_node(self): # ROSHealthcareNode 객체 반환 함수, MPHealthcareROS에서 ROSHealthcareNode에 직접 접근해야 할 때 호출됨
        return self._ros_node