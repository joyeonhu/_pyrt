# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ROSHealthcareNode.py
# Project Name : HealthcareRobotPyRT
# Description  : Integrated ROS interface for healthcare robot
# -------------------------------------------------------------------------------------------------------------------- #

import rclpy

from Commons import *

from ROSIntegration.ROSFollowNode import CROSFollowNode
from ROSIntegration.ROSNavigationNode import CROSNavigationNode
from ROSIntegration.ROSCmdVelNode import CROSCmdVelNode


class CROSHealthcareNode:
    """
    Healthcare ROS 통합 노드

    역할:
        1. Follow 기능 관리
        2. Navigation 기능 관리
        3. cmd_vel 정지/출력 관리
        4. MPHealthcareROS 프로세스에서 사용할 ROS 인터페이스 제공
    """

    def __init__(self):

        self._is_initialized = False # ROS 기능 초기화 여부

        self._follow_node = None # 환자 추적 노드
        self._navigation_node = None # 네비게이션 노드
        self._cmd_vel_node = None # cmd_vel 퍼블리셔 노드

        self.initialize() # ROS 초기화 및 내부 노드 생성

    # ==============================================================================================================
    # Initialize
    # ==============================================================================================================

    def initialize(self):
        """
        ROS2 초기화 및 내부 노드 생성
        """

        if not rclpy.ok(): # ROS2가 초기화되어 있지 않으면
            rclpy.init() # ROS2 초기화

        self._follow_node = CROSFollowNode() # 환자 추적 노드 생성
        self._navigation_node = CROSNavigationNode() # 네비게이션 노드 생성
        self._cmd_vel_node = CROSCmdVelNode() # cmd_vel 퍼블리셔 노드 생성

        self._is_initialized = True # ROSHealthcareNode 초기화 완료

        write_log("ROSHealthcareNode initialized.", self)

    # ==============================================================================================================
    # Spin
    # ==============================================================================================================

    def spin_once(self, timeout_sec: float = 0.0): # ROS callback을 한 번 처리하는 함수, Nav2 결과 callback 같은 게 실행되려면 이 함수가 주기적으로 호출되어야 함
        """
        ROS callback 처리

        Nav2 action callback, cmd_vel node callback 등을 처리하기 위해
        MPHealthcareROS 루프에서 주기적으로 호출해야 한다.
        """

        if not self._is_initialized: # ROSHealthcareNode가 초기화되지 않았으면
            return # ROS callback 처리하지 않고 종료

        rclpy.spin_once( # Navigation 노드의 콜백 처리
            self._navigation_node, # 콜백이 등록된 노드 객체
            timeout_sec=timeout_sec # 콜백이 실행될 때까지 대기하는 시간
        )
        # rclpy.spin_once : ROS한테 navigation 노드에 들어온 콜백이 있으면 처리해달라고 요청하는 함수
        # callback 등록 -> Nav2 응답 도착 -> ROS 내부 큐에 콜백 대기 목록에 저장됨 -> spin_once() -> ROS가 실행 가능한 콜백 찾아서 실행
        rclpy.spin_once( # cmd_vel 노드의 콜백 처리, cmd_vel 노드는 현재 콜백이 없지만, 향후 필요할 수 있으므로 spin_once()로 처리
            self._cmd_vel_node,
            timeout_sec=timeout_sec
        )

    # ==============================================================================================================
    # Follow
    # ==============================================================================================================

    def process_follow( # 환자 추적을 수행하는 함수, MPHealthcareROS 루프에서 주기적으로 호출되어야 함
            self,
            frame_bgr, # 카메라에서 캡처한 BGR 이미지 프레임, 환자 검출에 사용됨
            depth_map # 카메라에서 캡처한 깊이 맵, 환자와의 거리 계산에 사용됨
    ):
        """
        환자 추적 처리
        """

        if not self._is_initialized:
            return 0.0, 0.0

        return self._follow_node.process(
            frame_bgr,
            depth_map
        )

    def draw_follow_result( # 환자 추적 결과를 시각화하는 함수, MPHealthcareROS 루프에서 주기적으로 호출되어야 함
            self,
            frame_bgr,
            depth_map
    ):
        """
        follow 검출 결과 시각화
        """

        if not self._is_initialized:
            return frame_bgr

        return self._follow_node.draw_result(
            frame_bgr,
            depth_map
        )

    def stop_follow(self): # 환자 추적을 멈추는 함수, MPHealthcareROS에서 follow 기능을 중지할 때 호출됨
        """
        follow 정지
        """

        if self._follow_node is not None: # Follow 노드가 존재하면
            self._follow_node.stop() # Follow 노드의 stop() 함수를 호출하여 환자 추적 멈춤, 내부에서 /cmd_vel을 0으로 설정하여 로봇 정지

    # ==============================================================================================================
    # Navigation
    # ==============================================================================================================

    def go_to_destination(self, destination_id: str): # 목적지 ID로 네비게이션을 시작하는 함수, MPHealthcareROS에서 GUIDE/DELIVERY 기능을 수행할 때 호출됨
        """
        GUIDE / DELIVERY 목적지 이동 요청
        """

        if not self._is_initialized:
            return False

        return self._navigation_node.go_to_destination( # Navigation 노드의 go_to_destination() 함수를 호출하여 네비게이션 시작, 내부에서 등록된 목적지 ID에 해당하는 좌표로 Nav2 action client를 통해 이동 요청
            destination_id # 목적지 ID, 목적지 ID를 Nav2 goal로 바꿔서 이동 요청
        )

    def go_to_pose( # 직접 좌표를 넣어서 이동 요청하는 함수 (목적지 ID 없이)
            self,
            x: float,
            y: float,
            yaw: float,
            destination_id: str = None
    ):
        """
        직접 좌표 기반 이동 요청
        """

        if not self._is_initialized:
            return False

        return self._navigation_node.go_to_pose(
            x,
            y,
            yaw,
            destination_id
        )

    def cancel_navigation(self): # 네비게이션 취소하는 함수, MPHealthcareROS에서 네비게이션을 중지할 때 호출됨
        """
        navigation 취소
        """

        if self._navigation_node is None:
            return False

        return self._navigation_node.cancel_navigation()

    def is_navigating(self): # 네비게이션이 진행 중인지 여부 반환하는 함수, MPHealthcareROS에서 네비게이션 상태를 확인할 때 호출됨
        if self._navigation_node is None:
            return False

        return self._navigation_node.is_navigating()

    def is_arrived(self): # 네비게이션이 목적지에 도착했는지 여부 반환하는 함수, MPHealthcareROS에서 네비게이션 상태를 확인할 때 호출됨
        if self._navigation_node is None:
            return False

        return self._navigation_node.is_arrived()

    def get_navigation_result(self): # 네비게이션 결과 반환하는 함수, MPHealthcareROS에서 네비게이션 결과를 확인할 때 호출됨 (예: 성공, 실패, 취소 등)
        if self._navigation_node is None:
            return None

        return self._navigation_node.get_last_result()

    def get_distance_remaining(self): # 네비게이션으로 목적지까지 남은 거리 반환하는 함수, MPHealthcareROS에서 네비게이션 상태를 확인할 때 호출됨
        if self._navigation_node is None:
            return None

        return self._navigation_node.get_distance_remaining()

    # ==============================================================================================================
    # cmd_vel
    # ==============================================================================================================

    def publish_cmd_vel( # 직접 cmd_vel을 publish하는 함수, MPHealthcareROS에서 APF 외에 직접 cmd_vel을 제어할 때 호출됨
            self,
            linear_x: float,
            angular_z: float
    ):
        """
        직접 cmd_vel publish
        """

        if self._cmd_vel_node is None:
            return

        self._cmd_vel_node.publish_cmd_vel( # /cmd_vel에 속도 명령 publish
            linear_x,
            angular_z
        )

    def stop_robot(self): # 로봇을 정지시키는 함수, MPHealthcareROS에서 로봇을 멈출 때 호출됨
        """
        로봇 정지
        """

        if self._cmd_vel_node is not None:
            self._cmd_vel_node.stop_robot() # linear.x와 angular.z를 0으로 설정하여 cmd_vel을 publish, 로봇 정지

    # ==============================================================================================================
    # Shutdown
    # ==============================================================================================================

    def shutdown(self): # ROSHealthcareNode를 종료하는 함수, MPHealthcareROS 종료 시 호출됨
        """
        ROSHealthcareNode 종료
        """

        try:
            self.stop_robot() # 로봇 정지

            if self._navigation_node is not None: # 네비게이션 노드가 존재하면
                self._navigation_node.destroy_node() # 네비게이션 노드의 destroy_node() 함수를 호출하여 노드 종료, 내부에서 Nav2 action client도 종료

            if self._cmd_vel_node is not None: # cmd_vel 노드가 존재하면
                self._cmd_vel_node.destroy_node() # cmd_vel 노드의 destroy_node() 함수를 호출하여 노드 종료

            if rclpy.ok(): # ROS2가 초기화되어 있으면
                rclpy.shutdown() # ROS2 종료, 내부적으로 모든 노드가 종료되고 ROS2 관련 리소스가 해제됨

            self._is_initialized = False # ROSHealthcareNode 초기화 상태를 False로 설정

            write_log("ROSHealthcareNode shutdown.", self)

        except Exception:
            ErrorHandler().report()

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_follow_node(self): # Follow 노드 객체 반환하는 함수, MPHealthcareROS에서 Follow 노드의 상태나 결과를 확인할 때 호출됨
        return self._follow_node

    def get_navigation_node(self): # Navigation 노드 객체 반환하는 함수, MPHealthcareROS에서 Navigation 노드의 상태나 결과를 확인할 때 호출됨
        return self._navigation_node

    def get_cmd_vel_node(self): # cmd_vel 노드 객체 반환하는 함수, MPHealthcareROS에서 cmd_vel 노드의 상태나 결과를 확인할 때 호출됨
        return self._cmd_vel_node