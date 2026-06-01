# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DeliveryGuideManager.py
# Project Name : HealthcareRobotPyRT
# Description  : Guide and delivery mode manager
# -------------------------------------------------------------------------------------------------------------------- #

from Commons import *

from ROSIntegration.ROSHealthcareNode import CROSHealthcareNode


class CDeliveryGuideManager:
    """
    안내 / 배달 기능 관리자

    역할:
        1. GUIDE 시작/중지 관리
        2. DELIVERY 시작/중지 관리
        3. destination_id를 ROSNavigationNode로 전달
        4. navigation 도착 여부 확인
    """

    MODE_NONE = "NONE"
    MODE_GUIDE = "GUIDE"
    MODE_DELIVERY = "DELIVERY"

    def __init__(self):

        self._ros_node = CROSHealthcareNode() # ROS 통합 노드

        self._mode = self.MODE_NONE # 현재 모드 (NONE, GUIDE, DELIVERY)

        self._destination_id = None # 현재 안내/배달 목적지 ID, None이면 목적지 없음

        self._is_active = False # 현재 안내/배달이 진행 중인지 여부

        write_log("DeliveryGuideManager initialized.", self)

    # ==============================================================================================================
    # Guide
    # ==============================================================================================================

    def start_guide(self, destination_id: str): # 안내 시작 함수
        """
        안내 시작

        예:
            destination_id = "convenience_store"
        """

        self._mode = self.MODE_GUIDE # 모드 GUIDE로 설정
        self._destination_id = destination_id # 목적지 ID 설정
        self._is_active = True # 안내/배달 진행 중 상태로 설정

        success = self._ros_node.go_to_destination(destination_id) # ROSHealthcareNode의 go_to_destination 함수 호출하여 네비게이션 시작

        if success: # 네비게이션 시작 성공 시
            write_log(
                "Guide started | destination=%s"
                % str(destination_id),
                self
            )
        else:
            write_log(
                "Guide start failed | destination=%s"
                % str(destination_id),
                self
            )
            self.reset()

        return success # 네비게이션 시작 성공 여부 반환

    # ==============================================================================================================
    # Delivery
    # ==============================================================================================================

    def start_delivery(self, destination_id: str): # 배달 시작 함수
        """
        배달 시작

        예:
            destination_id = "room_19421"
        """

        self._mode = self.MODE_DELIVERY # 모드 DELIVERY로 설정
        self._destination_id = destination_id # 목적지 ID 설정
        self._is_active = True # 안내/배달 진행 중 상태로 설정

        success = self._ros_node.go_to_destination(destination_id) # ROSHealthcareNode의 go_to_destination 함수 호출하여 네비게이션 시작

        if success:
            write_log(
                "Delivery started | destination=%s"
                % str(destination_id),
                self
            )
        else:
            write_log(
                "Delivery start failed | destination=%s"
                % str(destination_id),
                self
            )
            self.reset()

        return success

    # ==============================================================================================================
    # Process
    # ==============================================================================================================

    def process(self): # 안내/배달 진행 상태 확인 함수, MPHealthcareROS 루프에서 주기적으로 호출되어야 함
        """
        GUIDE / DELIVERY 진행 상태 확인

        MPHealthcareROS 루프에서 주기적으로 호출한다.
        """

        if not self._is_active: # 안내/배달이 진행 중이 아니면
            return None # 상태 확인하지 않고 종료

        self._ros_node.spin_once(timeout_sec=0.0) # ROSHealthcareNode의 spin_once 함수 호출하여 ROS 콜백 처리, timeout_sec=0.0으로 설정하여 즉시 반환

        if self._ros_node.is_arrived(): # 도착했으면
            result = { # 도착 결과
                "mode": self._mode, # 현재 모드 (GUIDE 또는 DELIVERY)
                "destination_id": self._destination_id, # 현재 목적지 ID
                "result": "ARRIVED", # 도착 결과 (ARRIVED)
            }

            write_log(
                "%s arrived | destination=%s"
                % (
                    self._mode,
                    str(self._destination_id)
                ),
                self
            )

            self.reset()

            return result # 도착 결과 반환

        nav_result = self._ros_node.get_navigation_result() # 네비게이션 결과 가져오기, None이면 아직 결과 없음, "SUCCEEDED"이면 도착, 그 외에는 실패 또는 취소 등 다른 결과

        if nav_result is not None and nav_result != "SUCCEEDED": # 네비게이션 결과가 있고, 도착이 아닌 경우 (실패, 취소 등)
            result = { # 네비게이션 결과
                "mode": self._mode, # 현재 모드 (GUIDE 또는 DELIVERY)
                "destination_id": self._destination_id, # 현재 목적지 ID
                "result": nav_result, # 네비게이션 결과 (도착, 실패, 취소 등)
            }

            write_log(
                "%s navigation finished | destination=%s | result=%s"
                % (
                    self._mode,
                    str(self._destination_id),
                    str(nav_result)
                ),
                self
            )

            self.reset()

            return result # 네비게이션 결과 반환

        return { # 네비게이션 진행 중 상태 반환
            "mode": self._mode, # 현재 모드 (GUIDE 또는 DELIVERY)
            "destination_id": self._destination_id, # 현재 목적지 ID
            "result": "RUNNING", # 네비게이션 진행 중 결과 (RUNNING)
            "distance_remaining": self._ros_node.get_distance_remaining(), # ROSHealthcareNode의 get_distance_remaining 함수 호출하여 남은 거리 가져오기
        }

    # ==============================================================================================================
    # Stop / Cancel
    # ==============================================================================================================

    def stop(self): # 안내/배달 중지 함수
        """
        GUIDE / DELIVERY 중지
        """

        if self._is_active:
            self._ros_node.cancel_navigation()

        self.reset()

        write_log("DeliveryGuideManager stopped.", self)

    # ==============================================================================================================
    # Reset
    # ==============================================================================================================

    def reset(self): # 상태 초기화 함수
        self._mode = self.MODE_NONE # 모드 NONE으로 초기화
        self._destination_id = None # 목적지 ID 초기화
        self._is_active = False # 안내/배달 진행 중 상태 False로 초기화

    # ==============================================================================================================
    # State
    # ==============================================================================================================

    def is_active(self): # 안내/배달 진행 중인지 여부 반환하는 함수
        return self._is_active

    def is_guide_mode(self): # 현재 모드가 GUIDE인지 여부 반환하는 함수
        return self._mode == self.MODE_GUIDE

    def is_delivery_mode(self): # 현재 모드가 DELIVERY인지 여부 반환하는 함수
        return self._mode == self.MODE_DELIVERY

    def get_mode(self): # 현재 모드 반환하는 함수
        return self._mode

    def get_destination_id(self): # 현재 목적지 ID 반환하는 함수
        return self._destination_id

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_ros_node(self): # ROSHealthcareNode 객체 반환하는 함수
        return self._ros_node