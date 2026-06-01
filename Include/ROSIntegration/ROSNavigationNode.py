# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ROSNavigationNode.py
# Project Name : HealthcareRobotPyRT
# Description  : ROS2 Nav2 NavigateToPose action client
# -------------------------------------------------------------------------------------------------------------------- #

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from Commons import *
from HealthcareRobot.DestinationManager import CDestinationManager


class CROSNavigationNode(Node):
    """
    Nav2 목적지 이동 노드

    역할:
        1. destination_id를 goal pose로 변환
        2. Nav2 NavigateToPose action server에 goal 전송
        3. 이동 성공/실패/진행 상태 관리

    사용 예:
        nav_node = CROSNavigationNode()
        nav_node.go_to_destination("convenience_store")
    """

    def __init__(
            self,
            node_name: str = "healthcare_navigation_node", # 노드 이름
            action_name: str = "navigate_to_pose" # action server 이름, Nav2 기본값은 "navigate_to_pose"
    ):
        super().__init__(node_name) # 부모 클래스인 ROS2 Node 초기화 즉, ROS 노드 이름을 healthcare_navigation_node로 등록

        self._action_name = action_name # action server 이름
        self._action_client = ActionClient( # Nav2에 goal을 보내기 위한 action client 생성
            self,
            NavigateToPose, # action 타입은 nav2_msgs/action/NavigateToPose
            self._action_name
        )

        self._destination_manager = CDestinationManager() # destination_id를 goal pose로 변환하는 객체 생성

        # Nav2에 보낸 goal 상태를 저장할 변수
        self._goal_handle = None # 목적지 이동 요청 자체
        self._result_future = None # 요청에 결과를 나중에 받기 위한 예약표

        self._current_destination_id = None # 현재 이동 중인 목적지 이름
        self._is_navigating = False # 현재 이동 중인지 여부
        self._is_arrived = False # 현재 목적지에 도착했는지 여부
        self._last_result = None # 마지막 navigation 결과 상태, "SUCCEEDED", "FAILED", "CANCELED", "REJECTED" 등으로 저장
        self._start_time = 0.0 # 이동 시작 시간 저장

        self._last_feedback = None # Nav2가 이동 중에 보내주는 feedback 메시지 저장, feedback에는 distance_remaining(목적지까지 남은 거리) 등의 정보가 포함되어 있음
        self._distance_remaining = None # 목적지까지 남은 거리 저장, feedback에서 업데이트됨

        self._last_feedback_log_time = 0.0 # 마지막으로 feedback 로그를 출력한 시간, 1초마다 feedback 로그를 출력하기 위해 사용

        write_log("ROSNavigationNode initialized.", self)

    # ==============================================================================================================
    # Goal Message
    # ==============================================================================================================

    def make_goal_msg( # Nav2 NavigateToPose action의 goal 메시지 객체 생성하는 함수, x, y, yaw 값을 Nav2가 이해할 수 있는 goal 메시지로 변환
            self,
            x: float,
            y: float,
            yaw: float
    ):
        """
        x, y, yaw를 Nav2 NavigateToPose goal message로 변환
        """

        quat = self._destination_manager.yaw_to_quaternion(yaw) # yaw를 quaternion으로 변환, ROS2 메시지 형식 자체가 quaternion이므로

        goal_msg = NavigateToPose.Goal() # NavigateToPose action의 goal 메시지 객체 생성

        # ROS Pose : 위치 + 방향
        # 구조
        # pose.position.x
        # pose.position.y
        # pose.position.z - 로봇은 날아다니지 않으므로 항상 0으로 설정
        # pose.orientation.x : 앞뒤로 기울어지는 회전
        # pose.orientation.y : 좌우로 기울어지는 회전
        # pose.orientation.z : z축 회전 성분, 즉 바닥에서 좌우로 도는 회전
        # pose.orientation.w : 회전량의 기준값 같은 역할, quaternion에서 회전을 표현할 때 x, y, z와 함께 사용되어 회전 방향과 크기를 나타냄
        # position.x, y는 위치 즉, 맵 좌표
        # yaw : 회전 방향 각도

        goal_msg.pose = PoseStamped() # goal 안에 들어갈 pose 메시지 객체 생성
        goal_msg.pose.header.frame_id = "map" # 좌표 기준이 map 좌표계라는 뜻
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg() # 현재 ROS 시간을 메시지에 넣음

        goal_msg.pose.pose.position.x = float(x) # 목적지 위치 x 좌표 설정
        goal_msg.pose.pose.position.y = float(y) # 목적지 위치 y 좌표 설정
        goal_msg.pose.pose.position.z = 0.0 # 로봇은 날아다니지 않으므로 z 좌표는 항상 0으로 설정

        goal_msg.pose.pose.orientation.x = float(quat["x"]) # 목적지 방향의 quaternion x 성분 설정
        goal_msg.pose.pose.orientation.y = float(quat["y"]) # 목적지 방향의 quaternion y 성분 설정
        goal_msg.pose.pose.orientation.z = float(quat["z"]) # 목적지 방향의 quaternion z 성분 설정
        goal_msg.pose.pose.orientation.w = float(quat["w"]) # 목적지 방향의 quaternion w 성분 설정

        return goal_msg # 완성된 Nav2 goal 메시지 반환

    # ==============================================================================================================
    # Go To Pose
    # ==============================================================================================================

    def go_to_pose( # 실제 좌표를 받아 Nav2에 이동 요청하는 함수
            self,
            x: float,
            y: float,
            yaw: float,
            destination_id: str = None
    ):
        """
        특정 좌표로 이동 요청
        """

        write_log(
            "Waiting for Nav2 action server: %s" % self._action_name,
            self
        )

        server_ready = self._action_client.wait_for_server(timeout_sec=5.0) # Nav2 action server가 준비될 때까지 최대 5초간 대기

        if not server_ready: # Nav2 action server가 준비되지 않은 경우
            write_log(
                "Nav2 action server not available: %s" % self._action_name,
                self
            ) # Nav2 action server가 준비되지 않았다는 로그 출력
            return False # 함수 종료, 이동 요청 실패

        goal_msg = self.make_goal_msg(x, y, yaw) # x, y, yaw 값을 Nav2 goal 메시지로 변환

        self._current_destination_id = destination_id # 현재 이동하려는 목적지 ID 저장, 로그나 결과 처리할 때 사용
        self._is_navigating = True # 이동 요청이 시작되었으므로 현재 navigation 중임을 나타내는 변수 설정
        self._is_arrived = False # 이동이 완료되지 않았으므로 도착 여부는 False로 설정
        self._last_result = None # 마지막 결과 상태 초기화, 아직 결과를 받지 않았으므로 None으로 설정
        self._start_time = time.time() # 이동 요청 시작 시간 기록, 나중에 navigation 경과 시간 계산할 때 사용

        send_goal_future = self._action_client.send_goal_async( # Nav2에 goal을 비동기로 전송
            goal_msg,
            feedback_callback=self.feedback_callback # Nav2가 이동 중에 보내주는 feedback 메시지를 받을 때 호출되는 콜백 함수 지정, 이동 중간중간 보내주는 진행 상황
        )

        send_goal_future.add_done_callback(self.goal_response_callback) # Nav2가 goal을 받았는지 결과가 오면 goal_response_callback()을 실행하도록 등록, goal 요청 자체를 받았는지 확인하는 콜백

        write_log(
            "Navigation goal sent | destination=%s | x=%.2f, y=%.2f, yaw=%.2f"
            % (
                str(destination_id),
                x,
                y,
                yaw
            ),
            self
        ) # Nav2에 이동 요청을 보냈다는 로그 출력, 목적지 ID와 좌표 정보 포함

        return True # 이동 요청이 성공적으로 전송되었음을 나타내는 True 반환

    # ==============================================================================================================
    # Go To Destination
    # ==============================================================================================================

    def go_to_destination(self, destination_id: str): # 목적지 ID로 이동 요청하는 함수
        """
        목적지 ID를 받아 Nav2 goal 전송

        예:
            "convenience_store"
            "room_19421"
            "nurse_station"
            "home"
        """

        goal_pose = self._destination_manager.get_goal_pose(destination_id) # destination_id를 받아서 해당 목적지의 좌표 정보를 반환하는 함수 호출, destination_id가 등록되어 있지 않으면 None 반환

        if goal_pose is None: # destination_id가 등록되어 있지 않은 경우
            write_log(
                "Navigation failed: unknown destination_id=%s"
                % str(destination_id),
                self
            )
            return False # 함수 종료, 이동 요청 실패

        return self.go_to_pose( # 실제 좌표로 이동 요청하는 함수 호출, destination_id에 해당하는 좌표 정보(goal_pose)를 Nav2 goal 메시지로 변환하여 이동 요청
            goal_pose["x"],
            goal_pose["y"],
            goal_pose["yaw"],
            destination_id
        ) # destination_id로 이동 요청하는 함수의 반환값을 그대로 반환, 이동 요청 성공 여부 반환

    # ==============================================================================================================
    # Callback
    # ==============================================================================================================

    def goal_response_callback(self, future):
        """
        Nav2가 goal을 받았는지 확인하는 callback
        """
        # future : Nav2에 goal을 보냈을 때 나중에 결과가 오면 실행되는 예약표, goal이 받아졌는지 여부와 goal handle 객체를 포함
        # future은 ROS가 넣어줌, 내부적으로 callback(future) 이런 느낌

        self._goal_handle = future.result() # Nav2가 goal을 받았는지 여부와 goal handle 객체를 future에서 꺼내서 _goal_handle 변수에 저장, goal 응답 객체를 가져옴

        if not self._goal_handle.accepted: # Nav2가 goal을 받지 않은 경우, goal 요청이 거부된 경우
            write_log("Navigation goal rejected.", self) # Nav2가 goal을 받지 않았다는 로그 출력

            self._is_navigating = False # 이동 요청이 거부되었으므로 현재 navigation 중이 아님을 나타내는 변수 설정
            self._is_arrived = False # 이동 요청이 거부되었으므로 도착 여부는 False로 설정
            self._last_result = "REJECTED" # 이동 요청이 거부되었으므로 마지막 결과 상태를 "REJECTED"로 설정
            return # 함수 종료, 이동 요청 실패

        write_log("Navigation goal accepted.", self) # Nav2가 goal을 받았다는 로그 출력

        self._result_future = self._goal_handle.get_result_async() # 이동 완료 결과를 비동기로 기다림
        self._result_future.add_done_callback(self.result_callback) # 이동이 끝나면 result_callback()을 실행하도록 등록

    def feedback_callback(self, feedback_msg): # Nav2가 이동 중에 보내주는 feedback 메시지를 받을 때 호출되는 콜백 함수
        """
        Nav2 이동 중 feedback callback
        """

        feedback = feedback_msg.feedback # Nav2가 이동 중에 보내주는 feedback 메시지에서 feedback 정보 꺼냄, feedback에는 distance_remaining(목적지까지 남은 거리) 등의 정보가 포함되어 있음

        self._last_feedback = feedback # Nav2가 이동 중에 보내주는 feedback 메시지 저장, 나중에 필요하면 이 정보를 사용할 수 있도록 저장
        self._distance_remaining = feedback.distance_remaining # Nav2가 이동 중에 보내주는 feedback 메시지에서 목적지까지 남은 거리 정보 꺼냄

        curr_time = time.time()

        # 1초마다만 로그 출력
        if curr_time - self._last_feedback_log_time >= 1.0:

            write_log(
                "Navigation feedback | destination=%s | distance_remaining=%.2f m"
                % (
                    str(self._current_destination_id),
                    self._distance_remaining,
                ),
                self
            ) # Nav2가 이동 중에 보내주는 feedback 메시지에서 목적지까지 남은 거리 정보를 로그로 출력, 현재 이동 중인 목적지 ID와 남은 거리 포함

            self._last_feedback_log_time = curr_time

    def result_callback(self, future): # Nav2가 이동 완료 후 결과 메시지를 받을 때 호출되는 콜백 함수
        """
        Nav2 이동 완료 callback
        """

        _ = future.result().result # Nav2가 이동 완료 후 결과 메시지에서 result 정보 꺼냄, result에는 navigation이 성공했는지 여부와 최종 위치 정보 등이 포함되어 있음
        status = future.result().status # Nav2가 이동 완료 후 결과 메시지에서 status 정보 꺼냄, status 값은 GoalStatus 기준, 보통 4 = SUCCEEDED

        self._is_navigating = False # 이동이 완료되었으므로 현재 navigation 중이 아님을 나타내는 변수 설정

        # status 값은 GoalStatus 기준.
        # 보통 4 = SUCCEEDED.
        if status == 4: # Nav2가 이동 완료 후 결과 메시지에서 status 값이 4(SUCCEEDED)인 경우, 즉 이동이 성공한 경우
            self._is_arrived = True # 이동이 성공했으므로 도착 여부를 True로 설정
            self._last_result = "SUCCEEDED" # 이동이 성공했으므로 마지막 결과 상태를 "SUCCEEDED"로 설정

            write_log(
                "Navigation succeeded -> %s"
                % str(self._current_destination_id),
                self
            ) # 이동 성공 로그 출력, 현재 이동 중이었던 목적지 ID 포함

        else: # Nav2가 이동 완료 후 결과 메시지에서 status 값이 4(SUCCEEDED)가 아닌 경우, 즉 이동이 실패한 경우
            self._is_arrived = False # 이동이 실패했으므로 도착 여부는 False로 설정
            self._last_result = "FAILED_%s" % str(status) # 이동이 실패했으므로 마지막 결과 상태를 "FAILED_"와 status 값을 조합한 문자열로 설정, 실패 원인을 status 값으로 기록

            write_log(
                "Navigation failed -> %s | status=%s"
                % (
                    str(self._current_destination_id),
                    str(status)
                ),
                self
            )

    # ==============================================================================================================
    # Cancel
    # ==============================================================================================================
    def cancel_navigation(self):
        """
        현재 navigation goal 취소
        """

        if self._goal_handle is None: # 현재 goal handle이 없는 경우, 즉 Nav2에 이동 요청을 보낸 적이 없는 경우
            self._is_navigating = False # 이동 요청 자체가 없으므로 현재 navigation 중이 아님을 나타내는 변수 설정
            self._is_arrived = False # 이동 요청 자체가 없으므로 도착 여부는 False로 설정
            self._last_result = "NO_GOAL" # 이동 요청 자체가 없으므로 마지막 결과 상태를 "NO_GOAL"로 설정
            return False # 함수 종료, 취소 요청 실패

        cancel_future = self._goal_handle.cancel_goal_async() # Nav2에 현재 goal 취소 요청을 비동기로 보냄, 나중에 취소 결과가 오면 cancel_done_callback()을 실행하도록 등록
        cancel_future.add_done_callback(self.cancel_done_callback) # Nav2에 현재 goal 취소 요청 결과가 오면 cancel_done_callback()을 실행하도록 등록

        write_log("Navigation cancel requested.", self) # Nav2에 현재 goal 취소 요청을 보냈다는 로그 출력

        return True # Nav2에 현재 goal 취소 요청이 성공적으로 전송되었음을 나타내는 True 반환

    def cancel_done_callback(self, future):
        """
        Nav2 goal 취소 요청 결과 callback
        """

        try:
            cancel_response = future.result() # Nav2에 현재 goal 취소 요청 결과를 future에서 꺼냄, cancel_response에는 goals_canceling(취소된 goal 목록)과 goals_canceling(취소되지 않은 goal 목록) 정보가 포함되어 있음

            if len(cancel_response.goals_canceling) > 0: # Nav2에 현재 goal 취소 요청 결과에서 goals_canceling 목록이 0보다 큰 경우, 즉 취소된 goal이 있는 경우
                self._is_navigating = False # goal이 취소되었으므로 현재 navigation 중이 아님을 나타내는 변수 설정
                self._is_arrived = False # goal이 취소되었으므로 도착 여부는 False로 설정
                self._last_result = "CANCELED" # goal이 취소되었으므로 마지막 결과 상태를 "CANCELED"로 설정

                write_log("Navigation cancel accepted.", self) # Nav2에 현재 goal 취소 요청이 받아들여졌다는 로그 출력

            else: # Nav2에 현재 goal 취소 요청 결과에서 goals_canceling 목록이 0인 경우, 즉 취소된 goal이 없는 경우
                write_log("Navigation cancel failed or goal already finished.", self) # Nav2에 현재 goal 취소 요청이 실패했거나 이미 goal이 완료된 경우라는 로그 출력

        except Exception: # Nav2에 현재 goal 취소 요청 결과를 처리하는 과정에서 예외가 발생한 경우
            self._is_navigating = False # 취소 요청 처리 중 예외가 발생했으므로 현재 navigation 중이 아님을 나타내는 변수 설정
            self._is_arrived = False # 취소 요청 처리 중 예외가 발생했으므로 도착 여부는 False로 설정
            self._last_result = "CANCEL_ERROR" # 취소 요청 처리 중 예외가 발생했으므로 마지막 결과 상태를 "CANCEL_ERROR"로 설정

            ErrorHandler().report() # 예외가 발생한 경우 에러 핸들러를 통해 예외 보고, 에러 핸들러는 예외 정보를 로그로 출력하거나 외부 시스템에 알리는 등의 역할을 할 수 있음


    # ==============================================================================================================
    # Reset
    # ==============================================================================================================

    def reset(self): # navigation 상태 초기화, 주로 다음 navigation을 위해 상태를 초기화할 때 사용
        self._goal_handle = None
        self._result_future = None

        self._current_destination_id = None
        self._is_navigating = False
        self._is_arrived = False
        self._last_result = None
        self._start_time = 0.0
        self._last_feedback = None
        self._distance_remaining = None
        self._last_feedback_log_time = 0.0

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def is_navigating(self): # 현재 navigation 중인지 여부 반환
        return self._is_navigating

    def is_arrived(self): # 현재 목적지에 도착했는지 여부 반환
        return self._is_arrived

    def get_current_destination_id(self): # 현재 이동 중인 목적지 ID 반환
        return self._current_destination_id

    def get_last_result(self): # 마지막 navigation 결과 상태 반환
        return self._last_result

    def get_distance_remaining(self): # 남은 거리 반환
        return self._distance_remaining

    def get_navigation_time(self): # navigation 시작 후 경과 시간 반환, navigation 중이 아니면 0.0 반환
        if not self._is_navigating:
            return 0.0

        return time.time() - self._start_time # navigation 시작 후 경과 시간 계산하여 반환