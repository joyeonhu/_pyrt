# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ROSNavigationNode.py
# Project Name : HealthcareRobotPyRT
# Description  : ROS2 Nav2 NavigateToPose action client
# -------------------------------------------------------------------------------------------------------------------- #

import time

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

from Commons import *
from HealthcareRobot.DestinationManager import CDestinationManager


class CROSNavigationNode(Node):
    """
    Nav2 목적지 이동 노드

    역할:
        1. destination_id를 goal pose로 변환
        2. Nav2 NavigateToPose action server에 goal 전송
        3. 이동 성공/실패/진행 상태 관리
    """

    def __init__(
            self,
            node_name: str = "healthcare_navigation_node",
            action_name: str = "navigate_to_pose"
    ):
        super().__init__(node_name)

        self._action_name = action_name

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            self._action_name
        )

        self._destination_manager = CDestinationManager()

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

        write_log("ROSNavigationNode initialized.", self)

    # ==============================================================================================================
    # Goal Message
    # ==============================================================================================================

    def make_goal_msg(
            self,
            x: float,
            y: float,
            yaw: float
    ):
        """
        x, y, yaw를 Nav2 NavigateToPose goal message로 변환
        """

        quat = self._destination_manager.yaw_to_quaternion(yaw)

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = float(quat["x"])
        goal_msg.pose.pose.orientation.y = float(quat["y"])
        goal_msg.pose.pose.orientation.z = float(quat["z"])
        goal_msg.pose.pose.orientation.w = float(quat["w"])

        return goal_msg

    # ==============================================================================================================
    # Go To Pose
    # ==============================================================================================================

    def go_to_pose(
            self,
            x: float,
            y: float,
            yaw: float,
            destination_id: str = None
    ):
        """
        특정 좌표로 이동 요청
        """

        if self._is_navigating:
            self.cancel_navigation()
            time.sleep(0.1)

        write_log(
            "Waiting for Nav2 action server: %s" % self._action_name,
            self
        )

        server_ready = self._action_client.wait_for_server(
            timeout_sec=5.0
        )

        if not server_ready:
            write_log(
                "Nav2 action server not available: %s" % self._action_name,
                self
            )

            self._is_navigating = False
            self._is_arrived = False
            self._last_result = "SERVER_NOT_AVAILABLE"

            return False

        goal_msg = self.make_goal_msg(
            x,
            y,
            yaw
        )

        self._goal_handle = None
        self._result_future = None

        self._current_destination_id = destination_id
        self._is_navigating = True
        self._is_arrived = False
        self._last_result = None
        self._start_time = time.time()

        self._last_feedback = None
        self._distance_remaining = None
        self._last_feedback_log_time = 0.0

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

        write_log(
            "Navigation goal sent | destination=%s | x=%.2f, y=%.2f, yaw=%.2f"
            % (
                str(destination_id),
                float(x),
                float(y),
                float(yaw)
            ),
            self
        )

        return True

    # ==============================================================================================================
    # Go To Destination
    # ==============================================================================================================

    def go_to_destination(self, destination_id: str):
        """
        목적지 ID를 받아 Nav2 goal 전송
        """

        goal_pose = self._destination_manager.get_goal_pose(
            destination_id
        )

        if goal_pose is None:
            write_log(
                "Navigation failed: unknown destination_id=%s"
                % str(destination_id),
                self
            )

            self._is_navigating = False
            self._is_arrived = False
            self._last_result = "UNKNOWN_DESTINATION"

            return False

        return self.go_to_pose(
            goal_pose["x"],
            goal_pose["y"],
            goal_pose["yaw"],
            destination_id
        )

    # ==============================================================================================================
    # Callback
    # ==============================================================================================================

    def goal_response_callback(self, future):
        """
        Nav2가 goal을 받았는지 확인하는 callback
        """

        try:
            self._goal_handle = future.result()

            if self._goal_handle is None:
                self._is_navigating = False
                self._is_arrived = False
                self._last_result = "GOAL_HANDLE_NONE"

                write_log("Navigation goal handle is None.", self)
                return

            if not self._goal_handle.accepted:
                self._is_navigating = False
                self._is_arrived = False
                self._last_result = "REJECTED"

                write_log("Navigation goal rejected.", self)
                return

            write_log("Navigation goal accepted.", self)

            self._result_future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(
                self.result_callback
            )

        except Exception:
            self._is_navigating = False
            self._is_arrived = False
            self._last_result = "GOAL_RESPONSE_ERROR"

            ErrorHandler().report()

    def feedback_callback(self, feedback_msg):
        """
        Nav2 이동 중 feedback callback
        """

        try:
            feedback = feedback_msg.feedback

            self._last_feedback = feedback
            self._distance_remaining = feedback.distance_remaining

            curr_time = time.time()

            if curr_time - self._last_feedback_log_time >= 1.0:
                write_log(
                    "Navigation feedback | destination=%s | distance_remaining=%.2f m"
                    % (
                        str(self._current_destination_id),
                        float(self._distance_remaining),
                    ),
                    self
                )

                self._last_feedback_log_time = curr_time

        except Exception:
            ErrorHandler().report()

    def result_callback(self, future):
        """
        Nav2 이동 완료 callback
        """

        try:
            result_response = future.result()
            status = result_response.status

            self._is_navigating = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                self._is_arrived = True
                self._last_result = "SUCCEEDED"

                write_log(
                    "Navigation succeeded -> %s"
                    % str(self._current_destination_id),
                    self
                )

            elif status == GoalStatus.STATUS_CANCELED:
                self._is_arrived = False
                self._last_result = "CANCELED"

                write_log(
                    "Navigation canceled -> %s"
                    % str(self._current_destination_id),
                    self
                )

            elif status == GoalStatus.STATUS_ABORTED:
                self._is_arrived = False
                self._last_result = "ABORTED"

                write_log(
                    "Navigation aborted -> %s"
                    % str(self._current_destination_id),
                    self
                )

            else:
                self._is_arrived = False
                self._last_result = "FAILED_%s" % str(status)

                write_log(
                    "Navigation failed -> %s | status=%s"
                    % (
                        str(self._current_destination_id),
                        str(status)
                    ),
                    self
                )

        except Exception:
            self._is_navigating = False
            self._is_arrived = False
            self._last_result = "RESULT_ERROR"

            ErrorHandler().report()

    # ==============================================================================================================
    # Cancel
    # ==============================================================================================================

    def cancel_navigation(self):
        """
        현재 navigation goal 취소
        """

        if self._goal_handle is None:
            self._is_navigating = False
            self._is_arrived = False
            self._last_result = "NO_GOAL"

            return False

        try:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                self.cancel_done_callback
            )

            write_log("Navigation cancel requested.", self)

            return True

        except Exception:
            self._is_navigating = False
            self._is_arrived = False
            self._last_result = "CANCEL_REQUEST_ERROR"

            ErrorHandler().report()

            return False

    def cancel_done_callback(self, future):
        """
        Nav2 goal 취소 요청 결과 callback
        """

        try:
            cancel_response = future.result()

            if len(cancel_response.goals_canceling) > 0:
                self._is_navigating = False
                self._is_arrived = False
                self._last_result = "CANCELED"

                write_log("Navigation cancel accepted.", self)

            else:
                write_log(
                    "Navigation cancel failed or goal already finished.",
                    self
                )

        except Exception:
            self._is_navigating = False
            self._is_arrived = False
            self._last_result = "CANCEL_ERROR"

            ErrorHandler().report()

    # ==============================================================================================================
    # Reset
    # ==============================================================================================================

    def reset(self):
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

    def is_navigating(self):
        return self._is_navigating

    def is_arrived(self):
        return self._is_arrived

    def get_current_destination_id(self):
        return self._current_destination_id

    def get_last_result(self):
        return self._last_result

    def get_distance_remaining(self):
        return self._distance_remaining

    def get_navigation_time(self):
        if not self._is_navigating:
            return 0.0

        return time.time() - self._start_time