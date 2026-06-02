# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ROSCmdVelNode.py
# Project Name : HealthcareRobotPyRT
# Description  : ROS2 cmd_vel publisher node
# -------------------------------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from Commons import *
from Control.VelocityLimiter import CVelocityLimiter


class CROSCmdVelNode(Node):
    """
    ROS2 cmd_vel publisher node

    역할:
        1. /cmd_vel 토픽 생성
        2. linear.x / angular.z 제한
        3. 제한된 속도값 publish
        4. 필요 시 거리 평가용 토픽 publish
    """

    def __init__(
            self,
            node_name: str = "healthcare_cmd_vel_node", # 노드 이름
            cmd_vel_topic: str = "/cmd_vel", # cmd_vel 토픽 이름
            distance_topic: str = "/evaluation/target_distance", # 평가용 target distance 토픽 이름
    ):
        super().__init__(node_name) # 부모 클래스인 ROS2 Node 초기화

        qos = QoSProfile( # QoS 설정, 토픽 통신 품질 설정 객체 생성
            depth=10, # 메시지 큐를 최대 10개까지 저장
            reliability=ReliabilityPolicy.RELIABLE # 메시지를 안정적으로 전달, 즉, 메시지 손실이 없도록 보장, 도착할 때까지 재전송
        )

        self._cmd_vel_topic = cmd_vel_topic
        self._distance_topic = distance_topic

        self._cmd_vel_pub = self.create_publisher( # /cmd_vel 토픽 퍼블리셔 생성
            Twist, # 메시지 타입은 geometry_msgs/Twist
            self._cmd_vel_topic, # 토픽 이름은 cmd_vel_topic 매개변수로 설정
            qos # QoS 설정 적용
        )

        self._distance_pub = self.create_publisher( # 평가용 target distance 토픽 퍼블리셔 생성
            Float32, # 메시지 타입은 std_msgs/Float32
            self._distance_topic, # 토픽 이름은 distance_topic 매개변수로 설정
            qos # QoS 설정 적용
        )

        self._vel_limiter = CVelocityLimiter() # cmd_vel 제한 객체 생성

        self._last_linear_x = 0.0 # 마지막으로 publish한 linear.x 값, 초기값은 0.0
        self._last_angular_z = 0.0 # 마지막으로 publish한 angular.z 값, 초기값은 0.0

        write_log(
            "ROSCmdVelNode started. topic=%s" % self._cmd_vel_topic,
            self
        )

    # ==============================================================================================================
    # Publish cmd_vel
    # ==============================================================================================================

    def publish_cmd_vel(
            self,
            linear_x: float,
            angular_z: float
    ):
        """
        /cmd_vel publish
        """

        linear_x, angular_z = self._vel_limiter.limit_cmd_vel(
            linear_x,
            angular_z
        ) # 입력된 linear.x와 angular.z 값을 제한된 범위로 조정

        self._last_linear_x = linear_x # 제한된 linear.x 값을 마지막으로 publish한 값으로 저장
        self._last_angular_z = angular_z # 제한된 angular.z 값을 마지막으로 publish한 값으로 저장

        msg = Twist() # ROS2 속도 메시지 객체 생성
        msg.linear.x = float(linear_x) # linear.x 필드에 제한된 linear_x 값 설정
        msg.angular.z = float(angular_z) # angular.z 필드에 제한된 angular_z 값 설정

        self._cmd_vel_pub.publish(msg) # /cmd_vel 토픽에 메시지 publish

        self.get_logger().info( # ROS2 logger로 로그 출력
            "cmd_vel -> linear.x=%.3f, angular.z=%.3f"
            % (
                msg.linear.x,
                msg.angular.z
            )
        )

    # ==============================================================================================================
    # Stop
    # ==============================================================================================================

    def stop_robot(self):
        """
        로봇 정지 명령 publish
        """

        self._last_linear_x = 0.0 # 마지막으로 publish한 linear.x 값을 0.0으로 설정하여 정지 명령을 나타냄
        self._last_angular_z = 0.0 # 마지막으로 publish한 angular.z 값을 0.0으로 설정하여 정지 명령을 나타냄

        msg = Twist() # ROS2 속도 메시지 객체 생성
        msg.linear.x = 0.0 # linear.x 필드에 0.0 설정하여 정지 명령을 나타냄
        msg.angular.z = 0.0 # angular.z 필드에 0.0 설정하여 정지 명령을 나타냄

        self._cmd_vel_pub.publish(msg) # /cmd_vel 토픽에 정지 명령 메시지 publish

        self.get_logger().info("cmd_vel stop published.") # ROS2 logger로 정지 명령이 publish되었다는 로그 출력

    # ==============================================================================================================
    # Publish Distance
    # ==============================================================================================================

    def publish_target_distance(self, distance_cm: float):
        """
        평가용 target distance publish
        """
        # distance_cm : 환자와의 수평 거리 (cm 단위)
        msg = Float32() # ROS2 Float32 메시지 객체 생성
        msg.data = float(distance_cm) # 메시지의 data 필드에 distance_cm 값을 float으로 설정

        self._distance_pub.publish(msg) # /evaluation/target_distance 토픽에 메시지 publish

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_last_cmd_vel(self): # 마지막으로 publish한 cmd_vel 값을 반환하는 함수
        return self._last_linear_x, self._last_angular_z

    def get_cmd_vel_topic(self): # cmd_vel 토픽 이름 반환
        return self._cmd_vel_topic

    def get_distance_topic(self): # 평가용 target distance 토픽 이름 반환
        return self._distance_topic