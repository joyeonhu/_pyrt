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
from Control.CmdVelController import CCmdVelController


class CROSCmdVelNode(Node):
    """
    ROS2 cmd_vel publisher node

    역할:
        1. /cmd_vel 토픽 생성
        2. linear.x / angular.z publish
        3. 필요 시 거리 평가용 토픽 publish
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

        self._cmd_vel_controller = CCmdVelController() # cmd_vel 제어 객체 생성

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

        self._cmd_vel_controller.set_cmd_vel(
            linear_x,
            angular_z
        )

        limited_linear_x, limited_angular_z = self._cmd_vel_controller.get_cmd_vel() # 제한된 cmd_vel 값 가져오기

        msg = Twist() # ROS2 속도 메시지 객체 생성
        msg.linear.x = float(limited_linear_x) # linear.x 필드에 제한된 linear_x 값 설정
        msg.angular.z = float(limited_angular_z) # angular.z 필드에 제한된 angular_z 값 설정

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

        self._cmd_vel_controller.stop()

        linear_x, angular_z = self._cmd_vel_controller.get_cmd_vel()

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        self._cmd_vel_pub.publish(msg)

        self.get_logger().info("cmd_vel stop published.")

    # ==============================================================================================================
    # Publish Distance
    # ==============================================================================================================

    def publish_target_distance(self, distance_cm: float):
        """
        평가용 target distance publish
        """

        msg = Float32()
        msg.data = float(distance_cm)

        self._distance_pub.publish(msg)

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_cmd_vel_topic(self): # cmd_vel 토픽 이름 반환
        return self._cmd_vel_topic

    def get_distance_topic(self): # 평가용 target distance 토픽 이름 반환
        return self._distance_topic