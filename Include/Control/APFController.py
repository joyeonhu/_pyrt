# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : APFController.py
# Project Name : HealthcareRobotPyRT
# Description  : APF-based patient following controller
# -------------------------------------------------------------------------------------------------------------------- #

import math

from Commons import *
from Control.VelocityLimiter import CVelocityLimiter


class CAPFController:
    """
    APF 기반 환자 추적 제어기

    입력:
        - 환자의 화면 중심 x좌표
        - 실제 거리(m)

    출력:
        - linear.x 앞뒤 이동 속도
        - angular.z 회전 속도
    """

    def __init__(
            self,
            frame_width: int = 640, # 가로 픽셀 수
            camera_fov_deg: float = 69.4, # 수평 시야각
            stop_distance_m: float = 1.0, # 환자와의 목표 유지
            kp_linear: float = 3.0, # 직진 속도 gain, 값이 크면 멀리 있으면 빨리 접근, 값이 작으면 천천히 접근
            kp_angular: float = 3.0, # 회전 속도 gain, 값이 크면 환자가 살짝만 옆에 있어도 빠르게 방향 수정, 값이 작으면 천천히 방향 수정
    ):

        # 화면 가로 픽셀 수
        self._frame_width = frame_width

        # 카메라 시야각
        self._camera_fov_deg = camera_fov_deg
        self._camera_fov_rad = math.radians(camera_fov_deg)

        # 목표 유지 거리
        self._stop_distance_m = stop_distance_m

        # Gain
        self._kp_linear = kp_linear
        self._kp_angular = kp_angular

        # 속도 제한기
        self._vel_limiter = CVelocityLimiter()

    # ==============================================================================================================
    # Theta
    # ==============================================================================================================

    def calculate_theta(self, center_x: int) -> float: # 환자가 화면 중심에서 얼마나 벗어났는지 각도로 변환
        """
        화면 중심 기준 환자의 좌우 각도 계산
        """

        # 중심 대비 상대 위치 비율 (-1.0 ~ +1.0)
        rel = (
            (center_x - self._frame_width / 2)
            / (self._frame_width / 2)
        )

        # 실제 각도로 변환
        theta_rad = rel * (self._camera_fov_rad / 2)

        return theta_rad

    # ==============================================================================================================
    # Vector
    # ==============================================================================================================

    def calculate_vector( # 환자의 위치를 거리와 각도로부터 x, y 벡터로 변환
            self,
            center_x: int,
            real_distance_m: float
    ):
        """
        거리 벡터 계산
        """

        theta_rad = self.calculate_theta(center_x)

        dx = math.cos(theta_rad) * real_distance_m # 앞뒤
        dy = math.sin(theta_rad) * real_distance_m # 좌우

        dx_1m = math.cos(theta_rad) * 1.0
        dy_1m = math.sin(theta_rad) * 1.0

        return dx, dy, dx_1m, dy_1m

    # ==============================================================================================================
    # APF
    # ==============================================================================================================

    def calculate_cmd_vel( # APF 기반으로 linear.x와 angular.z 계산
            self,
            center_x: int,
            real_distance_m: float
    ):
        """
        APF 기반 cmd_vel 계산
        """

        # ----------------------------------------------------------------------------------------------------------
        # Vector
        # ----------------------------------------------------------------------------------------------------------

        dx, dy, dx_1m, dy_1m = self.calculate_vector(
            center_x,
            real_distance_m
        )

        # ----------------------------------------------------------------------------------------------------------
        # Distance
        # ----------------------------------------------------------------------------------------------------------

        dist = math.hypot(dx, dy)

        # 목표 거리와의 차이
        delta = dist - self._stop_distance_m

        # ----------------------------------------------------------------------------------------------------------
        # Linear Velocity
        # ----------------------------------------------------------------------------------------------------------

        linear_x = self._kp_linear * delta

        # ----------------------------------------------------------------------------------------------------------
        # Angular Velocity
        # ----------------------------------------------------------------------------------------------------------

        theta_rad = self.calculate_theta(center_x)

        angular_z = -1.0 * self._kp_angular * theta_rad # 카메라 좌표계랑 ROS 회전 방향이 서로 반대여서?

        theta_abs = abs(angular_z)

        # ----------------------------------------------------------------------------------------------------------
        # Stop Condition
        # ----------------------------------------------------------------------------------------------------------

        # 충분히 가까우면서 방향도 거의 맞으면 정지
        if delta <= 0 and theta_abs <= 0.3:
            linear_x = 0.0
            angular_z = 0.0

        # ----------------------------------------------------------------------------------------------------------
        # Velocity Limit
        # ----------------------------------------------------------------------------------------------------------

        linear_x, angular_z = self._vel_limiter.limit_cmd_vel(
            linear_x,
            angular_z
        )

        return linear_x, angular_z

    # ==============================================================================================================
    # Setter
    # ==============================================================================================================

    def set_stop_distance(self, stop_distance_m: float): # 환자와의 목표 유지 거리 설정
        self._stop_distance_m = stop_distance_m

    def set_linear_gain(self, kp_linear: float): # 직진 속도 gain 설정
        self._kp_linear = kp_linear

    def set_angular_gain(self, kp_angular: float): # 회전 속도 gain 설정
        self._kp_angular = kp_angular

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_stop_distance(self): # 환자와의 목표 유지 거리 반환
        return self._stop_distance_m

    def get_linear_gain(self): # 직진 속도 gain 반환
        return self._kp_linear

    def get_angular_gain(self): # 회전 속도 gain 반환
        return self._kp_angular