# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : VelocityLimiter.py
# Project Name : HealthcareRobotPyRT
# Description  : cmd_vel safety limiter
# -------------------------------------------------------------------------------------------------------------------- #

from Commons import *


class CVelocityLimiter:
    """
    cmd_vel 제한 클래스

    linear.x 와 angular.z 값을
    설정한 최대/최소 범위 안으로 제한한다.
    """

    def __init__(
            self,
            max_linear_x: float = 0.5,
            min_linear_x: float = -0.5,
            max_angular_z: float = 1.0,
            min_angular_z: float = -1.0,
    ):

        self._max_linear_x = max_linear_x
        self._min_linear_x = min_linear_x

        self._max_angular_z = max_angular_z
        self._min_angular_z = min_angular_z

    # ==============================================================================================================
    # Linear
    # ==============================================================================================================

    def limit_linear_x(self, linear_x: float) -> float: # 앞뒤로 움직이는 속도 제한
        """
        linear.x 제한
        """

        linear_x = clamp(
            linear_x,
            self._min_linear_x,
            self._max_linear_x
        )

        return linear_x

    # ==============================================================================================================
    # Angular
    # ==============================================================================================================

    def limit_angular_z(self, angular_z: float) -> float: # 좌우로 회전하는 속도 제한
        """
        angular.z 제한
        """

        angular_z = clamp(
            angular_z,
            self._min_angular_z,
            self._max_angular_z
        )

        return angular_z

    # ==============================================================================================================
    # cmd_vel
    # ==============================================================================================================

    def limit_cmd_vel( # linear.x 와 angular.z 값을 동시에 제한하는 함수
            self,
            linear_x: float,
            angular_z: float
    ):
        """
        linear.x / angular.z 동시 제한
        """

        linear_x = self.limit_linear_x(linear_x)
        angular_z = self.limit_angular_z(angular_z)

        return linear_x, angular_z

    # ==============================================================================================================
    # Setter
    # ==============================================================================================================

    def set_linear_limit( # linear.x 최대/최소 범위 설정 함수
            self,
            min_linear_x: float,
            max_linear_x: float
    ):

        self._min_linear_x = min_linear_x
        self._max_linear_x = max_linear_x

    def set_angular_limit( # angular.z 최대/최소 범위 설정 함수
            self,
            min_angular_z: float,
            max_angular_z: float
    ):

        self._min_angular_z = min_angular_z
        self._max_angular_z = max_angular_z

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    @property
    def max_linear_x(self):
        return self._max_linear_x

    @property
    def min_linear_x(self):
        return self._min_linear_x

    @property
    def max_angular_z(self):
        return self._max_angular_z

    @property
    def min_angular_z(self):
        return self._min_angular_z