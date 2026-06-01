# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : CmdVelController.py
# Project Name : HealthcareRobotPyRT
# Description  : cmd_vel controller
# -------------------------------------------------------------------------------------------------------------------- #

from Commons import *
from Control.VelocityLimiter import CVelocityLimiter


class CCmdVelController:
    """
    cmd_vel 제어 클래스

    APF 또는 다른 제어기에서 계산된
    linear.x / angular.z 를 관리한다.
    """

    def __init__(self):

        self._linear_x = 0.0
        self._angular_z = 0.0

        self._vel_limiter = CVelocityLimiter()

    # ==============================================================================================================
    # Set cmd_vel
    # ==============================================================================================================

    def set_cmd_vel(
            self,
            linear_x: float,
            angular_z: float
    ):
        """
        cmd_vel 설정
        """

        linear_x, angular_z = self._vel_limiter.limit_cmd_vel(
            linear_x,
            angular_z
        )

        self._linear_x = linear_x
        self._angular_z = angular_z

    # ==============================================================================================================
    # Stop
    # ==============================================================================================================

    def stop(self):
        """
        로봇 정지
        """

        self._linear_x = 0.0
        self._angular_z = 0.0

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_linear_x(self):
        return self._linear_x

    def get_angular_z(self):
        return self._angular_z

    def get_cmd_vel(self):
        return self._linear_x, self._angular_z

    # ==============================================================================================================
    # Print
    # ==============================================================================================================

    def print_cmd_vel(self):

        write_log(
            "cmd_vel -> linear_x: %.3f | angular_z: %.3f"
            % (
                self._linear_x,
                self._angular_z
            ),
            self
        )