# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DestinationManager.py
# Project Name : HealthcareRobotPyRT
# Description  : Destination ID to Nav2 goal pose manager
# -------------------------------------------------------------------------------------------------------------------- #

import math

from Commons import *


class CDestinationManager:
    """
    목적지 관리자

    역할:
        LLMManager가 결정한 목적지 ID를
        Nav2 goal pose로 변환한다.
    """

    def __init__(self):
        self._destinations = {}

        self.load_default_destinations()

    def load_default_destinations(self):
        """
        기본 목적지 좌표 등록

        주의:
            좌표는 임시값이다.
            실제 my_map 기준으로 RViz에서 좌표를 찍어 수정해야 한다.
        """

        self.add_destination(
            destination_id="convenience_store",
            x=0.0,
            y=0.0,
            yaw=0.0
        )

        self.add_destination(
            destination_id="nurse_station",
            x=0.0,
            y=0.0,
            yaw=0.0
        )

        self.add_destination(
            destination_id="room_19421",
            x=0.0,
            y=0.0,
            yaw=0.0
        )

        self.add_destination(
            destination_id="home",
            x=0.0,
            y=0.0,
            yaw=0.0
        )

    def add_destination(
            self,
            destination_id: str,
            x: float,
            y: float,
            yaw: float
    ):
        """
        목적지 ID와 좌표를 등록한다.
        """

        destination_id = str(destination_id).strip() # 목적지 ID를 문자열로 변환하고 양쪽 공백 제거

        self._destinations[destination_id] = { # 목적지 ID를 키로, 좌표 정보를 값으로 저장
            "destination_id": destination_id,
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }

    def has_destination(self, destination_id: str) -> bool:
        """
        목적지 ID가 등록되어 있는지 확인한다.
        """

        if destination_id is None:
            return False

        destination_id = str(destination_id).strip() # 목적지 ID를 문자열로 변환하고 양쪽 공백 제거

        return destination_id in self._destinations # 목적지 ID가 등록되어 있는지 여부 반환

    def get_destination(self, destination_id: str):
        """
        목적지 ID로 전체 목적지 정보를 반환한다.
        """

        if destination_id is None:
            return None

        destination_id = str(destination_id).strip() # 목적지 ID를 문자열로 변환하고 양쪽 공백 제거

        return self._destinations.get(destination_id, None) # 목적지 ID에 해당하는 전체 목적지 정보 반환, 없으면 None 반환

    def get_goal_pose(self, destination_id: str):
        """
        목적지 ID로 Nav2 goal pose를 반환한다.

        반환:
            {
                "x": float,
                "y": float,
                "yaw": float
            }
        """

        dest = self.get_destination(destination_id)

        if dest is None:
            write_log(
                "Unknown destination_id: %s" % str(destination_id),
                self
            )
            return None

        return {
            "x": dest["x"],
            "y": dest["y"],
            "yaw": dest["yaw"],
        }

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        """
        yaw(rad)를 quaternion으로 변환한다.
        Nav2 PoseStamped에 넣을 때 사용한다.
        """
        # quaternion란 3D 공간에서의 회전을 표현하는 방법 중 하나로, 4개의 요소(x, y, z, w)로 구성된다.
        # Nav2 goal을 보낼 때 orientation을 quaternion으로 표현해야 하기 때문에, yaw(로봇이 회전해야 하는 각도)를 quaternion으로 변환하는 함수가 필요하다.
        # orientation : 로봇의 방향을 나타내는 값으로, Nav2 goal pose에서 사용된다. quaternion으로 표현되어야 한다.

        # quanernion 계산 공식
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        return {
            "x": 0.0,
            "y": 0.0,
            "z": qz,
            "w": qw,
        }

    def get_all_destinations(self):
        """
        등록된 전체 목적지 반환
        """

        return self._destinations

    def print_destinations(self):
        """
        등록된 목적지 목록 출력
        """

        for destination_id, dest in self._destinations.items():
            write_log(
                "%s -> x=%.2f, y=%.2f, yaw=%.2f"
                % (
                    destination_id,
                    dest["x"],
                    dest["y"],
                    dest["yaw"],
                ),
                self
            )