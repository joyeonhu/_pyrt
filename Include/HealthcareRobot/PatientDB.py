ㅁ# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : PatientDB.py
# Project Name : HealthcareRobotPyRT
# Description  : Patient database manager
# -------------------------------------------------------------------------------------------------------------------- #

import os
import math

from Commons import *


class CPatientDB:
    """
    환자 DB 관리자

    역할:
        1. patients.txt 파일 로드
        2. marker_id로 환자 정보 조회
        3. 환자 신장 정보를 이용해 실제 수평거리 계산
    """

    def __init__(
            self,
            db_path: str = None, # patients.txt 파일 경로
            iv_height_cm: float = 173.0, # 링거대 높이
            gap_cm: float = 47.5, # 마커가 환자 머리 끝에 있는게 아니니까 환자 머리 끝과 마커 사이의 간격 (대략 47.5cm, 환자마다 다를 수 있음), 보정값
    ):

        if db_path is None:
            root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
            db_path = os.path.join(root_path, "Data", "DB", "patients.txt")

        self._db_path = db_path
        self._iv_height_cm = iv_height_cm
        self._gap_cm = gap_cm

        self._patients = {}
        self._last_distance_cm = None

        self.load() # patients.txt 파일 로드

    # ==============================================================================================================
    # Load
    # ==============================================================================================================

    def load(self):
        """
        patients.txt 파일을 읽어서 메모리에 저장한다.

        현재 patients.txt 형식:
            marker_id first_name last_name sex blood height weight is_warning_patient
        """

        self._patients.clear()

        if not os.path.exists(self._db_path):
            raise FileNotFoundError(
                "Patient DB file not found: %s" % self._db_path
            )

        with open(self._db_path, "r", encoding="utf-8") as f: # DB 파일 열기
            lines = f.readlines() # 모든 줄을 읽어 리스트로 저장

        for line in lines:
            line = line.strip() # 양쪽 공백 제거

            if not line:
                continue

            cols = line.split() # 공백 기준으로 문자열 분리

            if len(cols) < 8:
                continue

            marker_id = cols[0].strip() # marker_id 추출

            weight = None
            if cols[6] not in ["\\N", "None", "none", "NULL", "null"]:
                try:
                    weight = float(cols[6])
                except Exception:
                    weight = None

            height = None
            if cols[5] not in ["\\N", "None", "none", "NULL", "null"]:
                try:
                    height = float(cols[5])
                except Exception:
                    height = None

            is_warning = str(cols[7]).lower() == "true"

            self._patients[marker_id] = {
                "marker_id": marker_id,
                "first_name": cols[1],
                "last_name": cols[2],
                "final_name": "%s %s" % (cols[1], cols[2]),
                "sex": cols[3],
                "blood": cols[4],
                "height": height,
                "weight": weight,
                "is_warning_patient": is_warning,
            }

        write_log(
            "Patient DB loaded: %d patients" % len(self._patients),
            self
        )

    # ==============================================================================================================
    # Patient Info
    # ==============================================================================================================

    def get_patient_info(self, marker_id: str):
        """
        marker_id로 환자 정보 조회
        """

        marker_id = str(marker_id).strip()

        if marker_id in self._patients:
            return self._patients[marker_id]

        write_log(
            "No patient row for marker_id=%s" % marker_id,
            self
        )

        return None

    def get_patient_name(self, marker_id: str) -> str:
        """
        marker_id로 환자 이름 반환
        """

        patient = self.get_patient_info(marker_id)

        if patient is None:
            return "Unknown"

        return patient.get("final_name", "Unknown")

    def load_patient_list(self):
        """
        전체 환자 목록 반환
        """

        result = []

        for marker_id in sorted(self._patients.keys(), key=lambda x: int(x) if str(x).isdigit() else x):
            patient = self._patients[marker_id]
            result.append({
                "marker_id": patient["marker_id"],
                "final_name": patient["final_name"],
            })

        return result

    # ==============================================================================================================
    # Distance
    # ==============================================================================================================

    def calculate_range( # 로봇-환자 간 실제 수평거리 계산
            self,
            marker_id: str,
            depth_cm: float
    ):
        """
        ZED depth로 얻은 마커까지의 거리(depth_cm)를 기반으로
        환자와 로봇 사이의 실제 수평거리(cm)를 계산한다.

        기존 patient_info.py의 calculate_range() 로직 기반.

        기존 식:
            REAL_DISTANCE = sqrt(Depth^2 - (IV_HEIGHT - P_HEIGHT + GAP)^2)
        """

        marker_id = str(marker_id).strip()

        patient = self.get_patient_info(marker_id)

        if patient is None:
            return self._last_distance_cm

        patient_height = patient.get("height", None)

        if patient_height is None:
            write_log(
                "height is None for marker_id=%s" % marker_id,
                self
            )
            return self._last_distance_cm

        try:
            depth_cm = float(depth_cm)
            patient_height = float(patient_height)

            vertical_gap = self._iv_height_cm - patient_height + self._gap_cm

            pow_range = pow(depth_cm, 2) - pow(vertical_gap, 2)

            if pow_range < 0:
                return self._last_distance_cm

            real_distance_cm = math.sqrt(pow_range)

            self._last_distance_cm = real_distance_cm

            return real_distance_cm

        except Exception:
            ErrorHandler().report()
            return self._last_distance_cm

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_db_path(self): # DB 파일 경로 반환
        return self._db_path

    def get_patient_count(self): # DB에 저장된 환자 수 반환
        return len(self._patients)