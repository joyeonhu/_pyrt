# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : EmergencyManager.py
# Project Name : HealthcareRobotPyRT
# Description  : Emergency state manager
# -------------------------------------------------------------------------------------------------------------------- #

from Commons import *

from Perception.EmergencyDetector import CEmergencyDetector


class CEmergencyManager:
    """
    응급 상황 관리자

    역할:
        1. frame 기반 응급 감지 수행
        2. 현재 응급 상태 관리
        3. 응급 시작 / 해제 관리
        4. 응급 타입 저장
    """

    def __init__(self):

        self._detector = CEmergencyDetector() # 응급 감지 모델

        self._is_emergency = False # 현재 응급 상태 여부

        self._emergency_type = None # 현재 응급 타입 (예: fall, fire, bleeding)

        self._last_caption = None # 마지막 VLM caption, 디버깅 및 평가용

        self._last_confidence = 0.0 # 마지막 confidence, 디버깅 및 평가용

        write_log("EmergencyManager initialized.", self)

    # ==============================================================================================================
    # Process
    # ==============================================================================================================

    def process( # frame 기반 응급 감지 함수
            self,
            frame_bgr
    ):
        """
        frame 기반 응급 감지 수행

        반환:
            {
                "is_emergency": bool,
                "emergency_type": str or None,
                "confidence": float,
                "caption": str
            }
        """

        result = self._detector.detect(frame_bgr) # CEmergencyDetector의 detect 함수 호출하여 응급 감지 수행

        self._is_emergency = result["is_emergency"] # 응급 상태 여부 업데이트

        self._emergency_type = result["emergency_type"] # 응급 타입 업데이트

        self._last_caption = result["caption"] # 마지막 caption 업데이트

        self._last_confidence = result["confidence"] # 마지막 confidence 업데이트

        if self._is_emergency: # 응급 상황이 감지된 경우 로그 출력

            write_log(
                "EMERGENCY DETECTED | type=%s | conf=%.2f"
                % (
                    str(self._emergency_type),
                    self._last_confidence
                ),
                self
            )

        return result # 응급 감지 결과 반환

    # ==============================================================================================================
    # Reset
    # ==============================================================================================================

    def reset(self):
        """
        응급 상태 초기화
        """

        self._detector.reset()

        self._is_emergency = False

        self._emergency_type = None

        self._last_caption = None

        self._last_confidence = 0.0

        write_log("Emergency state reset.", self)

    # ==============================================================================================================
    # State
    # ==============================================================================================================

    def is_emergency(self):
        """
        현재 응급 상태 여부 반환
        """

        return self._is_emergency

    def has_emergency(self):
        """
        응급 타입이 존재하는지 반환
        """

        return self._emergency_type is not None

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_emergency_type(self):
        """
        현재 응급 타입 반환

        예:
            fall
            fire
            bleeding
        """

        return self._emergency_type

    def get_last_caption(self):
        """
        마지막 VLM caption 반환
        """

        return self._last_caption

    def get_last_confidence(self):
        """
        마지막 confidence 반환
        """

        return self._last_confidence

    def get_detector(self):
        return self._detector # CEmergencyDetector 객체 반환