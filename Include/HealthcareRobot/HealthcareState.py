# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : HealthcareState.py
# Project Name : HealthcareRobotPyRT
# Description  : Robot state, event, and FSM transition rule definitions
# -------------------------------------------------------------------------------------------------------------------- #

from enum import Enum


# ======================================================================================================================
# Robot State
# ======================================================================================================================

class RobotState(str, Enum): # 로봇 현재 상태 정의
    START = "START"
    IDLE = "IDLE"

    FOLLOW = "FOLLOW"
    GUIDE = "GUIDE"
    DELIVERY = "DELIVERY"
    DELIVERY_VERIFY = "DELIVERY_VERIFY"
    HOMING = "HOMING"

    EMERGENCY = "EMERGENCY"
    ERROR = "ERROR"


# ======================================================================================================================
# Robot Event
# ======================================================================================================================

class RobotEvent(str, Enum): # 상태를 바꾸는 트리거
    STARTUP_DONE = "STARTUP_DONE" # 시스템 초기화 완료

    CMD_FOLLOW = "CMD_FOLLOW" # 외부에서 Follow 명령이 들어옴
    CMD_GUIDE = "CMD_GUIDE" # 외부에서 Guide 명령이 들어옴
    CMD_DELIVERY = "CMD_DELIVERY" # 외부에서 Delivery 명령이 들어옴
    CMD_HOMING = "CMD_HOMING" # 외부에서 Homing 명령이 들어옴

    SWITCH_TO_GUIDE = "SWITCH_TO_GUIDE" # Follow 중에 Guide로 전환
    SWITCH_TO_FOLLOW = "SWITCH_TO_FOLLOW" # Guide 중에 Follow로 전환

    NAVIGATION_ARRIVED = "NAVIGATION_ARRIVED" # Navigation이 목적지에 도착했다고 보고
    NAVIGATION_FAILED = "NAVIGATION_FAILED" # Navigation이 실패했다고 보고

    GUIDE_DONE = "GUIDE_DONE" # Guide 과정이 성공적으로 완료되었다고 보고
    DELIVERY_DONE = "DELIVERY_DONE" # Delivery 과정이 성공적으로 완료되었다고 보고
    HOMING_DONE = "HOMING_DONE" # Homing 과정이 성공적으로 완료되었다고 보고

    DELIVERY_VERIFY_DONE = "DELIVERY_VERIFY_DONE" # Delivery Verify 과정이 성공적으로 완료되었다고 보고
    DELIVERY_VERIFY_FAILED = "DELIVERY_VERIFY_FAILED" # Delivery Verify 과정이 실패했다고 보고

    FOLLOW_FAILED = "FOLLOW_FAILED" # Follow 과정이 실패했다고 보고
    GUIDE_FAILED = "GUIDE_FAILED" # Guide 과정이 실패했다고 보고
    DELIVERY_FAILED = "DELIVERY_FAILED" # Delivery 과정이 실패했다고 보고
    HOMING_FAILED = "HOMING_FAILED" # Homing 과정이 실패했다고 보고

    GO_IDLE = "GO_IDLE" # 외부에서 Idle로 돌아가라는 명령이 들어옴

    PATIENT_DETECTED = "PATIENT_DETECTED" # 환자가 감지되었다고 보고
    PATIENT_LOST = "PATIENT_LOST" # 환자가 감지되지 않는다고 보고

    ARRIVAL_SIGNAL = "ARRIVAL_SIGNAL" # 도착 신호가 감지되었다고 보고
    LOCATION_RECEIVED = "LOCATION_RECEIVED" # 위치 정보가 수신되었다고 보고

    EMERGENCY_DETECTED = "EMERGENCY_DETECTED" # 비상 상황이 감지되었다고 보고
    EMERGENCY_CLEARED = "EMERGENCY_CLEARED" # 비상 상황이 해제되었다고 보고

    ERROR_OCCURRED = "ERROR_OCCURRED" # 시스템 오류가 발생했다고 보고


# ======================================================================================================================
# FSM Allowed Events
# ======================================================================================================================

ALLOWED_EVENTS = { # 각 상태에서 허용되는 이벤트 정의
    RobotState.START: {
        RobotEvent.STARTUP_DONE,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.IDLE: {
        RobotEvent.CMD_FOLLOW,
        RobotEvent.CMD_GUIDE,
        RobotEvent.CMD_DELIVERY,
        RobotEvent.CMD_HOMING,
        RobotEvent.EMERGENCY_DETECTED,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.FOLLOW: {
        RobotEvent.SWITCH_TO_GUIDE,
        RobotEvent.GO_IDLE,
        RobotEvent.FOLLOW_FAILED,
        RobotEvent.PATIENT_LOST,
        RobotEvent.EMERGENCY_DETECTED,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.GUIDE: {
        RobotEvent.SWITCH_TO_FOLLOW,
        RobotEvent.GO_IDLE,
        RobotEvent.NAVIGATION_ARRIVED,
        RobotEvent.NAVIGATION_FAILED,
        RobotEvent.GUIDE_DONE,
        RobotEvent.GUIDE_FAILED,
        RobotEvent.ARRIVAL_SIGNAL,
        RobotEvent.LOCATION_RECEIVED,
        RobotEvent.EMERGENCY_DETECTED,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.DELIVERY: {
        RobotEvent.GO_IDLE,
        RobotEvent.NAVIGATION_ARRIVED,
        RobotEvent.NAVIGATION_FAILED,
        RobotEvent.PATIENT_DETECTED,
        RobotEvent.ARRIVAL_SIGNAL,
        RobotEvent.LOCATION_RECEIVED,
        RobotEvent.EMERGENCY_DETECTED,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.DELIVERY_VERIFY: {
        RobotEvent.GO_IDLE,
        RobotEvent.DELIVERY_VERIFY_DONE,
        RobotEvent.DELIVERY_VERIFY_FAILED,
        RobotEvent.DELIVERY_DONE,
        RobotEvent.DELIVERY_FAILED,
        RobotEvent.EMERGENCY_DETECTED,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.HOMING: {
        RobotEvent.GO_IDLE,
        RobotEvent.NAVIGATION_ARRIVED,
        RobotEvent.NAVIGATION_FAILED,
        RobotEvent.HOMING_DONE,
        RobotEvent.HOMING_FAILED,
        RobotEvent.EMERGENCY_DETECTED,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.EMERGENCY: {
        RobotEvent.EMERGENCY_CLEARED,
        RobotEvent.GO_IDLE,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.ERROR: {
        RobotEvent.GO_IDLE,
    },
}


# ======================================================================================================================
# State Transition Function
# ======================================================================================================================

def is_event_allowed(state: RobotState, event: RobotEvent) -> bool: # 현재 상태에서 이벤트 허용 여부 확인
    return event in ALLOWED_EVENTS.get(state, set())


def get_next_state(current_state: RobotState, event: RobotEvent) -> RobotState: # FSM 상태 전이 함수
    """
    현재 상태와 이벤트를 받아 다음 상태를 반환한다.
    실제 행동 실행은 여기서 하지 않고, ControlCore 또는 각 Process에서 처리한다.
    """

    # --------------------------------------------------------------------------------------------------------------
    # Global interrupt
    # --------------------------------------------------------------------------------------------------------------
    if event == RobotEvent.EMERGENCY_DETECTED:
        if current_state not in {RobotState.START, RobotState.EMERGENCY}:
            return RobotState.EMERGENCY

    if event == RobotEvent.ERROR_OCCURRED:
        return RobotState.ERROR

    # --------------------------------------------------------------------------------------------------------------
    # Normal transition
    # --------------------------------------------------------------------------------------------------------------
    if current_state == RobotState.START:
        if event == RobotEvent.STARTUP_DONE:
            return RobotState.IDLE

    elif current_state == RobotState.IDLE:
        if event == RobotEvent.CMD_FOLLOW:
            return RobotState.FOLLOW
        if event == RobotEvent.CMD_GUIDE:
            return RobotState.GUIDE
        if event == RobotEvent.CMD_DELIVERY:
            return RobotState.DELIVERY
        if event == RobotEvent.CMD_HOMING:
            return RobotState.HOMING

    elif current_state == RobotState.FOLLOW:
        if event == RobotEvent.SWITCH_TO_GUIDE:
            return RobotState.GUIDE
        if event in {
            RobotEvent.GO_IDLE,
            RobotEvent.FOLLOW_FAILED,
            RobotEvent.PATIENT_LOST,
        }:
            return RobotState.IDLE

    elif current_state == RobotState.GUIDE:
        if event == RobotEvent.SWITCH_TO_FOLLOW:
            return RobotState.FOLLOW
        if event == RobotEvent.NAVIGATION_ARRIVED:
            return RobotState.IDLE
        if event in {
            RobotEvent.GO_IDLE,
            RobotEvent.GUIDE_DONE,
            RobotEvent.GUIDE_FAILED,
            RobotEvent.NAVIGATION_FAILED,
        }:
            return RobotState.IDLE

    elif current_state == RobotState.DELIVERY:
        if event == RobotEvent.NAVIGATION_ARRIVED:
            return RobotState.DELIVERY_VERIFY
        if event in {
            RobotEvent.GO_IDLE,
            RobotEvent.DELIVERY_FAILED,
            RobotEvent.NAVIGATION_FAILED,
        }:
            return RobotState.IDLE

    elif current_state == RobotState.DELIVERY_VERIFY:
        if event == RobotEvent.DELIVERY_VERIFY_DONE:
            return RobotState.IDLE
        if event in {
            RobotEvent.GO_IDLE,
            RobotEvent.DELIVERY_VERIFY_FAILED,
            RobotEvent.DELIVERY_FAILED,
        }:
            return RobotState.IDLE

    elif current_state == RobotState.HOMING:
        if event in {
            RobotEvent.GO_IDLE,
            RobotEvent.HOMING_DONE,
            RobotEvent.HOMING_FAILED,
            RobotEvent.NAVIGATION_ARRIVED,
            RobotEvent.NAVIGATION_FAILED,
        }:
            return RobotState.IDLE

    elif current_state == RobotState.EMERGENCY:
        if event in {
            RobotEvent.EMERGENCY_CLEARED,
            RobotEvent.GO_IDLE,
        }:
            return RobotState.IDLE

    elif current_state == RobotState.ERROR:
        if event == RobotEvent.GO_IDLE:
            return RobotState.IDLE

    return current_state

# ======================================================================================================================
# Helper
# ======================================================================================================================

def to_robot_state(state) -> RobotState: # 문자열 -> RobotState 변환 함수
    if isinstance(state, RobotState):
        return state
    return RobotState(str(state))


def to_robot_event(event) -> RobotEvent: # 문자열 -> RobotEvent 변환 함수
    if isinstance(event, RobotEvent):
        return event
    return RobotEvent(str(event))