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
    HOMING = "HOMING"

    EMERGENCY = "EMERGENCY"
    ERROR = "ERROR"


# ======================================================================================================================
# Robot Event
# ======================================================================================================================

class RobotEvent(str, Enum): # 상태를 바꾸는 트리거
    STARTUP_DONE = "STARTUP_DONE"

    CMD_FOLLOW = "CMD_FOLLOW"
    CMD_GUIDE = "CMD_GUIDE"
    CMD_DELIVERY = "CMD_DELIVERY"
    CMD_HOMING = "CMD_HOMING"

    SWITCH_TO_GUIDE = "SWITCH_TO_GUIDE"
    SWITCH_TO_FOLLOW = "SWITCH_TO_FOLLOW"

    GUIDE_DONE = "GUIDE_DONE"
    DELIVERY_DONE = "DELIVERY_DONE"
    HOMING_DONE = "HOMING_DONE"

    FOLLOW_FAILED = "FOLLOW_FAILED"
    GUIDE_FAILED = "GUIDE_FAILED"
    DELIVERY_FAILED = "DELIVERY_FAILED"
    HOMING_FAILED = "HOMING_FAILED"

    GO_IDLE = "GO_IDLE"

    PATIENT_DETECTED = "PATIENT_DETECTED"
    PATIENT_LOST = "PATIENT_LOST"

    ARRIVAL_SIGNAL = "ARRIVAL_SIGNAL"
    LOCATION_RECEIVED = "LOCATION_RECEIVED"

    EMERGENCY_DETECTED = "EMERGENCY_DETECTED"
    EMERGENCY_CLEARED = "EMERGENCY_CLEARED"

    ERROR_OCCURRED = "ERROR_OCCURRED"


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
        RobotEvent.GUIDE_DONE,
        RobotEvent.GUIDE_FAILED,
        RobotEvent.ARRIVAL_SIGNAL,
        RobotEvent.LOCATION_RECEIVED,
        RobotEvent.EMERGENCY_DETECTED,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.DELIVERY: {
        RobotEvent.GO_IDLE,
        RobotEvent.DELIVERY_DONE,
        RobotEvent.DELIVERY_FAILED,
        RobotEvent.PATIENT_DETECTED,
        RobotEvent.ARRIVAL_SIGNAL,
        RobotEvent.LOCATION_RECEIVED,
        RobotEvent.EMERGENCY_DETECTED,
        RobotEvent.ERROR_OCCURRED,
    },

    RobotState.HOMING: {
        RobotEvent.GO_IDLE,
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
        if event in {
            RobotEvent.GO_IDLE,
            RobotEvent.GUIDE_DONE,
            RobotEvent.GUIDE_FAILED,
        }:
            return RobotState.IDLE

    elif current_state == RobotState.DELIVERY:
        if event in {
            RobotEvent.GO_IDLE,
            RobotEvent.DELIVERY_DONE,
            RobotEvent.DELIVERY_FAILED,
        }:
            return RobotState.IDLE

    elif current_state == RobotState.HOMING:
        if event in {
            RobotEvent.GO_IDLE,
            RobotEvent.HOMING_DONE,
            RobotEvent.HOMING_FAILED,
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