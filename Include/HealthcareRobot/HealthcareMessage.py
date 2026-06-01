# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : HealthcareMessage.py
# Project Name : HealthcareRobotPyRT
# Description  : IPC message format definitions
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *


# ======================================================================================================================
# Message Type
# ======================================================================================================================

# 메시지 종류 정의

MSG_TYPE_COMMAND = "COMMAND"
MSG_TYPE_EVENT = "EVENT"
MSG_TYPE_FSM = "FSM"

MSG_TYPE_FRAME = "FRAME"
MSG_TYPE_TEXT = "TEXT"
MSG_TYPE_SPEECH = "SPEECH"

MSG_TYPE_DETECTION = "DETECTION"
MSG_TYPE_EMERGENCY = "EMERGENCY"

MSG_TYPE_CMD_VEL = "CMD_VEL"

MSG_TYPE_STATUS = "STATUS"
MSG_TYPE_ERROR = "ERROR"


# ======================================================================================================================
# Common Message Keys
# ======================================================================================================================

# 딕셔너리 키 이름 통일

KEY_TYPE = "type"

KEY_SOURCE = "source"
KEY_TARGET = "target"

KEY_TIMESTAMP = "timestamp"

KEY_DATA = "data"

KEY_COMMAND = "command"
KEY_EVENT = "event"
KEY_FSM = "fsm"

KEY_FRAME = "frame"

KEY_TEXT = "text"
KEY_SPEAK = "speak"

KEY_PATIENT_ID = "patient_id"
KEY_PATIENT_NAME = "patient_name"

KEY_DISTANCE = "distance"
KEY_ANGLE = "angle"

KEY_LINEAR_X = "linear_x"
KEY_ANGULAR_Z = "angular_z"

KEY_EMERGENCY_TYPE = "emergency_type"
KEY_CONFIDENCE = "confidence"

KEY_ERROR = "error"
KEY_STATUS = "status"


# ======================================================================================================================
# Base Message
# ======================================================================================================================

def make_message( # 모든 IPC 메시지의 기본 형식 생성 함수
        msg_type: str,
        source: str,
        target: str,
        data: dict = None
):
    """
    기본 IPC 메시지 생성 함수
    """

    if data is None:
        data = {}

    msg = {
        KEY_TYPE: msg_type,
        KEY_SOURCE: source,
        KEY_TARGET: target,
        KEY_TIMESTAMP: time.time(),
        KEY_DATA: data,
    }

    return msg


# ======================================================================================================================
# Event Message
# ======================================================================================================================

def make_event_message(
        event: str,
        source: str,
        target: str = PROC_CONTROL_CORE,
        data: dict = None
):
    """
    FSM 이벤트 메시지 생성
    """

    if data is None:
        data = {}

    msg = make_message(
        MSG_TYPE_EVENT,
        source,
        target,
        data
    )

    msg[KEY_EVENT] = event

    return msg


# ======================================================================================================================
# State Message
# ======================================================================================================================

def make_fsm_message( # FSM 상태 메시지 생성 함수
        state: str,
        source: str = PROC_CONTROL_CORE,
        target: str = "ALL"
):
    """
    FSM 상태 전송 메시지
    """

    msg = make_message(
        MSG_TYPE_FSM,
        source,
        target,
        {}
    )

    msg[KEY_FSM] = state

    return msg


# ======================================================================================================================
# Status Message
# ======================================================================================================================

def make_status_message(
        status: str,
        source: str,
        target: str = PROC_CONTROL_CORE,
        data: dict = None
):
    """
    프로세스 상태 메시지 생성

    예:
        CAMERA_READY
        MODEL_LOADED
        RUNNING
    """

    if data is None:
        data = {}

    msg = make_message(
        MSG_TYPE_STATUS,
        source,
        target,
        data
    )

    msg[KEY_STATUS] = status

    return msg


# ======================================================================================================================
# Text Message
# ======================================================================================================================

def make_text_message( # STT / LLM 텍스트 메시지 생성 함수
        text: str,
        source: str,
        target: str
):
    """
    STT / LLM text 전달용 메시지
    """

    msg = make_message(
        MSG_TYPE_TEXT,
        source,
        target,
        {}
    )

    msg[KEY_TEXT] = text

    return msg


# ======================================================================================================================
# Emergency Message
# ======================================================================================================================

def make_emergency_message( # 응급 상황 메시지 생성 함수
        emergency_type: str,
        confidence: float,
        source: str = PROC_EMERGENCY,
        target: str = PROC_CONTROL_CORE
):
    """
    응급 상황 전달 메시지
    """

    msg = make_message(
        MSG_TYPE_EMERGENCY,
        source,
        target,
        {}
    )

    msg[KEY_EMERGENCY_TYPE] = emergency_type
    msg[KEY_CONFIDENCE] = confidence

    return msg


# ======================================================================================================================
# cmd_vel Message
# ======================================================================================================================

def make_cmd_vel_message( # 속도 명령 메시지 생성 함수
        linear_x: float,
        angular_z: float,
        source: str = PROC_HEALTHCARE_ROS,
        target: str = PROC_CONTROL_CORE
):
    """
    cmd_vel 전달 메시지
    """

    msg = make_message(
        MSG_TYPE_CMD_VEL,
        source,
        target,
        {}
    )

    msg[KEY_LINEAR_X] = linear_x
    msg[KEY_ANGULAR_Z] = angular_z

    return msg


# ======================================================================================================================
# Error Message
# ======================================================================================================================

def make_error_message( # 에러 메시지 생성 함수
        error_msg: str,
        source: str,
        target: str = PROC_CONTROL_CORE
):
    """
    에러 전달 메시지
    """

    msg = make_message(
        MSG_TYPE_ERROR,
        source,
        target,
        {}
    )

    msg[KEY_ERROR] = error_msg

    return msg