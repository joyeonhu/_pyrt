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

KEY_DESTINATION_ID = "destination_id"

KEY_DOOR_TEXT = "door_text"
KEY_DOOR_BBOX = "door_bbox"

KEY_IS_MATCHED = "is_matched"

KEY_PATIENT_INFO = "patient_info"

KEY_ERROR = "error"
KEY_STATUS = "status"


# ======================================================================================================================
# Base Message
# ======================================================================================================================

def make_message(
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

def make_fsm_message(
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

def make_text_message(
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

def make_emergency_message(
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
# Delivery Verification Message
# ======================================================================================================================

def make_delivery_verify_message(
        destination_id: str,
        detected_text: str,
        is_matched: bool,
        door_bbox=None,
        source: str = PROC_HEALTHCARE,
        target: str = PROC_CONTROL_CORE
):
    """
    문 / 호실 확인 결과 메시지 생성
    """

    msg = make_message(
        MSG_TYPE_DETECTION,
        source,
        target,
        {}
    )

    msg[KEY_DESTINATION_ID] = destination_id
    msg[KEY_DOOR_TEXT] = detected_text
    msg[KEY_IS_MATCHED] = is_matched

    if door_bbox is not None:
        msg[KEY_DOOR_BBOX] = door_bbox

    return msg


# ======================================================================================================================
# cmd_vel Message
# ======================================================================================================================

def make_cmd_vel_message(
        linear_x: float,
        angular_z: float,
        source: str = PROC_HEALTHCARE,
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

def make_error_message(
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