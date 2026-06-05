# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : Commons.py
# Project Name : HealthcareRobotPyRT
# Description  : Common utilities for PyRT-style healthcare robot project
# -------------------------------------------------------------------------------------------------------------------- #

import os
import sys
import datetime
import logging
from logging import handlers
import threading
import traceback
import importlib.util


# ======================================================================================================================
# Path
# ======================================================================================================================

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(FILE_PATH)

INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
LOG_PATH = os.path.join(ROOT_PATH, "Log")

sys.path.extend([INCLUDE_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))


# ======================================================================================================================
# Basic Constants
# ======================================================================================================================

MAX_SERIAL_BUFFER = 1024 # 시리얼 통신 버퍼 최대 크기 (1KB)
MAX_SIZE = 100 * 1024 * 1024 # 로그 파일 최대 크기 (100MB)

PROJECT_NAME = "HealthcareRobotPyRT" # 프로젝트 이름


# ======================================================================================================================
# Process Names
# ======================================================================================================================
'''프로세스 이름 통일 (프로세스 간 통신 시 식별자 역할)'''

PROC_CONTROL_CORE = "ControlCore"

PROC_CAMERA = "MPCamera"
PROC_STT = "MPSTT"
PROC_TTS = "MPTTS"
PROC_LLM = "MPLLM"
PROC_EMERGENCY = "MPEmergency"
PROC_HEALTHCARE = "MPRobotTask"


# ======================================================================================================================
# PySignal - PyRT Style
# ======================================================================================================================

def check_argument_type(obj, arg): # PySignal에서 emit할 때, 타입이 맞는지 확인하는 함수
    if type(obj) == arg: # 타입이 정확히 일치하면 True
        return True
    if obj is None: # None은 모든 타입과 호환되도록 허용
        return True
    if arg == object: # object 타입은 모든 객체와 호환되도록 허용
        return True
    if arg in obj.__class__.__bases__: # obj의 클래스가 arg를 상속받았으면 True
    # obj.__class__ : 이 객체가 무슨 클래스인지
    # obj.__bases__ : 이 클래스가 어떤 클래스를 상속받았는지 (튜플 형태로 반환)
    # obj.__class__.__bases__ : obj의 클래스가 상속받은 클래스들의 튜플에서 arg 확인
    # ex) class Animal: pass; class Dog(Animal): pass; obj.__class__.__bases__ -> (Animal,)
        return True
    return False # 그 외의 경우는 타입 불일치로 간주하여 False


class PySignal(object): # 어떤 일이 생겼을 때 연결된 함수를 호출해주는 장치
    _args = None # 시그널이 어떤 타입의 데이터를 받을 건지 저장
    _callback = None # 시그널이 발생했을 때 실행할 함수

    def __init__(self, *args):
        self._args = args # 시그널이 받을 데이터 타입을 저장 (예: PySignal(int, str) -> _args = (int, str))

    def connect(self, callback): # 콜백 함수 연결, 연결만
        self._callback = callback # 시그널에 실행할 함수를 연결

    def emit(self, *args): # 연결된 함수 실행
        if len(args) != len(self._args): # emit할 때 전달된 인자 개수가 시그널이 정의된 인자 개수와 다르면 예외 발생
            raise Exception("Callback::Argument Length Mismatch")

        arglen = len(args) # 전달된 인자 개수

        if arglen > 0: # 인자가 있으면 타입 체크
            valid_types = [ # 각 인자에 대해 타입이 맞는지 체크하는 리스트
                check_argument_type(args[i], self._args[i])
                for i in range(arglen)
            ]

            if sum(valid_types) != arglen: # valid_types 리스트에서 True의 개수가 arglen과 다르면 타입 불일치로 간주하여 예외 발생
                raise Exception(
                    "Callback::Argument Type Mismatch "
                    "(Definition: {}, Call: {}, Result: {})".format(
                        self._args, args, valid_types
                    )
                )

        if self._callback is not None: # 연결된 함수가 있으면
            self._callback(*args) # 연결된 함수 실행


# ======================================================================================================================
# System Check
# ======================================================================================================================

def is_ros_installed() -> bool: # ROS 설치 여부 확인 (rclpy 모듈이 존재하는지 검사)
    spec = importlib.util.find_spec("rclpy")
    return spec is not None


def is_cuda_available() -> bool: # CUDA 사용 가능 여부 확인 (torch 모듈이 존재하고 CUDA가 사용 가능한지 검사)
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False


def set_cpu_affinity(pid=0, affinity_mask={1, 2}): # CPU affinity 설정 (특정 CPU 코어에 프로세스 할당)
    try:
        os.sched_setaffinity(pid, affinity_mask)
        return True
    except Exception:
        return False


# ======================================================================================================================
# Time / Log
# ======================================================================================================================

glob_logger = None # 전역 로거 객체 (로그 파일에 기록하기 위한 로거 인스턴스)


def ensurePathExist(path: str): # 지정된 경로가 존재하는지 확인하고, 존재하지 않으면 디렉토리를 생성하는 함수
    target_path = os.path.abspath(path)

    if not os.path.isdir(target_path):
        os.makedirs(target_path, exist_ok=True)


def timestamp_to_string(timestamp: datetime.datetime): # datetime 객체를 "HH:MM:SS.microseconds" 형식의 문자열로 변환하는 함수
    h = timestamp.hour
    m = timestamp.minute
    s = timestamp.second
    us = timestamp.microsecond

    return "%02d:%02d:%02d.%06d" % (h, m, s, us)


def get_curr_datetime(): # 현재 날짜와 시간을 datetime 객체로 반환하는 함수
    return datetime.datetime.now().strftime("%Y%m%d_%H_%M_%S")


def get_curr_time(): # 현재 시간을 "HH:MM:SS.microseconds" 형식의 문자열로 반환하는 함수
    return "<%s>" % timestamp_to_string(datetime.datetime.now())


def write_log(strMsg: str, obj: object = None, logfile: bool = True): # 로그 메시지를 콘솔에 출력하고, logfile이 True인 경우 로그 파일에도 기록하는 함수
    global glob_logger

    if glob_logger is None:
        ensurePathExist(LOG_PATH)

        log_file_path = os.path.join(LOG_PATH, "Console.log")
        glob_logger = logging.getLogger("console")

        if not glob_logger.handlers:
            fh = handlers.RotatingFileHandler(
                log_file_path,
                maxBytes=MAX_SIZE,
                backupCount=10,
                encoding="utf-8",
            )

            formatter = logging.Formatter("[%(asctime)s]%(message)s")
            fh.setFormatter(formatter)

            glob_logger.addHandler(fh)
            glob_logger.setLevel(logging.DEBUG)

    str_time = get_curr_time()

    if obj is not None:
        if isinstance(obj, threading.Thread):
            if obj.ident is not None:
                str_obj = " [%s][Thread ID:0x%x]" % (
                    type(obj).__name__,
                    obj.ident,
                )
            else:
                str_obj = " [%s]" % type(obj).__name__
        else:
            str_obj = " [%s]" % type(obj).__name__
    else:
        str_obj = ""

    print(str_time + str_obj + " " + strMsg)

    if logfile:
        glob_logger.info(str_obj + " " + strMsg)


class ErrorHandler: # 예외 발생 시 스택 트레이스를 로그 파일에 기록하는 클래스
    _logger = None
    _logFilePath = ""

    def __init__(self):
        file_path = os.path.join(LOG_PATH, "Error.log")
        self._logFilePath = file_path

        ensurePathExist(os.path.dirname(file_path))

        self._logger = logging.getLogger("error")

        if not self._logger.handlers:
            fh = handlers.RotatingFileHandler(
                file_path,
                maxBytes=MAX_SIZE,
                backupCount=10,
                encoding="utf-8",
            )

            formatter = logging.Formatter("[%(asctime)s] %(levelname)s - %(message)s")
            fh.setFormatter(formatter)

            self._logger.addHandler(fh)
            self._logger.setLevel(logging.DEBUG)

    def report(self):
        traceback.print_exc()
        self._logger.error(traceback.format_exc())

    @property
    def logFilePath(self) -> str:
        return self._logFilePath


# ======================================================================================================================
# Small Utils
# ======================================================================================================================

def clamp(value: float, min_value: float, max_value: float) -> float: # value를 min_value와 max_value 사이로 제한하는 함수 (value가 min_value보다 작으면 min_value를 반환하고, value가 max_value보다 크면 max_value를 반환하며, 그렇지 않으면 value를 그대로 반환)
    return max(min(float(value), max_value), min_value)