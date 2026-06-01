# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MultiProcessBase.py
# Project Name : HealthcareRobotPyRT
# Description  : Base class for PyRT-style multiprocessing modules
# -------------------------------------------------------------------------------------------------------------------- #

import time
import multiprocessing as mp

from Commons import *
from HealthcareRobot.HealthcareMessage import *


class CMultiProcessBase(mp.Process):
    """
    PyRT 스타일 멀티프로세스 기본 클래스

    통신 구조:
        1. Command
            ControlCore -> Process
            Pipe 사용

        2. Feedback
            Process -> ControlCore
            Queue 사용

    역할:
        1. multiprocessing.Process 상속
        2. 프로세스 실행/종료 플래그 관리
        3. ControlCore로부터 command 수신
        4. ControlCore로 feedback/event/status/error 전송
        5. 모든 하위 프로세스가 같은 구조로 동작하도록 공통 틀 제공
    """

    def __init__(
            self,
            process_name: str,
            command_pipe=None,
            feedback_queue=None
    ):
        super().__init__()

        self._process_name = process_name # 프로세스 이름

        # ControlCore -> Process command 전달용 Pipe
        self._command_pipe = command_pipe

        # Process -> ControlCore feedback 전달용 Queue
        self._feedback_queue = feedback_queue

        # 프로세스 상태 플래그
        self._is_running = mp.Value("b", False) # 프로세스 실행 중 여부 플래그
        self._is_terminated = mp.Value("b", False) # 프로세스 종료 여부 플래그
        # mp.Value() : multiprocessing에서 프로세스 간 공유 가능한 변수를 생성하는 함수. "b"는 boolean 타입을 의미하며, 초기값은 False로 설정됨.

    # ==============================================================================================================
    # Process Life Cycle
    # ==============================================================================================================

    def run(self):
        """
        multiprocessing.Process가 start() 되었을 때 자동 실행되는 함수
        """

        self._is_running.value = True # 프로세스 실행 중 상태로 설정
        self._is_terminated.value = False # 프로세스 종료 상태 비활성화

        write_log("Process started.", self)

        try:
            self.on_start() # 초기화 작업 수행

            while self._is_running.value:  # 프로세스 실행 중이면
                self.process_once()  # 프로세스 루프에서 반복 실행되는 함수 호출
                time.sleep(0.001)  # 루프가 너무 빠르게 도는 것을 방지하기 위해 1ms 대기

        except KeyboardInterrupt:
            pass

        except Exception:
            ErrorHandler().report()
            self.send_error("Process exception occurred.")

        finally: # 프로세스 종료 시
            try:
                self.on_stop() # 종료 작업 수행

            except Exception:
                ErrorHandler().report()

            self._is_running.value = False # 프로세스 실행 중 상태 비활성화
            self._is_terminated.value = True # 프로세스 종료 상태로 설정

            write_log("Process terminated.", self)

    def stop(self):
        """
        프로세스 종료 요청
        """

        self._is_running.value = False

    def terminate_process(self):
        """
        외부에서 명시적으로 종료 요청할 때 호출
        """

        self.stop()

    # ==============================================================================================================
    # Override Methods
    # ==============================================================================================================

    def on_start(self):
        """
        프로세스 시작 시 한 번 실행된다.
        하위 클래스에서 override 한다.
        """
        pass

    def process_once(self):
        """
        프로세스 루프에서 반복 실행된다.
        하위 클래스에서 override 한다.
        """
        pass

    def on_stop(self):
        """
        프로세스 종료 시 한 번 실행된다.
        하위 클래스에서 override 한다.
        """
        pass

    # ==============================================================================================================
    # Command Receive - Pipe
    # ==============================================================================================================

    def recv_command(self): # pipe에서 Controlccore가 보낸 명령을 받는 함수
        """
        command_pipe에서 command 메시지 하나를 읽는다.

        통신 방향:
            ControlCore -> Process

        없으면 None 반환.
        """

        if self._command_pipe is None: # command_pipe가 설정되어 있지 않으면
            return None

        try:
            if self._command_pipe.poll(): # command_pipe에 읽을 데이터가 있으면
                return self._command_pipe.recv() # command_pipe에서 데이터 읽기, .recv() : pipe에서 데이터를 읽는 함수, .recv()는 blocking 함수로, 데이터가 올 때까지 기다림. 따라서 .poll()로 먼저 데이터가 있는지 확인한 후에 .recv()를 호출해야 함.
            # .poll() : command_pipe에 읽을 데이터가 있는지 확인하는 함수, True면 데이터 있음, False면 데이터 없음

        except Exception:
            ErrorHandler().report()
            return None

        return None

    def has_command(self): # pipe에 읽을 명령이 있는지만 확인하는 함수
        """
        command_pipe에 받을 command가 있는지 확인한다.
        """

        if self._command_pipe is None:
            return False

        try:
            return self._command_pipe.poll()

        except Exception:
            return False

    # ==============================================================================================================
    # Feedback Send - Queue
    # ==============================================================================================================

    def send_feedback(self, msg: dict): # queue로 ControlCore에 메시지를 보내는 함수
        """
        feedback_queue로 ControlCore에 메시지를 보낸다.

        통신 방향:
            Process -> ControlCore
        """

        if self._feedback_queue is None:
            return False

        try:
            self._feedback_queue.put(msg) # feedback_queue에 메시지 넣기, .put() : queue에 데이터를 넣는 함수
            return True

        except Exception:
            ErrorHandler().report()
            return False

    def send_status(self, status: str, data: dict = None):
        """
        프로세스 상태 메시지를 ControlCore로 보낸다.

        예:
            CAMERA_READY
            MODEL_LOADED
            RUNNING
        """

        msg = make_status_message(
            status,
            self._process_name,
            PROC_CONTROL_CORE,
            data
        )

        return self.send_feedback(msg)

    def send_error(self, error_msg: str):
        """
        에러 메시지를 ControlCore로 보낸다.
        """

        msg = make_error_message(
            error_msg,
            self._process_name,
            PROC_CONTROL_CORE
        )

        return self.send_feedback(msg)

    def send_event(self, event: str, data: dict = None):
        """
        FSM 이벤트 메시지를 ControlCore로 보낸다.
        """

        msg = make_event_message(
            event,
            self._process_name,
            PROC_CONTROL_CORE,
            data
        )

        return self.send_feedback(msg)

    def send_text(self, text: str):
        """
        텍스트 결과를 ControlCore로 보낸다.

        예:
            STT 결과
            LLM 응답 텍스트
        """

        msg = make_text_message(
            text,
            self._process_name,
            PROC_CONTROL_CORE
        )

        return self.send_feedback(msg)

    def send_emergency(
            self,
            emergency_type: str,
            confidence: float
    ):
        """
        응급 상황 메시지를 ControlCore로 보낸다.
        """

        msg = make_emergency_message(
            emergency_type,
            confidence,
            self._process_name,
            PROC_CONTROL_CORE
        )

        return self.send_feedback(msg)

    def send_cmd_vel(
            self,
            linear_x: float,
            angular_z: float
    ):
        """
        cmd_vel 메시지를 ControlCore로 보낸다.
        """

        msg = make_cmd_vel_message(
            linear_x,
            angular_z,
            self._process_name,
            PROC_CONTROL_CORE
        )

        return self.send_feedback(msg)

    def send_detection(self, detection_data: dict):
        """
        perception 결과를 ControlCore로 보낸다.

        예:
            환자 감지 결과
            문 감지 결과
        """

        if detection_data is None:
            detection_data = {}

        msg = make_message(
            MSG_TYPE_DETECTION,
            self._process_name,
            PROC_CONTROL_CORE,
            detection_data
        )

        return self.send_feedback(msg)

    # ==============================================================================================================
    # Command Helper
    # ==============================================================================================================

    @staticmethod
    def get_command_type(command_msg: dict):
        """
        command 메시지에서 command 값을 꺼낸다.
        """

        if not isinstance(command_msg, dict):
            return None

        data = command_msg.get(KEY_DATA, {})

        if not isinstance(data, dict):
            return None

        return data.get(KEY_COMMAND, None)

    @staticmethod
    def get_command_data(command_msg: dict):
        """
        command 메시지에서 data 부분을 꺼낸다.
        """

        if not isinstance(command_msg, dict):
            return {}

        data = command_msg.get(KEY_DATA, {})

        if not isinstance(data, dict):
            return {}

        return data

    # ==============================================================================================================
    # State
    # ==============================================================================================================

    def is_running(self): # 프로세스가 실행 중인지 반환
        return self._is_running.value

    def is_terminated(self): # 프로세스가 종료되었는지 반환
        return self._is_terminated.value

    def get_process_name(self): # 프로세스 이름 반환
        return self._process_name