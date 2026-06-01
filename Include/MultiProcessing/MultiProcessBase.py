# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MultiProcessBase.py
# Project Name : HealthcareRobotPyRT
# Description  : PyRT-style multiprocessing wrapper
# -------------------------------------------------------------------------------------------------------------------- #

import time
import multiprocessing as mp

from Commons import *
from HealthcareRobot.HealthcareMessage import *


CMD_START = "START"
CMD_STOP = "STOP"
CMD_EXIT = "EXIT"


class CMultiProcessBase:
    """
    PyRT 스타일 멀티프로세스 wrapper

    통신 구조:
        ControlCore -> Process : Pipe
        Process -> ControlCore : Queue
    """

    def __init__(
            self,
            process_name: str,
            target,
            args: tuple = None
    ):
        self._process_name = process_name
        self._target = target
        self._args = args if args is not None else tuple()

        # Command: ControlCore -> Process
        self.pipeParent, self.pipeChild = mp.Pipe()

        # Feedback: Process -> ControlCore
        self.queueRecv = mp.Queue()

        self.proc = mp.Process(
            target=self._target,
            args=(self.pipeChild, self.queueRecv) + self._args,
            name=self._process_name
        )

    # ==============================================================================================================
    # Process Control
    # ==============================================================================================================

    def start(self):
        if not self.proc.is_alive():
            self.proc.start()
            write_log("Process started: %s" % self._process_name, self)

    def stop(self):
        self.send_command(CMD_STOP)

    def exit(self):
        self.send_command(CMD_EXIT)

    def join(self, timeout=None):
        self.proc.join(timeout)

    def terminate(self):
        if self.proc.is_alive():
            self.proc.terminate()
            write_log("Process terminated forcibly: %s" % self._process_name, self)

    def is_alive(self):
        return self.proc.is_alive()

    # ==============================================================================================================
    # Command Send - Pipe
    # ==============================================================================================================

    def send_command(
            self,
            command: str,
            data: dict = None
    ):
        """
        ControlCore가 Process에게 명령을 보낸다.
        """

        if data is None:
            data = {}

        data[KEY_COMMAND] = command

        msg = make_message(
            MSG_TYPE_COMMAND,
            PROC_CONTROL_CORE,
            self._process_name,
            data
        )

        try:
            self.pipeParent.send(msg)
            return True

        except Exception:
            ErrorHandler().report()
            return False

    # ==============================================================================================================
    # Feedback Receive - Queue
    # ==============================================================================================================

    def has_feedback(self):
        return not self.queueRecv.empty()

    def recv_feedback(self):
        """
        Process가 보낸 feedback을 하나 읽는다.
        없으면 None 반환.
        """

        try:
            if not self.queueRecv.empty():
                return self.queueRecv.get_nowait()

        except Exception:
            ErrorHandler().report()

        return None

    def recv_all_feedback(self):
        """
        Queue에 쌓인 feedback을 모두 읽는다.
        """

        feedback_list = []

        while True:
            msg = self.recv_feedback()

            if msg is None:
                break

            feedback_list.append(msg)

        return feedback_list

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_process_name(self):
        return self._process_name

    def get_pid(self):
        return self.proc.pid