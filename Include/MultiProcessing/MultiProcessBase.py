# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MultiProcessBase.py
# Project Name : HealthcareRobotPyRT
# Description  : PyRT-style multiprocessing wrapper
# -------------------------------------------------------------------------------------------------------------------- #

import time
import threading
from multiprocessing import Process, Queue, Pipe
from typing import Union

from Commons import *
from HealthcareRobot.HealthcareMessage import *


CMD_START = "START"
CMD_STOP = "STOP"
CMD_EXIT = "EXIT"


class ThreadQueueBroadcaster(threading.Thread):
    """
    Process -> ControlCore feedback Queue를 계속 감시하다가,
    메시지가 들어오면 signal로 알려주는 thread.
    """

    def __init__(self, queue: Queue):
        super(ThreadQueueBroadcaster, self).__init__()

        self.daemon = True
        self.keep_alive = True

        self._queue = queue

        self.sig_queue_bcast = PySignal(object)
        self.sig_terminated = PySignal()

    def stop(self):
        self.keep_alive = False

        try:
            self._queue.put("QUIT_THREAD")
        except Exception:
            pass

    def run(self):
        while self.keep_alive:
            msg = self._queue.get(block=True)

            if msg == "QUIT_THREAD":
                break

            self.sig_queue_bcast.emit(msg)

            time.sleep(0.001)

        self.sig_terminated.emit()


class CMultiProcessBase:
    """
    PyRT 스타일 멀티프로세스 wrapper

    통신 구조:
        ControlCore -> Process : Pipe
        Process -> ControlCore : Queue

    역할:
        1. Pipe 생성
        2. Queue 생성
        3. 실제 Process 생성
        4. ControlCore에서 command 전송
        5. Process feedback을 signal로 broadcast
    """

    def __init__(
            self,
            process_name: str,
            process_routine=None,
            use_backup_queue: bool = False,
            daemon: bool = True
    ):

        self._process_name = process_name

        self.sig_queue_bcast = PySignal(object)
        self.sig_queue_bcast_bk = PySignal(object)
        self.sig_error = PySignal(str)

        # ControlCore -> Process command pipe
        self.pipeParent, self.pipeChild = Pipe()

        # Process -> ControlCore feedback queue
        self.queueRecv = Queue()

        if use_backup_queue:
            self.queueRecvBk = Queue()
        else:
            self.queueRecvBk = None

        if process_routine is None:
            process_routine = self.routine_proc

        process_args = (
            self.pipeChild,
            self.queueRecv,
            self.queueRecvBk
        )

        self.procDescriptor = Process(
            target=process_routine,
            name=self._process_name,
            args=process_args
        )

        self.procDescriptor.daemon = daemon

        self.threadQueueBcast = None
        self.threadQueueBcastBk = None

        self.start_thread_queue_bcast()

        if use_backup_queue:
            self.start_thread_queue_bcast_bk()

    # ==============================================================================================================
    # Process Control
    # ==============================================================================================================

    def start(self):
        """
        실제 child process 시작
        """

        if not self.procDescriptor.is_alive():
            self.procDescriptor.start()
            write_log("Process started: %s" % self._process_name, self)

    def release(self):
        """
        프로세스와 thread 정리
        """

        try:
            if self.procDescriptor.is_alive():
                self.send_command(CMD_EXIT)
                time.sleep(0.1)

            if self.procDescriptor.is_alive():
                self.procDescriptor.terminate()

            self.procDescriptor.join()

        except Exception:
            ErrorHandler().report()

        self.stop_thread_queue_bcast()
        self.stop_thread_queue_bcast_bk()

        try:
            self.pipeParent.close()
        except Exception:
            pass

        write_log("Process released: %s" % self._process_name, self)

    def terminate(self):
        """
        강제 종료
        """

        if self.procDescriptor.is_alive():
            self.procDescriptor.terminate()
            self.procDescriptor.join()

        write_log("Process terminated forcibly: %s" % self._process_name, self)

    def is_alive(self):
        return self.procDescriptor.is_alive()

    # ==============================================================================================================
    # Command Send - Pipe
    # ==============================================================================================================

    def send_command(
            self,
            command,
            data: dict = None
    ):
        """
        ControlCore가 Process에게 command를 보낸다.

        command가 dict이면 그대로 보낼 수도 있고,
        command가 str이면 HealthcareMessage 형식으로 감싸서 보낸다.
        """

        if not self.procDescriptor.is_alive():
            self.sig_error.emit("MP ERROR: process is not alive")
            return False

        if isinstance(command, dict):
            msg = command
        else:
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
            while not self.pipeParent.writable:
                time.sleep(0.001)

            self.pipeParent.send(msg)
            return True

        except Exception:
            ErrorHandler().report()
            return False

    # ==============================================================================================================
    # Default Process Routine
    # ==============================================================================================================

    def routine_proc(
            self,
            pipeChild,
            feedbackQueue: Queue,
            feedbackQueueBk: Union[Queue, None]
    ):
        """
        기본 process routine.
        실제 프로세스들은 각자 proc_xxx 함수를 넘겨서 사용한다.
        """

        pass

    # ==============================================================================================================
    # Queue Broadcast Thread
    # ==============================================================================================================

    def start_thread_queue_bcast(self):
        if self.threadQueueBcast is None:
            self.threadQueueBcast = ThreadQueueBroadcaster(self.queueRecv)
            self.threadQueueBcast.sig_queue_bcast.connect(self.on_bcast_thread_queue_bcast)
            self.threadQueueBcast.sig_terminated.connect(self.on_terminate_thread_queue_bcast)
            self.threadQueueBcast.start()

    def stop_thread_queue_bcast(self):
        if self.threadQueueBcast is not None:
            self.threadQueueBcast.stop()

    def on_bcast_thread_queue_bcast(self, msg: object):
        self.sig_queue_bcast.emit(msg)

    def on_terminate_thread_queue_bcast(self):
        self.threadQueueBcast = None

    # ==============================================================================================================
    # Backup Queue Broadcast Thread
    # ==============================================================================================================

    def start_thread_queue_bcast_bk(self):
        if self.queueRecvBk is None:
            return

        if self.threadQueueBcastBk is None:
            self.threadQueueBcastBk = ThreadQueueBroadcaster(self.queueRecvBk)
            self.threadQueueBcastBk.sig_queue_bcast.connect(self.on_bcast_thread_queue_bcast_bk)
            self.threadQueueBcastBk.sig_terminated.connect(self.on_terminate_thread_queue_bcast_bk)
            self.threadQueueBcastBk.start()

    def stop_thread_queue_bcast_bk(self):
        if self.threadQueueBcastBk is not None:
            self.threadQueueBcastBk.stop()

    def on_bcast_thread_queue_bcast_bk(self, msg: object):
        self.sig_queue_bcast_bk.emit(msg)

    def on_terminate_thread_queue_bcast_bk(self):
        self.threadQueueBcastBk = None

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    @property
    def PID(self):
        return self.procDescriptor.pid

    @property
    def NAME(self):
        return self.procDescriptor.name