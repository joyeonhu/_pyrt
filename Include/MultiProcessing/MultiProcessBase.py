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

# 프로세스에 보낼 기본 명령어를 문자열로 정의
CMD_START = "START"
CMD_STOP = "STOP"
CMD_EXIT = "EXIT"


class ThreadQueueBroadcaster(threading.Thread):
    """
    Process -> ControlCore feedback Queue를 계속 감시하다가,
    메시지가 들어오면 signal로 알려주는 thread.
    """

    def __init__(self, queue: Queue): # 객체를 만들 때 감시할 queue를 매개변수로 받음
        super(ThreadQueueBroadcaster, self).__init__() # 부모 클래스인 threading.Thread 초기화

        self.daemon = True # 메인 프로그램이 종료될 때 이 thread도 같이 종료되게 하는 설정
        self.keep_alive = True # thread 루프를 계속 돌릴지 결정하는 플래그

        self._queue = queue # 감시할 queue를 변수로 저장

        self.sig_queue_bcast = PySignal(object) # queue에서 메시지를 받으면 이 signal로 밖에 알려줌
        self.sig_terminated = PySignal() # thread가 종료됐을 때 알려주는 signal

    def stop(self): # thread 종료 요청 함수
        self.keep_alive = False # 루프를 멈추도록 설정

        try:
            self._queue.put("QUIT_THREAD") # queue의 get()가 block 상태일 수 있으므로, 깨우기 위해 더미 메시지를 넣음
        except Exception:
            pass

    def run(self): # thread가 start()되면 자동 실행되는 함수
        while self.keep_alive: # 종료 요청이 없으면 계속 반복
            msg = self._queue.get(block=True) # queue에 메시지가 들어올 때까지 기다림

            if msg == "QUIT_THREAD": # 종료용 메시지면 루프 탈출
                break

            self.sig_queue_bcast.emit(msg) # 받은 feedback 메시지를 signal로 ControlCore에 알려줌

            time.sleep(0.001) # 너무 빠르게 도는 걸 방지하기 위해 아주 잠깐 쉼

        self.sig_terminated.emit() # thread가 끝났다고 알림


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
            process_name: str, # 프로세스 이름
            process_routine=None, # 프로세스가 실행할 함수
            use_backup_queue: bool = False, # backup queue 사용할지 여부 (feedback이 많거나, 중요한 메시지가 있을 때 backup queue로 따로 받도록 할 수 있음), 일단 우리 프로젝트에는 필요 없을 듯
            daemon: bool = True # 메인 프로그램이 종료될 때 이 프로세스도 같이 종료되게 하는 설정
    ):

        self._process_name = process_name

        self.sig_queue_bcast = PySignal(object) # feedback queue에서 메시지가 들어오면 ControlCore가 받을 signal
        self.sig_queue_bcast_bk = PySignal(object) # backup queue용 signal
        self.sig_error = PySignal(str) # 프로세스 관련 에러를 알리는 signal

        # ControlCore -> Process command pipe
        self.pipeParent, self.pipeChild = Pipe()

        # Process -> ControlCore feedback queue
        self.queueRecv = Queue()

        if use_backup_queue: # backup queue도 사용할 경우, feedback queue와 동일한 구조로 하나 더 만들어줌
            self.queueRecvBk = Queue()
        else:
            self.queueRecvBk = None

        if process_routine is None: # 프로세스가 실행할 함수가 지정되지 않았으면 예외 발생
            raise ValueError(
                "process_routine must be specified"
            )

        process_args = ( # 자식 프로세스 함수에 넘길 기본 인자, 프로세스가 시작될 때 처음 전달받는 인자들
            self.pipeChild,
            self.queueRecv,
            self.queueRecvBk
        )

        self.procDescriptor = Process( # 실제 프로세스 객체 생성
            target=process_routine, # 프로세스에서 실행할 함수
            name=self._process_name, # 프로세스 이름
            args=process_args # 프로세스 함수에 넘길 인자
        )

        self.procDescriptor.daemon = daemon

        self.threadQueueBcast = None # 기본 feedback queue 감시 thread 변수
        self.threadQueueBcastBk = None # backup feedback queue 감시 thread 변수

        self.start_thread_queue_bcast() # 기본 feedback queue 감시 thread 시작

        if use_backup_queue: # backup queue도 사용할 경우, backup feedback queue 감시 thread도 시작
            self.start_thread_queue_bcast_bk()

    # ==============================================================================================================
    # Process Control
    # ==============================================================================================================

    def start(self):
        """
        실제 child process 시작
        """

        if not self.procDescriptor.is_alive(): # 살아있는 프로세스가 아니면
            self.procDescriptor.start() # 프로세스 시작, 함수 실행됨
            write_log("Process started: %s" % self._process_name, self)

    def release(self):
        """
        프로세스와 thread 정리
        """

        try:
            if self.procDescriptor.is_alive(): # 프로세스가 살아있으면
                self.send_command(CMD_EXIT) # 프로세스에게 종료 명령어 보냄
                time.sleep(0.1) # 프로세스가 명령어 받고 종료할 시간 잠깐 줌

                if self.procDescriptor.is_alive(): # 그래도 살아있으면
                    self.procDescriptor.terminate() # 프로세스 강제 종료

            self.procDescriptor.join() # 프로세스가 완전히 종료될 때까지 기다림

        except Exception:
            ErrorHandler().report()

        self.stop_thread_queue_bcast() # feedback queue 감시 thread 정리
        self.stop_thread_queue_bcast_bk() # backup feedback queue 감시 thread 정리

        try:
            self.pipeParent.close() # pipe 닫음
        except Exception:
            pass

        write_log("Process released: %s" % self._process_name, self)

    def is_alive(self): # 프로세스가 살아있는지 여부 반환
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

        if not self.procDescriptor.is_alive(): # 프로세스가 살아있지 않으면
            self.sig_error.emit("MP ERROR: process is not alive") # 에러 signal 발생
            return False

        if isinstance(command, dict): # command가 dict이면 그대로 보냄
            msg = command
        else:
            if data is None:
                data = {}

            data[KEY_COMMAND] = command
            # data에 command 내용을 합침

            msg = make_message( # HealthcareMessage 형식으로 command와 data를 감싸서 보냄
                MSG_TYPE_COMMAND,
                PROC_CONTROL_CORE,
                self._process_name,
                data
            )

        try:
            self.pipeParent.send(msg) # pipe로 command 메시지 보냄
            return True

        except Exception:
            ErrorHandler().report()
            return False

    # ==============================================================================================================
    # Queue Broadcast Thread
    # ==============================================================================================================

    def start_thread_queue_bcast(self): # feedback queue 감시 thread 시작 함수
        if self.threadQueueBcast is None: # 아직 thread가 없으면
            self.threadQueueBcast = ThreadQueueBroadcaster(self.queueRecv) # 기본 feedback queue 감시 thread 생성
            self.threadQueueBcast.sig_queue_bcast.connect(self.on_bcast_thread_queue_bcast) # thread가 feedback을 받으면 on_bcast_thread_queue_bcast를 실행하도록 연결
            self.threadQueueBcast.sig_terminated.connect(self.on_terminate_thread_queue_bcast) # thread가 종료됐을 때 on_terminate_thread_queue_bcast를 실행하도록 연결
            self.threadQueueBcast.start() # thread 시작, start() 안에 run() 실행하는 코드가 있어서 start()하면 run()이 자동으로 실행됨

    def stop_thread_queue_bcast(self): # feedback 감시 thread 종료 요청
        if self.threadQueueBcast is not None: # thread가 있으면
            self.threadQueueBcast.stop() # thread의 stop() 함수 호출해서 종료 요청

    def on_bcast_thread_queue_bcast(self, msg: object): # thread가 feedback을 받았을 때 실행되는 함수, msg는 feedback queue에서 받은 메시지
        self.sig_queue_bcast.emit(msg) # 받은 메시지를 ControlCore가 받을 수 있도록 sig_queue_bcast signal로 전달

    def on_terminate_thread_queue_bcast(self): # thread가 종료됐을 때 실행되는 함수
        self.threadQueueBcast = None # thread가 종료됐으므로 threadQueueBcast 변수를 None으로 설정

    # ==============================================================================================================
    # Backup Queue Broadcast Thread
    # ==============================================================================================================

    def start_thread_queue_bcast_bk(self): #
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
    def PID(self): # 프로세스 ID 반환, 프로세스가 시작되면 PID가 할당되므로, 프로세스가 시작된 후에 이 속성을 호출해야 함
        return self.procDescriptor.pid

    @property
    def NAME(self): # 프로세스 이름 반환
        return self.procDescriptor.name