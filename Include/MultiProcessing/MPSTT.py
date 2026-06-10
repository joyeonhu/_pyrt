# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPSpeechToText.py
# Project Name : HealthcareRobotPyRT
# Description  : Speech-to-text multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT, CMD_LISTEN_ONCE
from AudioRecognition.SpeechToText import CSpeechToText

def proc_speech_to_text(command_pipe, feedback_queue, feedback_queue_bk=None):
    """
    SpeechToText Process Routine

    역할:
        1. ControlCore로부터 Pipe로 command 수신
        2. START 명령을 받으면 STT 객체 초기화 후 대기 상태
        3. LISTEN_ONCE 명령을 받으면 정해진 시간 동안 녹음 + STT 1회 수행
        4. STT 결과를 Queue로 ControlCore에 전달
        5. STOP / EXIT 명령을 받으면 STT 정지 또는 프로세스 종료
    """

    stt = None # STT 객체

    is_running = True # 프로세스 실행 상태
    is_stt_ready = False # STT 준비 상태, START 명령을 받으면 True, STOP 명령을 받으면 False

    write_log("MPSpeechToText process routine started.")

    try:
        while is_running: # 프로세스가 종료 요청을 받을 때까지 반복

            # ------------------------------------------------------------------------------------------------------
            # 1. Command Receive
            # ------------------------------------------------------------------------------------------------------

            if command_pipe.poll(): # ControlCore에서 명령이 도착했는지 확인
                cmd_msg = command_pipe.recv() # 명령 메시지를 pipe에서 꺼냄

                data = cmd_msg.get(KEY_DATA, {}) # 메시지 안의 data 부분을 꺼냄, 없으면 빈 딕셔너리 사용
                command = data.get(KEY_COMMAND, None) # data 안에서 실제 command를 꺼냄, ex) "START", "STOP", "EXIT", "LISTEN_ONCE" 등, 없으면 None 사용

                if command == CMD_START: # 시작 명령
                    if stt is None: # 객체가 아직 생성되지 않았다면
                        stt = CSpeechToText() # STT 객체 생성

                    is_stt_ready = True # STT 준비 상태로 전환

                    status_msg = make_status_message( # STT 준비됐다는 상태 메시지 생성
                        "STT_READY",
                        PROC_STT,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_LISTEN_ONCE: # 한 번 듣기 명령, 이 명령이 들어오면 STT가 음성을 녹음하고 텍스트로 변환하는 작업을 한 번 수행
                    if not is_stt_ready or stt is None: # STT가 준비되지 않았거나 객체가 생성되지 않았다면
                        error_msg = make_error_message( # STT가 준비되지 않았다는 에러 메시지 생성
                            "STT is not ready",
                            PROC_STT,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(error_msg) # 에러 메시지를 ControlCore로 보냄
                        continue

                    status_msg = make_status_message( # STT가 듣기 시작했다는 상태 메시지 생성
                        "STT_LISTENING",
                        PROC_STT,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                    text = stt.listen() # STT 객체의 listen() 메서드를 호출해서 음성을 녹음하고 텍스트로 변환, 이 메서드는 녹음과 인식이 완료될 때까지 블로킹됨, 반환값은 인식된 텍스트 또는 None

                    if text is not None and len(text) > 0: # 인식된 텍스트가 None이 아니고 길이가 0보다 크면 (즉, 유효한 텍스트가 인식되었다면)
                        msg = make_text_message( # STT 결과 텍스트 메시지 생성
                            text,
                            PROC_STT,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(msg) # 텍스트 메시지를 ControlCore로 보냄

                    else:
                        status_msg = make_status_message( # STT가 유효한 입력을 받지 못했다는 상태 메시지 생성
                            "STT_NO_INPUT",
                            PROC_STT,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_STOP: # 정지 명령
                    is_stt_ready = False # STT 준비 상태로 전환

                    status_msg = make_status_message( # STT가 정지됐다는 상태 메시지 생성
                        "STT_STOPPED",
                        PROC_STT,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_EXIT: # 종료 명령
                    is_stt_ready = False # STT 준비 상태로 전환
                    is_running = False # 프로세스 루프 종료

            time.sleep(0.001) # 루프가 너무 빠르게 도는 것을 방지하기 위해 약간의 짧은 대기 시간 추가

    except Exception:
        ErrorHandler().report()

        error_msg = make_error_message(
            "MPSpeechToText process exception",
            PROC_STT,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(error_msg)

    finally:
        if stt is not None: # STT 객체가 생성되어 있다면
            stt.cleanup() # STT 객체의 cleanup() 메서드를 호출해서 임시 파일 삭제 등 정리 작업 수행

        status_msg = make_status_message( # STT 프로세스가 종료됐다는 상태 메시지 생성
            "STT_PROCESS_TERMINATED",
            PROC_STT,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(status_msg)

        write_log("MPSpeechToText process routine terminated.")