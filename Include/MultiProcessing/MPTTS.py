# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPTextToSpeech.py
# Project Name : HealthcareRobotPyRT
# Description  : Text-to-speech multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT
from SpeechSynthesis.TextToSpeech import CTextToSpeech


CMD_SPEAK = "SPEAK"


def proc_text_to_speech(command_pipe, feedback_queue, feedback_queue_bk=None):
    """
    TextToSpeech Process Routine

    역할:
        1. ControlCore로부터 Pipe로 command 수신
        2. START 명령을 받으면 TTS 객체 초기화
        3. SPEAK 명령을 받으면 text를 음성으로 출력
        4. 완료 결과를 Queue로 ControlCore에 전달
        5. STOP / EXIT 명령을 받으면 정지 또는 프로세스 종료
    """

    tts = None # TTS 객체

    is_running = True # 프로세스 실행 상태
    is_tts_ready = False # TTS 준비 상태, START 명령을 받으면 True, STOP 명령을 받으면 False

    write_log("MPTextToSpeech process routine started.")

    try:
        while is_running: # 프로세스가 종료 요청을 받을 때까지 반복

            if command_pipe.poll(): # ControlCore에서 명령이 도착했는지 확인
                cmd_msg = command_pipe.recv() # 명령 메시지를 pipe에서 꺼냄

                data = cmd_msg.get(KEY_DATA, {}) # 메시지 안의 data 부분을 꺼냄, 없으면 빈 딕셔너리 사용
                command = data.get(KEY_COMMAND, None) # data 안에서 실제 command를 꺼냄, ex) "START", "SPEAK", "STOP", "EXIT" 등, 없으면 None 사용

                if command == CMD_START: # 시작 명령
                    if tts is None: # 객체가 아직 생성되지 않았다면
                        tts = CTextToSpeech() # TTS 객체 생성

                    is_tts_ready = True # TTS 준비 상태로 전환

                    status_msg = make_status_message( # TTS 준비됐다는 상태 메시지 생성
                        "TTS_READY",
                        PROC_TTS,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_SPEAK: # 말하기 명령, 이 명령이 들어오면 TTS가 text를 음성으로 출력하는 작업을 수행
                    if not is_tts_ready or tts is None: # TTS가 준비되지 않았거나 객체가 생성되지 않았다면
                        error_msg = make_error_message( # TTS가 준비되지 않았다는 에러 메시지 생성
                            "TTS is not ready",
                            PROC_TTS,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(error_msg) # 에러 메시지를 ControlCore로 보냄
                        continue

                    text = data.get(KEY_TEXT, None) # data 안에서 text를 꺼냄, 없으면 None 사용

                    if text is None: # text가 None이면 빈 문자열로 대체
                        text = data.get(KEY_SPEAK, None) # SPEAK 키로도 text를 꺼내봄, 호환성 위해

                    if text is None or len(str(text).strip()) == 0: # text가 None이거나 빈 문자열이면 TTS로 출력할 내용이 없다는 의미이므로 에러 메시지 생성
                        status_msg = make_status_message( # TTS로 출력할 내용이 없다는 상태 메시지 생성
                            "TTS_EMPTY_TEXT",
                            PROC_TTS,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄
                        continue

                    status_msg = make_status_message( # TTS가 말하기 시작했다는 상태 메시지 생성
                        "TTS_SPEAKING",
                        PROC_TTS,
                        PROC_CONTROL_CORE,
                        {
                            KEY_TEXT: text
                        }
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                    success = tts.speak(text) # TTS 객체의 speak() 메서드를 호출해서 text를 음성으로 출력, 이 메서드는 말하기가 완료될 때까지 블로킹됨, 반환값은 성공 여부 (True/False)

                    if success: # 말하기가 성공적으로 완료되었다면 TTS 완료 상태 메시지 생성
                        status_msg = make_status_message( # TTS가 말하기를 완료했다는 상태 메시지 생성
                            "TTS_DONE",
                            PROC_TTS,
                            PROC_CONTROL_CORE,
                            {
                                KEY_TEXT: text
                            }
                        )
                        feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                    else:
                        error_msg = make_error_message( # TTS 말하기 실패 메시지 생성
                            "TTS speak failed",
                            PROC_TTS,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(error_msg) # 에러 메시지를 ControlCore로 보냄

                elif command == CMD_STOP: # 정지 명령
                    if tts is not None: # TTS 객체가 존재하면
                        tts.stop() # TTS 정지, 실제로는 TTS 객체가 말하고 있는지 확인하는 로직이 CTextToSpeech 내부에 있기 때문에 stop() 메서드에서 안전하게 처리될 것임

                    is_tts_ready = False # TTS 준비 상태로 전환

                    status_msg = make_status_message( # TTS가 정지됐다는 상태 메시지 생성
                        "TTS_STOPPED",
                        PROC_TTS,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_EXIT: # 종료 명령
                    if tts is not None: # TTS 객체가 존재하면
                        tts.stop() # TTS 정지

                    is_tts_ready = False # TTS 준비 상태로 전환
                    is_running = False # 프로세스 루프 종료

            time.sleep(0.001) # 루프가 너무 빠르게 도는 것을 방지하기 위해 약간의 짧은 대기 시간 추가

    except Exception:
        ErrorHandler().report()

        error_msg = make_error_message(
            "MPTextToSpeech process exception",
            PROC_TTS,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(error_msg)

    finally:
        if tts is not None: # 프로세스가 종료되기 전에 TTS 객체가 존재한다면
            tts.stop() # TTS 정지
            tts.cleanup() # TTS 객체의 cleanup() 메서드를 호출해서 정리 작업 수행

        status_msg = make_status_message( # MPTextToSpeech 프로세스가 종료됐다는 상태 메시지 생성
            "TTS_PROCESS_TERMINATED",
            PROC_TTS,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

        write_log("MPTextToSpeech process routine terminated.")