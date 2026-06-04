# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPLLM.py
# Project Name : HealthcareRobotPyRT
# Description  : LLM multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT
from HealthcareRobot.LLMManager import CLLMManager


CMD_PARSE_TEXT = "PARSE_TEXT"


def proc_llm(command_pipe, feedback_queue, feedback_queue_bk=None):
    """
    LLM Process Routine

    역할:
        1. ControlCore로부터 Pipe로 command 수신
        2. START 명령을 받으면 LLM 모델 로드
        3. PARSE_TEXT 명령을 받으면 STT 텍스트를 로봇 command로 변환
        4. 결과를 Queue로 ControlCore에 전달
        5. STOP / EXIT 명령을 받으면 정지 또는 프로세스 종료
    """

    llm = None # LLM 매니저 객체

    is_running = True # 프로세스 실행 상태
    is_llm_ready = False # LLM 준비 상태, START 명령을 받으면 True, STOP 명령을 받으면 False

    write_log("MPLLM process routine started.")

    try:
        while is_running: # 프로세스가 종료 요청을 받을 때까지 반복

            if command_pipe.poll(): # ControlCore에서 명령이 도착했는지 확인
                cmd_msg = command_pipe.recv() # 명령 메시지를 pipe에서 꺼냄

                data = cmd_msg.get(KEY_DATA, {}) # 메시지 안의 data 부분을 꺼냄, 없으면 빈 딕셔너리 사용
                command = data.get(KEY_COMMAND, None) # data 안에서 실제 command를 꺼냄, ex) "START", "PARSE_TEXT", "STOP", "EXIT" 등, 없으면 None 사용

                if command == CMD_START: # 시작 명령
                    if llm is None: # 객체가 아직 생성되지 않았다면
                        llm = CLLMManager() # LLM 매니저 객체 생성, 이 객체는 LLM 모델을 로드하고 텍스트를 파싱하는 기능을 담당

                    is_llm_ready = True # LLM 준비 상태로 전환

                    status_msg = make_status_message( # LLM 준비됐다는 상태 메시지 생성
                        "LLM_READY",
                        PROC_LLM,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_PARSE_TEXT: # 텍스트 파싱 명령, 이 명령이 들어오면 LLM이 text를 로봇 command로 변환하는 작업을 수행
                    if not is_llm_ready or llm is None: # LLM이 준비되지 않았거나 객체가 생성되지 않았다면
                        error_msg = make_error_message( # LLM이 준비되지 않았다는 에러 메시지 생성
                            "LLM is not ready",
                            PROC_LLM,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(error_msg) # 에러 메시지를 ControlCore로 보냄
                        continue

                    text = data.get(KEY_TEXT, None) # data 안에서 text를 꺼냄, 없으면 None 사용

                    if text is None or len(str(text).strip()) == 0: # text가 None이거나 빈 문자열이면 LLM으로 파싱할 내용이 없다는 의미이므로 에러 메시지 생성
                        status_msg = make_status_message( # LLM으로 파싱할 내용이 없다는 상태 메시지 생성
                            "LLM_EMPTY_TEXT",
                            PROC_LLM,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄
                        continue

                    status_msg = make_status_message( # LLM이 텍스트를 파싱하기 시작했다는 상태 메시지 생성
                        "LLM_PARSING",
                        PROC_LLM,
                        PROC_CONTROL_CORE,
                        {
                            KEY_TEXT: text
                        }
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                    command_result = llm.parse_text(text) # LLM 매니저 객체의 parse_text() 메서드를 호출해서 text를 로봇 command로 변환, 이 메서드는 파싱이 완료될 때까지 블로킹됨, 반환값은 파싱 결과 (예: 로봇이 수행할 명령 리스트) 또는 None

                    result_msg = make_message( # LLM 파싱 결과 메시지 생성
                        MSG_TYPE_COMMAND,
                        PROC_LLM,
                        PROC_CONTROL_CORE,
                        command_result
                    )
                    feedback_queue.put(result_msg) # LLM 파싱 결과 메시지를 ControlCore로 보냄

                elif command == CMD_STOP: # 정지 명령
                    is_llm_ready = False # LLM 준비 상태로 전환

                    status_msg = make_status_message( # LLM 정지됐다는 상태 메시지 생성
                        "LLM_STOPPED",
                        PROC_LLM,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_EXIT: # 종료 명령
                    is_llm_ready = False # LLM 준비 상태로 전환
                    is_running = False # 프로세스 루프 종료

            time.sleep(0.001) # 루프가 너무 빠르게 도는 것을 방지하기 위해 약간의 짧은 대기 시간 추가

    except Exception:
        ErrorHandler().report()

        error_msg = make_error_message(
            "MPLLM process exception",
            PROC_LLM,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(error_msg)

    finally:
        status_msg = make_status_message( # MPLLM 프로세스가 종료된다는 상태 메시지 생성
            "LLM_PROCESS_TERMINATED",
            PROC_LLM,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

        write_log("MPLLM process routine terminated.")