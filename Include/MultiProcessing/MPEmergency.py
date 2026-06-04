# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPEmergency.py
# Project Name : HealthcareRobotPyRT
# Description  : Emergency detection multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT
from HealthcareRobot.EmergencyManager import CEmergencyManager


CMD_CHECK_EMERGENCY = "CHECK_EMERGENCY"


def proc_emergency(command_pipe, feedback_queue, feedback_queue_bk=None):
    """
    Emergency Process Routine

    역할:
        1. ControlCore로부터 Pipe로 command 수신
        2. START 명령을 받으면 EmergencyManager 초기화
        3. CHECK_EMERGENCY 명령을 받으면 frame 1장에 대해 응급 판단
        4. 응급이면 EMERGENCY_DETECTED event를 Queue로 ControlCore에 전달
        5. STOP / EXIT 명령을 받으면 정지 또는 프로세스 종료
    """

    emergency_manager = None # EmergencyManager 객체

    is_running = True # 프로세스 실행 상태
    is_emergency_ready = False # 응급 판단 준비 상태, START 명령을 받으면 True, STOP 명령을 받으면 False

    write_log("MPEmergency process routine started.")

    try:
        while is_running: # 프로세스가 종료 요청을 받을 때까지 반복

            if command_pipe.poll(): # ControlCore에서 명령이 도착했는지 확인
                cmd_msg = command_pipe.recv() # 명령 메시지를 pipe에서 꺼냄

                data = cmd_msg.get(KEY_DATA, {}) # 메시지 안의 data 부분을 꺼냄, 없으면 빈 딕셔너리 사용
                command = data.get(KEY_COMMAND, None) # data 안에서 실제 command를 꺼냄, ex) "START", "CHECK_EMERGENCY", "STOP", "EXIT" 등, 없으면 None 사용

                if command == CMD_START: # 시작 명령
                    if emergency_manager is None: # 객체가 아직 생성되지 않았다면
                        emergency_manager = CEmergencyManager() # EmergencyManager 객체 생성, 이 객체는 응급 상황을 판단하는 기능을 담당

                    is_emergency_ready = True # 응급 판단 준비 상태로 전환

                    status_msg = make_status_message( # 응급 판단 준비됐다는 상태 메시지 생성
                        "EMERGENCY_READY",
                        PROC_EMERGENCY,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_CHECK_EMERGENCY: # 응급 판단 명령, 이 명령이 들어오면 EmergencyManager가 frame을 분석해서 응급 상황인지 판단하는 작업을 수행
                    if not is_emergency_ready or emergency_manager is None: # EmergencyManager가 준비되지 않았거나 객체가 생성되지 않았다면
                        error_msg = make_error_message( # EmergencyManager가 준비되지 않았다는 에러 메시지 생성
                            "EmergencyManager is not ready",
                            PROC_EMERGENCY,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(error_msg) # 상태 메시지를 ControlCore로 보냄
                        continue

                    frame_bgr = data.get(KEY_FRAME, None) # data 안에서 frame을 꺼냄, 없으면 None 사용, frame은 BGR 형식의 이미지 데이터여야 함, EmergencyManager는 이 이미지를 분석해서 응급 상황인지 판단

                    if frame_bgr is None: # frame이 None이면 에러 메시지 생성
                        status_msg = make_status_message( # frame이 없어서 응급 판단을 할 수 없다는 상태 메시지 생성
                            "EMERGENCY_NO_FRAME",
                            PROC_EMERGENCY,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄
                        continue

                    status_msg = make_status_message( # 응급 판단을 시작했다는 상태 메시지 생성
                        "EMERGENCY_CHECKING",
                        PROC_EMERGENCY,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                    result = emergency_manager.process(frame_bgr) # EmergencyManager 객체의 process() 메서드를 호출해서 frame을 분석하고 응급 상황인지 판단, 이 메서드는 분석이 완료될 때까지 블로킹됨, 반환값은 분석 결과를 담은 딕셔너리, 예: {"is_emergency": True, "emergency_type": "FALL", "confidence": 0.95} 또는 {"is_emergency": False}

                    result_msg = make_message( # 응급 판단 결과 메시지 생성, 이 메시지는 응급 상황 여부와 그에 대한 정보를 담고 있음
                        MSG_TYPE_EMERGENCY,
                        PROC_EMERGENCY,
                        PROC_CONTROL_CORE,
                        result
                    )
                    feedback_queue.put(result_msg) # 응급 판단 결과 메시지를 ControlCore로 보냄

                    if result.get("is_emergency", False): # 분석 결과에서 is_emergency 키가 True이면 응급 상황이 감지된 것이므로 EMERGENCY_DETECTED 이벤트 메시지 생성
                        event_msg = make_event_message( #  응급 상황이 감지되었다는 이벤트 메시지 생성, 이 메시지는 응급 상황의 유형과 신뢰도 등의 정보를 담고 있음
                            "EMERGENCY_DETECTED",
                            PROC_EMERGENCY,
                            PROC_CONTROL_CORE,
                            {
                                KEY_EMERGENCY_TYPE: result.get("emergency_type", None),
                                KEY_CONFIDENCE: result.get("confidence", 0.0),
                                "caption": result.get("caption", None),
                            }
                        )
                        feedback_queue.put(event_msg) # EMERGENCY_DETECTED 이벤트 메시지를 ControlCore로 보냄

                elif command == CMD_STOP: # 정지 명령
                    is_emergency_ready = False # 응급 판단 준비 상태로 전환

                    if emergency_manager is not None: # EmergencyManager 객체가 존재하면
                        emergency_manager.reset() # EmergencyManager 객체의 reset() 메서드를 호출해서 내부 상태 초기화, 이 메서드는 EmergencyManager가 다음에 START 명령을 받을 때 다시 정상적으로 작동할 수 있도록 준비하는 역할을 함

                    status_msg = make_status_message( # 응급 판단이 정지됐다는 상태 메시지 생성
                        "EMERGENCY_STOPPED",
                        PROC_EMERGENCY,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_EXIT: # 종료 명령
                    is_emergency_ready = False # 응급 판단 준비 상태로 전환
                    is_running = False # 프로세스 루프 종료

            time.sleep(0.001) # 루프가 너무 빠르게 도는 것을 방지하기 위해 약간의 짧은 대기 시간 추가

    except Exception:
        ErrorHandler().report()

        error_msg = make_error_message(
            "MPEmergency process exception",
            PROC_EMERGENCY,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(error_msg)

    finally:
        if emergency_manager is not None: # EmergencyManager 객체가 존재하면
            emergency_manager.reset() # EmergencyManager 객체의 reset() 메서드를 호출해서 내부 상태 초기화

        status_msg = make_status_message( # MPEmergency 프로세스가 종료된다는 상태 메시지 생성
            "EMERGENCY_PROCESS_TERMINATED",
            PROC_EMERGENCY,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

        write_log("MPEmergency process routine terminated.")