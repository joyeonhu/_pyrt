# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPCamera.py
# Project Name : HealthcareRobotPyRT
# Description  : Camera multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT
from Perception.CameraModule import CZEDCameraModule


def proc_camera(command_pipe, feedback_queue, feedback_queue_bk=None):
    # command_pipe : ControlCore에서 명령을 받는 파이프
    # feedback_queue : ControlCore로 프레임 패킷과 상태 메시지를 보내는 큐, feedback_queue_bk는 예비 큐로, feedback_queue가 가득 찼을 때 사용 (현재는 단일 큐만 사용하도록 구현되어 있음, 향후 큐 관리 로직이 개선되면 feedback_queue_bk도 활용할 수 있음)
    """
    Camera Process Routine

    역할:
        1. ControlCore로부터 Pipe로 command 수신
        2. START 명령을 받으면 ZED camera open
        3. frame packet을 읽어서 Queue로 ControlCore에 전달
        4. STOP / EXIT 명령을 받으면 카메라 정지 또는 프로세스 종료
    """

    camera = None # 카메라 객체
    is_running = True # 프로세스 실행 상태
    is_camera_active = False # 카메라 활성 상태

    frame_id = 0 # 프레임 ID

    frame_interval = 1.0 / 30.0 # 30 FPS 목표 (프레임 간격) 즉, 0.033초마다 프레임 하나를 보내겠다는 뜻
    # 프로세스 루프가 너무 빠르게 돌아서 queue가 점점 밀리는 것을 방지하기 위해 프레임 간격을 설정, 실제로는 카메라에서 프레임을 읽는 시간도 있기 때문에 완벽하게 30 FPS를 유지하기는 어려울 수 있음, 하지만 이 방식으로 대략적인 프레임 속도를 제어할 수 있음
    last_frame_time = 0.0 # 마지막 프레임 전송 시각


    write_log("MPCamera process routine started.")

    try:
        while is_running: # 프로세스가 종료 요청을 받을 때까지 반복

            # ------------------------------------------------------------------------------------------------------
            # 1. Command Receive
            # ------------------------------------------------------------------------------------------------------

            if command_pipe.poll(): # ControlCore에서 명령이 도착했는지 확인, poll() : 데이터가 있는지 확인하는 메서드, True이면 recv()로 데이터를 읽을 수 있음
                cmd_msg = command_pipe.recv() # 명령 메시지를 pipe에서 꺼냄

                data = cmd_msg.get(KEY_DATA, {}) # 메시지 안의 data 부분을 꺼냄, 없으면 빈 딕셔너리 사용
                command = data.get(KEY_COMMAND, None) # data 안에서 실제 command를 꺼냄, ex) "START", "STOP", "EXIT" 등, 없으면 None 사용

                if command == CMD_START: # 시작 명령
                    if camera is None:
                        camera = CZEDCameraModule() # 객체 생성

                    if not camera.is_opened(): # 카메라가 열려있지 않으면
                        success = camera.open() # 엶

                        if success: # 열기에 성공하면
                            is_camera_active = True # 카메라 활성화 상태로 전환

                            status_msg = make_status_message( # 카메라 시작됐다는 상태 메시지 생성
                                "CAMERA_STARTED",
                                PROC_CAMERA, # 메시지 출처는 카메라 프로세스
                                PROC_CONTROL_CORE # 메시지 대상은 ControlCore
                            )
                            feedback_queue.put(status_msg) # 상태 메시지를 feedback_queue에 넣어서 ControlCore로 보냄

                        else: # 열기에 실패하면
                            error_msg = make_error_message( # 카메라 열기 실패 메시지 생성
                                "Camera open failed",
                                PROC_CAMERA,
                                PROC_CONTROL_CORE
                            )
                            feedback_queue.put(error_msg)

                elif command == CMD_STOP: # 정지 명령
                    is_camera_active = False # 카메라 비활성화 상태로 전환

                    if camera is not None: # 카메라 객체가 존재하면
                        camera.close() # 카메라 닫음, 실제로는 카메라 객체가 열려있는지 확인하는 로직이 CZEDCameraModule 내부에 있기 때문에 close() 메서드에서 안전하게 처리될 것임

                    status_msg = make_status_message( # 카메라 정지됐다는 상태 메시지 생성
                        "CAMERA_STOPPED",
                        PROC_CAMERA,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg)

                elif command == CMD_EXIT: # 종료 명령
                    is_camera_active = False # 카메라 비활성화 상태로 전환
                    is_running = False # 프로세스 루프 종료

            # ------------------------------------------------------------------------------------------------------
            # 2. Camera Frame Capture
            # ------------------------------------------------------------------------------------------------------

            if is_camera_active and camera is not None: # 카메라가 활성화 상태이고 카메라 객체가 존재하면

                curr_time = time.time() # 현재 시각을 초 단위로 가져옴

                if curr_time - last_frame_time >= frame_interval: # 마지막 프레임 이후 충분한 시간이 지났는지 확인, 처음에는 값이 크니까 바로 프레임 보냄
                    last_frame_time = curr_time # 마지막 프레임 시각 업데이트

                    frame_packet = camera.get_frame_packet() # 카메라에서 프레임 패킷을 가져옴, 프레임 패킷은 이미지 데이터와 함께 필요한 메타데이터

                    if frame_packet is not None: # 프레임 패킷을 성공적으로 가져왔으면
                        frame_id += 1 # 프레임 ID 증가, 각 프레임에 고유한 ID를 부여해서 ControlCore에서 프레임을 추적할 수 있도록 함
                        frame_packet[KEY_FRAME_ID] = frame_id # 프레임 패킷에 프레임 ID 추가

                        msg = make_message( # 프레임 패킷 메시지 생성
                            MSG_TYPE_FRAME,
                            PROC_CAMERA,
                            PROC_CONTROL_CORE,
                            frame_packet
                        )

                        feedback_queue.put(msg) # 프레임 패킷 메시지를 feedback_queue에 넣어서 ControlCore로 보냄

                    else:
                        error_msg = make_error_message( # 프레임 패킷 가져오기 실패 메시지 생성
                            "Failed to get camera frame",
                            PROC_CAMERA,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(error_msg)

            time.sleep(0.001) # 루프가 너무 빠르게 도는 것을 방지하기 위해 약간의 짧은 대기 시간 추가, 실제로는 카메라에서 프레임을 읽는 시간도 있기 때문에 이 정도의 대기만으로도 충분히 CPU 사용량을 낮출 수 있음

    except Exception:
        ErrorHandler().report()

        error_msg = make_error_message(
            "MPCamera process exception",
            PROC_CAMERA,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(error_msg)

    finally:
        if camera is not None: # 프로세스가 종료될 때 카메라 객체가 존재하면
            camera.close() # 카메라 닫음

        status_msg = make_status_message( # 카메라 프로세스가 종료됐다는 상태 메시지 생성
            "CAMERA_PROCESS_TERMINATED",
            PROC_CAMERA,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(status_msg)

        write_log("MPCamera process routine terminated.")