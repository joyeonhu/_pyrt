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
    """
    Camera Process Routine

    역할:
        1. ControlCore로부터 Pipe로 command 수신
        2. START 명령을 받으면 ZED camera open
        3. frame packet을 읽어서 Queue로 ControlCore에 전달
        4. STOP / EXIT 명령을 받으면 카메라 정지 또는 프로세스 종료
    """

    camera = None
    is_running = True
    is_camera_active = False

    frame_interval = 1.0 / 30.0
    last_frame_time = 0.0

    write_log("MPCamera process routine started.")

    try:
        while is_running:

            # ------------------------------------------------------------------------------------------------------
            # 1. Command Receive
            # ------------------------------------------------------------------------------------------------------

            if command_pipe.poll():
                cmd_msg = command_pipe.recv()

                data = cmd_msg.get(KEY_DATA, {})
                command = data.get(KEY_COMMAND, None)

                if command == CMD_START:
                    if camera is None:
                        camera = CZEDCameraModule()

                    if not camera.is_opened():
                        success = camera.open()

                        if success:
                            is_camera_active = True

                            status_msg = make_status_message(
                                "CAMERA_STARTED",
                                PROC_CAMERA,
                                PROC_CONTROL_CORE
                            )
                            feedback_queue.put(status_msg)

                        else:
                            error_msg = make_error_message(
                                "Camera open failed",
                                PROC_CAMERA,
                                PROC_CONTROL_CORE
                            )
                            feedback_queue.put(error_msg)

                elif command == CMD_STOP:
                    is_camera_active = False

                    if camera is not None:
                        camera.close()

                    status_msg = make_status_message(
                        "CAMERA_STOPPED",
                        PROC_CAMERA,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg)

                elif command == CMD_EXIT:
                    is_camera_active = False
                    is_running = False

            # ------------------------------------------------------------------------------------------------------
            # 2. Camera Frame Capture
            # ------------------------------------------------------------------------------------------------------

            if is_camera_active and camera is not None:

                curr_time = time.time()

                if curr_time - last_frame_time >= frame_interval:
                    last_frame_time = curr_time

                    frame_packet = camera.get_frame_packet()

                    if frame_packet is not None:
                        msg = make_message(
                            MSG_TYPE_FRAME,
                            PROC_CAMERA,
                            PROC_CONTROL_CORE,
                            frame_packet
                        )

                        feedback_queue.put(msg)

                    else:
                        error_msg = make_error_message(
                            "Failed to get camera frame",
                            PROC_CAMERA,
                            PROC_CONTROL_CORE
                        )
                        feedback_queue.put(error_msg)

            time.sleep(0.001)

    except Exception:
        ErrorHandler().report()

        error_msg = make_error_message(
            "MPCamera process exception",
            PROC_CAMERA,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(error_msg)

    finally:
        if camera is not None:
            camera.close()

        status_msg = make_status_message(
            "CAMERA_PROCESS_TERMINATED",
            PROC_CAMERA,
            PROC_CONTROL_CORE
        )
        feedback_queue.put(status_msg)

        write_log("MPCamera process routine terminated.")