# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPCamera.py
# Project Name : HealthcareRobotPyRT
# Description  : Camera multiprocessing module
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from MultiProcessing.MultiProcessBase import CMultiProcessBase
from Perception.CameraModule import CZEDCameraModule


class CMPCamera(CMultiProcessBase):
    """
    Camera Process

    역할:
        1. ZED 2i 카메라 실행
        2. frame / depth_map 획득
        3. ControlCore로 frame packet 전송

    통신:
        Command  : ControlCore -> MPCamera, Pipe
        Feedback : MPCamera -> ControlCore, Queue
    """

    CMD_START_CAMERA = "START_CAMERA"
    CMD_STOP_CAMERA = "STOP_CAMERA"
    CMD_EXIT = "EXIT"

    def __init__(
            self,
            command_pipe=None,
            feedback_queue=None,
            fps: int = 15
    ):
        super().__init__(
            PROC_CAMERA,
            command_pipe,
            feedback_queue
        )

        self._fps = fps
        self._period = 1.0 / float(fps)

        self._camera = None
        self._camera_enabled = True

        self._last_frame_time = 0.0

    # ==============================================================================================================
    # Start / Stop
    # ==============================================================================================================

    def on_start(self):
        """
        프로세스 시작 시 카메라 open
        """

        self._camera = CZEDCameraModule(
            fps=self._fps
        )

        success = self._camera.open()

        if success:
            self.send_status("CAMERA_READY")
        else:
            self.send_error("CAMERA_OPEN_FAILED")
            self.stop()

    def on_stop(self):
        """
        프로세스 종료 시 카메라 close
        """

        if self._camera is not None:
            self._camera.close()

        self.send_status("CAMERA_STOPPED")

    # ==============================================================================================================
    # Main Loop
    # ==============================================================================================================

    def process_once(self):
        """
        카메라 프로세스 반복 작업
        """

        self.process_command()

        if not self._camera_enabled:
            time.sleep(0.01)
            return

        now = time.time()

        if now - self._last_frame_time < self._period:
            return

        self._last_frame_time = now

        frame_packet = self._camera.get_frame_packet()

        if frame_packet is None:
            return

        msg = make_message(
            MSG_TYPE_FRAME,
            PROC_CAMERA,
            PROC_CONTROL_CORE,
            frame_packet
        )

        self.send_feedback(msg)

    # ==============================================================================================================
    # Command
    # ==============================================================================================================

    def process_command(self):
        """
        ControlCore에서 Pipe로 보낸 command 처리
        """

        command_msg = self.recv_command()

        if command_msg is None:
            return

        command = self.get_command_type(command_msg)

        if command == self.CMD_START_CAMERA:
            self._camera_enabled = True
            self.send_status("CAMERA_STARTED")

        elif command == self.CMD_STOP_CAMERA:
            self._camera_enabled = False
            self.send_status("CAMERA_PAUSED")

        elif command == self.CMD_EXIT:
            self.send_status("CAMERA_EXIT_REQUESTED")
            self.stop()

        else:
            self.send_status(
                "UNKNOWN_CAMERA_COMMAND",
                {
                    "command": command
                }
            )