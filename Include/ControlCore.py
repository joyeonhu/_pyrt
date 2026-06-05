# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ControlCore.py
# Project Name : HealthcareRobotPyRT
# Description  : Main control core for healthcare robot
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from HealthcareRobot.HealthcareState import *

from MultiProcessing.MultiProcessBase import (
    CMultiProcessBase,
    CMD_START,
    CMD_STOP,
    CMD_EXIT,
)

from MultiProcessing.MPCamera import proc_camera
from MultiProcessing.MPSpeechToText import proc_speech_to_text, CMD_LISTEN_ONCE
from MultiProcessing.MPTextToSpeech import proc_text_to_speech, CMD_SPEAK
from MultiProcessing.MPLLM import proc_llm, CMD_PARSE_TEXT
from MultiProcessing.MPEmergency import proc_emergency, CMD_CHECK_EMERGENCY
from MultiProcessing.MPHealthcareROS import (
    proc_healthcare_ros,
    CMD_PROCESS_FOLLOW,
    CMD_START_GUIDE,
    CMD_START_DELIVERY,
    CMD_GO_TO_DESTINATION,
    CMD_STOP_ROBOT,
)


class CControlCore:
    """
    Healthcare Robot ControlCore

    역할:
        1. child process 생성 / 시작 / 종료
        2. Pipe로 command 전송
        3. Queue feedback 수신
        4. FSM 상태 관리
        5. 로봇 기능 실행 흐름 제어
    """

    def __init__(self):

        self._state = STATE_START
        self._prev_state = None
        self._last_frame_packet = None
        self._last_stt_text = None
        self._last_llm_command = None
        self._is_running = False

        self._mp_camera = None
        self._mp_stt = None
        self._mp_tts = None
        self._mp_llm = None
        self._mp_emergency = None
        self._mp_ros = None

        self.create_processes()

        write_log("ControlCore initialized.", self)

    # ==============================================================================================================
    # Process Create
    # ==============================================================================================================

    def create_processes(self):

        self._mp_camera = CMultiProcessBase(PROC_CAMERA, proc_camera)
        self._mp_stt = CMultiProcessBase(PROC_STT, proc_speech_to_text)
        self._mp_tts = CMultiProcessBase(PROC_TTS, proc_text_to_speech)
        self._mp_llm = CMultiProcessBase(PROC_LLM, proc_llm)
        self._mp_emergency = CMultiProcessBase(PROC_EMERGENCY, proc_emergency)
        self._mp_ros = CMultiProcessBase(PROC_HEALTHCARE, proc_healthcare_ros)

        self._mp_camera.sig_queue_bcast.connect(self.on_camera_feedback)
        self._mp_stt.sig_queue_bcast.connect(self.on_stt_feedback)
        self._mp_tts.sig_queue_bcast.connect(self.on_tts_feedback)
        self._mp_llm.sig_queue_bcast.connect(self.on_llm_feedback)
        self._mp_emergency.sig_queue_bcast.connect(self.on_emergency_feedback)
        self._mp_ros.sig_queue_bcast.connect(self.on_ros_feedback)

        for mp_proc in self.get_process_list():
            mp_proc.sig_error.connect(self.on_process_error)

    # ==============================================================================================================
    # Start / Stop
    # ==============================================================================================================

    def start(self):

        self._is_running = True

        for mp_proc in self.get_process_list():
            mp_proc.start()

        time.sleep(0.2)

        self._mp_camera.send_command(CMD_START)
        self._mp_stt.send_command(CMD_START)
        self._mp_tts.send_command(CMD_START)
        self._mp_llm.send_command(CMD_START)
        self._mp_emergency.send_command(CMD_START)
        self._mp_ros.send_command(CMD_START)

        self.handle_event(EVT_STARTUP_DONE)

        write_log("ControlCore started.", self)

    def stop(self):

        self._is_running = False

        try:
            if self._mp_ros is not None:
                self._mp_ros.send_command(CMD_STOP_ROBOT)

            for mp_proc in self.get_process_list():
                mp_proc.send_command(CMD_STOP)

            time.sleep(0.2)

            for mp_proc in self.get_process_list():
                mp_proc.release()

        except Exception:
            ErrorHandler().report()

        write_log("ControlCore stopped.", self)

    # ==============================================================================================================
    # Main Loop
    # ==============================================================================================================

    def process_once(self):

        if not self._is_running:
            return

        if self._state == STATE_FOLLOW:
            self.process_follow_once()

        if self._last_frame_packet is not None:
            self.request_emergency_check(self._last_frame_packet)

    # ==============================================================================================================
    # Feedback Handlers
    # ==============================================================================================================

    def on_camera_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_FRAME:
            self._last_frame_packet = msg.get(KEY_DATA, {})

        elif msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_CAMERA, msg)

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_CAMERA, msg)

    def on_stt_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_TEXT:
            text = msg.get(KEY_TEXT, None)

            if text is None:
                text = msg.get(KEY_DATA, {}).get(KEY_TEXT, None)

            if text is None:
                return

            self._last_stt_text = text

            write_log("STT TEXT | %s" % str(text), self)

            self._mp_llm.send_command(
                CMD_PARSE_TEXT,
                {
                    KEY_TEXT: text
                }
            )

        elif msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_STT, msg)

            status = self.get_msg_status(msg)

            if status == "STT_NO_INPUT" and self._state == STATE_IDLE:
                self.request_stt_once()

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_STT, msg)

    def on_tts_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_TTS, msg)

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_TTS, msg)

    def on_llm_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_COMMAND:
            data = msg.get(KEY_DATA, {})

            command = data.get(KEY_COMMAND, None)
            destination_id = data.get("destination_id", None)

            self._last_llm_command = data

            write_log(
                "LLM COMMAND | command=%s | destination=%s"
                % (
                    str(command),
                    str(destination_id)
                ),
                self
            )

            if command == "FOLLOW":
                self.handle_event(EVT_CMD_FOLLOW)

            elif command == "GUIDE":
                self.handle_event(EVT_CMD_GUIDE, {"destination_id": destination_id})

            elif command == "DELIVERY":
                self.handle_event(EVT_CMD_DELIVERY, {"destination_id": destination_id})

            elif command == "HOMING":
                self.handle_event(EVT_CMD_HOMING, {"destination_id": "home"})

            elif command == "NONE":
                self.speak("I did not understand the command.")
                self.request_stt_once()

        elif msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_LLM, msg)

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_LLM, msg)

    def on_emergency_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_EMERGENCY:
            data = msg.get(KEY_DATA, {})

            if data.get("is_emergency", False):
                self.handle_event(
                    EVT_EMERGENCY_DETECTED,
                    {
                        KEY_EMERGENCY_TYPE: data.get(KEY_EMERGENCY_TYPE, data.get("emergency_type", None)),
                        KEY_CONFIDENCE: data.get(KEY_CONFIDENCE, data.get("confidence", 0.0)),
                    }
                )

        elif msg_type == MSG_TYPE_EVENT:
            event = msg.get(KEY_EVENT, None)
            data = msg.get(KEY_DATA, {})
            self.handle_event(event, data)

        elif msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_EMERGENCY, msg)

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_EMERGENCY, msg)

    def on_ros_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_EVENT:
            event = msg.get(KEY_EVENT, None)

            if event == "NAVIGATION_ARRIVED":
                self.handle_navigation_arrived()

            elif event == "NAVIGATION_FAILED":
                self.handle_navigation_failed()

            else:
                self.handle_event(event, msg.get(KEY_DATA, {}))

        elif msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_HEALTHCARE, msg)

        elif msg_type == MSG_TYPE_CMD_VEL:
            pass

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_HEALTHCARE, msg)

    def on_process_error(self, error_msg: str):
        write_log("Process error: %s" % str(error_msg), self)

    # ==============================================================================================================
    # FSM
    # ==============================================================================================================

    def handle_event(self, event: str, data: dict = None):

        if event is None:
            return

        if data is None:
            data = {}

        write_log(
            "EVENT | state=%s | event=%s"
            % (
                self._state,
                event
            ),
            self
        )

        if event == EVT_EMERGENCY_DETECTED:
            self.enter_emergency(data)
            return

        if self._state == STATE_START:

            if event == EVT_STARTUP_DONE:
                self.change_state(STATE_IDLE)
                self.request_stt_once()

        elif self._state == STATE_IDLE:

            if event == EVT_CMD_FOLLOW:
                self.enter_follow()

            elif event == EVT_CMD_GUIDE:
                self.enter_guide(data)

            elif event == EVT_CMD_DELIVERY:
                self.enter_delivery(data)

            elif event == EVT_CMD_HOMING:
                self.enter_homing()

            elif event == EVT_GO_IDLE:
                self.enter_idle()

        elif self._state == STATE_FOLLOW:

            if event == EVT_GO_IDLE:
                self.enter_idle()

            elif event == EVT_CMD_GUIDE:
                self.enter_guide(data)

            elif event == EVT_FOLLOW_FAILED:
                self.enter_idle()

        elif self._state == STATE_GUIDE:

            if event in {EVT_GUIDE_DONE, EVT_GO_IDLE, EVT_GUIDE_FAILED}:
                self.enter_idle()

        elif self._state == STATE_DELIVERY:

            if event in {EVT_DELIVERY_DONE, EVT_GO_IDLE, EVT_DELIVERY_FAILED}:
                self.enter_idle()

        elif self._state == STATE_HOMING:

            if event in {EVT_HOMING_DONE, EVT_GO_IDLE, EVT_HOMING_FAILED}:
                self.enter_idle()

        elif self._state == STATE_EMERGENCY:

            if event in {EVT_EMERGENCY_CLEARED, EVT_GO_IDLE}:
                self.enter_idle()

    def change_state(self, new_state: str):

        if self._state == new_state:
            return

        prev = self._state

        self._prev_state = prev
        self._state = new_state

        write_log(
            "STATE CHANGE | %s -> %s"
            % (
                prev,
                new_state
            ),
            self
        )

    # ==============================================================================================================
    # State Enter
    # ==============================================================================================================

    def enter_idle(self):

        self._mp_ros.send_command(CMD_STOP_ROBOT)

        self.change_state(STATE_IDLE)

        self.request_stt_once()

    def enter_follow(self):

        self.change_state(STATE_FOLLOW)

        self.speak("I will follow you.")

    def enter_guide(self, data: dict):

        destination_id = data.get("destination_id", None)

        if destination_id is None:
            self.speak("I do not know the destination.")
            self.enter_idle()
            return

        self.change_state(STATE_GUIDE)

        self.speak("I will guide you.")

        self._mp_ros.send_command(
            CMD_START_GUIDE,
            {
                "destination_id": destination_id
            }
        )

    def enter_delivery(self, data: dict):

        destination_id = data.get("destination_id", None)

        if destination_id is None:
            self.speak("I do not know the delivery destination.")
            self.enter_idle()
            return

        self.change_state(STATE_DELIVERY)

        self.speak("Starting delivery.")

        self._mp_ros.send_command(
            CMD_START_DELIVERY,
            {
                "destination_id": destination_id
            }
        )

    def enter_homing(self):

        self.change_state(STATE_HOMING)

        self.speak("Returning home.")

        self._mp_ros.send_command(
            CMD_GO_TO_DESTINATION,
            {
                "destination_id": "home"
            }
        )

    def enter_emergency(self, data: dict):

        if self._state == STATE_EMERGENCY:
            return

        self._mp_ros.send_command(CMD_STOP_ROBOT)

        emergency_type = data.get(KEY_EMERGENCY_TYPE, "unknown")

        self.change_state(STATE_EMERGENCY)

        self.speak("Emergency detected. Please check the patient.")

        write_log(
            "EMERGENCY | type=%s"
            % str(emergency_type),
            self
        )

    # ==============================================================================================================
    # Requests
    # ==============================================================================================================

    def request_stt_once(self):

        if self._state == STATE_EMERGENCY:
            return

        self._mp_stt.send_command(CMD_LISTEN_ONCE)

    def request_emergency_check(self, frame_packet: dict):

        if frame_packet is None:
            return

        frame_bgr = frame_packet.get("frame_bgr", None)

        if frame_bgr is None:
            return

        self._mp_emergency.send_command(
            CMD_CHECK_EMERGENCY,
            {
                KEY_FRAME: frame_bgr
            }
        )

    def process_follow_once(self):

        if self._last_frame_packet is None:
            return

        frame_bgr = self._last_frame_packet.get("frame_bgr", None)
        depth_map = self._last_frame_packet.get("depth_map", None)

        if frame_bgr is None:
            return

        self._mp_ros.send_command(
            CMD_PROCESS_FOLLOW,
            {
                KEY_FRAME: frame_bgr,
                "depth_map": depth_map
            }
        )

    def speak(self, text: str):

        self._mp_tts.send_command(
            CMD_SPEAK,
            {
                KEY_TEXT: text
            }
        )

    # ==============================================================================================================
    # Navigation Result
    # ==============================================================================================================

    def handle_navigation_arrived(self):

        if self._state == STATE_GUIDE:
            self.handle_event(EVT_GUIDE_DONE)

        elif self._state == STATE_DELIVERY:
            self.handle_event(EVT_DELIVERY_DONE)

        elif self._state == STATE_HOMING:
            self.handle_event(EVT_HOMING_DONE)

    def handle_navigation_failed(self):

        if self._state == STATE_GUIDE:
            self.handle_event(EVT_GUIDE_FAILED)

        elif self._state == STATE_DELIVERY:
            self.handle_event(EVT_DELIVERY_FAILED)

        elif self._state == STATE_HOMING:
            self.handle_event(EVT_HOMING_FAILED)

    # ==============================================================================================================
    # Log Utils
    # ==============================================================================================================

    @staticmethod
    def get_msg_status(msg: dict):

        status = msg.get(KEY_STATUS, None)

        if status is None:
            status = msg.get(KEY_DATA, {}).get(KEY_STATUS, None)

        return status

    def log_status(self, source: str, msg: dict):

        status = self.get_msg_status(msg)

        write_log(
            "STATUS | source=%s | status=%s"
            % (
                str(source),
                str(status)
            ),
            self
        )

    @staticmethod
    def get_msg_error(msg: dict):

        error_msg = msg.get(KEY_ERROR, None)

        if error_msg is None:
            error_msg = msg.get(KEY_DATA, {}).get(KEY_ERROR, None)

        return error_msg

    def log_error(self, source: str, msg: dict):

        error_msg = self.get_msg_error(msg)

        write_log(
            "ERROR | source=%s | error=%s"
            % (
                str(source),
                str(error_msg)
            ),
            self
        )

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_process_list(self):
        return [
            self._mp_camera,
            self._mp_stt,
            self._mp_tts,
            self._mp_llm,
            self._mp_emergency,
            self._mp_ros,
        ]

    def get_state(self):
        return self._state

    def get_prev_state(self):
        return self._prev_state

    def get_last_stt_text(self):
        return self._last_stt_text

    def get_last_llm_command(self):
        return self._last_llm_command