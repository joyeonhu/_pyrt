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
)

from MultiProcessing.MPCamera import proc_camera
from MultiProcessing.MPSTT import proc_speech_to_text, CMD_LISTEN_ONCE
from MultiProcessing.MPTTS import proc_text_to_speech, CMD_SPEAK
from MultiProcessing.MPLLM import proc_llm, CMD_PARSE_TEXT
from MultiProcessing.MPEmergency import proc_emergency, CMD_CHECK_EMERGENCY
from MultiProcessing.MPRobotTask import (
    proc_robot_task,
    CMD_PROCESS_FOLLOW,
    CMD_START_GUIDE,
    CMD_START_DELIVERY,
    CMD_GO_TO_DESTINATION,
    CMD_STOP_ROBOT,
    CMD_CHECK_PATIENT_ON_PATH,
    CMD_CHECK_DOOR_ARRIVAL,
)


class CControlCore:
    """
    Healthcare Robot ControlCore
    """

    def __init__(self):

        self._state = RobotState.START # 로봇 현재 상태
        self._prev_state = None # 로봇 이전 상태
        self._is_running = False # ControlCore 메인 루프 실행 여부

        self._last_frame_packet = None # 카메라에서 받은 마지막 프레임 패킷 (frame_bgr, depth_map 등 포함)
        self._last_stt_text = None # STT에서 받은 마지막 텍스트
        self._last_llm_command = None # LLM에서 받은 마지막 명령 (command, destination_id 등 포함)

        self._delivery_destination_id = None # 현재 배달 목적지 ID (예: 병실 번호), DELIVERY 및 DELIVERY_VERIFY 상태에서 사용
        self._greeted_patient = False # 환자에게 인사했는지 여부, DELIVERY 상태에서 환자에게 한 번만 인사하기 위해 사용

        self._last_follow_frame_id = None  # Follow 상태에서 마지막으로 처리한 프레임 ID
        self._last_emergency_frame_id = None # 응급 상황 판단을 위해 마지막으로 처리한 프레임 ID
        self._last_patient_check_frame_id = None # 환자 확인을 위해 마지막으로 처리한 프레임 ID

        self._door_check_requested = False # 문 도착 확인 요청 여부

        self._mp_camera = None # 카메라 프로세스
        self._mp_stt = None # 음성인식 프로세스
        self._mp_tts = None # 음성합성 프로세스
        self._mp_llm = None # LLM 프로세스
        self._mp_emergency = None # 응급 상황 판단 프로세스
        self._mp_robot_task = None # 로봇 작업 처리 프로세스 (Follow, Guide, Delivery 등)

        self.create_processes() # 각 프로세스 생성 및 초기화

        write_log("ControlCore initialized.", self)

    # ==============================================================================================================
    # Process Create
    # ==============================================================================================================

    def create_processes(self): # 각 프로세스 생성 및 초기화, 프로세스 간 신호 연결

        self._mp_camera = CMultiProcessBase(PROC_CAMERA, proc_camera)
        self._mp_stt = CMultiProcessBase(PROC_STT, proc_speech_to_text)
        self._mp_tts = CMultiProcessBase(PROC_TTS, proc_text_to_speech)
        self._mp_llm = CMultiProcessBase(PROC_LLM, proc_llm)
        self._mp_emergency = CMultiProcessBase(PROC_EMERGENCY, proc_emergency)
        self._mp_robot_task = CMultiProcessBase(PROC_HEALTHCARE, proc_robot_task)

        self._mp_camera.sig_queue_bcast.connect(self.on_camera_feedback)
        self._mp_stt.sig_queue_bcast.connect(self.on_stt_feedback)
        self._mp_tts.sig_queue_bcast.connect(self.on_tts_feedback)
        self._mp_llm.sig_queue_bcast.connect(self.on_llm_feedback)
        self._mp_emergency.sig_queue_bcast.connect(self.on_emergency_feedback)
        self._mp_robot_task.sig_queue_bcast.connect(self.on_robot_task_feedback)

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
        self._mp_robot_task.send_command(CMD_START)

        self.handle_event(RobotEvent.STARTUP_DONE)

        write_log("ControlCore started.", self)

    def stop(self):

        self._is_running = False

        try:
            if self._mp_robot_task is not None:
                self._mp_robot_task.send_command(CMD_STOP_ROBOT)

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

        if self._last_frame_packet is not None:
            self.request_emergency_check()

        if self._state == RobotState.FOLLOW:
            self.process_follow_once()

        elif self._state == RobotState.DELIVERY:
            self.request_patient_check_on_path()

        elif self._state == RobotState.DELIVERY_VERIFY:
            self.request_door_arrival_check()

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
            text = self.get_msg_text(msg)

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

            if status == "STT_NO_INPUT" and self._state == RobotState.IDLE:
                self.request_stt_once()

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_STT, msg)

    def on_tts_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_TTS, msg)

            status = self.get_msg_status(msg)

            if status == "TTS_DONE" and self._state == RobotState.IDLE:
                self.request_stt_once()

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_TTS, msg)

    def on_llm_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_COMMAND:
            data = msg.get(KEY_DATA, {})

            command = data.get(KEY_COMMAND, None)
            destination_id = data.get(KEY_DESTINATION_ID, None)

            self._last_llm_command = data

            write_log(
                "LLM COMMAND | command=%s | destination=%s"
                % (str(command), str(destination_id)),
                self
            )

            if command == "FOLLOW":
                self.handle_event(RobotEvent.CMD_FOLLOW)

            elif command == "GUIDE":
                self.handle_event(
                    RobotEvent.CMD_GUIDE,
                    {
                        KEY_DESTINATION_ID: destination_id
                    }
                )

            elif command == "DELIVERY":
                self.handle_event(
                    RobotEvent.CMD_DELIVERY,
                    {
                        KEY_DESTINATION_ID: destination_id
                    }
                )

            elif command == "HOMING":
                self.handle_event(
                    RobotEvent.CMD_HOMING,
                    {
                        KEY_DESTINATION_ID: "home"
                    }
                )

            elif command == "NONE":
                self.speak("I did not understand the command.")

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
            is_emergency = data.get("is_emergency", False)

            if is_emergency:
                self.handle_event(
                    RobotEvent.EMERGENCY_DETECTED,
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

    def on_robot_task_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_EVENT:
            event = msg.get(KEY_EVENT, None)
            data = msg.get(KEY_DATA, {})
            self.handle_event(event, data)

        elif msg_type == MSG_TYPE_DETECTION:
            self.handle_detection_message(msg)

        elif msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_HEALTHCARE, msg)

        elif msg_type == MSG_TYPE_CMD_VEL:
            pass

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_HEALTHCARE, msg)

    def on_process_error(self, error_msg: str):
        write_log("Process error: %s" % str(error_msg), self)

    # ==============================================================================================================
    # Detection Message
    # ==============================================================================================================

    def handle_detection_message(self, msg: dict):

        destination_id = msg.get(KEY_DESTINATION_ID, None)
        door_text = msg.get(KEY_DOOR_TEXT, None)
        is_matched = msg.get(KEY_IS_MATCHED, None)

        if destination_id is None:
            data = msg.get(KEY_DATA, {})
            destination_id = data.get(KEY_DESTINATION_ID, None)
            door_text = data.get(KEY_DOOR_TEXT, None)
            is_matched = data.get(KEY_IS_MATCHED, None)

        write_log(
            "DETECTION | destination=%s | door_text=%s | matched=%s"
            % (str(destination_id), str(door_text), str(is_matched)),
            self
        )

        if self._state != RobotState.DELIVERY_VERIFY:
            return

        if is_matched:
            self.handle_event(
                RobotEvent.DELIVERY_VERIFY_DONE,
                {
                    KEY_DESTINATION_ID: destination_id,
                    KEY_DOOR_TEXT: door_text,
                    KEY_IS_MATCHED: True,
                }
            )
        else:
            self.handle_event(
                RobotEvent.DELIVERY_VERIFY_FAILED,
                {
                    KEY_DESTINATION_ID: destination_id,
                    KEY_DOOR_TEXT: door_text,
                    KEY_IS_MATCHED: False,
                }
            )


    # ==============================================================================================================
    # FSM
    # ==============================================================================================================

    def handle_event(self, event, data: dict = None):

        if event is None:
            return

        if data is None:
            data = {}

        try:
            event = to_robot_event(event)
        except Exception:
            write_log("Unknown event: %s" % str(event), self)
            return

        write_log(
            "EVENT | state=%s | event=%s"
            % (self._state.value, event.value),
            self
        )

        if not is_event_allowed(self._state, event):
            write_log(
                "EVENT NOT ALLOWED | state=%s | event=%s"
                % (self._state.value, event.value),
                self
            )
            return

        prev_state = self._state
        next_state = get_next_state(self._state, event)

        # ----------------------------------------------------------------------------------------------------------
        # Global / special event actions
        # ----------------------------------------------------------------------------------------------------------

        if event == RobotEvent.EMERGENCY_DETECTED:
            self.do_emergency(data)

        elif event == RobotEvent.PATIENT_DETECTED:
            self.do_patient_greeting(data)

        elif event == RobotEvent.NAVIGATION_ARRIVED:
            if self._state == RobotState.GUIDE:
                self.handle_event(RobotEvent.GUIDE_DONE)

            elif self._state == RobotState.DELIVERY:
                self.change_state(RobotState.DELIVERY_VERIFY)
                self.do_delivery_verify_start()

            elif self._state == RobotState.HOMING:
                self.handle_event(RobotEvent.HOMING_DONE)

            return

        elif event == RobotEvent.NAVIGATION_FAILED:
            if self._state == RobotState.GUIDE:
                self.handle_event(RobotEvent.GUIDE_FAILED)

            elif self._state == RobotState.DELIVERY:
                self.handle_event(RobotEvent.DELIVERY_FAILED)

            elif self._state == RobotState.HOMING:
                self.handle_event(RobotEvent.HOMING_FAILED)

            return

        elif event == RobotEvent.DELIVERY_VERIFY_DONE:
            self.speak("Delivery destination confirmed. Delivery complete.")

        elif event == RobotEvent.DELIVERY_VERIFY_FAILED:
            self.speak("I could not confirm the delivery room.")

        # ----------------------------------------------------------------------------------------------------------
        # State transition
        # ----------------------------------------------------------------------------------------------------------

        if next_state != prev_state:
            self.change_state(next_state)
            self.on_enter_state(next_state, data, event)

    def change_state(self, new_state: RobotState):

        if isinstance(new_state, str):
            new_state = to_robot_state(new_state)

        if self._state == new_state:
            return

        prev = self._state
        self._prev_state = prev
        self._state = new_state

        write_log(
            "STATE CHANGE | %s -> %s"
            % (prev.value, new_state.value),
            self
        )

    def on_enter_state(
            self,
            state: RobotState,
            data: dict = None,
            event: RobotEvent = None
    ):

        if data is None:
            data = {}

        if state == RobotState.IDLE:
            self.do_idle()

        elif state == RobotState.FOLLOW:
            self.do_follow()

        elif state == RobotState.GUIDE:
            self.do_guide(data)

        elif state == RobotState.DELIVERY:
            self.do_delivery(data)

        elif state == RobotState.HOMING:
            self.do_homing()

        elif state == RobotState.EMERGENCY:
            pass

        elif state == RobotState.ERROR:
            self.do_error()

    # ==============================================================================================================
    # State Actions
    # ==============================================================================================================

    def do_idle(self):

        self._delivery_destination_id = None
        self._greeted_patient = False

        self._last_follow_frame_id = None
        self._last_emergency_frame_id = None
        self._last_patient_check_frame_id = None

        self._door_check_requested = False

        self._mp_robot_task.send_command(CMD_STOP_ROBOT)

        self.request_stt_once()

    def do_follow(self):

        self._delivery_destination_id = None
        self._greeted_patient = False

        self.speak("I will follow you.")

    def do_guide(self, data: dict):

        self._delivery_destination_id = None
        self._greeted_patient = False

        destination_id = data.get(KEY_DESTINATION_ID, None)

        if destination_id is None:
            self.speak("I do not know the destination.")
            self.handle_event(RobotEvent.GO_IDLE)
            return

        self.speak("I will guide you.")

        self._mp_robot_task.send_command(
            CMD_START_GUIDE,
            {
                KEY_DESTINATION_ID: destination_id
            }
        )

    def do_delivery(self, data: dict):

        destination_id = data.get(KEY_DESTINATION_ID, None)

        if destination_id is None:
            self.speak("I do not know the delivery destination.")
            self.handle_event(RobotEvent.GO_IDLE)
            return

        self._delivery_destination_id = destination_id
        self._greeted_patient = False

        self._last_patient_check_frame_id = None
        self._door_check_requested = False

        self.speak("Starting delivery.")

        self._mp_robot_task.send_command(
            CMD_START_DELIVERY,
            {
                KEY_DESTINATION_ID: destination_id
            }
        )

    def do_homing(self):

        self._delivery_destination_id = None
        self._greeted_patient = False

        self.speak("Returning home.")

        self._mp_robot_task.send_command(
            CMD_GO_TO_DESTINATION,
            {
                KEY_DESTINATION_ID: "home"
            }
        )

    def do_emergency(self, data: dict):

        self._mp_robot_task.send_command(CMD_STOP_ROBOT)

        emergency_type = data.get(KEY_EMERGENCY_TYPE, "unknown")

        self.speak("Emergency detected. Please check the patient.")

        write_log(
            "EMERGENCY | type=%s"
            % str(emergency_type),
            self
        )

    def do_error(self):

        self._mp_robot_task.send_command(CMD_STOP_ROBOT)
        self.speak("System error occurred.")

    def do_patient_greeting(self, data: dict):

        if self._state != RobotState.DELIVERY:
            return

        if self._greeted_patient:
            return

        self._greeted_patient = True

        self.speak("Hello, how are you feeling?")

        write_log(
            "DELIVERY | patient greeting done | info=%s"
            % str(data.get(KEY_PATIENT_INFO, None)),
            self
        )

    def do_delivery_verify_start(self):

        self.speak("I have arrived. Checking the room number.")

        self.request_door_arrival_check()

    # ==============================================================================================================
    # Requests
    # ==============================================================================================================

    def request_stt_once(self):

        if self._state == RobotState.EMERGENCY:
            return

        self._mp_stt.send_command(CMD_LISTEN_ONCE)

    def request_emergency_check(self):

        if self._last_frame_packet is None:
            return

        frame_id = self._last_frame_packet.get(KEY_FRAME_ID, None)

        if frame_id is not None and frame_id == self._last_emergency_frame_id:
            return

        self._last_emergency_frame_id = frame_id

        frame_bgr = self._last_frame_packet.get("frame_bgr", None)

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

        frame_id = self._last_frame_packet.get(KEY_FRAME_ID, None)

        if frame_id is not None and frame_id == self._last_follow_frame_id:
            return

        self._last_follow_frame_id = frame_id

        frame_bgr = self._last_frame_packet.get("frame_bgr", None)
        depth_map = self._last_frame_packet.get("depth_map", None)

        if frame_bgr is None:
            return

        self._mp_robot_task.send_command(
            CMD_PROCESS_FOLLOW,
            {
                KEY_FRAME: frame_bgr,
                "depth_map": depth_map
            }
        )

    def request_patient_check_on_path(self):

        if self._last_frame_packet is None:
            return

        if self._greeted_patient:
            return

        frame_id = self._last_frame_packet.get(KEY_FRAME_ID, None)

        if frame_id is not None and frame_id == self._last_patient_check_frame_id:
            return

        self._last_patient_check_frame_id = frame_id

        frame_bgr = self._last_frame_packet.get("frame_bgr", None)

        if frame_bgr is None:
            return

        self._mp_robot_task.send_command(
            CMD_CHECK_PATIENT_ON_PATH,
            {
                KEY_FRAME: frame_bgr
            }
        )

    def request_door_arrival_check(self):

        if self._door_check_requested:
            return

        if self._last_frame_packet is None:
            return

        if self._delivery_destination_id is None:
            return

        frame_bgr = self._last_frame_packet.get("frame_bgr", None)

        if frame_bgr is None:
            return

        self._door_check_requested = True

        self._mp_robot_task.send_command(
            CMD_CHECK_DOOR_ARRIVAL,
            {
                KEY_FRAME: frame_bgr,
                KEY_DESTINATION_ID: self._delivery_destination_id
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
    # Log Utils
    # ==============================================================================================================

    @staticmethod
    def get_msg_status(msg: dict):

        status = msg.get(KEY_STATUS, None)

        if status is None:
            status = msg.get(KEY_DATA, {}).get(KEY_STATUS, None)

        return status

    @staticmethod
    def get_msg_error(msg: dict):

        error_msg = msg.get(KEY_ERROR, None)

        if error_msg is None:
            error_msg = msg.get(KEY_DATA, {}).get(KEY_ERROR, None)

        return error_msg

    @staticmethod
    def get_msg_text(msg: dict):

        text = msg.get(KEY_TEXT, None)

        if text is None:
            text = msg.get(KEY_DATA, {}).get(KEY_TEXT, None)

        return text

    def log_status(self, source: str, msg: dict):

        status = self.get_msg_status(msg)

        write_log(
            "STATUS | source=%s | status=%s"
            % (str(source), str(status)),
            self
        )

    def log_error(self, source: str, msg: dict):

        error_msg = self.get_msg_error(msg)

        write_log(
            "ERROR | source=%s | error=%s"
            % (str(source), str(error_msg)),
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
            self._mp_robot_task,
        ]

    def get_state(self):
        return self._state

    def get_prev_state(self):
        return self._prev_state

    def get_last_stt_text(self):
        return self._last_stt_text

    def get_last_llm_command(self):
        return self._last_llm_command