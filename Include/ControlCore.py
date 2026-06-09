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
from MultiProcessing.MPStellaB2 import (
    proc_stella_b2,
    CMD_CONNECT,
    CMD_SET_VELOCITY_CONTROL,
    CMD_MOVE_STOP,
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
        self._last_cmd_vel = { # 로봇에게 보낸 마지막 cmd_vel
            KEY_LINEAR_X: 0.0,
            KEY_ANGULAR_Z: 0.0,
        }

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
        self._mp_stella_b2 = None # 스텔라B2 로봇 제어 프로세스

        self.create_processes() # 각 프로세스 생성 및 초기화

        write_log("ControlCore initialized.", self)

    # ==============================================================================================================
    # Process Create
    # ==============================================================================================================

    def create_processes(self): # 각 프로세스 생성 및 초기화, 프로세스 간 신호 연결

        self._mp_camera = CMultiProcessBase(PROC_CAMERA, proc_camera) # 카메라 프로세스 생성
        self._mp_stt = CMultiProcessBase(PROC_STT, proc_speech_to_text) # 음성인식 프로세스 생성
        self._mp_tts = CMultiProcessBase(PROC_TTS, proc_text_to_speech) # 음성합성 프로세스 생성
        self._mp_llm = CMultiProcessBase(PROC_LLM, proc_llm) # LLM 프로세스 생성
        self._mp_emergency = CMultiProcessBase(PROC_EMERGENCY, proc_emergency) # 응급 상황 판단 프로세스 생성
        self._mp_robot_task = CMultiProcessBase(PROC_HEALTHCARE, proc_robot_task) # 로봇 작업 처리 프로세스 생성
        self._mp_stella_b2 = CMultiProcessBase(PROC_STELLA_B2, proc_stella_b2) # 스텔라B2 로봇 제어 프로세스 생성

        # 프로세스가 queue로 보낸 메시지를 받을 때 실행할 콜백 함수 연결
        # MPCamera - feedback_queue.put(frame_msg)
        # -> ThreadQueueBroadcaster - sig_queue_bcast.emit(frame_msg)
        # -> ControlCore - on_camera_feedback(frame_msg)
        self._mp_camera.sig_queue_bcast.connect(self.on_camera_feedback)
        self._mp_stt.sig_queue_bcast.connect(self.on_stt_feedback)
        self._mp_tts.sig_queue_bcast.connect(self.on_tts_feedback)
        self._mp_llm.sig_queue_bcast.connect(self.on_llm_feedback)
        self._mp_emergency.sig_queue_bcast.connect(self.on_emergency_feedback)
        self._mp_robot_task.sig_queue_bcast.connect(self.on_robot_task_feedback)
        self._mp_stella_b2.sig_queue_bcast.connect(self.on_stella_b2_feedback)

        for mp_proc in self.get_process_list(): # 각 프로세스에
            mp_proc.sig_error.connect(self.on_process_error) # 프로세스 에러 시 on_process_error 콜백 연결

    # ==============================================================================================================
    # Start / Stop
    # ==============================================================================================================

    def start(self): # ControlCore 메인 루프 시작, 각 프로세스 시작 명령 전송

        self._is_running = True # ControlCore 메인 루프 실행 상태로 설정

        for mp_proc in self.get_process_list(): # 각 프로세스
            mp_proc.start() # 시작

        time.sleep(0.2) # 각 프로세스가 시작되고 안정화될 때까지 잠시 대기

        # 각 프로세스에 START 명령 전송, 각 프로세스는 내부적으로 필요한 초기화 작업 수행
        self._mp_camera.send_command(CMD_START)
        self._mp_stt.send_command(CMD_START)
        self._mp_tts.send_command(CMD_START)
        self._mp_llm.send_command(CMD_START)
        self._mp_emergency.send_command(CMD_START)
        self._mp_robot_task.send_command(CMD_START)
        self._mp_stella_b2.send_command(CMD_START)

        time.sleep(0.2)

        self._mp_stella_b2.send_command(
            CMD_CONNECT,
            {
                KEY_PORT: "/dev/ttyUSB0"
            }
        )

        self.handle_event(RobotEvent.STARTUP_DONE) # STARTUP_DONE 이벤트 발생, 초기 상태로 전환 및 초기 작업 수행

        write_log("ControlCore started.", self)

    def stop(self): # ControlCore 메인 루프 정지, 각 프로세스에 STOP 명령 전송 및 자원 해제

        self._is_running = False # ControlCore 메인 루프 정지 상태로 설정

        try:
            if self._mp_robot_task is not None: # 로봇 작업 처리 프로세스가 존재하면
                self._mp_robot_task.send_command(CMD_STOP_ROBOT) # 로봇 정지 명령 전송

            if self._mp_stella_b2 is not None: # 스텔라B2 제어 프로세스가 존재하면
                self._mp_stella_b2.send_command(CMD_MOVE_STOP) # 스텔라B2 정지 명령 전송

            for mp_proc in self.get_process_list(): # 각 프로세스에 STOP 명령 전송
                mp_proc.send_command(CMD_STOP) # 각 프로세스는 내부적으로 필요한 정지 작업 수행

            time.sleep(0.2) # 각 프로세스가 명령을 처리하고 안정화될 때까지 잠시 대기

            for mp_proc in self.get_process_list(): # 각 프로세스
                mp_proc.release() # 자원 해제 (프로세스 종료 및 정리)

        except Exception:
            ErrorHandler().report()

        write_log("ControlCore stopped.", self)

    # ==============================================================================================================
    # Main Loop
    # ==============================================================================================================

    def process_once(self): # ControlCore 메인 루프에서 주기적으로 호출되는 함수, 현재 상태에 따라 필요한 작업 수행

        if not self._is_running: # 메인 루프가 실행 중이 아니면 아무 작업도 수행하지 않고
            return # 함수 종료

        if self._last_frame_packet is not None: # 마지막으로 받은 프레임 패킷이 있으면
            self.request_emergency_check() # 응급 상황 판단 요청

        if self._state == RobotState.FOLLOW: # 현재 상태가 FOLLOW이면
            self.process_follow_once() # Follow 상태에서 한 번만 프레임 처리하여 Follow 명령 생성 및 로봇 작업 프로세스에 전달

        elif self._state == RobotState.DELIVERY: # 현재 상태가 DELIVERY이면
            self.request_patient_check_on_path() # 배달 경로에 환자가 있는지 확인 요청, 환자에게 인사하기 위해 사용

        elif self._state == RobotState.DELIVERY_VERIFY: # 현재 상태가 DELIVERY_VERIFY이면
            self.request_door_arrival_check() # 문 도착 확인 요청, 배달 목적지에 도착했는지 확인하기 위해 사용

    # ==============================================================================================================
    # Feedback Handlers
    # ==============================================================================================================

    def on_camera_feedback(self, msg: dict): # MPCamera에서 보낸 메시지를 처리하는 콜백 함수, 프레임 패킷 업데이트 및 응급 상황 판단 요청 등 수행

        if not isinstance(msg, dict): # 메시지가 딕셔너리 형태가 아니면 아무 작업도 수행하지 않고 함수 종료
            return

        msg_type = msg.get(KEY_TYPE, None) # 메시지에서 타입을 가져옴

        if msg_type == MSG_TYPE_FRAME: # 메시지 타입이 프레임이면 (프레임이 왔다는 뜻)
            self._last_frame_packet = msg.get(KEY_DATA, {}) # 카메라에서 받은 최신 프레임 저장

        elif msg_type == MSG_TYPE_STATUS: # 카메라 상태 메시지인 경우
            self.log_status(PROC_CAMERA, msg) # 카메라 상태 메시지 로그로 기록

        elif msg_type == MSG_TYPE_ERROR: # 카메라 에러 메시지인 경우
            self.log_error(PROC_CAMERA, msg) # 카메라 에러 메시지 로그로 기록

    def on_stt_feedback(self, msg: dict): # MPSTT에서 보낸 메시지를 처리하는 콜백 함수, STT 텍스트 업데이트 및 LLM에 명령 파싱 요청 등 수행

        if not isinstance(msg, dict): # 메시지가 딕셔너리 형태가 아니면 아무 작업도 수행하지 않고 함수 종료
            return

        msg_type = msg.get(KEY_TYPE, None) # 메시지에서 타입을 가져옴

        if msg_type == MSG_TYPE_TEXT: # 메시지 타입이 텍스트이면 (STT 결과가 왔다는 뜻)
            text = self.get_msg_text(msg) # 메시지에서 텍스트를 가져옴

            if text is None: # 텍스트가 없으면 아무 작업도 수행하지 않고 함수 종료
                return

            self._last_stt_text = text # STT에서 받은 최신 텍스트 저장

            write_log("STT TEXT | %s" % str(text), self) # STT 텍스트 로그로 기록

            self._mp_llm.send_command( # LLM에 명령 파싱 요청, LLM은 텍스트를 분석해서 명령과 목적지 ID 등을 추출
                CMD_PARSE_TEXT,
                {
                    KEY_TEXT: text
                }
            )

        elif msg_type == MSG_TYPE_STATUS: # STT 상태 메시지인 경우
            self.log_status(PROC_STT, msg) # STT 상태 메시지 로그로 기록

            status = self.get_msg_status(msg) # 메시지에서 상태를 가져옴

            if status == "STT_NO_INPUT" and self._state == RobotState.IDLE: # STT가 입력을 받지 못했는데 현재 상태가 IDLE이면
                self.request_stt_once() # 다시 한 번 STT 입력 요청, IDLE 상태에서는 계속해서 STT 입력을 받아야 하기 때문에 이렇게 함

        elif msg_type == MSG_TYPE_ERROR: # STT 에러 메시지인 경우
            self.log_error(PROC_STT, msg) # STT 에러 메시지 로그로 기록

    def on_tts_feedback(self, msg: dict): # MPTTS에서 보낸 메시지를 처리하는 콜백 함수, TTS 상태 업데이트 및 STT 입력 요청 등 수행

        if not isinstance(msg, dict): # 메시지가 딕셔너리 형태가 아니면 아무 작업도 수행하지 않고 함수 종료
            return

        msg_type = msg.get(KEY_TYPE, None) # 메시지에서 타입을 가져옴

        if msg_type == MSG_TYPE_STATUS: # TTS 상태 메시지인 경우
            self.log_status(PROC_TTS, msg) # TTS 상태 메시지 로그로 기록

            status = self.get_msg_status(msg) # 메시지에서 상태를 가져옴

            if status == "TTS_DONE": # TTS가 말을 마쳤다는 상태인 경우
                if self._state == RobotState.IDLE: # 현재 상태가 IDLE이면
                    self.request_stt_once() # 다시 한 번 STT 입력 요청, IDLE 상태에서는 계속해서 STT 입력을 받아야 하기 때문에 이렇게 함

                elif self._state == RobotState.DELIVERY_VERIFY: # 현재 상태가 DELIVERY_VERIFY이면 (배달 검증이 끝났는데 TTS로 결과를 말한 후에)
                    self.handle_event(RobotEvent.GO_IDLE) # GO_IDLE 이벤트 발생, IDLE 상태로 전환, 배달 작업 완료 후 초기 상태로 돌아가기 위해 사용

        elif msg_type == MSG_TYPE_ERROR: # TTS 에러 메시지인 경우
            self.log_error(PROC_TTS, msg) # TTS 에러 메시지 로그로 기록

    def on_llm_feedback(self, msg: dict): # MPLLM에서 보낸 메시지를 처리하는 콜백 함수, LLM 명령 업데이트 및 로봇 작업 프로세스에 명령 전달 등 수행

        if not isinstance(msg, dict): # 메시지가 딕셔너리 형태가 아니면 아무 작업도 수행하지 않고 함수 종료
            return

        msg_type = msg.get(KEY_TYPE, None) # 메시지에서 타입을 가져옴

        if msg_type == MSG_TYPE_COMMAND: # 메시지 타입이 명령이면 (LLM에서 명령이 왔다는 뜻)
            data = msg.get(KEY_DATA, {}) # 메시지에서 데이터 부분을 가져옴, 이 데이터에는 LLM이 텍스트를 분석해서 추출한 명령과 목적지 ID 등이 포함되어 있어야 함

            command = data.get(KEY_COMMAND, None) # 데이터에서 실제 명령을 가져옴, ex) "FOLLOW", "GUIDE", "DELIVERY", "HOMING", "NONE" 등, LLM이 텍스트를 분석해서 추출한 명령이 여기에 있어야 함
            destination_id = data.get(KEY_DESTINATION_ID, None) # 데이터에서 목적지 ID를 가져옴

            self._last_llm_command = data # LLM에서 받은 최신 명령 데이터 저장

            write_log( # LLM에서 받은 명령 로그로 기록
                "LLM COMMAND | command=%s | destination=%s"
                % (str(command), str(destination_id)),
                self
            )

            if command == "FOLLOW": # 명령이 FOLLOW이면
                self.handle_event(RobotEvent.CMD_FOLLOW) # CMD_FOLLOW 이벤트 발생, Follow 상태로 전환 및 Follow 작업 수행

            elif command == "GUIDE": # 명령이 GUIDE이면
                self.handle_event( # CMD_GUIDE 이벤트 발생
                    RobotEvent.CMD_GUIDE,
                    {
                        KEY_DESTINATION_ID: destination_id
                    }
                )

            elif command == "DELIVERY": # 명령이 DELIVERY이면
                self.handle_event( # CMD_DELIVERY 이벤트 발생
                    RobotEvent.CMD_DELIVERY,
                    {
                        KEY_DESTINATION_ID: destination_id
                    }
                )

            elif command == "HOMING": # 명령이 HOMING이면
                self.handle_event( # CMD_HOMING 이벤트 발생
                    RobotEvent.CMD_HOMING,
                    {
                        KEY_DESTINATION_ID: "home"
                    }
                )

            elif command == "NONE": # 명령이 NONE이면 (LLM이 명령을 추출하지 못했거나 명령이 없는 경우)
                self.speak("I did not understand the command.") # 사용자에게 명령을 이해하지 못했다는 메시지 전달

        elif msg_type == MSG_TYPE_STATUS: # LLM 상태 메시지인 경우
            self.log_status(PROC_LLM, msg) # LLM 상태 메시지 로그로 기록

        elif msg_type == MSG_TYPE_ERROR: # LLM 에러 메시지인 경우
            self.log_error(PROC_LLM, msg) # LLM 에러 메시지 로그로 기록

    def on_emergency_feedback(self, msg: dict): # MPEmergency에서 보낸 메시지를 처리하는 콜백 함수, 응급 상황 판단 결과에 따라 이벤트 발생 등 수행

        if not isinstance(msg, dict): # 메시지가 딕셔너리 형태가 아니면 아무 작업도 수행하지 않고 함수 종료
            return

        msg_type = msg.get(KEY_TYPE, None) # 메시지에서 타입을 가져옴

        if msg_type == MSG_TYPE_EMERGENCY: # 메시지 타입이 응급 상황이면 (응급 상황 판단 결과가 왔다는 뜻)
            data = msg.get(KEY_DATA, {}) # 메시지에서 데이터 부분을 가져옴
            is_emergency = data.get("is_emergency", False) # 데이터에서 응급 상황 여부를 가져옴

            if is_emergency: # 응급 상황인 경우
                self.handle_event( # EMERGENCY_DETECTED 이벤트 발생, 응급 상황 유형과 신뢰도를 데이터로 전달
                    RobotEvent.EMERGENCY_DETECTED,
                    {
                        KEY_EMERGENCY_TYPE: data.get(KEY_EMERGENCY_TYPE, data.get("emergency_type", None)),
                        KEY_CONFIDENCE: data.get(KEY_CONFIDENCE, data.get("confidence", 0.0)),
                    }
                )

        elif msg_type == MSG_TYPE_EVENT: # 메시지 타입이 이벤트이면 (응급 상황 판단과 관련된 이벤트가 왔다는 뜻)
            event = msg.get(KEY_EVENT, None) # 메시지에서 이벤트 이름을 가져옴
            data = msg.get(KEY_DATA, {}) # 메시지에서 데이터 부분을 가져옴, 이 데이터에는 이벤트와 관련된 추가 정보가 포함되어 있을 수 있음
            self.handle_event(event, data) # 이벤트 처리 함수 호출

        elif msg_type == MSG_TYPE_STATUS: # 응급 상황 판단 상태 메시지인 경우
            self.log_status(PROC_EMERGENCY, msg) # 응급 상황 판단 상태 메시지 로그로 기록

        elif msg_type == MSG_TYPE_ERROR: # 응급 상황 판단 에러 메시지인 경우
            self.log_error(PROC_EMERGENCY, msg) # 응급 상황 판단 에러 메시지 로그로 기록

    def on_robot_task_feedback(self, msg: dict): # MPRobotTask에서 보낸 메시지를 처리하는 콜백 함수, 로봇 작업과 관련된 이벤트 발생 및 상태/에러 메시지 로그 기록 등 수행

        if not isinstance(msg, dict): # 메시지가 딕셔너리 형태가 아니면 아무 작업도 수행하지 않고 함수 종료
            return

        msg_type = msg.get(KEY_TYPE, None) # 메시지에서 타입을 가져옴

        if msg_type == MSG_TYPE_EVENT: # 메시지 타입이 이벤트이면 (로봇 작업과 관련된 이벤트가 왔다는 뜻)
            event = msg.get(KEY_EVENT, None) # 메시지에서 이벤트 이름을 가져옴
            data = msg.get(KEY_DATA, {}) # 메시지에서 데이터 부분을 가져옴, 이 데이터에는 이벤트와 관련된 추가 정보가 포함되어 있을 수 있음
            self.handle_event(event, data) # 이벤트 처리 함수 호출

        elif msg_type == MSG_TYPE_DETECTION: # 메시지 타입이 감지이면 (로봇 작업과 관련된 감지 결과가 왔다는 뜻)
            self.handle_detection_message(msg) # 감지 메시지 처리 함수 호출

        elif msg_type == MSG_TYPE_STATUS: # 로봇 작업 상태 메시지인 경우
            self.log_status(PROC_HEALTHCARE, msg) # 로봇 작업 상태 메시지 로그로 기록

        elif msg_type == MSG_TYPE_CMD_VEL: # 메시지 타입이 cmd_vel이면
            self.handle_cmd_vel_message(msg) # cmd_vel 메시지 처리 함수 호출, Follow 상태에서 로봇에게 보낼 cmd_vel을 업데이트하고 로그로 기록

        elif msg_type == MSG_TYPE_ERROR: # 로봇 작업 에러 메시지인 경우
            self.log_error(PROC_HEALTHCARE, msg) # 로봇 작업 에러 메시지 로그로 기록

    def on_stella_b2_feedback(self, msg: dict):

        if not isinstance(msg, dict):
            return

        msg_type = msg.get(KEY_TYPE, None)

        if msg_type == MSG_TYPE_STATUS:
            self.log_status(PROC_STELLA_B2, msg)

        elif msg_type == MSG_TYPE_CMD_VEL:
            self.handle_cmd_vel_message(msg)

        elif msg_type == MSG_TYPE_ERROR:
            self.log_error(PROC_STELLA_B2, msg)

    def on_process_error(self, error_msg: str): # 각 프로세스에서 에러가 발생했을 때 호출되는 콜백 함수, 에러 메시지를 로그로 기록
        write_log("Process error: %s" % str(error_msg), self)

    # ==============================================================================================================
    # Detection Message
    # ==============================================================================================================

    def handle_detection_message(self, msg: dict): # MPRobotTask에서 보낸 감지 메시지를 처리하는 함수, 배달 검증과 관련된 감지 결과를 처리하여 DELIVERY_VERIFY_DONE 또는 DELIVERY_VERIFY_FAILED 이벤트 발생 등 수행

        # 메시지에서 꺼냄
        destination_id = msg.get(KEY_DESTINATION_ID, None) # 목적지
        door_text = msg.get(KEY_DOOR_TEXT, None) # 문에서 읽은 글자
        is_matched = msg.get(KEY_IS_MATCHED, None) # 일치 여부

        if destination_id is None: # 목적지 정보가 메시지에 없으면 데이터 부분에서 꺼냄
            data = msg.get(KEY_DATA, {}) # 메시지에서 데이터 부분을 가져옴, 이 데이터에는 감지 결과와 관련된 추가 정보가 포함되어 있을 수 있음
            destination_id = data.get(KEY_DESTINATION_ID, None) # 데이터에서 목적지 ID를 가져옴
            door_text = data.get(KEY_DOOR_TEXT, None) # 데이터에서 문에서 읽은 글자를 가져옴
            is_matched = data.get(KEY_IS_MATCHED, None) # 데이터에서 일치 여부를 가져옴

        write_log(
            "DETECTION | destination=%s | door_text=%s | matched=%s"
            % (str(destination_id), str(door_text), str(is_matched)),
            self
        )

        if self._state != RobotState.DELIVERY_VERIFY: # 현재 상태가 DELIVERY_VERIFY가 아니면 감지 메시지를 처리할 필요가 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        if is_matched: # 목적지와 문에서 읽은 글자가 일치하는 경우 (배달 검증 성공)
            self.handle_event( # DELIVERY_VERIFY_DONE 이벤트 발생
                RobotEvent.DELIVERY_VERIFY_DONE,
                {
                    KEY_DESTINATION_ID: destination_id,
                    KEY_DOOR_TEXT: door_text,
                    KEY_IS_MATCHED: True,
                }
            )
        else: # 목적지와 문에서 읽은 글자가 일치하지 않는 경우 (배달 검증 실패)
            self.handle_event( # DELIVERY_VERIFY_FAILED 이벤트 발생
                RobotEvent.DELIVERY_VERIFY_FAILED,
                {
                    KEY_DESTINATION_ID: destination_id,
                    KEY_DOOR_TEXT: door_text,
                    KEY_IS_MATCHED: False,
                }
            )

    def handle_cmd_vel_message(self, msg: dict): # RobotTask가 보고한 마지막 속도 명령을 ControlCore에 저장하는 함수
        """
        MPRobotTask에서 보고한 마지막 cmd_vel 저장
        GUI 표시 / 디버깅 / 실험 로그용
        """

        linear_x = msg.get(KEY_LINEAR_X, None) # 메시지에서 선형 속도를 가져옴
        angular_z = msg.get(KEY_ANGULAR_Z, None) # 메시지에서 각속도를 가져옴

        if linear_x is None or angular_z is None: # 메시지에 선형 속도나 각속도 정보가 없으면 데이터 부분에서 꺼냄
            data = msg.get(KEY_DATA, {})
            linear_x = data.get(KEY_LINEAR_X, 0.0)
            angular_z = data.get(KEY_ANGULAR_Z, 0.0)

        self._last_cmd_vel = { # ControlCore에 마지막 cmd_vel 저장
            KEY_LINEAR_X: linear_x,
            KEY_ANGULAR_Z: angular_z,
        }

        write_log(
            "CMD_VEL | linear_x=%.3f | angular_z=%.3f"
            % (
                float(linear_x),
                float(angular_z)
            ),
            self
        )

        if msg.get(KEY_SOURCE, None) != PROC_STELLA_B2:
            if self._mp_stella_b2 is not None:
                self._mp_stella_b2.send_command(
                    CMD_SET_VELOCITY_CONTROL,
                    {
                        KEY_LINEAR_X: linear_x,
                        KEY_ANGULAR_Z: angular_z,
                    }
                )


    # ==============================================================================================================
    # FSM
    # ==============================================================================================================

    def handle_event(self, event, data: dict = None): # 이벤트 처리 함수, 현재 상태와 이벤트에 따라 상태 전환 및 상태 진입 시 작업 수행

        if event is None: # 이벤트가 없으면 아무 작업도 수행하지 않고 함수 종료
            return

        if data is None: # 데이터가 없으면 빈 딕셔너리로 초기화
            data = {}

        try:
            event = to_robot_event(event) # 이벤트가 문자열로 들어온 경우 RobotEvent enum으로 변환, 변환할 수 없는 경우 예외 발생
        except Exception:
            write_log("Unknown event: %s" % str(event), self)
            return

        write_log(
            "EVENT | state=%s | event=%s"
            % (self._state.value, event.value),
            self
        )

        if not is_event_allowed(self._state, event): # 현재 상태에서 이벤트가 허용되지 않는 경우 (FSM 정의에 따라), 이벤트를 무시하고 아무 작업도 수행하지 않고 함수 종료
            write_log(
                "EVENT NOT ALLOWED | state=%s | event=%s"
                % (self._state.value, event.value),
                self
            )
            return

        prev_state = self._state # 현재 상태 저장
        next_state = get_next_state(self._state, event) # 현재 상태와 이벤트에 따른 다음 상태 가져옴 (FSM 정의에 따라)

        # ----------------------------------------------------------------------------------------------------------
        # Global / special event actions
        # ----------------------------------------------------------------------------------------------------------

        if event == RobotEvent.EMERGENCY_DETECTED: # 응급 상황이 감지된 경우
            self.do_emergency(data) # 응급 상황 처리 작업 수행 (로봇 정지, 사용자에게 알림, 로그 기록 등)

        elif event == RobotEvent.PATIENT_DETECTED: # 환자가 감지된 경우 (배달 경로에 환자가 있는지 확인 결과 환자가 있다고 판단한 경우)
            self.do_patient_greeting(data) # 환자에게 인사하는 작업 수행 (DELIVERY 상태에서 한 번만 인사하기 위해 _greeted_patient 플래그 사용)

        elif event == RobotEvent.NAVIGATION_ARRIVED: # 로봇이 목적지에 도착한 경우
            if self._state == RobotState.GUIDE: # 현재 상태가 GUIDE이면
                self.handle_event(RobotEvent.GUIDE_DONE) # GUIDE_DONE 이벤트 발생, 안내 작업 완료 처리

            elif self._state == RobotState.DELIVERY: # 현재 상태가 DELIVERY이면
                self.change_state(RobotState.DELIVERY_VERIFY) # DELIVERY_VERIFY 상태로 전환, 배달 검증 단계로 넘어감
                self.do_delivery_verify_start() # 배달 검증 시작 작업 수행 (목적지에 도착했으니 문에서 글자를 읽어서 배달 검증을 시작하라는 의미)

            elif self._state == RobotState.HOMING: # 현재 상태가 HOMING이면
                self.handle_event(RobotEvent.HOMING_DONE) # HOMING_DONE 이벤트 발생, 귀가 작업 완료 처리

            return

        elif event == RobotEvent.NAVIGATION_FAILED: # 로봇이 목적지로 이동하는 데 실패한 경우
            if self._state == RobotState.GUIDE: # 현재 상태가 GUIDE이면
                self.handle_event(RobotEvent.GUIDE_FAILED) # GUIDE_FAILED 이벤트 발생, 안내 작업 실패 처리

            elif self._state == RobotState.DELIVERY: # 현재 상태가 DELIVERY이면
                self.handle_event(RobotEvent.DELIVERY_FAILED) # DELIVERY_FAILED 이벤트 발생, 배달 작업 실패 처리

            elif self._state == RobotState.HOMING: # 현재 상태가 HOMING이면
                self.handle_event(RobotEvent.HOMING_FAILED) # HOMING_FAILED 이벤트 발생, 귀가 작업 실패 처리

            return

        elif event == RobotEvent.DELIVERY_VERIFY_DONE: # 배달 검증이 성공적으로 완료된 경우 (목적지와 문에서 읽은 글자가 일치하는 경우)
            self.speak("Delivery destination confirmed. Delivery complete.") # 사용자에게 배달 목적지가 확인되었고 배달이 완료되었다는 음성 멘트를 말함
            return

        elif event == RobotEvent.DELIVERY_VERIFY_FAILED: # 배달 검증이 실패한 경우 (목적지와 문에서 읽은 글자가 일치하지 않는 경우)
            self.speak("I could not confirm the delivery room.") # 사용자에게 배달 목적지를 확인할 수 없다는 음성 멘트를 말함

        # ----------------------------------------------------------------------------------------------------------
        # State transition
        # ----------------------------------------------------------------------------------------------------------

        if next_state != prev_state: # 다음 상태가 이전 상태와 다르면 (상태 전환이 필요한 경우)
            self.change_state(next_state) # 상태 전환 수행 (현재 상태를 다음 상태로 업데이트하고 로그 기록)
            self.on_enter_state(next_state, data, event) # 새로 전환된 상태에 진입할 때 수행할 작업 처리

    def change_state(self, new_state: RobotState):

        if isinstance(new_state, str): # new_state가 문자열로 들어온 경우
            new_state = to_robot_state(new_state) # 문자열을 RobotState enum으로 변환

        if self._state == new_state: # 현재 상태와 새 상태가 같으면 상태를 변경할 필요가 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        self._prev_state = self._state # 이전 상태를 현재 상태로 업데이트
        self._state = new_state # 현재 상태를 새 상태로 업데이트

        write_log(
            "STATE CHANGE | %s -> %s"
            % (
                self._prev_state.value,
                self._state.value
            ),
            self
        )

    def on_enter_state( # 상태가 바뀌었을 때 그 상태에 들어가면서 해야 할 행동 실행
            self,
            state: RobotState,
            data: dict = None,
            event: RobotEvent = None
    ):

        if data is None: # 데이터가 없으면 빈 딕셔너리로 초기화
            data = {}

        if state == RobotState.IDLE: # IDLE 상태에 진입하면 IDLE 상태에서 해야 할 작업 수행 (로봇 정지, 초기화, STT 입력 요청 등)
            self.do_idle()

        elif state == RobotState.FOLLOW: # FOLLOW 상태에 진입하면 FOLLOW 상태에서 해야 할 작업 수행 (목적지 초기화, 환자 인사 플래그 초기화, 프레임 처리 초기화, 사용자에게 안내 메시지 전달 등)
            self.do_follow()

        elif state == RobotState.GUIDE: # GUIDE 상태에 진입하면 GUIDE 상태에서 해야 할 작업 수행 (목적지 초기화, 환자 인사 플래그 초기화, LLM에서 받은 목적지 ID로 안내 작업 시작 명령을 로봇 작업 프로세스에 전달 등)
            self.do_guide(data)

        elif state == RobotState.DELIVERY: # DELIVERY 상태에 진입하면 DELIVERY 상태에서 해야 할 작업 수행 (목적지 ID 설정, 환자 인사 플래그 초기화, 프레임 처리 초기화, LLM에서 받은 목적지 ID로 배달 작업 시작 명령을 로봇 작업 프로세스에 전달 등)
            self.do_delivery(data)

        elif state == RobotState.HOMING: # HOMING 상태에 진입하면 HOMING 상태에서 해야 할 작업 수행 (목적지 초기화, 환자 인사 플래그 초기화, 귀가 작업 시작 명령을 로봇 작업 프로세스에 전달 등)
            self.do_homing()

        elif state == RobotState.EMERGENCY: # EMERGENCY 상태에 진입하면 EMERGENCY 상태에서 해야 할 작업 수행 (로봇 정지, 사용자에게 응급 상황 알림, 로그 기록 등)
            pass # EMERGENCY 상태는 이벤트 처리 함수에서 do_emergency()를 직접 호출하여 처리하기 때문에 여기서는 별도의 작업이 필요하지 않음

        elif state == RobotState.ERROR: # ERROR 상태에 진입하면 ERROR 상태에서 해야 할 작업 수행 (로봇 정지, 사용자에게 시스템 에러 알림 등)
            self.do_error() # 시스템 에러 처리 작업 수행 (로봇 정지, 사용자에게 알림 등)

    # ==============================================================================================================
    # State Actions
    # ==============================================================================================================

    def do_idle(self): # IDLE 상태에서 수행할 작업

        self._delivery_destination_id = None # 배달 목적지 초기화
        self._greeted_patient = False # 환자에게 인사했는지 여부 초기화

        self._last_follow_frame_id = None # Follow 상태에서 프레임 처리할 때 중복 처리를 방지하기 위해 마지막으로 처리한 프레임 ID 초기화
        self._last_emergency_frame_id = None # 응급 상황 판단할 때 중복 처리를 방지하기 위해 마지막으로 처리한 프레임 ID 초기화
        self._last_patient_check_frame_id = None # 배달 경로에 환자가 있는지 확인할 때 중복 처리를 방지하기 위해 마지막으로 처리한 프레임 ID 초기화

        self._door_check_requested = False # 문 도착 확인을 요청했는지 여부 초기화, 배달 검증 단계에서 목적지에 도착했는지 확인하기 위해 사용

        self._mp_robot_task.send_command(CMD_STOP_ROBOT) # 로봇 정지 명령을 로봇 작업 처리 프로세스에 전달, IDLE 상태에서는 로봇이 움직이지 않도록 함
        if self._mp_stella_b2 is not None:
            self._mp_stella_b2.send_command(CMD_MOVE_STOP)

        self.request_stt_once() # IDLE 상태에서는 계속해서 STT 입력을 받아야 하기 때문에 한 번 STT 입력 요청, 사용자가 명령을 말하면 LLM이 명령을 추출해서 이벤트를 발생시키는 방식으로 동작

    def do_follow(self): # FOLLOW 상태에서 수행할 작업

        self._delivery_destination_id = None # FOLLOW 상태에서는 특정 목적지로 이동하는 것이 아니기 때문에 배달 목적지 초기화
        self._greeted_patient = False # 환자에게 인사했는지 여부 초기화, FOLLOW 상태에서는 환자에게 인사하는 기능이 없지만 혹시 모를 상황에 대비해서 초기화
        self._last_follow_frame_id = None # Follow 상태에서 프레임 처리할 때 중복 처리를 방지하기 위해 마지막으로 처리한 프레임 ID 초기화

        self.speak("I will follow you.") # 사용자에게 안내 메시지 전달, FOLLOW 상태에서는 사용자가 로봇을 따라오도록 안내하는 메시지를 말함

    def do_guide(self, data: dict): # GUIDE 상태에서 수행할 작업

        self._delivery_destination_id = None # GUIDE 상태에서는 특정 목적지로 이동하는 것이 아니기 때문에 배달 목적지 초기화
        self._greeted_patient = False # 환자에게 인사했는지 여부 초기화, GUIDE 상태에서는 환자에게 인사하는 기능이 없지만 혹시 모를 상황에 대비해서 초기화

        destination_id = data.get(KEY_DESTINATION_ID, None) # 목적지 ID를 데이터에서 가져옴

        if destination_id is None: # 목적지 ID가 데이터에 없으면
            self.speak("I do not know the destination.") # 사용자에게 목적지를 알 수 없다는 메시지 전달
            self.handle_event(RobotEvent.GO_IDLE) # GO_IDLE 이벤트 발생, IDLE 상태로 전환
            return

        self.speak("I will guide you.") # 사용자에게 안내 메시지 전달

        self._mp_robot_task.send_command( # LLM에서 받은 목적지 ID로 안내 작업 시작 명령을 로봇 작업 프로세스에 전달
            CMD_START_GUIDE,
            {
                KEY_DESTINATION_ID: destination_id
            }
        )

    def do_delivery(self, data: dict): # DELIVERY 상태에서 수행할 작업

        destination_id = data.get(KEY_DESTINATION_ID, None) # 목적지 ID를 데이터에서 가져옴

        if destination_id is None: # 목적지 ID가 데이터에 없으면
            self.speak("I do not know the delivery destination.")
            self.handle_event(RobotEvent.GO_IDLE) # GO_IDLE 이벤트 발생, IDLE 상태로 전환
            return

        self._delivery_destination_id = destination_id # 배달 목적지 설정, 배달 검증 단계에서 목적지에 도착했는지 확인할 때 사용
        self._greeted_patient = False # 환자에게 인사했는지 여부 초기화, DELIVERY 상태에서는 배달 경로에 환자가 있을 수 있기 때문에 인사할 수 있도록 초기화

        self._last_patient_check_frame_id = None # 배달 경로에 환자가 있는지 확인할 때 중복 처리를 방지하기 위해 마지막으로 처리한 프레임 ID 초기화
        self._door_check_requested = False # 문 도착 확인을 요청했는지 여부 초기화, 배달 검증 단계에서 목적지에 도착했는지 확인하기 위해 사용

        self.speak("Starting delivery.") # 사용자에게 안내 메시지 전달, DELIVERY 상태에서는 배달을 시작한다는 메시지를 말함

        self._mp_robot_task.send_command( # LLM에서 받은 목적지 ID로 배달 작업 시작 명령을 로봇 작업 프로세스에 전달
            CMD_START_DELIVERY,
            {
                KEY_DESTINATION_ID: destination_id
            }
        )

    def do_homing(self): # HOMING 상태에서 수행할 작업

        self._delivery_destination_id = None # HOMING 상태에서는 특정 목적지로 이동하는 것이 아니기 때문에 배달 목적지 초기화
        self._greeted_patient = False # 환자에게 인사했는지 여부 초기화
        self._door_check_requested = False # 문 도착 확인을 요청했는지 여부 초기화

        self.speak("Returning home.")

        self._mp_robot_task.send_command( # 귀가 작업 시작 명령을 로봇 작업 프로세스에 전달
            CMD_GO_TO_DESTINATION,
            {
                KEY_DESTINATION_ID: "home"
            }
        )

    def do_emergency(self, data: dict): # EMERGENCY 상태에서 수행할 작업

        self._mp_robot_task.send_command(CMD_STOP_ROBOT) # 로봇 정지 명령을 로봇 작업 처리 프로세스에 전달, 응급 상황이 감지되면 로봇이 즉시 멈추도록 함
        if self._mp_stella_b2 is not None:
            self._mp_stella_b2.send_command(CMD_MOVE_STOP)

        emergency_type = data.get(KEY_EMERGENCY_TYPE, "unknown") # 응급 상황 유형을 데이터에서 가져옴, 데이터에 없으면 "unknown"으로 설정

        self.speak("Emergency detected. Please check the patient.") # 사용자에게 응급 상황이 감지되었다는 메시지 전달, 환자를 확인해 달라는 메시지를 말함

        write_log( # 응급 상황 감지 로그로 기록, 응급 상황 유형을 로그에 포함
            "EMERGENCY | type=%s"
            % str(emergency_type),
            self
        )

    def do_error(self): # ERROR 상태에서 수행할 작업

        self._mp_robot_task.send_command(CMD_STOP_ROBOT) # 로봇 정지 명령을 로봇 작업 처리 프로세스에 전달, 시스템 에러가 발생하면 로봇이 멈추도록 함
        if self._mp_stella_b2 is not None:
            self._mp_stella_b2.send_command(CMD_MOVE_STOP)
        self.speak("System error occurred.") # 사용자에게 시스템 에러가 발생했다는 메시지 전달

    def do_patient_greeting(self, data: dict): # 환자가 감지된 경우 환자에게 인사하는 작업 수행, DELIVERY 상태에서 한 번만 인사하기 위해 _greeted_patient 플래그 사용

        if self._state != RobotState.DELIVERY: # 현재 상태가 DELIVERY가 아니면 환자에게 인사할 필요가 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        if self._greeted_patient: # 이미 환자에게 인사했다면 다시 인사할 필요가 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        self._greeted_patient = True # 환자에게 인사했음을 표시하는 플래그 설정

        self.speak("Hello, how are you feeling?") # 사용자에게 환자에게 인사하는 메시지 전달, 환자에게 안부를 묻는 메시지를 말함

        write_log( # 환자 인사 로그로 기록, 환자 정보가 있으면 로그에 포함, 없으면 None으로 표시
            "DELIVERY | patient greeting done | info=%s"
            % str(data.get(KEY_PATIENT_INFO, None)),
            self
        )

    def do_delivery_verify_start(self): # 배달 검증 시작 작업 수행, 목적지에 도착했으니 문에서 글자를 읽어서 배달 검증을 시작하라는 의미

        self.speak("I have arrived. Checking the room number.")

        self.request_door_arrival_check() # 문 도착 확인 요청

    # ==============================================================================================================
    # Requests
    # ==============================================================================================================

    def request_stt_once(self): # STT 입력을 한 번 요청하는 함수

        if self._state == RobotState.EMERGENCY: # 현재 상태가 EMERGENCY이면 STT 입력을 받을 필요가 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        self._mp_stt.send_command(CMD_LISTEN_ONCE) # MPSpeechToText 프로세스에 CMD_LISTEN_ONCE 명령을 보내서 STT 입력을 한 번 받도록 함

    def request_emergency_check(self): # 응급 상황 판단을 요청하는 함수

        if self._last_frame_packet is None: # 프레임 패킷이 없으면 응급 상황 판단을 수행할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        frame_id = self._last_frame_packet.get(KEY_FRAME_ID, None) # 프레임 패킷에서 프레임 ID를 가져옴

        if frame_id is not None and frame_id == self._last_emergency_frame_id: # 프레임 ID가 이전에 응급 상황 판단을 수행할 때 사용한 프레임 ID와 같으면 이미 이 프레임으로 응급 상황 판단을 수행했으므로 다시 수행할 필요가 없어서 아무 작업도 수행하지 않고 함수 종료
            return

        self._last_emergency_frame_id = frame_id # 현재 프레임 ID를 마지막으로 응급 상황 판단을 수행한 프레임 ID로 저장

        frame_bgr = self._last_frame_packet.get("frame_bgr", None) # 프레임 패킷에서 BGR 프레임을 가져옴

        if frame_bgr is None: # BGR 프레임이 없으면 응급 상황 판단을 수행할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        self._mp_emergency.send_command( # MPEmergency 프로세스에 응급 상황 판단을 수행하도록 명령을 보냄, 프레임도 함께 전달하여 응급 상황 판단에 사용하도록 함
            CMD_CHECK_EMERGENCY,
            {
                KEY_FRAME: frame_bgr
            }
        )

    def process_follow_once(self): # Follow 작업을 한 번 수행하는 함수

        if self._last_frame_packet is None: # 프레임 패킷이 없으면 Follow 작업을 수행할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        frame_id = self._last_frame_packet.get(KEY_FRAME_ID, None) # 프레임 패킷에서 프레임 ID를 가져옴

        if frame_id is not None and frame_id == self._last_follow_frame_id: # 프레임 ID가 이전에 Follow 작업을 수행할 때 사용한 프레임 ID와 같으면 이미 이 프레임으로 Follow 작업을 수행했으므로 다시 수행할 필요가 없어서 아무 작업도 수행하지 않고 함수 종료
            return

        self._last_follow_frame_id = frame_id # 현재 프레임 ID를 마지막으로 Follow 작업을 수행한 프레임 ID로 저장

        frame_bgr = self._last_frame_packet.get("frame_bgr", None) # 프레임 패킷에서 BGR 프레임을 가져옴
        depth_map = self._last_frame_packet.get("depth_map", None) # 프레임 패킷에서 Depth 맵을 가져옴

        if frame_bgr is None: # BGR 프레임이 없으면 Follow 작업을 수행할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        self._mp_robot_task.send_command( # MPRobotTask 프로세스에 Follow 작업을 수행하도록 명령을 보냄, 프레임과 Depth 맵도 함께 전달하여 Follow 작업에 사용하도록 함
            CMD_PROCESS_FOLLOW,
            {
                KEY_FRAME: frame_bgr,
                "depth_map": depth_map
            }
        )

    def request_patient_check_on_path(self): # 배달 경로에 환자가 있는지 확인하기 위해 프레임을 처리하여 환자가 있다고 판단되면 환자에게 인사하는 작업을 시작하도록 로봇 작업 프로세스에 명령을 보내는 함수

        if self._last_frame_packet is None: # 프레임 패킷이 없으면 배달 경로에 환자가 있는지 확인할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        if self._greeted_patient: # 이미 환자에게 인사했다면 다시 배달 경로에 환자가 있는지 확인할 필요가 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        frame_id = self._last_frame_packet.get(KEY_FRAME_ID, None) # 프레임 패킷에서 프레임 ID를 가져옴

        if frame_id is not None and frame_id == self._last_patient_check_frame_id: # 프레임 ID가 이전에 배달 경로에 환자가 있는지 확인할 때 사용한 프레임 ID와 같으면 이미 이 프레임으로 배달 경로에 환자가 있는지 확인했으므로 다시 수행할 필요가 없어서 아무 작업도 수행하지 않고 함수 종료
            return

        self._last_patient_check_frame_id = frame_id # 현재 프레임 ID를 마지막으로 배달 경로에 환자가 있는지 확인한 프레임 ID로 저장

        frame_bgr = self._last_frame_packet.get("frame_bgr", None) # 프레임 패킷에서 BGR 프레임을 가져옴, 배달 경로에 환자가 있는지 확인하기 위해서는 프레임이 필요함

        if frame_bgr is None: # 프레임이 없으면 배달 경로에 환자가 있는지 확인할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        self._mp_robot_task.send_command( # MPRobotTask 프로세스에 배달 경로에 환자가 있는지 확인하는 작업을 시작하도록 명령을 보냄, 프레임도 함께 전달하여 환자 확인 작업에 사용하도록 함, 환자가 있다고 판단되면 로봇 작업 프로세스에서 환자에게 인사하는 작업도 함께 수행하도록 함
            CMD_CHECK_PATIENT_ON_PATH,
            {
                KEY_FRAME: frame_bgr
            }
        )

    def request_door_arrival_check(self): # 배달 검증 단계에서 목적지에 도착했는지 확인하기 위해 문에서 글자를 읽어서 배달 목적지와 일치하는지 확인하는 작업을 시작하도록 로봇 작업 프로세스에 명령을 보내는 함수

        if self._door_check_requested: # 이미 문 도착 확인을 요청한 상태라면 다시 요청할 필요가 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        if self._last_frame_packet is None: # 프레임 패킷이 없으면 문 도착 확인을 수행할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        if self._delivery_destination_id is None: # 배달 목적지 ID가 설정되어 있지 않으면 문 도착 확인을 수행할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        frame_bgr = self._last_frame_packet.get("frame_bgr", None) # 프레임 패킷에서 BGR 프레임을 가져옴, 문 도착 확인 작업을 수행하기 위해서는 프레임이 필요함

        if frame_bgr is None: # 프레임이 없으면 문 도착 확인을 수행할 수 없으므로 아무 작업도 수행하지 않고 함수 종료
            return

        self._door_check_requested = True # 문 도착 확인을 요청했음을 표시하는 플래그 설정, 이후에는 다시 요청할 필요가 없도록 함

        self._mp_robot_task.send_command( # 문 도착 확인 작업을 시작하도록 로봇 작업 프로세스에 명령을 보냄, LLM에서 받은 목적지 ID도 함께 전달하여 문에서 읽은 글자와 비교할 때 사용
            CMD_CHECK_DOOR_ARRIVAL,
            {
                KEY_FRAME: frame_bgr,
                KEY_DESTINATION_ID: self._delivery_destination_id
            }
        )

    def speak(self, text: str): # TTS로 텍스트를 말하는 함수

        self._mp_tts.send_command( # TTS 프로세스에 말할 텍스트를 전달하여 음성으로 출력하도록 명령
            CMD_SPEAK,
            {
                KEY_TEXT: text
            }
        )

    # ==============================================================================================================
    # Log Utils
    # ==============================================================================================================

    @staticmethod
    def get_msg_status(msg: dict): # 메시지에서 상태 정보를 추출하는 함수

        status = msg.get(KEY_STATUS, None) # 메시지에서 직접 상태 정보를 가져옴

        if status is None: # 메시지에 직접 상태 정보가 없으면, 메시지의 데이터 필드에서 상태 정보를 가져옴 (일부 메시지는 상태 정보를 데이터 필드에 넣어서 보낼 수 있기 때문)
            status = msg.get(KEY_DATA, {}).get(KEY_STATUS, None)

        return status # 상태 정보를 반환, 상태 정보가 없으면 None 반환

    @staticmethod
    def get_msg_error(msg: dict): # 메시지에서 에러 정보를 추출하는 함수

        error_msg = msg.get(KEY_ERROR, None)

        if error_msg is None:
            error_msg = msg.get(KEY_DATA, {}).get(KEY_ERROR, None)

        return error_msg

    @staticmethod
    def get_msg_text(msg: dict): # 메시지에서 텍스트 정보를 추출하는 함수

        text = msg.get(KEY_TEXT, None)

        if text is None:
            text = msg.get(KEY_DATA, {}).get(KEY_TEXT, None)

        return text

    def log_status(self, source: str, msg: dict): # 메시지에서 상태 정보를 추출하여 로그로 출력

        status = self.get_msg_status(msg) # 메시지에서 상태 정보를 추출하는 유틸리티 함수 사용

        write_log(
            "STATUS | source=%s | status=%s"
            % (str(source), str(status)),
            self
        )

    def log_error(self, source: str, msg: dict): # 메시지에서 에러 정보를 추출하여 로그로 출력

        error_msg = self.get_msg_error(msg)

        write_log(
            "ERROR | source=%s | error=%s"
            % (str(source), str(error_msg)),
            self
        )

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_process_list(self): # ControlCore에서 관리하는 모든 프로세스 객체를 리스트로 반환
        return [
            self._mp_camera, # 카메라 프로세스 객체
            self._mp_stt, # 음성인식 프로세스 객체
            self._mp_tts, # 음성합성 프로세스 객체
            self._mp_llm, # LLM 프로세스 객체
            self._mp_emergency, # 응급 상황 판단 프로세스 객체
            self._mp_robot_task, # 로봇 작업 처리 프로세스 객체
            self._mp_stella_b2, # Stella B2 제어 프로세스 객체
        ]

    def get_state(self): # 현재 로봇의 상태를 반환하는 함수
        return self._state

    def get_prev_state(self): # 이전 로봇의 상태를 반환하는 함수
        return self._prev_state

    def get_last_stt_text(self): # STT에서 받은 최신 텍스트를 반환하는 함수
        return self._last_stt_text

    def get_last_llm_command(self): # LLM에서 받은 최신 명령 데이터를 반환하는 함수
        return self._last_llm_command

    def get_last_cmd_vel(self): # RobotTask가 보고한 최신 cmd_vel을 반환하는 함수
        return self._last_cmd_vel