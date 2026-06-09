# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MainWindow.py
# Project Name : HealthcareRobotPyRT
# Description  : Main GUI window for healthcare robot
# -------------------------------------------------------------------------------------------------------------------- #

import datetime

from PyQt5 import QtCore, QtGui, QtWidgets

from Commons import *
from ControlCore import CControlCore
from HealthcareRobot.HealthcareState import *


class CMainWindow(QtWidgets.QMainWindow):
    """
    MainWindow

    역할:
        1. ControlCore 생성
        2. Start / Stop 버튼으로 전체 시스템 제어
        3. 주기적으로 ControlCore.process_once() 호출
        4. 현재 FSM 상태 표시
        5. 로그 출력
    """

    def __init__(self, parent=None):
        super(CMainWindow, self).__init__(parent) # 부모 클래스 초기화

        self._control_core = CControlCore() # ControlCore 객체 생성
        self._is_started = False # 시스템 시작 여부

        self._timer_core = QtCore.QTimer(self) # ControlCore의 process_once()를 주기적으로 호출하는 타이머, 10ms마다 호출하도록 설정
        self._timer_core.setInterval(10) # 10ms 간격으로 타이머 설정, 즉 100Hz로 process_once()를 호출하겠다는 뜻
        self._timer_core.timeout.connect(self.on_timer_core) # 타이머가 시간이 될 때마다 on_timer_core() 메서드가 호출되도록 연결

        self._timer_status = QtCore.QTimer(self) # 상태 표시 업데이트 타이머, 500ms마다 상태 표시를 업데이트하도록 설정
        self._timer_status.setInterval(500) # 500ms 간격으로 타이머 설정
        self._timer_status.timeout.connect(self.on_timer_status) # 타이머가 시간이 될 때마다 on_timer_status() 메서드가 호출되도록 연결

        self.init_ui() # UI 초기화

        write_log("MainWindow initialized.", self)

    # ==============================================================================================================
    # UI
    # ==============================================================================================================

    def init_ui(self): # UI 구성
        self.setWindowTitle("Healthcare Robot PyRT") # 창 제목
        self.resize(900, 600) # 창 크기

        central_widget = QtWidgets.QWidget(self) # 창 중앙에 들어갈 기본 위젯 생성
        self.setCentralWidget(central_widget) # 그 위젯은 MainWindow의 중앙 위젯으로 설정

        main_layout = QtWidgets.QVBoxLayout(central_widget) # 세로 방향 레이아웃 생성, 위에서 아래로 UI 요소들이 배치되도록 설정

        # ----------------------------------------------------------------------------------------------------------
        # Status Area
        # ----------------------------------------------------------------------------------------------------------

        status_group = QtWidgets.QGroupBox("System Status") # "System Status"라는 박스 영역 생성
        status_layout = QtWidgets.QGridLayout(status_group) # 상태 표시를 격자 형태로 배치할 레이아웃 생성

        self.label_state_title = QtWidgets.QLabel("FSM State:") # "FSM State:"라는 제목 레이블 생성
        self.label_state = QtWidgets.QLabel(RobotState.START.value) # 현재 상태 표시 라벨 생성, 초기값은 RobotState.START의 값으로 설정
        self.label_state.setStyleSheet("font-weight: bold; font-size: 16px;") # 상태 라벨의 글씨를 굵게 하고 크기를 16px로 설정

        self.label_time_title = QtWidgets.QLabel("Current Time:") # "Current Time:"이라는 제목 레이블 생성
        self.label_time = QtWidgets.QLabel("-") # 현재 시간 표시 라벨 생성, 초기값은 "-"로 설정

        self.label_stt_title = QtWidgets.QLabel("Last STT:") # "Last STT:"라는 제목 레이블 생성
        self.label_stt = QtWidgets.QLabel("-") # 마지막 STT 결과 표시 라벨 생성, 초기값은 "-"로 설정

        self.label_llm_title = QtWidgets.QLabel("Last LLM Command:") # "Last LLM Command:"라는 제목 레이블 생성
        self.label_llm = QtWidgets.QLabel("-") # 마지막 LLM 명령 결과 표시 라벨 생성, 초기값은 "-"로 설정

        status_layout.addWidget(self.label_state_title, 0, 0) # 0행 0열에 제목
        status_layout.addWidget(self.label_state, 0, 1) # 0행 1열에 상태값

        # 1행에 현재 시간 표시
        status_layout.addWidget(self.label_time_title, 1, 0)
        status_layout.addWidget(self.label_time, 1, 1)

        # 2행에 마지막 STT 결과 표시
        status_layout.addWidget(self.label_stt_title, 2, 0)
        status_layout.addWidget(self.label_stt, 2, 1)

        # 3행에 마지막 LLM 명령 결과 표시
        status_layout.addWidget(self.label_llm_title, 3, 0)
        status_layout.addWidget(self.label_llm, 3, 1)

        main_layout.addWidget(status_group) # 상태 박스를 메인 레이아웃에 추가

        # ----------------------------------------------------------------------------------------------------------
        # Button Area
        # ----------------------------------------------------------------------------------------------------------

        button_group = QtWidgets.QGroupBox("Control") # 버튼들을 넣을 Control 박스 생성
        button_layout = QtWidgets.QHBoxLayout(button_group) # 버튼들을 가로로 배치할 레이아웃 생성

        self.btn_start = QtWidgets.QPushButton("Start") # Start 버튼 생성
        self.btn_stop = QtWidgets.QPushButton("Stop") # Stop 버튼 생성
        self.btn_stt_once = QtWidgets.QPushButton("Listen Once") # Listen Once 버튼 생성, 이 버튼을 누르면 STT가 한 번만 실행되도록 요청하는 기능이 구현될 예정
        self.btn_go_idle = QtWidgets.QPushButton("Go Idle") # Go Idle 버튼 생성, 이 버튼을 누르면 로봇이 대기 상태로 전환되도록 요청하는 기능이 구현될 예정
        self.btn_clear_emergency = QtWidgets.QPushButton("Clear Emergency") # Clear Emergency 버튼 생성, 이 버튼을 누르면 응급 상태가 해제되도록 요청하는 기능이 구현될 예정

        self.btn_start.clicked.connect(self.on_click_start) # Start 버튼이 클릭되면 on_click_start() 메서드가 호출되도록 연결
        self.btn_stop.clicked.connect(self.on_click_stop) # Stop 버튼이 클릭되면 on_click_stop() 메서드가 호출되도록 연결
        self.btn_stt_once.clicked.connect(self.on_click_stt_once) # Listen Once 버튼이 클릭되면 on_click_stt_once() 메서드가 호출되도록 연결
        self.btn_go_idle.clicked.connect(self.on_click_go_idle) # Go Idle 버튼이 클릭되면 on_click_go_idle() 메서드가 호출되도록 연결
        self.btn_clear_emergency.clicked.connect(self.on_click_clear_emergency) # Clear Emergency 버튼이 클릭되면 on_click_clear_emergency() 메서드가 호출되도록 연결

        button_layout.addWidget(self.btn_start) # 버튼들을 버튼 레이아웃에 추가
        button_layout.addWidget(self.btn_stop) # 버튼들을 버튼 레이아웃에 추가
        button_layout.addWidget(self.btn_stt_once) # 버튼들을 버튼 레이아웃에 추가
        button_layout.addWidget(self.btn_go_idle) # 버튼들을 버튼 레이아웃에 추가
        button_layout.addWidget(self.btn_clear_emergency) # 버튼들을 버튼 레이아웃에 추가

        main_layout.addWidget(button_group) # 버튼 박스를 메인 레이아웃에 추가

        # ----------------------------------------------------------------------------------------------------------
        # Log Area
        # ----------------------------------------------------------------------------------------------------------

        log_group = QtWidgets.QGroupBox("Runtime Log") # 로그 표시용 박스 생성
        log_layout = QtWidgets.QVBoxLayout(log_group) # 로그 박스 안에 세로 레이아웃 생성

        self.text_log = QtWidgets.QPlainTextEdit() # 여러 줄 테그슽 표시창 생성
        self.text_log.setReadOnly(True) # 사용자가 로그창 내용을 직접 수정하지 못하게 설정

        log_layout.addWidget(self.text_log) # 로그창을 로그 레이아웃에 추가

        main_layout.addWidget(log_group) # 로그 박스를 메인 레이아웃에 추가

        self.statusBar().showMessage("Ready") # 창 아래 상태바에 Ready 표시

    # ==============================================================================================================
    # Start / Stop
    # ==============================================================================================================

    def start_system(self): # 시스템 시작 함수, Start 버튼 클릭 시 호출됨
        if self._is_started: # 이미 시스템이 시작된 상태라면
            return # 아무 작업도 하지 않고 종료, 중복으로 시작하는 것을 방지하기 위한 체크

        try:
            self._control_core.start() # ControlCore의 start() 메서드를 호출해서 시스템 시작, 여기서 각 프로세스가 시작됨

            self._timer_core.start() # ControlCore의 메인 루프 타이머 시작, process_once()가 주기적으로 호출됨
            self._timer_status.start() # 화면 상태 갱신 타이머 시작

            self._is_started = True # 시스템 시작 상태로 전환

            self.append_log("System started.") # 로그에 시스템이 시작되었다는 메시지 추가
            self.statusBar().showMessage("System started") # 창 아래 상태바에 System started 표시

        except Exception:
            ErrorHandler().report()
            self.append_log("System start failed.")

    def stop_system(self): # 시스템 정지 함수, Stop 버튼 클릭 시 호출됨
        if not self._is_started: # 시스템이 시작되지 않은 상태라면
            return # 아무 작업도 하지 않고 종료, 중복으로 정지하는 것을 방지하기 위한 체크

        try:
            self._timer_core.stop() # ControlCore 루프 타이머 정지, process_once()가 더 이상 주기적으로 호출되지 않음
            self._timer_status.stop() # 상태 업데이트 타이머 정지

            self._control_core.stop() # ControlCore 정지

            self._is_started = False # 시스템 정지 상태로 전환

            self.append_log("System stopped.")
            self.statusBar().showMessage("System stopped")

        except Exception:
            ErrorHandler().report()
            self.append_log("System stop failed.")

    # ==============================================================================================================
    # Timer
    # ==============================================================================================================

    def on_timer_core(self): # 10ms마다 실행되는 함수
        if self._is_started: # 시스템이 시작된 상태라면
            self._control_core.process_once() # ControlCore의 process_once() 메서드를 호출해서 시스템의 메인 루프를 한 번 실행

    def on_timer_status(self): # 500ms마다 실행되는 함수
        self.update_status_view() # 화면에 표시되는 상태 정보를 업데이트하는 함수

    # ==============================================================================================================
    # Button Slots
    # ==============================================================================================================

    def on_click_start(self): # Start 버튼 클릭 시 실행되는 함수
        self.start_system() # 시스템 시작 함수 호출

    def on_click_stop(self): # Stop 버튼 클릭 시 실행되는 함수
        self.stop_system() # 시스템 정지 함수 호출

    def on_click_stt_once(self): # Listen Once 버튼 클릭 시 실행되는 함수
        if not self._is_started: # 시스템이 시작되지 않은 상태라면
            return # 아무 작업도 하지 않고 종료, 시스템이 시작된 상태에서만 STT 요청이 가능하도록 체크

        self._control_core.request_stt_once() # ControlCore의 request_stt_once() 메서드를 호출해서 STT를 한 번 실행하도록 요청
        self.append_log("STT listen once requested.")

    def on_click_go_idle(self): # Go Idle 버튼 클릭 시 실행되는 함수
        if not self._is_started: # 시스템이 시작되지 않은 상태라면
            return # 아무 작업도 하지 않고 종료, 시스템이 시작된 상태에서만 Go Idle 요청이 가능하도록 체크

        self._control_core.handle_event(RobotEvent.GO_IDLE) # ControlCore에 GO_IDLE 이벤트 전달
        self.append_log("GO_IDLE event requested.")

    def on_click_clear_emergency(self): # Clear Emergency 버튼 클릭 시 실행되는 함수
        if not self._is_started: # 시스템이 시작되지 않은 상태라면
            return # 아무 작업도 하지 않고 종료, 시스템이 시작된 상태에서만 Clear Emergency 요청이 가능하도록 체크

        self._control_core.handle_event(RobotEvent.EMERGENCY_CLEARED) # ControlCore에 EMERGENCY_CLEARED 이벤트 전달
        self.append_log("EMERGENCY_CLEARED event requested.")

    # ==============================================================================================================
    # View Update
    # ==============================================================================================================

    def update_status_view(self): # 화면 상태 갱신 함수
        state = self._control_core.get_state() # ControlCore에서 현재 FSM 상태 가져오기
        stt_text = self._control_core.get_last_stt_text() # ControlCore에서 마지막 STT 결과 텍스트 가져오기
        llm_cmd = self._control_core.get_last_llm_command() # ControlCore에서 마지막 LLM 명령 텍스트 가져오기

        self.label_state.setText(str(state)) # 상태 라벨 갱신
        self.label_time.setText(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) # 현재 시간을 문자열로 만들어 화면에 표시

        self.label_stt.setText(str(stt_text) if stt_text is not None else "-") # STT 결과가 있으면 표시, 없으면 -
        self.label_llm.setText(str(llm_cmd) if llm_cmd is not None else "-") # LLM 명령 결과가 있으면 표시, 없으면 -

    def append_log(self, text: str): # GUI 로그창에 텍스트 추가하는 함수
        now = datetime.datetime.now().strftime("%H:%M:%S") # 현재 시각을 "시:분:초" 형식의 문자열로 만듦
        self.text_log.appendPlainText("[%s] %s" % (now, text)) # 로그창에 "[시각] 텍스트" 형식으로 로그 메시지 추가

    # ==============================================================================================================
    # Close
    # ==============================================================================================================

    def closeEvent(self, event: QtGui.QCloseEvent): # 창을 닫을 때 자동 호출되는 함수
        self.stop_system() # 창 닫기 전에 시스템 정지
        write_log("MainWindow closed.", self) # 창 닫힘 로그 출력
        event.accept() # 창 닫기를 허용

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_control_core(self): # 외부에서 ControlCore 객체를 가져올 수 있게 하는 getter
        return self._control_core # ControlCore 객체 반환