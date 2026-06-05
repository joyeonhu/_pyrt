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
        super(CMainWindow, self).__init__(parent)

        self._control_core = CControlCore()
        self._is_started = False

        self._timer_core = QtCore.QTimer(self)
        self._timer_core.setInterval(10)
        self._timer_core.timeout.connect(self.on_timer_core)

        self._timer_status = QtCore.QTimer(self)
        self._timer_status.setInterval(500)
        self._timer_status.timeout.connect(self.on_timer_status)

        self.init_ui()

        write_log("MainWindow initialized.", self)

    # ==============================================================================================================
    # UI
    # ==============================================================================================================

    def init_ui(self):
        self.setWindowTitle("Healthcare Robot PyRT")
        self.resize(900, 600)

        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # ----------------------------------------------------------------------------------------------------------
        # Status Area
        # ----------------------------------------------------------------------------------------------------------

        status_group = QtWidgets.QGroupBox("System Status")
        status_layout = QtWidgets.QGridLayout(status_group)

        self.label_state_title = QtWidgets.QLabel("FSM State:")
        self.label_state = QtWidgets.QLabel(STATE_START)
        self.label_state.setStyleSheet("font-weight: bold; font-size: 16px;")

        self.label_time_title = QtWidgets.QLabel("Current Time:")
        self.label_time = QtWidgets.QLabel("-")

        self.label_stt_title = QtWidgets.QLabel("Last STT:")
        self.label_stt = QtWidgets.QLabel("-")

        self.label_llm_title = QtWidgets.QLabel("Last LLM Command:")
        self.label_llm = QtWidgets.QLabel("-")

        status_layout.addWidget(self.label_state_title, 0, 0)
        status_layout.addWidget(self.label_state, 0, 1)

        status_layout.addWidget(self.label_time_title, 1, 0)
        status_layout.addWidget(self.label_time, 1, 1)

        status_layout.addWidget(self.label_stt_title, 2, 0)
        status_layout.addWidget(self.label_stt, 2, 1)

        status_layout.addWidget(self.label_llm_title, 3, 0)
        status_layout.addWidget(self.label_llm, 3, 1)

        main_layout.addWidget(status_group)

        # ----------------------------------------------------------------------------------------------------------
        # Button Area
        # ----------------------------------------------------------------------------------------------------------

        button_group = QtWidgets.QGroupBox("Control")
        button_layout = QtWidgets.QHBoxLayout(button_group)

        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_stt_once = QtWidgets.QPushButton("Listen Once")
        self.btn_go_idle = QtWidgets.QPushButton("Go Idle")
        self.btn_clear_emergency = QtWidgets.QPushButton("Clear Emergency")

        self.btn_start.clicked.connect(self.on_click_start)
        self.btn_stop.clicked.connect(self.on_click_stop)
        self.btn_stt_once.clicked.connect(self.on_click_stt_once)
        self.btn_go_idle.clicked.connect(self.on_click_go_idle)
        self.btn_clear_emergency.clicked.connect(self.on_click_clear_emergency)

        button_layout.addWidget(self.btn_start)
        button_layout.addWidget(self.btn_stop)
        button_layout.addWidget(self.btn_stt_once)
        button_layout.addWidget(self.btn_go_idle)
        button_layout.addWidget(self.btn_clear_emergency)

        main_layout.addWidget(button_group)

        # ----------------------------------------------------------------------------------------------------------
        # Log Area
        # ----------------------------------------------------------------------------------------------------------

        log_group = QtWidgets.QGroupBox("Runtime Log")
        log_layout = QtWidgets.QVBoxLayout(log_group)

        self.text_log = QtWidgets.QPlainTextEdit()
        self.text_log.setReadOnly(True)

        log_layout.addWidget(self.text_log)

        main_layout.addWidget(log_group)

        self.statusBar().showMessage("Ready")

    # ==============================================================================================================
    # Start / Stop
    # ==============================================================================================================

    def start_system(self):
        if self._is_started:
            return

        try:
            self._control_core.start()

            self._timer_core.start()
            self._timer_status.start()

            self._is_started = True

            self.append_log("System started.")
            self.statusBar().showMessage("System started")

        except Exception:
            ErrorHandler().report()
            self.append_log("System start failed.")

    def stop_system(self):
        if not self._is_started:
            return

        try:
            self._timer_core.stop()
            self._timer_status.stop()

            self._control_core.stop()

            self._is_started = False

            self.append_log("System stopped.")
            self.statusBar().showMessage("System stopped")

        except Exception:
            ErrorHandler().report()
            self.append_log("System stop failed.")

    # ==============================================================================================================
    # Timer
    # ==============================================================================================================

    def on_timer_core(self):
        if self._is_started:
            self._control_core.process_once()

    def on_timer_status(self):
        self.update_status_view()

    # ==============================================================================================================
    # Button Slots
    # ==============================================================================================================

    def on_click_start(self):
        self.start_system()

    def on_click_stop(self):
        self.stop_system()

    def on_click_stt_once(self):
        if not self._is_started:
            return

        self._control_core.request_stt_once()
        self.append_log("STT listen once requested.")

    def on_click_go_idle(self):
        if not self._is_started:
            return

        self._control_core.handle_event(EVT_GO_IDLE)
        self.append_log("GO_IDLE event requested.")

    def on_click_clear_emergency(self):
        if not self._is_started:
            return

        self._control_core.handle_event(EVT_EMERGENCY_CLEARED)
        self.append_log("EMERGENCY_CLEARED event requested.")

    # ==============================================================================================================
    # View Update
    # ==============================================================================================================

    def update_status_view(self):
        state = self._control_core.get_state()
        stt_text = self._control_core.get_last_stt_text()
        llm_cmd = self._control_core.get_last_llm_command()

        self.label_state.setText(str(state))
        self.label_time.setText(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

        self.label_stt.setText(str(stt_text) if stt_text is not None else "-")
        self.label_llm.setText(str(llm_cmd) if llm_cmd is not None else "-")

    def append_log(self, text: str):
        now = datetime.datetime.now().strftime("%H:%M:%S")
        self.text_log.appendPlainText("[%s] %s" % (now, text))

    # ==============================================================================================================
    # Close
    # ==============================================================================================================

    def closeEvent(self, event: QtGui.QCloseEvent):
        self.stop_system()
        write_log("MainWindow closed.", self)
        event.accept()

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_control_core(self):
        return self._control_core