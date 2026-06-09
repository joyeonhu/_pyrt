# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : main.py
# Project Name : HealthcareRobotPyRT
# Description  : Application entry point
# -------------------------------------------------------------------------------------------------------------------- #

import sys

from PyQt5 import QtWidgets

from Commons import *
from MainWindow import CMainWindow


def main():
    """
    HealthcareRobotPyRT 시작 함수
    """

    write_log("Application start.")

    app = QtWidgets.QApplication(sys.argv) # PyQt5 애플리케이션 객체 생성

    window = CMainWindow() # 메인 윈도우 객체 생성
    window.show() # 메인 윈도우 표시

    exit_code = app.exec_() # PyQt5 이벤트 루프 시작, 사용자가 창을 닫거나 프로그램이 종료될 때까지 계속 실행, 종료 시 반환되는 exit code를 저장

    write_log("Application terminated.")

    sys.exit(exit_code) # 애플리케이션 종료, exit code를 시스템에 반환


if __name__ == "__main__":
    main()