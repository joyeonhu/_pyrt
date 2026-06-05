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

    app = QtWidgets.QApplication(sys.argv)

    window = CMainWindow()
    window.show()

    exit_code = app.exec_()

    write_log("Application terminated.")

    sys.exit(exit_code)


if __name__ == "__main__":
    main()