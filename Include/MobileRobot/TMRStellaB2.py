# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : TMRStellaB2.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.01.29 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import threading
import time

from numpy import pi
from MobileRobot.TMRDiff import *

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
DOCK_PATH = os.path.join(ROOT_PATH, "DockWidgets")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH

from Library.Serial.SerialPort import CSerialPort
from Commons import PySignal, is_system_win

@unique
class EnumStellaB2Stop(IntEnum):
    MOTOR_FREE = 1  # free-wheeling
    MOTOR_HOLD = 2  # stop immediately, keep motor holding torque
    MOTOR_HOLD_DECC = 3  # decelerate then stop, keep motor holding torque


@unique
class EnumStellaB2MotorCtrl(IntEnum):
    ALL = 0
    RIGHT = 1
    LEFT = 2


@unique
class EnumStellaB2Status(IntEnum):
    NORMAL = 0
    OVERLOAD_RIGHT = 1
    OVERLOAD_LEFT = 2
    OVERLOAD_ALL = 3
    OVER_VOLTAGE = 4  # voltage input > 15
    LOW_VOLTAGE = 5  # voltage input < 10
    POSITION_REACHED = 6  # only when using built-in position controller
    RUNNING = 7  # if wheel is rolling (serial commands)


@unique
class EnumStellaB2Cmd(IntEnum):
    MOVE_JOINT_SPACE = 0
    MOVE_GET_ENCODER = auto()
    MOVE_STOP = auto()
    FACTORY_RESET = auto()
    GET_STATE = auto()
    GET_VELOCITY = auto()
    GET_WHEEL_POSITION = auto()
    GET_VERSION = auto()
    GET_ENCODER = auto()


class CTMRStellaB2(CTMRDiff):
    _serial: CSerialPort
    btaSerialBuff: bytearray
    strSerialPort: str
    nSerialBaud: int
    nSerialByteSize: int
    strSerialParity: str
    fSerialStopBits: float
    fRadiusLimit: float
    fPrevTime: float
    nPrevCmd: EnumStellaB2Cmd  # for parsing
    threadMonitoring: threading.Thread = None

    def __init__(self, adictConfig: Union[None, dict] = None):
        if adictConfig is None:
            # this is according to the CTMStellaB2 parameters
            fWheelBase = 0.289
            fWheelRadius = 0.0752  # circumference is 47.25 cm according to specs
            nEncoderRes = 1024  # PPR = 256
            fGearRatio = 13.71  # Spec = 14, but actual calculation states 13.71
            fMaxVel = 1.45  # motor max speed is 270 rads
            fMaxAcc = 0.4  # arbitrary
            fMaxJerk = 0.2  # arbitrary
            fMaxYaw = 3.14
            super(CTMRStellaB2, self).__init__(fWheelRadius, fWheelBase, fMaxVel, fMaxAcc, fMaxJerk, fMaxYaw)
            self.set_wheel_spec(nEncoderRes, fGearRatio)
        else:
            # todo: add configuration file, preferably JSON
            pass

        # Signals #
        self.sig_is_run_monitoring = PySignal(bool)
        self.sig_move_get_enc = PySignal(int, int, int)

        # Serial Communication Specifications
        self.nSerialBaud = 115200
        self.nSerialByteSize = 8
        self.strSerialParity = 'N'
        self.fSerialStopBits = 1

        # Serial Callbacks
        self.btaSerialBuff = bytearray()
        self._serial = CSerialPort(abIsRecvDirect=True)  # receive after send, deserialization is pretty difficult
        self._serial.sig_connected.connect(self.on_serial_connect)
        self._serial.sig_disconnected.connect(self.on_serial_disconnect)
        self._serial.sig_send_data.connect(self.on_serial_send)
        self._serial.sig_serial_error.connect(self.on_serial_error)
        self._serial.sig_recv_data.connect(self.on_serial_recv)

        # calculate time difference to calculate velocity
        self.fPrevTime = 0.
        self.nPrevEncL = 0
        self.nPrevEncR = 0

        self.IsSerial = True # serial robot

    def release(self):
        self.stop_monitoring_thread()
        self.disconnect()

    ### Serial Communication ###
    def connect(self, strPort: Union[None, str] = None):
        if strPort is None:
            if is_system_win():
                self.strSerialPort = "COM1"
            else:
                self.strSerialPort = "/dev/ttyUSB0"
        else:
            self.strSerialPort = str(strPort)

        self._serial.setParams(port=self.strSerialPort, baudrate=self.nSerialBaud, bytesize=self.nSerialByteSize,
                               parity=self.strSerialParity, stopbits=self.fSerialStopBits, timeout=10)
        self._serial.connect()

    def is_connected(self) -> bool:
        return self._serial.is_connected()

    def disconnect(self):
        self.stop_monitoring_thread()
        if self.is_connected():
            self._serial.disconnect()

    ### Monitoring Thread ###
    def start_monitoring_thread(self):
        if self.threadMonitoring is None:
            self.threadMonitoring: CThreadStateMonitor = CThreadStateMonitor(self)
            self.threadMonitoring.sig_is_run.connect(self.on_thread_monitor_is_run)
            self.threadMonitoring.sig_is_done.connect(self.on_thread_monitor_is_done)
            self.threadMonitoring.start()

    def stop_monitoring_thread(self):
        if self.threadMonitoring is not None:
            self.threadMonitoring.stop()
            while self.threadMonitoring.is_alive():
                pass
            self.threadMonitoring = None

    def on_thread_monitor_is_run(self):
        if self.is_connected():
            self.sig_is_run_monitoring.emit(True)

    def on_thread_monitor_is_done(self):
        if self.is_connected():
            self.sig_is_run_monitoring.emit(False)

    ### Serial Callbacks ###
    def on_serial_connect(self):
        self.reset_curr_pos()
        self.get_version()  # get software version when started
        time.sleep(0.02)
        # self.start_monitoring_thread()
        self.sig_connected.emit()

    def on_serial_disconnect(self):
        self.stop_monitoring_thread()
        self.sig_disconnected.emit()

    def on_serial_error(self):
        self.sig_comm_error.emit()

    def on_serial_send(self, abtData: bytes):
        pass

    def on_serial_recv(self, abtData: bytes):
        try:
            # print(abtData, self.nPrevCmd)
            self.btaSerialBuff.extend(bytearray(abtData))

            # check STX and ETX
            if self.btaSerialBuff[0] == 0x02 and self.btaSerialBuff[-1] == 0x03:
                feedback = bytes(self.btaSerialBuff[1:-1]).decode("utf-8")

                if self.nPrevCmd == EnumStellaB2Cmd.MOVE_GET_ENCODER:
                    # todo: implement try except
                    right_encoder, left_encoder, robot_stat = 0, 0, 0
                    if feedback[0] == 'F':
                        right_encoder = int(feedback[1:10])
                    elif feedback[0] == 'B':
                        right_encoder = int(feedback[1:10])  # * -1

                    if feedback[10] == 'F':
                        left_encoder = int(feedback[11:20])
                    elif feedback[10] == 'B':
                        left_encoder = int(feedback[11:20])  # * -1

                    if feedback[20] == 'S':
                        robot_stat = EnumStellaB2Status(int(feedback[21]))

                    self.sig_move_get_enc.emit(right_encoder, left_encoder, robot_stat)

                if self.nPrevCmd == EnumStellaB2Cmd.GET_ENCODER:
                    right_encoder, left_encoder = 0, 0
                    if feedback[0] == 'F':
                        right_encoder = int(feedback[1:10])
                    elif feedback[0] == 'B':
                        right_encoder = int(feedback[1:10])  # * -1

                    if feedback[10] == 'F':
                        left_encoder = int(feedback[11:20])
                    elif feedback[10] == 'B':
                        left_encoder = int(feedback[11:20])  # * -1

                    self.sig_get_enc.emit(right_encoder, left_encoder)

                if self.nPrevCmd == EnumStellaB2Cmd.GET_STATE:
                    robot_stat = EnumStellaB2Status(int(feedback[0]))
                    self.sig_get_state.emit(robot_stat)

                if self.nPrevCmd == EnumStellaB2Cmd.GET_VELOCITY:
                    right_vel, left_vel = 0., 0.
                    if feedback[0] == 'F':
                        right_vel = float(feedback[1:6])
                    elif feedback[0] == 'B':
                        right_vel = float(feedback[1:6]) * -1

                    if feedback[6] == 'F':
                        left_vel = float(feedback[7:12])
                    elif feedback[6] == 'B':
                        left_vel = float(feedback[7:12]) * -1

                    self.sig_get_vel.emit(right_vel, left_vel)

                if self.nPrevCmd == EnumStellaB2Cmd.GET_WHEEL_POSITION:
                    right_pos, left_pos = 0., 0.
                    if feedback[0] == 'F':
                        right_pos = float(feedback[1:9])
                    elif feedback[0] == 'B':
                        right_pos = float(feedback[1:9]) * -1

                    if feedback[9] == 'F':
                        left_pos = float(feedback[10:18])
                    elif feedback[9] == 'B':
                        left_pos = float(feedback[10:18]) * -1

                    self.sig_get_wheel_pos.emit(right_pos, left_pos)

                if self.nPrevCmd == EnumStellaB2Cmd.GET_VERSION:
                    try:
                        version = float(feedback[0:4])
                    except:
                        version = 0.00

                    self.sig_get_ver.emit("{:.2f}".format(version))

            self.btaSerialBuff.clear()


        except Exception:
            self.btaSerialBuff.clear()

    ### COMMANDS ###
    def move_stop(self, anStopType: EnumStellaB2Stop = EnumStellaB2Stop.MOTOR_HOLD_DECC):
        try:
            n_stop_type = int(EnumStellaB2Stop(anStopType))
        except:
            n_stop_type = int(EnumStellaB2Stop.MOTOR_HOLD_DECC)

        buff = bytearray()
        buff_str = "CSTOP" + str(n_stop_type)
        buff.extend(map(ord, buff_str))
        self.send_packet(0, buff)

    def move_joint_space(self, anAngVelR: float, anAngVelL: float, abGetEnc: bool = False):
        buff = bytearray()
        buff_str = "CV"
        buff.extend(map(ord, buff_str))

        # todo: add joint limits as a robot parameter
        n_AngVelR = abs(anAngVelR)
        if n_AngVelR > 270:
            n_AngVelR = 270

        if 0 <= anAngVelR:
            tmpR = "F" + str("{:03d}").format(n_AngVelR)
        else:
            tmpR = "B" + str("{:03d}").format(n_AngVelR)

        buff.extend(map(ord, tmpR))

        # todo: add joint limits as a robot parameter
        n_AngVelL = abs(anAngVelL)
        if n_AngVelL > 270:
            n_AngVelL = 270

        if 0 <= anAngVelL:
            tmpL = "F" + str("{:03d}").format(n_AngVelL)
        else:
            tmpL = "B" + str("{:03d}").format(n_AngVelL)

        buff.extend(map(ord, tmpL))

        nCmd = EnumStellaB2Cmd.MOVE_JOINT_SPACE
        if abGetEnc:
            buff.extend(map(ord, "E"))
            nCmd = EnumStellaB2Cmd.MOVE_GET_ENCODER

        self.send_packet(nCmd, buff, abGetEnc)

    def set_velocity_control(self, afVel: float, afYawRate: float):
        f_r, f_l = self.calculate_joint_space_vel_actl(afVel, afYawRate)
        # todo: check scaling of joint space velocities

        self.move_joint_space(int(f_r), int(f_l))

    def reset_motor(self, anMotor: EnumStellaB2MotorCtrl = EnumStellaB2MotorCtrl.ALL):
        try:
            n_motor = EnumStellaB2MotorCtrl(anMotor)
        except:
            n_motor = EnumStellaB2MotorCtrl.ALL

        if n_motor == EnumStellaB2MotorCtrl.LEFT:
            str_motor = "L"
        elif n_motor == EnumStellaB2MotorCtrl.RIGHT:
            str_motor = "R"
        else:
            str_motor = "A"

        buff = bytearray()
        buff_str = "CRESET" + str_motor
        buff.extend(map(ord, buff_str))
        self.send_packet(0, buff)

    def set_factory_settings(self):
        buff = bytearray()
        buff_str = "CINIT"
        buff.extend(map(ord, buff_str))
        self.send_packet(0, buff)

    def get_state(self):
        buff = bytearray()
        buff_str = "GSTATE"
        buff.extend(map(ord, buff_str))
        self.send_packet(EnumStellaB2Cmd.GET_STATE, buff, True)

    def get_motor_velocity(self, anMotor: EnumStellaB2MotorCtrl = EnumStellaB2MotorCtrl.ALL):
        try:
            n_motor = EnumStellaB2MotorCtrl(anMotor)
        except:
            n_motor = EnumStellaB2MotorCtrl.ALL

        if n_motor == EnumStellaB2MotorCtrl.LEFT:
            str_motor = "L"
        elif n_motor == EnumStellaB2MotorCtrl.RIGHT:
            str_motor = "R"
        else:
            str_motor = "A"

        buff = bytearray()
        buff_str = "GVELOCITY" + str_motor
        buff.extend(map(ord, buff_str))
        self.send_packet(EnumStellaB2Cmd.GET_VELOCITY, buff, True)

    def get_motor_position(self, anMotor: EnumStellaB2MotorCtrl = EnumStellaB2MotorCtrl.ALL):
        try:
            n_motor = EnumStellaB2MotorCtrl(anMotor)
        except:
            n_motor = EnumStellaB2MotorCtrl.ALL

        if n_motor == EnumStellaB2MotorCtrl.LEFT:
            str_motor = "L"
        elif n_motor == EnumStellaB2MotorCtrl.RIGHT:
            str_motor = "R"
        else:
            str_motor = "A"

        buff = bytearray()
        buff_str = "GPOSITION" + str_motor
        buff.extend(map(ord, buff_str))
        self.send_packet(EnumStellaB2Cmd.GET_WHEEL_POSITION, buff)

    def get_version(self):
        buff = bytearray()
        buff_str = "GVERSION"
        buff.extend(map(ord, buff_str))
        self.send_packet(EnumStellaB2Cmd.GET_VERSION, buff, True)

    def get_encoder_pulse(self):
        buff = bytearray()
        buff_str = "GENC"
        buff.extend(map(ord, buff_str))
        self.send_packet(EnumStellaB2Cmd.GET_ENCODER, buff, True)

    def make_packet(self, abtaPayload: bytearray) -> bytearray:
        buff = bytearray([0x02])  # STX
        buff.extend(abtaPayload)
        buff.append(0x03)  # ETX
        return buff

    def send_packet(self, anCmd: int, abtaPayload: bytearray, abIsWaitFeedback: bool = False):
        try:
            tmp_cmd = EnumStellaB2Cmd(anCmd)
        except:
            tmp_cmd = EnumStellaB2Cmd.GET_ENCODER

        # print(tmp_cmd)

        self.nPrevCmd = tmp_cmd
        buff = self.make_packet(abtaPayload)

        if self._serial.is_connected():
            self._serial.sendData(buff, abIsWaitFeedback)
            return True
        else:
            return False


# todo: try to implement this on the parent class
class CThreadStateMonitor(threading.Thread):
    _keepAlive: bool

    def __init__(self, robot: CTMRStellaB2):
        super(CThreadStateMonitor, self).__init__()
        self.setDaemon(True)
        self._keepAlive = True
        self._robot = robot
        self.sig_is_run = PySignal()
        self.sig_is_done = PySignal()

    def stop(self):
        self._keepAlive = False

    def run(self) -> None:
        self.sig_is_run.emit()
        while self._keepAlive:
            self._robot.get_state()
            time.sleep(0.02)
            self._robot.get_encoder_pulse()
            time.sleep(0.02)
            self._robot.get_motor_velocity()
            time.sleep(0.02)

        self.sig_is_done.emit()


