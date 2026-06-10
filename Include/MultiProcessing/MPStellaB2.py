# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPStellaB2.py
# Project Name : HealthcareRobotPyRT
# Description  : Stella B2 multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from Commons import *
from HealthcareRobot.HealthcareMessage import *

from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT

from MobileRobot.TMRStellaB2 import (
    CTMRStellaB2,
    EnumStellaB2Stop,
)


CMD_CONNECT = "CONNECT"
CMD_DISCONNECT = "DISCONNECT"


class CStellaB2CmdVelNode(Node):
    def __init__(self, callback):
        super().__init__("healthcare_stella_b2_node")
        self.create_subscription(Twist, "/cmd_vel", callback, 10)
        write_log("StellaB2CmdVelNode initialized. Subscribed to /cmd_vel", self)

CMD_SET_VELOCITY_CONTROL = "SET_VELOCITY_CONTROL"
CMD_MOVE_STOP = "MOVE_STOP"

CMD_RESET_MOTOR = "RESET_MOTOR"
CMD_INIT_MOTOR = "INIT_MOTOR"

CMD_GET_STATE = "GET_STATE"


def proc_stella_b2(command_pipe, feedback_queue, feedback_queue_bk=None):
    """
    Stella B2 Process Routine

    역할:
        1. STELLA B2 객체 생성
        2. 시리얼 연결
        3. linear_x / angular_z 속도 명령을 STELLA B2로 전달
        4. 정지 / 연결 해제 / 종료 처리
    """

    robot = None
    cmd_vel_node = None

    is_running = True
    is_connected = False

    def on_cmd_vel(msg: Twist):

        if robot is None:
            return

        if not robot.is_connected():
            return

        robot.set_velocity_control(
            float(msg.linear.x),
            float(msg.angular.z)
        )

    write_log("MPStellaB2 process routine started.")

    try:
        while is_running:

            if command_pipe.poll():

                cmd_msg = command_pipe.recv()

                data = cmd_msg.get(KEY_DATA, {})
                command = data.get(KEY_COMMAND, None)

                # ==============================================================================================
                # START / CONNECT
                # ==============================================================================================

                if command == CMD_START:

                    if robot is None:
                        robot = CTMRStellaB2()

                    if not rclpy.ok():
                        rclpy.init()

                    if cmd_vel_node is None:
                        cmd_vel_node = CStellaB2CmdVelNode(on_cmd_vel)

                    feedback_queue.put(
                        make_status_message(
                            "STELLA_B2_CREATED",
                            PROC_STELLA_B2,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_CONNECT:

                    if robot is None:
                        robot = CTMRStellaB2()

                    port = data.get(KEY_PORT, None)

                    robot.connect(port)

                    time.sleep(0.2)

                    is_connected = robot.is_connected()

                    feedback_queue.put(
                        make_status_message(
                            "STELLA_B2_CONNECTED" if is_connected else "STELLA_B2_CONNECT_FAILED",
                            PROC_STELLA_B2,
                            PROC_CONTROL_CORE,
                            {
                                "port": robot.strSerialPort if hasattr(robot, "strSerialPort") else port,
                                "is_connected": is_connected,
                            }
                        )
                    )

                # ==============================================================================================
                # VELOCITY
                # ==============================================================================================

                elif command == CMD_SET_VELOCITY_CONTROL:

                    if robot is None or not is_connected:
                        feedback_queue.put(
                            make_error_message(
                                "Stella B2 is not connected",
                                PROC_STELLA_B2,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    linear_x = data.get(KEY_LINEAR_X, 0.0)
                    angular_z = data.get(KEY_ANGULAR_Z, 0.0)

                    robot.set_velocity_control(
                        float(linear_x),
                        float(angular_z)
                    )

                    feedback_queue.put(
                        make_cmd_vel_message(
                            float(linear_x),
                            float(angular_z),
                            PROC_STELLA_B2,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # STOP
                # ==============================================================================================

                elif command == CMD_MOVE_STOP or command == CMD_STOP:

                    if robot is not None and is_connected:
                        robot.move_stop(EnumStellaB2Stop.MOTOR_HOLD_DECC)

                    feedback_queue.put(
                        make_status_message(
                            "STELLA_B2_STOPPED",
                            PROC_STELLA_B2,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # MOTOR
                # ==============================================================================================

                elif command == CMD_RESET_MOTOR:

                    if robot is not None and is_connected:
                        robot.reset_motor()

                    feedback_queue.put(
                        make_status_message(
                            "STELLA_B2_MOTOR_RESET",
                            PROC_STELLA_B2,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_INIT_MOTOR:

                    if robot is not None and is_connected:
                        robot.set_factory_settings()

                    feedback_queue.put(
                        make_status_message(
                            "STELLA_B2_MOTOR_INIT",
                            PROC_STELLA_B2,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # STATE
                # ==============================================================================================

                elif command == CMD_GET_STATE:

                    if robot is not None and is_connected:
                        robot.get_state()

                    feedback_queue.put(
                        make_status_message(
                            "STELLA_B2_STATE_REQUESTED",
                            PROC_STELLA_B2,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # DISCONNECT / EXIT
                # ==============================================================================================

                elif command == CMD_DISCONNECT:

                    if robot is not None:
                        robot.disconnect()

                    is_connected = False

                    feedback_queue.put(
                        make_status_message(
                            "STELLA_B2_DISCONNECTED",
                            PROC_STELLA_B2,
                            PROC_CONTROL_CORE
                        )
                    )


                elif command == CMD_EXIT:

                    if robot is not None:

                        if robot.is_connected():
                            robot.move_stop(

                                EnumStellaB2Stop.MOTOR_HOLD_DECC

                            )

                        robot.release()

                    is_connected = False

                    is_running = False

            if cmd_vel_node is not None:
                rclpy.spin_once(cmd_vel_node, timeout_sec=0.0)

            time.sleep(0.001)

    except Exception:
        ErrorHandler().report()

        feedback_queue.put(
            make_error_message(
                "MPStellaB2 process exception",
                PROC_STELLA_B2,
                PROC_CONTROL_CORE
            )
        )

    finally:

        try:
            if robot is not None:
                if robot.is_connected():
                    robot.move_stop(EnumStellaB2Stop.MOTOR_HOLD_DECC)
                robot.release()

            if cmd_vel_node is not None:
                cmd_vel_node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

        except Exception:
            ErrorHandler().report()

        feedback_queue.put(
            make_status_message(
                "STELLA_B2_PROCESS_TERMINATED",
                PROC_STELLA_B2,
                PROC_CONTROL_CORE
            )
        )

        write_log("MPStellaB2 process routine terminated.")