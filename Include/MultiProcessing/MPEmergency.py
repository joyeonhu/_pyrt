# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPEmergency.py
# Project Name : HealthcareRobotPyRT
# Description  : Emergency detection multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT

from Perception.EmergencyDetector import CEmergencyDetector


CMD_CHECK_EMERGENCY = "CHECK_EMERGENCY"


def proc_emergency(command_pipe, feedback_queue, feedback_queue_bk=None):

    detector = None

    is_running = True
    is_emergency_ready = False

    write_log("MPEmergency process routine started.")

    try:
        while is_running:

            if command_pipe.poll():

                cmd_msg = command_pipe.recv()

                data = cmd_msg.get(KEY_DATA, {})
                command = data.get(KEY_COMMAND, None)

                # ==========================================================================================
                # START
                # ==========================================================================================

                if command == CMD_START:

                    if detector is None:
                        detector = CEmergencyDetector()

                    is_emergency_ready = True

                    feedback_queue.put(
                        make_status_message(
                            "EMERGENCY_READY",
                            PROC_EMERGENCY,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==========================================================================================
                # CHECK
                # ==========================================================================================

                elif command == CMD_CHECK_EMERGENCY:

                    if not is_emergency_ready or detector is None:

                        feedback_queue.put(
                            make_error_message(
                                "EmergencyDetector is not ready",
                                PROC_EMERGENCY,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    frame_bgr = data.get(KEY_FRAME, None)

                    if frame_bgr is None:

                        feedback_queue.put(
                            make_status_message(
                                "EMERGENCY_NO_FRAME",
                                PROC_EMERGENCY,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    feedback_queue.put(
                        make_status_message(
                            "EMERGENCY_CHECKING",
                            PROC_EMERGENCY,
                            PROC_CONTROL_CORE
                        )
                    )

                    result = detector.detect(frame_bgr)

                    feedback_queue.put(
                        make_message(
                            MSG_TYPE_EMERGENCY,
                            PROC_EMERGENCY,
                            PROC_CONTROL_CORE,
                            result
                        )
                    )

                # ==========================================================================================
                # STOP
                # ==========================================================================================

                elif command == CMD_STOP:

                    is_emergency_ready = False

                    if detector is not None:
                        detector.reset()

                    feedback_queue.put(
                        make_status_message(
                            "EMERGENCY_STOPPED",
                            PROC_EMERGENCY,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==========================================================================================
                # EXIT
                # ==========================================================================================

                elif command == CMD_EXIT:

                    is_emergency_ready = False
                    is_running = False

            time.sleep(0.001)

    except Exception:

        ErrorHandler().report()

        feedback_queue.put(
            make_error_message(
                "MPEmergency process exception",
                PROC_EMERGENCY,
                PROC_CONTROL_CORE
            )
        )

    finally:

        if detector is not None:
            detector.reset()

        feedback_queue.put(
            make_status_message(
                "EMERGENCY_PROCESS_TERMINATED",
                PROC_EMERGENCY,
                PROC_CONTROL_CORE
            )
        )

        write_log("MPEmergency process routine terminated.")