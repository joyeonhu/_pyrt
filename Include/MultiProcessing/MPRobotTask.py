# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPRobotTask.py
# Project Name : HealthcareRobotPyRT
# Description  : Robot task multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

import rclpy

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from HealthcareRobot.HealthcareState import RobotEvent

from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT

from ROSIntegration.ROSFollowNode import CROSFollowNode
from ROSIntegration.ROSNavigationNode import CROSNavigationNode
from ROSIntegration.ROSCmdVelNode import CROSCmdVelNode

from Perception.PatientDetector import CPatientDetector
from Perception.DoorDetector import CDoorDetector


CMD_PROCESS_FOLLOW = "PROCESS_FOLLOW"

CMD_START_GUIDE = "START_GUIDE"
CMD_START_DELIVERY = "START_DELIVERY"

CMD_GO_TO_DESTINATION = "GO_TO_DESTINATION"
CMD_GO_TO_POSE = "GO_TO_POSE"

CMD_CANCEL_NAVIGATION = "CANCEL_NAVIGATION"
CMD_STOP_ROBOT = "STOP_ROBOT"
CMD_PUBLISH_CMD_VEL = "PUBLISH_CMD_VEL"

CMD_CHECK_PATIENT_ON_PATH = "CHECK_PATIENT_ON_PATH"
CMD_CHECK_DOOR_ARRIVAL = "CHECK_DOOR_ARRIVAL"


def proc_robot_task(command_pipe, feedback_queue, feedback_queue_bk=None):
    """
    Robot Task Process Routine

    역할:
        1. ROS Follow / Navigation / CmdVel 노드 초기화
        2. FOLLOW / GUIDE / DELIVERY / cmd_vel 명령 처리
        3. 배달 중 환자 감지
        4. 배달 도착 후 문/호실 확인
        5. Nav2 도착/실패 결과를 ControlCore로 전달
    """

    follow_node = None
    navigation_node = None
    cmd_vel_node = None

    patient_detector = None
    door_detector = None

    is_running = True
    is_robot_task_ready = False

    last_arrived_sent = False
    last_failed_sent = False

    write_log("MPRobotTask process routine started.")

    try:
        while is_running:

            if command_pipe.poll():
                cmd_msg = command_pipe.recv()

                data = cmd_msg.get(KEY_DATA, {})
                command = data.get(KEY_COMMAND, None)

                # ==============================================================================================
                # START
                # ==============================================================================================

                if command == CMD_START:
                    if not rclpy.ok():
                        rclpy.init()

                    if follow_node is None:
                        follow_node = CROSFollowNode()

                    if navigation_node is None:
                        navigation_node = CROSNavigationNode()

                    if cmd_vel_node is None:
                        cmd_vel_node = CROSCmdVelNode(
                            node_name="healthcare_robot_task_cmd_vel_node"
                        )

                    if patient_detector is None:
                        patient_detector = CPatientDetector()

                    if door_detector is None:
                        door_detector = CDoorDetector()

                    is_robot_task_ready = True

                    feedback_queue.put(
                        make_status_message(
                            "ROBOT_TASK_READY",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # FOLLOW
                # ==============================================================================================

                elif command == CMD_PROCESS_FOLLOW:
                    if not is_robot_task_ready or follow_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    frame_bgr = data.get(KEY_FRAME, None)
                    depth_map = data.get("depth_map", None)

                    if frame_bgr is None:
                        feedback_queue.put(
                            make_status_message(
                                "FOLLOW_NO_FRAME",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    linear_x, angular_z = follow_node.process(
                        frame_bgr,
                        depth_map
                    )

                    feedback_queue.put(
                        make_cmd_vel_message(
                            linear_x,
                            angular_z,
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                    feedback_queue.put(
                        make_status_message(
                            "FOLLOW_PROCESSED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # GUIDE / DELIVERY / NAVIGATION
                # ==============================================================================================

                elif command == CMD_START_GUIDE:
                    if not is_robot_task_ready or navigation_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get(KEY_DESTINATION_ID, None)

                    last_arrived_sent = False
                    last_failed_sent = False

                    success = navigation_node.go_to_destination(destination_id)

                    feedback_queue.put(
                        make_status_message(
                            "GUIDE_STARTED" if success else "GUIDE_START_FAILED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                KEY_DESTINATION_ID: destination_id
                            }
                        )
                    )

                elif command == CMD_START_DELIVERY:
                    if not is_robot_task_ready or navigation_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get(KEY_DESTINATION_ID, None)

                    last_arrived_sent = False
                    last_failed_sent = False

                    success = navigation_node.go_to_destination(destination_id)

                    feedback_queue.put(
                        make_status_message(
                            "DELIVERY_STARTED" if success else "DELIVERY_START_FAILED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                KEY_DESTINATION_ID: destination_id
                            }
                        )
                    )

                elif command == CMD_GO_TO_DESTINATION:
                    if not is_robot_task_ready or navigation_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get(KEY_DESTINATION_ID, None)

                    last_arrived_sent = False
                    last_failed_sent = False

                    success = navigation_node.go_to_destination(destination_id)

                    feedback_queue.put(
                        make_status_message(
                            "NAVIGATION_STARTED" if success else "NAVIGATION_START_FAILED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                KEY_DESTINATION_ID: destination_id
                            }
                        )
                    )

                elif command == CMD_GO_TO_POSE:
                    if not is_robot_task_ready or navigation_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    x = data.get("x", None)
                    y = data.get("y", None)
                    yaw = data.get("yaw", None)
                    destination_id = data.get(KEY_DESTINATION_ID, None)

                    if x is None or y is None or yaw is None:
                        feedback_queue.put(
                            make_error_message(
                                "GO_TO_POSE requires x, y, yaw",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    last_arrived_sent = False
                    last_failed_sent = False

                    success = navigation_node.go_to_pose(
                        x,
                        y,
                        yaw,
                        destination_id
                    )

                    feedback_queue.put(
                        make_status_message(
                            "NAVIGATION_STARTED" if success else "NAVIGATION_START_FAILED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                KEY_DESTINATION_ID: destination_id,
                                "x": x,
                                "y": y,
                                "yaw": yaw,
                            }
                        )
                    )

                # ==============================================================================================
                # DELIVERY: 환자 감지
                # ==============================================================================================

                elif command == CMD_CHECK_PATIENT_ON_PATH:
                    if not is_robot_task_ready or patient_detector is None:
                        feedback_queue.put(
                            make_error_message(
                                "PatientDetector is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    frame_bgr = data.get(KEY_FRAME, None)

                    if frame_bgr is None:
                        feedback_queue.put(
                            make_status_message(
                                "PATIENT_CHECK_NO_FRAME",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    patients = patient_detector.detect(frame_bgr)

                    if patients is not None and len(patients) > 0:
                        patient_info = patients[0]

                        feedback_queue.put(
                            make_event_message(
                                RobotEvent.PATIENT_DETECTED.value,
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE,
                                {
                                    KEY_PATIENT_INFO: patient_info
                                }
                            )
                        )

                # ==============================================================================================
                # DELIVERY_VERIFY: 문 / 호실 확인
                # ==============================================================================================

                elif command == CMD_CHECK_DOOR_ARRIVAL:
                    if not is_robot_task_ready or door_detector is None:
                        feedback_queue.put(
                            make_error_message(
                                "DoorDetector is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    frame_bgr = data.get(KEY_FRAME, None)
                    destination_id = data.get(KEY_DESTINATION_ID, None)

                    if destination_id is None:
                        feedback_queue.put(
                            make_error_message(
                                "CMD_CHECK_DOOR_ARRIVAL requires destination_id",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    verify_result = door_detector.check_destination(
                        frame_bgr,
                        destination_id
                    )

                    if verify_result is None:
                        verify_result = {
                            KEY_DESTINATION_ID: destination_id,
                            KEY_DOOR_TEXT: None,
                            KEY_DOOR_BBOX: None,
                            KEY_IS_MATCHED: False,
                        }

                    detected_text = verify_result.get(KEY_DOOR_TEXT, None)
                    door_bbox = verify_result.get(KEY_DOOR_BBOX, None)
                    is_matched = verify_result.get(KEY_IS_MATCHED, False)

                    feedback_queue.put(
                        make_delivery_verify_message(
                            destination_id,
                            detected_text,
                            is_matched,
                            door_bbox,
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # Direct cmd_vel / stop / cancel
                # ==============================================================================================

                elif command == CMD_CANCEL_NAVIGATION:
                    if navigation_node is not None:
                        navigation_node.cancel_navigation()

                    feedback_queue.put(
                        make_status_message(
                            "NAVIGATION_CANCEL_REQUESTED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_PUBLISH_CMD_VEL:
                    if not is_robot_task_ready or cmd_vel_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    linear_x = data.get(KEY_LINEAR_X, 0.0)
                    angular_z = data.get(KEY_ANGULAR_Z, 0.0)

                    cmd_vel_node.publish_cmd_vel(
                        linear_x,
                        angular_z
                    )

                elif command == CMD_STOP_ROBOT:
                    if follow_node is not None:
                        follow_node.stop()

                    if cmd_vel_node is not None:
                        cmd_vel_node.stop_robot()

                    feedback_queue.put(
                        make_cmd_vel_message(
                            0.0,
                            0.0,
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                    feedback_queue.put(
                        make_status_message(
                            "ROBOT_STOPPED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_STOP:
                    if follow_node is not None:
                        follow_node.stop()

                    if cmd_vel_node is not None:
                        cmd_vel_node.stop_robot()

                    if navigation_node is not None:
                        navigation_node.cancel_navigation()

                    is_robot_task_ready = False

                    feedback_queue.put(
                        make_status_message(
                            "ROBOT_TASK_STOPPED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_EXIT:
                    if follow_node is not None:
                        follow_node.stop()

                    if cmd_vel_node is not None:
                        cmd_vel_node.stop_robot()

                    if navigation_node is not None:
                        navigation_node.cancel_navigation()

                    is_robot_task_ready = False
                    is_running = False

            # ==============================================================================================
            # ROS spin / Navigation result check
            # ==============================================================================================

            if is_robot_task_ready and navigation_node is not None:

                rclpy.spin_once(
                    navigation_node,
                    timeout_sec=0.0
                )

                if cmd_vel_node is not None:
                    rclpy.spin_once(
                        cmd_vel_node,
                        timeout_sec=0.0
                    )

                nav_result = navigation_node.get_last_result()

                if navigation_node.is_arrived() and not last_arrived_sent:
                    last_arrived_sent = True

                    feedback_queue.put(
                        make_event_message(
                            RobotEvent.NAVIGATION_ARRIVED.value,
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                "result": nav_result,
                                "distance_remaining": navigation_node.get_distance_remaining(),
                            }
                        )
                    )

                elif (
                        nav_result is not None
                        and nav_result not in ["SUCCEEDED", "NO_GOAL"]
                        and not last_failed_sent
                ):
                    last_failed_sent = True

                    feedback_queue.put(
                        make_event_message(
                            RobotEvent.NAVIGATION_FAILED.value,
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                "result": nav_result,
                                "distance_remaining": navigation_node.get_distance_remaining(),
                            }
                        )
                    )

            time.sleep(0.001)

    except Exception:
        ErrorHandler().report()

        feedback_queue.put(
            make_error_message(
                "MPRobotTask process exception",
                PROC_HEALTHCARE,
                PROC_CONTROL_CORE
            )
        )

    finally:
        try:
            if follow_node is not None:
                follow_node.stop()

            if cmd_vel_node is not None:
                cmd_vel_node.stop_robot()
                cmd_vel_node.destroy_node()

            if navigation_node is not None:
                navigation_node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

        except Exception:
            ErrorHandler().report()

        feedback_queue.put(
            make_status_message(
                "ROBOT_TASK_PROCESS_TERMINATED",
                PROC_HEALTHCARE,
                PROC_CONTROL_CORE
            )
        )

        write_log("MPRobotTask process routine terminated.")