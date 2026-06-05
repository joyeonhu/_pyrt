# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPHealthcareROS.py
# Project Name : HealthcareRobotPyRT
# Description  : Healthcare ROS multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT
from ROSIntegration.ROSHealthcareNode import CROSHealthcareNode


CMD_PROCESS_FOLLOW = "PROCESS_FOLLOW" # FOLLOW 명령, 카메라 프레임과 depth map을 받아서 FOLLOW 처리 수행 (예: 사람 인식 및 추적), 결과는 Nav2로 이동 명령으로 변환되어 ROSHealthcareNode에서 처리
CMD_START_GUIDE = "START_GUIDE" # GUIDE 명령, 목적지 ID를 받아서 해당 위치로 이동 시작
CMD_START_DELIVERY = "START_DELIVERY" # DELIVERY 명령, 목적지 ID를 받아서 해당 위치로 이동 시작
CMD_GO_TO_DESTINATION = "GO_TO_DESTINATION" # GO_TO_DESTINATION 명령, 목적지 ID를 받아서 해당 위치로 이동 시작
CMD_GO_TO_POSE = "GO_TO_POSE" # GO_TO_POSE 명령, x, y, yaw 정보를 받아서 해당 위치로 이동 시작
CMD_CANCEL_NAVIGATION = "CANCEL_NAVIGATION" # CANCEL_NAVIGATION 명령, 현재 진행 중인 네비게이션 취소
CMD_STOP_ROBOT = "STOP_ROBOT" # STOP_ROBOT 명령, 로봇을 즉시 정지, 네비게이션 취소 포함
CMD_PUBLISH_CMD_VEL = "PUBLISH_CMD_VEL" # PUBLISH_CMD_VEL 명령, 수동으로 cmd_vel을 받아서 ROSHealthcareNode를 통해 cmd_vel 토픽에 발행, data에는 linear_x와 angular_z가 포함되어야 함 (예: {"command": "PUBLISH_CMD_VEL", "linear_x": 0.5, "angular_z": 0.1})


def proc_healthcare_ros(command_pipe, feedback_queue, feedback_queue_bk=None):
    """
    Healthcare ROS Process Routine

    역할:
        1. ControlCore로부터 Pipe로 command 수신
        2. ROSHealthcareNode 초기화
        3. FOLLOW / GUIDE / DELIVERY / cmd_vel 명령 처리
        4. Nav2 feedback / 도착 결과를 ControlCore로 전달
        5. STOP / EXIT 명령 처리
    """

    ros_node = None # ROSHealthcareNode 객체, 이 객체는 ROS와의 통신을 담당, 네비게이션 명령을 보내고 피드백을 받는 기능을 포함

    is_running = True # 프로세스 실행 상태, EXIT 명령을 받으면 False로 변경되어 루프 종료
    is_ros_ready = False # ROS 준비 상태, START 명령을 받으면 True, STOP 명령을 받으면 False, 이 상태에 따라 FOLLOW/GO_TO_DESTINATION/GO_TO_POSE 등의 명령 처리 여부 결정

    write_log("MPHealthcareROS process routine started.")

    try:
        while is_running: # 프로세스가 종료 요청을 받을 때까지 반복

            # ------------------------------------------------------------------------------------------------------
            # 1. Command Receive
            # ------------------------------------------------------------------------------------------------------

            if command_pipe.poll(): # ControlCore에서 명령이 도착했는지 확인
                cmd_msg = command_pipe.recv() # 명령 메시지를 pipe에서 꺼냄

                data = cmd_msg.get(KEY_DATA, {}) # 메시지 안의 data 부분을 꺼냄, 없으면 빈 딕셔너리 사용
                command = data.get(KEY_COMMAND, None) # data 안에서 실제 command를 꺼냄

                if command == CMD_START: # 시작 명령, 이 명령이 들어오면 ROSHealthcareNode 객체를 초기화하고 ROS 준비 상태로 전환
                    if ros_node is None: # 객체가 아직 생성되지 않았다면
                        ros_node = CROSHealthcareNode() # ROSHealthcareNode 객체 생성, 이 객체는 ROS와의 통신을 담당, 네비게이션 명령을 보내고 피드백을 받는 기능을 포함

                    is_ros_ready = True # ROS 준비 상태로 전환

                    status_msg = make_status_message( # ROS 준비됐다는 상태 메시지 생성
                        "HEALTHCARE_ROS_READY",
                        PROC_HEALTHCARE_ROS,
                        PROC_CONTROL_CORE
                    )
                    feedback_queue.put(status_msg) # 상태 메시지를 ControlCore로 보냄

                elif command == CMD_PROCESS_FOLLOW: #
                    if not is_ros_ready or ros_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "Healthcare ROS is not ready",
                                PROC_HEALTHCARE_ROS,
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
                                PROC_HEALTHCARE_ROS,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    ros_node.process_follow(
                        frame_bgr,
                        depth_map
                    )

                    feedback_queue.put(
                        make_status_message(
                            "FOLLOW_PROCESSED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_START_GUIDE:
                    if not is_ros_ready or ros_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "Healthcare ROS is not ready",
                                PROC_HEALTHCARE_ROS,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get("destination_id", None)

                    success = ros_node.go_to_destination(destination_id)

                    feedback_queue.put(
                        make_status_message(
                            "GUIDE_STARTED" if success else "GUIDE_START_FAILED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE,
                            {
                                "destination_id": destination_id
                            }
                        )
                    )

                elif command == CMD_START_DELIVERY:
                    if not is_ros_ready or ros_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "Healthcare ROS is not ready",
                                PROC_HEALTHCARE_ROS,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get("destination_id", None)

                    success = ros_node.go_to_destination(destination_id)

                    feedback_queue.put(
                        make_status_message(
                            "DELIVERY_STARTED" if success else "DELIVERY_START_FAILED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE,
                            {
                                "destination_id": destination_id
                            }
                        )
                    )

                elif command == CMD_GO_TO_DESTINATION:
                    if not is_ros_ready or ros_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "Healthcare ROS is not ready",
                                PROC_HEALTHCARE_ROS,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get("destination_id", None)

                    success = ros_node.go_to_destination(destination_id)

                    feedback_queue.put(
                        make_status_message(
                            "NAVIGATION_STARTED" if success else "NAVIGATION_START_FAILED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE,
                            {
                                "destination_id": destination_id
                            }
                        )
                    )

                elif command == CMD_GO_TO_POSE:
                    if not is_ros_ready or ros_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "Healthcare ROS is not ready",
                                PROC_HEALTHCARE_ROS,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    x = data.get("x", None)
                    y = data.get("y", None)
                    yaw = data.get("yaw", None)
                    destination_id = data.get("destination_id", None)

                    if x is None or y is None or yaw is None:
                        feedback_queue.put(
                            make_error_message(
                                "GO_TO_POSE requires x, y, yaw",
                                PROC_HEALTHCARE_ROS,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    success = ros_node.go_to_pose(
                        x,
                        y,
                        yaw,
                        destination_id
                    )

                    feedback_queue.put(
                        make_status_message(
                            "NAVIGATION_STARTED" if success else "NAVIGATION_START_FAILED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE,
                            {
                                "destination_id": destination_id,
                                "x": x,
                                "y": y,
                                "yaw": yaw,
                            }
                        )
                    )

                elif command == CMD_CANCEL_NAVIGATION:
                    if ros_node is not None:
                        ros_node.cancel_navigation()

                    feedback_queue.put(
                        make_status_message(
                            "NAVIGATION_CANCEL_REQUESTED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_PUBLISH_CMD_VEL:
                    if not is_ros_ready or ros_node is None:
                        feedback_queue.put(
                            make_error_message(
                                "Healthcare ROS is not ready",
                                PROC_HEALTHCARE_ROS,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    linear_x = data.get(KEY_LINEAR_X, 0.0)
                    angular_z = data.get(KEY_ANGULAR_Z, 0.0)

                    ros_node.publish_cmd_vel(
                        linear_x,
                        angular_z
                    )

                    feedback_queue.put(
                        make_cmd_vel_message(
                            linear_x,
                            angular_z,
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_STOP_ROBOT:
                    if ros_node is not None:
                        ros_node.stop_robot()

                    feedback_queue.put(
                        make_status_message(
                            "ROBOT_STOPPED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_STOP:
                    if ros_node is not None:
                        ros_node.stop_robot()
                        ros_node.cancel_navigation()

                    is_ros_ready = False

                    feedback_queue.put(
                        make_status_message(
                            "HEALTHCARE_ROS_STOPPED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_EXIT:
                    is_ros_ready = False
                    is_running = False

            # ------------------------------------------------------------------------------------------------------
            # 2. ROS Callback Spin / Navigation State Check
            # ------------------------------------------------------------------------------------------------------

            if is_ros_ready and ros_node is not None:
                ros_node.spin_once(timeout_sec=0.0)

                if ros_node.is_arrived():
                    feedback_queue.put(
                        make_event_message(
                            "NAVIGATION_ARRIVED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE,
                            {
                                "result": ros_node.get_navigation_result(),
                                "distance_remaining": ros_node.get_distance_remaining(),
                            }
                        )
                    )

                nav_result = ros_node.get_navigation_result()

                if nav_result is not None and nav_result != "SUCCEEDED":
                    feedback_queue.put(
                        make_event_message(
                            "NAVIGATION_FAILED",
                            PROC_HEALTHCARE_ROS,
                            PROC_CONTROL_CORE,
                            {
                                "result": nav_result,
                                "distance_remaining": ros_node.get_distance_remaining(),
                            }
                        )
                    )

            time.sleep(0.001)

    except Exception:
        ErrorHandler().report()

        feedback_queue.put(
            make_error_message(
                "MPHealthcareROS process exception",
                PROC_HEALTHCARE_ROS,
                PROC_CONTROL_CORE
            )
        )

    finally:
        if ros_node is not None:
            ros_node.shutdown()

        feedback_queue.put(
            make_status_message(
                "HEALTHCARE_ROS_PROCESS_TERMINATED",
                PROC_HEALTHCARE_ROS,
                PROC_CONTROL_CORE
            )
        )

        write_log("MPHealthcareROS process routine terminated.")