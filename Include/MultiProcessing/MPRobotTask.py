# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPRobotTask.py
# Project Name : HealthcareRobotPyRT
# Description  : Robot task multiprocessing routine
# -------------------------------------------------------------------------------------------------------------------- #

import time

from Commons import *
from HealthcareRobot.HealthcareMessage import *
from HealthcareRobot.HealthcareState import RobotEvent

from MultiProcessing.MultiProcessBase import CMD_START, CMD_STOP, CMD_EXIT

from ROSIntegration.ROSHealthcareNode import CROSHealthcareNode
from Perception.PatientDetector import CPatientDetector
from Perception.DoorDetector import CDoorDetector


CMD_PROCESS_FOLLOW = "PROCESS_FOLLOW" # Follow 명령 처리

CMD_START_GUIDE = "START_GUIDE" # Guide 명령 처리
CMD_START_DELIVERY = "START_DELIVERY" # Delivery 명령 처리

CMD_GO_TO_DESTINATION = "GO_TO_DESTINATION" # Navigation 명령 처리
CMD_GO_TO_POSE = "GO_TO_POSE" # Navigation 명령 처리 (목적지 ID 대신 좌표로 이동)

CMD_CANCEL_NAVIGATION = "CANCEL_NAVIGATION" # Navigation 취소 명령
CMD_STOP_ROBOT = "STOP_ROBOT" # 로봇 정지 명령 (긴급 정지 포함)
CMD_PUBLISH_CMD_VEL = "PUBLISH_CMD_VEL" # cmd_vel 직접 퍼블리시 명령 (원격 제어용)

CMD_CHECK_PATIENT_ON_PATH = "CHECK_PATIENT_ON_PATH" # 배달 경로 상에서 환자 감지 명령 (배달 중 환자 발견 시 즉시 보고)
CMD_CHECK_DOOR_ARRIVAL = "CHECK_DOOR_ARRIVAL" # 배달 도착 후 문/호실 확인 명령


def proc_robot_task(command_pipe, feedback_queue, feedback_queue_bk=None):
    """
    Robot Task Process Routine

    역할:
        1. ROSHealthcareNode 초기화
        2. FOLLOW / GUIDE / DELIVERY / cmd_vel 명령 처리
        3. 배달 중 환자 감지
        4. 배달 도착 후 문/호실 확인
        5. Nav2 도착/실패 결과를 ControlCore로 전달
    """

    ros_node = None # ROSHealthcareNode 인스턴스, ROS 인터페이스 담당
    patient_detector = None # CPatientDetector 인스턴스, 환자 감지 담당
    door_detector = None # CDoorDetector 인스턴스, 문/호실 확인 담당

    is_running = True # 프로세스 실행 여부
    is_robot_task_ready = False # 로봇 태스크 준비 여부, START 명령 처리 후 True

    last_nav_result = None # 마지막 네비게이션 결과 저장, 도착/실패 이벤트 중복 방지용
    last_arrived_sent = False # 마지막 도착 이벤트 전송 여부, 도착 이벤트 중복 방지용
    last_failed_sent = False # 마지막 실패 이벤트 전송 여부, 실패 이벤트 중복 방지용

    write_log("MPRobotTask process routine started.")

    try:
        while is_running: # 종료 명령이 들어올 때까지 루프

            if command_pipe.poll(): # 명령이 들어왔는지 확인
                cmd_msg = command_pipe.recv() # 명령 수신

                data = cmd_msg.get(KEY_DATA, {}) # 명령 데이터 추출
                command = data.get(KEY_COMMAND, None) # 명령 타입 추출

                # ==============================================================================================
                # START
                # ==============================================================================================

                if command == CMD_START: # 시스템 초기화 명령 처리
                    if ros_node is None: # ROSHealthcareNode가 아직 생성되지 않았다면 생성
                        ros_node = CROSHealthcareNode()

                    if patient_detector is None: # 환자 감지기가 아직 생성되지 않았다면 생성
                        patient_detector = CPatientDetector()

                    if door_detector is None: # 문/호실 확인기가 아직 생성되지 않았다면 생성
                        door_detector = CDoorDetector()

                    is_robot_task_ready = True # 로봇 태스크 준비 완료

                    feedback_queue.put( # 로봇 태스크 준비 완료 상태 메시지 전송
                        make_status_message(
                            "ROBOT_TASK_READY",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # FOLLOW
                # ==============================================================================================

                elif command == CMD_PROCESS_FOLLOW: # Follow 명령 처리
                    if not is_robot_task_ready or ros_node is None: # 로봇 태스크가 준비되지 않았거나 ROSHealthcareNode가 생성되지 않았다면 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # Follow 처리 불가 오류 메시지 전송
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    frame_bgr = data.get(KEY_FRAME, None) # Follow 처리에 사용할 BGR 이미지 프레임 추출
                    depth_map = data.get("depth_map", None) # Follow 처리에 사용할 깊이 맵 추출

                    if frame_bgr is None: # BGR 이미지 프레임이 없으면 Follow 처리할 수 없으므로 상태 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # Follow 처리 불가 상태 메시지 전송
                            make_status_message(
                                "FOLLOW_NO_FRAME",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    ros_node.process_follow(frame_bgr, depth_map) # ROSHealthcareNode의 Follow 처리 함수 호출, 내부에서 환자 검출 -> 거리 계산 -> APF -> /cmd_vel publish

                    feedback_queue.put( # Follow 처리 완료 상태 메시지 전송
                        make_status_message(
                            "FOLLOW_PROCESSED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                # ==============================================================================================
                # GUIDE / DELIVERY / NAVIGATION
                # ==============================================================================================

                elif command == CMD_START_GUIDE: # Guide 명령 처리
                    if not is_robot_task_ready or ros_node is None: # 로봇 태스크가 준비되지 않았거나 ROSHealthcareNode가 생성되지 않았다면 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # Guide 처리 불가 오류 메시지 전송
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get(KEY_DESTINATION_ID, None) # Guide 처리에 사용할 목적지 ID 추출

                    last_nav_result = None # 네비게이션 결과 초기화, 도착/실패 이벤트 중복 방지용
                    last_arrived_sent = False # 도착 이벤트 전송 여부 초기화, 도착 이벤트 중복 방지용
                    last_failed_sent = False # 실패 이벤트 전송 여부 초기화, 실패 이벤트 중복 방지용

                    success = ros_node.go_to_destination(destination_id) # ROSHealthcareNode의 목적지 이동 함수 호출, 내부에서 Nav2로 네비게이션 시작

                    feedback_queue.put( # Guide 시작 상태 메시지 전송, 성공 여부에 따라 GUIDE_STARTED 또는 GUIDE_START_FAILED 메시지 전송
                        make_status_message( # Guide 시작 상태 메시지 생성
                            "GUIDE_STARTED" if success else "GUIDE_START_FAILED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                KEY_DESTINATION_ID: destination_id
                            }
                        )
                    )

                elif command == CMD_START_DELIVERY: # Delivery 명령 처리
                    if not is_robot_task_ready or ros_node is None: # 로봇 태스크가 준비되지 않았거나 ROSHealthcareNode가 생성되지 않았다면 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # Delivery 처리 불가 오류 메시지 전송
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get(KEY_DESTINATION_ID, None) # Delivery 처리에 사용할 목적지 ID 추출

                    last_nav_result = None # 네비게이션 결과 초기화, 도착/실패 이벤트 중복 방지용
                    last_arrived_sent = False # 도착 이벤트 전송 여부 초기화, 도착 이벤트 중복 방지용
                    last_failed_sent = False # 실패 이벤트 전송 여부 초기화, 실패 이벤트 중복 방지용

                    success = ros_node.go_to_destination(destination_id) # ROSHealthcareNode의 목적지 이동 함수 호출, 내부에서 Nav2로 네비게이션 시작

                    feedback_queue.put( # Delivery 시작 상태 메시지 전송, 성공 여부에 따라 DELIVERY_STARTED 또는 DELIVERY_START_FAILED 메시지 전송
                        make_status_message( # Delivery 시작 상태 메시지 생성
                            "DELIVERY_STARTED" if success else "DELIVERY_START_FAILED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                KEY_DESTINATION_ID: destination_id
                            }
                        )
                    )

                elif command == CMD_GO_TO_DESTINATION: # Navigation 명령 처리 (Guide/Delivery 구분 없이 목적지로 이동)
                    if not is_robot_task_ready or ros_node is None: # 로봇 태스크가 준비되지 않았거나 ROSHealthcareNode가 생성되지 않았다면 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # Navigation 처리 불가 오류 메시지 전송
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    destination_id = data.get(KEY_DESTINATION_ID, None) # Navigation 처리에 사용할 목적지 ID 추출

                    last_nav_result = None # 네비게이션 결과 초기화, 도착/실패 이벤트 중복 방지용
                    last_arrived_sent = False # 도착 이벤트 전송 여부 초기화, 도착 이벤트 중복 방지용
                    last_failed_sent = False # 실패 이벤트 전송 여부 초기화, 실패 이벤트 중복 방지용

                    success = ros_node.go_to_destination(destination_id) # ROSHealthcareNode의 목적지 이동 함수 호출, 내부에서 Nav2로 네비게이션 시작

                    feedback_queue.put( # Navigation 시작 상태 메시지 전송, 성공 여부에 따라 NAVIGATION_STARTED 또는 NAVIGATION_START_FAILED 메시지 전송
                        make_status_message(
                            "NAVIGATION_STARTED" if success else "NAVIGATION_START_FAILED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                KEY_DESTINATION_ID: destination_id
                            }
                        )
                    )

                elif command == CMD_GO_TO_POSE: # Navigation 명령 처리 (목적지 ID 대신 좌표로 이동)
                    if not is_robot_task_ready or ros_node is None: # 로봇 태스크가 준비되지 않았거나 ROSHealthcareNode가 생성되지 않았다면 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # Navigation 처리 불가 오류 메시지 전송
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    x = data.get("x", None) # Navigation 처리에 사용할 목표 위치 x 좌표 추출
                    y = data.get("y", None) # Navigation 처리에 사용할 목표 위치 y 좌표 추출
                    yaw = data.get("yaw", None) # Navigation 처리에 사용할 목표 방향 yaw 추출
                    destination_id = data.get(KEY_DESTINATION_ID, None) # Navigation 처리에 사용할 목적지 ID 추출 (선택 사항, 목적지 ID도 함께 전달되면 로그에 포함)

                    if x is None or y is None or yaw is None: # x, y, yaw 중 하나라도 없으면 Navigation 처리할 수 없으므로 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # Navigation 처리 불가 오류 메시지 전송
                            make_error_message(
                                "GO_TO_POSE requires x, y, yaw",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    last_nav_result = None # 네비게이션 결과 초기화, 도착/실패 이벤트 중복 방지용
                    last_arrived_sent = False # 도착 이벤트 전송 여부 초기화, 도착 이벤트 중복 방지용
                    last_failed_sent = False # 실패 이벤트 전송 여부 초기화, 실패 이벤트 중복 방지용

                    success = ros_node.go_to_pose( # ROSHealthcareNode의 좌표 이동 함수 호출, 내부에서 Nav2로 네비게이션 시작
                        x,
                        y,
                        yaw,
                        destination_id
                    )

                    feedback_queue.put( # Navigation 시작 상태 메시지 전송, 성공 여부에 따라 NAVIGATION_STARTED 또는 NAVIGATION_START_FAILED 메시지 전송, 로그에 좌표 정보 포함
                        make_status_message( # Navigation 시작 상태 메시지 생성
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

                elif command == CMD_CHECK_PATIENT_ON_PATH: # 배달 경로 상에서 환자 감지 명령 처리, 배달 중에 환자가 갑자기 나타나는 상황에 대비하여 주기적으로 환자 감지 수행
                    if not is_robot_task_ready or patient_detector is None: # 로봇 태스크가 준비되지 않았거나 환자 감지기가 생성되지 않았다면 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # 환자 감지 처리 불가 오류 메시지 전송
                            make_error_message(
                                "PatientDetector is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    frame_bgr = data.get(KEY_FRAME, None) # 환자 감지 처리에 사용할 BGR 이미지 프레임 추출

                    if frame_bgr is None: # BGR 이미지 프레임이 없으면 환자 감지 처리할 수 없으므로 상태 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # 환자 감지 처리 불가 상태 메시지 전송
                            make_status_message(
                                "PATIENT_CHECK_NO_FRAME",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    patients = patient_detector.detect(frame_bgr) # 환자 감지기의 detect() 함수 호출하여 프레임에서 환자 감지 수행, 반환값은 감지된 환자 정보 리스트 (없으면 빈 리스트 또는 None)

                    if patients is not None and len(patients) > 0: # 환자가 감지된 경우, 첫 번째 환자 정보를 추출하여 이벤트 메시지로 전송
                        patient_info = patients[0] # 감지된 환자 정보 중 첫 번째 환자 정보 추출

                        feedback_queue.put( # 환자 감지 이벤트 메시지 전송, 환자 정보 포함
                            make_event_message(
                                RobotEvent.PATIENT_DETECTED.value,
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE,
                                {
                                    KEY_PATIENT_INFO: patient_info
                                }
                            )
                        )

                    else:
                        feedback_queue.put( # 환자가 감지되지 않은 경우 상태 메시지 전송
                            make_status_message(
                                "PATIENT_NOT_DETECTED",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )

                # ==============================================================================================
                # DELIVERY_VERIFY: 문 / 호실 확인
                # ==============================================================================================

                elif command == CMD_CHECK_DOOR_ARRIVAL: # 배달 도착 후 문/호실 확인 명령 처리, 네비게이션이 도착했지만 문/호실이 맞는지 최종 확인이 필요한 상황에 대비하여 도착 후 문/호실 확인 수행
                    if not is_robot_task_ready or door_detector is None: # 로봇 태스크가 준비되지 않았거나 문/호실 확인기가 생성되지 않았다면 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # 문/호실 확인 처리 불가 오류 메시지 전송
                            make_error_message(
                                "DoorDetector is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    frame_bgr = data.get(KEY_FRAME, None) # 문/호실 확인 처리에 사용할 BGR 이미지 프레임 추출
                    destination_id = data.get(KEY_DESTINATION_ID, None) # 문/호실 확인 처리에 사용할 목적지 ID 추출, 도착한 목적지가 맞는지 확인하는데 필요

                    if frame_bgr is None: # BGR 이미지 프레임이 없으면 문/호실 확인 처리할 수 없으므로 상태 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # 문/호실 확인 처리 불가 상태 메시지 전송
                            make_status_message(
                                "DOOR_CHECK_NO_FRAME",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    if destination_id is None: # 목적지 ID가 없으면 문/호실 확인 처리할 수 없으므로 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # 문/호실 확인 처리 불가 오류 메시지 전송
                            make_error_message(
                                "CMD_CHECK_DOOR_ARRIVAL requires destination_id",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    verify_result = door_detector.check_destination( # 문/호실 확인기의 check_destination() 함수 호출하여 프레임에서 목적지 ID에 해당하는 문/호실이 맞는지 확인 수행
                        frame_bgr,
                        destination_id
                    )

                    if verify_result is None: # 확인 결과가 None이면 문/호실 확인 처리 실패로 간주하여 기본값으로 초기화
                        verify_result = {
                            KEY_DESTINATION_ID: destination_id,
                            KEY_DOOR_TEXT: None,
                            KEY_DOOR_BBOX: None,
                            KEY_IS_MATCHED: False,
                        }

                    detected_text = verify_result.get(KEY_DOOR_TEXT, None) # 감지된 문/호실 텍스트 정보 추출, 도착한 목적지가 맞는지 확인하는데 필요
                    door_bbox = verify_result.get(KEY_DOOR_BBOX, None) # 감지된 문/호실의 바운딩 박스 정보 추출, 도착한 목적지가 맞는지 확인하는데 필요
                    is_matched = verify_result.get(KEY_IS_MATCHED, False) # 감지된 문/호실이 목적지 ID에 해당하는지 여부 추출, 도착한 목적지가 맞는지 확인하는데 필요

                    feedback_queue.put( # 문/호실 확인 결과 메시지 전송, 감지된 정보 포함
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

                elif command == CMD_CANCEL_NAVIGATION: # Navigation 취소 명령 처리, 네비게이션 도중에 경로를 변경해야 하거나 긴급 상황이 발생해서 네비게이션을 즉시 취소해야 하는 상황에 대비하여 네비게이션 취소 수행
                    if ros_node is not None: # ROSHealthcareNode가 생성되어 있다면 네비게이션 취소 함수 호출하여 네비게이션 취소, 내부에서 Nav2로 네비게이션 취소 요청
                        ros_node.cancel_navigation() # ROSHealthcareNode의 네비게이션 취소 함수 호출, 내부에서 Nav2로 네비게이션 취소 요청

                    feedback_queue.put( # 네비게이션 취소 완료 상태 메시지 전송
                        make_status_message(
                            "NAVIGATION_CANCEL_REQUESTED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_PUBLISH_CMD_VEL: # cmd_vel 직접 퍼블리시 명령 처리, 원격 제어용으로 cmd_vel을 직접 퍼블리시해야 하는 상황에 대비하여 cmd_vel 퍼블리시 수행
                    if not is_robot_task_ready or ros_node is None: # 로봇 태스크가 준비되지 않았거나 ROSHealthcareNode가 생성되지 않았다면 오류 메시지 전송하고 명령 처리 건너뜀
                        feedback_queue.put( # cmd_vel 퍼블리시 처리 불가 오류 메시지 전송
                            make_error_message(
                                "RobotTask is not ready",
                                PROC_HEALTHCARE,
                                PROC_CONTROL_CORE
                            )
                        )
                        continue

                    linear_x = data.get(KEY_LINEAR_X, 0.0) # cmd_vel 퍼블리시 처리에 사용할 선형 속도 x 추출, 기본값은 0.0 (정지)
                    angular_z = data.get(KEY_ANGULAR_Z, 0.0) # cmd_vel 퍼블리시 처리에 사용할 각속도 z 추출, 기본값은 0.0 (회전 없음)

                    ros_node.publish_cmd_vel(linear_x, angular_z) # ROSHealthcareNode의 cmd_vel 퍼블리시 함수 호출, 내부에서 /cmd_vel에 속도 명령 publish

                    feedback_queue.put( # cmd_vel 퍼블리시 완료 상태 메시지 전송, 퍼블리시한 속도 정보 포함
                        make_cmd_vel_message(
                            linear_x,
                            angular_z,
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_STOP_ROBOT: # 로봇 정지 명령 처리, 긴급 상황이 발생해서 로봇을 즉시 정지시켜야 하는 상황에 대비하여 로봇 정지 수행
                    if ros_node is not None: # ROSHealthcareNode가 생성되어 있다면 로봇 정지 함수 호출하여 로봇 정지, 내부에서 /cmd_vel에 0으로 퍼블리시하여 로봇 정지
                        ros_node.stop_robot() # ROSHealthcareNode의 로봇 정지 함수 호출, 내부에서 /cmd_vel에 0으로 퍼블리시하여 로봇 정지

                    feedback_queue.put( # 로봇 정지 완료 상태 메시지 전송
                        make_status_message(
                            "ROBOT_STOPPED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_STOP: # 로봇 태스크 완전 정지 명령 처리, 시스템을 완전히 초기화해야 하는 상황에 대비하여 ROSHealthcareNode 종료 및 로봇 태스크 준비 상태 초기화 수행
                    if ros_node is not None: # ROSHealthcareNode가 생성되어 있다면 로봇 정지 함수 호출하여 로봇 정지, 네비게이션 취소 함수 호출하여 네비게이션 취소, 내부에서 /cmd_vel에 0으로 퍼블리시하여 로봇 정지 및 Nav2로 네비게이션 취소 요청
                        ros_node.stop_robot() # ROSHealthcareNode의 로봇 정지 함수 호출, 내부에서 /cmd_vel에 0으로 퍼블리시하여 로봇 정지
                        ros_node.cancel_navigation() # ROSHealthcareNode의 네비게이션 취소 함수 호출, 내부에서 Nav2로 네비게이션 취소 요청

                    is_robot_task_ready = False # 로봇 태스크 준비 상태 초기화

                    feedback_queue.put( # 로봇 태스크 완전 정지 완료 상태 메시지 전송
                        make_status_message(
                            "ROBOT_TASK_STOPPED",
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE
                        )
                    )

                elif command == CMD_EXIT: # 프로세스 종료 명령 처리, 시스템 종료 시에만 사용되어야 하는 명령으로, ROSHealthcareNode 종료 및 루프 종료 수행
                    is_robot_task_ready = False # 로봇 태스크 준비 상태 초기화
                    is_running = False # 루프 종료 플래그 설정

            # ==============================================================================================
            # ROS spin / Navigation result check
            # ==============================================================================================

            if is_robot_task_ready and ros_node is not None: # 로봇 태스크가 준비되었고 ROSHealthcareNode가 생성되어 있다면 ROS spin 수행 및 네비게이션 결과 확인, 네비게이션이 도착했는지 또는 실패했는지 주기적으로 확인하여 도착/실패 이벤트 전송
                ros_node.spin_once(timeout_sec=0.0) # ROSHealthcareNode의 spin_once() 함수 호출하여 ROS 이벤트 처리, timeout_sec=0.0으로 설정하여 논블로킹 방식으로 호출

                nav_result = ros_node.get_navigation_result() # ROSHealthcareNode의 네비게이션 결과 반환 함수 호출하여 네비게이션 결과 확인, 도착/실패 여부를 반환 (예: "SUCCEEDED", "FAILED", "CANCELED" 등)

                if ros_node.is_arrived() and not last_arrived_sent: # 네비게이션이 도착했지만 아직 도착 이벤트를 전송하지 않았다면 도착 이벤트 전송, 도착 이벤트 중복 방지 위해 last_arrived_sent 플래그 사용
                    last_arrived_sent = True # 도착 이벤트 전송 여부 플래그 설정
                    last_nav_result = nav_result # 마지막 네비게이션 결과 저장

                    feedback_queue.put( # 네비게이션 도착 이벤트 메시지 전송, 네비게이션 결과 및 남은 거리 정보 포함
                        make_event_message(
                            RobotEvent.NAVIGATION_ARRIVED.value,
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                "result": nav_result,
                                "distance_remaining": ros_node.get_distance_remaining(),
                            }
                        )
                    )

                elif (
                        nav_result is not None # nav 결과가 있고
                        and nav_result != "SUCCEEDED" # nav 결과가 성공이 아니고
                        and not last_failed_sent # 아직 실패 이벤트를 전송하지 않았다면
                ):
                    last_failed_sent = True # 실패 이벤트 전송 여부 플래그 설정
                    last_nav_result = nav_result # 마지막 네비게이션 결과 저장

                    feedback_queue.put( # 네비게이션 실패 이벤트 메시지 전송, 네비게이션 결과 및 남은 거리 정보 포함
                        make_event_message(
                            RobotEvent.NAVIGATION_FAILED.value,
                            PROC_HEALTHCARE,
                            PROC_CONTROL_CORE,
                            {
                                "result": nav_result,
                                "distance_remaining": ros_node.get_distance_remaining(),
                            }
                        )
                    )

            time.sleep(0.001) # 루프 주기 조절, 너무 빠르게 루프가 도는 것을 방지하여 CPU 사용량 감소

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
        if ros_node is not None: # ROSHealthcareNode가 생성되어 있다면 종료 함수 호출하여 ROSHealthcareNode 종료, 내부에서 로봇 정지 및 ROS 노드 종료 수행
            ros_node.shutdown() # ROSHealthcareNode의 종료 함수 호출, 내부에서 로봇 정지 및 ROS 노드 종료 수행

        feedback_queue.put( # 프로세스 종료 상태 메시지 전송
            make_status_message(
                "ROBOT_TASK_PROCESS_TERMINATED",
                PROC_HEALTHCARE,
                PROC_CONTROL_CORE
            )
        )

        write_log("MPRobotTask process routine terminated.")