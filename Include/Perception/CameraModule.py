# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : CameraModule.py
# Project Name : HealthcareRobotPyRT
# Description  : ZED 2i camera module
# -------------------------------------------------------------------------------------------------------------------- #

import time
import numpy as np
import cv2

from Commons import *


try:
    import pyzed.sl as sl
except ImportError:
    sl = None


class CZEDCameraModule:
    """
    ZED 2i 카메라 모듈

    역할:
        1. ZED 카메라 open
        2. color frame 가져오기
        3. depth map 가져오기
        4. 특정 픽셀의 depth 값 반환
        5. 카메라 close
    """

    def __init__(
            self,
            resolution=None, # 카메라 해상도
            fps: int = 30, # 프레임
            depth_mode=None, # depth 모드
            coordinate_units=None, # depth 단위, ZED SDK에서 depth 값을 반환할 때 사용할 단위, 예: meter, millimeter, centimeter
    ):

        if sl is None:
            raise ImportError(
                "pyzed.sl 모듈을 찾을 수 없습니다. ZED SDK Python API 설치가 필요합니다."
            )

        self._fps = fps # 저장용 카메라 프레임 속도

        self._zed = sl.Camera() # ZED 카메라 객체 생성

        self._init_params = sl.InitParameters() # 카메라 초기화 설정 객체 생성
        self._init_params.camera_fps = fps # 카메라 프레임 설정용

        if resolution is None:
            self._init_params.camera_resolution = sl.RESOLUTION.HD720
        else:
            self._init_params.camera_resolution = resolution

        if depth_mode is None:
            self._init_params.depth_mode = sl.DEPTH_MODE.NEURAL # NEURAL : ZED SDK의 딥 러닝 기반 depth 모드, 일반적으로 더 정확한 depth 정보를 제공하지만 계산 비용이 더 높음, 다른 옵션으로는 PERFORMANCE (빠르지만 덜 정확), ULTRA (매우 정확하지만 매우 느림) 등이 있음, 프로젝트 요구 사항에 따라 적절한 depth 모드 선택 필요
        else:
            self._init_params.depth_mode = depth_mode

        if coordinate_units is None:
            self._init_params.coordinate_units = sl.UNIT.METER # depth 값 반환 단위 m로 설정
        else:
            self._init_params.coordinate_units = coordinate_units

        self._runtime_params = sl.RuntimeParameters() # grab 시 사용할 런타임 설정 객체 생성, grab : 카메라에서 새 프레임을 가져오는 작업, grab() 메서드는 카메라에서 새로운 프레임을 가져오고, 이때 RuntimeParameters 객체를 인자로 전달하여 grab 작업에 필요한 설정을 지정할 수 있음, 우리는 기본값 사용

        self._image_mat = sl.Mat() # ZED SDK에서 이미지 데이터를 저장하는 객체
        self._depth_mat = sl.Mat() # ZED SDK에서 depth 데이터를 저장하는 객체

        self._is_opened = False # 카메라가 열렸는지 여부를 나타내는 플래그
        self._last_frame_id = 0 # 마지막으로 grab된 프레임의 ID, grab() 메서드가 성공적으로 새로운 프레임을 가져올 때마다 이 값이 1씩 증가하여 각 프레임에 고유한 ID를 부여하는 데 사용
        self._last_timestamp = 0.0 # 마지막으로 grab된 프레임의 타임스탬프, grab() 메서드가 성공적으로 새로운 프레임을 가져올 때마다 이 값이 현재 시간으로 업데이트되어 각 프레임이 언제 grab되었는지 기록하는 데 사용

    # ==============================================================================================================
    # Open / Close
    # ==============================================================================================================

    def open(self) -> bool:
        """
        ZED 카메라를 연다.
        """

        err = self._zed.open(self._init_params) # err : sl.ERROR_CODE.SUCCESS이면 성공적으로 열렸음을 의미, 다른 값이면 열기에 실패

        if err != sl.ERROR_CODE.SUCCESS:
            write_log("ZED camera open failed: %s" % str(err), self)
            self._is_opened = False
            return False

        self._is_opened = True
        write_log("ZED camera opened.", self)
        return True

    def close(self):
        """
        ZED 카메라를 닫는다.
        """

        if self._is_opened:
            self._zed.close()
            self._is_opened = False
            write_log("ZED camera closed.", self)

    # ==============================================================================================================
    # Grab
    # ==============================================================================================================

    def grab(self) -> bool:
        """
        새 frame을 grab한다.
        """

        if not self._is_opened:
            return False

        err = self._zed.grab(self._runtime_params)
        # .grap() 파라미터 : RuntimeParameters 객체, grab() 메서드는 카메라에서 새로운 프레임을 가져오고, 이때 RuntimeParameters 객체를 인자로 전달하여 grab 작업에 필요한 설정을 지정할 수 있음, 우리는 기본값 사용
        # err : sl.ERROR_CODE.SUCCESS이면 성공적으로 grab되었음을 의미, 다른 값이면 grab에 실패

        if err != sl.ERROR_CODE.SUCCESS:
            return False

        self._last_frame_id += 1
        self._last_timestamp = time.time()

        return True

    # ==============================================================================================================
    # Retrieve Image / Depth
    # ==============================================================================================================

    def get_color_frame(self): # OpenCV에서 사용할 수 있는 BGR numpy 배열로 color frame 반환
        """
        현재 grab된 left color frame을 BGR numpy 배열로 반환한다.

        반환:
            frame_bgr 또는 None
        """

        if not self._is_opened:
            return None

        self._zed.retrieve_image( # ZED 카메라에서 현재 grab된 left color frame을 self._image_mat 객체에 저장
            self._image_mat,
            sl.VIEW.LEFT
        )

        frame = self._image_mat.get_data() # ZED image -> numpy 배열로 변환

        if frame is None:
            return None

        # ZED image는 보통 BGRA 형태로 들어온다.
        if frame.shape[2] == 4: # 채널이 4개인 경우 (BGRA)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # alpha 채널 제거하여 BGR로 변환
        else:
            frame_bgr = frame.copy() # 이미 BGR 형식이면 그대로 복사

        return frame_bgr

    def get_depth_map(self):
        """
        현재 grab된 depth map을 numpy 배열로 반환한다.

        반환:
            depth map 또는 None
        """

        if not self._is_opened:
            return None

        self._zed.retrieve_measure(
            self._depth_mat,
            sl.MEASURE.DEPTH
        )

        depth = self._depth_mat.get_data()

        return depth

    def get_depth_value(
            self,
            x: int,
            y: int
    ):
        """
        특정 픽셀의 depth 값을 반환한다.

        coordinate_units를 METER로 설정했으므로 단위는 meter.
        """

        if not self._is_opened:
            return None

        width = self._depth_mat.get_width()
        height = self._depth_mat.get_height()

        if width <= 0 or height <= 0: # depth map의 크기가 유효하지 않으면 retrieve_measure() 호출하여 depth map 업데이트
            self._zed.retrieve_measure(
                self._depth_mat,
                sl.MEASURE.DEPTH
            )
            width = self._depth_mat.get_width()
            height = self._depth_mat.get_height()

        if x < 0 or y < 0 or x >= width or y >= height:
            return None

        err, depth_value = self._depth_mat.get_value(x, y)

        if err != sl.ERROR_CODE.SUCCESS:
            return None

        if depth_value is None:
            return None

        if np.isnan(depth_value) or np.isinf(depth_value):
            return None

        return float(depth_value)

    # ==============================================================================================================
    # Get Frame Packet
    # ==============================================================================================================

    def get_frame_packet(self):
        """
        color frame과 depth map을 하나의 dict로 반환한다.

        CameraProcess에서 ControlCore로 보낼 때 사용하기 좋다.
        """

        if not self.grab():
            return None

        frame_bgr = self.get_color_frame()
        depth_map = self.get_depth_map()

        if frame_bgr is None:
            return None

        return {
            "frame_id": self._last_frame_id,
            "timestamp": self._last_timestamp,
            "frame_bgr": frame_bgr,
            "depth_map": depth_map,
        }

    # ==============================================================================================================
    # State
    # ==============================================================================================================

    def is_opened(self): # 카메라가 열렸는지 여부 반환
        return self._is_opened

    def get_frame_id(self): # 마지막으로 grab된 프레임의 ID 반환
        return self._last_frame_id

    def get_fps(self): # 카메라 프레임 속도 반환
        return self._fps