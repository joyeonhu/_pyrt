# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DoorDetector.py
# Project Name : HealthcareRobotPyRT
# Description  : Door detector for guide/delivery arrival checking
# -------------------------------------------------------------------------------------------------------------------- #

import os
import re
import cv2
import numpy as np

from ultralytics import YOLO

from Commons import *
from HealthcareRobot.HealthcareMessage import *

try:
    import torch
    from PIL import Image
    from transformers import Qwen2VLForConditionalGeneration, AutoProcessor
    from qwen_vl_utils import process_vision_info
except ImportError:
    torch = None
    Image = None
    Qwen2VLForConditionalGeneration = None
    AutoProcessor = None
    process_vision_info = None


class CDoorDetector:
    """
    문 검출기

    역할:
        - YOLO door model을 이용해서 frame 안의 문을 검출
        - 검출된 door bbox를 반환
        - 필요하면 door crop 이미지를 반환
        - door crop을 VLM에 넣어서 문/호실 글자를 읽음
        - 읽은 글자가 destination_id와 맞는지 확인
    """

    def __init__(
            self,
            yolo_model_path: str = None,
            door_conf: float = 0.85,
            vlm_model_id: str = "Qwen/Qwen2-VL-2B-Instruct",
            device: str = None,
    ):

        if yolo_model_path is None:
            root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
            yolo_model_path = os.path.join(root_path, "Data", "Models", "door_yolov8n.pt")

        self._yolo_model_path = yolo_model_path
        self._door_conf = door_conf # 문 검출 confidence threshold

        self._door_model = None # YOLO 모델 객체

        self._vlm_model_id = vlm_model_id # VLM 모델 ID

        if device is None: # device가 명시적으로 지정되지 않은 경우, CUDA 사용 가능하면 CUDA, 아니면 CPU로 설정
            if torch is not None and torch.cuda.is_available(): # CUDA 사용 가능 여부 확인
                device = "cuda" # CUDA 사용 가능하면 device를 "cuda"로 설정
            else:
                device = "cpu" # CUDA 사용 불가능하면 device를 "cpu"로 설정

        self._device = device # 모델이 실행될 디바이스 ("cuda" 또는 "cpu")

        self._vlm_model = None # VLM 모델 객체
        self._vlm_processor = None # VLM 프로세서 객체

        self.load_model() # YOLO 모델 로드

    # ==============================================================================================================
    # Load Model
    # ==============================================================================================================

    def load_model(self):
        """
        YOLO door detector 로드
        """

        if not os.path.exists(self._yolo_model_path):
            raise FileNotFoundError(
                "Door YOLO model not found: %s" % self._yolo_model_path
            )

        self._door_model = YOLO(self._yolo_model_path)

        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        _ = self._door_model.predict(
            dummy,
            conf=self._door_conf,
            verbose=False
        )

        write_log("Door detector loaded: %s" % self._yolo_model_path, self)

    def load_vlm(self):
        """
        VLM 모델 lazy loading
        """

        if self._vlm_model is not None and self._vlm_processor is not None: # 이미 VLM 모델과 프로세서가 로드되어 있으면 True 반환
            return True

        if (
                torch is None # torch 모듈이 없거나
                or Image is None # PIL Image 모듈이 없거나
                or Qwen2VLForConditionalGeneration is None # Qwen2VLForConditionalGeneration 클래스가 없거나
                or AutoProcessor is None # AutoProcessor 클래스가 없거나
                or process_vision_info is None # process_vision_info 함수가 없으면
        ):
            write_log("VLM libraries are not available.", self) # VLM 라이브러리가 사용 불가능하다는 로그 출력
            return False

        try: # VLM 모델과 프로세서 로드 시도
            write_log("Loading Door VLM: %s" % self._vlm_model_id, self)

            self._vlm_processor = AutoProcessor.from_pretrained( # VLM 프로세서 로드
                self._vlm_model_id
            )

            self._vlm_model = Qwen2VLForConditionalGeneration.from_pretrained( # VLM 모델 로드
                self._vlm_model_id,
                torch_dtype=torch.float16 if self._device == "cuda" else torch.float32,
                device_map="auto" if self._device == "cuda" else None,
            )

            if self._device == "cpu": # CPU로 실행하는 경우 모델을 CPU로 이동
                self._vlm_model.to("cpu")

            self._vlm_model.eval() # VLM 모델을 평가 모드로 설정

            write_log("Door VLM loaded.", self)

            return True

        except Exception:
            ErrorHandler().report()
            return False

    # ==============================================================================================================
    # Utils
    # ==============================================================================================================

    @staticmethod
    def clamp_box(x1, y1, x2, y2, width, height):
        """
        bbox가 이미지 밖으로 나가지 않도록 제한
        """

        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(width, x2)
        y2 = min(height, y2)

        return x1, y1, x2, y2

    @staticmethod
    def crop_door(image_bgr: np.ndarray, bbox):
        """
        door bbox 영역 crop
        """

        x1, y1, x2, y2 = bbox
        return image_bgr[y1:y2, x1:x2]

    # ==============================================================================================================
    # Detect
    # ==============================================================================================================

    def detect(self, image_bgr: np.ndarray):
        """
        frame에서 문을 검출한다.

        반환:
            [
                {
                    "bbox": [x1, y1, x2, y2],
                    "confidence": conf,
                    "crop": door_crop
                },
                ...
            ]
        """

        detected_doors = []

        if image_bgr is None:
            return detected_doors

        if image_bgr.size == 0:  # 이미지가 비어있는 경우 빈 결과 반환
            return detected_doors

        height, width = image_bgr.shape[:2]  # 이미지 크기 가져오기

        results = self._door_model.predict(
            image_bgr,
            conf=self._door_conf,
            verbose=False  # 로그 출력하지 않도록 설정
        )

        for r in results:
            if r.boxes is None:
                continue

            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                # b.xyxy : 검출된 객체의 bbox 좌표 (x1, y1, x2, y2) 형태로 반환, b.xyxy[0]은 첫 번째 검출된 객체의 bbox 좌표를 가져옴, .tolist()는 텐서를 리스트로 변환, map(int, ...)는 좌표 값을 정수로 변환하여 x1, y1, x2, y2에 할당
                x1, y1, x2, y2 = self.clamp_box(
                    x1, y1, x2, y2,
                    width,
                    height
                )

                conf = 0.0
                if b.conf is not None:
                    conf = float(b.conf[0])  # [0.92] 이렇게 텐서로 반환되므로 첫 번째 요소를 가져와서 float로 변환하여 confidence 값으로 사용

                bbox = [x1, y1, x2, y2]
                crop = self.crop_door(image_bgr, bbox)

                detected_doors.append({  # 검출된 문 정보를 결과 리스트에 추가, 각 문 정보는 bbox, confidence, crop 이미지를 포함하는 딕셔너리 형태
                    "bbox": bbox,
                    "confidence": conf,
                    "crop": crop,
                })

        return detected_doors

    def get_best_door(self, image_bgr: np.ndarray):
        """
        가장 confidence가 높은 door 하나 반환
        """

        doors = self.detect(image_bgr) # 이미지에서 문 검출하여 리스트로 반환

        if len(doors) == 0: # 검출된 문이 없는 경우 None 반환
            return None

        doors.sort( # 검출된 문 리스트를 confidence 기준으로 내림차순 정렬하여 가장 confidence가 높은 문이 리스트의 첫 번째 요소가 되도록 함
            key=lambda door: door.get("confidence", 0.0),
            reverse=True
        )

        return doors[0] # 가장 confidence가 높은 문 정보 반환

    # ==============================================================================================================
    # VLM Text Reading
    # ==============================================================================================================

    def read_door_text(self, door_crop_bgr: np.ndarray):
        """
        door crop 이미지를 VLM에 넣어서 문/호실 글자를 읽는다.
        """

        if door_crop_bgr is None: # door crop 이미지가 None인 경우
            return None

        if door_crop_bgr.size == 0: # door crop 이미지가 비어있는 경우
            return None

        if not self.load_vlm(): # VLM 모델이 로드되지 않은 경우
            return None

        try:
            door_crop_rgb = cv2.cvtColor(door_crop_bgr, cv2.COLOR_BGR2RGB) # BGR 이미지를 RGB 이미지로 변환
            image = Image.fromarray(door_crop_rgb) # NumPy 배열을 PIL 이미지로 변환

            prompt = ( # VLM에 제공할 프롬프트 텍스트, 문 글자 인식에 필요한 지시사항을 포함
                "Read the room number or text on this hospital door sign. "
                "Answer only the visible text or room number. "
                "If no text is visible, answer NONE."
            )

            messages = [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image",
                            "image": image,
                        },
                        {
                            "type": "text",
                            "text": prompt,
                        },
                    ],
                }
            ]

            text = self._vlm_processor.apply_chat_template( # VLM에 맞는 형식으로 메시지 텍스트 생성
                messages,
                tokenize=False,
                add_generation_prompt=True
            )

            image_inputs, _ = process_vision_info(messages) # 메시지에서 이미지 정보를 추출하여 VLM 입력 형식에 맞게 처리

            inputs = self._vlm_processor( # VLM 프로세서를 사용하여 텍스트와 이미지 입력을 토크나이즈하고 텐서로 변환하여 VLM 모델에 입력할 준비를 함
                text=[text],
                images=image_inputs,
                return_tensors="pt"
            ).to(self._vlm_model.device)

            with torch.no_grad(): # 그래디언트 계산 비활성화하여 VLM 모델의 추론 속도 향상 및 메모리 사용량 감소
                outputs = self._vlm_model.generate( # VLM 모델을 사용하여 텍스트 생성
                    **inputs,
                    max_new_tokens=32,
                    do_sample=False,
                )

            generated = outputs[:, inputs.input_ids.shape[1]:] # VLM 모델이 생성한 텍스트 부분만 추출

            result = self._vlm_processor.batch_decode( # 생성된 텍스트를 디코딩하여 사람이 읽을 수 있는 문자열로 변환
                generated,
                skip_special_tokens=True
            )[0].strip()

            return result

        except Exception:
            ErrorHandler().report()
            return None

    # ==============================================================================================================
    # Destination Check
    # ==============================================================================================================

    def check_destination(
            self,
            image_bgr: np.ndarray,
            destination_id: str
    ):
        """
        문 검출 + VLM 글자 읽기 + 목적지 일치 여부 확인
        """

        best_door = self.get_best_door(image_bgr) # 이미지에서 가장 confidence가 높은 문 하나 검출

        if best_door is None: # 문이 검출되지 않은 경우, 목적지 ID만 반환하고 나머지는 None 또는 False로 설정하여 반환
            return {
                KEY_DESTINATION_ID: destination_id,
                KEY_DOOR_TEXT: None,
                KEY_DOOR_BBOX: None,
                KEY_IS_MATCHED: False,
            }

        door_crop = best_door.get("crop", None) # 검출된 문 정보에서 crop 이미지를 가져옴, crop 이미지가 없는 경우 None 반환
        door_bbox = best_door.get("bbox", None) # 검출된 문 정보에서 bbox 좌표를 가져옴, bbox가 없는 경우 None 반환

        detected_text = self.read_door_text(door_crop) # door crop 이미지를 VLM에 넣어서 문 글자를 읽음, 읽은 글자가 없는 경우 None 반환

        is_matched = self.is_destination_matched( # 목적지 ID와 VLM이 읽은 문 글자 비교하여 일치 여부 확인, 일치하면 True, 아니면 False 반환
            destination_id,
            detected_text
        )

        return { # 목적지 ID, VLM이 읽은 문 글자, 문 bbox, 목적지 일치 여부를 포함하는 딕셔너리 형태로 결과 반환
            KEY_DESTINATION_ID: destination_id,
            KEY_DOOR_TEXT: detected_text,
            KEY_DOOR_BBOX: door_bbox,
            KEY_IS_MATCHED: is_matched,
        }

    @staticmethod
    def get_expected_text(destination_id: str):
        """
        destination_id에서 비교해야 할 실제 문 글자 추출

        예:
            room_19421 -> 19421
        """

        if destination_id is None: # destination_id가 None인 경우 None 반환
            return None

        destination_id = str(destination_id) # destination_id를 문자열로 변환

        match = re.search(r"\d+", destination_id) # destination_id에서 숫자 부분을 정규 표현식으로 검색하여 추출, 예를 들어 "room_19421"에서 "19421"을 추출

        if match: # 숫자 부분이 존재하는 경우, 추출된 숫자 부분을 반환
            return match.group(0) # 정규 표현식에서 첫 번째 그룹(숫자 부분)을 반환

        return destination_id.lower().strip() # 숫자 부분이 없는 경우, destination_id 전체를 소문자로 변환하고 양쪽 공백 제거하여 반환

    @staticmethod
    def normalize_text(text: str):
        """
        비교를 위한 문자열 정규화
        """

        if text is None: # text가 None인 경우 빈 문자열 반환
            return ""

        text = str(text).lower() # text를 문자열로 변환하고 소문자로 변환하여 대소문자 구분 없이 비교할 수 있도록 함
        text = re.sub(r"[^a-z0-9가-힣]", "", text) # text에서 영어 소문자, 숫자, 한글을 제외한 모든 문자를 제거하여 비교에 방해되는 특수 문자나 공백 등을 제거

        return text # 정규화된 문자열 반환

    def is_destination_matched(
            self,
            destination_id: str,
            detected_text: str
    ):
        """
        목적지 ID와 VLM이 읽은 문 글자 비교
        """

        expected = self.get_expected_text(destination_id) # destination_id에서 비교해야 할 실제 문 글자 추출

        expected_norm = self.normalize_text(expected) # 비교를 위한 문자열 정규화
        detected_norm = self.normalize_text(detected_text) # 비교를 위한 문자열 정규화

        if len(expected_norm) == 0 or len(detected_norm) == 0: # 정규화된 문자열 중 하나라도 길이가 0인 경우, 비교할 수 없으므로 False 반환
            return False

        return expected_norm in detected_norm # 정규화된 expected 문자열이 정규화된 detected 문자열에 포함되어 있는지 여부 반환, 예를 들어 expected가 "19421"이고 detected가 "room 19421"인 경우 True 반환

    # ==============================================================================================================
    # Draw
    # ==============================================================================================================

    def draw_result(self, image_bgr: np.ndarray, doors: list):
        """
        검출된 문 bbox를 화면에 표시
        """

        for door in doors:
            x1, y1, x2, y2 = door["bbox"]
            conf = door.get("confidence", 0.0)

            cv2.rectangle(
                image_bgr,
                (x1, y1),
                (x2, y2),
                (0, 255, 255),
                2
            )

            cv2.putText(
                image_bgr,
                "Door %.2f" % conf,
                (x1, y1 - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

        return image_bgr

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_yolo_model_path(self): # YOLO 모델 경로 반환
        return self._yolo_model_path

    def get_door_conf(self): # 문 검출 confidence threshold 반환
        return self._door_conf

    def get_vlm_model_id(self): # VLM 모델 ID 반환
        return self._vlm_model_id