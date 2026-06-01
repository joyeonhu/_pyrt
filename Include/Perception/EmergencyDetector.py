# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : EmergencyDetector.py
# Project Name : HealthcareRobotPyRT
# Description  : VLM-based emergency detector
# -------------------------------------------------------------------------------------------------------------------- #

import os
import cv2
import torch
from PIL import Image

from transformers import (
    Qwen2VLForConditionalGeneration,
    AutoProcessor,
    BitsAndBytesConfig,
)

from qwen_vl_utils import process_vision_info

from Commons import *


class CEmergencyDetector:
    """
    응급상황 감지기

    역할:
        1. camera frame을 VLM에 입력
        2. 짧은 caption 생성
        3. caption 안에서 응급 키워드 탐지
        4. 같은 응급상황이 연속으로 감지되면 최종 응급으로 판단
    """

    EMERGENCY_KEYWORDS = { # caption 안에서 응급상황을 나타내는 키워드와 대응되는 응급 타입
        "fallen": "fall",
        "fell": "fall",
        "falling": "fall",
        "collapse": "fall",
        "collapsed": "fall",
        "slipped": "fall",

        "fire": "fire",
        "flames": "fire",
        "burning": "fire",
        "explosion": "fire",

        "smoke": "smoke",
        "fumes": "smoke",

        "blood": "bleeding",
        "bleeding": "bleeding",
        "wound": "bleeding",
        "injury": "bleeding",

        "unconscious": "unconscious",
        "passed out": "unconscious",
        "fainted": "unconscious",

        "seizure": "seizure",
        "convulsion": "seizure",

        "choking": "choking",
        "not breathing": "choking",
        "difficulty breathing": "choking",

        "heart attack": "cardiac_arrest",
        "cardiac arrest": "cardiac_arrest",
        "stroke": "stroke",

        "scream": "scream",
        "cry for help": "scream",
    }

    def __init__(
            self,
            model_id: str = "Qwen/Qwen2-VL-2B-Instruct",
            cache_dir: str = None,
            min_pixels: int = 256 * 28 * 28,
            max_pixels: int = 1280 * 28 * 28,
            consecutive_threshold: int = 3, # 같은 응급상황이 연속으로 감지되어야 최종 응급으로 판단 (예: 3이면 3연속 감지 시 최종 응급 판단)
    ):

        self._model_id = model_id

        if cache_dir is None:
            cache_dir = os.path.expanduser("~/.cache/huggingface")

        self._cache_dir = cache_dir
        self._min_pixels = min_pixels
        self._max_pixels = max_pixels

        self._consecutive_threshold = consecutive_threshold
        self._emergency_count = 0 # 같은 응급상황이 연속으로 감지된 횟수 카운터
        self._last_emergency_type = None

        self._model = None
        self._processor = None # VLM 입력 전처리 및 caption 후처리를 위한 processor 객체 초기화

        self.load_model()

    # ==============================================================================================================
    # Load Model
    # ==============================================================================================================

    def load_model(self):
        """
        Qwen2-VL 모델 로드
        """

        write_log("Loading emergency VLM model...", self)

        bnb_config = BitsAndBytesConfig( # 4-bit 양자화 설정, VRAM 사용량 줄임
            load_in_4bit=True,
            bnb_4bit_use_double_quant=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16,
        )

        self._model = Qwen2VLForConditionalGeneration.from_pretrained( # 모델 로드, 양자화 설정 적용, 자동으로 GPU 사용
            self._model_id,
            quantization_config=bnb_config,
            device_map="auto",
            cache_dir=self._cache_dir,
            offload_folder="offload_vlm", # VRAM 부족 시 CPU로 자동 오프로드할 때 사용할 폴더 (자동으로 생성됨)
        )

        self._processor = AutoProcessor.from_pretrained( # 모델에 맞는 processor 로드, 이미지 전처리 및 caption 후처리에 사용
            self._model_id,
            cache_dir=self._cache_dir,
            min_pixels=self._min_pixels,
            max_pixels=self._max_pixels,
        )

        write_log("Emergency VLM model loaded.", self)

    # ==============================================================================================================
    # Image Convert
    # ==============================================================================================================

    @staticmethod
    def bgr_to_pil(frame_bgr): # OpenCV BGR 이미지를 PIL RGB 이미지로 변환하는 함수
        """
        OpenCV BGR 이미지를 PIL RGB 이미지로 변환
        """

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb)

    # ==============================================================================================================
    # Caption
    # ==============================================================================================================

    def generate_caption(self, frame_bgr) -> str: # VLM으로 현재 frame에 대한 짧은 caption 생성하는 함수
        """
        VLM으로 현재 frame에 대한 짧은 caption 생성
        """

        if frame_bgr is None:
            return ""

        if frame_bgr.size == 0:
            return ""

        pil_img = self.bgr_to_pil(frame_bgr)

        messages = [ # VLM에 입력할 메시지 템플릿, 시스템 메시지로 영어로 답변하도록 지시, 사용자 메시지로 이미지와 caption 생성 지침 포함
            {
                "role": "system", # 모델 행동 규칙
                "content": [
                    {
                        "type": "text",
                        "text": "Always answer in English only.",
                    }
                ],
            },
            {
                "role": "user", # 실제 사용자 입력, 이미지와 caption 생성 지침 포함
                "content": [
                    {
                        "type": "image",
                        "image": pil_img,
                    },
                    {
                        "type": "text",
                        "text":
                            "- Describe in a COMPLETE noun phrase within 10 tokens. "
                            "- Describe exactly what is visible in the image. "
                            "- If the image contains a person or any part of a person, "
                            "describe only that person or body part, and nothing else. "
                            "- If a person is lying on the floor or ground, output exactly 'fallen'. "
                            "- If a person is lying but not on the floor or ground, output exactly 'lying'.",
                    },
                ],
            },
        ]

        text = self._processor.apply_chat_template( # VLM 입력 메시지 템플릿 적용, 이미지와 텍스트를 모델 입력 형식에 맞게 변환
            messages,
            tokenize=False,
            add_generation_prompt=True,
        )

        image_inputs, video_inputs = process_vision_info(messages) # 이미지와 비디오 입력을 모델에 맞게 전처리하는 함수, 여기서는 이미지 입력만 사용하므로 video_inputs는 빈 리스트가 됨

        inputs = self._processor( # 모델 입력 전처리, 텍스트와 이미지 입력을 모델에 맞는 텐서로 변환
            text=[text], # 텍스트 입력은 리스트 형태로 감싸서 전달 (배치 입력 형식)
            images=image_inputs,
            videos=video_inputs,
            return_tensors="pt",
        ).to(self._model.device)

        with torch.inference_mode(): # 추론스 모드로 모델 실행, 그래디언트 계산 비활성화 (메모리 절약 및 속도 향상)
            output = self._model.generate(
                **inputs,
                max_new_tokens=10,
            )

            generated = output[:, inputs.input_ids.shape[1]:] # output에는 입력 프롬프트 + 새로 생성한 답변이 모두 포함되어 있으므로, inputs.input_ids.shape[1]: 이후의 토큰이 새로 생성된 답변에 해당

            caption = self._processor.batch_decode( # 모델 출력 토큰을 텍스트로 디코딩, skip_special_tokens=True로 특수 토큰 제거, 결과는 리스트 형태이므로 [0]으로 첫 번째 요소(생성된 caption) 추출, strip()으로 양쪽 공백 제거
                generated,
                skip_special_tokens=True,
            )[0].strip()

        return caption

    # ==============================================================================================================
    # Keyword Detection
    # ==============================================================================================================

    def detect_emergency_type(self, caption: str):
        """
        caption 안에서 응급 키워드를 찾아 응급 타입 반환
        """

        if caption is None:
            return None

        caption_lower = caption.lower()

        for keyword, emergency_type in self.EMERGENCY_KEYWORDS.items():
            if keyword.lower() in caption_lower:
                return emergency_type

        return None

    # ==============================================================================================================
    # Detect
    # ==============================================================================================================

    def detect(self, frame_bgr):
        """
        frame 1장에 대해 응급상황 판단

        반환:
            {
                "is_emergency": True/False,
                "emergency_type": "fall" or None,
                "confidence": 0.95,
                "caption": caption
            }
        """

        caption = self.generate_caption(frame_bgr)
        emergency_type = self.detect_emergency_type(caption)

        if emergency_type is None:
            self._emergency_count = 0
            self._last_emergency_type = None

            return {
                "is_emergency": False,
                "emergency_type": None,
                "confidence": 0.0,
                "caption": caption,
            }

        if emergency_type == self._last_emergency_type:
            self._emergency_count += 1
        else:
            self._emergency_count = 1
            self._last_emergency_type = emergency_type

        is_emergency = self._emergency_count >= self._consecutive_threshold

        return {
            "is_emergency": is_emergency,
            "emergency_type": emergency_type,
            "confidence": 0.95 if is_emergency else 0.5,
            "caption": caption,
        }

    # ==============================================================================================================
    # Reset
    # ==============================================================================================================

    def reset(self): # 응급상황 감지 상태 초기화
        """
        연속 감지 카운터 초기화
        """

        self._emergency_count = 0
        self._last_emergency_type = None