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
            model_id: str = "Qwen/Qwen2-VL-2B-Instruct", # 사용할 Qwen2-VL 모델 ID
            cache_dir: str = None, # 모델 캐시 디렉토리, None이면 기본 Hugging Face 캐시 디렉토리 사용
            min_pixels: int = 256 * 28 * 28, # 이미지 크기가 너무 작으면 VLM이 제대로 caption을 생성하지 못할 수 있으므로, 최소 픽셀 수를 설정하여 작은 이미지는 확대해서 입력 (예: 256*28*28은 28x28 크기의 이미지보다 작은 경우 확대해서 입력)
            max_pixels: int = 1280 * 28 * 28, # 이미지 크기가 너무 크면 VRAM 부족으로 모델이 실행되지 않을 수 있으므로, 최대 픽셀 수를 설정하여 큰 이미지는 축소해서 입력 (예: 1280*28*28은 1280x1280 크기의 이미지보다 큰 경우 축소해서 입력)
            consecutive_threshold: int = 3, # 같은 응급상황이 연속으로 감지되어야 최종 응급으로 판단 (예: 3이면 3연속 감지 시 최종 응급 판단)
    ):

        self._model_id = model_id

        if cache_dir is None:
            cache_dir = os.path.expanduser("~/.cache/huggingface")

        self._cache_dir = cache_dir
        self._min_pixels = min_pixels
        self._max_pixels = max_pixels

        self._consecutive_threshold = consecutive_threshold
        self._emergency_count = 0 # 현재까지 연속으로 감지된 같은 응급상황의 횟수, consecutive_threshold에 도달하면 최종 응급으로 판단
        self._last_emergency_type = None # 마지막으로 감지된 응급상황 타입, consecutive_threshold에 도달하기 전에 다른 응급상황이 감지되면 emergency_count를 초기화하는 데 사용

        self._model = None # Qwen2-VL 모델 객체
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

        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_use_double_quant=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16,
        )
        # BitsAndBytesConfig : 모델을 얼마나 압축해서 GPU에 올릴 건지 정하는 클래스
        # load_in_4bit=True : 모델을 4-bit 양자화해서 로드하겠다는 의미, model은 보통 16-bit 또는 32-bit로 로드되는데, 4-bit으로 양자화하면 모델 크기가 대폭 줄어들어서 VRAM 부족 문제를 완화할 수 있음
        # bnb_4bit_use_double_quant=True : 압축한 데이터를 한 번 더 압축, 계산이 아주 약간 느려질 수 있음
        # bnb_4bit_quant_type="nf4" : 어떤 방식으로 4bit 압축할 거냐, nf4는 Normal Float 4의 약자로, 4bit로 표현할 때 숫자의 범위를 더 넓게 표현할 수 있도록 하는 방식
        # bnb_4bit_compute_dtype=torch.float16 : 모델 계산에 사용할 데이터 타입, 저장할 때는 4bit로 압축하지만, 계산할 때는 float16으로 변환해서 계산하겠다는 의미, 이렇게 하면 양자화로 인한 성능 저하를 줄일 수 있음

        self._model = Qwen2VLForConditionalGeneration.from_pretrained(
            self._model_id,
            quantization_config=bnb_config,
            device_map="auto",
            cache_dir=self._cache_dir,
            offload_folder="offload_vlm",
        )
        # Qwen2VLForConditionalGeneration : Qwen2-VL 모델 클래스, from_pretrained() 메서드로 사전 학습된 모델 로드
        # self._model_id : 사용할 모델 ID
        # quantization_config=bnb_config : 모델을 어떻게 양자화해서 로드할지 설정한 BitsAndBytesConfig 객체 전달
        # device_map="auto" : 모델의 각 레이어를 자동으로 CPU와 GPU에 분배해서 로드, VRAM이 부족한 경우 일부 레이어는 CPU에 올려서 로드할 수 있음
        # cache_dir=self._cache_dir : 다운로드한 모델 저장 위치, 처음 -> 모델 다운로드, 이후 -> 캐시에서 모델 로드
        # offload_folder="offload_vlm" : GPU 메모리 부족하면 모델 일부를 CPU RAM이나 SSD로 내리는데 그때 저장 위치

        self._processor = AutoProcessor.from_pretrained(
            self._model_id,
            cache_dir=self._cache_dir,
            min_pixels=self._min_pixels,
            max_pixels=self._max_pixels,
        )
        # AutoProcessor : 모델 입력 전처리와 출력 후처리를 담당하는 클래스, 사진을 모델이 이해하는 숫자로 변환, from_pretrained() 메서드로 사전 학습된 프로세서 로드
        # self._model_id : 사용할 모델 ID, 이 모델에 맞는 프로세서 가져와라
        # cache_dir=self._cache_dir : 다운로드한 프로세서 저장 위치, 처음 -> 프로세서 다운로드, 이후 -> 캐시에서 프로세서 로드
        # min_pixels=self._min_pixels, max_pixels=self._max_pixels : 프로세서가 입력 이미지 크기를 조정할 때 사용할 최소/최대 픽셀 수, 이미지 크기가 너무 작으면 확대, 너무 크면 축소해서 모델에 입력하도록 설정

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

        # messages에서 text 부분만 추출해서 문자열 생성
        text = self._processor.apply_chat_template( # VLM 입력 메시지 템플릿 적용, 이미지와 텍스트를 모델 입력 형식에 맞게 변환
            messages,
            tokenize=False,
            add_generation_prompt=True,
        )
        # apply_chat_template() : 모델 전용 프롬프트로 변환
        # messages : 모델에 입력할 메시지 템플릿
        # tokenize=False : 텍스트를 토큰으로 변환하지 않고 문자열 그대로 반환
        # add_generation_prompt=True : 모델이 답변을 생성할 때 사용할 프롬프트를 자동으로 추가, 모델이 답변을 생성할 때 "Answer:" 같은 프롬프트가 자동으로 붙어서 답변 생성 시작 지점 명확하게 만들어줌

        # messages에서 이미지/비디오 부분만 추출
        image_inputs, video_inputs = process_vision_info(messages) # 이미지와 비디오 입력을 모델에 맞게 전처리하는 함수, 여기서는 이미지 입력만 사용하므로 video_inputs는 빈 리스트가 됨

        inputs = self._processor( # 모델 입력 전처리, 텍스트와 이미지 입력을 모델에 맞는 텐서로 변환
            text=[text], # 텍스트 입력은 리스트 형태로 감싸서 전달 (배치 입력 형식)
            images=image_inputs,
            videos=video_inputs,
            return_tensors="pt", # PyTorch 텐서로 반환
        ).to(self._model.device) # 모델이 로드된 디바이스(GPU 또는 CPU)로 텐서 이동

        with torch.inference_mode(): # 추론 모드로 모델 실행, 그래디언트 계산 비활성화 (메모리 절약 및 속도 향상)
            output = self._model.generate( # 모델에 입력을 주고 답변 생성, max_new_tokens=10으로 최대 10개의 토큰까지만 생성하도록 설정
                **inputs,
                max_new_tokens=10,
            )

            generated = output[:, inputs.input_ids.shape[1]:] # output에는 입력 프롬프트 + 새로 생성한 답변이 모두 포함되어 있으므로, inputs.input_ids.shape[1]: 이후의 토큰이 새로 생성된 답변에 해당
            # output = [a, b, c, d, e, f, g, h], 입력 프롬프트 + 생성된 답변
            # [:, inputs.input_ids.shape[1]:] : 모든 배치에 대해서, 입력 프롬프트 길이 이후의 토큰을 선택, 즉 새로 생성된 답변 부분만 선택
            # inputs.input_ids.shape[1] = 입력 프롬프트 토큰 개수

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