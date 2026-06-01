# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : LLMManager.py
# Project Name : HealthcareRobotPyRT
# Description  : LLM-based command manager
# -------------------------------------------------------------------------------------------------------------------- #

import json
import re

import torch
from transformers import AutoTokenizer, AutoModelForCausalLM

from Commons import *


class CLLMManager:
    """
    LLM 명령 관리자

    역할:
        1. STT 텍스트를 LLM에 입력
        2. LLM이 로봇 명령 JSON 생성
        3. ControlCore가 사용할 command dict 반환
    """
    # 명령 종류
    CMD_NONE = "NONE"
    CMD_FOLLOW = "FOLLOW"
    CMD_GUIDE = "GUIDE"
    CMD_DELIVERY = "DELIVERY"
    CMD_HOMING = "HOMING"

    # 허용되는 명령 목록
    VALID_COMMANDS = {
        CMD_NONE,
        CMD_FOLLOW,
        CMD_GUIDE,
        CMD_DELIVERY,
        CMD_HOMING,
    }

    # 허용되는 목적지 ID 목록, LLM이 목적지를 만들더라도 이 안에 있는 것만 유효한 것으로 인정
    VALID_DESTINATIONS = {
        "convenience_store",
        "nurse_station",
        "room_19421",
        "home",
    }

    def __init__(
            self,
            model_id: str = "Qwen/Qwen2.5-1.5B-Instruct", # LLM 모델 ID, Hugging Face 모델 허브에 있는 모델 ID를 입력하면 됨
            device: str = None, # "cuda" 또는 "cpu"
            max_new_tokens: int = 128, # LLM이 생성할 최대 토큰 수
    ):

        self._model_id = model_id # LLM 모델 ID 설정
        self._max_new_tokens = max_new_tokens # LLM이 생성할 최대 토큰 수 설정

        if device is None: # device가 명시되지 않은 경우
            if is_cuda_available(): # CUDA 사용 가능하면
                device = "cuda" # CUDA 사용
            else: # CUDA 사용 불가능하면
                device = "cpu" # CPU 사용

        self._device = device # 사용할 디바이스 설정

        self._tokenizer = None # LLM 토크나이저 객체, 모델과 함께 로드됨
        self._model = None # LLM 모델 객체, 모델과 함께 로드됨

        self._last_input_text = None # 마지막 입력 ex) 배고파
        self._last_response_text = None # 마지막 LLM 응답 ex) {"command": "GUIDE", "destination_id": "convenience_store"}
        self._last_command = None # 마지막 명령 dict, 최종 검증이 끝난 command dict ex) {"command": "GUIDE", "destination_id": "convenience_store"}

        self.load_model() # 모델 로드

    # ==============================================================================================================
    # Load Model
    # ==============================================================================================================

    def load_model(self):
        """
        LLM 모델 로드
        """

        write_log("Loading LLM model: %s" % self._model_id, self)

        self._tokenizer = AutoTokenizer.from_pretrained( # Hugging Face 모델 허브에서 토크나이저 로드, 모델 ID는 self._model_id
            self._model_id
        )
        # 토큰 : 문장을 읽는 최소 단위
        # 토크나이저 : 문장을 토큰으로 나누는 도구
        # hugging face : AI 모델 깃허브 같은 사이트, 다양한 AI 모델이 등록되어 있고, 모델 ID를 통해 원하는 모델과 토크나이저를 쉽게 불러올 수 있음
        # AutoTokenizer : 모델에 맞는 토크나이저를 자동으로 만들어주는 클래스
        # from_pretrained() 메서드 : H사전 학습된 토크나이저를 로드하는 함수
        # 최종적으로 토크나이저 객체를 반환

        self._model = AutoModelForCausalLM.from_pretrained( # Hugging Face 모델 허브에서 LLM 모델 로드
            self._model_id,
            torch_dtype=torch.float16 if self._device == "cuda" else torch.float32,
            device_map="auto" if self._device == "cuda" else None,
        )
        # AutoModelForCausalLM : 사전 학습된 언어 모델을 불러오는 클래스
        # from_pretrained() 메서드 : 사전 학습된 모델을 로드하는 함수
        # torch_dtype : 몇 비트로 모델을 로드할지 설정, CUDA 사용 시 메모리 절약을 위해 float16으로 로드, CPU 사용 시 float32로 로드
        # device_map : 모델을 어느 디바이스에 로드할지 설정, device가 cuda면 auto : 가능한 GPU에 모델을 자동 배치, 아니면 자동 배치 X, 직접 지정 필요

        if self._device == "cpu": # CPU로 모델을 로드한 경우
            self._model.to("cpu") # 모델을 CPU로 이동, CPU에서 모델이 실행되도록 보장

        self._model.eval() # 추론 모드로 설정

        write_log("LLM model loaded.", self)

    # ==============================================================================================================
    # Prompt
    # ==============================================================================================================

    def build_prompt(self, text: str) -> str:
        """
        로봇 명령 해석용 prompt 생성
        """

        prompt = f"""
You are an intent parser for a hospital healthcare robot. 

The robot has four main functions:
1. FOLLOW: follow the patient.
2. GUIDE: guide the patient to a destination.
3. DELIVERY: deliver an item to a destination.
4. HOMING: return to home position.

You must output ONLY one JSON object.
Do not explain.
Do not include markdown.

Valid commands:
- FOLLOW
- GUIDE
- DELIVERY
- HOMING
- NONE

Valid destination_id values:
- convenience_store
- nurse_station
- room_19421
- home

Rules:
1. Infer the user's intent from the utterance and choose exactly one command only from the valid commands.
2. Infer the user's intent from the utterance and choose destination_id only from the valid destination_id values.
3. For FOLLOW and NONE commands, destination_id must be null.
4. For HOMING command, destination_id must be home.
5. If the utterance is unclear or unrelated to the robot functions, choose NONE.

Output JSON format:
{
    "command": "<VALID_COMMAND>",
    "destination_id": "<VALID_DESTINATION_ID or null>"
}

User utterance:
{text}

Output JSON:
"""
        return prompt.strip() # 생성된 prompt에서 양쪽 공백 제거 후 반환

    # ==============================================================================================================
    # Generate
    # ==============================================================================================================

    def generate_response(self, prompt: str) -> str: # LLM에 prompt를 입력하여 응답을 생성하는 함수
        """
        LLM 응답 생성
        """

        messages = [ # LLM이 이해할 수 있는 메시지 형식으로 prompt를 감싸기, 시스템 메시지로 역할과 규칙을 전달하고, 사용자 메시지로 실제 입력을 전달
            {
                "role": "user",
                "content": prompt,
            }
        ]

        input_text = self._tokenizer.apply_chat_template( # Qwen 스타일 문자열로 메시지 변환
            messages,
            tokenize=False,
            add_generation_prompt=True
        )
        # apply_chat_template() : 메시지를 LLM이 이해할 수 있는 입력 형식으로 변환하는 함수
        # toknize : 메시지를 토큰으로 변환할지 여부, False로 설정하여 문자열 그대로 사용
        # ex) False : "<|user|>\n배고파\n<|assistant|>", True : [151644, 872, 198, ...]
        # add_generation_prompt : LLM이 응답을 생성할 때 필요한 프롬프트를 추가할지 여부, True로 설정하여 LLM이 응답을 생성할 때 필요한 프롬프트를 자동으로 추가

        inputs = self._tokenizer( # 근데 어차피 모델은 문자열 자체를 못 읽고 토큰으로 읽으니까, 토크나이저로 토큰화해서 모델 입력 형식으로 변환
            input_text,
            return_tensors="pt" #
        ).to(self._model.device)

        # Qwen 스타일 문자열로 바꾸면서 토큰화안하고 문자열로 바꾼뒤에 토큰화따로 하는 이유
        # 중간 문자열을 눈으로 확인 가능

        with torch.no_grad(): # gradient 계산 비활성화
            outputs = self._model.generate( # LLM 응답 생성
                **inputs, # 입력
                max_new_tokens=self._max_new_tokens, # LLM이 생성할 최대 토큰 수
                do_sample=False, # 생성 과정에서 랜덤으로 뽑지 않고, 가장 가능성 높은 토큰을 선택하여 응답 생성
                temperature=0.0, # 랜덤성 조절 옵션, 0.0으로 설정하여 가장 가능성 높은 토큰을 항상 선택
            )

        generated = outputs[:, inputs.input_ids.shape[-1]:] # 입력 프롬프트 부분을 제회하고 새로 생성된 답변 부분만 잘라냄
        # ex) 입력 prompt 토큰 = [10, 20, 30, 40], outputs = [10, 20, 30, 40, 50, 60], 50, 60이 모델이 새로 만든 답변
        # inputs.input_ids.shape[-1] : 입력 토큰 개수

        response = self._tokenizer.decode( # 토큰을 다시 문자열로 변환
            generated[0],
            skip_special_tokens=True # 특수 토큰 제거 옵션, True로 설정하여 <|user|>, <|assistant|> 같은 특수 토큰을 응답에서 제거
        ).strip() # 응답 문자열 양쪽 공백 제거

        return response # 최종적으로 LLM이 생성한 응답 문자열 반환

    # ==============================================================================================================
    # Parse JSON
    # ==============================================================================================================

    @staticmethod
    def extract_json(text: str):
        """
        LLM 응답에서 JSON object만 추출
        """

        if text is None: # 입력이 None이면
            return None # None 반환

        text = text.strip() # 입력 문자열 양쪽 공백 제거

        try:
            return json.loads(text)
        except Exception:
            pass
        # json.loads() : json 형식의 문자열을 파이썬 dict로 바꾸는 함수

        match = re.search(r"\{.*\}", text, re.DOTALL) #
        # re.search() : 문자열에서 패턴을 검색하는 함수
        # {~~~} <- 이렇게 생긴 부분 찾으라는 뜻, .* : 아무 문자나 몇 개가 나오든 상관없음, . : 아무 문자 1개, * : 앞에 있는 걸 0개 이상 반복
        # re.DOTALL : .은 줄바꿈을 못 잡음, 이 옵션을 주면 .가 줄바꿈까지 포함해서 다 찾음

        if match is None:
            return None

        try:
            return json.loads(match.group(0)) # .group(0) : 찾은 문자열 반환
        except Exception:
            return None

    # ==============================================================================================================
    # Validate
    # ==============================================================================================================

    def validate_command(self, command_dict: dict):
        """
        LLM 결과를 우리 시스템 명령 형식으로 검증/보정
        """

        if not isinstance(command_dict, dict):
            return self.make_none_command()

        command = command_dict.get("command", self.CMD_NONE) # command 값 꺼내서 없으면 CMD_NONE으로 설정

        if command not in self.VALID_COMMANDS: # command가 VALID_COMMANDS에 없으면
            command = self.CMD_NONE # command를 CMD_NONE으로 설정

        destination_id = command_dict.get("destination_id", None) # destination_id 값 꺼내서 없으면 None으로 설정

        if command in {self.CMD_GUIDE, self.CMD_DELIVERY}: # command가 GUIDE 또는 DELIVERY인데
            if destination_id not in self.VALID_DESTINATIONS: # destination_id가 VALID_DESTINATIONS에 없으면
                return self.make_none_command() # 명령이 GUIDE 또는 DELIVERY인데 destination_id가 유효하지 않으면, 명령 자체가 이상한 거니까 CMD_NONE으로 만들어서 반환
        else:
            destination_id = None # command가 GUIDE 또는 DELIVERY가 아니면, destination_id는 None으로 설정 (예: FOLLOW, HOMING, NONE은 destination_id 필요 없음)

        result = {
            "command": command,
            "destination_id": destination_id,
        }

        self._last_command = result

        return result # 검증/보정된 명령 dict 반환

    # ==============================================================================================================
    # Parse Text
    # ==============================================================================================================

    def parse_text( # 이 파일의 메인 함수
            self,
            text: str
    ):
        """
        STT 텍스트를 LLM으로 해석하여 command dict 반환
        """

        if text is None:
            return self.make_none_command()

        text = str(text).strip()

        if len(text) == 0:
            return self.make_none_command()

        self._last_input_text = text # 마지막 입력 텍스트 저장

        prompt = self.build_prompt(text) # LLM 프롬프트 생성

        response = self.generate_response(prompt) # LLM에 프롬프트 입력하여 응답 생성

        self._last_response_text = response # 마지막 LLM 응답 텍스트 저장

        command_dict = self.extract_json(response) # LLM 응답에서 JSON object 추출, 명령이 제대로 생성되었는지 확인

        command = self.validate_command(command_dict) # LLM 결과를 우리 시스템 명령 형식으로 검증/보정, 명령이 유효한지 확인하고, 필요하면 보정하여 최종 명령 dict 생성

        write_log(
            "LLM parsed | input=%s | response=%s | command=%s"
            % (
                text,
                response,
                str(command)
            ),
            self
        )

        return command # 최종 명령 dict 반환

    # ==============================================================================================================
    # Command Factory
    # ==============================================================================================================

    def make_none_command(self): # 인식 실패/ 무효 명령일 때 쓸 기본 명령 생성
        cmd = {
            "command": self.CMD_NONE,
            "destination_id": None,
        }

        self._last_command = cmd

        return cmd

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_last_input_text(self): # 마지막 입력 텍스트 반환
        return self._last_input_text

    def get_last_response_text(self): # 마지막 LLM 응답 텍스트 반환
        return self._last_response_text

    def get_last_command(self): # 마지막 명령 dict, 최종 검증이 끝난 command dict 반환
        return self._last_command