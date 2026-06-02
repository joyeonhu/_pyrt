# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : GownClassifier.py
# Project Name : HealthcareRobotPyRT
# Description  : Patient gown classifier using MobileNetV2
# -------------------------------------------------------------------------------------------------------------------- #

import os

import cv2
import torch
import numpy as np

from torchvision import models, transforms

from Commons import *


class CGownClassifier:
    """
    환자복 분류기

    입력:
        - 사람 상체 crop 이미지(BGR, OpenCV format)

    출력:
        - True  : 환자복(gown)
        - False : 일반복(normal)
    """

    def __init__(
            self,
            model_path: str = None,
            device: str = "cpu",
    ):

        if model_path is None: # 경로 지정 안했으면 프로젝트 루트에서 Data/Models/gown_classifier.pth 로드
            root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
            model_path = os.path.join(root_path, "Data", "Models", "gown_classifier.pth")

        self._model_path = model_path
        self._device = device

        self._transform = transforms.Compose([ # 이미지 전처리 파이프라인, MobileNetV2 모델이 요구하는 입력 형식에 맞게 변환
            transforms.ToPILImage(), # OpenCV numpy 이미지 -> PIL 이미지
            transforms.Resize((224, 224)), # MobileNetV2 입력 크기 (224x224)
            transforms.ToTensor(), # PIL 이미지 -> PyTorch 텐서 (C x H x W, [0, 1])
            transforms.Normalize( # MobileNetV2 사전학습된 모델이 사용한 정규화 값으로 정규화
                [0.485, 0.456, 0.406],
                [0.229, 0.224, 0.225]
            )
        ])

        self._model = None # 모델 객체 초기화
        self.load_model() # 모델 로드

    # ==============================================================================================================
    # Load Model
    # ==============================================================================================================

    def load_model(self): # 모델 로드
        """
        MobileNetV2 기반 환자복 분류 모델 로드
        """

        if not os.path.exists(self._model_path): # 파일 없으면 에러
            raise FileNotFoundError(
                "Gown classifier model not found: %s" % self._model_path
            )

        model = models.mobilenet_v2(pretrained=False) # MobileNetV2 모델 객체 생성, pretrained=False : ImageNet 사전학습된 가중치 사용 안함, 우리가 직접 학습한 모델이므로 pretrained=False로 설정
        model.classifier[1] = torch.nn.Linear(model.last_channel, 2) # MobileNetV2의 마지막 분류 레이어를 2 클래스 (gown vs normal)로 변경
        # .classifier[1] : MobileNetV2 모델의 마지막 분류 레이어 (nn.Linear), model.last_channel : MobileNetV2의 마지막 특징 채널 수 (1280), 2 : 클래스 수 (gown vs normal)

        model.load_state_dict( # 저장된 모델 가중치 로드
            torch.load(
                self._model_path, # gown_classifier.pth
                map_location=self._device # 모델이 저장된 디바이스와 현재 디바이스가 다를 수 있으므로, map_location을 사용하여 모델 가중치를 현재 디바이스로 로드
            )
        )
        # .load_state_dict() : 모델 객체에 저장된 가중치를 로드하는 메서드, 예전에 학습해놓은 지식을 넣는 과정
        # torch.load() : 저장된 모델 가중치를 로드하는 함수

        model.to(self._device) # 모델을 지정된 디바이스 (CPU 또는 GPU)로 이동
        model.eval() # 모델을 평가 모드로 설정

        self._model = model # 모델 객체 저장

        write_log("Gown classifier loaded: %s" % self._model_path, self)

    # ==============================================================================================================
    # Predict
    # ==============================================================================================================

    def predict(self, crop_bgr: np.ndarray) -> int: # 클래스 인덱스 반환
        """
        crop 이미지의 클래스 index 반환

        반환:
            0 = gown
            1 = normal
        """
        # crop_bgr : 잘라낸(crop) 사람 이미지

        if crop_bgr is None: # 이미지 없으면 normal
            return 1

        if crop_bgr.size == 0: # 빈 이미지면 normal
            return 1

        img_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB) # OpenCV는 BGR 형식이므로 RGB로 변환

        tensor = self._transform(img_rgb) # 이미지 전처리 파이프라인을 통해 텐서로 변환 (C x H x W)
        tensor = tensor.unsqueeze(0) # 배치 차원 추가 (1 x C x H x W), 모델 입력 형식에 맞게 변환
        tensor = tensor.to(self._device) # 텐서를 지정된 디바이스 (CPU 또는 GPU)로 이동

        with torch.no_grad(): # 모델 예측 시 그래디언트 계산 비활성화 (메모리 절약 및 속도 향상)
            output = self._model(tensor) # 모델에 입력 텐서를 넣어 예측 결과 얻기 (1 x 2 텐서, 클래스별 점수)
            pred = torch.argmax(output, dim=1).item() # 예측 결과에서 가장 높은 점수를 가진 클래스 인덱스 추출 (0 또는 1)

        return pred # 0이면 환자복(gown), 1이면 일반복(normal)으로 분류

    # ==============================================================================================================
    # Is Gown
    # ==============================================================================================================

    def is_gown(self, crop_bgr: np.ndarray) -> bool: # 환자복 여부 반환
        """
        환자복 여부 반환
        """

        pred = self.predict(crop_bgr) # 클래스 인덱스 예측

        return pred == 0 # 클래스 인덱스가 0이면 환자복(gown)으로 분류, 1이면 일반복(normal)으로 분류, 따라서 pred == 0이 True이면 환자복, False이면 일반복

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_model_path(self): # 모델 경로 반환
        return self._model_path