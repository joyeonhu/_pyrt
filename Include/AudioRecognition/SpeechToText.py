# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : SpeechToText.py
# Project Name : HealthcareRobotPyRT
# Description  : Speech-to-text module
# -------------------------------------------------------------------------------------------------------------------- #

import os
import subprocess

from Commons import *

try:
    import speech_recognition as sr
except ImportError:
    sr = None


class CSpeechToText:
    """
    STT 모듈

    역할:
        1. arecord로 음성 녹음
        2. SpeechRecognition으로 음성 인식
        3. 텍스트 반환
    """

    def __init__(
            self,
            language: str = "en-US",
            sample_rate: int = 44100, # 녹음 샘플링 레이트, 16000이나 44100으로 설정하는 경우가 많음, 너무 낮으면 음질이 나빠지고 인식률이 떨어질 수 있음, 너무 높으면 파일 크기가 커지고 처리 시간이 길어질 수 있음
            duration_sec: int = 5,
            temp_file: str = "_temp_stt.wav",
    ):

        self._language = language
        self._sample_rate = sample_rate
        self._duration_sec = duration_sec
        self._temp_file = temp_file

        self._recognizer = None # STT 인식기 객체

        self.initialize()

    # ==============================================================================================================
    # Initialize
    # ==============================================================================================================

    def initialize(self):
        """
        SpeechRecognition 초기화
        """

        if sr is None:
            raise ImportError(
                "speech_recognition 라이브러리를 찾을 수 없습니다."
            )

        self._recognizer = sr.Recognizer()

        write_log("SpeechToText initialized.", self)

    # ==============================================================================================================
    # Record
    # ==============================================================================================================

    def record(self):
        """
        arecord로 음성 녹음
        """

        arecord_cmd = [
            "arecord", # 리눅스 녹음 명령어
            "-D", "pulse", # PulseAudio를 사용하여 녹음
            "-f", "S16_LE", # 녹음 포맷, 16-bit little-endian PCM
            "-r", str(self._sample_rate), # 샘플링 레이트
            "-d", str(self._duration_sec), # 녹음 시간
            self._temp_file # 저장할 파일 경로
        ]

        try:
            subprocess.run( # 파이썬에서 외부 명령어를 실행하는 함수
                arecord_cmd,
                stdout=subprocess.DEVNULL, # 명령어의 일반 출력은 화면에서 안 보이게 함
                stderr=subprocess.DEVNULL, # 명령어의 에러 출력도 화면에서 안 보이게 함
                check=True, # 명령어 실행 실패 시 예외를 발생시키도록 설정
            )

            return True

        except Exception:
            ErrorHandler().report()
            return False

    # ==============================================================================================================
    # Recognize
    # ==============================================================================================================

    def recognize(self):
        """
        녹음된 wav 파일을 STT 수행
        """

        if not os.path.exists(self._temp_file):
            return None

        try:
            with sr.AudioFile(self._temp_file) as source: # 녹음된 wav 파일을 SpeechRecognition이 읽을 수 있는 형식으로 열기
                audio_data = self._recognizer.record(source) # wav 파일 전체를 읽어서 음성 데이터로 저장, self._recognizer.record() 함수는 AudioFile 객체에서 음성 데이터를 읽어서 AudioData 객체로 반환하는 함수

            text = self._recognizer.recognize_google( # Google STT를 사용해 음성을 텍스트로 변환
                audio_data,
                language=self._language
            )

            text = text.lower().strip() # 인식된 텍스트를 소문자로 변환하고 양쪽 공백 제거

            write_log(
                "STT Result: %s" % text,
                self
            )

            return text

        except Exception:
            return None

    # ==============================================================================================================
    # Listen
    # ==============================================================================================================

    def listen(self):
        """
        녹음 + STT 수행
        """

        success = self.record()

        if not success:
            return None

        text = self.recognize()

        return text

    # ==============================================================================================================
    # Cleanup
    # ==============================================================================================================

    def cleanup(self): # 임시 wav 파일 삭제
        """
        임시 wav 파일 삭제
        """

        if os.path.exists(self._temp_file):
            try:
                os.remove(self._temp_file)
            except Exception:
                pass

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def get_language(self): # 현재 설정된 언어 반환 함수
        return self._language

    def get_duration(self): # 현재 설정된 녹음 시간 반환 함수
        return self._duration_sec