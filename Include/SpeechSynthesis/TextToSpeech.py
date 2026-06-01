# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : TextToSpeech.py
# Project Name : HealthcareRobotPyRT
# Description  : Text-to-speech module
# -------------------------------------------------------------------------------------------------------------------- #

import os
import time

from gtts import gTTS
import pygame

from Commons import *


class CTextToSpeech:
    """
    TTS 모듈

    역할:
        1. 텍스트를 음성(mp3)으로 변환
        2. pygame으로 재생
    """

    def __init__(
            self,
            language: str = "en",
            temp_file: str = "_temp_tts.mp3",
    ):

        self._language = language
        self._temp_file = temp_file

        self._is_initialized = False # 초기화 여부
        self._is_speaking = False # 현재 음성 재생 중인지 여부

        self.initialize()

    # ==============================================================================================================
    # Initialize
    # ==============================================================================================================

    def initialize(self):
        """
        pygame mixer 초기화
        """

        try:
            pygame.mixer.init()

            self._is_initialized = True

            write_log("TextToSpeech initialized.", self)

        except Exception:
            ErrorHandler().report()

    # ==============================================================================================================
    # Speak
    # ==============================================================================================================

    def speak(self, text: str):
        """
        텍스트를 음성으로 출력
        """

        if not self._is_initialized:
            return False

        if text is None:
            return False

        text = str(text).strip() # 입력값을 문자열로 바꾸고 앞뒤 공백 제거

        if len(text) == 0:
            return False

        try:
            self._is_speaking = True

            write_log(
                "TTS Speak: %s" % text,
                self
            )

            # ----------------------------------------------------------------------------------------------
            # gTTS
            # ----------------------------------------------------------------------------------------------

            tts = gTTS( # gTTS 객체 생성
                text=text,
                lang=self._language
            )

            tts.save(self._temp_file) # gTTS 객체를 mp3 파일로 저장

            # ----------------------------------------------------------------------------------------------
            # Play
            # ----------------------------------------------------------------------------------------------

            pygame.mixer.music.load(self._temp_file) # 저장된 mp3 파일을 pygame mixer에 로드
            pygame.mixer.music.play() # pygame mixer로 mp3 파일 재생 시작

            while pygame.mixer.music.get_busy(): # pygame mixer가 재생 중인지 확인, 재생이 끝날 때까지 대기
                time.sleep(0.05) # 50ms마다 재생 상태 확인

            self._is_speaking = False

            return True

        except Exception:
            self._is_speaking = False

            ErrorHandler().report()

            return False

        finally:
            self.cleanup()

    # ==============================================================================================================
    # Stop
    # ==============================================================================================================

    def stop(self):
        """
        현재 음성 재생 중지
        """

        try:
            pygame.mixer.music.stop()

            self._is_speaking = False

        except Exception:
            pass

    # ==============================================================================================================
    # Cleanup
    # ==============================================================================================================

    def cleanup(self):
        """
        임시 mp3 파일 삭제
        """

        if os.path.exists(self._temp_file):
            try:
                time.sleep(0.1)
                os.remove(self._temp_file)
            except Exception:
                pass

    # ==============================================================================================================
    # Getter
    # ==============================================================================================================

    def is_speaking(self): # 현재 음성 재생 중인지 여부 반환
        return self._is_speaking

    def get_language(self): # 현재 설정된 언어 반환
        return self._language