import io
import threading
from dotenv import load_dotenv
from langchain.chains import LLMChain
from langchain_community.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from gtts import gTTS
import pyaudio
import pydub
from wakeup_word import WakeupWord
from STT import STT
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_pkg_srv_test.srv import CustomService

OPENAI_API_KEY = "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"
openai_api_key = OPENAI_API_KEY
MAX_RETRIES = 10


CIGARETTE_NAME_MAP = {
    "theone": "더원",
    "parliament": "필라멘트",
    "africa": "아프리카",
    "dunhill": "던힐",
    "bohem": "보햄",
    "halla": "한라산",
    "esse": "에쎄",
    "raison": "레종",
    "mevius": "메비우스",
    "thisplus": "디스플러스"
}

def text_to_speech_streaming(text):
    """텍스트를 음성으로 변환해 스트리밍"""
    print(f"TTS triggered with text: {text}")
    try:
        tts = gTTS(text, lang='ko')
        mp3_fp = io.BytesIO()
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)
        audio = pydub.AudioSegment.from_file(mp3_fp, format="mp3")
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=audio.channels, rate=audio.frame_rate, output=True)
        stream.write(audio.raw_data)
        stream.stop_stream()
        stream.close()
        p.terminate()
    except Exception as e:
        print(f"TTS error: {e}")
class ExtractCigaretteInfo:
    def __init__(self):
        """담배 정보 추출 초기화"""
        print("Initializing ExtractCigaretteInfo")
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )
        prompt_content = """
            당신은 담배의 종류를 같은 종류로 구분 해야합니다. 예를들어 팔라멘트,parliament, アクア, 아쿠아 등등이 parliament로 출력해줬으면해
            또한 갯수를 정확히 인식하여 추출해야합니다.

            <목표>
            - 문장에서 담배의 여러이름을 하나의 이름으로 정하여 반환 해주세요.
            - 문장에 등장하는 갯수를 추출하여 반환해 주세요.

            <담배 리스트>
            -  theone, parliament, africa, dunhill, bohem, halla, esse, raison, mevius, thisplus

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [담배1 담배2 ... / 갯수1 갯수2 ...]
            - 담배와 갯수는 공백으로 구분하세요.
            - 사용자가 입력한 순서대로 리스트를 생성하고 그 리스트와 알맞은 갯수를 순서대로 입력해 주세요
            - 담배가 없으면 앞쪽은 공백 없이 비우고, 갯수가 없으면 해당 제품은 '0'개로 반환 해주세요.
            - 단 여러개의 담배를 구매 한다고하는데 각1개씩 이라던가 몇개만 누락하고 말한다면 누락된곳은 공백에,를 추가하여 순서에 문제 없게 해주세요.
            - 각 몇개 이런 식의로의 것은 예를 들어 팔라 아프리카 던힐 각 1개 그리고 보햄 두 갑 주세요 하면 parliament africa dunhill bohem/1 1 1 2 이렇게 출력해주세요.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.
            - 담배의 리스트와 담배의 갯수만 출력하고 그외의 입력은 하지 말아주세요
            - 담배의 종류가 같은걸 따로 말했을때 각각 순서에 맞춰서 출력하게 해주세요.

            <특수 규칙>
            - 명확한 담배 명칭이 없지만 문맥상 유추 가능한 경우(예: "한라산" → halla)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 옛날 명칭을 사용하시는 분들도 있어요 감안해서 해당 브랜드를 연결 시켜반환 해주세요.
            - 줄여서 말하는 분들도 있어요 가령 마쎄 마세 -> 마일드 세븐 -> 메비우스 거든요 이걸 잘 처리 해주세요.
            - 다수의 담배가 등장 시 담배-갯수, 담배-갯수 순으로 사람이 말할태니 잘 구분해서 정확히 매칭하여 순서대로 출력해 주세요.
            - the one 과 this plus는 "the one" "this plus"로 하나로 인식 되어야 합니다
            - ['-', '입력:', '"디', '한갑"', '출력:', 'dunhill'] 이런식의 출력은 잘못되었습니다 담배 리스트에 있는것만 출력해 주세요
            - 발음이 좋지 않은 분들도 많아요 최대한 잘 해석해서 처리해 주세요. 가령 둘값은 두갑을 잘못 발음 한 것이에요. 최대한 비슷한 것은 알아서 처리해주세요.

            <예시>
            - 입력: "더원 줘"  
            출력: theone / 1 
            - 입력: "아쿠아 한갑 줘"  
            출력: parliament / 1
            - 입력: "아프리카랑 던힐 각 두갑씩줘"  
            출력: afirca dunhill /2 2
            - 입력: "보햄 한보루줘"  
            출력: bohem /10
            - 입력: "팔라멘트 아쿠아 마쎄 하나 그리고 마일드 하나 그리고 아프리카 하나줘봐"
            출력: parliament mevius mevius africa / 1 1 1 1
            - 입력: "마쎄 하나 그리고 마일드 하나 팔라멘트 아쿠아 그리고 아프리카 하나줘봐"
            출력:mevius mevius parliament africa / 1 1 1 1

            <사용자 입력>
            "{user_input}"                
        """
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)

    def process_input(self, user_input):
        """사용자 입력에서 담배 정보 추출"""
        print(f"Processing cigarette info: user_input='{user_input}'")
        response = self.lang_chain.invoke({"user_input": user_input})
        result = response["text"].strip().split("/")
        print(f"LLM response: {result}")
        brands = result[0].strip().split() if result[0].strip() else []
        counts = result[1].strip().split() if result[1].strip() else []
        return {
            "type": "success",
            "brands": brands,
            "counts": counts,
            "message": ""
        }

class ConfirmOrder:
    def __init__(self):
        """주문 확인 초기화"""
        print("Initializing ConfirmOrder")
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )
        prompt_content = """
            당신은 편의점 무인 로봇 시스템에서 사용자의 음성 응답을 분석하는 AI입니다. 사용자의 응답을 분석해 긍정 또는 부정 응답인지 판단합니다.

            <목표>
            - 사용자의 음성 응답을 분석해 긍정(True), 부정(False), 또는 모호한 경우 재확인 메시지 반환.
            - 응답이 담배 주문과 관련이 있더라도, 오직 긍정/부정 여부만 판단.

            <출력 형식>
            - 긍정 응답: True
            - 부정 응답: False
            - 모호한 응답: None

            <규칙>
            - 긍정 응답(예: "맞아요", "네", "응", "오케이", "좋아")이면 True 반환.
            - 부정 응답(예: "아니요", "틀렸어요", "다른 거", "아니")이면 False 반환.
            - 모호한 응답(예: "오 마이 갓", "맛있지 않아", "뭐지")이면 "맞는지 다시 말씀해주세요." 반환.
            - 응답에 담배 이름이나 갯수 정보가 포함되어도 무시하고 긍정/부정 여부만 판단.

            <예시>
            1. 응답: "네"
               출력: True
            2. 응답: "아니요"
               출력: False
            3. 응답: "오 마이 갓"
               출력: None
            4. 응답: "맞아요 좋아"
               출력: True
            5. 응답: "그게 아니야 다른 거야"
               출력: False
            6. 응답: "음... 뭐였더라"
               출력: None
            7. 응답: "더원 두갑이야"
               출력: None

            <사용자 응답>
            "{confirmation_input}"
        """
        self.prompt_template = PromptTemplate(
            input_variables=["confirmation_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)

    def generate_confirmation_prompt(self, brands, counts):
        """주문 확인 프롬프트 생성"""
        prompt_parts = []
        for brand, count in zip(brands, counts):
            korean_name = CIGARETTE_NAME_MAP.get(brand, brand)
            unit = "보루" if int(count) == 10 else "갑"
            prompt_parts.append(f"{korean_name} {count}{unit}")
        return ", ".join(prompt_parts) + " 맞으신가요?"

    def process_confirmation(self, confirmation_input):
        """사용자 확인 응답 처리"""
        print(f"Processing confirmation: confirmation_input='{confirmation_input}'")
        response = self.lang_chain.invoke({"confirmation_input": confirmation_input})
        result = response["text"].strip()
        print(f"LLM confirmation response: {result}")
        if result in ["True", "False"]:
            return {"type": "confirmation", "result": result == "True", "message": ""}
        else:
            return {"type": "retry", "message": result}



class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(CustomService, 'custom_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')
        self.req = CustomService.Request()

    def send_request(self, brand, count):
        self.req.brand = brand
        self.req.count = count
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        print("a")
        return future.result()


class VoiceOrderNode(Node):
    def __init__(self):
        super().__init__('voice_order_node')
        # 상태 토픽 구독자
        self.status_sub = self.create_subscription(
            String, '/status', self.status_callback, 10)
        self.current_status = "disabled"  # 초기 상태
        
        # 타이머 및 상태
        self.order_timer = None
        self.is_moving = False
        self.get_logger().info('Voice order node started, subscribed to /status topic')
        # wake up
        self.mic = pyaudio.PyAudio()
        self.wakeup = WakeupWord(buffer_size=1024)
        self.re_flag = True
        # 주문 정보 서비스
        self.service = ServiceClient()

    def status_callback(self, msg):
        """상태 토픽 콜백"""
        self.current_status = msg.data
        self.get_logger().info(f'Received status: {self.current_status}')

    def get_status(self):
        """로봇 상태 반환"""
        if self.current_status in ["moving", "stopped"]:
            return self.current_status
        else:
            self.get_logger().error(f'Invalid status received: {self.current_status}')
            return "disabled"

    def publish_complete(self, brands, counts):
        """주문 완료 서비스 호출"""
        self.service.send_request(brands,counts)

    def wakeup_set(self):
        self.stream = self.mic.open(format=pyaudio.paInt16, channels=1, rate=48000, input=True, frames_per_buffer=1024)
        print("Audio stream opened")
        self.wakeup.set_stream(self.stream)
        print("WakeupWord initialized")
        self.stt = STT(openai_api_key)

    def reset_wakeup(self):
        print("stop audio stream")
        self.stream.stop_stream()
        print("Closing audio stream")
        self.stream.close()
        print("mic terminate")
        self.mic.terminate()
        time.sleep(5)
        self.wakeup_set()

    def process_order(self):
        """주문 처리 루틴"""
        self.wakeup_set()
        extract_cigarette = ExtractCigaretteInfo()
        confirm_order = ConfirmOrder()
        print("ExtractCigaretteInfo and ConfirmOrder initialized")

        while rclpy.ok():
            print("Waiting for wakeword '헬로우 로키'...")
            while not self.wakeup.is_wakeup() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)  # 토픽 메시지 처리
            if not rclpy.ok():
                break
            print("Wakeword detected")
            state = self.get_status()
            self.get_logger().info(f'Robot status: {state}')
            if state == 'moving':
                text_to_speech_streaming("현재 동작 중입니다. 잠시 후 다시 시도해주세요.")
                self.reset_wakeup()
                continue
            elif state != 'stopped':
                self.get_logger().info('Robot not ready, skipping order processing')
                continue
            if self.re_flag:
                text_to_speech_streaming("안녕하세요, 어떤 담배를 찾으시나요?")
                self.re_flag = False
            print("Waiting for user speech input")
            user_input = self.stt.speech2text()
            print(f"User speech input received: {user_input}")
            if not user_input:
                text_to_speech_streaming("음성을 인식하지 못했습니다. 다시 말씀해주세요.")
                continue
            result = extract_cigarette.process_input(user_input)
            print(f"Input processing result: {result}")
            if not result["brands"]:
                text_to_speech_streaming("담배 이름을 인식하지 못했습니다. 다시 말씀해주세요.")
                result = None
                continue
            stored_order = {"brands": result["brands"], "counts": result["counts"]}
            print(stored_order)
            confirmation_prompt = confirm_order.generate_confirmation_prompt(result["brands"], result["counts"])
            text_to_speech_streaming(confirmation_prompt)
            print(f"Confirmation prompt sent: {confirmation_prompt}")
            retry_count = 0
            

            while retry_count < MAX_RETRIES:
                print("Waiting for confirmation input")
                confirmation_input = self.stt.speech2text()
                print(f"Confirmation input received: {confirmation_input}")
                if not confirmation_input:
                    text_to_speech_streaming("응답을 인식하지 못했습니다. 다시 말씀해주세요.")
                    retry_count += 1
                    continue
                confirm_result = confirm_order.process_confirmation(confirmation_input)
                print(f"Confirmation processing result: {confirm_result}")
                if confirm_result["type"] == "confirmation" and confirm_result["result"]:
                    print("Order confirmed")
                    text_to_speech_streaming("주문이 완료되었습니다. 감사합니다.")
                    print(result["brands"])
                    print(type(result["counts"][0]))
                    self.publish_complete(result["brands"], [int(x) for x in result["counts"]])
                    result = None
                    confirmation_input = None
                    self.re_flag = True
                    print(f"Final output: {' '.join(stored_order['brands'])} / {' '.join(stored_order['counts'])}")
                    self.reset_wakeup()
                    break
                else:
                    print("Order rejected")
                    text_to_speech_streaming("다시 주문해주세요. 어떤 담배를 드릴까요?")
                    result = None
                    confirmation_input = None
                    confirm_result = None
                break
            # 주문 처리 완료 후 웨이크 워드 감지 루프로 돌아감

        print("Closing audio stream")
        self.reset_wakeup()

def ros_main(args=None):
    rclpy.init(args=args)
    node = VoiceOrderNode()
    
    try:
        node.process_order()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    ros_main()