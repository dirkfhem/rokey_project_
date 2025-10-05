import os
from dotenv import load_dotenv
from langchain.chains import LLMChain
from langchain.chat_models import ChatOpenAI
import warnings


from langchain.prompts import PromptTemplate


class ExtractKeyword:
    def __init__(self):
        OPENAI_API_KEY='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'

        openai_api_key = OPENAI_API_KEY
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

            <예시>
            - 입력: "더원 줘"  
            출력: theone / 0

            - 입력: "아쿠아 한갑 줘"  
            출력: parliament / 1

            - 입력: "아프리카랑 던힐 각 두갑씩줘"  
            출력: afirca dunhill /2 2

            - 입력: "보햄 한보루줘"  
            출력: bohem /10

            - 입력: "팔라멘트 아쿠아 마쎄 하나 그리고 마일드 하나 그리고 아프리카 하나줘봐"
            출력: parliament mevius mevius africa / 0 1 1 1

            - 입력: "마쎄 하나 그리고 마일드 하나 팔라멘트 아쿠아 그리고 아프리카 하나줘봐"
            출력:mevius mevius parliament africa / 1 1 0 1

            <사용자 입력>
            "{user_input}"                
        """
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response["text"].strip().split("/")
        if len(result) != 2:
            warnings.warn("The object list is more than one.")
            return None

        object, destination = result[0], result[1]
        object = object.split()
        destination = destination.split()

        print(f"llm's response(object): {object}")
        print(f"llm's response(destination): {destination}")

        return object, destination


if __name__ == "__main__":
    # stt = STT()
    # output_message = stt.speech2text()
    output_message = "한라 마쌔 주고 필라멘트 하나 마일드 하나 그리고 아쿠아 보햄 레종 각 두개씩줘 마지막으로 던힐 한보루 줄래?"
    extract_keyword = ExtractKeyword()
    keyword = extract_keyword.extract_keyword(output_message)
