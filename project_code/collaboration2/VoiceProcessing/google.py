
import speech_recognition as sr

recognizer = sr.Recognizer()
with sr.Microphone() as source:
    print("말하세요 (예: '위로', '지갑 줘'):")
    recognizer.adjust_for_ambient_noise(source)  # 배경 소음 보정
    audio = recognizer.listen(source, timeout=5)
    try:
        text = recognizer.recognize_google(audio, language="ko-KR")
        print("인식된 명령:", text)
    except sr.UnknownValueError:
        print("음성을 이해하지 못했습니다.")
    except sr.RequestError as e:
        print(f"Google API 오류: {e}")