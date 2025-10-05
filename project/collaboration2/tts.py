from gtts import gTTS
import io
import pyaudio
import pydub

def text_to_speech_streaming(text):
    """입력된 텍스트를 음성으로 변환해 실시간 스트리밍"""
    try:
        # gTTS로 텍스트를 MP3로 변환
        tts = gTTS(text, lang='ko')
        mp3_fp = io.BytesIO()
        tts.write_to_fp(mp3_fp)

        # MP3를 WAV로 변환
        mp3_fp.seek(0)
        audio = pydub.AudioSegment.from_file(mp3_fp, format="mp3")

        # pyaudio로 스트리밍 재생
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=audio.channels, rate=audio.frame_rate, output=True)
        stream.write(audio.raw_data)
        stream.stop_stream()
        stream.close()
        p.terminate()
    except Exception as e:
        print(f"에러 발생: {e}")

def main():
    print("한국어 텍스트를 입력하세요 (종료하려면 '종료' 입력):")
    while True:
        # 사용자 입력 받기
        text = input("> ")
        
        # '종료' 입력 시 프로그램 종료
        if text.strip() == "종료":
            print("프로그램을 종료합니다.")
            break
        
        # 텍스트가 비어있지 않으면 음성 재생
        if text.strip():
            print(f"재생 중: {text}")
            text_to_speech_streaming(text)
        else:
            print("텍스트를 입력해주세요.")

if __name__ == "__main__":
    main()