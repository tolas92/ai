from openai import OpenAI


class OpenaiTranscript:

    def transcribe_():

        audio_file = open("recorded_audio.wav", "rb")
        transcript = client.audio.transcriptions.create(
        model="whisper-1", 
        file=audio_file, 
        response_format="text"
        )
        print (transcript)
