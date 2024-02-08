import sounddevice as sd
import soundfile as sf

class UserAudio:

    def record_audio(self):
        file_path="user_input.wav"
        duration=5
        sample_rate=96000
        print(f"Recording {duration} seconds of audio...")
        audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=2, dtype='int16')
        sd.wait()
        

        print(f"Saving recorded audio to {file_path}")
        sf.write(file_path, audio_data, sample_rate)
        return file_path

    def play_audio(file_path):
        print(f"Playing audio from {file_path}")
        audio_data, sample_rate = sf.read(file_path, dtype='int16')
        sd.play(audio_data, samplerate=sample_rate)
        sd.wait()
    """
    if __name__ == "__main__":
        audio_file_path = "recorded_audio.wav"

        # Record audio to a file
    # record_audio(audio_file_path)

        # Play back the recorded audio
        play_audio(audio_file_path)
    """