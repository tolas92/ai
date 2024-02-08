import sounddevice as sd
import soundfile as sf
import numpy as np
import time

class UserAudio:
    def __init__(self):
        self.audio_queue = []
        self.threshold=0.05

    def callback(self, indata, frames, time, status):
        # This is called asynchronously for each audio block
        if status:
            print(status, flush=True)

        # Check if the audio block contains any sound (above a certain threshold)
        if np.max(np.abs(indata)) > self.threshold:
            self.audio_queue.extend(indata.copy())

    def record_audio(self):
        file_path = "user_input.wav"
        duration = 5
        sample_rate = 96000
        self.threshold = 0.05  # Adjust based on your environment

        # Start streaming audio input with the callback
        with sd.InputStream(callback=self.callback, channels=2, samplerate=sample_rate):
            print(f"Recording automatically when you speak for {duration} seconds...")

            # Wait for a certain duration of speaking
            sd.sleep(int(duration * 1000))

        # Convert the recorded audio queue to numpy array
        audio_data = np.array(self.audio_queue)

        print(f"Saving recorded audio to {file_path}")
        sf.write(file_path, audio_data, sample_rate)
        time.sleep(1)

    def play_audio(self, file_path):
        print(f"Playing audio from {file_path}")
        audio_data, sample_rate = sf.read(file_path, dtype='int16')
        sd.play(audio_data, samplerate=sample_rate)
        sd.wait()


if __name__ == "__main__":
    user_audio = UserAudio()
    while True:
    # Record audio automatically when speaking
     user_audio.record_audio()

    # Play back the recorded audio
   # user_audio.play_audio(audio_file_path)
