from pathlib import Path
from openai import OpenAI
import pygame

client = OpenAI()

# Set the file path using pathlib
speech_file_path = Path(__file__).parent / "speech.mp3"

response = client.audio.speech.create(
    model="tts-1",
    voice="nova",
    input="Good evening, and welcome to our fine dining establishment. I'm Rosie, your server for the evening. Allow me to present to you our carefully curated menu for this Wednesday: For the main course:- Sambar Rice: A flavorsome blend of lentils and spices mixed with rice, a staple dish in South Indian cuisine. Accompanying the main course as a snack:

)

response.stream_to_file(speech_file_path)

pygame.init()
pygame.mixer.init()
pygame.mixer.music.load(str(speech_file_path))
pygame.mixer.music.play()

# Wait for the audio to finish playing
while pygame.mixer.music.get_busy():
    pygame.time.Clock().tick(10)
