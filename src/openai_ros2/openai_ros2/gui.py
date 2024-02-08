"""
Minimal GUI for Assistant
"""
import threading
import PySimpleGUI as sg
from assistant import *
from audio2 import *
from audio import *
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from prototype.action import FoodMenu
from functions import *



class OpenaiAssitant(Node):
    def __init__(self):
        super().__init__('openai_assistant') 
        self.OUTPUT = '-OUT-'+sg.WRITE_ONLY_KEY
        

        self.AI = Assistant()
        self.useraudio=UserAudio()
        self.full_text = ""
        self.functions=Functions()

        sg.theme('DarkGrey 5')
        sg.Print('STDOUT logged here', do_not_reroute_stdout=True)  # Routes stdout to a "debug" window
        """
        layout = [[sg.MLine(key='Input', size=(60,2), enter_submits=True), sg.Text("<< Message the Assistant"), sg.Button('Submit', bind_return_key=True)],
                [sg.MLine(key=OUTPUT, size=(120, 30), autoscroll=True, write_only=True, disabled=True)],
                [sg.Button('Record Audio',key='RecordAudio')]]
        """
        self.layout = [
            [sg.Multiline(key=self.OUTPUT, size=(80, 20), autoscroll=True, font=('Helvetica', 12), text_color='white', background_color='#333')],
            [sg.MLine(key='Input', size=(60, 2), enter_submits=True, font=('Helvetica', 12), text_color='white', background_color='#444'),
                    sg.Button('↑', key='UpArrow', button_color=('white', '#555'), size=(3, 1),bind_return_key=True)],
            [sg.Button('Record Audio', key='RecordAudio', button_color=('white', '#555'), size=(15, 1))]
        ]
        self.window = sg.Window('Rosie', self.layout,background_color='#222', resizable=True, finalize=True)
        #self.timer=self.create_timer(0.1,self.update_gui)
        self.action_client= ActionClient(self,FoodMenu, '/Kitchen')


    def update_gui(self):
            
        while True:
            event, values = self.window.read()
            print('\n', event, values)
            if event == sg.WIN_CLOSED or event == 'Exit':
                break

            if event == 'RecordAudio':
                audio_data = self.useraudio.record_audio()
                user_text=self.AI.transcribe_(audio_data)
                self.window['Input'].Update(user_text)
                message = user_text
                self.window[self.OUTPUT].print('Sending:\n\t' + message, c='white')
                self.window['Input'].Update('')  # Clear the input text
                threading.Thread(target=self.AI.send_message, args=(self.window, message), daemon=True).start()
                
            if event == 'UpArrow' or event == "Input_Enter":
                message = values['Input']
                self.window[self.OUTPUT].print('Sending:\n\t' + message, c='blue')
                self.window['Input'].Update('')  # Clear the input text
                threading.Thread(target=self.AI.send_message, args=(self.window, message), daemon=True).start()

            if event == AI_RESPONSE:
            # window[OUTPUT].print('\n[Received response:]', c='red')
                for m in values[AI_RESPONSE]:
                    """
                    window[OUTPUT].print(f"{m.role}: {m.content[0].text.value}")
                    """
                    role = m.role
                    content = m.content[0].text.value
                    self.window[self.OUTPUT].print(f"{role}: {content}")

                    # Concatenate content to the full text
                    self.full_text += content + " "
                    print(self.full_text)
                    print('\n')
                self.AI.speak_rosie(self.full_text)
                self.full_text=" "

                                
        self.window.close()



def main(args=None):
  rclpy.init(args=args)
  openai_node=OpenaiAssitant()
  rclpy.spin(openai_node)
  openai_node.destroy_node()
  rclpy.shutdown()
        
if __name__ == '__main__':
    main()
