#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math

from tkinter import *
import random
import itertools
from PIL import Image, ImageTk, ImageSequence

class ButtonPage():
  

    def __init__(self, root):
        self.tk = Frame(root)
        self.tk.pack(expand=True, fill="both")
        self.width=1280
        self.height=1080

        self.canvas = Canvas(self.tk, width=self.width, height=self.height)
        self.canvas.pack(anchor='nw')
        self.canvas.configure(bg='white')
       
        self.tk.configure(bg="white")
        self.tk.rowconfigure(tuple(range(1)), weight=1)
        self.tk.columnconfigure(tuple(range(1)), weight=1)
        self.col_a = "red"
        self.col_b = "red"
        button_width = 10  # Set the width of the buttons
        button_height = 5 
        self.counter = 0
        self.gif_path = '/home/tolasing/Downloads/speaking.gif'  # Update this to the path of your GIF
        self.gif = Image.open(self.gif_path)
        self.gif_frames = [ImageTk.PhotoImage(img) for img in ImageSequence.Iterator(self.gif)]

        self.current_frame = 0
        self.gif_label = None


    def update_image(self):
        self.tk.update()
        if self.gif_label:
            self.canvas.itemconfig(self.gif_label, image=self.gif_frames[self.current_frame])
        else:
            self.gif_label = self.canvas.create_image(self.width // 2, self.height // 2, image=self.gif_frames[self.current_frame], anchor=CENTER)

        # Update the frame index
        self.current_frame = (self.current_frame + 1) % len(self.gif_frames)

        # Schedule the next update
        self.tk.after(500, self.update_image)  # Adjust the delay as needed

    def destroy(self):
        self.tk.destroy()


