__author__ = 'jeremie'

from bus import *
from parser_core import *

class F130_Keypad:
    def __init__(self):
        # Create bus object
        self.bus = Bus()
        # Create a dictionary to be used to keep states from joy_core
        self.states = { 'A':0, 'B':0, 'X':0, 'Y':0, \
                        'Back':0, 'Start':0, 'Middle':0, \
                        'Left':0, 'Right':0, 'Up':0, 'Down':0, \
                        'LB':0, 'RB':0, 'LT':0, 'RT':0, \
                        'LJ/Button':0, 'RJ/Button':0, \
                        'LJ/Left':0, 'LJ/Right':0, 'LJ/Up':0, 'LJ/Down':0, \
                        'RJ/Left':0, 'RJ/Right':0, 'RJ/Up':0, 'RJ/Down':0, \
                        'Byte0':0, 'Byte1':0, 'Byte2':0, 'Byte3':0, \
                        'Byte4':0, 'Byte5':0, 'Byte6':0, 'Byte7':0, \
                        'Byte0/INT':0, 'Byte1/INT':0, 'Byte2/INT':0, \
                        'Byte3/INT':0, 'Byte4/INT':0, 'Byte5/INT':0, \
                        'Byte6/INT':0, 'Byte7/INT':0}

        # Launch Parser_Core as a seperate thread to parse the gamepad
        self.parsercore = ParserCore(self.bus, self.states)
        self.parsercore.start()
