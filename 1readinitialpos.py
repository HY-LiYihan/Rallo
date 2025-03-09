import tkinter as tk
from rapid_utils.rapidnode import RapidNode
from math import pi
import time
gello = RapidNode(config_path='args/gello_init.json', port='/dev/ttyUSB0', init_set=False)

while True:
    print(gello.read_pos())
    time.sleep(1)