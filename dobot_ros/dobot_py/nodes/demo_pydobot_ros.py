#!/usr/bin/env python
from __future__ import print_function

from dobot_py.pydobot_ros import Dobot

import tkinter as tk

if __name__ == "__main__":
    dobot = Dobot('dobot1')
    dobot.home()
    dobot.speed(400, 400)
    dobot.move_to(200, 0, 0, 0)
    dobot.move_to(250, 0, 0, 0)
    dobot.wait(1000)
    dobot.move_to(200, 0, 0, 0)
    dobot.move_to(250, 0, 0, 0)
    dobot.suck(1)
    dobot.wait(3000)
    dobot.suck(0)
    print(dobot.pose())
