#!/usr/bin/env python
from __future__ import print_function

from dobot_py import DobotClient as dc
from dobot_py.jogMode import JOGMode
import tkinter as tk

if __name__ == "__main__":
    isJoint = False
    master = tk.Tk()
    button1 = tk.Button(master, text="maju", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.FOWARD.value))
    button1.pack()
    button2 = tk.Button(master, text="back", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.BACK.value))
    button2.pack()
    button3 = tk.Button(master, text="left", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.LEFT.value))
    button3.pack()
    button4 = tk.Button(master, text="right", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.RIGHT.value))
    button4.pack()
    button5 = tk.Button(master, text="up", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.UP.value))
    button5.pack()
    button6 = tk.Button(master, text="down", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.DOWN.value))
    button6.pack()
    button7 = tk.Button(master, text="left roll", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.LEFT_ROLL.value))
    button7.pack()
    button8 = tk.Button(master, text="right roll", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.RIGHT_ROLL.value))
    button8.pack()
    button9 = tk.Button(master, text="stop", command=lambda: dc.set_jog_cmd(isJoint, JOGMode.STOP.value))
    button9.pack()
    button10 = tk.Button(master, text="homing", command=lambda: dc.set_home_cmd())
    button10.pack()
    button11 = tk.Button(master, text="get pose", command=lambda: print(dc.get_pose()))
    button11.pack()
    master.mainloop()
