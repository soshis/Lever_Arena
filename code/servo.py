#!/usr/bin/python
import RPi.GPIO as g
g.setwarnings(False)
g.setmode(g.BCM)
import time

def setActuator(n,s):
    g.setup(6,g.OUT)
    start = time.time()
    current = time.time()
    while current - start < 1:
        g.output(n,True)
        time.sleep(s)
        g.output(n,False)
        time.sleep(0.05)
        current = time.time()
    

