# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 11:13:57 2017

@author: Adrien Boissenin

@Definition: Step Random: each 3 sucesses, the lever moves to a random row and a random column. This code is use after the training,
to prevent the rat to forget the task. 
5min every 2 or 3 days until you stop water restriction and perform surgery on the rat.
"""


import spidev  # SPI interface (inside the RaspPi)
import time
import random
import math
import numpy as np
import mcp3208
import RPi.GPIO as GPIO
import os
from datetime import datetime as dt
from lever_class import Lever

# XXX Do not run multiple instances of this program at once (or it will confuse position tracking)!


# Arena params
v_positions = [0.00130]
h_positions = [0.00135]

threshold = np.array([3200])

ITI_time = 0     # Inter Trials Interval

successes = 0
LED_state = False
last_arena_pos = (0, 0)
leverL_old = 0


#################
# Pin definitions
#################
pin_v_act_L = 20  # Vertical actuator, left; 13.5% - 9.5%
pin_h_act_L = 26   # Horizontal actuator, left; 15.5% - 10%
pin_pump = 27       
pin_speaker = 15   # speaker link to the reward
pin_success = 14 
pin_LED = 13       # check if it is applicable
pin_v_act_manual = 5  # pin to manually advance vertical position
pin_h_act_manual = 5  # pin to manually advance horizontal position


lever_ch1 = 0#Lever adc channel
lever_ch2 = 3
nose_ch = 7 #nosepoke adc channel

#######################################
# Initialize pins
#######################################
GPIO.setmode(GPIO.BCM)
for temp_pin in [pin_pump, pin_speaker, pin_LED, pin_success]: 
    GPIO.setup(temp_pin, GPIO.OUT)
pump = GPIO.PWM(pin_pump, 1000)
LED = GPIO.PWM(pin_LED,1000)


for temp_pin in [pin_v_act_manual, pin_h_act_manual]:
    GPIO.setup(temp_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Create lever object
lever_obj = Lever(pin_v_act_L, pin_h_act_L, h_positions, v_positions, threshold, verbose=True)
lever_obj.reset_pos()  # Move lever back to initial position
time.sleep(1.5)


####################
# Setup log file
####################


def give_reward():
    """Helper function to turn on water"""
    print "Success number %i." % (successes)
    LED_on()
    pump.start(50)
    time.sleep(0.3)
    pump.stop()

def give_reward_max():
    """Helper function to turn on water"""
    print "Success number %i." % (successes)
    LED_on()
    pump.start(50)
    time.sleep(0.5)
    pump.stop()

    
def LED_on():
    """ Turn the LED ON to indicate the end of the trials"""
    LED.start(1)
    global LED_state
    LED_state = True

    
def LED_off():
    """ Turn the LED off to indicate the start of the trials"""
    LED.stop()
    global LED_state
    LED_state = False
    
def Timeout():
    """ Time penality when trials pause"""
    print('TIMEOUT')
    LED_on()
    time.sleep(timeout)
    
def ITI():
    """ Inter trials Interval"""
    print('ITI time')
    time.sleep(ITI_time)

success_give = False

    
####################
# Main program loop
####################
# Initialize lever and nose sensors
adc1 = mcp3208.mcp3208(0)
adc2 = mcp3208.mcp3208(1)


# Set initial position of lever/nose sensors
last_arena_pos = (adc1.readChannel(lever_ch1),adc1.readChannel(nose_ch) )

print " \tLed State Initially: %i" % (LED_state)

while True :
    currentTime = time.time()
    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)
    
    #print (leverL)
    
 # Look at a peak in the signal (a local maximum) until a max it reached, it computes if it was a Good trials or not a good trial
    if leverL > leverL_old :
        leverL_old = leverL
        print (leverL)
    elif (LED_state == False and
        nose < 100 and
        leverL >  lever_obj.get_threshold()):
        # Check for max threshold reaching, so lever go directly to next horizontal_position
        success_give = True
        LED_on()
        successes = successes + 1
        leverL_old = 0
       # give_reward()  # Activate pump/speaker
            # every 3 sucess threshold increase
        
       # ITI() # set an ITI time and after that time switch the LED on again
        
  
        #check if lever correctly went back to original positon after a success
    if LED_state == True and leverL < 2900 :
        LED_off()
        if success_give == True:
            give_reward()
            success_give = False
            successes = successes + 1
  

    time.sleep(0.016)

LED_off() #end of the step
