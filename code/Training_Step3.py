  # -*- coding: utf-8 -*-
"""
Created on Wed Mar 2017

@author: Adrien Boissenin

@definiton: - Step 3: Learning to push the lever on a wider range with an adaptive threshold. Joystick is moved at v_pos = 0.00130 and 
	h_pos = 0.00120 horizontal position (threshold set at 2400). Success for the trial is notified with beep sound. 
	Lever threshold increases each 5 corrects push. If 5 incorrect pushes on the Lever before a successful trial, the 
	lever threshold decrease. When the rat reached a threshold of at least 3100 (maximum threshold is 3200), the step 
	is complete. (expected time: 1 sessions or 2 sessions, can be done right after step2 if training time remains).
        As soon as they learn switch to next one, no need of rehinforcing to much this behavior
"""

#!/usr/bin/python
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

# gdrive
LOGGING = True


""" You Can Modify """

#threshold = np.array([ 2400,2500,2600,2700,2800,2900,3000,3100,3200]) #day 1
threshold = np.array([2600, 2700,2800,2900,3000,3100,3200]) #day 2
#threshold = np.array([ 2900,3000,3100,3200]) #day 3
#threshold = np.array([ 3000,3100,3200]) #day mille

numb_error = 10 #number of fail before decreasing the threshold

"""Do not Modify"""
# Arena params
v_positions = [0.00130]
h_positions = [0.00120, 0.00125] #two different positions here are neccessary for the function "decrease threshold to work"

#initilization parameters
init_threshold = 2300
max_threshold = threshold[-1]
errors = 0 
ITI_time = 2     # Inter Trials Interval
successes = 0
LED_state = False
last_arena_pos = (0, 0)
leverL_old = 0
it_thres = 0


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

#for temp_pin in [pin_power]:
#    GPIO.setup(temp_pin, GPIO.IN)
for temp_pin in [pin_v_act_manual, pin_h_act_manual]:
    GPIO.setup(temp_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Create lever object
lever_obj = Lever(pin_v_act_L, pin_h_act_L, h_positions, v_positions, threshold, verbose=True)
lever_obj.reset_pos()  # Move lever back to initial position
lever_obj.advance_lever() #move to the second horizontal position
time.sleep(1.5)


####################
# Setup log file
####################
animal_number = raw_input("Input animal number and press ENTER: ")
print('Begin training animal ' + str(animal_number))
fileName = (dt.fromtimestamp(time.time()).strftime("%y,%m,%d,%H,%M,%S")+str('_rat_')+str(animal_number)+str('_training_')+str('step3'))

# Make data folder if doesn't exist
try:
    os.makedirs('training')
except OSError:
    if not os.path.isdir('training'):
        raise
fileName = "training/" + fileName + ".csv"
if LOGGING:
    with open(fileName, 'w') as data_file:
        data_file.write("Time,LED_state,leverL,nose,threshold,threshold_error,h_pos,v_pos,success,ITI,err\n")

def give_reward():
    """Helper function to turn on water"""
    print "Success number %i." % (successes)
    LED_on()
    pump.start(50)
    time.sleep(0.2)
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
 #   print('LED ON')
    LED.start(1)
    global LED_state
    LED_state = True

    
def LED_off():
    """ Turn the LED off to indicate the start of the trials"""
#    print('LED OFF')
    LED.stop()
    global LED_state
    LED_state = False
    

def ITI():
    """ Inter trials Interval"""
    print('ITI time')
    time.sleep(ITI_time)


    
####################
# Main program loop
####################
# Initialize lever and nose sensors
adc1 = mcp3208.mcp3208(0)
adc2 = mcp3208.mcp3208(1)


# Set initial position of lever/nose sensors
last_arena_pos = (adc1.readChannel(lever_ch1),adc1.readChannel(nose_ch) )

print " \tLed State Initially: %i" % (LED_state)
lever_obj.advance_h_pos()


while True :

    currentTime = time.time()
    success = 0
    err = 0

    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)

        
 # Look at a peak in the signal (a local maximum) until a max it reached, it computes if it was a Good trials or not a good trial
    if leverL > leverL_old :
        leverL_old = leverL
    elif (LED_state == False and
        leverL >  lever_obj.get_threshold()):
        if leverL> max_threshold:
            print('\t*** Max threshold reached ***')
            errors = 0
            success = 1
            successes = successes + 1
            leverL_old = 0
            give_reward()
            lever_obj.increase_threshold()# Activate pump/speaker      
            ITI() # set an ITI time and after that time switch the LED on again                       
        # normal case
        else:
            success = 1
            successes = successes + 1
            it_thres += 1
            errors = 0
            leverL_old = 0
            give_reward()  # Activate pump/speaker
            ITI() # set an ITI time and after that time switch the LED on again
            # every 3 sucess threshold increase
            if it_thres == 5:
                lever_obj.increase_threshold()
                it_thres = 0
    elif leverL < leverL_old - 50:    
        # Detect some fails
        if (LED_state == False and
            leverL > lever_obj.get_threshold_error() and
            leverL <  lever_obj.get_threshold() ):
            err = 1 #interesting for the data recording
            errors += 1
            print('Fail detected')
            leverL_old = 0 # once a max is reached, start again the computation   
           
    #check for error trial
    if errors >= numb_error:
        lever_obj.decrease_threshold()
        errors = 0


    #check if lever correctly went back to original positon after a success
    if LED_state == True and leverL < 2400 :
        LED_off()
        

        
    if LOGGING:
        # Check if lever or nose sensors have changed since last loop
        if (leverL, nose) != last_arena_pos:
            last_arena_pos = (leverL, nose)

            # Write data
            data_file = open(fileName, 'a')  # open file in append mode
            data_file.write(str(currentTime) + "," +
                            str(LED_state) + "," +
                            str(leverL) + "," +
                            str(nose) + "," +
                            str(lever_obj.get_threshold()) + "," +
                            str(lever_obj.get_threshold_error()) + "," +
                            str(lever_obj.get_h_pos()) + "," +
                            str(lever_obj.get_v_pos()) + "," +
                            str(success)+ "," +                           
                            str(ITI_time)+ "," +                            
                            str(err) +"\n")
            #print "Lever: %i\tNose: %i" % last_arena_pos
            data_file.close()

    time.sleep(0.016)

LED_off() #end of the step
