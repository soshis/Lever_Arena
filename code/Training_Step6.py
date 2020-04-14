"""
Created on Wed Mar 2017

@author: Adrien Boissenin

@definiton:  Step 6: Last step. Starting from row 0, column 0, the goal is to go through each position 2 times (aka 90 successes). After 
	three corrects push the lever will move (threshold at 3200). Once complete the training in over. I observed that to be
	completed the first time the rat need at least 20 minutes but this time is then reduced to 15 min the next session 
	(expected time: 1 or 2 sessions).
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

                                                            

                                                            

###############################################################################
# Setup log file
###############################################################################

# gdrive
LOGGING = True

animal_number = raw_input("Input animal number and press ENTER: ")
print('Begin training animal ' + str(animal_number))
fileName = (dt.fromtimestamp(time.time()).strftime("%y,%m,%d,%H,%M,%S")+str('_rat_')+str(animal_number)+str('_trainig_')+str('step6'))

# Make data folder if doesn't exist
try:
    os.makedirs('training')
except OSError:
    if not os.path.isdir('training'):
        raise
fileName = "training/" + fileName + ".csv"
if LOGGING:
    with open(fileName, 'w') as data_file:
        data_file.write("Time,LED_state,LeverL,nose,threshold,threshold error,horizontal_position,vertical_position,success,ITI_time,error\n")

###############################################################################
# Init Variable
###############################################################################


init_threshold = 2600
threshold = np.array([3200])
max_threshold = threshold[-1]

v_positions = [ 0.00120, 0.001150, 0.00110]
#h_positions = [0.00135, 0.00145]
#v_positions = [0.00120]
h_positions = [ 0.00090,0.00115, 0.00125, 0.00135, 0.00145]



# Arena parameters (Can be changed)
max_reached_numb = 3 #number of succ before switching to next position
ITI_time = 1     # Inter Trials Interval



#initialize parameters DO NOT MODIFY
successes = 0 #initialize 
LED_state = False #initialize 
last_arena_pos = (0, 0)#initialize 
leverL_old = 0#initialize 
it_thres = 0 #initialize


#################
# Pin definitions (DO not modify)
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

################################################
# Create Object lever
################################################

lever_obj = Lever(pin_v_act_L, pin_h_act_L, h_positions, v_positions, threshold, verbose=True)
lever_obj.reset_pos()  # Move lever back to initial position
time.sleep(2)



################################################
# Function Definiton
################################################


def give_reward():
    """Helper function to turn on water"""
    print "Success number %i." % (successes)
    LED_on()
    pump.start(50)
    time.sleep(0.15)
    pump.stop()

def give_reward_max():
    """Helper function to turn on water"""
    print "Success number %i." % (successes)
    LED_on()
    pump.start(50)
    time.sleep(0.15)
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
    
def Timeout():
    """ Time penality when trials pause"""
    print('TIMEOUT')
    LED_on()
    time.sleep(timeout)
    
def ITI():
    """ Inter trials Interval"""
    print('ITI time')
    time.sleep(ITI_time)
#    LED_off()

    
####################
# Main program loop
####################
# Initialize lever and nose sensors
adc1 = mcp3208.mcp3208(0)
adc2 = mcp3208.mcp3208(1)


# Set initial position of lever/nose sensors
last_arena_pos = (adc1.readChannel(lever_ch1),adc1.readChannel(nose_ch) )



print " \tLed State Initially: %i" % (LED_state)


max_reached = 0

while True :

    currentTime = time.time() # Initialize time
    success = 0 # Initialize success count
    err = 0 # Initialize err count
    
    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)

 # Look at a peak in the signal (a local maximum) until a max it reached, it computes if it was a Good trials or not a good trial
    if leverL > leverL_old : #look if lever keep increase
        leverL_old = leverL
    elif (LED_state == False and #as soon as start to decrease look if it reach a level above the threshold
        nose < 100 and
        leverL >  lever_obj.get_threshold()):      
        if leverL> max_threshold:# Check for max threshold reaching, so lever go directly to next horizontal_position
            it_thres = 0
            success = 1
            successes = successes + 1
            leverL_old = 0
            max_reached += 1
            give_reward_max()  # Activate pump/speaker
            if max_reached == max_reached_numb :
                lever_obj.advance_lever()
                max_reached = 0             
            ITI() # set an ITI time and after that time switch the LED on again      

    #check if lever correctly went back to original positon after a success
    if LED_state == True and leverL < 2400 :
        LED_off()

    # SAVING DATA 
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
