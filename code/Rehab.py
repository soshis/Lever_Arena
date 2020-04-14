  # -*- coding: utf-8 -*-
"""
Created on Wed Mar 2017

@author: Adrien Boissenin

@definiton: Association nose spoke and lever push: Begin with 10 nose spokes-only with a lever at an unreachable (low) position 
	(v_pos = 0.00110). Then, lever is moved: v_pos = 0.00130 and h_pos = 0.00120, threshold= 2400. Usually, the rat will 
	push the lever, maintaining it pushed while exploring the environment with its nose. once he learns the association: 
	push + nose spoke => reward, he will become more efficient (doing nose spoke, then a short push on the lever). Again 
	don’t underestimate the importance of the step. The rat need to stick is head right in front of the sensor. Rat 
	performing well here, usually learn quicker more complex step as they can focus on the lever push. When you 
	noticed the rat learn to do both tasks to get the reward, go step 5. (expected time: 2 sessions).
        
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

# Do not run multiple instances of this program at once (or it will confuse position tracking)!

# gdrive
LOGGING = True

# Arena params
v_positions = [0.00135]
h_positions = [0.00140]



#threshold = np.array([2600, 2700,2800,2900,3000,3100,3200])
threshold = np.array([2250,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200])

max_threshold = threshold[-1]
errors = 0 # number of too weak pushed  on the lever
ITI_time = 2     # Inter Trials Interval
inc_thr = 100 #number of success beofre increasing the threshold

successes = 0
LED_state = False
last_arena_pos = (0, 0)
leverL_old = 0
numb_error = 1
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
time.sleep(1.5)


####################
# Setup log file
####################
##stim_state = raw_input("Will the animal be stimulated? Input ON or OFF and press ENTER: ")
#training_number = raw_imput("Input training step number and press ENTER: "
animal_number = raw_input("Input animal number and press ENTER: ")
print('Begin training animal ' + str(animal_number))
fileName = (dt.fromtimestamp(time.time()).strftime("%y,%m,%d,%H,%M,%S")+str('_rat_')+str(animal_number)+str('_Rehab'))

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


#read constant file to initializate the threshold error
with open("constant.txt") as constant:
    for line in constant:
        if line[3:5] == animal_number:
            lever_obj.set_threshold_error(np.ones(np.shape(threshold))*int(line[7:12]))
  

    
#lever_obj.set_threshold_error(np.array([2250, 2250,2250,2250,2250,2250,2250,2250,2250,2250,2250]))






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
    time.sleep(0.3)
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
#    LED_off()

def Save_data(position): # position is the last_arena_position 
    if (leverL, nose) != position:
            position = (leverL, nose)

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
                            str(ITI_time)+ "\n")
            data_file.close()


    
####################
# Main program loop
####################
# Initialize lever and nose sensors
adc1 = mcp3208.mcp3208(0)
adc2 = mcp3208.mcp3208(1)


# Set initial position of lever/nose sensors
last_arena_pos = (adc1.readChannel(lever_ch1),adc1.readChannel(nose_ch) )




while True :

    currentTime = time.time()
    success = 0
    err = 0

    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)

  #  print " %i" % (leverL)

        
 # Look at a peak in the signal (a local maximum) until a max it reached, it computes if it was a Good trials or not a good trial
    if leverL > leverL_old :
        leverL_old = leverL
    elif (LED_state == False and
        nose < 100 and
        leverL >  lever_obj.get_threshold()):
        success = 1
        successes = successes + 1
        it_thres += 1
        errors = 0
        leverL_old = 0
        give_reward()  # Activate pump/speaker
        
        #Here update threshold from last maximum opsition reached
        print " Lever: %i" % (leverL)

        ind_t = np.where(leverL>threshold)[-1] #find which threshold value is the closest
        lever_obj.set_threshold(ind_t[-1]) #update threshold to the closest value of the lever

        ITI() # set an ITI time and after that time switch the LED on again
 
        if it_thres == inc_thr:
            lever_obj.increase_threshold()
            it_thres = 0           
    elif leverL < leverL_old -30 and nose < 100:    
        # Detect some fails
        if (LED_state == False and
            leverL > lever_obj.get_threshold_error() and
            leverL <  lever_obj.get_threshold() ):
            err = 1 #interesting for the data recording
            errors += 1
            print('Fail detected')
            #print('Threshold error: %i' %lever_obj.get_threshold_error() )
            LED_on()
            leverL_old = 0 # once a max is reached, start again the computation   
    # Check for successful trial
    else:
            leverL_old = 0


           
    #check for error trial
    if errors >= numb_error:
        ind_t = np.where(leverL>threshold)[-1]
        if  ind_t.size ==0 :
            ind_t = np.array([0])
        lever_obj.set_threshold(ind_t[-1])
        #lever_obj.decrease_threshold()
        errors = 0
        #print('enter')

    #check if lever correctly went back to original positon after a success
    if LED_state == True and leverL < 2600 :
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
