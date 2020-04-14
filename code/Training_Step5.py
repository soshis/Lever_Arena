"""
Created on Wed Mar 2017

@author: Adrien Boissenin

@definiton: Step 5:Last and longest step. In this step, the lever is set at the initial position (v_pos = 0.00130 and h_pos = 0.00105)
	and will move to every position row after row. The rat is expected to push the lever, beyond the threshold of 3200, 
	4 times before moving to the next position. An adaptive threshold is setup to assist the rat in that task. First, 
	4 incorrect push allow to reduce the threshold, the number is low to keep the rats motivated to do a large number of 
	pushes. However, for the last row, it was observed that the rats performed better when the number of pushes allowing to 
	reduce the threshold is set to 7 or 8. The task is considered complete when the rats reach maximum threshold (3200) 4 
	times at row 2 and column 4 (most remote position). (expected time: as long as needed, 4-8 sessions).

@modification:

-Change the number of failed trails needed to reduce the threshold by changing the following line:
line 89: "numb_error = 4 #  8 for late training, 4 in early training"
        ==> "numb_error = 8 #  8 for late training, 4 in early training"



- Reduce the number of step to reach the maximum threshold  by changing the following line:
line 69: "threshold = np.array([2600,2700, 2800,2900,3000,3050, 3100,3150,3200]) #for the first row"
        ==> "threshold = np.array([2800, 2900 ,3100,3200]) # suitable in late training"
        
-Remove easy trials at row 0 and 1 (when you are row 2) by changing the following line:
line 187: "if v_ == 2:" ==> "if v_ == 4:"
line 252: "if v_ == 2:" ==> "if v_ == 4:"

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

                                                            

###############################################################################
# Setup log file
###############################################################################

# gdrive
LOGGING = True

animal_number = raw_input("Input animal number and press ENTER: ")
print('Begin training animal ' + str(animal_number))
fileName = (dt.fromtimestamp(time.time()).strftime("%y,%m,%d,%H,%M,%S")+str('_rat_')+str(animal_number)+str('_training_')+str('step5'))

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

""" You can modify """
threshold = np.array([2700,2800,2900,3000,3100,3200]) #for the first row
#threshold = np.array([3000,3100,3200]) # suitable in late training

numb_error = 6#  8 for late training, 4 in early training
it_thres_num = 3 #number of maximum threshold successes before going to next position


""" Do Not Modify"""

init_threshold = 2500

max_threshold = threshold[-1]

v_positions = [0.00130, 0.00120]
h_positions = [0.00100, 0.00105]

errors = 0 # number of too weak pushed  on the lever
#rewardTime = time.time()
ITI_time = 2     # Inter Trials Interval
successes = 0
LED_state = False
last_arena_pos = (0, 0)
leverL_old = 0
it_thres = 0
max_reached = 0


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



###############################################################################
# Adaptive lever positioning based on the previous score#
###############################################################################
v_ = int(raw_input("Input last max Vertcial position reached (0,1,2) and press ENTER: "))
h_ = int(raw_input("Input last max Horizontal position reached (0,1,2,3,4) and press ENTER: "))


    


################################################
# Function Definiton
################################################

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
    time.sleep(0.4)
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

#print " \tLed State Initially: %i" % (LED_state)

# At the last row, it is intersting to start with some easy trials at row 0 and 1 (column 0)
if v_ == 2:
    
    lever_obj = Lever(pin_v_act_L, pin_h_act_L, [0.00115], [0.00130, 0.00120], [3200], verbose=True)
    lever_obj.reset_pos()  # Move lever back to initial position    
    time.sleep(2)
                      
    while successes < 4 :
        currentTime = time.time()
        success = 0
        err = 0
        leverL = adc1.readChannel(lever_ch1)
        nose = adc1.readChannel(nose_ch)


 # Look at a peak in the signal (a local maximum) until a max it reached, it computes if it was a Good trials or not a good trial
        if leverL > leverL_old :
            leverL_old = leverL
        elif (LED_state == False and
            nose < 100 and
            leverL >  lever_obj.get_threshold()):
            # Check for max threshold reaching, so lever go directly to next horizontal_position
            if leverL> max_threshold:
                print('\t*** Max threshold reached ***')
                errors = 0
                max_reached += 1
                success = 1
                successes = successes + 1
                leverL_old = 0
                success_give = True
                give_reward()  # Activate pump/speaker      
                ITI() # set an ITI time and after that time switch the LED on again                       
                if max_reached == 2:
                    lever_obj.advance_v_pos()
                    max_reached = 0
                    
       
        #check if lever correctly went back to original positon after a success
        if LED_state == True and leverL < 2500 :
            LED_off()
            if success_give == True:
                give_reward()
                success_give = False
                ITI() 
            
            
    
        if LOGGING:
            # Check if lever or nose sensors have changed since last loop
            if (leverL, nose) != last_arena_pos:
                last_arena_pos = (leverL, nose)
    
                # Write data
                data_file = open(fileName, 'a')  # open file in append mode##  
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


if v_ == 2:
    print " Lever_easy deleted"
    del lever_obj



if h_ != 0:


    lever_obj_easy = Lever(pin_v_act_L, pin_h_act_L, h_positions[0:h_], [v_positions[v_]], [3200], verbose=True)
    lever_obj_easy.reset_pos()  # Move lever back to initial position    


    while  True :
                
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
            leverL >  lever_obj_easy.get_threshold()):
            # Check for max threshold reaching, so lever go directly to next horizontal_position
            if leverL> max_threshold:
                print('\t*** Max threshold reached ***')
                errors = 0
                max_reached += 1
                success = 1
                successes = successes + 1
                leverL_old = 0
                give_reward()  # Activate pump/speaker      
                ITI() # set an ITI time and after that time switch the LED on again                       
                if max_reached == 3 :
                    if abs(lever_obj_easy.get_h_pos() - (h_positions[h_]-0.00010)) < 1.0e-5:
                        del lever_obj_easy
                        print " Lever_easy deleted"
                        max_reached = 0
                        break
                    else:
                        lever_obj_easy.advance_lever()
                        max_reached = 0
                    
       
        #check if lever correctly went back to original positon after a success
        if LED_state == True and leverL < 2400 :
            LED_off()
            
    
        if LOGGING:
            # Check if lever or nose sensors have changed since last loop
            if (leverL, nose) != last_arena_pos:    
                last_arena_pos = (leverL, nose)
    
                # Write data
                data_file = open(fileName, 'a')  # open file in append mode##  
                data_file.write(str(currentTime) + "," +
                            str(LED_state) + "," +
                            str(leverL) + "," +
                            str(nose) + "," +
                            str(lever_obj_easy.get_threshold()) + "," +
                            str(lever_obj_easy.get_threshold_error()) + "," +
                            str(lever_obj_easy.get_h_pos()) + "," +
                            str(lever_obj_easy.get_v_pos()) + "," +
                            str(success)+ "," +
                            str(ITI_time)+ "," +                            
                            str(err) +"\n")
                #print "Lever: %i\tNose: %i" % last_arena_pos
                data_file.close()    
     
        time.sleep(0.016)

################################################
# Create Object lever
################################################

lever_obj = Lever(pin_v_act_L, pin_h_act_L, h_positions, v_positions, threshold, verbose=True)
lever_obj.set_v_pos(v_)  # Move lever back to initial position
time.sleep(2)
lever_obj.set_h_pos(h_)

print "**** Lever_obj created "
time.sleep(1.5)


while True :

    currentTime = time.time()
    success = 0
    err = 0

    
    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)


   # print " %i" % (leverL) 
        
 # Look at a peak in the signal (a local maximum) until a max it reached, it computes if it was a Good trials or not a good trial
    if leverL > leverL_old :
        leverL_old = leverL
    elif (LED_state == False and
        nose < 100 and
        leverL >  lever_obj.get_threshold()):
        # Check for max threshold reaching, so lever go directly to next horizontal_position
        if leverL> max_threshold:
            print('\t*** Max threshold reached ***')
            errors = 0
            it_thres = 0
            success = 1
            successes = successes + 1
            leverL_old = 0
            max_reached += 1
            lever_obj.increase_threshold()
            give_reward_max()  # Activate pump/speaker
            if max_reached == 3:
                lever_obj.advance_lever()
                lever_obj.reset_threshold()
                max_reached = 0

           
            ITI() # set an ITI time and after that time switch the LED on again      

        #normal case
        else:
            success = 1
            successes = successes + 1
            it_thres += 1
            errors = 0
            leverL_old = 0
            give_reward()  # Activate pump/speaker
            ITI() # set an ITI time and after that time switch the LED on again
            # every 3 sucess threshold increase
            if it_thres == it_thres_num:
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
    # Check for successful trial
           
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
