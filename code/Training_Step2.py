# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 11:57:37 2017

@author: Adrien Boissenin

@definitions: - Step2: Learning to push the lever. Each time, start with 5 nose spokes to get the reward. First real lever is in low 
	position (v_pos = 0.00110, h_pos =0 .0130). Use dummy lever to draw rat’s attention. Give reward when rat touches 
	the dummy lever and then when the rat puches the dummy lever. Start by putting the dummy lever at a similar level 
	than the platform and then, to prevent him using its teeth, lower your dummy lever. Once rat learned to pushes a 
	little bit the dummy lever, shift attention from dummy lever to real lever by rising the real lever to 
	h_pos = 0.00120 and finalnly h_pos = 0.00130 (threshold = 2500). The script will stop and ask if you want to rise 
	the lever after the 20th and 35th successes. press 'y' (yes) or 'n' (no) then press 'enter' to move it 
	-or not-. Once you believe rats acquire the movement step is complete. (expected time: 2 sessions).
"""



import spidev  # SPI interface (inside the RaspPi)
import time
import mcp3002
import mcp3208
import RPi.GPIO as GPIO
import os
from datetime import datetime as dt
from lever_class import Lever


LOGGING = True

# Arena params

"""
What you can modify
"""
nb_nose_poke = 5 # Session will start with nose spoke only, here set the numbr of nose poke to start with (can be 10,20 for first time at step2,or 1,5 if you are sure the rat learn to nose spoke 



""" Do not modify"""
v_positions = [0.00100, 0.00110, 0.00130]
h_positions = [0.001350]

threshold = [2300]
threshold_error = 2200 # arbitrary fixed for the moment
errors = 0 # number of too weak pushed  on the lever
ITI_time = 2     # Inter Trials Interval
successes = 0
LED_state = False
move_lever = False
move_threshold = False
last_arena_pos = (0, 0)

#################
# Pin definitions
#################
pin_v_act_L = 20 # Vertical actuator, left; 13.5% - 9.5%
pin_h_act_L = 26   # Horizontal actuator, left; 15.5% - 10%
pin_pump = 27       
pin_speaker = 15   # speaker link to the reward
pin_success = 14 
pin_LED = 13       # check if it is applicable
pin_v_act_manual = 5  # pin to manually advance vertical position
pin_h_act_manual = 5  # pin to manually advance horizontal position


lever_ch1 = 0 #Lever adc channel
lever_ch2 = 3
nose_ch = 7 #nosepoke adc channel

#######################################
# Initialize pins
#######################################
GPIO.setmode(GPIO.BCM)
for temp_pin in [pin_pump, pin_speaker, pin_LED, pin_success]: 
    GPIO.setup(temp_pin, GPIO.OUT)
pump = GPIO.PWM(pin_pump, 1000)
LED = GPIO.PWM(pin_LED, 1000)

for temp_pin in [pin_v_act_manual, pin_h_act_manual]:
    GPIO.setup(temp_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Create lever object
lever_obj = Lever(pin_v_act_L, pin_h_act_L, h_positions, v_positions,
                  threshold, verbose=True)
lever_obj.reset_pos()  # Move lever back to initial position
time.sleep(1.5)



####################
# Setup log file
####################
animal_number = raw_input("Input animal number and press ENTER: ")
print('Begin training animal ' + str(animal_number))
fileName = (dt.fromtimestamp(time.time()).strftime("%y,%m,%d,%H,%M,%S")+str('_rat_')+str(animal_number)+str('_training_')+str('step2'))

# Make data folder if doesn't exist
try:
    os.makedirs('training')
except OSError:
    if not os.path.isdir('training'):
        raise
fileName = "training/" + fileName + ".csv"
if LOGGING:
    with open(fileName, 'w') as data_file:
        data_file.write("Time,LED_state,leverL,nose,threshold,h_pos,v_pos,success,ITI,\n")


def give_reward():
    """Helper function to turn on water"""
    print "\tSuccess number %i" % (successes)
    LED_on()
    pump.start(50)
    time.sleep(0.3)
    pump.stop()
    
def LED_on():
    """ Turn the LED ON to indicate the end of the trials"""
#    print('LED ON')
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
    LED_on()
    time.sleep(timeout)
#    print('TIMEOUT')
    
def ITI():
    """ Inter trials Interval"""
    time.sleep(ITI_time)
    LED_off()

def Save_data(position): # position is the last_arena_position 
    if (leverL, nose) != position:
            position = (leverL, nose)

            # Write data
            data_file = open(fileName, 'a')  # open file in append mode
            data_file.write(str(currentTime) + "," +
                            str(LED_state) + "," +
                            str(leverL) + "," +
                            str(nose) + "," +
                            str(threshold) + "," +
                            str(lever_obj.get_h_pos()) + "," +
                            str(lever_obj.get_v_pos()) + "," +
                            str(success)+ "," +
                            str(ITI_time)+ "\n")
            #print "Lever: %i\tNose: %i" % last_arena_pos
            data_file.close()

    
####################
# Main program loop
####################
# Initialize lever and nose sensors
adc1 = mcp3208.mcp3208(0)
adc2 = mcp3208.mcp3208(1)


# Set initial position of lever/nose sensors
last_arena_pos = (adc1.readChannel(lever_ch1),adc1.readChannel(nose_ch) )



print " \tLed State Initially: %i" % (LED_state)

"Some easy trials to start with -Nose spoke trials"
while successes < nb_nose_poke:
    currentTime = time.time()
    success = 0

    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)
    
    if (LED_state == False and
        nose < 100):
            success =  1
            successes = successes + success
            give_reward()  # Activate pump/speaker

            ITI() # set an ITI time and after that time switch the LED on again
 
    if LOGGING:
        Save_data(last_arena_pos)
    time.sleep(0.016)


while successes < 30:

    currentTime = time.time()
    success = 0

    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)
      
       
    # Check for successful trial
    if (LED_state == False and
        leverL > lever_obj.get_threshold()):
        success =  1
        successes = successes + success
        give_reward()
        ITI() # set an ITI time and after that time switch the LED on again
    
    if LOGGING:
        Save_data(last_arena_pos)
    time.sleep(0.016)
  
        
         
    # Check if we need to advance to next position
LED_on()
    
"Rise Lever?"
answer = raw_input("move lever? [y]/[n]")
if answer == 'y':
    lever_obj.advance_lever()
  

LED_off()
"Again same trials but with a Lever closer"      
while successes < 50:
    currentTime = time.time()
    success = 0
    manual_lever_v = 0
    manual_lever_h = 0
    fail_push = 0
    fail_nose = 0
    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)
    
     # Check for successful trial
    if (LED_state == False and
        leverL > lever_obj.get_threshold()):
        success =  1
        successes = successes + success
        give_reward()  # Activate pump/speaker
        #Sound() # Make the reward sound
        ITI() # set an ITI time and after that time switch the LED on again

    if LOGGING:
        Save_data(last_arena_pos)
    time.sleep(0.016)

LED_on()


answer = raw_input("move lever? [y]/[n]")
if answer == 'y':
    lever_obj.advance_lever()   



LED_off() 
      
while successes < 90:
    currentTime = time.time()
    success = 0

    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)
    
     # Check for successful trial
    if (LED_state == False and
        leverL > lever_obj.get_threshold()):
        success =  1
        successes = successes + success
        give_reward()  # Activate pump/speaker
        ITI() # set an ITI time and after that time switch the LED on again    
        
        
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
                            str(threshold) + "," +
                            str(lever_obj.get_h_pos()) + "," +
                            str(lever_obj.get_v_pos()) + "," +
                            str(success)+ "," +                          
                            str(ITI_time)+ "\n")
            #print "Lever: %i\tNose: %i" % last_arena_pos
            data_file.close()

    time.sleep(0.016)

LED_off() #end of the step
