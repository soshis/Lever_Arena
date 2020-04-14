# -*- coding: utf-8 -*-
"""
Created on Tue Mar 21 15:34:57 2017

@author: Adrien Boissenin

@Definition: Step 1, perform nose poke. Success for nose poke is notified with beep sound and reward. After a single session 
	with 40 trials, the step is completed. Don’t forget to progressively adjust the gain knob to its minimum. Nose 
	spoke should not be neglected. Rats that don't learn to do nose spoke right inside the sensor neck performed 
	poorer when the task become more complicated. They learn slower as they get easier confuse and their lever pushes 
	are less ‘proper’(expected time: 2 sessions).

"""


import spidev  # SPI interface (inside the RaspPi)
import time
import mcp3002
import mcp3208
import RPi.GPIO as GPIO
import os
from datetime import datetime as dt
from lever_class import Lever

# XXX Do not run multiple instances of this program at once (or it will confuse position tracking)!

# gdrive
LOGGING = True

# Arena params
v_positions = [0.00090] # There is no need of Lever movement at this stage
h_positions = [0.00130] # An intermediate value is choosen

threshold = [3000] # for step 1 no need of Lever threshold but required for Lever_class definition
ITI_time = 2     # Inter Trials Interval # no need for step1
successes = 0
leverL_old = 0
LED_state = False
last_arena_pos = (0, 0)

#################
# Pin definitions
#################

pin_v_act_L = 20 # Vertical actuator, left; 13.5% - 9.5%
pin_h_act_L = 26   # Horizontal actuator, left; 15.5% - 10%
pin_pump = 27
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
for temp_pin in [pin_pump, pin_LED]: 
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
fileName = (dt.fromtimestamp(time.time()).strftime("%y,%m,%d,%H,%M,%S")+str('_rat_')+str(animal_number)+str('_trainig_')+str('step1'))

# Make data folder if doesn't exist
try:
    os.makedirs('training')
except OSError:
    if not os.path.isdir('training'):
        raise
fileName = "training/" + fileName + ".csv"
if LOGGING:
    with open(fileName, 'w') as data_file:
        data_file.write("time,leverL,nose,threshold,horizontal,vertical,manual_lever_h,manual_lever_v,success\n")


def give_reward():
    """Helper function to turn on water"""

    print "\tSuccess number %i." % (successes)
    LED_on()
    pump.start(50)
    time.sleep(0.3)
    pump.stop()

    
def LED_on():
    """ Turn the LED ON to indicate the end of the trials"""
    print('LED ON')
    LED.start(1)
    global LED_state
    LED_state = True

    
def LED_off():
    """ Turn the LED off to indicate the start of the trials"""
    print('LED OFF')
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
    LED_off()

  
################################################################################
#                                       Main program loop
################################################################################

# Initialize lever and nose sensors
adc1 = mcp3208.mcp3208(0)
adc2 = mcp3208.mcp3208(1)

# Set initial position of lever/nose sensors
last_arena_pos = (adc1.readChannel(lever_ch1),adc1.readChannel(nose_ch) )



while True :
    
    currentTime = time.time()
    success = 0
    
    leverL = adc1.readChannel(lever_ch1)
    nose = adc1.readChannel(nose_ch)
    

    # Check for successful trial
    if (LED_state == False and nose < 100):
        success = 1 # interesting for the data recording
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
                            str(lever_obj.get_threshold()) + "," +
                            str(lever_obj.get_threshold_error()) + "," +
                            str(lever_obj.get_h_pos()) + "," +
                            str(lever_obj.get_v_pos()) + "," +
                            str(success)+ "," +                         
                            str(ITI_time)+ "\n")                            
            #print "Lever: %i\tNose: %i" % last_arena_pos
            data_file.close()

    time.sleep(0.016)

LED_off() #end of the step
