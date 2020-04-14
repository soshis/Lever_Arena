# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 16:09:34 2017

@author: Adrien Boissenin

@definiton: Function definition used in the different step
"""

import Training_Step3_Adrien

def give_reward():
    """Helper function to turn on water"""

    print "\tSuccess number %i. (%i total)" % (successes, tot_successes)
    pump.start(50)
    #GPIO.output(pin_speaker, True)  
    time.sleep(0.3)
    #GPIO.output(pin_speaker, False)
    pump.stop()

    
def LED_on(lever_pos):
    """ Turn the LED ON if the Lever is back in original position to indicate the start of the trials"""
    if lever_pos < 2400:
        GPIO.output(pin_LED, GPIO.HIGH)
        global LED_state
        LED_state = True
#    print('LED ON')

    
def LED_off():
    """ Turn the LED off to indicate the end of the trials"""
    GPIO.output(pin_LED, GPIO.LOW)
    global LED_state
    LED_state = False
#    print('LED OFF')
    
def Timeout():
    """ Time penality when trials pause"""
    time.sleep(timeout)
    print('TIMEOUT')
    
def ITI():
    """ Inter trials Interval"""
    time.sleep(ITI_time)
   # LED_on()

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
                            str(manual_lever_h) + "," +
                            str(manual_lever_v) + "," +
                            str(success)+ "," +
                            str(timeout)+ "," +                            
                            str(ITI_time)+ "\n")
            #print "Lever: %i\tNose: %i" % last_arena_pos
            data_file.close()

