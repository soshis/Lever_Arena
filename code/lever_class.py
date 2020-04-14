"""
lever_class.py

File to define the lever object
"""
import random
import numpy as np
import RPi.GPIO as GPIO
from servo import setActuator


class Lever():
    """Lever object for rat arena"""

    def __init__(self, pin_v_act, pin_h_act, h_positions, v_positions,
                 threshold, v_pos=0, h_pos=0, t_pos=0, verbose=False):
        """Initialize instance of Lever

        Parameters
        ----------
        pin_v_act: int
            GPIO pin for vertical actuator
        pin_h_act: int
            GPIO pin for horizontal actuator
        v_positions: list
            List of possibible vertical positions for lever
        h_positions: list
            List of possibible horizontal positions for lever
        threshold: int
            Lever threshold for success
        verbose: bool
            Whether or not to print out debugging messages

        Attributes
        ----------
        pin_v_act: int
            GPIO pin for vertical actuator
        pin_h_act: int
            GPIO pin for horizontal actuator
        h_positions: list
            List of possibible horizontal positions for lever
        v_positions: list
            List of possibible vertical positions for lever
        threshold: int
            Lever threshold for success
        v_pos: int
            Index (into v_positions) identifying current vertical position
        h_pos: int
            Index (into h_positions) identifying current horizontal position

        """

        # Assign instance variables
        self.pin_v_act = pin_v_act
        self.pin_h_act = pin_h_act
        self.h_positions = h_positions
        self.v_positions = v_positions
        self.threshold = threshold
        self.threshold_error = threshold - np.floor(np.linspace(50.,400.,len(threshold))) 
        self.verbose = verbose

        self.v_pos = v_pos
        self.h_pos = h_pos
        self.t_pos = t_pos

        # Initialize lever
        for temp_pin in [self.pin_v_act, self.pin_h_act]:
            GPIO.setup(temp_pin, GPIO.OUT)

    def advance_lever(self):
        """ Advance position of lever one step"""
        
        # Advance only h_pos if not at maximum h_pos
        if self.h_pos < len(self.h_positions) - 1:
            self.advance_h_pos()
        # Advance both v_pos and h_pos if at maximum h_pos
        else:
            self.h_pos = -1
            self.advance_h_pos()
            self.advance_v_pos()
        
        '''
        if self.verbose:
            print "Lever advancing to vertical: %i, horizontal: %i" % \
                (self.get_v_pos(), self.get_h_pos())
            '''

    def advance_v_pos(self, manual=False):
        """Advance vertical position one step"""
        # Update state of lever object
        self.v_pos += 1
        if self.v_pos >= len(self.v_positions):
            self.v_pos = 0
        # Actually change physical position of actuator
        setActuator(self.pin_v_act, self.v_positions[self.v_pos])
        if self.verbose:
            self.print_pos(manual)
   
    def advance_h_pos(self, manual=False):
        """Advance horizontal position one step"""
        # Update state of lever object
        self.h_pos += 1
        if self.h_pos == len(self.h_positions):
            self.h_pos -= 1
##            self.h_pos = 0
        # Actually change physical position of actuator
        setActuator(self.pin_h_act, self.h_positions[self.h_pos])
        if self.verbose:
            self.print_pos(manual)
            
    def advance_random(self):
        """Advance horizontal and vertical position randomlyt\  """
        # Update state of lever object
        self.h_pos = random.randint(0, len(self.h_positions)-1)
        self.v_pos = random.randint(0, len(self.v_positions)-1)        

        # Actually change physical position of actuator
        setActuator(self.pin_h_act, self.h_positions[self.h_pos])
        setActuator(self.pin_v_act, self.v_positions[self.v_pos])

    def reset_pos(self):
        """Reset vertical and horizontal position to zero"""
        self.v_pos = 0
        self.h_pos = 0
        setActuator(self.pin_v_act, self.v_positions[self.v_pos])
        setActuator(self.pin_h_act, self.h_positions[self.h_pos])

        if self.verbose:
            print "Lever positions reset."
            self.print_pos()

    def increase_threshold(self):
        """Advance threshold one step"""
        # Update state of lever object
        self.t_pos += 1
        if self.t_pos >= len(self.threshold):
            self.t_pos -= 1

        print " \tThreshold increase to: %i" % (self.get_threshold())


    def decrease_threshold(self):
        """Advance threshold one step"""
        # Update state of lever object
        self.t_pos -= 1
        
        if self.t_pos < 0 :
            if self.h_pos > 0:
                self.t_pos = len(self.threshold)-1
                self.h_pos = self.h_pos - 2 # reduce 2 plus advance 1 to reduce 1,
                self.advance_h_pos() # advance of 1 step
            else:
                self.t_pos +=1
            

        print " \tThreshold reduced to: %i" % (self.get_threshold())


    def reset_threshold(self):
        """Reset threshold iterator to zero"""
        self.t_pos = 0
       
            
    def get_threshold_error(self):
        """Helper set and get a wanted threshold for the error."""
        return self.threshold_error[self.t_pos]

    def get_threshold(self):
        """Helper set and get a wanted threshold for the error."""
        return self.threshold[self.t_pos]
    
    def get_v_pos(self):
        return self.v_positions[self.v_pos]

    def get_h_pos(self):
        return self.h_positions[self.h_pos]

    def set_threshold(self,ind_t):
        self.t_pos = ind_t
        print " \tThreshold updated to: %i" % (self.get_threshold())

    def set_threshold_error(self,new_thr_err):

        self.threshold_error = new_thr_err
        print(self.threshold_error)


        
    def set_v_pos(self,ind_v):
        self.v_pos = ind_v
        setActuator(self.pin_v_act, self.v_positions[self.v_pos])
        
    def set_h_pos(self,ind_h):
        self.h_pos = ind_h
        setActuator(self.pin_h_act, self.h_positions[self.h_pos])
        
    

    def print_pos(self, manual=False):
        """Helper to print out the current position"""
        if manual:
            print 'Row: %s, Position: %s; Manual Advance' % (self.v_pos, self.h_pos)
        else:
            print '********************   Row: %s, Position: %s' % (self.v_pos, self.h_pos)


