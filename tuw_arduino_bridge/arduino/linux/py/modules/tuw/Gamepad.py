import pygame
import time 
import numpy as np

class Gamepad:
    
    def __init__(self, joystick_id = 0):
        pygame.init()
        # Initialize the joysticks
        pygame.joystick.init()
          
        # Loop until the user clicks the close button.
        self.done = False
        
        # Check joystick
        self.joystick_count = pygame.joystick.get_count()
        if self.joystick_count == 0:
            print "no joystick connected"
            self.done = True
        if self.joystick_count <= joystick_id:
            print "wrong joystick id, only "+str(self.joystick_count)+" connected!"
            self.done = True
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
             
        self.axes = self.joystick.get_numaxes() 
        self.axis = np.zeros((self.axes,), dtype=np.float)
    
        self.buttons = self.joystick.get_numbuttons()
        self.button = np.zeros((self.buttons,), dtype=np.float)
    
        self.hats = self.joystick.get_numhats() 
        for i in range(self.hats):
            self.hat = self.joystick.get_hat(i)
        
        print self.joystick.get_name()

        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                done=True # Flag that we are done so we exit this loop
            
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")
        
    def read(self):  
        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                self.done=True # Flag that we are done so we exit this loop
            
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")
            
        for i in range(self.axes):            
            self.axis[i] = self.joystick.get_axis(i)
        for i in range(self.hats):
            self.hat = self.joystick.get_hat(i)
        for i in range(self.buttons):
            self.button[i] = self.joystick.get_button(i)
        
        
               