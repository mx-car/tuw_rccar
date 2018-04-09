"""
Sample Python/Pygame Programs
Simpson College Computer Science
http://programarcadegames.com/
http://simpson.edu/computer-science/
 
Show everything we can pull off the joystick
"""
import pygame
import time 
import numpy as np
 
# Initialize the joysticks
pygame.joystick.init()
  
# Loop until the user clicks the close button.
done = False

# Check joystick
joystick_count = pygame.joystick.get_count()
if pygame.joystick.get_count() == 0:
    print "no joystick connected"
    done = True

 
pygame.init()
#name = joystick.get_name()
#print name

# -------- Main Program Loop -----------
while not done:
     # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    axes = joystick.get_numaxes() 
    axis = np.zeros((axes,), dtype=np.float)
    for i in range(axes):            
        v = joystick.get_axis(i)
        axis[i] = v
        
    hats = joystick.get_numhats() 
    #hat = np.zeros((hats,), dtype=np.float)
    for i in range(hats):
        v = joystick.get_hat(i)
        hat = v
        
    buttons = joystick.get_numbuttons()
    button = np.zeros((buttons,), dtype=np.float)
    for i in range(buttons):
        v = joystick.get_button(i)
        button[i] = v
        
        
    print "axis: " + str(axis)
    print "hat: " + str(hat)
    print "button: " + str(button)
    time.sleep(0.5)