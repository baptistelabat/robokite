# -*- coding: utf8 -*-
import pygame
from pygame.locals import *

pygame.init()
fenetre = pygame.display.set_mode((300,300))

pygame.joystick.init()
nb_joysticks = pygame.joystick.get_count()
print("Il y a", nb_joysticks, "joystick(s) branché(s)")
mon_joystick = pygame.joystick.Joystick(0)
mon_joystick.init() #Initialisation

print("Axes :", mon_joystick.get_numaxes())
print("Boutons :", mon_joystick.get_numbuttons())
print("Trackballs :", mon_joystick.get_numballs())
print("Hats :", mon_joystick.get_numhats())


continuer = 1
while continuer:
  for event in pygame.event.get():
    if event.type == JOYBUTTONDOWN:
        print "button"
        print event.button == 0
    if event.type == JOYAXISMOTION:
      if event.axis == 2:
        print "direction control ", event.value
      if event.axis == 3:
        print "power control ", event.value
