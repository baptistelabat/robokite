from pymouse import PyMouse
import time
# This script demonstrates the possibility to use a mouse as an unbound sensor.
# To do that the cursor position is brought back to the middle of the screen at each step, and the distance moved by the mouse are integrated
# This script is intended to be used with a second external mouse.
# This can be achieved using MPX, and inverting the laptop mouse and externa mouse.
# Warning: it may be difficult to stop this script with the mouse, so be sure you are able to do it with keyboard command!
cursor = PyMouse()
x = 0
y = 0
screen_size = cursor.screen_size()
cursor.move(screen_size[0]/2, screen_size[1]/2)
while True:
  p = cursor.position()
  x = x + p[0] - screen_size[0]/2
  y = y + p[1] - screen_size[1]/2
  print x, y, p[0] - s[0]/2, p[1] - s[1]/2
  cursor.move(screen_size[0]/2, screen_size[1]/	2)
  time.sleep(0.01)
  
