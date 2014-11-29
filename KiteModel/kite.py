# -*- coding: utf8 -*-
import cmath
import math
from Tkinter import *

def load_naca(filename):
    '''Loads naca profile from file
    Files from http://airfoiltools.com/airfoil/naca4digit'''
    xy = []
    f = file(filename)
    lines = f.readlines()
    scaleFactor = 200
    for line in lines[1:]:
        # Reference point at quarter of chord
        xy.append((scaleFactor*(float(line.split()[0])-1/4.0), -scaleFactor*float(line.split()[1])))
    return xy


def rotate(xy, angle, center):
    '''Rotates the coordinates'''
    newxy = []
    offset = complex(center[0], center[1])
    cangle = cmath.exp(angle*1j)
    for x, y in xy:
        v = cangle * (complex(x, y) - offset) + offset
        newxy.append((v.real, v.imag))
    return newxy
    
def translate(xy, vector):
    '''Translates the coordinates'''
    newxy = []
    for x, y in xy:
        newxy.append( (x+vector[0], y+vector[1]))
    return newxy
        
def move():
    "Animates"
    
    # Get the gui slider position
    kite_azimuth = wx.get()/100.0 * math.pi -math.pi/2
    kite_angle = w.get()/100.0 * math.pi -math.pi/2
    
    # Computed the updated data
    xy_k = rotate([(0, -lines_length)], kite_azimuth, (0, 0))
    x_kite = x_cart + xy_k[0][0]
    y_kite = y_cart + xy_k[0][1]
  
    xy =  translate(rotate(xy_kite, kite_angle, (0, 0)), (x_kite, y_kite))
    xy = [item for sublist in xy for item in sublist]
    
    # Modify the display
    can1.coords(kite, *xy)
    can1.coords(lines, x_cart, y_cart, x_kite, y_kite)
    
    # Ensure the animation
    win1.after(50, move)
    # => loop after 50 millisecondes
    
        
#========== Main =============

# Creates parent widget:
win1 = Tk()
win1.title("Kite and cart 2D animation")
# Creates children widgets:
can1 = Canvas(win1, bg='white',height=600, width=1000)
can1.pack(side=LEFT, padx=5, pady=5)

# Cart
height_cart = 30
width_cart = 100
x_cart = 500
y_cart = 500
x1 = - width_cart/2.0
y1 = - height_cart/2.0
x2 = + width_cart/2.0
y2 = - height_cart/2.0
x3 = + width_cart/2.0
y3 = + height_cart/2.0
x4 = - width_cart/2.0
y4 = + height_cart/2.0
xy_cart = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
cart_angle = 0
cart = can1.create_polygon(translate(rotate(xy_cart, cart_angle, (0, 0)), (x_cart, y_cart)), width=2, fill='brown')
radius = 10

# Wheels
can1.create_oval(x_cart-width_cart/4.0-radius, y_cart+height_cart/2.0, x_cart-width_cart/4.0+radius, y_cart+height_cart/2.0 + 2*radius)
can1.create_oval(x_cart+width_cart/4.0-radius, y_cart+height_cart/2.0, x_cart+width_cart/4.0+radius, y_cart+height_cart/2.0 + 2*radius)

# Kite
xy = load_naca("naca2412.dat")
width_kite = 200
height_kite = 20
lines_length = 400
x1 = - width_kite/2.0
y1 = - height_kite/2.0
x2 = + width_kite/2.0
y2 = - height_kite/2.0
x3 = + width_kite/2.0
y3 = + height_kite/2.0
x4 = - width_kite/2.0
y4 = + height_kite/2.0
xy_kite = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
xy_kite = xy
kite_azimuth = -math.pi/5
xy_k = rotate([(0, -lines_length)], kite_azimuth, (0, 0))
x_kite = x_cart + xy_k[0][0]
y_kite = y_cart + xy_k[0][1]
kite_angle =-math.pi/10
kite = can1.create_polygon(translate(rotate(xy_kite, kite_angle, (0, 0)), (x_kite, y_kite)), width=2, fill='blue')

# Lines
lines = can1.create_line(x_cart, y_cart, x_kite, y_kite, width = 3)
#force = can1.create_line(x_kite, y_kite, x_kite-30, y_kite-30, width = 3, arrow=LAST)

# UI controls
but1 = Button(win1, text='Quit', width=8, command=win1.quit)
but1.pack(side=BOTTOM)

w = Scale(win1, from_=0, to=100)
w.pack()
wx = Scale(win1, from_=0, to=100)
wx.pack()

# Launch animation
move()
# Launch the main_loop:
win1.mainloop()
