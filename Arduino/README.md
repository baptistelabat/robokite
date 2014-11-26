Arduino sketches
========

This directory host the arduino sketches.
Some are just some tests to checks sensors, actuators or communication links.

Simple: load this program on an arduino connected to a Sabertooth card controlling motors.
It receives order via NMEA messages on serial port. 
See motorJoy.py to generate orders from an USB joystick.

MPU9150_AHRS: this is the program to load on the arduino board connected to the IMU onboard kite.
It is making the data fusion of accelerometer, gyro and magnetometer
The software was found to work with MPU6050 alone and with MPU9050

Tests program
ArduinoMAVLink: this is a test to send fake attitude and gps data from arduino to ground control station with MAVLink

Electronic_rule: this is the program used to read measurement from the diy linear encoder.

Other sketches are obsolete


