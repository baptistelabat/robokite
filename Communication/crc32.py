#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT, 
#
# This file is an example of the use of the Cyclic redundancy check
import serial
import time
import binascii

def openSerial():
	global ser
	# Open the serial port
	locations=['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
	for device in locations:
  	  try:
    	    print "Trying...",device
            ser = serial.Serial(device, 9600)
            print "Connected on ", device
            break
  	  except:
    	    print "Failed to connect on ", device
            time.sleep(1.5) # Arduino is reset when opening port so wait before communicating

def add_crc32(msg):
''' This function adds a crc32 key to a message before sending it over a connection with loss
    It can then be used to check the validity of the message
Input:
  msg: string
Return:
  modified string'''
  
  msg_ascii = msg
  crc_hex = hex(binascii.crc32(msg_ascii) & 0xffffffff)
  crc_ascii = binascii.unhexlify(crc_hex[2:])
  return msg_ascii+crc_ascii

openSerial()
print add_crc32("bonjour")
ser.write(add_crc32("bonjour"))
