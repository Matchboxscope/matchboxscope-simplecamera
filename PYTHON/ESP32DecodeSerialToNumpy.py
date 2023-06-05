#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun  4 22:35:28 2023

@author: bene
"""
import numpy as np
import serial
import matplotlib.pyplot as plt
H=240/2
W=320/2

# Open the serial connection
ser = serial.Serial('/dev/cu.usbmodem1101', 100000)  # Adjust the port and baud rate accordingly

# %%
while 1:
    # Receive the byte array
    byte_array_length = H * W * 2  # Size of uint16_t is 2 bytes
    try:
        received_data = ser.read(int(byte_array_length*2))
        frames = received_data.split(b'###NEWFRAME###')[1]
        # Convert the byte array to a 2D NumPy array
        np_array = np.frombuffer(frames, dtype=np.uint16)[0:int(H*W)]
        np_array = np_array.reshape((int(H), int(W)))
        
        plt.imshow(np_array), plt.show()
    except:
        pass
    
#%%
# Create an empty 2D NumPy array to store the frames
frames = []

# Read frames continuously
while True:
    # Read until a line break is encountered
    line = ser.readline().decode().strip()
    
    # Check if the line break is encountered
    if line == '':
        # Convert the received frames to a 2D NumPy array
        np_array = np.array(frames, dtype=np.uint16)
        np_array = np_array.reshape((-1, W))  # Reshape to the desired dimensions
        
        # Process the received frames
        # ...

        # Clear the frames list
        frames = []
    else:
        # Split the line by spaces to get individual values
        values = line.split()
        
        # Convert the values to integers and add them to the frames list
        frame = [int(value) for value in values]
        frames.append(frame)
        
#%%
while(1):
    # Read the data from serial
    data = ser.read_until(b'---').decode().strip()
    
    # Remove the leading '+++' and trailing '---' markers
    data = data.strip('+++').strip('---')
    
    # Split the data into rows
    rows = data.split("\r\n")[1:-1] #data.split(',')
    
    # Initialize an empty 4x4 NumPy array
    matrix = np.empty((int(H), int(W)), dtype=np.int16)
    
    # Fill the matrix with the parsed values
    for i, row in enumerate(rows):
        elements = row.split(',')[0:-1]
        matrix[i] = [int(element) for element in elements]
    
    # Print the resulting matrix
    plt.imshow(matrix), plt.show()