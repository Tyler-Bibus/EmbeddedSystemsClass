# Author: Phillip Jones
# Date: 08/19/2022, updated 05/25/2023, updated 07/13/2023
# Description: Simple GUI client for Cybot

import tkinter
from tkinter import *

import numpy as np
import matplotlib.pyplot as plt

# Serial library:  https://pyserial.readthedocs.io/en/latest/shortintro.html 
import serial
import time # Time library   
# Socket library:  https://realpython.com/python-sockets/  
# See: Background, Socket API Overview, and TCP Sockets  
import socket   

def get_graph():
    distance = []
    print("Graphing")
    rx_message = "e"
    angle_degrees = []
    angle_radians = []
    while (rx_message != "start\n".encode()):
        rx_message = cybot.readline()
    rx_message = cybot.readline()
    while (rx_message != "done\n".encode()):
        rx_message = cybot.readline()
        rx_message = rx_message.decode()
        data = rx_message.split()
        print(data)
        if (data[0] == '180'):
            break
        distance.append(float(data[1]))
        angle_degrees.append(float(data[0]))
        
    
    angle_degrees = np.array(angle_degrees)
    angle_radians = (np.pi/180) * angle_degrees
    # Create a polar plot
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}) # One subplot of type polar
    ax.plot(angle_radians, distance, color='r', linewidth=4.0)  # Plot distance verse angle (in radians), using red, line width 4.0
    ax.set_xlabel('Distance (m)', fontsize = 14.0)  # Label x axis
    ax.set_ylabel('Angle (degrees)', fontsize = 14.0) # Label y axis
    ax.xaxis.set_label_coords(0.5, 0.15) # Modify location of x axis label (Typically do not need or want this)
    ax.tick_params(axis='both', which='major', labelsize=12) # set font size of tick labels
    ax.set_rmax(1.0)                    # Saturate distance at 2.5 meters
    ax.set_rticks([0.25, 0.5, 0.75, 1])   # Set plot "distance" tick marks at .5, 1, 1.5, 2, and 2.5 meters
    ax.set_rlabel_position(-22.5)     # Adjust location of the radial labels
    ax.set_thetamax(180)              # Saturate angle to 180 degrees
    ax.set_xticks(np.arange(0,np.pi+.1,np.pi/4)) # Set plot "angle" tick marks to pi/4 radians (i.e., displayed at 45 degree) increments
                                                 # Note: added .1 to pi to go just beyond pi (i.e., 180 degrees) so that 180 degrees is displayed
    ax.grid(True)                     # Show grid lines
    ax.set_title("Plot", size=14, y=1.0, pad=-24) 
    plt.show()  # Display plot

def send_command(character):
    command = character
    cybot.write(command.encode())
    if character == "m":
        get_graph()
    elif character == "q":
        print(cybot.readline().decode())

# TCP Socket BEGIN (See Echo Client example): https://realpython.com/python-sockets/#echo-client-and-server
#HOST = "127.0.0.1"
#PORT = 65432
HOST = "192.168.1.1"  # The server's hostname or IP address
PORT = 288        # The port used by the server
cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a socket object
cybot_socket.connect((HOST, PORT))   # Connect to the socket  (Note: Server must first be running)

cybot = cybot_socket.makefile("rbw", buffering=0) 

command = "e"

window = tkinter.Tk()

mainLabel = Label(window, width = 15 ,text = "Basic CyBot GUI!")
mainLabel.grid(row = 0, column = 1)

forward = Button(window, text = "Forward", command = lambda: send_command("w"))
backward = Button(window,text = "Backward", command = lambda: send_command("s"))
left = Button(window,text = "Left", command = lambda: send_command("a"))
right = Button(window,text = "Right", command = lambda: send_command("d"))
manualScan = Button(window, text = "Man-Scan", command = lambda: send_command("m"))
ping = Button(window,text = "Ping", command = lambda: send_command("q"))
toggle = Button(window, text = "Toggle", command = lambda: send_command("t"))
autonCommand = Button(window, text = "Auto", command = lambda: send_command("h"))

forward.grid(row = 1, column = 1)
backward.grid(row = 3, column = 1)
left.grid(row = 2, column = 0)
right.grid(row = 2, column = 2)
ping.grid(row = 2, column = 1)
manualScan.grid(row = 4, column = 0)
toggle.grid(row = 4, column = 2)
autonCommand.grid(row = 5, column = 1)


window.mainloop()
