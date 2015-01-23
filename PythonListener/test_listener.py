###############################################################################
###
### test_listener.py - a simple server which listens for UDP data on a port
###                    and then prints all data recieved to the console 
###                    until forced to quit
###
###
### For FRC Team 1736 RobotCasserole - 2015 Build Season
###
###
###  Revision History:
###  1/21/2015 - Chris Gerth - Created 
###
###
###############################################################################

#!/usr/bin/env python

import socket
import sys
import os


UDP_IP = "10.17.36.199" #see http://wpilib.screenstepslive.com/s/4485/m/13503/l/242608-roborio-networking
                        #driver station is techincally assigned over DHCP between 10.17.36.20 and 199.
                        #Having no network design experience, I choose 199 and hopefully some magic happens.
UDP_PORT = 5801 #See Pg. 16 0f game manual - ports 5800 through 5810 are available for team use.
bufferSize = 32

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

print("Listening for data on UDP port "+ str(UDP_PORT)+". Press ctrl-c to terminate.")
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(bufferSize) 
    print(data)
