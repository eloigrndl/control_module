import socket
import sys
import pickle
import time
import tcplib
import mpc_robot
import input_processing
import math as mt

PATH_PLANNING_PORT = 10050
MOTION_EST_PORT = 10051



IP_ADDR = 'localhost'
BUFF_SIZE = 2**5    # has to be a power of 2
MY_MODULE_NAME = 'mpc_module' # Please enter here an arbitrary name for your code, which will help in logging

print('INFO:', MY_MODULE_NAME, 'starting.')

######################################################
#   MPC MODULE
######################################################
mpc_module = mpc_robot.MPC_robot()

# Create a TCP/IP socket for the input
sock_path = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in = (IP_ADDR, PATH_PLANNING_PORT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in))
sock_path.connect(server_address_in)

sock_motion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in = (IP_ADDR, MOTION_EST_PORT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in))
sock_motion.connect(server_address_in)

"""
# Create a TCP/IP socket for the output
sock_output = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_out = (IP_ADDR, VEHICLE_PORT)

print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_address_out))
sock_output.connect(server_address_out)

"""

module_running = True

try:
    iter = 0
    while module_running:

        ######################################################
        # WHEN YOU ARE READY, START READING INCOMING DATA
        # TO PROCESS IT. THIS FUNCTION CALL IS BLOCKING,
        # IT WILL WAIT UNTIL THERE'S DATA. THE DATA WILL 
        # BE WAITING AND PILE UP UNTIL YOU READ IT.
        ######################################################

        # On the first iteration, it is mandatory to receive a path and a state
        # Otherwise, I can't compute anything since I have no data
        new_state = tcplib.receiveData(sock_motion)
        new_path = mpc_robot.local_to_gloabl(tcplib.receiveData(sock_path), new_state)


        #X,Y,Head., Velocity, Acceleration, Yaw Rate

        new_state = (new_state[0], new_state[1], new_state[5])
        previous_state = (new_state[3], new_state[5])

        mpc_module.acquire_path(new_path)
        mpc_module.set_state(new_state)
        mpc_module.previous_state = previous_state



        

        iter += 1

finally:
    sock_path.close()
    sock_motion.close()