from libraries.comms_wrapper import Arduino

# fluidic sensor libraries
from fluidic_sensor.fluidic_sensor_class import Fluidic_Sensor
from fluidic_sensor.utility import *
from fluidic_sensor.config import *

# Online libraries
import socket
from time import time
import json
from sklearn.linear_model import LinearRegression
import numpy as np
from evaluation import Evaluate_Pick


def main():

    file = open('human trial5.txt', 'w+')

    file.write('Human trial picking the raspberry twin \n\n time    ||     output     ||     loadcell\n\n')

    closed = False

    # Define what data to send to plotjuggler
    newData = {
        "zero":0
    }
    plot_juggler_port = 9871
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
    
    eval = Evaluate_Pick()

    eval.take_offset()
    
    seconds = []

    zeroTime = time()
    printTimer = time()
    while 1:     
        t = time() - zeroTime
        seconds.append(t)

        if len(seconds) == 400:
            break

        output = eval.get_data()
        #output = 1
        force = eval.get_vertical_force()
        
        if not closed:
            file.write('      %f' % t+'       %f' % output+'       %f \n' % force)

        newData["output"]               = output 

        # Fill in other signals
        newData["force"]                = force
        # Send to plot juggler
        sock.sendto( json.dumps(newData).encode(), ("127.0.0.1", plot_juggler_port) )    

        #print(s1.offset_adjustment)
        if time() - printTimer > 1:
            print(int(t), "seconds since start of loop")
            printTimer = time()

        if int(t) > 20 and not closed:
            file.close()
            closed = True
            print('closed')


if __name__ == '__main__':
    main()