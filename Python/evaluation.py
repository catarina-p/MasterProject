import csv
from time import time
from scipy.cluster.vq import kmeans2, whiten, cdist

from libraries.comms_wrapper import Arduino

# fluidic sensor libraries
from fluidic_sensor.fluidic_sensor_class import Fluidic_Sensor
from fluidic_sensor.utility import *
from fluidic_sensor.config import *

import json
import socket



class Evaluate_Pick():
    """
    This class compares the picking reference data with the robotic picking trial data.
    """
    def __init__(self) -> None:
        self.previous_force          = 0
        self.previous_verticalForce  = 0
        self.off_stem_thresh         = 70#100

        self.raspberrySensors = Arduino(descriptiveDeviceName="Pressure and pulling sensor arduino", portName="COM6", baudrate=115200)
        
        self.raspberrySensors.connect_and_handshake()

        # offset = find_offset(self.raspberrySensors, ["pressure1"])[0]
        # print("The baseline threshold is: ", offset)
        self.s1 = 0

        self.zeroTime = time()
        self.raspberrySensors.receive_message(printOutput=False)

        # Plotjuggler
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
        self.plot_juggler_port = 9871   
        # Define what data to send to plotjuggler
        self.newData = {
            "zero":0
        }
    
    def take_offset(self):
        """
        Take offset from pressure sensor measurements.
        """
        offset = find_offset(self.raspberrySensors, ["pressure1"])[0]
        print("The baseline threshold is: ", offset)
        self.s1 = Fluidic_Sensor(offset)

    def reset(self):
        """
        Reset values that keep track of the vertical force values.
        """
        self.previous_force          = 0
        self.previous_verticalForce  = 0

    def get_vertical_force(self):
        """
        Get vertical force from the rasberry twin's load cell.
        """
        self.raspberrySensors.receive_message()
        if self.raspberrySensors.newMsgReceived:
            force = np.abs(float(self.raspberrySensors.receivedMessages['lc3']))
            self.previous_verticalForce = force
            return force
        else:
            return self.previous_verticalForce

    def is_off_stem(self):
        """
        Check if the raspberry is off the plant.
        """
        force = self.get_vertical_force()
        if abs(self.previous_force - force) > self.off_stem_thresh:
            # self.previous_force = force
            return True
        else:
            self.previous_force = force
            return False

    def get_data(self):    
        """
        Get pressure reading from the physical twin.
        """ 
        stop = False  
        while not stop:
            self.raspberrySensors.receive_message(printOutput=False) 
            if self.raspberrySensors.newMsgReceived:  
                t = time() - self.zeroTime
                raw = float(self.raspberrySensors.receivedMessages["pressure1"])

                # Perform data processing and auto drift compensation
                #s1.auto_drift_compensation(raw, t)
                self.s1.smooth_signal(raw, t)
                output, _ = self.s1.get_processed_signal()
                stop = True
                # output = raw
                return output

    def get_reference(self, filename):
        """
        Read reference values from csv file.
        """
        harvest_ref = []
        rows = []
        # reading csv file
        with open(filename, 'r') as csvfile:
            # creating a csv reader object
            csvreader = csv.reader(csvfile)
            # extracting each data row one by one
            for row in csvreader:
                rows.append(row)
        harvest_ref = [float(x) for x in rows[0]]
        return np.array(harvest_ref)

    # through standard deviation
    def clip_outliers(self, mean_vals, std_limit):
        """ 
        Clip outliers bigger than a certain standard deviation limit (std_limit).
        """
        ## NOTE USE IN CASE WE COMPARE A VECTOR OF THE HIGHEST VALUES ##
        # mean_vals = np.mean(np.array(n_trials), axis=1) 
        std = np.std(mean_vals)
        mean = np.mean(mean_vals)
        mean_vals[mean_vals > mean + std*std_limit] = mean + std*std_limit
        mean_vals[mean_vals < mean - std*std_limit] = mean - std*std_limit
        return mean_vals  

    def cost_function(self, y, y_hat):
        """
        Get error between the reference pick and the robotic pick.
        """
        dif = y-y_hat
        relative_error = dif/y
        return dif, relative_error#,MSE

    def plot_data(self, compression_force, F_d, rasp_output, pulling_force, pulling_gripper, t):
        """
        Plot data in PlotJuggler.
        """
        self.newData["gripping force"]           = compression_force
        self.newData["desired gripping force"]   = F_d
        self.newData["raspberry"]                = rasp_output
        self.newData["pulling force"]            = pulling_force
        self.newData["pulling gripper"]          = pulling_gripper
        self.newData['timestamp']                = t

        # Send to plot juggler            
        self.sock.sendto(json.dumps(self.newData).encode(), ("127.0.0.1", self.plot_juggler_port))  

   
   
    