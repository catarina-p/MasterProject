import os
import csv
import sys
import numpy as np

from PID_Gains import *

# from yaml import load
sys.path.append('libraries/')

# Dynamixel libraries
from libraries.dynamixel_controller import dynamixel
from libraries.dynamixel_address_book import *
from libraries.comms_wrapper import Arduino
from libraries.keyboard import Key

# from time import time, sleep
# import socket
# from threading import Thread
from datetime import datetime

class Harvest():
    def __init__(self, experimentID):
        self.experimentID         = experimentID
        # Gripper positions and max velocity
        self.open                 = 5000
        self.mid_pos              = 4200
        self.close                = 3600
        self.max_vel              = 300
        ## loadcell memory
        self.previous_compression = 0
        self.previous_pulling     = 0
        # PID parameters
        self.previous_error       = 0
        self.error_sum            = 0
        self.Kp                   = 0
        self.Ki                   = 0
        self.Kd                   = 0
        # Off stem detection
        self.check_stem           = True
        self.off_stem_thresh      = 4 
        self.pulling_difs         = []
        self.previous_force       = 0
        # Threshold for TOF sensor
        self.approach_thresh      = 50
        self.TOF                  = False
        # Speed to open the gripper"s fingers
        self.approaching_speed    = 20
        # For real raspberry picking
        self.pulling_force_range  = [F1LT, F1MT, F1HT]
        self.setpoint             = 0
        self.pulling_force        = [(F1LT, self.setpoint)]
        self.all_pulling_forces   = []
        self.setpoint_thresh      = 20
        # Directory to save data
        # self.directory            = '../data/Real raspberry trials'
        self.directory            = 'D:/IST/5 ano/2 semestre/Tese/raspberry-grasping/data/Real raspberry trials'
        # Contast for setting graping force method 3
        self.b                    = 0.5

        # Dynamixel
        self.gripper = dynamixel(ID=112, descriptive_device_name="Gripper", series_name="xm", baudrate=1000000, port_name="COM4")
        self.gripper.begin_communication()
        self.gripper.set_operating_mode("velocity")

        # Gripper sensor: loadcells and TOF sensor
        self.loadcell = Arduino(descriptiveDeviceName="loadcell arduino", portName="COM5", baudrate=115200)
        self.loadcell.connect_and_handshake()

    def approach_raspberry(self):
        """
        This function recieves the TOF sensor's data, and signals when the raspberry is 
        in position between the gripper's fingers.
        """
        self.loadcell.send_message("read_d")
        self.loadcell.receive_message()
        if self.loadcell.newMsgReceived:
            dist = self.loadcell.receivedMessages['d']
            print(dist)
            if dist != 'null':
                dist = np.abs(float(self.loadcell.receivedMessages['d']))
                if dist <= self.approach_thresh: 
                    return True
                else: 
                    return False
            else:
                return False
        else:
            return False

    def set_controller(self, Kp, Ki, Kd):
        """
        Define controller gains.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.error_sum      = 0

    def reset(self):
        """
        Reset for next iteration.
        """
        self.previous_force = 0
        self.check_stem = True

    def is_off_stem_gripper(self):
        """
        This function signals when the raspberry is off the plant by analysing the data 
        from the gripper's load cell.
        """
        force = self.get_gripper_vertical_force()
        new_dif = self.previous_force - force
        if len(self.pulling_difs) > 4:
            pull_array = np.array(self.pulling_difs)
            pull_std = np.std(pull_array)
            # if new_dif > 1:
                # print('std', pull_std)
                # print('new dif', new_dif)
                # print('thresh*std', self.off_stem_thresh*pull_std)
            if new_dif > self.off_stem_thresh*pull_std and new_dif > 20:
                # print('True \n\n\n')
                return True
            else:
                self.pulling_difs.append(new_dif)
                self.previous_force = force
                # print('False \n\n\n')
                return False
        else:
            self.pulling_difs.append(new_dif)
            self.previous_force = force
            # print('False \n\n\n')
            return False

    def is_in_position(self, position):
        """
        Checks the position of the gripper's fingers.
        """
        if abs(self.gripper.read_position() - position) <= 5:
            return True
        else:
            return False

    def move_fingers_to_position(self, position):
        """
        This functions moves the gripper's finger's to the chosen position.
        """
        current_pos = self.gripper.read_position()
        if current_pos < position:
            signal = 1
        elif current_pos > position:
            signal = -1
        while not self.is_in_position(position):
            self.move_gripper(self.approaching_speed*signal)
        self.move_gripper(0)

    def PID_gripper(self, error):
        """ 
        PID controller for the gripper 
        """
        self.error_sum += error
        error_dif = error - self.previous_error
        self.previous_error = error

        return self.Kp*error + self.Ki*self.error_sum - self.Kd*error_dif

    def get_compression_force(self):
        """
        This function gets the compressive force value read from the gripper's load cell.
        """
        flag = False
        while not flag:
            self.loadcell.receive_message()
            flag = self.loadcell.newMsgReceived
            if self.loadcell.newMsgReceived:
                force = np.abs(float(self.loadcell.receivedMessages['lc2']))
                self.previous_compression = force
                # print('Received',force)
                break
            else:
                force = self.previous_compression
        return force

    def get_gripper_vertical_force(self):
        """
        This function gets the vertical pulling force value read from the gripper's 
        load cell.
        """
        flag = False
        while not flag:
            self.loadcell.receive_message()
            flag = self.loadcell.newMsgReceived
            if self.loadcell.newMsgReceived:
                force = np.abs(float(self.loadcell.receivedMessages['lc1']))
                self.previous_pulling = force
                break
            else:
                force = self.previous_pulling
        return force

    def control_gripper(self, F_d, is_off_stem, force_list):
        """
        This function applies the control action.
        """
        compression_force_temp = 0
        while compression_force_temp == 0:
            compression_force_temp = self.get_compression_force()
        
        # if len(force_list) > 0:
        #     while compression_force_temp-force_list[-1] > 100:
        #         compression_force_temp = force_list[-1] + 100
        #         break
        #         print("abnormal force!!", compression_force_temp-force_list[-1])
        #         compression_force_temp = self.get_compression_force()

        compression_force = compression_force_temp

        error = compression_force - F_d
        if is_off_stem and self.check_stem:
            self.set_controller(K2[0], K2[1], K2[2])
            self.check_stem = False
            return error, compression_force, 0
    
        action = self.PID_gripper(error)
        if action > self.max_vel:
            action = self.max_vel
        if action < 0 and self.is_in_position(self.close):
            action = 0
        if self.is_in_position(self.open) and action > 0:
            action = 0
        self.move_gripper(action)
        return error, compression_force, action

    def check_raspberry(self):
        """
        After harvest check if the raspberry was harvested as intended.
        """
        verify = []
        verify.append(self.get_compression_force())
        while self.gripper.read_position() > 0:
            self.move_gripper(-10)
            verify.append(self.get_compression_force())
        self.move_gripper(0)
        verify.append(self.get_compression_force())
        return verify

    def move_gripper(self, action):
        """
        This function defines the speed at which the gripper's fingers are moving.
        """
        self.gripper.write_velocity(action)

    def set_grasping_force(self, method):
        """
        This function defines the Force setpoint after slip has been detected.
        There are 3 methods to do this. This function is not meant for training.
        """
        main_dir = os.getcwd()
        os.chdir(self.directory)
        if os.path.isdir('Harvesting data/Controller_force_setpoint') is True:
            if method == 0:   
                Fd = self.pulling_force[-1][0]
            
            elif method == 1:
                filename = 'Harvesting data/Controller_force_setpoint/'+os.listdir('Harvesting data/Controller_force_setpoint')[-1]
                Fd = (self.pulling_force_range[0] + self.read_csv_file(filename)[-1][0])/2
                self.setpoint = self.read_csv_file(filename)[-1][1]
                self.pulling_force = [(Fd, self.setpoint)]

            elif method == 2:
                f_success = 0 
                i = 0 
                setpoints = 0
                for file in os.listdir('Harvesting data/Controller_force_setpoint'):
                    file_list = self.read_csv_file(file)
                    f_success += file_list[-1][0]
                    setpoints += file_list[-1][1]
                    i += 1
                Fd = (self.pulling_force_range[0] + f_success)/(i + 1)
                self.setpoint = int(np.floor(setpoints/i))
                self.pulling_force = [(Fd, self.setpoint)]

            elif method == 3:
                f_success = 0 
                i = 0 
                setpoints = 0
                for file in os.listdir('Harvesting data/Controller_force_setpoint'):
                    file_list = self.read_csv_file(file)
                    f_success += file_list[-1][0]
                    setpoints += file_list[-1][1]
                    i += 1
                Fd = self.pulling_force_range[0]*self.b**i + (f_success/i)**(1-self.b**i)
                self.setpoint = int(np.floor(setpoints/i))
                self.pulling_force = [(Fd, self.setpoint)]
        else:
            Fd = self.pulling_force[-1][0]
            
        os.chdir(main_dir)
        return Fd

    # For real raspberry picking
    def change_grasping_force(self, is_off_stem):
        """
        Recording the force data.
        """
        if not is_off_stem:
            self.setpoint += 1
            if self.setpoint < len(self.pulling_force_range):
                force = self.pulling_force_range[self.setpoint]
            else:
                force = self.pulling_force[-1][0]*1.5
            self.pulling_force.append((force, self.setpoint))
        else:
            self.save_csv_data(self.pulling_force, 'controller_force_setpoint'+str(self.experimentID))

    def save_csv_data(self, data, file_name):
        """
        This function saves the harvesting data in a csv file.
        """
        main_dir = os.getcwd()
        os.chdir(self.directory)
        if os.path.isdir('Harvesting data day 2') is False:
            os.makedirs('Harvesting data day 2')
        os.chdir('Harvesting data day 2')
        # Writing coordinates to csv file
        if os.path.isdir(file_name) is False:
            os.makedirs(file_name)
        os.chdir(file_name)
        data_str = []
        with open(file_name+str(self.experimentID)+'.csv', 'w') as csvfile:
            # creating a csv writer object
            csvwriter = csv.writer(csvfile)
            
            # writing the data rows
            data_str.append(str(x) for x in data)
            csvwriter.writerows(data_str)
        os.chdir(main_dir)

    def read_csv_file(self, filename):
        """
        This function reads csv files.
        """
        # reading csv file
        with open(filename, newline='') as f:
            reader = csv.reader(f)
            data = list(reader)
        return data




