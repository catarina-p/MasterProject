import json
import socket
from threading import Thread
from time import sleep, time

import cv2 as cv
import depthai as dai
import numpy as np

from harvest import Harvest
from PID_Gains import *
from reset_robot import Reset_Robot
from vision import Vision

experimentID = input('\n\n\nID number for current experiment: ')
print('\n\n\n')

vis   = Vision(experimentID)
harv  = Harvest(experimentID)
reset = Reset_Robot()
    
# Plotjuggler communication
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
plot_juggler_port = 9871
newData = {"zero":0}

#----------------------------------------------------------------------------------------------------------------------------------------------------------
# Slip detection communication
#----------------------------------------------------------------------------------------------------------------------------------------------------------
localIP              = "10.15.20.195"
localPort            = 20001
bufferSize           = 1024

# Create a datagram socket
s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
s.bind((localIP, localPort))
msg = 'null'

def read_socket():
    global msg
    while 1:
        bytesAddressPair = s.recvfrom(bufferSize)
        msg_str = bytesAddressPair[0].decode('utf-8')
        msg = json.loads(msg_str)
 
t = Thread(target = read_socket)
t.daemon = True
t.start()

def align_stereo_cam(align_thresh):
    vis.chose_raspberry()
    vis.maxRadius = 60
    qRgb = vis.open_stereo_cam()
    pipeline, _ = vis.set_camera_pipeline()
    with dai.Device(pipeline) as device:
        # Output queue will be used to get the rgb frames from the output defined above
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        # Skip the first frames while the camera focuses
        count = 0
        while count < 20:
            _  = qRgb.get().getCvFrame() 
            count += 1
        # Align x and y
        for i in range(2):
            aligned = False
            actual_speed = [0, 0, 0]
            align_speed = [0, 0, 0]
            while not aligned:
                aligned, direction = vis.align_frame(qRgb, i, align_thresh)
                
                if direction == 'move left':
                    align_speed = [-0.01, 0, 0]
                elif direction == 'move right':
                    align_speed = [0.01, 0, 0]  
                elif direction == 'move up':
                    align_speed = [0, -0.01, 0]
                elif direction == 'move down':
                    align_speed = [0, 0.01, 0]  
                
                print("align speed", align_speed)
                if actual_speed != align_speed:
                    reset.stop_arm()
                    sleep(0.5)

                    print("\n\n\n STOP ARM", actual_speed, align_speed, (actual_speed != align_speed))

                    actual_speed = align_speed
                    R = reset.get_R06('UR5')
                    actual_speed_for_arm = R@actual_speed
                    pos = reset.change_arm(actual_speed_for_arm, 0)
                    reset.move_arm(pos)
                print(direction)
            reset.stop_arm()

def approach_rasp_depth():
    depth = 1000
    move_closer = [0, 0, 0.05] #m
    i = 0
    while depth > 300:
        raspberries = vis.find_raspberry_coordinate()
        
        vis.save_video_data(vis.depthMap, "disparity_map"+str(i), (vis.w_adj, vis.h_adj))
        vis.save_video_data(vis.raspDetection, "detected_raspberries"+str(i), (vis.w_adj, vis.h_adj))
        vis.save_video_data(vis.rgb_frames, "rgb_frame"+str(i), (vis.w_rgb, vis.h_rgb))
        i += 1

        centre_rasp, _= vis.centre_raspberry(raspberries, [0,0])
        depth = centre_rasp[2]
        if depth <= 300: #mm
            move_closer = [0, 0, 0.1] #m
        R = reset.get_R06('UR5')
        ur_pose = reset.change_arm(R@move_closer, 1)
        reset.move_arm_to_position(ur_pose)

def approach_rasp_CHT():
    maxRadius = 40
    approach_rasp_CHT_frames = []
    # qRgb = vis.open_stereo_cam()
    pipeline, _ = vis.set_camera_pipeline()
    with dai.Device(pipeline) as device:
        # Output queue will be used to get the rgb frames from the output defined above
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        # Skip the first frames while the camera focuses
        count = 0
        while count < 20:
            _  = qRgb.get().getCvFrame() 
            count += 1
        vis.verified_radius = 0
        move_forward = [0, 0, 0.01]
        R = reset.get_R06('UR5')
        pos = reset.change_arm(R@move_forward, 0)
        reset.move_arm(pos)
        while vis.verified_radius < maxRadius:
            rasp_coord, radii, centre, frame= vis.get_rasp_pixel_coord(qRgb, False)
            approach_rasp_CHT_frames.append(frame)
            centre_rasp,_ = vis.centre_raspberry(rasp_coord, centre)
            _ = vis.verify_raspberry(centre_rasp, 0, radii)
            print('Radius',vis.verified_radius)
            # vis.param2 += 1 # change parameteer of CHT as the robot arm moves forward
        reset.stop_arm()
        vis.save_video_data(approach_rasp_CHT_frames, "Approach_rasp_CHT", (frame.shape[1], frame.shape[0]))

def test_gripper():
    harv.move_fingers_to_position(harv.open)
    sleep(0.5)
    harv.move_fingers_to_position(harv.mid_pos)
    sleep(0.5)
    harv.move_fingers_to_position(harv.open)
    sleep(0.5)

def main():
    # Position the arm takes to detect the raspberries
    # NOTE: MIGHT NEED TO BE CHANGED ON THE FIELD 
    harv.move_fingers_to_position(harv.open)
    start_joints = [0.4539504647254944, -1.3301237265216272, 1.3692049980163574, -3.1960156599627894, -0.32082921663393194, -0.010236088429586232]# basket_joints = [1.81415855884552, -1.359485928212301, -2.449453655873434, -0.4877827803241175, 1.3663604259490967, 0.06709630787372589]
    pulling_force_init = 0
    time_stamps = []
    # lowest_z = 0.11720719872012049

    zeroTime = time()
    reset.move_arm_joints(start_joints,0.1)
    test_gripper()
    time_stamps.append(time() - zeroTime)
    input('\n\nPress Enter to align')
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    # Align raspberry with OAK-D RGB camera
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    align_stereo_cam(align_thresh = 5)
    vis.save_video_data(vis.align_frames, "Aligning_w_raspberry", vis.size_align_frames)
    time_stamps.append(time() - zeroTime)

    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    # Depth detection 
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    input('\n\nPress Enter to approach')
    time_stamps.append(time() - zeroTime)
    harv.move_fingers_to_position(harv.close)
    # approach_rasp_depth()    
    approach_rasp_CHT()    

    harv.move_fingers_to_position(harv.mid_pos)
    sleep(0.5)
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    # Align with the fingers
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    move_up = [0, -0.045, 0]
    R = reset.get_R06('UR3')
    ur_pose = reset.change_arm(R@move_up, 1)
    reset.move_arm_to_position(ur_pose, 0.1)
    
    # time_stamps.append(time() - zeroTime)
    
    input('\n\nPress Enter to approach with TOF')       
    sleep(0.5)
    time_stamps.append(time() - zeroTime)
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    # Approach raspberry using TOF sensor
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    approached = harv.approach_raspberry()
    approach_speed = [0, 0,  0.005]
    R = reset.get_R06('UR5')
    pos = reset.change_arm(R@approach_speed, 0)
    reset.move_arm(pos)
    previous = []
    while not approached:
        approached = harv.approach_raspberry()
        if msg != previous:
            print(msg)
            previous = msg
        if msg[1] == 1:
            break

    reset.stop_arm()
    sleep(0.5)
    move_forward = [0, 0, 0.032]
    R = reset.get_R06('UR5')
    pos = reset.change_arm(R@move_forward, 1)
    reset.move_arm_to_position(pos)

    sleep(0.5)
    move_up = [0, -0.005, 0]
    R = reset.get_R06('UR5')
    ur_pose = reset.change_arm(R@move_up, 1)
    reset.move_arm_to_position(ur_pose, 0.025)

    time_stamps.append(time() - zeroTime)

    print('\n\nTime to approach raspberry [s]: ', time() - zeroTime)

    input('\n\nPress Enter to grasp and pull raspberry') 
    time_stamps.append(time() - zeroTime)
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    # Grasp and pull
    #----------------------------------------------------------------------------------------------------------------------------------------------------------s     
    #for debugging #
    # joints raspberry = [0.7063584923744202, -1.6583831946002405, -1.6437566916095179, 0.16852736473083496, 1.6586774587631226, 0.00222756783477962]
    raspberry = reset.get_arm_pose() # keep position of the raspberry
    init_final_pos = [raspberry]
    # raspberry = [0.4544665714974, 0.24128676737548374, 0.5356093246361839, -1.5143836568366442, 0.601165144548914, -0.5797220039919777]
    harv.move_fingers_to_position(harv.open)
    # reset.move_arm_to_position(raspberry, 0.05, 0.1,False)

    check_stem = False
    detect_slip = False
    fail = False
    slip_thresh = 40
    slip_ref = 0
    compression_force = 0
    time_vec = []
    pulling_gripper = []
    compression_loadcell = []
    gripper_positions = []
    F_d1 = harv.set_grasping_force(pulling_force_init)
    Fd = F_d1
    print('Force', Fd)
    pull_speed = [0, 0.01, 0] # pulling speed
    harv.set_controller(K1[0], K1[1], K1[2]) 
    error_margin = 10
    error = 12

    while True:
        while np.abs(error) > error_margin:
            # if detect_slip:
                # print('Slip', msg)

            # Check when raspberry gets picked
            if harv.is_off_stem_gripper() and check_stem:
                reset.stop_arm()
                Fd = F_d2
                error_margin = 5
                check_stem = False 
                detect_slip = False
                print('OFF STEM')
                harv.set_controller(K2[0], K2[1], K2[2])   
                harv.change_grasping_force(is_off_stem=True)
                # vis.close_raspi_cam()0

            # Stop arm and readjust if slip is detected 
            # NOTE: RUN SLIP DETECTION IN PARALLEL
            if detect_slip and abs(msg[0]-slip_ref) >= slip_thresh:# and compression_force <= Fd:
                if harv.is_off_stem_gripper():
                    continue
                print('slipping')
                reset.stop_arm()
                harv.move_gripper(0)
                move_up = [0, -0.01, 0]
                R = reset.get_R06('UR5')
                pos = reset.change_arm(R@move_up, 1)
                # if msg[1] == '0':
                #     harv.move_fingers_to_position(harv.mid_pos)
                reset.move_arm_to_position(pos, 0.03)
                harv.change_grasping_force(is_off_stem=False)
                F_d1 = harv.pulling_force[-1][0]
                Fd = F_d1
                print('Fd', Fd)
                harv.pulling_difs = []
                slip_thresh += 1
                detect_slip = False
                check_stem = False 
                error_margin = 10
                sleep(0.5)

            # NOTE TRY TO RUN THE CONTROLLER IN PARALLEL AS WELL (when the rest is working...)
            error, compression_force, action = harv.control_gripper(Fd, harv.is_off_stem_gripper(),compression_loadcell)
            t = time() - zeroTime
            time_vec.append(t)
            compression_loadcell.append(compression_force)
            pulling_gripper.append(harv.get_gripper_vertical_force())
            gripper_positions.append(harv.gripper.read_position())

            # Plotjuggler
            newData["gripping force"]           = compression_force
            newData["desired gripping force"]   = Fd
            newData["pulling gripper"]          = harv.get_gripper_vertical_force()
            newData["control action"]           = action
            newData['timestamp']                = t
            sock.sendto(json.dumps(newData).encode(), ("127.0.0.1", plot_juggler_port))
            
            # If the picking is not detected
            if np.abs(reset.get_arm_pose()[2]-raspberry[2]) >= 0.05 and Fd == F_d1:
            # if abs(reset.get_arm_pose()[2]-lowest_z) >= 0.01 and Fd == F_d1:
                harv.move_gripper(0)
                reset.stop_arm()
                print('\n\n\nOff stem was not detected!')
                fail = True
                break
            
            # if slipping is not detected
            elif msg[1] == 0 and Fd == F_d2:
            # elif Fd == F_d2 and harv.is_in_position(harv.close):
                fail = True
                break
        if fail:
            print('\n\n\n Fail \n\n\n')
            break
        sock.sendto(json.dumps(newData).encode(), ("127.0.0.1", plot_juggler_port))
        
        # Pull raspberry
        if np.abs(error) <=  error_margin and Fd == F_d1:
            sleep(0.5)
            error_margin = 0
            R = reset.get_R06('UR5')
            pos = reset.change_arm(R@pull_speed, 0)
            reset.move_arm(pos)
            # slip_ref = msg[0]
            # harv.pulling_difs = []
            # detect_slip = True
            check_stem = True
            # start getting slip readings

        # Stop if raspberry was picked
        elif np.abs(error) <=  error_margin and Fd == F_d2:
            count = 0 
            while count < 10:
                error, compression_force, action = harv.control_gripper(Fd, harv.is_off_stem_gripper(),compression_loadcell)
                t = time() - zeroTime
                time_vec.append(t)
                compression_loadcell.append(compression_force)
                pulling_gripper.append(harv.get_gripper_vertical_force())
                gripper_positions.append(harv.gripper.read_position())

                # Plotjuggler
                newData["gripping force"]           = compression_force
                newData["desired gripping force"]   = Fd
                newData["pulling gripper"]          = harv.get_gripper_vertical_force()
                newData["control action"]           = action
                newData['timestamp']                = t
                sock.sendto(json.dumps(newData).encode(), ("127.0.0.1", plot_juggler_port))
                count += 1
            break
    sock.sendto(json.dumps(newData).encode(), ("127.0.0.1", plot_juggler_port))
    harv.move_gripper(0)
    init_final_pos.append(reset.get_arm_pose())
    harv.save_csv_data(init_final_pos, 'Initial_and_final_robot_position'+str(experimentID))
    harv.save_csv_data(compression_loadcell, 'Compression_force_gripper'+str(experimentID))
    harv.save_csv_data(pulling_gripper, 'Pulling_force_gripper'+str(experimentID))
    harv.save_csv_data(time_vec, 'Time_vector'+str(experimentID))
    harv.save_csv_data(gripper_positions, 'Gripper positions'+str(experimentID))

    time_stamps.append(time() - zeroTime)
    
    input('\n\nPress Enter to go back to start position')
    time_stamps.append(time() - zeroTime)
   
    # Put raspberry in the basket
    reset.move_arm_joints(start_joints, 0.1)

    time_stamps.append(time() - zeroTime)
    
    input('\n\nPress Enter to put raspberry in basket')
    time_stamps.append(time() - zeroTime)

    reset.move_arm_joints(basket_joints, 0.05)

    time_stamps.append(time() - zeroTime)

    input('\n\nPress Enter to release raspberry')
    time_stamps.append(time() - zeroTime)
    
    harv.move_fingers_to_position(harv.mid_pos)
    sleep(2)
    # reset.move_arm_joints(start_joints, 0.3)
    # reset.rtde_c.stopScript()

    time_stamps.append(time() - zeroTime)

    input('\n\nPress Enter to go back to start position')
    time_stamps.append(time() - zeroTime)
   
    # Put raspberry in the basket
    reset.move_arm_joints(start_joints, 0.05)
    time_stamps.append(time() - zeroTime)

    
    harv.save_csv_data(time_stamps, 'Time_stamps'+str(experimentID))


if __name__ == '__main__':
    main()




