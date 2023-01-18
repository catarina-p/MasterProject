import numpy as np
import depthai as dai
from vision import Vision
from reset_robot import Reset_Robot
from libraries.keyboard import Key

import cv2 as cv



def main():
    # Position the arm takes to detect the raspberries
    # NOTE: MIGHT NEED TO BE CHANGED ON THE FIELD 
    # start_joints = [0.8161766529083252, -0.8411205450641077, -2.3005085627185267, 0.006565093994140625, 1.5466278791427612, 3.5952674807049334e-05]
    
    vis   = Vision(1)
    reset = Reset_Robot()
    key = Key()
    # reset.move_arm_joints(start_joints)
    # input()
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    # Align raspberry with OAK-D RGB camera
    #----------------------------------------------------------------------------------------------------------------------------------------------------------
    pipeline, _ = vis.set_camera_pipeline()
    with dai.Device(pipeline) as device:
        # Output queue will be used to get the rgb frames from the output defined above
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        # Skip the first frames while the camera focus
        count = 0
        while count < 20:
            frame  = qRgb.get().getCvFrame() 
            count += 1

        actual_direction = ""
        demand_direction = ""
        while True:
            # Show camera and detected raspberry
            frame  = qRgb.get().getCvFrame()          	
            flipVertical = cv.flip(frame, 0)
            flipHorizontal = cv.flip(flipVertical, 1)
            centre = [int(frame.shape[1]/2),int(frame.shape[0]/2)]
            cv.circle(flipHorizontal, centre, 1, (255, 255, 0), 3)
            cv.imshow('frame',flipHorizontal)
            if cv.waitKey(1) == ord('q'):
                break 

            demand_direction = key.keyPress

            if demand_direction is not actual_direction:
                actual_direction = demand_direction
                if demand_direction == "a":
                    arm_speed = [-0.01, 0, 0]
                elif demand_direction == "d":
                    arm_speed = [0.01, 0, 0] 
                elif demand_direction == "w":
                    arm_speed = [0, -0.01, 0]
                elif demand_direction == "s":
                    arm_speed = [0, 0.01, 0]
                elif demand_direction == "z":
                    arm_speed = [0, 0, 0.01]
                elif demand_direction == "x":
                    arm_speed = [0, 0, -0.01]
                else:
                    reset.stop_arm()
                    continue
                
                R = reset.get_R06('UR5')
                arm_speed = R@arm_speed
            
                pos = reset.change_arm(arm_speed, 0)
                reset.move_arm(pos)

        # reset.move_arm_joints(start_joints)

if __name__ == '__main__':
    main()