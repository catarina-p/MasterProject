import math
import os
from datetime import datetime
from typing import Union

import cv2 as cv
import depthai as dai
import numpy as np
from scipy.cluster.vq import kmeans2, whiten
from scipy.spatial.distance import cdist

from vision_files.gen2_calc_spatials_on_host.calc import HostSpatialsCalc
from vision_files.gen2_calc_spatials_on_host.utility import *


class Vision():
    def __init__(self, experimentID) -> None:
        """
        This class contains the methods used for raspberry detction and approach.
        """
        self.experimentID      = experimentID
        # Blur kernel
        self.n_blur            = 9
        # Hue threshold
        self.hue_thresh        = [10,150]#150 
        self.value_thresh      = 240
        # Circle Hough Transform Parameters
        self.minDist           = 30
        self.param1            = 100
        self.param2            = 17  
        self.minRadius         = 5    
        self.maxRadius         = 50   
        # RGB window
        self.w_rgb             = 640
        self.h_rgb             = 400
        self.interLeaved       = False
        # Camera window offset
        self.y_init            = 60
        self.x_init            = 25#50
        self.h_adj             = 300
        self.w_adj             = 480
        self.scale             = self.w_adj/self.w_rgb
        # Stereo Parametres
        self.LR_check          = False
        self.subPixel          = False
        self.extDisparity      = True
        # Define how many frames will be analysed and the initial delay 
        self.delay             = 20
        self.cameraOn          = 80+self.delay
        # Region of interest margin
        self.delta             = 5
        # Parameters for camera to UR3 coordinate transformation
        ## Camera position in frame 6
        self.pcam6             = np.array([[0], [0.055], [0.125]])
        ## camera z measurement systematic error
        self.z_offset          = np.array([[0],[0],[0]]) #0.17
        self.robot_offset      = np.array([[0],[0],[0]]) #0 0.05 0.08
        # Percentile for outlier removal
        self.percentile        = 90
        # Directory to save data
        self.directory         = 'D:/IST/5 ano/2 semestre/Tese/raspberry-grasping/data/Real raspberry trials'
        # Slip detection memory 
        self.slip_thresh       = 10
        self.previous_frame    = 255
        # Align with raspberry
        self.align_frames      = []
        self.chosen_raspberry  = 0

    def set_raspicam_pipeline(self,
        sensor_id=0,
        capture_width=1920,
        capture_height=1080,
        display_width=960,
        display_height=540,
        framerate=30,
        flip_method=0,
    ):
        """
        Setting Raspberry Pi camera.
        """
        return (
            "nvarguscamerasrc sensor-id=%d !"
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )

    def open_raspi_cam(self) -> None:
        """
        Opens Raspberry Pi camera.
        """
        self.video_capture = cv.VideoCapture(self.set_raspicam_pipeline(flip_method=0), cv.CAP_GSTREAMER)

    def raspi_cam_frame(self):
        """
        Obtains Raspberry Pi camera's frame.
        """
        if self.video_capture.isOpened():
            _, frame = self.video_capture.read()
            return frame

    def close_raspi_cam(self):
        """
        Stops Raspberry Pi camera's video.
        """
        self.video_capture.release()

    def set_camera_pipeline(self):                                                                                      
        """
        Create pipeline for OAK-D camera.
        """
        # Create pipeline
        pipeline = dai.Pipeline()
        
        # Define source and output
        camRgb = pipeline.create(dai.node.ColorCamera)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        # Properties
        camRgb.setPreviewSize(self.w_rgb, self.h_rgb)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(self.interLeaved)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo.initialConfig.setConfidenceThreshold(255)
        stereo.setLeftRightCheck(self.LR_check)
        stereo.setSubpixel(self.subPixel)
        stereo.setExtendedDisparity(self.extDisparity)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        camRgb.preview.link(xoutRgb.input)

        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        stereo.depth.link(xoutDepth.input)

        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("disp")
        stereo.disparity.link(xoutDepth.input)
        
        return pipeline, stereo

    def open_stereo_cam(self):
        """
        Turn on OAK-D camera.
        """
        pipeline, _ = self.set_camera_pipeline()
        with dai.Device(pipeline) as device:
            # Output queue will be used to get the rgb frames from the output defined above
            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            # Skip the first frames while the camera focuses
            count = 0
            while count < 20:
                _  = qRgb.get().getCvFrame() 
                count += 1
        return qRgb

    def rescale_frame(self, frame):
        """
        This function resizes the the input image (frame) by a certain scale.
        """
        width = int(frame.shape[1]*self.scale)
        height = int(frame.shape[0]*self.scale)
        dimensions = (width, height)
        return cv.resize(frame, dimensions, interpolation = cv.INTER_AREA)

    def clip_outliers(self, features, k, percentile):
        """
        This function takes out the outliers from the found stereo coordinates.
        """
        whitened = whiten(features)
        (centroids, labels) = kmeans2(whitened, k, minit='points')
        std = np.std(features, axis=0)
        denormalized = list(centroids*std)
        # points array will be used to reach the index easily
        points = np.empty((0,len(features[0])), float)
        # distances will be used to calculate outliers
        distances = np.empty((0,len(features[0])), float)
        # getting points and distances
        for i, center_elem in enumerate(denormalized):
            # cdist is used to calculate the distance between center and other points
            distances = np.append(distances, cdist([center_elem],features[labels == i], 'euclidean')) 
            points = np.append(points, features[labels == i], axis=0)
        # Remove outliers
        new_data = points[np.where(distances <= np.percentile(distances, percentile))]
        std2 = np.std(new_data, axis=0)
        # Recalculate clusters
        whitened = whiten(new_data)
        (centroids, labels) = kmeans2(whitened, k, minit='points')
        final_coords = centroids*std2
        return final_coords

    def find_circles(self, image, img_process):
        """
        Apply circle Hough transform.
        """
        # source: https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
        circles = cv.HoughCircles(img_process, cv.HOUGH_GRADIENT, 1, self.minDist,
                                    param1=self.param1, param2=self.param2,
                                    minRadius=self.minRadius, maxRadius=self.maxRadius)
        rasp_coord = []
        # rasp_window = []
        radii = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (int(i[0]), int(i[1]))
                # circle center
                cv.circle(image, center, 1, (255, 0, 0), 3)
                # circle outline
                radius = i[2]
                # coord = (i[0]-radius, i[1]-radius, i[0]+radius, i[1]+radius)
                cv.circle(image, center, radius, (255, 0, 255), 3)
                rasp_coord.append(center)
                # rasp_window.append(coord)
                radii.append(radius)
        return rasp_coord, radii, image

    def get_rasperries(self, image):
        """
        Find raspberries in frame.
        """
        image = self.rescale_frame(image)
        image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        hue = image_hsv[:,:,0] 
        value = image_hsv[:,:,2] 
        image_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY) 
        new_hue = np.zeros_like(hue)
        idx = np.logical_or(hue<self.hue_thresh[0],hue>self.hue_thresh[1])
        idx = np.logical_and(idx, value<self.value_thresh)
        new_hue[idx] = image_gray[idx]
        # new_hue[np.where(hue>self.hue_thresh)] = image_gray[np.where(hue>self.hue_thresh)]# + bias
        blur_rasps = cv.GaussianBlur(new_hue,(self.n_blur,self.n_blur), 0)            
        return self.find_circles(image, blur_rasps)

    def find_raspberry_coordinate(self):
        """
        Find raspberry coordinate through stereo vision.
        """        
        pipeline, stereo = self.set_camera_pipeline()         
        # Connect to device and start pipeline
        with dai.Device(pipeline) as device:
            # Output queue will be used to get the rgb frames from the output defined above
            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            dispQ = device.getOutputQueue(name="disp")

            hostSpatials = HostSpatialsCalc(device)
            text = TextHelper()
            hostSpatials.setDeltaRoi(self.delta)

            all_depth_coord = []
            picture_coord = []
            self.raspDetection = []
            self.rgb_frames = []
            self.depthMap = []
            sum_rasp = 0
            all_x = []
            all_y = []
            all_z = []
            cameraOn = self.cameraOn
            delay = self.delay
            while cameraOn > 0:
                # blocking call, will wait until a new data has arrived
                image  = qRgb.get().getCvFrame() 

                image = cv.flip(image, 0)
                image = cv.flip(image, 1)

                rasp_coords, _, found_rasps = self.get_rasperries(image)

                depthFrame = depthQueue.get().getFrame() 
                depthFrame = depthFrame[self.y_init:self.y_init+self.h_adj, self.x_init:self.x_init+self.w_adj]

                depthFrame = cv.flip(depthFrame, 0)
                depthFrame = cv.flip(depthFrame, 1)

                # Get disparity frame for nicer depth visualization
                disp = dispQ.get().getFrame()
                disp = (disp * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
                disp = cv.applyColorMap(disp, cv.COLORMAP_JET)
                disp = disp[self.y_init:self.y_init+self.h_adj, self.x_init:self.x_init+self.w_adj]

                disp = cv.flip(disp, 0)
                disp = cv.flip(disp, 1)                

                for raspberry in rasp_coords:
                    # Calculate spatial coordiantes from depth frame
                    (x, y) = (raspberry[0],raspberry[1])
                    spatials, centroid = hostSpatials.calc_spatials(depthFrame, (x,y)) # centroid == x/y in our case
                    
                    x_depth = spatials['x']
                    y_depth = spatials['y']
                    z_depth = spatials['z']

                    text.rectangle(disp, (x-self.delta, y-self.delta), (x+self.delta, y+self.delta))
                    text.putText(found_rasps, "X: " + ("{:.1f}mm".format(x_depth) if not math.isnan(x_depth) else "--"), (x + 10, y + 20))
                    text.putText(found_rasps, "Y: " + ("{:.1f}mm".format(y_depth) if not math.isnan(y_depth) else "--"), (x + 10, y + 35))
                    text.putText(found_rasps, "Z: " + ("{:.1f}mm".format(z_depth) if not math.isnan(z_depth) else "--"), (x + 10, y + 50))

                    if delay <= 0:
                        if (not math.isnan(x_depth)) and (not math.isnan(y_depth)) and (not math.isnan(z_depth)):
                            all_depth_coord.append((x_depth, y_depth, z_depth))   
                            picture_coord.append((x, y))
                            sum_rasp += len(rasp_coords)
                            all_x.append(x_depth)
                            all_y.append(y_depth)
                            all_z.append(z_depth)

                delay = delay - 1
                cameraOn = cameraOn - 1
                cv.imshow("detected circles", found_rasps)
                cv.imshow("depth", disp)
                # cv.imshow("reds", new_hue)
                self.depthMap.append(disp)
                self.raspDetection.append(found_rasps)
                self.rgb_frames.append(image)

                if cv.waitKey(1) == ord('q'):
                    break
        total_rasp = np.ceil(sum_rasp/len(all_depth_coord))
        all_coord = np.array(all_depth_coord)
        final_coords = self.clip_outliers(all_coord, int(total_rasp), self.percentile)
        cv.destroyAllWindows()
        return final_coords

    def centre_raspberry(self, raspberries, centre):
        """
        Find the raspberry on frame that is closest to the centre of frame.
        """
        # Find the raspberry closest to the centre of the frame
        distances = []
        for elem in raspberries:
            # cdist is used to calculate the distance between center and other points
            distances = np.append(distances,np.sqrt((centre[0]-elem[0])**2+(centre[1]-elem[1])**2))
            # distances = np.append(distances, cdist(np.array([centre[0],centre[1]]),np.array([elem[0],elem[1]]), 'euclidean'))
        self.centre_idx = np.where(distances == np.min(distances))[0][0]
        centre_rasp = raspberries[self.centre_idx][:]
        return centre_rasp, np.min(distances)

    def get_rasp_pixel_coord(self, qRgb, print_centre):
        """
        Find the raspberries on frame.
        """
        rasp_coord = []
        while rasp_coord == []:
            frame = qRgb.get().getCvFrame() 
            rasp_coord,radii, img = self.get_rasperries(frame)
            centre = [int(img.shape[1]/2),int(img.shape[0]/2)]
            if print_centre:
                cv.circle(img, (centre[0], centre[1]), 1, (255, 255, 0), 3)
            flipVertical = cv.flip(img, 0)
            flipHorizontal = cv.flip(flipVertical, 1)
            cv.imshow('frame',flipHorizontal)
            if cv.waitKey(1) == ord('q'):
                break 
        return rasp_coord, radii, centre, img
    
    def chose_raspberry(self):
        """
        Choose raspberry to be harvested.
        """
        # qRgb = self.open_stereo_cam()
        pipeline, _ = self.set_camera_pipeline()
        with dai.Device(pipeline) as device:
            # Output queue will be used to get the rgb frames from the output defined above
            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            # Skip the first frames while the camera focuses
            count = 0
            while count < 50:
                _  = qRgb.get().getCvFrame() 
                count += 1
            count = 0
            dist_0 = 1000
            while count < 50:
                rasp_coord, radii, centre, img = self.get_rasp_pixel_coord(qRgb, False)
                centre_rasp, dist = self.centre_raspberry(rasp_coord, centre)
                if dist < dist_0:
                    self.chosen_raspberry = centre_rasp
                    dist_0 = dist
                count += 1
                print(radii)
            cv.circle(img, self.chosen_raspberry, 10, (100, 255, 0), 3)
            flipVertical = cv.flip(img, 0)
            flipHorizontal = cv.flip(flipVertical, 1)
            print('\n\n\nPress q to continue')
            while True:
                cv.imshow('chosen',flipHorizontal)
                if cv.waitKey(1) == ord('q'):
                    break
            cv.destroyAllWindows()
            # return dist_0
        
    def align_frame(self, qRgb, coordinate, align_thresh): 
        """
        Move end-effector to align the camera with the chosen raspberry.
        """ 
        rasp_coord, radii, centre, img = self.get_rasp_pixel_coord(qRgb, True)  
        flipVertical = cv.flip(img, 0)
        flipHorizontal = cv.flip(flipVertical, 1)            
        self.align_frames.append(flipHorizontal)
        self.size_align_frames = (self.w_adj, self.h_adj) 
        centre_rasp,_ = self.centre_raspberry(rasp_coord, centre)

        if centre_rasp == []:
            return False, None
        elif not self.verify_raspberry(centre_rasp, coordinate, radii):
            return False, None

        print("verified")        
        dist = centre_rasp[coordinate]-centre[coordinate]
        move = []
        if coordinate == 0 and dist > 0:
            move = 'move left'
        elif coordinate == 0 and dist < 0:
            move = 'move right'
        elif coordinate == 1 and dist > 0:
            move = 'move up'
        elif coordinate == 1 and dist < 0:
            move = 'move down'
        if np.abs(dist) < align_thresh:
            return True, move
        else:
            return False, move

    def verify_raspberry(self, raspberry, coordinate, radii):
        """
        Check if the raspberry the end-effector is aligning with, is the chosen raspberry.
        """
        if coordinate == 0:
            if np.abs(raspberry[1]-self.chosen_raspberry[1]) <= 50:
                self.chosen_raspberry = raspberry
                self.verified_radius = radii[self.centre_idx]
                return True
            else:
                return False
        elif coordinate == 1:
            if np.abs(raspberry[0]-self.chosen_raspberry[0]) <= 50:
                self.chosen_raspberry = raspberry
                self.verified_radius = radii[self.centre_idx]
                return True
            else:
                return False

    def cam_to_UR(self, coords, R, p6):
        """
        Coordinate transformation from camera frame to robot arm frame.
        """
        coord = np.array([[-coords[0]/1000],[-coords[1]/1000],[coords[2]/1000]])
        pcam0 = R@self.pcam6
        pcam = np.array([[p6[0]+pcam0[0][0]],[p6[1]+pcam0[1][0]],[p6[2]+pcam0[2][0]]])
        return (R@(coord-self.z_offset))+pcam-self.robot_offset

    def save_video_data(self, data, file_name, size):
        """
        Save camera videos to chosen directory.
        """
        main_dir = os.getcwd()
        os.chdir(self.directory)
        if os.path.isdir('Raspberry detection'+str(self.experimentID)) is False:
            os.makedirs('Raspberry detection'+str(self.experimentID))
        os.chdir('Raspberry detection'+str(self.experimentID))
        # Saves videos
        out = cv.VideoWriter(file_name+str(self.experimentID)+".avi",cv.VideoWriter_fourcc(*'DIVX'), 15, size)
        for i in range(len(data)):
            out.write(data[i])
        os.chdir(main_dir)
