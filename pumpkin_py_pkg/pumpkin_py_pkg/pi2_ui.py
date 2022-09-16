#!/usr/bin/env python3

from tkinter.constants import LEFT, RIGHT
import rclpy
from rclpy.node import Node
import tkinter as tk 
import threading
import time
import subprocess
import psutil
from tkinter import Canvas, Frame, IntVar, messagebox
import numpy as np
import cv2
from example_interfaces.srv import SetBool
from PIL import Image, ImageTk

# SSD CNN Constants
SSD_INPUT_SIZE = 320

RESIZED_DIMENSIONS = (300, 300) # Dimensions that SSD was trained on. 
IMG_NORM_RATIO = 0.007843 # In grayscale a pixel can range between 0 and 255
 

#------------------------------------------------------------
# class Pi2UI
#------------------------------------------------------------
class Pi2UI(tk.Tk):

    def __init__(self):
        super().__init__()

        # Create the main window
        self.title("Pumpkin Bot - Pi 2")

        #self.geometry("1536x864")
        self.geometry("1280x720")
        self.resizable(False, False)

        # set the position of the window to the center of the screen        
        #self.eval('tk::PlaceWindow . center')

        self.configure(background="wheat3")

        # Adds an icon to the app in the Activities bar
        iconImg = tk.Image("photo", file = "/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/launch/pumpkin.png")
        self.tk.call('wm', 'iconphoto', self._w, iconImg)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=3)
        self.columnconfigure(2, weight=1)

        leftFrame = Frame(self, borderwidth=3, relief=tk.SUNKEN, width=300, height=600)
        leftFrame.pack_propagate(0)
        leftFrame.grid(row=0, column=0, pady=(10,10))

        self.statusLabel = tk.Label(self, bg="black", fg="orange", text="Status: Initializing", font=("Arial", 22), takefocus=0)
        self.statusLabel.grid(row=1, column=1, pady=(25,0))

        rightFrame = Frame(self, borderwidth=3, relief=tk.SUNKEN, width=300, height=600)
        rightFrame.pack_propagate(0)
        rightFrame.grid(row=0, column=2, pady=(10,10))

        self.laneDetectionButton = tk.Button(leftFrame, text = "Lane Detection", takefocus=0, width=100, 
                command=self.laneDetectionButtonPressed)
        self.laneDetectionButton.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.faceDetectionButton = tk.Button(leftFrame, text = "Face Detection", takefocus=0, width=100, 
                command=self.faceDetectionButtonPressed)
        self.faceDetectionButton.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.ssdCNNButton = tk.Button(leftFrame, text = "SSD CNN", takefocus=0, width=100, 
                command=self.ssdCNNButtonPressed)
        self.ssdCNNButton.pack(anchor="center", padx=(10,10), pady=(10,10))

        quitButton = tk.Button(leftFrame, text = "Quit", takefocus=0, width=100, command=self.quitButtonCallback)
        quitButton.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.protocol("WM_DELETE_WINDOW", self.quitButtonCallback)

        self.statusLabel.config(text="Status: Waiting for Initialization Command...")

        self.startedNavigation = False
        self.startedSLAM = False
        self.startedLaneDetection = False
        self.startedFaceDetection = False
        self.startedSSDCNN = False

        self.laneDetectionCancelId = 0
        self.faceDetectionCancelId = 0
        self.ssdCNNCancelId = 0

        # self.videoCapture = cv2.VideoCapture(0)
        # self.laneDetectionImage = self.videoCapture.read()
        # self.faceDetectionImage = self.videoCapture.read()
        # self.ssdCNNImage = self.videoCapture.read()

        # # setting the width and height of the video window
        # self.videoCapture.set(3, 640)
        # self.videoCapture.set(4, 480)

        # self.cascade_classifier = cv2.CascadeClassifier('/home/ubuntu/ros2_ws/src/pumpkin_py_pkg/pumpkin_py_pkg/haarcascade_frontalface_alt.xml')

        # self.openCVCanvas = Canvas(self, width=600, height=600, takefocus=0)  
        # self.openCVCanvas.grid(row=0, column=1, pady=(10,0))

        # # Load the pre-trained neural network
        # self.neural_network = cv2.dnn.readNetFromCaffe('/home/ubuntu/ros2_ws/src/pumpkin_py_pkg/pumpkin_py_pkg/MobileNetSSD_deploy.prototxt.txt', 
        # '/home/ubuntu/ros2_ws/src/pumpkin_py_pkg/pumpkin_py_pkg/MobileNetSSD_deploy.caffemodel')
 
        # # List of categories and classes
        # self.categories = { 0: 'background', 1: 'aeroplane', 2: 'bicycle', 3: 'bird', 
        #        4: 'boat', 5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 
        #        9: 'chair', 10: 'cow', 11: 'diningtable', 12: 'dog', 
        #       13: 'horse', 14: 'motorbike', 15: 'person', 
        #       16: 'pottedplant', 17: 'sheep', 18: 'sofa', 
        #       19: 'train', 20: 'tvmonitor'}
 
        # self.classes = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", 
        #     "bus", "car", "cat", "chair", "cow", 
        #    "diningtable",  "dog", "horse", "motorbike", "person", 
        #    "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
                      
        # # Create the bounding boxes
        # self.bbox_colors = np.random.uniform(255, 0, size=(len(self.categories), 3))
     
        
    def startNavigation(self):
        self.startedNavigation = True
        #self.navigationProcess = subprocess.Popen(["ros2", "launch", "nav2_bringup", "bringup_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/nav2_params.yaml", "map:=/home/ubuntu/bonus_room.yaml"])
        self.navigationProcess = subprocess.Popen(["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/nav2_params.yaml"])
        time.sleep(1)

    def startSLAM(self):
        self.startedSLAM = True
        #self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "offline_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_offline.yaml"])
        self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_online_async.yaml"])
        #self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_sync_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_online_sync.yaml"])
        #self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "localization_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_localization.yaml"])
        time.sleep(1)
 
    def quitButtonCallback(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):

            self.statusLabel.config(text="Status: Quitting application...")
    
            if self.startedSLAM:
                # Try to kill the SLAM process so the node doesn't stick around
                slamProcess = psutil.Process(self.slamProcess.pid)
                for proc in slamProcess.children(recursive=True):
                    proc.kill()
                slamProcess.kill()

            # Try to kill the Navigation process so the node doesn't stick around
            if self.startedNavigation:
                navigationProcess = psutil.Process(self.navigationProcess.pid)
                for proc in navigationProcess.children(recursive=True):
                    proc.kill()
                navigationProcess.kill()

            # self.videoCapture.release()
            cv2.destroyAllWindows()

            self.destroy()  

    def startProceses(self):
        initSlamThread = threading.Thread(target=self.startSLAM)
        initSlamThread.start()

        initNavigationThread = threading.Thread(target=self.startNavigation)
        initNavigationThread.start()

        self.statusLabel.config(text="Status: Initialization Complete")

    def killProcesses(self):
            self.statusLabel.config(text="Status: Quitting Application...")
    
            # Try to kill the SLAM process so the node doesn't stick around
            if self.startedSLAM:
                slamProcess = psutil.Process(self.slamProcess.pid)
                for proc in slamProcess.children(recursive=True):
                    proc.kill()
                slamProcess.kill()

            # Try to kill the Navigation process so the node doesn't stick around
            if self.startedNavigation:
                navigationProcess = psutil.Process(self.navigationProcess.pid)
                for proc in navigationProcess.children(recursive=True):
                    proc.kill()
                navigationProcess.kill()

            self.destroy()      

    def startMainLoop(self):
        self.mainloop() 

    #------------------------------------------------------------
    # Lane Detection Methods
    #------------------------------------------------------------

    def draw_the_lines(self, image, lines):
        # create a distinct image for the lines [0,255] - all 0 values means black image
        lines_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        # there are (x,y) for the starting and end points of the lines
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), thickness=3)

        # finally we have to merge the image with the lines
        image_with_lines = cv2.addWeighted(image, 0.8, lines_image, 1, 0.0)

        return image_with_lines


    def region_of_interest(self, image, region_points):
        # we are going to replace pixels with 0 (black) - the regions we are not interested
        mask = np.zeros_like(image)
        # the region that we are interested in is the lower triangle - 255 white pixels
        cv2.fillPoly(mask, region_points, 255)
        # we have to use the mask: we want to keep the regions of the original image where  
        # the mask has white colored pixels
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image


    def get_detected_lanes(self, image):

        _, image = self.videoCapture.read()

        (height, width) = (image.shape[0], image.shape[1])

        # we have to turn the image into grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # edge detection kernel (Canny's algorithm)
        canny_image = cv2.Canny(gray_image, 100, 120)

        # we are interested in the "lower region" of the image (there are the driving lanes)
        region_of_interest_vertices = [
            (0, height),
            (width / 2, height * 0.65),
            (width, height)
        ]

        # we can get rid of the un-relevant part of the image
        # we just keep the lower triangle region
        cropped_image = self.region_of_interest(
            canny_image, np.array([region_of_interest_vertices], np.int32))

        # use the line detection algorithm (radians instead of degrees 1 degree = pi / 180)
        lines = cv2.HoughLinesP(cropped_image, rho=2, theta=np.pi / 180, threshold=50, lines=np.array([]),
                            minLineLength=40, maxLineGap=150)

        # draw the lines on the image
        image_with_lines = self.draw_the_lines(image, lines)

        return image_with_lines
    
 
    def startLaneDetection(self):
 
        self.laneDetectionImage = self.get_detected_lanes(self.laneDetectionImage)

        self.laneDetectionImage = cv2.cvtColor(self.laneDetectionImage, cv2.COLOR_BGR2RGB)
        self.laneDetectionImage = Image.fromarray(self.laneDetectionImage) # to PIL format
        self.laneDetectionImage = ImageTk.PhotoImage(self.laneDetectionImage) # to ImageTk format
        # Update image
        self.openCVCanvas.create_image(0, 0, anchor=tk.NW, image=self.laneDetectionImage)
        # Repeat every 'interval' ms
        self.laneDetectionCancelId = self.after(10, self.startLaneDetection)


    def laneDetectionButtonPressed(self):
        if self.startedLaneDetection:
            self.startedLaneDetection = False
            self.laneDetectionButton['text'] = 'Lane Detection'
            self.openCVCanvas.delete("all")
            self.after_cancel(self.laneDetectionCancelId)
        elif not self.startedFaceDetection:
            self.openCVCanvas.delete("all")
            self.startedLaneDetection = True
            self.laneDetectionButton['text'] = 'Lane Detection*'
            self.startLaneDetection()

    #------------------------------------------------------------
    # Face Detection Methods
    #------------------------------------------------------------

    def get_detected_faces(self, image):
        
        _, image = self.videoCapture.read()

        (height, width) = (image.shape[0], image.shape[1])

        # we have to turn the image into grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # using face detection algorithm with the trained classifier
        detected_faces = self.cascade_classifier.detectMultiScale(gray_image, scaleFactor=1.2, minNeighbors=5, minSize=(30, 30),
                                 flags=cv2.CASCADE_SCALE_IMAGE)

        # draw the faces (rectangle) in every video frame
        for (x, y, width, height) in detected_faces:
            cv2.rectangle(image, (x, y), (x + width, y + height), (0, 0, 255), 10)

        return image


    def startFaceDetection(self):

        self.faceDetectionImage = self.get_detected_faces(self.faceDetectionImage)

        self.faceDetectionImage = cv2.cvtColor(self.faceDetectionImage, cv2.COLOR_BGR2RGB)
        self.faceDetectionImage = Image.fromarray(self.faceDetectionImage) # to PIL format
        self.faceDetectionImage = ImageTk.PhotoImage(self.faceDetectionImage) # to ImageTk format
        # Update image
        self.openCVCanvas.create_image(0, 0, anchor=tk.NW, image=self.faceDetectionImage)
        # Repeat every 'interval' ms
        self.faceDetectionCancelId = self.after(10, self.startFaceDetection)


    def faceDetectionButtonPressed(self):
        if self.startedFaceDetection:
            self.startedFaceDetection = False
            self.faceDetectionButton['text'] = 'Face Detection'
            self.openCVCanvas.delete("all")
            self.after_cancel(self.faceDetectionCancelId)
        elif not self.startedLaneDetection:
            self.openCVCanvas.delete("all")
            self.startedFaceDetection = True
            self.faceDetectionButton['text'] = 'Face Detection*'
            self.startFaceDetection()

    #------------------------------------------------------------
    # SSD CNN Methods
    #------------------------------------------------------------

    def startSSDCNN(self):

        _, self.ssdCNNImage = self.videoCapture.read()
        (h, w) = (self.ssdCNNImage.shape[0], self.ssdCNNImage.shape[1])

        # Create a blob. A blob is a group of connected pixels in a binary 
        # frame that share some common property (e.g. grayscale value)
        # Preprocess the frame to prepare it for deep learning classification
        frame_blob = cv2.dnn.blobFromImage(cv2.resize(self.ssdCNNImage, RESIZED_DIMENSIONS), 
                     IMG_NORM_RATIO, RESIZED_DIMENSIONS, 127.5)
     
        # Set the input for the neural network
        self.neural_network.setInput(frame_blob)
 
        # Predict the objects in the image
        neural_network_output = self.neural_network.forward()
 
        # Put the bounding boxes around the detected objects
        for i in np.arange(0, neural_network_output.shape[2]):
             
            confidence = neural_network_output[0, 0, i, 2]
     
            # Confidence must be at least 30%       
            if confidence > 0.30:
                 
                idx = int(neural_network_output[0, 0, i, 1])
 
                bounding_box = neural_network_output[0, 0, i, 3:7] * np.array(
                    [w, h, w, h])
 
                (startX, startY, endX, endY) = bounding_box.astype("int")
 
                label = "{}: {:.2f}%".format(self.classes[idx], confidence * 100) 
         
                cv2.rectangle(self.ssdCNNImage, (startX, startY), (
                    endX, endY), self.bbox_colors[idx], 2)     
                         
                y = startY - 15 if startY - 15 > 15 else startY + 15    
 
                cv2.putText(self.ssdCNNImage, label, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, self.bbox_colors[idx], 2)
         
        # We now need to resize the frame so its dimensions
        # are equivalent to the dimensions of the original frame
        #frame = cv2.resize(self.ssdCNNImage, file_size, interpolation=cv2.INTER_NEAREST)
 

        self.ssdCNNImage = cv2.cvtColor(self.ssdCNNImage, cv2.COLOR_BGR2RGB)
        self.ssdCNNImage = Image.fromarray(self.ssdCNNImage) # to PIL format
        self.ssdCNNImage = ImageTk.PhotoImage(self.ssdCNNImage) # to ImageTk format

        # Update image
        self.openCVCanvas.create_image(0, 0, anchor=tk.NW, image=self.ssdCNNImage)

        # Repeat every 'interval' ms
        self.ssdCNNCancelId = self.after(10, self.startSSDCNN)

    def ssdCNNButtonPressed(self):
        if self.startedSSDCNN:
            self.startedSSDCNN = False
            self.ssdCNNButton['text'] = 'SSD CNN'
            self.openCVCanvas.delete("all")
            self.after_cancel(self.ssdCNNCancelId)
        elif not self.startedLaneDetection:
            self.openCVCanvas.delete("all")
            self.startedSSDCNN = True
            self.ssdCNNButton['text'] = 'SSD CNN*'
            self.startSSDCNN()

#------------------------------------------------------------
# class Pi2UserInterfaceNode
#------------------------------------------------------------
class Pi2UserInterfaceNode(Node):
    
    def __init__(self):
        super().__init__("pi2_user_interface") 

        self.userInterface = Pi2UI()

        self.server = self.create_service(SetBool, "set_pi2_enable_flag", self.callback_set_pi2_enable_flag)

        spinningThread = threading.Thread(target=self.startSpinning)
        spinningThread.start()

        self.userInterface.startMainLoop()
  
    def startSpinning(self):
        rclpy.spin(self)
        rclpy.shutdown()

    def callback_set_pi2_enable_flag(self, request, response):
        if (request.data == True):
            self.userInterface.startProceses()
        else:
            self.userInterface.killProcesses()
        response.success = True

        return response

#------------------------------------------------------------
# main
#------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Pi2UserInterfaceNode()

#------------------------------------------------------------
# Entry Point
#------------------------------------------------------------
if __name__ == "__main__":
    main()

