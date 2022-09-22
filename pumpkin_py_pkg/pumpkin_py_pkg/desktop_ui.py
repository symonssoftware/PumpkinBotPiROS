#!/usr/bin/env python3

from tkinter.constants import LEFT, RIGHT
import rclpy
from rclpy.node import Node
import tkinter as tk 
import threading
import time
import subprocess
import psutil
from tkinter import Canvas, Frame, messagebox
import numpy as np
import cv2
from cv_bridge import CvBridge
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ROSImage

# SSD CNN Constants
SSD_INPUT_SIZE = 320

RESIZED_DIMENSIONS = (300, 300) # Dimensions that SSD was trained on. 
IMG_NORM_RATIO = 0.007843 # In grayscale a pixel can range between 0 and 255


#------------------------------------------------------------
# class DesktopUI
#------------------------------------------------------------
class DesktopUI(tk.Tk):

    def __init__(self):
        super().__init__()

        # Create the main window
        self.title("Desktop UI")

        self.geometry("1536x864")
        #self.geometry("1280x720")
        self.resizable(False, False)

        # set the position of the window to the center of the screen        
        #self.eval('tk::PlaceWindow . center')

        self.configure(background="wheat3")

        # Adds an icon to the app in the Activities bar
        icon_img = tk.Image("photo", file = "/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/launch/pumpkin.png")
        self.tk.call('wm', 'iconphoto', self._w, icon_img)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=3)
        self.columnconfigure(2, weight=1)

        left_frame = Frame(self, borderwidth=3, relief=tk.SUNKEN, width=300, height=600)
        left_frame.pack_propagate(0)
        left_frame.grid(row=0, column=0, pady=(10,10))

        self.status_label = tk.Label(self, bg="black", fg="orange", text="Status: Initializing", font=("Arial", 22), takefocus=0)
        self.status_label.grid(row=1, column=1, pady=(25,0))

        right_frame = Frame(self, borderwidth=3, relief=tk.SUNKEN, width=300, height=600)
        right_frame.pack_propagate(0)
        right_frame.grid(row=0, column=2, pady=(10,10))

        self.start_nav_slam_button = tk.Button(left_frame, text = "Nav/SLAM", takefocus=0, width=100, 
                command=self.start_nav_slam_processes)
        self.start_nav_slam_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.lane_detection_button = tk.Button(left_frame, text = "Lane Detection", takefocus=0, width=100, 
                command=self.lane_detection_button_pressed)
        self.lane_detection_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.face_detection_button = tk.Button(left_frame, text = "Face Detection", takefocus=0, width=100, 
                command=self.face_detection_button_pressed)
        self.face_detection_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.ssd_cnn_button = tk.Button(left_frame, text = "SSD CNN", takefocus=0, width=100, 
                command=self.ssd_cnn_button_pressed)
        self.ssd_cnn_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.apriltag_detection_button = tk.Button(left_frame, text = "AprilTag Detection", takefocus=0, width=100, 
                command=self.apriltag_detection_button_pressed)
        self.apriltag_detection_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        quit_button = tk.Button(left_frame, text = "Quit", takefocus=0, width=100, command=self.quit_button_callback)
        quit_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.protocol("WM_DELETE_WINDOW", self.quit_button_callback)

        #self.statusLabel.config(text="Status: Waiting for Initialization Command...")

        self.started_navigation = False
        self.started_slam = False
        self.started_lane_detection = False
        self.started_face_detection = False
        self.started_ssd_cnn = False
        self.started_apriltag_detection = False

        self.lane_detection_cancel_id = 0
        self.face_detection_cancel_id = 0
        self.ssd_cnn_cancel_id = 0
        self.apriltag_detection_cancel_id = 0

        self.ros_frame = np.zeros((640, 480, 3), np.uint8)

        self.lane_detection_image = self.ros_frame
        self.face_detection_image = self.ros_frame
        self.ssd_cnn_image = self.ros_frame
        self.apriltag_detection_image = self.ros_frame

        self.cascade_classifier = cv2.CascadeClassifier('/home/ubuntu/ros2_ws/src/pumpkin_py_pkg/pumpkin_py_pkg/haarcascade_frontalface_alt.xml')

        self.open_cv_canvas = Canvas(self, width=600, height=480, takefocus=0)  
        self.open_cv_canvas.grid(row=0, column=1, pady=(10,0))

        # Load the pre-trained neural network
        self.neural_network = cv2.dnn.readNetFromCaffe('/home/ubuntu/ros2_ws/src/pumpkin_py_pkg/pumpkin_py_pkg/MobileNetSSD_deploy.prototxt.txt', 
        '/home/ubuntu/ros2_ws/src/pumpkin_py_pkg/pumpkin_py_pkg/MobileNetSSD_deploy.caffemodel')
 
        # List of categories and classes
        self.categories = { 0: 'background', 1: 'aeroplane', 2: 'bicycle', 3: 'bird', 
               4: 'boat', 5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 
               9: 'chair', 10: 'cow', 11: 'diningtable', 12: 'dog', 
              13: 'horse', 14: 'motorbike', 15: 'person', 
              16: 'pottedplant', 17: 'sheep', 18: 'sofa', 
              19: 'train', 20: 'tvmonitor'}
 
        self.classes = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", 
            "bus", "car", "cat", "chair", "cow", 
           "diningtable",  "dog", "horse", "motorbike", "person", 
           "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
                      
        # Create the bounding boxes
        self.bbox_colors = np.random.uniform(255, 0, size=(len(self.categories), 3))

    #     self.update_canvas()
   
    # def update_canvas(self):

    #     self.status_label.config(text="Status: Processing video...")

    #     #self.current_frame = cv2.cvtColor(self.ros_frame, cv2.COLOR_BGR2RGB)
    #     self.current_frame = ImageTk.PhotoImage(Image.fromarray(self.current_frame))
    #     self.open_cv_canvas.create_image(0, 0, image = self.current_frame, anchor = tk.NW)

    #     self.after(10, self.update_canvas)
 
    def start_navigation(self):
        self.started_navigation = True
        #self.navigation_process = subprocess.Popen(["ros2", "launch", "nav2_bringup", "bringup_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/nav2_params.yaml", "map:=/home/ubuntu/bonus_room.yaml"])
        self.navigation_process = subprocess.Popen(["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/nav2_params.yaml"])
        time.sleep(1)

    def start_slam(self):
        self.started_slam = True
        #self.slam_process = subprocess.Popen(["ros2", "launch", "slam_toolbox", "offline_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_offline.yaml"])
        self.slam_process = subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_online_async.yaml"])
        #self.slam_process = subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_sync_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_online_sync.yaml"])
        #self.slam_process = subprocess.Popen(["ros2", "launch", "slam_toolbox", "localization_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_localization.yaml"])
        time.sleep(1)
 
    def quit_button_callback(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):

            self.status_label.config(text="Status: Quitting application...")
    
            if self.started_slam:
                # Try to kill the SLAM process so the node doesn't stick around
                slam_process = psutil.Process(self.slam_process.pid)
                for proc in slam_process.children(recursive=True):
                    proc.kill()
                slam_process.kill()

            # Try to kill the Navigation process so the node doesn't stick around
            if self.started_navigation:
                navigation_process = psutil.Process(self.navigation_process.pid)
                for proc in navigation_process.children(recursive=True):
                    proc.kill()
                navigation_process.kill()

            cv2.destroyAllWindows()

            self.destroy()  

    def start_nav_slam_processes(self):
        init_slam_thread = threading.Thread(target=self.start_slam)
        init_slam_thread.start()

        init_navigation_thread = threading.Thread(target=self.start_navigation)
        init_navigation_thread.start()

        #self.status_label.config(text="Status: Initialization Complete")

    def kill_processes(self):
            self.status_label.config(text="Status: Quitting Application...")
    
            # Try to kill the SLAM process so the node doesn't stick around
            if self.started_slam:
                slam_process = psutil.Process(self.slam_process.pid)
                for proc in slam_process.children(recursive=True):
                    proc.kill()
                slam_process.kill()

            # Try to kill the Navigation process so the node doesn't stick around
            if self.started_navigation:
                navigation_process = psutil.Process(self.navigation_process.pid)
                for proc in navigation_process.children(recursive=True):
                    proc.kill()
                navigation_process.kill()

            self.destroy()      

    def start_main_loop(self):
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

        image = self.ros_frame

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
    
 
    def start_lane_detection(self):
 
        self.lane_detection_image = self.get_detected_lanes(self.lane_detection_image)

        #self.lane_detection_image = cv2.cvtColor(self.lane_detection_image, cv2.COLOR_BGR2RGB)
        self.lane_detection_image = Image.fromarray(self.lane_detection_image) # to PIL format
        self.lane_detection_image = ImageTk.PhotoImage(self.lane_detection_image) # to ImageTk format

        # Update image
        self.open_cv_canvas.create_image(0, 0, anchor=tk.NW, image=self.lane_detection_image)

        # Repeat every 'interval' ms
        self.lane_detection_cancel_id = self.after(10, self.start_lane_detection)


    def lane_detection_button_pressed(self):
        if self.started_lane_detection:
            self.started_lane_detection = False
            self.lane_detection_button['text'] = 'Lane Detection'
            self.open_cv_canvas.delete("all")
            self.after_cancel(self.lane_detection_cancel_id)
        elif not self.started_face_detection:
            self.open_cv_canvas.delete("all")
            self.started_lane_detection = True
            self.lane_detection_button['text'] = 'Lane Detection*'
            self.start_lane_detection()

    #------------------------------------------------------------
    # Face Detection Methods
    #------------------------------------------------------------

    def get_detected_faces(self, image):
        
        image = self.ros_frame

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


    def start_face_detection(self):

        self.face_detection_image = self.get_detected_faces(self.face_detection_image)

        #self.face_detection_image = cv2.cvtColor(self.face_detection_image, cv2.COLOR_BGR2RGB)
        self.face_detection_image = Image.fromarray(self.face_detection_image) # to PIL format
        self.face_detection_image = ImageTk.PhotoImage(self.face_detection_image) # to ImageTk format
        # Update image
        self.open_cv_canvas.create_image(0, 0, anchor=tk.NW, image=self.face_detection_image)
        # Repeat every 'interval' ms
        self.face_detection_cancel_id = self.after(10, self.start_face_detection)


    def face_detection_button_pressed(self):
        if self.started_face_detection:
            self.started_face_detection = False
            self.face_detection_button['text'] = 'Face Detection'
            self.open_cv_canvas.delete("all")
            self.after_cancel(self.face_detection_cancel_id)
        elif not self.started_lane_detection:
            self.open_cv_canvas.delete("all")
            self.started_face_detection = True
            self.face_detection_button['text'] = 'Face Detection*'
            self.start_face_detection()

    #------------------------------------------------------------
    # SSD CNN Methods
    #------------------------------------------------------------

    def start_ssd_cnn(self):

        self.ssd_cnn_image = self.ros_frame

        (h, w) = (self.ssd_cnn_image.shape[0], self.ssd_cnn_image.shape[1])

        # Create a blob. A blob is a group of connected pixels in a binary 
        # frame that share some common property (e.g. grayscale value)
        # Preprocess the frame to prepare it for deep learning classification
        frame_blob = cv2.dnn.blobFromImage(cv2.resize(self.ssd_cnn_image, RESIZED_DIMENSIONS), 
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
         
                cv2.rectangle(self.ssd_cnn_image, (startX, startY), (
                    endX, endY), self.bbox_colors[idx], 2)     
                         
                y = startY - 15 if startY - 15 > 15 else startY + 15    
 
                cv2.putText(self.ssd_cnn_image, label, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, self.bbox_colors[idx], 2)
         
        # We now need to resize the frame so its dimensions
        # are equivalent to the dimensions of the original frame
        #frame = cv2.resize(self.ssd_cnn_image, file_size, interpolation=cv2.INTER_NEAREST)
 
        #self.ssd_cnn_image = cv2.cvtColor(self.ssd_cnn_image, cv2.COLOR_BGR2RGB)
        self.ssd_cnn_image = Image.fromarray(self.ssd_cnn_image) # to PIL format
        self.ssd_cnn_image = ImageTk.PhotoImage(self.ssd_cnn_image) # to ImageTk format

        # Update image
        self.open_cv_canvas.create_image(0, 0, anchor=tk.NW, image=self.ssd_cnn_image)

        # Repeat every 'interval' ms
        self.ssd_cnn_cancel_id = self.after(10, self.start_ssd_cnn)

    def ssd_cnn_button_pressed(self):
        if self.started_ssd_cnn:
            self.started_ssd_cnn = False
            self.ssd_cnn_button['text'] = 'SSD CNN'
            self.open_cv_canvas.delete("all")
            self.after_cancel(self.ssd_cnn_cancel_id)
        elif not self.started_lane_detection:
            self.open_cv_canvas.delete("all")
            self.started_ssd_cnn = True
            self.ssd_cnn_button['text'] = 'SSD CNN*'
            self.start_ssd_cnn()


    #------------------------------------------------------------
    # AprilTag Detection Methods
    #------------------------------------------------------------
    def get_detected_apriltags(self, image):
        
        image = self.ros_frame

        # # we have to turn the image into grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        detections = self.apriltag_detector.detect(gray_image)

        print("Saw tags {} at\n{}". \
          format([d['id']     for d in detections],
                 np.array([d['center'] for d in detections])))

        for d in detections:
            lb, rb, rt, lt = d['lb-rb-rt-lt']
            cv2.rectangle(image, (int(lt[0]), int(lt[1])), (int(rb[0]), int(rb[1])), (0, 0, 255), 1) 

            cX, cY = d['center']
            cv2.circle(image, (int(cX), int(cY)), 2, (255, 0, 0), -1)   

            cv2.putText(image, str(d['id']), (int(cX), int(cY)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1) 


        return image

    def start_apriltag_detection(self):

        self.apriltag_detection_image = self.get_detected_apriltags(self.apriltag_detection_image)

        #self.apriltag_detection_image = cv2.cvtColor(self.apriltag_detection_image, cv2.COLOR_BGR2RGB)
        self.apriltag_detection_image = Image.fromarray(self.apriltag_detection_image) # to PIL format
        self.apriltag_detection_image = ImageTk.PhotoImage(self.apriltag_detection_image) # to ImageTk format
        # Update image
        self.open_cv_canvas.create_image(0, 0, anchor=tk.NW, image=self.apriltag_detection_image)
        # Repeat every 'interval' ms
        self.apriltag_detection_cancel_id = self.after(10, self.start_apriltag_detection)


    def apriltag_detection_button_pressed(self):
        if self.started_apriltag_detection:
            self.started_apriltag_detection = False
            self.apriltag_detection_button['text'] = 'AprilTag Detection'
            self.open_cv_canvas.delete("all")
            self.after_cancel(self.apriltag_detection_cancel_id)
        elif not self.started_lane_detection:
            self.open_cv_canvas.delete("all")
            self.apriltag_detector = apriltag("tag36h11")
            self.started_apriltag_detection = True
            self.apriltag_detection_button['text'] = 'AprilTag Detection*'
            self.start_apriltag_detection()



#------------------------------------------------------------
# class DesktopUserInterfaceNode
#------------------------------------------------------------
class DesktopUserInterfaceNode(Node):
    
    def __init__(self):
        super().__init__("desktop_user_interface") 

        self.user_interface = DesktopUI()

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.video_frames_subscription = self.create_subscription(
            ROSImage, 
            'image_raw', 
            self.video_frames_listener_callback, 
            10)
        self.video_frames_subscription # prevent unused variable warning
      
        # Used to convert between ROS and OpenCV images
        self.cv_bridge = CvBridge()

        # Start spinning
        spinning_thread = threading.Thread(target=self.start_spinning)
        spinning_thread.start()

        self.user_interface.start_main_loop()
  
    def video_frames_listener_callback(self, data):
        """
        Callback function for video frame processing.
        """
        # Display the message on the console
        #self.get_logger().info('Receiving video frame')
 
        # Convert ROS Image message to OpenCV image and display it
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)
        self.user_interface.ros_frame = current_frame
    
        # Display image
        #cv2.imshow("camera", current_frame)
        #cv2.waitKey(1)

    def start_spinning(self):
        rclpy.spin(self)
        rclpy.shutdown()

#------------------------------------------------------------
# main
#------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = DesktopUserInterfaceNode()

#------------------------------------------------------------
# Entry Point
#------------------------------------------------------------
if __name__ == "__main__":
    main()

