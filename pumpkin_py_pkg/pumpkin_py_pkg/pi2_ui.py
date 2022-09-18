#!/usr/bin/env python3

from tkinter.constants import LEFT, RIGHT
import rclpy
from rclpy.node import Node
import tkinter as tk 
import threading
from tkinter import Canvas, Frame, messagebox
import cv2
import numpy as np
from example_interfaces.srv import SetBool
from PIL import Image, ImageTk
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImage

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

        start_camera_button = tk.Button(right_frame, text = "Start Camera", takefocus=0, width=100, command=self.start_camera_callback)
        start_camera_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        quit_button = tk.Button(right_frame, text = "Quit", takefocus=0, width=100, command=self.quit_button_callback)
        quit_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.protocol("WM_DELETE_WINDOW", self.quit_button_callback)

        self.status_label.config(text="Status: Waiting for Initialization Command...")

        self.ros_frame = np.zeros((640, 480, 3), np.uint8)

        self.open_cv_canvas = Canvas(self, width=600, height=480, takefocus=0)  
        self.open_cv_canvas.grid(row=0, column=1, pady=(10,0))

    def update_canvas(self):

        self.status_label.config(text="Status: Processing video...")

        self.current_frame = cv2.cvtColor(self.ros_frame, cv2.COLOR_BGR2RGB)
        self.current_frame = ImageTk.PhotoImage(Image.fromarray(self.current_frame))
        self.open_cv_canvas.create_image(0, 0, image = self.current_frame, anchor = tk.NW)

        self.after(10, self.update_canvas)
           
    def start_camera_callback(self):
        self.open_cv_canvas.delete("all")
        self.update_canvas()  

    def quit_button_callback(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):

            self.status_label.config(text="Status: Quitting application...")
 
            cv2.destroyAllWindows()            
            self.destroy()  
 
    def start_processes(self):
        self.status_label.config(text="Status: Initialization Complete")
        self.open_cv_canvas.delete("all")
        self.update_canvas()

    def kill_processes(self):
        self.destroy()      

    def start_main_loop(self):
        self.mainloop() 

#------------------------------------------------------------
# class Pi2UserInterfaceNode
#------------------------------------------------------------
class Pi2UserInterfaceNode(Node):
    
    def __init__(self):
        super().__init__("pi2_user_interface") 

        self.user_interface = Pi2UI()

        self.server = self.create_service(SetBool, "set_pi2_enable_flag", self.callback_set_pi2_enable_flag)

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.video_frames_publisher = self.create_publisher(ROSImage, 'video_frames', 10)
    
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.video_capture = cv2.VideoCapture(0)

        # setting the width and height of the video window
        self.video_capture.set(3, 640)
        self.video_capture.set(4, 480)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds
         
        # Create the timer
        self.video_frames_timer = self.create_timer(timer_period, self.video_frames_timer_callback)

        # Used to convert between ROS and OpenCV images
        self.cv_bridge = CvBridge()

           # Start spinning
        spinning_thread = threading.Thread(target=self.start_spinning)
        spinning_thread.start()

        self.user_interface.start_main_loop()
  
    def video_frames_timer_callback(self):
        """
        Callback function.
        This function gets called every 'timer_period' seconds.
        """

        ret, frame = self.video_capture.read()
        self.user_interface.ros_frame = frame

        if ret:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS2 image message
            if not (frame is None):

                try:
                    self.video_frames_publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
                except CvBridgeError as e:
                    # catch any errors
                    print(e)

    def start_spinning(self):
        rclpy.spin(self)
        rclpy.shutdown()

    def callback_set_pi2_enable_flag(self, request, response):
        if (request.data == True):
            self.user_interface.start_processes()
        else:           
            self.video_capture.release()
            self.user_interface.kill_processes()
            rclpy.shutdown()

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

