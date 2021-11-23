#!/usr/bin/env python3

from tkinter.constants import LEFT, RIGHT
import math
import rclpy
from rclpy.node import Node
import tkinter as tk 
import threading
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import sensor_msgs.msg
import RPi.GPIO as GPIO
import time
import subprocess
import psutil
from tkinter import Canvas, Frame, IntVar, PhotoImage, Radiobutton, messagebox

#------------------------------------------------------------
# class RobotUserInterface
#------------------------------------------------------------
class RobotUserInterface(tk.Tk):

    def __init__(self, robotState):
        super().__init__()
        self.setRobotState(robotState)

        self.lidarDemoStarted = False
        self.lastZAxisValue = 0.0

        # Create the main window
        self.title("Pumpkin Bot")

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

        self.robotStateLabel = tk.Label(leftFrame, text="Robot State", takefocus=0)
        self.robotStateLabel.pack(anchor="nw", pady=(10,2))

        self.robotStateSelection = IntVar()
        
        self.disableRobotButton = Radiobutton(leftFrame, text="Disable Robot", variable=self.robotStateSelection, takefocus=0, value=0,
                    command=self.robotStateSelectionCallback)
        self.disableRobotButton.pack(anchor="nw")

        self.teleopRobotButton = Radiobutton(leftFrame, text="Teleop Robot", variable=self.robotStateSelection, takefocus=0, value=1,
                    command=self.robotStateSelectionCallback)
        self.teleopRobotButton.pack(anchor="nw")

        self.autoRobotButton = Radiobutton(leftFrame, text = "Auto Robot", variable=self.robotStateSelection, takefocus=0, value=2,
                    command=self.robotStateSelectionCallback)
        self.autoRobotButton.pack(anchor="nw")
        self.autoRobotButton.invoke() #TODO - REMOVE THIS LATER

        self.imuLabel = tk.Label(leftFrame, text="ZAxis: Unknown", takefocus=0)
        self.imuLabel.pack(anchor ="nw", pady=(10,10))

        self.pumpkinCanvas = Canvas(self, width=600, height=600, takefocus=0)  
        self.pumpkinCanvas.grid(row=0, column=1, pady=(10,0))

        self.pumpkinDeadImg = PhotoImage(file='/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/launch/pumpkin_dead.png')
        self.pumpkinLeftImg = PhotoImage(file='/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/launch/pumpkin_left.png')
        self.pumpkinCenterImg = PhotoImage(file='/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/launch/pumpkin_center.png')
        self.pumpkinRightImg = PhotoImage(file='/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/launch/pumpkin_right.png')

        self.pumpkinImageID = self.pumpkinCanvas.create_image(0, 0, anchor="nw", image=self.pumpkinDeadImg) 

        self.statusLabel = tk.Label(self, bg="black", fg="orange", text="Status: Initializing", font=("Arial", 22), takefocus=0)
        self.statusLabel.grid(row=1, column=1, pady=(25,0))

        rightFrame = Frame(self, borderwidth=3, relief=tk.SUNKEN, width=300, height=600)
        rightFrame.pack_propagate(0)
        rightFrame.grid(row=0, column=2, pady=(10,10))

        self.lidarDemoButton = tk.Button(rightFrame, text = "LIDAR Demo", takefocus=0, width=100, command=self.startLidarDemoButtonCallback)
        self.lidarDemoButton.pack(anchor="center", padx=(10,10), pady=(10,10))

        quitButton = tk.Button(rightFrame, text = "Quit", takefocus=0, width=100, command=self.quitButtonCallback)
        quitButton.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.RESET_CONTROL_PIN = 3 # physical pin 5

        self.protocol("WM_DELETE_WINDOW", self.quitButtonCallback)

        initMicroROSThread = threading.Thread(target=self.initMicroROS)
        initMicroROSThread.start()

        initNavigationThread = threading.Thread(target=self.startNavigation)
        initNavigationThread.start()

        initSlamThread = threading.Thread(target=self.startSLAM)
        initSlamThread.start()

    def getRobotState(self):
        return self._robotState

    def setRobotState(self,value):
        self._robotState = value

    def setupPiPins(self):
        GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
        GPIO.setup(self.RESET_CONTROL_PIN, GPIO.OUT)
    
    def powerOffTeensy(self):
        GPIO.output(self.RESET_CONTROL_PIN, GPIO.LOW)
        time.sleep(6)
        GPIO.output(self.RESET_CONTROL_PIN, GPIO.HIGH)

    def startAgent(self):
        # Default baud rate is 9600, make sure you bump it up to 115200
        #self.microROSAgentProcess = subprocess.Popen(["ros2", "run", "micro_ros_agent", "micro_ros_agent", "serial", "--dev", "/dev/ttyACM0", "-b", "115200", "-v6"])
        self.microROSAgentProcess = subprocess.Popen(["ros2", "run", "micro_ros_agent", "micro_ros_agent", "serial", "--dev", "/dev/ttyACM0", "-b", "115200", "-v4"])
        time.sleep(3)

    def powerOnTeensy(self):
        GPIO.output(self.RESET_CONTROL_PIN, GPIO.LOW)
        time.sleep(3)
        GPIO.output(self.RESET_CONTROL_PIN, GPIO.HIGH)

    def initMicroROS(self):
        self.setupPiPins()
        self.statusLabel.config(text="Status: Powering off Teensy board...")
        self.powerOffTeensy()
        self.statusLabel.config(text="Status: Starting MicroROS agent...")
        self.startAgent()
        self.statusLabel.config(text="Status: Powering on Teensy board...")
        self.powerOnTeensy()
        self.statusLabel.config(text="Status: Initialization complete!")

    def startNavigation(self):
        #self.navigationProcess = subprocess.Popen(["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=false"])
        self.navigationProcess = subprocess.Popen(["ros2", "launch", "nav2_bringup", "bringup_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/nav2_params.yaml", "map:=/home/ubuntu/bonus_room.yaml"])
        #self.navigationProcess = subprocess.Popen(["ros2", "launch", "nav2_bringup", "localization_launch.py", "use_sim_time:=false"])
        time.sleep(1)

    def startSLAM(self):
        self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "offline_launch.py", "use_sim_time:=false"])
        #self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=false"])
        #self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "localization_launch.py", "use_sim_time:=false"])
        time.sleep(1)

    def startLidarDemoButtonCallback(self):
        self.lidarDemoStarted = True
        self.lidarDemoProcess = subprocess.Popen(["ros2", "launch", "rplidar_ros2", "view_rplidar_launch.py"])

    def robotStateSelectionCallback(self):
        self._robotState = self.robotStateSelection.get()

    def quitButtonCallback(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.pumpkinImg = PhotoImage(file='/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/launch/pumpkin_dead.png')
            self.pumpkinCanvas.itemconfig(self.pumpkinImageID, image=self.pumpkinImg)

            self.statusLabel.config(text="Status: Quitting application...")
    
            # Try to kill the SLAM process so the node doesn't stick around
            slamProcess = psutil.Process(self.slamProcess.pid)
            for proc in slamProcess.children(recursive=True):
                proc.kill()
            slamProcess.kill()

            # Try to kill the Navigation process so the node doesn't stick around
            navigationProcess = psutil.Process(self.navigationProcess.pid)
            for proc in navigationProcess.children(recursive=True):
                proc.kill()
            navigationProcess.kill()

            # Try to kill the microROS agent so the node doesn't stick around
            microROSProcess = psutil.Process(self.microROSAgentProcess.pid)
            for proc in microROSProcess.children(recursive=True):
                proc.kill()
            microROSProcess.kill()

            #Try to kill the LIDAR Demo so the node doesn't stick around
            if (self.lidarDemoStarted):
                self.lidarProcess = psutil.Process(self.lidarDemoProcess.pid)
                for proc in self.lidarProcess.children(recursive=True):
                    proc.kill()
                self.lidarProcess.kill()

            GPIO.cleanup()
            self.destroy()   

    def updateImageForTurningLeft(self):
            self.pumpkinCanvas.itemconfig(self.pumpkinImageID, image=self.pumpkinLeftImg)

    def updateImageForTurningRight(self):
            self.pumpkinCanvas.itemconfig(self.pumpkinImageID, image=self.pumpkinRightImg)

    def updateImageForGoingStraight(self):
            self.pumpkinCanvas.itemconfig(self.pumpkinImageID, image=self.pumpkinCenterImg)

    def startMainLoop(self):
        self.mainloop()

#------------------------------------------------------------
# class RobotUserInterfaceNode
#------------------------------------------------------------
class RobotUserInterfaceNode(Node):
    
    def __init__(self):
        super().__init__("robot_user_interface") 

        self.lastZAxisValue = 0.0
        self.robotState = 2 # Autonomous for now
        self.rollX = 0.0
        self.pitchY = 0.0
        self.yawZ = 0.0
        
        self.imuDataSubscriber = self.create_subscription(sensor_msgs.msg.Imu, "/imu/data", self.callback_imu_data, 10)

        self.declare_parameter("robot_state_to_publish", 0) # 0 = disabled and the default value
        self.declare_parameter("robot_state_to_publish_frequency", 1.0)

        self.publishedRobotState = self.get_parameter("robot_state_to_publish").value
        self.robotStatePublishFrequency = self.get_parameter("robot_state_to_publish_frequency").value

        self.robotStatePublisher = self.create_publisher(Int32, "robot_state", 10) #10 is the msg queue depth/buffer

        # The (1.0 / frequency) will give you the 'period' in seconds to publish the data
        self.robotStateTimer = self.create_timer(1.0 / self.robotStatePublishFrequency, self.publishRobotState)

        self.userInterface = RobotUserInterface(self.robotState)

        spinningThread = threading.Thread(target=self.startSpinning)
        spinningThread.start()

        self.userInterface.startMainLoop()

    def callback_imu_data(self, msg):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z)
        t1 = +1.0 - 2.0 * (msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y)
        self.rollX = math.atan2(t0, t1)
     
        t2 = +2.0 * (msg.orientation.w * msg.orientation.y - msg.orientation.z * msg.orientation.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.pitchY = math.asin(t2)
     
        t3 = +2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y)
        t4 = +1.0 - 2.0 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
        self.yawZ = math.atan2(t3, t4)

        prettyYawZ = round(math.degrees(self.yawZ), 2)

        self.userInterface.imuLabel.config(text = "ZAxis: " + format(prettyYawZ, '.2f'))

        if ((prettyYawZ - self.lastZAxisValue) > 0.0):
            self.userInterface.updateImageForTurningLeft()
        elif ((prettyYawZ - self.lastZAxisValue) < 0.0):
            self.userInterface.updateImageForTurningRight()
        else:
            self.userInterface.updateImageForGoingStraight()

        self.lastZAxisValue = prettyYawZ

    def publishRobotState(self):
        msg = Int32()
        msg.data = self.userInterface.getRobotState()
        self.robotStatePublisher.publish(msg)

    def startSpinning(self):
        rclpy.spin(self)
        rclpy.shutdown()

#------------------------------------------------------------
# main
#------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RobotUserInterfaceNode()

#------------------------------------------------------------
# Entry Point
#------------------------------------------------------------
if __name__ == "__main__":
    main()

