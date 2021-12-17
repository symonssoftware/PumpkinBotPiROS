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
from example_interfaces.srv import SetBool

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

        quitButton = tk.Button(leftFrame, text = "Quit", takefocus=0, width=100, command=self.quitButtonCallback)
        quitButton.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.protocol("WM_DELETE_WINDOW", self.quitButtonCallback)

        self.statusLabel.config(text="Status: Waiting for Initialization Command...")

    def startNavigation(self):
        #self.navigationProcess = subprocess.Popen(["ros2", "launch", "nav2_bringup", "bringup_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/nav2_params.yaml", "map:=/home/ubuntu/bonus_room.yaml"])
        self.navigationProcess = subprocess.Popen(["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/nav2_params.yaml"])
        time.sleep(1)

    def startSLAM(self):
        #self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "offline_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_offline.yaml"])
        self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_online_async.yaml"])
        #self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "online_sync_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_online_sync.yaml"])
        #self.slamProcess = subprocess.Popen(["ros2", "launch", "slam_toolbox", "localization_launch.py", "use_sim_time:=false", "params_file:=/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/config/slam_toolbox_mapper_params_localization.yaml"])
        time.sleep(1)
 
    def quitButtonCallback(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):

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
            slamProcess = psutil.Process(self.slamProcess.pid)
            for proc in slamProcess.children(recursive=True):
                proc.kill()
            slamProcess.kill()

            # Try to kill the Navigation process so the node doesn't stick around
            navigationProcess = psutil.Process(self.navigationProcess.pid)
            for proc in navigationProcess.children(recursive=True):
                proc.kill()
            navigationProcess.kill()

            self.destroy()      

    def startMainLoop(self):
        self.mainloop() 

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

