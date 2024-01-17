#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int16
from actionlib import SimpleActionClient
from mark10_trial_control.msg import RecordAction, RecordGoal
import os
import numpy as np
import time

# General idea - this is high level trial control

# Gripper_Cmd - publish gripper position/current values here
# UR_something - publish cartesian movement changes here
# __action_server - use this to start data collection
# camera_service - takes photo, saves it, and returns cable pose

class GripperControl:
    def __init__(self):
        self.num_trials = 1 # If 10 trials is desired, enter 9 (index 0)
        self.trial_duration = 10

        self.storage_directory = '/home/marcus/contactile_trials/data_01_05_24/'
    
        # Setup the gripper position publisher
        # Possible messages are "current_000" or "position_000" where 000 is the value
        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)

        # Setup the recording action client
        self.record_ac = SimpleActionClient("record_server", RecordAction)
        rospy.loginfo("Waiting for recording server to come up.")
        self.record_ac.wait_for_server()
        rospy.loginfo("Recording server up, ready!")

    def main(self):
        # Get the image number to save
        print("opening gripper")
        self.gripper_pos_pub.publish("position_10000")
        
        # print("Sending bias request to tactile sensors")
        # srv_clients.bias_request_srv_client()
        
        # Working here!

        while not rospy.is_shutdown():
            user_continue = input("Enter to start.")

            # if not int(user_continue):
            #     # os.system("rosnode kill -a")
            #     break

            # Get new data file name
            name = str(self.get_start_file_index())
            goal = RecordGoal(file_name=name)
            print('New recording file: ' + name)

            # Start recording data
            self.record_ac.send_goal(goal)

            # Close the gripper
            self.gripper_pos_pub.publish("current_4")
            
            # Sleep for the duration of the trial
            # start_time = time.time()
            rospy.sleep(self.trial_duration)
            # print("Duration: ", time.time() - start_time)

            # Stop recording data
            self.record_ac.cancel_all_goals()

            # Open the gripper
            self.gripper_pos_pub.publish("position_10000")

            # self.file_num = self.get_start_file_index()

        # rospy.spin()

    def get_start_file_index(self):
        # Returns the starting file number (old number)
        current_files = os.listdir(self.storage_directory)
        try:
            numbers = np.array([i.split('.csv', 1)[0] for i in current_files], dtype=int)
            print(numbers)
            return np.max(numbers) + 1
        except Exception as e:
            return 0


if __name__ == '__main__':
    rospy.init_node('gripper_control', anonymous=True)
    gripper_control = GripperControl()
    gripper_control.main()
