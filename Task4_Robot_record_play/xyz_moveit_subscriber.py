#!/usr/bin/env python3
import rospy
import csv
import os
from kortex_driver.msg import BaseCyclic_Feedback
from datetime import datetime


CSV_FOLDER = "demo datasets"
os.makedirs(CSV_FOLDER, exist_ok=True)
CSV_FILE = os.path.join(CSV_FOLDER, f"vla_piack_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
counter = 0

def init_csv():
    try:
        with open(CSV_FILE, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['tool_pose_x', 'tool_pose_y', 'tool_pose_z', 'tool_pose_theta_x', 'tool_pose_theta_y', 'tool_pose_theta_z', 'finger_joint_position'])
        rospy.loginfo(f"CSV file initialized: {CSV_FILE}")
    except Exception as e:
        rospy.logerr(f"Failed to initialize CSV file: {e}")

def feedback_callback(feedback):
    print(feedback)

    try:
        # Tool pose
        x = feedback.base.tool_pose_x
        y = feedback.base.tool_pose_y
        z = feedback.base.tool_pose_z
        theta_x = feedback.base.tool_pose_theta_x
        theta_y = feedback.base.tool_pose_theta_y
        theta_z = feedback.base.tool_pose_theta_z
        try:
            finger_position = feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position
        except (IndexError, AttributeError) as e:
            finger_position = float('nan')


        global counter
        rospy.loginfo(f"{counter}: Tool Pose - x: {x:.3f}, y: {y:.3f}, z: {z:.3f}, theta x: {theta_x}, theta y: {theta_y}, theta z: {theta_z}, Finger: {finger_position:.3f}")
        counter += 1

        # Write to CSV
        with open(CSV_FILE, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([x, y, z, theta_x, theta_y, theta_z, finger_position])

    except Exception as e:
        rospy.logerr(f"Error processing feedback: {e}")

def listener():
    rospy.init_node('moveit_tool_finger_data_logger', anonymous=True)
    init_csv()
    rospy.Subscriber("/my_gen3/base_feedback", BaseCyclic_Feedback, feedback_callback)
    rospy.loginfo("Started listening to /my_gen3/base_feedback topic")

    # Keep node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
