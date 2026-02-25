#!/usr/bin/env python3

import rospy
import csv
import math
import actionlib

from kortex_driver.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianWaypoint,
    CartesianReferenceFrame
)

# ================= CONFIG ================= #

CSV_FILE = "demo datasets/vla_piack_20260225_114125.csv"

MIN_DISTANCE = 0.01       # Minimum distance between waypoints (meters)
BLENDING_RADIUS = 0.003  # Blending radius for smooth motion
BATCH_SIZE = 215            # Number of waypoints per trajectory batch

ROBOT_NAME = "my_gen3"

# ========================================== #


class CartesianTrajectoryReplayer:
    """
    Replays Cartesian trajectory from CSV file on Kinova Gen3 robot.
    """

    def __init__(self):

        rospy.init_node("cartesian_trajectory_replayer")

        action_topic = f"/{ROBOT_NAME}/cartesian_trajectory_controller/follow_cartesian_trajectory"

        self.client = actionlib.SimpleActionClient(
            action_topic,
            FollowCartesianTrajectoryAction
        )

        rospy.loginfo("Waiting for Kinova action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to Kinova controller")


    # ---------- Utility Functions ---------- #


    def distance(self, p1, p2):
        """Compute Euclidean distance between two 3D points."""
        return math.sqrt(
            (p1[0] - p2[0])**2 +
            (p1[1] - p2[1])**2 +
            (p1[2] - p2[2])**2
        )



    def create_waypoint(self, row):
        """Create CartesianWaypoint from CSV row."""

        waypoint = CartesianWaypoint()

        waypoint.pose.x = float(row['tool_pose_x'])
        waypoint.pose.y = float(row['tool_pose_y'])
        waypoint.pose.z = float(row['tool_pose_z'])

        waypoint.pose.theta_x = math.radians(float(row['tool_pose_theta_x']))
        waypoint.pose.theta_y = math.radians(float(row['tool_pose_theta_y']))
        waypoint.pose.theta_z = math.radians(float(row['tool_pose_theta_z']))

        waypoint.reference_frame = (
            CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        )

        return waypoint


    # ---------- Load Waypoints ---------- #

    def load_waypoints(self):
        """
        Load and filter waypoints from CSV.
        Removes points closer than MIN_DISTANCE.
        """

        filtered_waypoints = []
        previous_point = None

        with open(CSV_FILE, "r") as file:

            reader = csv.DictReader(file)

            for row in reader:

                x = float(row['tool_pose_x'])
                y = float(row['tool_pose_y'])
                z = float(row['tool_pose_z'])

                current_point = (x, y, z)

                # Skip very close points
                if previous_point is not None:
                    if self.distance(previous_point, current_point) < MIN_DISTANCE:
                        continue

                waypoint = self.create_waypoint(row)

                filtered_waypoints.append(waypoint)

                previous_point = current_point

        return filtered_waypoints


    # ---------- Execute Trajectory ---------- #

    def execute(self):

        waypoints = self.load_waypoints()

        rospy.loginfo(f"Executing {len(waypoints)} waypoints")

        for start in range(0, len(waypoints), BATCH_SIZE):

            batch = waypoints[start:start + BATCH_SIZE]

            goal = FollowCartesianTrajectoryGoal()

            for i, waypoint in enumerate(batch):

                wp = CartesianWaypoint()

                wp.pose = waypoint.pose
                wp.reference_frame = waypoint.reference_frame

                # Last waypoint must have zero blending
                if i == len(batch) - 1:
                    wp.blending_radius = 0.0
                else:
                    wp.blending_radius = BLENDING_RADIUS

                goal.trajectory.append(wp)

            rospy.loginfo(
                f"Executing batch {start} → {start + len(batch)}"
            )

            self.client.send_goal(goal)
            self.client.wait_for_result()

        rospy.loginfo("Trajectory execution complete")


# ---------- Main ---------- #

if __name__ == "__main__":

    try:
        replayer = CartesianTrajectoryReplayer()
        replayer.execute()

    except rospy.ROSInterruptException:
        pass