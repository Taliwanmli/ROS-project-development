#!/usr/bin/env python3
import math
import rospy
import actionlib
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry

from motion_control.srv import LocationLookup
from motion_control.msg import (
    GoToLocationAction,
    GoToLocationFeedback,
    GoToLocationResult,
    LookAtAction,
    LookAtFeedback,
    LookAtResult,
)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseTracker:
    def __init__(self, odom_topic):
        self.pose = None  # (x, y, yaw)
        self._sub = rospy.Subscriber(odom_topic, Odometry, self._callback, queue_size=1)

    def _callback(self, msg):
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation)
        position = msg.pose.pose.position
        self.pose = (position.x, position.y, yaw)

    def wait_for_pose(self, timeout=5.0):
        start = rospy.Time.now()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and self.pose is None:
            if timeout and (rospy.Time.now() - start).to_sec() > timeout:
                return False

            rate.sleep()

        return self.pose is not None


class NavigationActions:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/base_pose_ground_truth")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "cmd_vel")
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.3)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.05)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.7)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.linear_gain = rospy.get_param("~linear_gain", 0.8)
        self.angular_gain = rospy.get_param("~angular_gain", 2.0)

        self.pose_tracker = PoseTracker(self.odom_topic)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        rospy.wait_for_service("location_lookup")
        self.lookup = rospy.ServiceProxy("location_lookup", LocationLookup)


        self.goto_server = actionlib.SimpleActionServer(
            "go_to_location", GoToLocationAction, execute_cb=self.handle_go_to, auto_start=False
        )
        self.lookat_server = actionlib.SimpleActionServer(
            "look_at", LookAtAction, execute_cb=self.handle_look_at, auto_start=False
        )

        self.goto_server.start()
        self.lookat_server.start()

        rospy.loginfo("NavigationActions server ready.")

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _set_goto_result(self, success, message, preempted=False):
        if preempted:
            self.goto_server.set_preempted(GoToLocationResult(success=False, message=message))
        elif success:
            self.goto_server.set_succeeded(GoToLocationResult(success=True, message=message))
        else:
            self.goto_server.set_aborted(GoToLocationResult(success=False, message=message))

    def _set_lookat_result(self, success, message, preempted=False):
        if preempted:
            self.lookat_server.set_preempted(LookAtResult(success=False, message=message))
        elif success:
            self.lookat_server.set_succeeded(LookAtResult(success=True, message=message))
        else:
            self.lookat_server.set_aborted(LookAtResult(success=False, message=message))

    def _get_location_coordinates(self, location_name):
        try:
            response = self.lookup(location_name)
            return response, None
        except rospy.ServiceException as exc:
            return None, "Location lookup failed: %s" % exc

    def handle_go_to(self, goal):
        if not self.pose_tracker.wait_for_pose():
            self._set_goto_result(False, "No odometry data available.")
            return

        response, error_msg = self._get_location_coordinates(goal.location_name)
        if error_msg:
            rospy.logerr(error_msg)
            self._set_goto_result(False, error_msg)
            return

        if not response.success:
            msg = response.message or "Unknown location"
            self._set_goto_result(False, msg)
            return

        success, text, preempted = self._drive_to(goal.location_name, response.position)
        self._set_goto_result(success, text, preempted)

    def _get_current_pose(self):
        return self.pose_tracker.pose

    def _compute_distance_and_heading(self, target, pose):
        dx = target.x - pose[0]
        dy = target.y - pose[1]
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        return dx, dy, dist, heading

    def _compute_velocity_commands(self, dist, heading_error):
        linear_vel = max(min(self.linear_gain * dist, self.max_linear_speed), -self.max_linear_speed)
        angular_vel = max(min(self.angular_gain * heading_error, self.max_angular_speed), -self.max_angular_speed)
        return linear_vel, angular_vel


    def _drive_to(self, name, target):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose = self._get_current_pose()
            if pose is None:
                rate.sleep()
                continue

            dx, dy, dist, heading = self._compute_distance_and_heading(target, pose)

            if dist < self.distance_tolerance:
                self.publish_stop()

                return True, "Reached %s" % name, False

            heading_error = normalize_angle(heading - pose[2])
            linear_vel, angular_vel = self._compute_velocity_commands(dist, heading_error)


            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel

            self.cmd_pub.publish(cmd)

            feedback = GoToLocationFeedback(distance_remaining=dist)
            self.goto_server.publish_feedback(feedback)

            if self.goto_server.is_preempt_requested():
                self.publish_stop()
                return False, "GoToLocation preempted", True

            rate.sleep()

        self.publish_stop()
        return False, "GoToLocation interrupted by shutdown", False

    def handle_look_at(self, goal):
        if not self.pose_tracker.wait_for_pose():
            self._set_lookat_result(False, "No odometry data available.")
            return

        success, text, preempted = self._turn_toward(goal.target)
        self._set_lookat_result(success, text, preempted)

    def _check_target_at_position(self, dx, dy):
        return abs(dx) < 1e-3 and abs(dy) < 1e-3

    def _turn_toward(self, target):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose = self._get_current_pose()
            if pose is None:
                rate.sleep()
                continue

            dx = target.x - pose[0]
            dy = target.y - pose[1]
            if self._check_target_at_position(dx, dy):
                self.publish_stop()
                return False, "Target point equals robot position; orientation undefined", False

            desired_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(desired_heading - pose[2])

            if abs(heading_error) < self.angle_tolerance:
                self.publish_stop()

                return True, "Oriented toward target", False

            _, angular_vel = self._compute_velocity_commands(0.0, heading_error)

            cmd = Twist()
            cmd.angular.z = angular_vel

            self.cmd_pub.publish(cmd)

            feedback = LookAtFeedback(angle_error=abs(heading_error))
            self.lookat_server.publish_feedback(feedback)

            if self.lookat_server.is_preempt_requested():
                self.publish_stop()
                return False, "LookAt preempted", True

            rate.sleep()

        self.publish_stop()
        return False, "LookAt interrupted by shutdown", False


def main():
    rospy.init_node("navigation_actions")
    NavigationActions()
    rospy.spin()


if __name__ == "__main__":
    main()
