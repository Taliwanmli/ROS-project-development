#!/usr/bin/env python3
import math
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

latest = None

def pose_cb(msg: Pose):
    global latest
    latest = msg

def closed_loop_to_corner(pub, goal_x=1.0, goal_y=1.0, goal_theta=0.0):
    rate = rospy.Rate(20)                # run control at 20 Hz
    lin_k = 1.0                          # linear P gain
    ang_k = 4.0                          # angular P gain
    while not rospy.is_shutdown():
        if latest is None:
            rate.sleep(); continue

        # 1) drive to (goal_x, goal_y)
        dx, dy = goal_x - latest.x, goal_y - latest.y
        dist  = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        ang_err = (heading - latest.theta + math.pi) % (2*math.pi) - math.pi

        cmd = Twist()
        if dist > 0.05:                  # not at corner yet
            cmd.linear.x  = max(min(lin_k * dist, 2.0), -2.0)
            cmd.angular.z = max(min(ang_k * ang_err, 2.0), -2.0)
        else:
            # 2) align orientation to goal_theta when at the corner
            th_err = (goal_theta - latest.theta + math.pi) % (2*math.pi) - math.pi
            if abs(th_err) < 0.02:       # done
                pub.publish(Twist())
                rospy.loginfo("Reached corner and aligned.")
                return
            cmd.angular.z = max(min(ang_k * th_err, 1.5), -1.5)

        pub.publish(cmd)
        rate.sleep()

def main():
    rospy.init_node("motion_node")
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, pose_cb)
    rospy.loginfo("Starting closed-loop corner approach …")
    closed_loop_to_corner(pub, goal_x=1.0, goal_y=1.0, goal_theta=0.0)
    rospy.loginfo("Closed-loop step complete. Ctrl+C to exit or keep node running.")
    rospy.spin()

if __name__ == "__main__":
    main()



