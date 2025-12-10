#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Point

from motion_control.msg import (
    GoToLocationAction,
    GoToLocationGoal,
    LookAtAction,
    LookAtGoal,
)


def list_to_point(values):
    if not values:
        return Point()

    x = values[0] if len(values) > 0 else 0.0
    y = values[1] if len(values) > 1 else 0.0
    z = values[2] if len(values) > 2 else 0.0

    return Point(x=x, y=y, z=z)


def goto_feedback_cb(feedback):
    pass


def look_feedback_cb(feedback):
    pass


def send_goal_and_wait(client, goal, description, feedback_cb=None):
    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()

    result = client.get_result()
    if result and getattr(result, "success", False):
        rospy.loginfo("%s succeeded: %s", description, result.message)
        return True

    msg = result.message if result else "No result received"
    rospy.logwarn("%s failed or aborted: %s", description, msg)

    return False


def main():
    rospy.init_node("delivery_client")

    first_location = rospy.get_param("~first_location", "kitchen")
    second_location = rospy.get_param("~second_location", "mail_room")
    look_point = list_to_point(rospy.get_param("~look_at_point", [6.5, 6.5]))

    goto_client = actionlib.SimpleActionClient("go_to_location", GoToLocationAction)
    look_client = actionlib.SimpleActionClient("look_at", LookAtAction)

    goto_client.wait_for_server()
    look_client.wait_for_server()

    rospy.loginfo("Connected to action servers.")

    first = GoToLocationGoal(location_name=first_location)
    if not send_goal_and_wait(goto_client, first, "first GoToLocation", feedback_cb=goto_feedback_cb):
        return

    look = LookAtGoal(target=look_point)
    if not send_goal_and_wait(look_client, look, "LookAt", feedback_cb=look_feedback_cb):
        return

    second = GoToLocationGoal(location_name=second_location)

    send_goal_and_wait(goto_client, second, "second GoToLocation", feedback_cb=goto_feedback_cb)


if __name__ == "__main__":
    main()
