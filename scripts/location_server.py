#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from motion_control.srv import LocationLookup, LocationLookupResponse

# Named delivery spots; coordinates should correspond to distinct rooms in the Stage world.
LOCATIONS = {
    "kitchen": Point(2.0, 2.0, 0.0),
    "mail_room": Point(8.5, 2.5, 0.0),
    "lab": Point(6.0, 8.0, 0.0),
}


def normalize_location_name(name):
    return name.strip().lower()

def resolve_location(req):
    """Return coordinates for a requested location name."""
    key = normalize_location_name(req.name)

    if key not in LOCATIONS:
        rospy.logwarn("LocationLookup: %s is unknown", req.name)
        return LocationLookupResponse(success=False, position=Point(), message="Unknown location")

    pos = LOCATIONS[key]

    return LocationLookupResponse(success=True, position=pos, message="ok")


def main():
    rospy.init_node("location_lookup_server")
    rospy.Service("location_lookup", LocationLookup, resolve_location)

    rospy.loginfo("Location lookup service ready with %d locations", len(LOCATIONS))
    rospy.spin()


if __name__ == "__main__":
    main()
