#!/bin/bash
# Test script for Task 2 verification

set -e

echo "=========================================="
echo "Task 2 Functionality Test"
echo "=========================================="

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo ""
echo "1. Checking package structure..."
PACKAGE_PATH=$(rospack find motion_control)
echo "   ✓ Package found at: $PACKAGE_PATH"

# Check required files
echo ""
echo "2. Checking required files..."
REQUIRED_FILES=(
    "$PACKAGE_PATH/package.xml"
    "$PACKAGE_PATH/CMakeLists.txt"
    "$PACKAGE_PATH/scripts/location_server.py"
    "$PACKAGE_PATH/scripts/navigation_actions.py"
    "$PACKAGE_PATH/scripts/delivery_client.py"
    "$PACKAGE_PATH/launch/task2_delivery.launch"
    "$PACKAGE_PATH/srv/LocationLookup.srv"
    "$PACKAGE_PATH/action/GoToLocation.action"
    "$PACKAGE_PATH/action/LookAt.action"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "   ✓ $(basename $file)"
    else
        echo "   ✗ MISSING: $(basename $file)"
        exit 1
    fi
done

echo ""
echo "3. Checking service definition..."
if grep -q "string name" "$PACKAGE_PATH/srv/LocationLookup.srv" && \
   grep -q "geometry_msgs/Point position" "$PACKAGE_PATH/srv/LocationLookup.srv"; then
    echo "   ✓ LocationLookup service definition correct"
else
    echo "   ✗ LocationLookup service definition incorrect"
    exit 1
fi

echo ""
echo "4. Checking action definitions..."
if grep -q "string location_name" "$PACKAGE_PATH/action/GoToLocation.action" && \
   grep -q "geometry_msgs/Point target" "$PACKAGE_PATH/action/LookAt.action"; then
    echo "   ✓ Action definitions correct"
else
    echo "   ✗ Action definitions incorrect"
    exit 1
fi

echo ""
echo "5. Checking service server implementation..."
if grep -q "rospy.Service.*location_lookup" "$PACKAGE_PATH/scripts/location_server.py" && \
   grep -q "LocationLookup" "$PACKAGE_PATH/scripts/location_server.py"; then
    echo "   ✓ Service server implements location_lookup service"
else
    echo "   ✗ Service server missing or incorrect"
    exit 1
fi

echo ""
echo "6. Checking action server implementation..."
if grep -q "go_to_location" "$PACKAGE_PATH/scripts/navigation_actions.py" && \
   grep -q "look_at" "$PACKAGE_PATH/scripts/navigation_actions.py" && \
   grep -q "location_lookup" "$PACKAGE_PATH/scripts/navigation_actions.py"; then
    echo "   ✓ Action server implements both GoToLocation and LookAt actions"
    echo "   ✓ Action server uses location_lookup service"
else
    echo "   ✗ Action server missing required functionality"
    exit 1
fi

echo ""
echo "7. Checking action client implementation..."
if grep -q "GoToLocationGoal" "$PACKAGE_PATH/scripts/delivery_client.py" && \
   grep -q "LookAtGoal" "$PACKAGE_PATH/scripts/delivery_client.py" && \
   grep -q "first_location\|second_location" "$PACKAGE_PATH/scripts/delivery_client.py"; then
    echo "   ✓ Action client implements required sequence"
else
    echo "   ✗ Action client missing required functionality"
    exit 1
fi

echo ""
echo "8. Checking launch file..."
if grep -q "location_server.py" "$PACKAGE_PATH/launch/task2_delivery.launch" && \
   grep -q "navigation_actions.py" "$PACKAGE_PATH/launch/task2_delivery.launch" && \
   grep -q "delivery_client.py" "$PACKAGE_PATH/launch/task2_delivery.launch"; then
    echo "   ✓ Launch file includes all three nodes"
    if ! grep -q "stage" "$PACKAGE_PATH/launch/task2_delivery.launch"; then
        echo "   ✓ Launch file does not start simulator (as required)"
    fi
else
    echo "   ✗ Launch file missing required nodes"
    exit 1
fi

echo ""
echo "9. Checking location definitions..."
if grep -q "kitchen\|mail_room\|lab" "$PACKAGE_PATH/scripts/location_server.py"; then
    echo "   ✓ Multiple locations defined"
    KITCHEN=$(grep -o "kitchen.*Point" "$PACKAGE_PATH/scripts/location_server.py" | head -1)
    MAILROOM=$(grep -o "mail_room.*Point" "$PACKAGE_PATH/scripts/location_server.py" | head -1)
    if [ -n "$KITCHEN" ] && [ -n "$MAILROOM" ]; then
        echo "   ✓ At least two locations in different rooms (kitchen and mail_room)"
    fi
else
    echo "   ✗ Insufficient location definitions"
    exit 1
fi

echo ""
echo "=========================================="
echo "✓ All static checks passed!"
echo "=========================================="
echo ""
echo "Next steps for runtime testing:"
echo "1. Terminal 1: roscore"
echo "2. Terminal 2: rosrun stage_ros stageros <world_file>"
echo "3. Terminal 3: roslaunch motion_control task2_delivery.launch"
echo "4. Verify nodes: rosnode list"
echo "5. Test service: rosservice call /location_lookup \"name: 'kitchen'\""
echo "6. Check actions: rostopic list | grep -E '(go_to_location|look_at)'"
echo ""

