#!/bin/bash
# Runtime Test Script for Task 2
# Run this script to test the Task 2 system

set -e

echo "=========================================="
echo "Task 2 Runtime Test"
echo "=========================================="
echo ""

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Check if roscore is running
echo "Step 1: Checking roscore..."
if rostopic list &>/dev/null; then
    echo "   ✓ roscore is running"
else
    echo "   ✗ roscore is NOT running"
    echo "   Please start roscore in a separate terminal:"
    echo "   source /opt/ros/noetic/setup.bash && roscore"
    exit 1
fi

echo ""
echo "Step 2: Starting Stage simulator..."
echo "   Starting stageros with willow-erratic.world..."
rosrun stage_ros stageros $(rospack find stage_ros)/world/willow-erratic.world &
STAGE_PID=$!
sleep 5
echo "   ✓ Stage simulator started (PID: $STAGE_PID)"

echo ""
echo "Step 3: Launching Task 2 system..."
roslaunch motion_control task2_delivery.launch &
LAUNCH_PID=$!
sleep 5
echo "   ✓ Task 2 launch started (PID: $LAUNCH_PID)"

echo ""
echo "Step 4: Verifying nodes are running..."
sleep 3
NODES=$(rosnode list 2>/dev/null)
if echo "$NODES" | grep -q "location_lookup_server"; then
    echo "   ✓ location_lookup_server node is running"
else
    echo "   ✗ location_lookup_server node NOT found"
fi
if echo "$NODES" | grep -q "navigation_actions"; then
    echo "   ✓ navigation_actions node is running"
else
    echo "   ✗ navigation_actions node NOT found"
fi
if echo "$NODES" | grep -q "delivery_client"; then
    echo "   ✓ delivery_client node is running"
else
    echo "   ✗ delivery_client node NOT found"
fi

echo ""
echo "Step 5: Testing location_lookup service..."
SERVICE_TEST=$(rosservice call /location_lookup "name: 'kitchen'" 2>&1)
if echo "$SERVICE_TEST" | grep -q "success: True"; then
    echo "   ✓ Service works correctly"
    echo "   Response: $SERVICE_TEST"
else
    echo "   ✗ Service test failed"
    echo "   Response: $SERVICE_TEST"
fi

echo ""
echo "Step 6: Verifying action topics..."
TOPICS=$(rostopic list 2>/dev/null)
if echo "$TOPICS" | grep -q "go_to_location"; then
    echo "   ✓ go_to_location action topics found"
    echo "$TOPICS" | grep "go_to_location" | sed 's/^/      /'
else
    echo "   ✗ go_to_location action topics NOT found"
fi
if echo "$TOPICS" | grep -q "look_at"; then
    echo "   ✓ look_at action topics found"
    echo "$TOPICS" | grep "look_at" | sed 's/^/      /'
else
    echo "   ✗ look_at action topics NOT found"
fi

echo ""
echo "Step 7: Monitoring action execution..."
echo "   Waiting 30 seconds to observe robot behavior..."
echo "   (Watch the Stage simulator window to see robot movement)"
sleep 30

echo ""
echo "Step 8: Checking action status..."
ACTION_STATUS=$(rostopic echo -n 1 /go_to_location/status 2>/dev/null | head -20 || echo "No status available")
echo "   GoToLocation status:"
echo "$ACTION_STATUS" | sed 's/^/      /' || echo "      (Status topic not available)"

echo ""
echo "=========================================="
echo "Test completed!"
echo "=========================================="
echo ""
echo "To stop the test:"
echo "  kill $STAGE_PID $LAUNCH_PID"
echo ""
echo "To check robot behavior manually:"
echo "  rostopic echo /cmd_vel"
echo "  rostopic echo /base_pose_ground_truth"

