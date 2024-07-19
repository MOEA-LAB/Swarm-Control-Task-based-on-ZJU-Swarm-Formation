#!/bin/bash

# Function to send the message to the ROS topic
send_message() {
  local message=$1
  rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "$message"
}

# Define the message content (modify as needed)
MESSAGE_CONTENT="header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 1.0
    y: 1.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

# Send the first message
echo "First message sent"
send_message "$MESSAGE_CONTENT"
sleep 7

# Send the second message
echo "Second message sent"
send_message "$MESSAGE_CONTENT"
sleep 5

# Send the third message
echo "Third message sent"
send_message "$MESSAGE_CONTENT"
sleep 7

# Send the fourth message
echo "Fourth message"
send_message "$MESSAGE_CONTENT"


