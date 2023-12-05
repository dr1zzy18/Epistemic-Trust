#!/usr/bin/env python

import sys
import rospy
from epistemic.msg import filtered_info, choice_info
import random
from epistemic.srv import choice

# Create a service proxy for the predict_robot_expression service
proxy_choice = rospy.ServiceProxy('choice_service', choice, persistent=True)
# Create a publisher for the robot_info topic
pub = rospy.Publisher('choiceInfo', choice_info, queue_size=10)

# Define a callback function to handle incoming perceived_info messages
def callback(data):
    global proxy_choice
    global pub
    # Wait for the predict_robot_expression service to become available
    rospy.wait_for_service('choice_service')
    try:
        # Call the predict_robot_expression service with the perceived_info data
        response = proxy_choice(age = data.age, test = data.test, accuracy = data.accuracy, type = data.type)
        # Create a robot_info message with the predicted probabilities
        choice_probabilities = choice_info(id = data.id, age = data.age, test_type = data.test, informant = data.type, informant_accuracy = data.accuracy, p_chooses_with_informant = response.p_choice)
        # Publish the robot_info message to the robotInfo topic
        pub.publish(choice_probabilities)
        rospy.loginfo(choice_probabilities)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Define a listener function to wait for incoming perceived_info messages
def listener():
    global choice_probabilities
    # Initialize the ROS node
    rospy.init_node('choiceController', anonymous=True)
    # Subscribe to the perceivedInfo topic and call the callback function for each message
    rospy.Subscriber('filteredInfo', filtered_info, callback,queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    listener()
