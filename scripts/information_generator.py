#!/usr/bin/env python

import rospy
from epistemic.msg import informant_info
from epistemic.msg import child_info
import random

# Initialize global ID counter
id_counter = 0

# Define the interaction generator function
def information_generator():
    # Declare the global ID counter variable
    global id_counter
    # Initialize the ROS node and publishers
    pub1 = rospy.Publisher('informantInfo', informant_info, queue_size=10)
    pub2 = rospy.Publisher('childInfo', child_info, queue_size=10)
    rospy.init_node('informationGenerator', anonymous=True)
    # Set the rate at which the loop will execute
    rate = rospy.Rate(0.1)
    # Loop until ROS is shut down
    while not rospy.is_shutdown():
        # Create instances of human_info and object_info messages
        informantinfo = informant_info()
        childinfo = child_info()
        type_values = ["Familiar","Unfamiliar"]
        accuracy_values = ["Accurate","Inaccurate"]
        test_values = ["Pretest","Posttest"]
        # Assign the current ID to each message and increment the ID counter
        informantinfo.id = id_counter
        childinfo.id = id_counter
        id_counter += 1
        # Generate random values for the message fields
        childinfo.age = random.randint(3,4)
        informantinfo.test = random.choice(test_values)
        informantinfo.accuracy = random.choice(accuracy_values)
        informantinfo.type = random.choice(type_values)
        # Publish the messages
        pub1.publish(informantinfo)
        pub2.publish(childinfo)
        # Sleep to maintain the specified loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        information_generator()
    except rospy.ROSInterruptException:
        pass
