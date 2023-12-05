#!/usr/bin/env python
import rospy
from epistemic.msg import informant_info
from epistemic.msg import child_info
from epistemic.msg import filtered_info
import random
import message_filters

# Initialize the publisher for perceived information messages
pub = rospy.Publisher('filteredInfo', filtered_info, queue_size=10)

# Define the callback function for synchronized messages
def callback(idata, cdata):
    global pub
    # Generate a random filter
    filter = random.randint(1,4)
    # Create an instance of perceived information message
    f_info = filtered_info()

    # Apply filters to create the perceived information message
    if filter != 2 and filter != 3 and filter != 4:
        f_info.type = idata.type
        f_info.age = cdata.age
        f_info.test = idata.test
        f_info.accuracy = idata.accuracy
    else:
        f_info.type = "Unfamiliar"
        f_info.age = cdata.age
        f_info.test = idata.test
        f_info.accuracy = idata.accuracy

    if filter != 1 and filter != 3 and filter != 4:
        f_info.type = idata.type
        f_info.age = cdata.age
        f_info.test = idata.test
        f_info.accuracy = idata.accuracy
    else:
        f_info.type = "Familiar"
        f_info.age = cdata.age
        f_info.test = idata.test
        f_info.accuracy = idata.accuracy

    if filter != 1 and filter != 2 and filter != 4:
        f_info.accuracy = idata.accuracy
        f_info.age = cdata.age
        f_info.test = idata.test
        f_info.type = idata.type
    else:
        f_info.accuracy = "Inaccurate"
        f_info.age = cdata.age
        f_info.test = idata.test
        f_info.type = idata.type

    if filter != 1 and filter != 2 and filter != 3:
        f_info.accuracy = idata.accuracy
        f_info.age = cdata.age
        f_info.test = idata.test
        f_info.type = idata.type
    else:
        f_info.accuracy = "Accurate"
        f_info.age = cdata.age
        f_info.test = idata.test
        f_info.type = idata.type

    f_info.id = idata.id
    # Publish the perceived information message
    pub.publish(f_info)

# Define the listener function
def listener():
    # Initialize the ROS node
    rospy.init_node('perceptionFilter', anonymous=True)
    # Subscribe to the 'humanInfo' topic
    subsciber1 = message_filters.Subscriber('informantInfo', informant_info)
    # Subscribe to the 'objectInfo' topic
    subscriber2 = message_filters.Subscriber('childInfo', child_info)
    # Synchronize the messages from both the topics
    timeSynchronizer = message_filters.ApproximateTimeSynchronizer([subsciber1, subscriber2], queue_size = 10, slop = 1, allow_headerless=True)
    timeSynchronizer.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    # Call the listener function
    listener()
