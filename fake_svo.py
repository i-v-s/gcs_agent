#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from svo_msgs.msg import Info
import random

stage = 0
def on_key(key):
    global stage
    print 'key:' + key.data
    if key.data == 'r' or key.data == 'R':
        stage = 0
    elif stage == 0 and (key.data == 's' or key.data == 'S'):
        stage = 1


def fake_svo():
    global stage
    rospy.init_node('fake_svo', anonymous = True)
    pub_info = rospy.Publisher('/svo/info', Info, queue_size = 10)
    rospy.Subscriber('/svo/remote_key', String, on_key)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if stage == 1 and random.random() > 0.7:
            stage = 2
        if stage == 2 and random.random() > 0.95:
            stage = 3
        if stage == 3 and random.random() > 0.95:
            stage = 4
        if stage == 4 and random.random() > 0.9:
            stage = 3
        if stage == 3 and random.random() > 0.95:
            stage = 0
        pub_info.publish(stage = stage)
        rate.sleep()


if __name__ == '__main__':
    try:
        fake_svo()
    except rospy.ROSInterruptException:
        pass
