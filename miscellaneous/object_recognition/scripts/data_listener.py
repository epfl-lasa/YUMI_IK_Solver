#!/usr/bin/python
from subprocess import call
import easygui
import rospy
from std_msgs.msg import Int64

'''
TO DO:
- define parameters in .launch file (uising include statement). Parameters: topics, data types
- use subprocess to call record_ros service
- call subprocess using space_bar (or gui)
'''

def data_listener():

    pub = rospy.Publisher('/KUKA/Target/selector', Int64, queue_size=10)
    rospy.init_node('Target_selector', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        msg = "Select the desired position of each robot"
        choices = ["Robot_7_target_First", "Robot_7_target_Second","Robot_14_target_First", "Robot_14_target_Second"]
        reply = easygui.buttonbox(msg, choices=choices)

        if reply == 'Robot_7_target_First':
            src=71;
        if reply == 'Robot_7_target_Second':
            src=72;
        if reply == 'Robot_14_target_First':
            src=141;
        if reply == 'Robot_14_target_Second':
            src=142;

        count=1
        while (count < 20):
            rospy.loginfo(src)
            pub.publish(src)
            count=count+1
        rate.sleep()

if __name__ == '__main__':
	data_listener()