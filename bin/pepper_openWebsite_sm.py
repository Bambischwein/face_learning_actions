#!/usr/bin/env python

import rospy
import smach
import smach_ros

from pepper_smach.msg import NaoQi_openWebsiteAction, NaoQi_openWebsiteGoal

from actionlib import *
from actionlib_msgs.msg import *

from std_msgs.msg import String

def main():
    rospy.init_node('smach_example_actionlib')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm0:

        smach.StateMachine.add('SHOW_MASKORLOGO',
                               smach_ros.SimpleActionState('naoqi_openWebsite_server/openWebsite', NaoQi_openWebsiteAction,
                                                       goal = NaoQi_openWebsiteGoal(url=String(data="http://149.201.37.49:8000/ERIKA/erika.html"))),
                               {'succeeded':'succeeded'})

    outcome = sm0.execute()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()
