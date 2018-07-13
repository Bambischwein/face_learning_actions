#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import smach
import smach_ros
from smach_ros import ServiceState


from face_learning_actions.msg import FaceDetectionAction, FaceDetectionGoal
from pepper_smach.msg import NaoQi_sayAction, NaoQi_sayGoal
from pepper_smach.msg import NaoQi_openWebsiteAction, NaoQi_openWebsiteGoal
from pepper_smach.msg import NaoQi_dialogAction, NaoQi_dialogGoal
from pepper_smach.msg import NaoQi_animatedSayAction, NaoQi_animatedSayGoal
from pepper_smach.msg import NaoQi_subscribeAction, NaoQi_subscribeGoal
from pepper_smach.msg import NaoQi_animationAction, NaoQi_animationGoal
from pepper_smach.msg import smach_teleopAction, smach_teleopGoal
from pepper_smach.msg import NaoQi_lookAtAction, NaoQi_lookAtGoal

from pepper_smach.srv import *

from actionlib import *
from actionlib_msgs.msg import *

from std_msgs.msg import String

web_server_url="192.168.1.138:8000"

def main():
    rospy.init_node('Face_Detection')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm0:

        global web_server_url

        def smach_teleop_resultCB(userdata, status, result): # result
                if status == GoalStatus.SUCCEEDED:
                        print "succeeded Transitioning -> " + result.outcome
                        return result.outcome

        smach.StateMachine.add('SHOW_BECHTLE_LOGO',
                               smach_ros.SimpleActionState('naoqi_openWebsite_server/openWebsite', NaoQi_openWebsiteAction, #Don't forget to change the picture adress
                                                       goal = NaoQi_openWebsiteGoal(url=String(data="http://"+ web_server_url +"/examples/images/C-Days_Bild.jpg"))),
                               {'succeeded':'LOOK_AT','aborted':'LOOK_AT'})

        def lookAtSetup(userdata, default_goal):
                lookAt = NaoQi_lookAtGoal()
                lookAt.position = (10.0,0.0,0.0)
                lookAt.frame = 2
                lookAt.fractionMaxSpeed = 1
                lookAt.useWholeBody = False
                return lookAt

        smach.StateMachine.add('LOOK_AT',
                               smach_ros.SimpleActionState('naoqi_lookAt_server/lookAt', NaoQi_lookAtAction,
                                                       goal_cb = lookAtSetup),
                               {'succeeded':'WAIT_FOR_TELEOP'})


	smach.StateMachine.add('WAIT_FOR_TELEOP',
                               smach_ros.SimpleActionState('smach_teleop_server/smach_teleop', smach_teleopAction,
                                                       goal = smach_teleopGoal(buttonIDs=(0,),outcomes=('out1',)), #outcomes you want to generate
                                                        result_cb = smach_teleop_resultCB, #function which is called after action finishes
                                                        outcomes={'out1'}), #register same outcomes to smach
                              transitions={'succeeded':'WAIT_FOR_TELEOP', # define transitions for custom outcomes
                                          'out1':'SAY_INTRO'}),

        smach.StateMachine.add('SAY_INTRO',
                               smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay', NaoQi_animatedSayAction,
                                                       goal = NaoQi_animatedSayGoal(animatedMessage=String(data="Guten Tag Zusammen, mein Name ist Pepper und ich wünsche viel Spaß bei der Demo."))),
                               {'succeeded':'WAIT_FOR_TELEOP_2'})

        smach.StateMachine.add('WAIT_FOR_TELEOP_2',
                               smach_ros.SimpleActionState('smach_teleop_server/smach_teleop', smach_teleopAction,
                                                       goal = smach_teleopGoal(buttonIDs=(0,),outcomes=('out1',)), #outcomes you want to generate
                                                        result_cb = smach_teleop_resultCB, #function which is called after action finishes
                                                        outcomes={'out1'}), #register same outcomes to smach
                              transitions={'succeeded':'WAIT_FOR_TELEOP_2', # define transitions for custom outcomes
                                          'out1':'SAY_OUTRO'}),

        smach.StateMachine.add('SAY_OUTRO',
                               smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay', NaoQi_animatedSayAction,
                                                       goal = NaoQi_animatedSayGoal(animatedMessage=String(data=" Auch von mir nochmal ^start(animations/Stand/Gestures/Hey_2) ein herzliches Dankeschön für Ihre Aufmerksamkeit und Ihnen allen einen schönen und interessanten Tag heute ^wait(animations/Stand/Gestures/Hey_2)"))),
                               {'succeeded':'LOOK_AT'})

   # create Introspect Server for the smash_viewer for smach_viewer
    sis = smach_ros.IntrospectionServer('smash_debug_server', sm0, '/PEPPER_DEMO_SM')
    sis.start()

   # Execute SMACH plan
    outcome = sm0.execute()

   # Wait for ctrl-c to stop application
    rospy.spin()
    sis.stop()
    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()
