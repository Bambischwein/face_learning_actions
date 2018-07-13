#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import ServiceState

from face_learning_actions.msg import FaceLearningAction, FaceLearningGoal
from face_learning_actions.msg import FaceDetectionAction, FaceDetectionGoal
from pepper_smach.msg import NaoQi_animatedSayAction, NaoQi_animatedSayGoal

from actionlib import *
from actionlib_msgs.msg import *
from pepper_smach.srv import *
from std_msgs.msg import String

def main():
    rospy.init_node('SAY_INTRO')
    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    # Open the container
    with sm0:
         smach.StateMachine.add('FACE_DETECTION', smach_ros.SimpleActionState('/FaceDetectionServer/', FaceDetectionAction, result_slots=['face_id', 'face_name'],
                                                            goal = FaceDetectionGoal(detection_mode=0)), {'succeeded':'FACE_DETECTION', 'aborted':'FACE_LEARNING'})
         
         smach.StateMachine.add('FACE_LEARNING', smach_ros.SimpleActionState('/FaceLearningServer/', FaceLearningAction,
                    goal = FaceLearningGoal(save_files=5)), {'succeeded':'FACE_DETECTION', 'aborted':'FACE_LEARNING'})

         
         # smach.StateMachine.add('SAY_OUTRO', smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay', NaoQi_animatedSayAction,
         #            goal = NaoQi_animatedSayGoal(animatedMessage=String(data="Vielen Dank fuer Ihre Aufmerksamkeit."))))
         
         # smach.StateMachine.add('SAY_INTRO', smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay', NaoQi_animatedSayAction,
         #           goal = NaoQi_animatedSayGoal(animatedMessage=String(data="Herzlich Willkommen zu meiner Demo Gesichtserkennung."))), {'succeeded':'FACE_DETECTION'})

         # smach.StateMachine.add('SAY_NEW_FACE', smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay', NaoQi_animatedSayAction,
         #           goal = NaoQi_animatedSayGoal(animatedMessage=String(data="Ich habe ein neues Gesicht entdeckt und speichere es."))), {'succeeded':'FACE_LEARNING'})
         
         # smach.StateMachine.add('SAY_HELLO', smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay', NaoQi_animatedSayAction,
         #           goal = NaoQi_animatedSayGoal(animatedMessage=String(data="Ich kenne dich. Dein Name ist: "))), {'succeeded':'FACE_DETECTION'})



                                
        # For more examples on how to set goals and process results, see
        # executive_smach/smach_ros/tests/smach_actionlib.py
    # Execute SMACH plan
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('SIS', sm0, '/SM0')
    sis.start()
    # Execute the state machine
    outcome = sm0.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    rospy.signal_shutdown('All done.')
if __name__ == '__main__':
    main()
