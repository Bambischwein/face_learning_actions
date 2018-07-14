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
    sm0.userdata.face_name = 'du Arschgesicht'
   
    # Open the container
    with sm0:

        def FACE_DETECTION_GOALCB(userdata, goal):
            rospy.loginfo('FACE_DETECTION_GOALCB')
            say_goal = NaoQi_animatedSayGoal()
            say_goal.animatedMessage.data = "Hallo " + str(userdata.face_name) + ". Ich freue mich dich wiederzusehen. "
            return say_goal

        def FACE_DETECTION_RESULTCB(userdata, status, result):
            rospy.loginfo('FACE_DETECTION_RESULTCB')
            rospy.loginfo(userdata.face_name)
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo('FACE_DETECTION_RESULTCB 2')
                rospy.loginfo(result.face_name)
                userdata.face_name = result.face_name
                return 'succeeded'
            
        smach.StateMachine.add('SAY_INTRO',
                               smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay',
                                                           NaoQi_animatedSayAction,
                                                           goal = NaoQi_animatedSayGoal(animatedMessage=String(data="Herzlich Willkommen zu meiner Demo Gesichtserkennung."))),
                               {'succeeded':'FACE_DETECTION'})

        smach.StateMachine.add('FACE_DETECTION',
                               smach_ros.SimpleActionState('/FaceDetectionServer/',
                                                           FaceDetectionAction,
                                                           result_cb=FACE_DETECTION_RESULTCB,
                                                           goal = FaceDetectionGoal(detection_mode=0),
                                                           input_keys=['face_name'],
                                                           output_keys=['face_name']),
                               transitions={'succeeded':'SAY_HELLO', 'aborted':'SAY_NEW_FACE', 'preempted':'FACE_DETECTION'})

        smach.StateMachine.add('SAY_HELLO',
                               smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay',
                                                           NaoQi_animatedSayAction,
                                                           goal_cb=FACE_DETECTION_GOALCB,
                                                           input_keys=['face_name']),
                               {'succeeded':'FACE_DETECTION'})

        # smach.StateMachine.add('FACE_DETECTION',
        #                        smach_ros.SimpleActionState('/FaceDetectionServer/',
        #                                                    FaceDetectionAction,
        #                                                    result_cb=FACE_DETECTION_RESULTCB,
        #                                                    goal = FaceDetectionGoal(detection_mode=0),
        #                                                    output_keys=['face_name'],
        #                                                    input_keys=['face_name']),
        #                        {'succeeded':'SAY_HELLO', 'aborted':'SAY_NEW_FACE'},
        #                        remapping={'face_name':'userdata_output'})

        smach.StateMachine.add('FACE_LEARNING',
                               smach_ros.SimpleActionState('/FaceLearningServer/',
                                                           FaceLearningAction,
                                                           goal = FaceLearningGoal(save_files=5)),
                               {'succeeded':'SAY_HELLO', 'aborted':'FACE_LEARNING'})
        
      
        smach.StateMachine.add('SAY_NEW_FACE',
                               smach_ros.SimpleActionState('naoqi_animatedSay_server/animatedSay',
                                                           NaoQi_animatedSayAction,
                                                           goal = NaoQi_animatedSayGoal(animatedMessage=String(data="Ich habe ein neues Gesicht entdeckt und speichere es."))),
                               {'succeeded':'FACE_LEARNING'})
        


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
