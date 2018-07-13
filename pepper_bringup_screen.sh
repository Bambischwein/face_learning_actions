#!/bin/bash

NAO_IP='149.201.37.46';
#NAO_IP='192.168.0.250'
#ROSCORE_IP='192.168.1.114'
ROSCORE_IP='149.201.37.169'
IFACE='wlp3s0'
#IFACE='enp0s25';

#Terminal to use
TERM_CMD='gnome-terminal'
#arguments to start new window in tab
TAB_CMD='--tab -e'
# Session Name
SCREEN_SESSION='pepper_bringup'


#DO NOT close Terminal if process crashes
#TODO FIND SOLUTION FOR THAT

#TODO only first param is parseble fix this for 

case "$1" in
	-w) TAB_CMD='--window -e';;
	-t) TAB_CMD='--tab -e';;
esac

OPEN_CMD=$TERM_CMD

echo "********* STARTING STIXX PEPPER *********"

screen -AdmS $SCREEN_SESSION -t bash bash
screen -S $SCREEN_SESSION -X screen -t naoqi_driver bash -i -c "roslaunch pepper_bringup pepper_full.launch nao_ip:=$NAO_IP roscore_ip:=$ROSCORE_IP network_interface:=$IFACE"
sleep 5
screen -S $SCREEN_SESSION -X screen -t pepper_actions bash -i -c "roslaunch pepper_smach bringup_full.launch nao_ip:=$NAO_IP"
screen -S $SCREEN_SESSION -X screen -t pepper_webStream bash -i -c "roslaunch face_learning_actions web_video.launch"
screen -S $SCREEN_SESSION -X screen -t FaceDetectionAction bash -i -c "roslaunch face_learning_actions face_learning_actions.launch"
# screen -S $SCREEN_SESSION -X screen -t rosbridge bash -i -c "roslaunch rosbridge_server rosbridge_websocket.launch" 
# screen -S $SCREEN_SESSION -X screen -t teleop bash -i -c "roslaunch simple_joy simple_teleop.launch"
# screen -S $SCREEN_SESSION -X screen -t webserver bash -i -c "./start-quiz-server.sh"

# disable autonomous life
echo "disable Pepper Autonomous Life"
./disableAutonomousLife.sh
./disableExternalCollision.sh
echo "********* DONE **********"
echo "use screen -x to attatch to screen session"
