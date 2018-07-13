#!/bin/bash

NAO_IP='149.201.37.46';
#NAO_IP='192.168.1.134'
#ROSCORE_IP='192.168.1.139'
ROSCORE_IP='149.201.37.169'
IFACE='wlp3s0'
#IFACE='enp0s25';

#Terminal to use
TERM_CMD='gnome-terminal'
#arguments to start new window in tab
TAB_CMD='--tab -e'

#DO NOT close Terminal if process crashes
#TODO FIND SOLUTION FOR THAT

#TODO only first param is parseble fix this for 

case "$1" in
	-w) TAB_CMD='--window -e';;
	-t) TAB_CMD='--tab -e';;
esac

echo $TAB_CMD

OPEN_CMD=$TERM_CMD 
echo "Start Pepper ROS NAOqi Interface"
OPEN_CMD="$OPEN_CMD $TAB_CMD 'bash -i -c \"roslaunch pepper_bringup pepper_full.launch nao_ip:=$NAO_IP roscore_ip:=$ROSCORE_IP network_interface:=$IFACE\" '"

echo "Start Default naoqi Actions"
OPEN_CMD="$OPEN_CMD $TAB_CMD 'bash -i -c \"sleep 10;roslaunch pepper_smach bringup_full.launch nao_ip:=$NAO_IP\" '"

echo "Start Default Face Actions"
OPEN_CMD="$OPEN_CMD $TAB_CMD 'bash -i -c \"sleep 10;roslaunch face_learning_actions face_learning_actions.launch\" '"

echo "start Web Video Server. Publish Camera stream in HTML5 friendly way"
OPEN_CMD="$OPEN_CMD $TAB_CMD 'bash -i -c \"sleep 10;roslaunch face_learning_actions web_video.launch \" '"

echo "start web server"
OPEN_CMD="$OPEN_CMD $TAB_CMD 'bash -i -c \"sleep 10; python -m SimpleHTTPServer \" '"

echo "show web site on pepper tablet"
OPEN_CMD="$OPEN_CMD $TAB_CMD 'bash -i -c \"sleep 10;./show_video.sh \" '"

# execute command string
echo $OPEN_CMD
eval $OPEN_CMD


sleep 10
# disable autonomous life
echo "disable Pepper Autonomous Life"
./disableAutonomousLife.sh

echo "********* DONE **********"
