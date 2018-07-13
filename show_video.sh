#!/bin/bash

rostopic pub /naoqi_openWebsite_server/openWebsite/goal pepper_smach/NaoQi_openWebsiteActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  url:
    data: 'http://149.201.37.169:8000/pepper_videostream.html'
  waitForWebCommand: false" 
