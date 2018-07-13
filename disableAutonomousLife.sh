#!/bin/bash

rosservice call /pepper_robot/pose/life/disable
sleep 6s
rosservice call /pepper_robot/pose/wakeup

