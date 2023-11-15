# cd ~/carla/PythonAPI/util/
# python config.py -m Town04
# sleep 4 # Time in seconds
# echo "T4AC Project" 
# roslaunch t4ac_global_planner_ros global_planner.launch map_name:=Town04 &
# sleep 2 # Time in seconds
# roslaunch t4ac_lqr_ros t4ac_lqr_ros.launch &
sleep 1 # Time in seconds
echo "Goal" 
# rostopic pub /t4ac/planning/goal geometry_msgs/PoseStamped "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: 'map'
# pose:
#   position:
#     x: -11.86
#     y: -213.34
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#     w: 1.0" &
  rostopic pub /t4ac/planning/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose: 
  position: 
    x: 86.65473937988281
    y: 175.96510314941406
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.6804877375902856
    w: 0.7327594687134753" &
sleep 1 # Time in seconds
echo "Start" 
rostopic pub /t4ac/decision_making/run_start std_msgs/Bool "data: true" &
sleep 1 # Time in seconds 