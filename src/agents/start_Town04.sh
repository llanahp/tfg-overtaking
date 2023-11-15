sleep 1 # Time in seconds
echo "Goal" 
rostopic pub /t4ac/planning/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: -11.86
    y: -213.34
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" &
sleep 3 # Time in seconds
echo "Start" 
rostopic pub /t4ac/decision_making/run_start std_msgs/Bool "data: true" &
sleep 1 # Time in seconds 
