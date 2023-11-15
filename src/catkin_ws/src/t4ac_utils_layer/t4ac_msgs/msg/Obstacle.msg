# Techs4AgeCar project

Header header

geometry_msgs/Pose pose		    # Pose at the center of mass (local coordinates)
geometry_msgs/Pose global_pose  # Pose at the center of mass (local coordinates)
float64 dist2ego                # Distance to ego-vehicle
geometry_msgs/Twist twist	    # Velocity at the center of mass
geometry_msgs/Vector3 shape	    # Box containing the obstacle
int64 road_id			        # Road_id in which the obstacle is
int64 lane_id                   # From left to right, the lane in which th obstacle is
                                # in the same road
bool in_lane                    # The obstacle is in on the road (any obstacle), not in the 
                                # sidewalk, etc. 
string lane_type                # Type of the lane in which the obstacle has been detected
string type			            # Obstacle type if it is known (car, pedestrian, sign, etc)
