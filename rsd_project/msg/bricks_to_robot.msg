#This is the custom msg file, that sends the header and pose
Header header

# Dont send the pose after we wound a bug in the vision system. 
#geometry_msgs/Pose pose

# However we still send the speed which is actually a constant.  
float64 speed

#Trying to make it a bit more simple with this kind of variables.
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw


