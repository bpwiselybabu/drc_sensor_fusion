# This message relays pertinent information for falling. 
# It can be used to display fall stats information in GUI's.

geometry_msgs/Vector3 CoM			# center of mass coord.
geometry_msgs/Vector3 LeftFootPos	# left foot coord.
geometry_msgs/Vector3 RightFootPos	# right foot coord.
float32 leftPressure				# pressure on left
float32 rightPressure				# pressure on right
#tf/quaternion orientIMU				# orientation of IMU
geometry_msgs/Vector3 accelIMU		# linear acceleration of IMU
geometry_msgs/Vector3 velIMU		# angular velocity of IMU