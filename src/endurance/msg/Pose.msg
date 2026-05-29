# Pose.msg
#
# This message contains inertial sensor data (pose, velocity, orientation, etc.)
#
# Position in X, Y, Z from InertialSensor[Nb].Pos_0[0..2]
geometry_msgs/Point position

# Velocity from InertialSensor[Nb].Vel_0[0..1] and Vehicle.v (for the Z component)
geometry_msgs/Vector3 velocity

# Orientation in X, Y, Z from Vhcl.Pitch, Vhcl.Roll, Vhcl.Yaw
geometry_msgs/Vector3 orientation

# Angular velocity (for example, using Vhcl.PitchVel, Vhcl.RollVel, Vhcl.YawRate or InertialSensor[Nb].Omega_0[0..2])
geometry_msgs/Vector3 angular_velocity

# Linear acceleration in X, Y, Z
geometry_msgs/Vector3 linear_acceleration
