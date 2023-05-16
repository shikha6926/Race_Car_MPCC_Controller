# This is a message to hold the control action that a car applies locally on the
# embedded system.
#
# The control action consists of the power supplied to the actuators, plus
# direction. On the cars, the following conventions hold:
# - The power is normalized in the range [-1, 1], where the extremes are the
#   maximum amount of power that can be supplied to the actuator.
# - The sign denotes the direction that the torque is applied in: for driving,
#   power > 0 means forward torque and for steering, power > 0 means turning the
#   wheels to the left.
#

Header header

float32 steer_power
float32 drive_power

float32 steer_ref
float32 torque_ref
