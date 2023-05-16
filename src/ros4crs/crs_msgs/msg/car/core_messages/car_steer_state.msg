# This is a message to hold the steering data from a car.
#
# The steering data consist of the readout of the ADC on the car, which reads
# the position feedback signal from the potentiometer of the steering actuator.
#

Header header

float32 steer_angle                 # Position of the wheels in radians

int16 steer_discrete_pos            # [mV] reading of the potentiometer
int16 steer_min_pos                 # [mV] reading when wheels are right-most
int16 steer_max_pos                 # [mV] reading when wheels are left-most
