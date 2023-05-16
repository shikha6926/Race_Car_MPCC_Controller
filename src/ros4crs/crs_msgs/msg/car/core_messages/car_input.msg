Header header
float64 torque
float64 steer
bool lap_flag

# In some cases the car's low-level steer map should be overriden (e.g., for
# system identification). If this flag is set to true, then the steer field
# is not interpreted as an angle in radians, but as a voltage reference for the
# potentiometer in the cars.
bool steer_override
