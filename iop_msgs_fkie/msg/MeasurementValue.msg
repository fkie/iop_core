# name of the sensor
string sensor

# the source of the sensor data, e.g. wind, rain or other
string source

# the type of the sensor data, e.g. direction, speed, duration, intensity
string type

# one value or a spectrum of values
float64[] value
string unit

# sometimes the resulting values are accumulated, so you can specify the details
float64 min
float64 avg
float64 max

# the level of the alarm: 0-3 (no) - (highest), -1 (not supported)
int8 alert_level
# The explanation of the alarm
string alert_explanation

# a field for extra information
string extended_info
