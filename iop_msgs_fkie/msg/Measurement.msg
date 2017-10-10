Header header

# custom name of the detecting device for differentiation
string device_name

# device manufacturing name
string device_designation

# classification of the device, e.g. atomic (A), biological (B), chemical (C), meteorologic (M)
string classification

# values of the measurement
iop_msgs_fkie/MeasurementValue[] values

# position where the measurement was taken
# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

