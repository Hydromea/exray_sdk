# Transform from source to reference scan (latest)
geometry_msgs/Transform transform
# Covariance matrix
float64[36] covariance
# The source (previous) scan timestamp
builtin_interfaces/Time source_timestamp
# The reference scan (latest) timestamp
builtin_interfaces/Time reference_timestamp