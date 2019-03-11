Header header

# Recap of admiral orders
# -----------------------
# all tabs have `num_drones` cells
int8 num_drones
# current coordinates for drones
float32[] current_xs
float32[] current_ys

# target coordinates for drones
float32[] target_xs
float32[] target_ys


# Captain added value
# -------------------
bool valid
# map of drone_id mapped to each target
# ideally, that should be 1..num_drones
int8[] drone_association

# distance of the path between current and target for each drone
float32[] distance
float32 total_distance

int32[] waypoints_start_indices
int32[] waypoints_stop_indices
float32[] waypoints_x
float32[] waypoints_y

