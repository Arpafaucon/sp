Header header
uint8 num_drones

# meters
float32 sight_radius
# time interval before next order
float32 period

# length = num_drones
# current drone states
float32[] current_xs
float32[] current_ys
# target drone state
float32[] target_xs
float32[] target_ys
