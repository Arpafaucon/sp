Header header

uint8 num_drones
# RECAP of configuration
# ----------------------
# meters
float32 sight_radius
# time interval before next order
float32 period


# Grid optimisation results
# -------------------------
# length = num_drones
# current drone states
int32[] current_is
int32[] current_js
# target drone state
int32[] target_is
int32[] target_js

## Convergence status
uint32 num_convergence_steps
float32 score
float32 avg_score


# Real world orders
# -----------------
# length = num_drones
# current drone states
float32[] current_xs
float32[] current_ys
# target drone state
float32[] target_xs
float32[] target_ys
