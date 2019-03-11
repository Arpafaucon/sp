Header header

# DEPRECATED

## Convergence results
uint8 num_drones
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

