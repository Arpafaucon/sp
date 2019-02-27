Header header

int8 STATUS_OFF = 0
int8 STATUS_READY = 1
int8 STATUS_ACTIVE = 2

# total number of drones we have a link to
int8 num_drones_connected
# total of controllable drones
int8 num_drones_active

# true if allocation differs from last one
bool changed_since_last

# ------------------
# size is `num_drones_connected`. 

# Indicates if connected drone is real or simulated
bool[] connected_drones_isreal
# Each cell describes current status of connected drone. Sum of
# 10: OK - 20: Faulty
# 1: GROUND - 2: TAKEOFF - 3: ACTIVE - 4: LANDING
int8[] connected_drones_status
# Maps connected drone to an active ID, or -1 if drone is not active
int8[] connected_drones_active_ids
# ROS and TF namespace for each connected drone
string[] connected_drones_namespaces

# TODO : add info on HW address or simulation port

# ------------------
# size is `num_drones_active`. 

# for each active drone, get the connected id
int8[] active_drones_connected_ids
