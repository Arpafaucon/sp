"""
This script performs the drone setup. Two cases:


SIM mode :
Input: 
- num of drones
Result
- spawns one crazyflie_server 
- spawns one cf2 process per drone, 
- connect cf2 processes to server under the names cf0, cf1, cf2...

Real mode:
Input :
- num of drones
- drones addresses
Result:
- spawns one crazyflie_server 
- connect drones to server under the names cf0, cf1, cf2..
"""

