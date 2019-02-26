from threading import Thread
import rospy
from drone_state_machine import DroneStateMachine, MODE, STATE

class TransitionDaemon(Thread):
    def __init__(self, drone_cid, rate_hz=.5, action="Transition"):
        # type: (int, crazyflie, float, float, float, DroneStateMachine, float)
        Thread.__init__(self)

        self.cid = drone_cid
        self.complete = False
        self.first = True
        self.rate = rospy.Rate(rate_hz)

        self.action = action

    def cb_position(self, drone_altitude):
        pass

    def send_order(self):
        pass

    def on_complete(self):
        pass

    def completed(self):
        if not self.complete:
            self.on_complete()
            self.complete = True
    
    def run(self):
        rospy.loginfo("#%d : %s started", self.cid, self.action)
        while not self.complete:
            if rospy.is_shutdown():
                return

            if not self.first:
                rospy.loginfo("#%d : resending order for '%s'", self.cid, self.action)
            self.send_order()
            self.first = False

            self.rate.sleep()

        rospy.loginfo("#%d : %s complete", self.cid, self.action)

class TakeOffDaemon(TransitionDaemon):
    def __init__(self, drone_cid, cf_instance, altitude, threshold, pm_transition_duration, modestate_machine, rate_hz=.5):
        # type: (int, crazyflie, float, float, float, DroneStateMachine, float)
        TransitionDaemon.__init__(self, drone_cid, rate_hz=rate_hz, action="TakeOff")

        self.cf_instance = cf_instance

        self.pm_altitude = altitude
        self.pm_transition_duration = pm_transition_duration
        self.pm_threshold = threshold

        self.ms_machine = modestate_machine #type: DroneStateMachine
    
    def on_complete(self):
        self.ms_machine.register_drone_update(self.cid, mode=MODE.ACTIVE)

    def cb_position(self, drone_altitude):
        if drone_altitude > self.pm_threshold:
            # takeoff complete
            self.completed()

    def send_order(self):
        self.cf_instance.takeoff(
                targetHeight=self.pm_altitude,
                duration=self.pm_transition_duration)

class LandDaemon(TransitionDaemon):
    def __init__(self, drone_cid, cf_instance, altitude, threshold, pm_transition_duration, modestate_machine, rate_hz=.5):
        # type: (int, crazyflie, float, float, float, DroneStateMachine, float)
        TransitionDaemon.__init__(self, drone_cid, rate_hz=rate_hz, action="Land")

        self.cf_instance = cf_instance

        self.pm_altitude = altitude
        self.pm_transition_duration = pm_transition_duration
        self.pm_threshold = threshold

        self.ms_machine = modestate_machine #type: DroneStateMachine
    
    def on_complete(self):
        self.ms_machine.register_drone_update(self.cid, mode=MODE.GROUND)

    def cb_position(self, drone_altitude):
        if drone_altitude < self.pm_threshold:
            # land
            self.completed()

    def send_order(self):
        self.cf_instance.land(
                targetHeight=0.,
                duration=self.pm_transition_duration)



# class TakeOffDaemon2(Thread):
#     def __init__(self, drone_cid, cf_instance, altitude, threshold, pm_transition_duration, modestate_machine, rate_hz=.5):
#         # type: (int, crazyflie, float, float, float, DroneStateMachine, float)
#         Thread.__init__(self)

#         self.cid = drone_cid
#         self.cf_instance = cf_instance
#         self.complete = False
#         self.rate = rospy.Rate(rate_hz)

#         self.pm_altitude = altitude
#         self.pm_transition_duration = pm_transition_duration
#         self.pm_threshold = threshold

#         self.ms_machine = modestate_machine #type: DroneStateMachine

#     def cb_position(self, drone_altitude):
#         if drone_altitude > self.pm_threshold:
#             # takeoff complete
#             self.ms_machine.register_drone_update(self.cid, mode=MODE.ACTIVE)
#             self.complete = True

#     def run(self):
#         rospy.loginfo("#%d : TakeOff started", self.cid)
#         while not self.complete and not rospy.is_shutdown():
#             self.cf_instance.takeoff(
#                 targetHeight=self.pm_altitude,
#                 duration=self.pm_transition_duration)
#             self.rate.sleep()
#         rospy.loginfo("#%d : TakeOff complete", self.cid)


# class LandDaemon2(Thread):
#     def __init__(self, drone_cid, cf_instance, altitude, threshold, pm_transition_duration, modestate_machine, rate_hz=.5):
#         # type: (int, crazyflie, float, float, float, DroneStateMachine, float)

#         Thread.__init__(self)

#         self.cid = drone_cid
#         self.cf_instance = cf_instance
#         self.complete = False
#         self.rate = rospy.Rate(rate_hz)

#         self.pm_altitude = altitude
#         self.pm_threshold = threshold
#         self.pm_transition_duration = pm_transition_duration

#         self.ms_machine = modestate_machine

#     def cb_position(self, drone_altitude):
#         if drone_altitude < self.pm_threshold:
#             # takeoff complete
#             self.complete = True

#     def run(self):
#         rospy.loginfo("#%d : Land started", self.cid)
#         while not self.complete:
#             if rospy.is_shutdown():
#                 return

#             self.cf_instance.land(
#                 targetHeight=0.,
#                 duration=self.pm_transition_duration)
#             self.rate.sleep()
        
#         self.ms_machine.register_drone_update(self.cid, MODE.GROUND)
#         rospy.loginfo("#%d : Land complete", self.cid)