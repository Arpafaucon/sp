from __future__ import print_function

from typing import Callable, List

from enum import Enum

# not flying, ready to be controlled
STATUS_READY = "RDY"
# flying, doing missions
STATUS_ACTIVE = "ACT"
# flying, reportedly faulty. Must get to earth
STATUS_FAULTY = "FTY"
# not flying, not available
STATUS_UNAVAILABLE = "OFF"


class MODE(Enum):
    GROUND = 1
    TAKEOFF = 2
    ACTIVE = 3
    LANDING = 4


class STATE(Enum):
    OK = 10
    FAULTY = 20
# class MODE(object):
#     GROUND = "GND"
#     TAKEOFF = "TOF"
#     ACTIVE = "ACT"
#     LANDING = "LND"


# class STATE(object):
#     OK = "OK "
#     FAULTY = "FLT"

class UpdateType(object):
    MODESTATE = 1
    NUM_ACTIVE = 2


class DroneAllocation(object):
    def __init__(self):
        self.num_active: int = 0
        self.ad_connected_drones: List[int] = None
        self.cd_active_drones: List[int] = None
        self.state: List[STATE] = None
        self.mode: List[MODE] = None


class DroneStateMachine(object):
    @staticmethod
    def new(num_connected_drones: int):
        state = [STATE.OK]*num_connected_drones
        mode = [MODE.GROUND]*num_connected_drones
        da = DroneStateMachine(state, mode)
        return da

    def __init__(self, state: List[STATE], mode: List[MODE]):
        self._state = state
        self._mode = mode
        self._target_active = 0
        self._mode_changed = lambda: print("No mode callback set")
        self._state_changed = lambda: print("No state callback set")
        self._update_list = []

    def set_faulty(self, drone_index: int):
        self._state[drone_index] = STATE.FAULTY

    def on_mode_change(self, mode_changed_cb: Callable[int, MODE, MODE]):
        self._mode_changed = mode_changed_cb

    def on_state_change(self, state_changed_cb: Callable[int, STATE, STATE]):
        self._state_changed = state_changed_cb

    def __str__(self):
        return "DSM[{}] -> {}\n{}\n{}".format(len(self._mode),
                                              self._target_active,
                                              '\t'.join(self._state),
                                              '\t'.join(self._mode))

    def spin(self):
        # get status updates from the outside
        self.apply_updates()

        # land all flying faulty drones
        while STATE.FAULTY in self._state:
            faulty_drone = self._state.index(STATE.FAULTY)
            if self._mode[faulty_drone] in [MODE.TAKEOFF, MODE.ACTIVE]:
                # landing faulty drone
                self._set_drone(faulty_drone, mode=MODE.LANDING)

        # check if target is met
        current_active = self._mode.count(MODE.ACTIVE)\
            + self._mode.count(MODE.TAKEOFF)
        delta_active = current_active - self._target_active

        if delta_active == 0:
            pass

        elif delta_active > 0:
            # too much drones flying - must land some
            print("landing {}".format(delta_active))
            candidate_drones = [True if (mode in [MODE.ACTIVE, MODE.TAKEOFF]
                                         and state == STATE.OK)
                                else False
                                for mode, state in zip(self._mode, self._state)]
            for _ in range(delta_active):
                ad_index = candidate_drones.index(True)
                self._set_drone(ad_index, STATUS_READY)

        elif delta_active < 0:
            # too few drones
            candidate_drones = [True if (mode in [MODE.GROUND, MODE.LANDING]
                                         and state == STATE.OK)
                                else False
                                for mode, state in zip(self._mode, self._state)]

            current_ready = candidate_drones.count(True)
            to_takeoff = min(abs(delta_active), current_ready)
            print("taking off {}".format(to_takeoff))
            for _ in range(to_takeoff):
                ad_index = candidate_drones.index(True)
                self._set_drone(ad_index, mode=MODE.LANDING)

        return self._build_drone_allocation()

    def _build_drone_allocation(self):
        da = DroneAllocation()
        controllable_drones = [True if (mode in [MODE.ACTIVE]
                                        and state == STATE.OK)
                               else False
                               for mode, state in zip(self._mode, self._state)]
        da.num_active = controllable_drones.count(True)
        active_indices = [i for i, active in enumerate(
            controllable_drones) if active]
        da.ad_connected_drones = active_indices

        cd_active = [-1]*len(self._mode)
        for active_id, connected_id in enumerate(active_indices):
            cd_active[connected_id] = active_id

        da.cd_active_drones = cd_active

        da.state = self._state.copy()
        da.mode = self._mode.copy()

        return da

    def _set_drone(self, drone_index: int, state: STATE = None, mode: MODE = None):
        if state is not None:
            old_state = self._state[drone_index]
            print("{} : {} -> {}".format(drone_index,
                                         old_state, state))
            old_state = state
            self._mode_changed(drone_index=drone_index, old_mode=mode,
                               new_mode=None, old_state=old_state, new_state=state)

        if mode is not None:
            old_mode = self._mode[drone_index]
            print("{} : {} -> {}".format(drone_index,
                                         old_mode, mode))
            self._mode[drone_index] = mode
            self._mode_changed(drone_index, old_mode=old_mode,
                               new_mode=mode, old_state=None, new_state=None)

    def register_drone_update(self, drone_index, state: STATE = None, mode: MODE = None):
        self._update_list.append(
            (UpdateType.MODESTATE, drone_index, state, mode))

    def register_active_target_update(self, new_active_target: int):
        self._update_list.append((UpdateType.NUM_ACTIVE, new_active_target))

    def apply_updates(self):
        for update in self._update_list:
            if update[0] == UpdateType.MODESTATE:
                _, drone_index, state, mode = update
                if state is not None:
                    self._state[drone_index] = state
                if mode is not None:
                    self._mode[drone_index] = mode
            elif update[0] == UpdateType.NUM_ACTIVE:
                self._target_active = update[1]
        self._update_list = []


def test():
    da = DroneStateMachine.new(num_connected_drones=4)

    da.spin()
    print(da)

    da._target_active = 2
    da.spin()
    print(da)

    da._set_drone(1, STATUS_FAULTY)
    da.spin()
    print(da)

    da._target_active = 0
    da.spin()
    print(da)


if __name__ == "__main__":
    test()
