from __future__ import print_function

from typing import Callable, List, Any
from copy import copy

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

class UpdateType(Enum):
    MODESTATE = 1
    NUM_ACTIVE = 2


class DroneAllocation(object):
    def __init__(self):
        self.num_active = 0  # type: int
        self.ad_connected_drones = None  # type: List[int]
        self.cd_active_drones = None  # type: List[int]
        self.state = None  # type: List[STATE]
        self.mode = None  # type: List[MODE]

    @staticmethod
    def same_allocation(da1, da2):
        # type: (DroneAllocation, DroneAllocation) -> bool
        """
        Returns:
            bool: True if allocations are the same
        """
        if da1 is None or da2 is None:
            return False
        same = da1.num_active == da2.num_active and da1.ad_connected_drones == da2.ad_connected_drones and da1.cd_active_drones == da2.cd_active_drones
        return same


class DroneStateMachine(object):
    @staticmethod
    def new(num_connected_drones):
        # type: (int) -> DroneStateMachine
        state = [STATE.OK]*num_connected_drones
        mode = [MODE.GROUND]*num_connected_drones
        da = DroneStateMachine(state, mode)
        return da

    def __init__(self, state, mode):
        # type: (List[STATE], List[MODE]) -> None
        self._state = state
        self._mode = mode
        self._target_active = 0
        self._mode_changed = lambda *args, **kwargs: print("No mode callback set")
        self._state_changed = lambda *args, **kwargs: print("No state callback set")
        # TODO: check Thread safety
        self._update_list = []  # type: List[Any]

    def set_faulty(self, drone_index):
        # type: (int) -> None
        self._state[drone_index] = STATE.FAULTY

    def on_mode_change(self, mode_changed_cb):
        # type: (Callable[int, MODE, MODE]) -> None
        self._mode_changed = mode_changed_cb

    def on_state_change(self, state_changed_cb):
        # type: (Callable[int, STATE, STATE]) -> None
        self._state_changed = state_changed_cb

    def __str__(self):
        # type () -> str
        return "DSM[{}] -> {}\n{}\n{}".format(len(self._mode),
                                              self._target_active,
                                              '\t'.join(str(state)
                                                        for state in self._state),
                                              '\t'.join(str(mode) for mode in self._mode))

    def spin(self):
        # type: () -> DroneAllocation
        # get status updates from the outside
        self.apply_updates()

        # land all flying faulty drones
        for ix, state in enumerate(self._state):
            if state == STATE.FAULTY:
                mode = self._mode[ix]
                if mode in [MODE.TAKEOFF, MODE.ACTIVE]:
                    # landing faulty drone
                    print("landing faulty drone {}".format(ix))
                    self._set_drone(ix, mode=MODE.LANDING)

        # check if target is met
        current_active = self._mode.count(MODE.ACTIVE)\
            + self._mode.count(MODE.TAKEOFF)
        delta_active = current_active - self._target_active

        if delta_active == 0:
            pass

        elif delta_active > 0:
            # too much drones flying - must land some
            print("landing {} more drones".format(delta_active))
            candidate_drones = [True if (mode in [MODE.ACTIVE, MODE.TAKEOFF]
                                         and state == STATE.OK)
                                else False
                                for mode, state in zip(self._mode, self._state)]
            to_land = delta_active
            for drone_ix, candidate in enumerate(candidate_drones):
                if candidate:
                    self._set_drone(drone_ix, mode=MODE.LANDING)
                    to_land -= 1
                if to_land <= 0:
                    break

        elif delta_active < 0:
            # too few drones
            candidate_drones = [True if (mode in [MODE.GROUND, MODE.LANDING]
                                         and state == STATE.OK)
                                else False
                                for mode, state in zip(self._mode, self._state)]

            current_ready = candidate_drones.count(True)
            to_takeoff = min(abs(delta_active), current_ready)
            print("taking off {} more drones".format(to_takeoff))
            for drone_ix, candidate in enumerate(candidate_drones):
                if candidate:
                    self._set_drone(drone_ix, mode=MODE.TAKEOFF)
                    to_takeoff -= 1
                if to_takeoff <= 0:
                    break

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

        da.state = copy(self._state)
        da.mode = copy(self._mode)
        return da

    def _set_drone(self, drone_index, state=None, mode=None):
        # type: (int, STATE, MODE) -> None
        if state is not None:
            old_state = self._state[drone_index]
            print("{} : {} -> {}".format(drone_index,
                                         old_state, state))
            old_state = state
            self._state_changed(drone_index=drone_index,
                                old_state=old_state, new_state=state)

        if mode is not None:
            old_mode = self._mode[drone_index]
            print("{} : {} -> {}".format(drone_index,
                                         old_mode, mode))
            self._mode[drone_index] = mode
            self._mode_changed(drone_index, old_mode=old_mode,
                               new_mode=mode)

    def register_drone_update(self, drone_index, state=None, mode=None):
        # type (int, STATE, MODE) -> None
        self._update_list.append(
            (UpdateType.MODESTATE, drone_index, state, mode))

    def register_active_target_update(self, new_active_target):
        # type (int) -> None
        self._update_list.append((UpdateType.NUM_ACTIVE, new_active_target))
        print("active target is now {}".format(new_active_target))

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

    da._set_drone(1, state=STATE.FAULTY)
    da.spin()
    print(da)

    da._target_active = 0
    da.spin()
    print(da)


if __name__ == "__main__":
    test()
