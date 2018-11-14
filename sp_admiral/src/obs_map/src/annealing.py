import random
import math
import time
import pygame
import pygame.locals
import obs_map.src.support as obm


from constants import *

EVT_STEP = 1
EVT_CONVERGE = 2

class AnnealingPygameDisplay(object):
    def __init__(self, observation_map):
        self.observation_map = observation_map
        pygame.init()
        h, w, _ = observation_map.shape
        self.disp_dimensions = (DISP_SCALE*w, DISP_SCALE*h)
        # self.disp_dimensions = tuple(
            # DISP_SCALE * i for i in self.map_dimensions[0:2])
        self.windowSurface = pygame.display.set_mode(
            self.disp_dimensions, pygame.HWSURFACE, 32)
        pygame.display.set_caption('Simul Annealing')

    def _pg_update_image(self, state):
        # self.windowSurface.fill(RgbColors.BLACK)
        h, w, _ = self.obs_map.shape
        for i in range(h):
            for j in range(w):
                # topleft = (i*SCALE, j*SCALE)
                rect = pygame.Rect(j*DISP_SCALE, i*DISP_SCALE,
                                   DISP_SCALE, DISP_SCALE)
                color = pygame.color.Color(0, 0, 0, 0)
                cell = self.obs_map[i, j]
                hue = int(360 * cell[0] / 255)
                saturation = int(100 * cell[1] / 255)
                value = int(100 * cell[2] / 255)
                hsv_color = (hue, saturation, value, 100)
                color.hsva = hsv_color
                pygame.draw.rect(self.windowSurface, color, rect)
        for cdrone_y, cdrone_x in state:
            rect = pygame.Rect(cdrone_x*DISP_SCALE, cdrone_y *
                               DISP_SCALE, DISP_SCALE, DISP_SCALE)
            pygame.draw.ellipse(self.windowSurface, RgbColors.RED, rect)

            see_rect = pygame.Rect((cdrone_x-DRONE_SIGHT_RADIUS)*DISP_SCALE, (cdrone_y-DRONE_SIGHT_RADIUS) *
                               DISP_SCALE, (2*DRONE_SIGHT_RADIUS+1)*DISP_SCALE, (2*DRONE_SIGHT_RADIUS+1)*DISP_SCALE)
            
            pygame.draw.ellipse(self.windowSurface, (255, 0, 0, 255), see_rect, 1)

        pygame.display.update()

    def handle_event(self, event, data):
        if event == EVT_STEP:
            pass
        elif event == EVT_CONVERGE:
            pass


class SimulAnnealingOptimisation(object):
    def __init__(self, observation_map, num_drones, display=False, interface=None):
        self.num_drones = num_drones
        self.obs_map = observation_map
        self.map_dimensions = observation_map.shape
        self.interface = interface

    def _get_drone_coordinates(self, state, drone_index):
        return state[drone_index]

    def _is_legal_drone(self, drone_x, drone_y):
        return self.obs_map[drone_x, drone_y, M_PHY] > PHY_WALL_THRES

    def _is_legal_state(self, state):
        for drone_index in range(self.num_drones):
            drone_coord = self._get_drone_coordinates(state, drone_index)
            if not self._is_legal_drone(*drone_coord):
                return False
            # two robots at the same place
            if state.count(drone_coord) > 1:
                return False
        return True

    def _neighbour_drone_positions(self, drone_x, drone_y, allow_illegal=False):
        neighbour_list = []
        deltas = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for delta in deltas:
            next_x, next_y = drone_x + delta[0], drone_y + delta[1]
            if allow_illegal or self._is_legal_drone(next_x, next_y):
                neighbour_list.append((next_x, next_y))
        return neighbour_list

    def _neighbour_state(self, state):
        moved_drone_index = random.randint(0, len(state)-1)
        moved_drone_coord = state[moved_drone_index]
        new_coords = random.choice(
            self._neighbour_drone_positions(*moved_drone_coord))
        new_state = state.copy()
        new_state[moved_drone_index] = new_coords
        return new_state

    def _score_state(self, state):
        local_obs_array = self.obs_map.copy()
        score = 0
        for drone_index, drone_coords in enumerate(state):
            score += obm.reap_obs_score(local_obs_array,
                                        drone_coords, DRONE_SIGHT_RADIUS)
        return score

    def _init_pos_drones(self, init_positions):
        if init_positions is None:
            # assert init_positions is not None, "not supported for now"
            state = []
            h, w = self.map_dimensions[:2]
            drones_to_places =self.num_drones
            for i, j in obm.spiraling_coordinates_generator(self.map_dimensions):
                if self._is_legal_drone(i, j):
                    state.append((i, j))
                    drones_to_places -= 1
                    if drones_to_places == 0:
                        return state
            print('Not enough free cell : {} unplaced drones'.format(drones_to_places))
            raise AttributeError("not enough free space")
                
        elif(self._is_legal_state(init_positions)):
            return init_positions
        else:
            raise NotImplementedError
            # return 0

    def _simul_annealing_step(self, temperature, state, current_score, num_retries = SMA_MAX_RETRY_STEP):
        for i_attempt in range(SMA_MAX_RETRY_STEP):
            proposed_state = self._neighbour_state(state)
            score = self._score_state(proposed_state)
            if(score > current_score):
                # take it
                # state = proposed_state
                return proposed_state, score
            else:
                delta = current_score - score
                proba = math.exp(-delta/temperature)
                take_it = random.random() < proba
                if(take_it):
                    # state = proposed_state
                    return proposed_state, score
        print('step stalled: no better state found after {} attempts'.format(SMA_MAX_RETRY_STEP))
        return state, score
        #     else:
        #         # try again
        #         return self._simul_annealing_step(temperature, state, current_score)

    def _simul_annealing_evt(self, initial_state, n_iteration, temp0=100):
        temp = temp0
        state = initial_state
        score = self._score_state(state)
        avg_score = 0
        while not done:
            self._simul_annealing_step(temp, state, score)
            self.interface.handle(EVT_STEP, (state, score, temp))
            temp *= 0.95
            avg_score = 0.9*avg_score+0.1*score
            if (abs(score - avg_score) / score) < 1e-3 and step > 100:
                    print("good enough, exiting")
                    pause = True
                    # done = True
        print('finished')
        self.interface.handle(EVT_STEP, (state, score, temp, ))




    def _graphic_simul_annealing(self, initial_state, n_iteration, temp0=100):
        # def on_score_step():

        temp = temp0
        state = self._init_pos_drones(initial_state)
        score = self._score_state(state)
        avg_score = score
        done = False
        step = 0
        pause = True
        self._pg_update_image(state)
        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print('exiting')
                    done = True
                    break
                elif event.type == pygame.KEYDOWN:
                    print("You pressed {:c}".format(event.key))
                    if event.key == ord(' '):
                        pause = not pause
                    elif event.key == ord('s'):
                        reaped = obm.reap_state_score(self.obs_map, state, DRONE_SIGHT_RADIUS)
                        generated = obm.score_generation_step(self.obs_map)
                        print('score update ; gen {}, reaped {}'.format(generated, reaped))
                        pause = True
                        self._pg_update_image(state)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    print(event)
                    cell_col = int(event.pos[0] / DISP_SCALE)
                    cell_line = int(event.pos[1] / DISP_SCALE)
                    d_coord = (cell_line, cell_col)
                    print(d_coord)
                    if d_coord in state:
                        print('removing robot in {}'.format(d_coord))
                        state.remove(d_coord)
                        self.num_drones -= 1
                        self._pg_update_image(state)
                    else:
                        if self._is_legal_drone(*d_coord):
                            # adding a drone
                            state.append(d_coord)
                            self.num_drones += 1
                            self._pg_update_image(state)
                            self._pg_update_image(state)

            if not pause:
                step += 1
                state, score = self._simul_annealing_step(temp, state, score)
                avg_score = 0.9*avg_score+0.1*score
                if step%20 == 0:
                    print('s {} : state={} ; score={} / {} ; temp={:.3f}'
                    .format(step,
                            state,
                            score,
                            avg_score,
                            temp))

                self._pg_update_image(state)

                if (abs(score - avg_score) / score) < 1e-3 and step > 100:
                    print("good enough, exiting")
                    pause = True
                    # done = True

                temp *= 0.95
            else:
                time.sleep(0.5)
                
        print('finished')
        pygame.quit()