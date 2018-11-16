import pygame
import pygame.locals

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
        h, w, _ = self.observation_map.shape
        for i in range(h):
            for j in range(w):
                # topleft = (i*SCALE, j*SCALE)
                rect = pygame.Rect(j*DISP_SCALE, i*DISP_SCALE,
                                   DISP_SCALE, DISP_SCALE)
                color = pygame.color.Color(0, 0, 0, 0)
                cell = self.observation_map[i, j]
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
