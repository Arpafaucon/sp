from collections import namedtuple

M_SCORE = 1
M_CONST = 0
M_PHY = 2


def _enum(**enums):
    return type('Enum', (), enums)


# M = _enum(CONST=0,
#           SCORE=1,
#           PHY=2)



class M(object):
    CONST= 0
    SCORE=1
    PHY = 2

PHY_WALL_THRES = 5

SCOREABLE_PHY_FLOOR = 1
# SCORE_STEP_INCREMENT = 4
MAX_SCORE = 255

# DRONE_SIGHT_RADIUS = 3

Parameters = namedtuple('AdmiralParameters', [
                        # 'NUM_DRONES',
                        'WALL_RADIUS',
                        'INITIAL_TEMP',
                        'N_ITERATIONS',
                        'DRONE_SIGHT_RADIUS',
                        'RATE',
                        'SCORE_STEP_INCREMENT'
                        ])

DISP_SCALE = 25

SMA_MAX_RETRY_STEP = 20


class RgbColors(object):
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
