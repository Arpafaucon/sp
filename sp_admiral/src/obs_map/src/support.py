#!usr/bin/python
"""


Image.size: width, height

array.shape: height, width
    or height, width, mode

access element in array:
array[line, column, mode]
"""
from PIL import Image
import numpy as np

from .constants import *


def disp_image(im_array, mode='HSV', scale=20):
    """
    Display the observation map

    Args:
        im_array (np.array): observation map
        mode (str, optional): Defaults to 'HSV'. image channels description ('HSV' ou '')
        scale (int, optional): Defaults to 20. upscale factor
    """
    orig = Image.fromarray(im_array, mode=mode)
    new_format = tuple([scale*dim for dim in orig.size])
    enlarged = orig.resize(new_format)
    enlarged.show()


def create_observation_map(map_filename, wall_radius=4, time_bonus=1):
    floorplan = Image.open(map_filename)
    dynamic_map = floorplan.convert("RGB").convert("HSV")
    dyn_map_array = np.array(dynamic_map)
    dyn_map_array[:, :, M_CONST] = 32
    print(dyn_map_array.shape)
    return dyn_map_array




def get_ring_coordinates(center_coord, ring_radius, array_dim):
    """
    list of coordinates of a Manhattan ring
    coordinates are given clockwise

    Args:
        center_coord (tuple): [description]
        ring_radius (int): [description]
        array_dim (tuple): [description]

    Returns:
        [type]: [description]

    Tests:
    >>> get_ring_coordinates((5, 5), 2, (10, 10, 3))
    [(3, 3), (3, 4), (3, 5), (3, 6), (3, 7), (4, 7), (5, 7), (6, 7), (7, 7), (7, 6), (7, 5), (7, 4), (7, 3), (6, 3), (5, 3), (4, 3)]

    # null radius
    >>> get_ring_coordinates((5, 5), 0, (10, 10, 3))
    [(5, 5)]

    # corner
    >>> get_ring_coordinates((0, 0), 2, (10, 10, 3))
    [(0, 2), (1, 2), (2, 2), (2, 1), (2, 0)]

    # flat image
    >>> get_ring_coordinates((0, 0), 2, (10, 1, 3))
    [(2, 0)]

    >>> get_ring_coordinates((0,0), 2, (1,1,3))
    []


    """
    assert len(
        array_dim) >= 2, "expect at least two coordinates"
    assert ring_radius >= 0
    if ring_radius == 0:
        return [center_coord]

    h, w = array_dim[0:2]
    x, y = center_coord
    top = [(x - ring_radius, y + j)
           for j in range(-ring_radius, ring_radius+1)]
    bottom = [(x + ring_radius, y - j)
              for j in range(-ring_radius, ring_radius+1)]
    left = [(x - i, y - ring_radius)
            for i in range(-ring_radius+1, ring_radius)]
    right = [(x + i, y + ring_radius)
             for i in range(-ring_radius+1, ring_radius)]
    total_nonfiltered = top + right + bottom + left
    total_filtered = [(i, j) for i, j in total_nonfiltered if i >=
                      0 and i < h and j >= 0 and j < w]
    return total_filtered


def spiraling_coordinates_generator(array_dim):
    """
    generates an outwards coordinates spiral starting from the center and cropped at the boundaries

    Args:
        array_dim (tuple[int, 2+]): dimensions of the array
    """
    h, w = array_dim[0:2]
    center_line = int(h/2)
    center_col = int(w/2)

    max_ring = max(center_line, center_col)
    for ring in range(max_ring+1):

        cell_list_ring = get_ring_coordinates((center_line, center_col), ring, (h, w))
        for cell in  cell_list_ring:
            yield cell


def test_sp(h, w):
    """
    test the spiraling generator
    """
    return list(spiraling_coordinates_generator((h, w)))


def get_visible_cells(center_coord, sight_radius, array_dim):
    """
    Return as a list the set of cells visible from an observator
    
    Args:
        center_coord (tuple[2 int]): coordinates of the observator (line, column)
        sight_radius (int): vision radius
        array_dim (tuple[2 int]): size of the observation map
    
    Returns:
        list[n tuple[2 int]]: list of coordinates of visible cells
    """
    visible_cells = []
    for ring in range(sight_radius+1):
        visible_cells += get_ring_coordinates(center_coord, ring, array_dim)
    return visible_cells


def inflate_cell(obs_array, wall_cell_coord, radius):
    h, w, nmodes = obs_array.shape
    assert nmodes == 3, 'we expect HSV images with 3 modes'
    for ring_radius in range(1, radius+1):
        ring_cell_score = int(256.*(1-math.exp(-ring_radius)))
        # print('ring {} of score {}'.format(ring_radius, ring_cell_score))
        ring_cell_list = get_ring_coordinates(
            wall_cell_coord, ring_radius, obs_array.shape)
        for i_cell, j_cell in ring_cell_list:
            obs_array[i_cell, j_cell, M_PHY] = min(
                obs_array[i_cell, j_cell, M_PHY], ring_cell_score)


def inflate_walls(observation_array, wall_radius):
    w, h, _ = observation_array.shape
    for i in range(w):
        for j in range(h):
            if observation_array[i, j, M_PHY] == 0:
                # wall
                inflate_cell(observation_array, (i, j), wall_radius)


def update_cell_score(cell_tuple):
    if cell_tuple[M_PHY] > SCOREABLE_PHY_FLOOR:
        cell_tuple[M_SCORE] += SCORE_STEP_INCREMENT
        return SCORE_STEP_INCREMENT
    return 0


def score_generation_step(observation_array):
    h, w, _ = observation_array.shape
    total = 0
    for i in range(h):
        for j in range(w):
            total += update_cell_score(observation_array[i, j])
    return total


def reap_obs_score(im_array, coordinates, sight_radius=DRONE_SIGHT_RADIUS):
    score = 0
    for i, j in get_visible_cells(coordinates, sight_radius, im_array.shape):
        score += im_array[i, j, M_SCORE]
        im_array[i, j, M_SCORE] = 0
    return score


def reap_state_score(obs_map, state, sight_radius=DRONE_SIGHT_RADIUS):
    score = 0
    for coord in state:
        score += reap_obs_score(obs_map, coord, sight_radius)
    return score


def observation_score(im_array, coordinates, radius=DRONE_SIGHT_RADIUS):
    # w, h, _ = im_array.shape
    x, y = coordinates
    radius2 = radius**2
    score = 0
    visible_cells = get_visible_cells(coordinates, radius, im_array.shape)

    for i, j in visible_cells:
        rad_ij = (x-i)**2 + (y-j)**2
        if rad_ij <= radius2:
            score += im_array[x, y, M_SCORE]

    # for i in range(h):
    #     for j in range(w):
    #         if (x-i)**2 + (y-j)**2 < radius2:
    #             score += im_array[x, y]
    return score


def enhance_with_observation_score(im_array, radius):
    oscore = im_array.copy()
    w, h, _ = im_array.shape
    for i in range(w):
        for j in range(h):
            oscore[i, j, M_SCORE] = observation_score(im_array, (i, j), radius)
    print(oscore[:, :, M_SCORE])
    print('max score {}'.format(oscore[:, :, M_SCORE].max()))
    return oscore
