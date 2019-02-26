import numpy as np

def distance_to_collision(drone_pos, drone_target, collision_pos):
    assert len(drone_pos) == len(drone_target) == len(collision_pos) == 3
    point = np.zeros(3)
    point[:] = collision_pos

    line = np.zeros((2, 3))
    line[0, :] = drone_pos
    line[1, :] = drone_target

    return point_to_line_dist(point, line)
    

def point_to_line_dist(point, line):
    """Calculate the distance between a point and a line segment.

    To calculate the closest distance to a line segment, we first need to check
    if the point projects onto the line segment.  If it does, then we calculate
    the orthogonal distance from the point to the line.
    If the point does not project to the line segment, we calculate the 
    distance to both endpoints and take the shortest distance.

    credit : https://stackoverflow.com/a/45483585/7002298

    :param point: Numpy array of form [x,y,z], describing the point.
    :type point: numpy.core.multiarray.ndarray
    :param line: list of endpoint arrays of form [P1, P2]
    :type line: list of numpy.core.multiarray.ndarray
    :return: The minimum distance to a point.
    :rtype: float
    """
    # unit vector
    unit_line = line[1] - line[0]
    norm_unit_line = unit_line / np.linalg.norm(unit_line)

    # compute the perpendicular distance to the theoretical infinite line
    segment_dist = (
        np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
        np.linalg.norm(unit_line)
    )

    diff = (
        (norm_unit_line[0] * (point[0] - line[0][0])) + 
        (norm_unit_line[1] * (point[1] - line[0][1])) +
        (norm_unit_line[2] * (point[2] - line[0][2])) 
    )

    x_seg = (norm_unit_line[0] * diff) + line[0][0]
    y_seg = (norm_unit_line[1] * diff) + line[0][1]
    z_seg = (norm_unit_line[2] * diff) + line[0][2]
    

    endpoint_dist = min(
        np.linalg.norm(line[0] - point),
        np.linalg.norm(line[1] - point),

    )

    # decide if the intersection point falls on the line segment
    lp1_x = line[0][0]  # line point 1 x
    lp1_y = line[0][1]  # line point 1 y
    lp1_z = line[0][2]  # line point 1 z
    lp2_x = line[1][0]  # line point 2 x
    lp2_y = line[1][1]  # line point 2 y
    lp2_z = line[1][2]  # line point 2 z
    is_betw_x = (lp1_x <= x_seg <= lp2_x) or (lp2_x <= x_seg <= lp1_x)
    is_betw_y = (lp1_y <= y_seg <= lp2_y) or (lp2_y <= y_seg <= lp1_y)
    is_betw_z = (lp1_z <= z_seg <= lp2_z) or (lp2_z <= z_seg <= lp1_z)
    if is_betw_x and is_betw_y and is_betw_z:
        return segment_dist
    else:
        # if not, then return the minimum distance to the segment endpoints
        return endpoint_dist