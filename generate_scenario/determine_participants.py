import math
from math import atan, sqrt, tan

from scenic.core.vectors import Vector
from shapely.geometry import Point

import lgsvl
from behavior_trajectory_solver.connect_lanes import combine_center_line

# define threshold of ETTC
ETTC = 2.0

def cal_coord(x1, y1, x2, y2, theta1, theta2):
    x = ((y2 - y1) - (x2 * tan(theta2) - x1 * tan(theta1))) / (tan(theta1) - tan(theta2))
    y = ((x2 - x1) - (y2 / tan(theta2) - y1 / tan(theta1))) / (1 / tan(theta1) - 1 / tan(theta2))
    return x, y

def get_coord_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1-x2, 2) + math.pow(y1-y2, 2))

def cal_TTC(network, init_ego, dst_ego, speed_ego, traj, behavior):
    ego_init_lane = network.laneAt(Vector(init_ego[0], init_ego[1]))
    ego_dest_lane = network.laneAt(Vector(dst_ego[0], dst_ego[1]))
    line_string = combine_center_line(network, ego_init_lane, ego_dest_lane)
    ego_init_project_length = line_string.project(Point(init_ego[0], init_ego[1]))
    if behavior == 'Park':
        park_length = line_string.project(Point(traj[0], traj[1]))
        ettc = abs(park_length - ego_init_project_length) / speed_ego
    else:
        ettc = 200
        new_ettc = 200
        ego_length = ego_init_project_length + speed_ego * traj[1]
        for i in range(len(traj[0]) - 1):
            ego_current = line_string.interpolate(ego_length)
            ego_next_length = ego_length + speed_ego * 0.1
            ego_next = line_string.interpolate(ego_next_length)
            theta1, theta2 = 0, 0
            if ego_next.x == ego_current.x:
                theta1 = math.pi / 2 - 0.01
            else:
                theta1 = tan((ego_next.y - ego_current.y)/(ego_next.x - ego_current.y))
            if traj[0][i + 1][0] == traj[0][i][0]:
                theta2 = math.pi / 2 - 0.01
            else:
                theta2 = tan((traj[0][i+1][1] - traj[0][i][1]) / (traj[0][i + 1][0] - traj[0][i][0]))
                
            if theta1 == theta2:
                if speed_ego != traj[0][i][2]:
                    new_ettc = sqrt(pow((traj[0][i][1] - ego_current.y), 2) + pow((traj[0][i][0] - ego_current.x), 2)) / abs(speed_ego - traj[0][i][2])
                if speed_ego == traj[0][i][2]:
                    new_ettc = 200
            else:
                new_ettc = point_ettc(ego_current.x, ego_current.y, traj[0][i][0], traj[0][i][1], theta1, theta2, speed_ego, traj[0][i][2])
            if new_ettc < ettc:
                ettc = new_ettc
            ego_length = ego_length + speed_ego * 0.1

    return ettc

def point_ettc(x1, y1, x2, y2, theta1, theta2, v1, v2):
    x_plus = (y2 - y1 + x1 * tan(theta1) - x2 * tan(theta2))/(tan(theta1 - tan(theta2)))
    y_plus = (x2 - x1 + y1/tan(theta1) - y2/tan(theta2))/(1/tan(theta1) - 1/tan(theta2))
    return (sqrt(pow(y_plus - y1, 2) + pow(x_plus - x1, 2)) / (sqrt(pow(v1, 2) + pow(v2, 2))))

def get_threshold():
    return ETTC