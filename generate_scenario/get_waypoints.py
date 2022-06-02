import json
import math
import os

import lgsvl
import glob

def spawn(sim, x, y, speed, reverse=False):
    state = lgsvl.AgentState()
    state.transform = get_position(sim, x, y)
    if reverse:
        state.transform.rotation = state.transform.rotation - lgsvl.Vector(0, 180, 0)
    forward = lgsvl.utils.transform_to_forward(state.transform)
    state.velocity = speed * forward

    return state, state.transform.rotation


def spawn_direct(sim, x, y, speed, reverse=False):
    state = lgsvl.AgentState()
    transform = get_position(sim, x, y)
    state.transform.position = lgsvl.Vector(x, 10.2, y)
    state.transform.rotation = transform.rotation
    if reverse:
        state.transform.rotation = state.transform.rotation - lgsvl.Vector(0, 180, 0)
    forward = lgsvl.utils.transform_to_forward(state.transform)
    state.velocity = speed * forward

    return state, state.transform.rotation


def get_position(sim, x, y):
    position = lgsvl.Vector(x, 0, y)
    position = sim.map_point_on_lane(position).position
    # position.y = sim.map_point_on_lane(position).position.y
    rotation = sim.map_point_on_lane(position).rotation
    return lgsvl.Transform(position, rotation)


# def get_waypoints_time(sim, waypoints, rotation=None, idle=0):
#     npc_waypoints = []
#     for i in range(len(waypoints)):
#         transform = get_position(sim, waypoints[i][0], waypoints[i][1])
#         if i == 0:
#             if rotation is not None:
#                 npc_waypoints.append(lgsvl.DriveWaypoint(transform.position, waypoints[i][2], rotation, idle, False))
#             else:
#                 npc_waypoints.append(
#                     lgsvl.DriveWaypoint(transform.position, waypoints[i][2], transform.rotation, idle, False))
#         else:
#             transform_former = get_position(sim, waypoints[i-1][0], waypoints[i-1][1])
#             rotation = transform.rotation - transform_former.rotation
#             # rotation = transform.rotation
#             npc_waypoints.append(
#                 lgsvl.DriveWaypoint(transform.position, waypoints[i][2], rotation, 0, False))
#
#     return npc_waypoints

def get_vehicle_waypoints_time(sim, waypoints, idle=0):
    npc_waypoints = []
    for i in range(len(waypoints)):
        transform = get_position(sim, round(waypoints[i][0], 2), round(waypoints[i][1], 2))
        if i == 0:
            npc_waypoints.append(
                lgsvl.DriveWaypoint(transform.position, round(waypoints[i][2], 2), angle=transform.rotation, idle=idle, deactivate=False))
        else:
            transform_former = get_position(sim, waypoints[i - 1][0], waypoints[i - 1][1])
            rotation_vector = transform.position - transform_former.position
            rotation = lgsvl.Vector(0, math.degrees(math.atan2(rotation_vector.x, rotation_vector.z)), 0)
            # rotation = transform.position - transform_former.position
            npc_waypoints.append(
                lgsvl.DriveWaypoint(transform.position, waypoints[i][2], angle=rotation, idle=0, deactivate=False))

    return npc_waypoints

def get_waypoints_distance(sim, waypoints, rotation=None, distance=0):
    npc_waypoints = []
    for i in range(len(waypoints)):
        transform = get_position(sim, waypoints[i][0], waypoints[i][1])
        if rotation is not None:
            npc_waypoints.append(lgsvl.DriveWaypoint(transform.position, waypoints[i][2], rotation, idle=0, deactivate=False,
                                                     trigger_distance=distance))
        else:
            npc_waypoints.append(lgsvl.DriveWaypoint(transform.position, waypoints[i][2], transform.rotation, idle=0, deactivate=False,
                                                     trigger_distance=distance))

    return npc_waypoints

def get_walk_waypoints_time(sim, waypoints, idle=0):
    npc_waypoints = []
    for i in range(len(waypoints)):
        transform = get_position(sim, round(waypoints[i][0], 2), round(waypoints[i][1], 2))
        if i == 0:
            npc_waypoints.append(lgsvl.WalkWaypoint(transform.position, speed=round(waypoints[i][2], 2), idle=idle))
        else:
            npc_waypoints.append(lgsvl.WalkWaypoint(transform.position, speed=round(waypoints[i][2], 2), idle=0))

    return npc_waypoints

def get_walk_waypoints_distance(sim, waypoints, distance=0):
    npc_waypoints = []
    for i in range(len(waypoints)):
        transform = get_position(sim, waypoints[i][0], waypoints[i][1])
        npc_waypoints.append(lgsvl.WalkWaypoint(transform.position, speed=waypoints[i][2], idle=0, trigger_distance=distance))

    return npc_waypoints

def get_waypoints_direct(sim, waypoints, rotation=None, distance=0):
    npc_waypoints = []
    for i in range(len(waypoints)):
        transform_map = get_position(sim, waypoints[i][0], waypoints[i][1])
        transform = lgsvl.Vector(waypoints[i][0], 10.2, waypoints[i][1])
        if rotation is not None:
            npc_waypoints.append(lgsvl.DriveWaypoint(transform, waypoints[i][2], rotation, 0, False,
                                                     trigger_distance=distance))
        else:
            npc_waypoints.append(lgsvl.DriveWaypoint(transform, waypoints[i][2], transform_map.rotation, 0, False,
                                                     trigger_distance=distance))

    return npc_waypoints


def get_point_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def visitDir(path):
    totalSize, fileNum, dirNum = 0, 0, 0
    for lists in os.listdir(path):
        sub_path = os.path.join(path, lists)
        if os.path.isfile(sub_path):
            fileNum = fileNum+1
            totalSize = totalSize+os.path.getsize(sub_path)
        elif os.path.isdir(sub_path):
            dirNum = dirNum+1
            visitDir(sub_path)
    return fileNum

