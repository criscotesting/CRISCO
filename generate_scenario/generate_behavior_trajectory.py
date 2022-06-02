import random

from func_timeout import func_set_timeout

from CRISCO.generate_scenario.behavior_trajectory_solver.cut_in import cut_in
from CRISCO.generate_scenario.behavior_trajectory_solver.cut_out import cut_out
from CRISCO.generate_scenario.behavior_trajectory_solver.follow_lane import follow_lane
from CRISCO.generate_scenario.behavior_trajectory_solver.follow_vehicle import follow_vehicle
from CRISCO.generate_scenario.behavior_trajectory_solver.park import park
from CRISCO.generate_scenario.behavior_trajectory_solver.pedestrian_cross import pedestrian_cross
from CRISCO.generate_scenario.behavior_trajectory_solver.pedestrian_walk import pedestrian_walk
from CRISCO.generate_scenario.behavior_trajectory_solver.retrograde import retrograde
from CRISCO.generate_scenario.behavior_trajectory_solver.turn_around import turn_around
from CRISCO.generate_scenario.behavior_trajectory_solver.vehicle_cross import vehicle_cross
from CRISCO.generate_scenario.get_waypoints import *

# @func_set_timeout(10)
def generate_trajectory(network, road_type, behavior, ego_init, ego_dest, ego_speed, num):
    if behavior == 'Follow Vehicle':
        return follow_vehicle(network, road_type, ego_init, ego_dest, ego_speed, num)
    elif behavior == 'Follow Lane':
        return follow_lane(network, road_type, ego_init, ego_dest, ego_speed, num)
    elif behavior == 'Cut In':
        return cut_in(network, road_type, ego_init, ego_dest, ego_speed, num)
    elif behavior == 'Cut Out':
        return cut_out(network, road_type, ego_init, ego_dest, ego_speed, num)
    elif behavior == 'Retrograde':
        return retrograde(network, road_type, ego_init, ego_dest, ego_speed, num)
    elif behavior == 'Park':
        return park(network, road_type, ego_init, ego_dest, num)
    elif behavior == 'Pedestrian Cross':
        return pedestrian_cross(network, road_type, ego_init, ego_dest, ego_speed, num)
    elif behavior == 'Pedestrian Walk':
        return pedestrian_walk(network, road_type, ego_init, ego_dest, ego_speed, num)
    elif behavior == 'Vehicle Cross':
        return vehicle_cross(network, road_type, ego_init, ego_dest, ego_speed, num)
    elif behavior == 'Turn Around':
        return turn_around(network, road_type, ego_init, ego_dest, ego_speed, num)

def generate_behavior_trajectory(network, road_type, behavior, ego_init, ego_dest, ego_speed, num):
    # try:
    #     trajectory = generate_trajectory(network, road_type, behavior, ego_init, ego_dest, ego_speed, num)
    # except:
    #     trajectory = generate_trajectory(network, road_type, behavior, ego_init, ego_dest, ego_speed, num)
    print("start generating trajectory")
    trajectory = generate_trajectory(network, road_type, behavior, ego_init, ego_dest, ego_speed, num)
    print("finish generating trajectory")
    return trajectory

def generate_vehicle_participants(sim, trajectories, behavior, ego = False):
    if behavior == 'Park':
        vehicle, rotation = spawn(sim, trajectories[0], trajectories[1], 0, reverse=True)
        vehicle_type = random.choice(["Sedan", "BoxTruck"])
        agent = sim.add_agent(vehicle_type, lgsvl.AgentType.NPC, vehicle)
    else:
        if ego:
            vehicle_type = "SUV"
            vehicle, rotation = spawn(sim, trajectories[0][0], trajectories[0][1], 0)
            waypoints = get_vehicle_waypoints_time(sim, trajectories)
        else:
            vehicle_type = random.choice(["Sedan", "BoxTruck"])
            vehicle, rotation = spawn(sim, trajectories[0][0][0], trajectories[0][0][1], 0)
            waypoints = get_vehicle_waypoints_time(sim, trajectories[0], trajectories[1])

        agent = sim.add_agent(vehicle_type, lgsvl.AgentType.NPC, vehicle)
        agent.follow(waypoints)

def generate_pedestrian_participants(sim, trajectories):
    walker, _ = spawn(sim, trajectories[0][0][0], trajectories[0][0][1], 0)
    agent = sim.add_agent("Bob", lgsvl.AgentType.PEDESTRIAN, walker)
    waypoints = get_walk_waypoints_time(sim, trajectories[0], trajectories[1])
    agent.follow(waypoints)