import json
import os
import random
import time

import lgsvl

from BehaviorPattern import get_intersection_patterns, get_lane_patterns
from define_environment_svl import define_weather_svl
from environment_sampling import pairwise_environment_parameters
from get_map import get_network
from generate_scenario.connect_simulator import connect_svl
from generate_scenario.define_ego import define_ego_state
from generate_scenario.determine_participants import cal_TTC, get_threshold
from generate_scenario.generate_behavior_trajectory import generate_behavior_trajectory, \
    generate_pedestrian_participants, generate_vehicle_participants
from generate_scenario.get_waypoints import spawn_direct, spawn, visitDir
from func_timeout import func_set_timeout
import func_timeout

network = get_network()

participants, participant_trajectories = [], []

def test_generation(road_type):
    global participants
    global participant_trajectories
    global road
    global isCollision
    isCollision = False
    road = road_type
    sim = connect_svl()
    if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco:
        sim.reset()
    else:
        sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)

    ego_destination, ego_spawn, ego_speed = define_ego_state(network, road_type)
    ego_state, _ = spawn_direct(sim, ego_spawn[0], ego_spawn[1], ego_speed)
    ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_modular, lgsvl.AgentType.EGO, ego_state)
    ego.on_collision(on_collision)

    this_patterns, first_pattern, now_pattern = [], [], []
    if road_type == 'freeway' or road_type == 'urban':
        this_patterns = get_lane_patterns()
    elif road_type == 'intersection':
        this_patterns = get_intersection_patterns()
    this_pattern_size = len(this_patterns)
    first_pattern = this_patterns[random.randint(0, this_pattern_size - 1)]
    trajectories = []
    # determine whether the scenario meet stop criteria
    ettc = 100
    new_ettc = 100
    while new_ettc >= ettc:
        first_pattern = this_patterns[random.randint(0, this_pattern_size - 1)]
        for behavior in first_pattern:
            trajectory = generate_behavior_trajectory(network, road_type, behavior, ego_spawn, ego_destination,
                                                      ego_speed)
            new_ettc = cal_TTC(network, ego_spawn, ego_destination, ego_speed, trajectory, behavior)
            if 'Pedestrian' in behavior:
                generate_pedestrian_participants(sim, trajectory)
            else:
                generate_vehicle_participants(sim, trajectory, behavior)
            participants.append(behavior)
            participant_trajectories.append(trajectory)
            if new_ettc < ettc:
                ettc = new_ettc
        now_pattern = first_pattern
        break
    selected, search = 0, 0
    while ettc > get_threshold() and this_patterns is not None and search == selected:
        search = search + 1
        this_patterns.remove(now_pattern)
        temp_pattern = now_pattern
        for pattern in this_patterns:
            for behavior in pattern:
                if behavior in now_pattern:
                    selected = selected + 1
                    for pattern_behavior in pattern:
                        trajectory = generate_behavior_trajectory(network, road_type, pattern_behavior, ego_spawn,
                                                                  ego_destination, ego_speed)
                        new_ettc = cal_TTC(network, ego_spawn, ego_destination, ego_speed, trajectory, behavior)
                        if 'Pedestrian' in pattern_behavior:
                            generate_pedestrian_participants(sim, trajectory)
                        else:
                            generate_vehicle_participants(sim, trajectory, pattern_behavior)
                        participants.append(behavior)
                        participant_trajectories.append(trajectory)
                        if new_ettc < ettc:
                            ettc = new_ettc
                    now_pattern = pattern
                    if now_pattern != temp_pattern:
                        break
            if now_pattern != temp_pattern:
                break

    # execute the scenario
    for i in range(100):
        if not isCollision:
            sim.run(0.1)

def on_collision(agent1, agent2, contact):
    global isCollision
    isCollision = True
    root_path = os.path.abspath(os.path.dirname(__file__)).split('CRISCO')[0]
    scenario_example = os.path.join(root_path, "CRISCO\\reproduce_safety_violation_scenarios\\scenario_example")
    f = open(scenario_example, encoding='utf-8')
    scenario_json = json.load(f)
    f.close()
    scenario_folder = os.path.join(root_path, "CRISCO\\safety_violation_scenarios\\" + road)
    file_nums = visitDir(scenario_folder)
    scenario_record = os.path.join(root_path,
                                   "CRISCO\\safety_violation_scenarios\\" + road + "\\safety_violation" + str(file_nums))
    for i in range(len(participants)):
        if participants[i] in scenario_json:
            this_key = participants[i] + str(1)
            scenario_json[this_key] = participant_trajectories[i]
        elif participants[i] not in scenario_json:
            scenario_json[participants[i]] = participant_trajectories[i]
    del scenario_json["agent"]
    file = open(scenario_record, 'w')
    json.dump(scenario_json, file, ensure_ascii=False)

def test_behavior(road_type, behavior):
    global participants
    global participant_trajectories
    global road
    global isCollision
    isCollision = False
    road = road_type
    sim = connect_svl()
    if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco:
        sim.reset()
    else:
        sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)

    participants.append("road_type")
    participant_trajectories.append(road_type)

    parameter_list, parameters = pairwise_environment_parameters()
    weatherState = define_weather_svl(parameter_list, parameters)
    sim.weather = weatherState

    participants.append("environment")
    participant_trajectories.append([weatherState.rain, weatherState.fog, weatherState.wetness, weatherState.cloudiness])

    ego_destination, ego_spawn, ego_speed = define_ego_state(network, road_type)
    ego_state, _ = spawn(sim, ego_spawn[0], ego_spawn[1], ego_speed)
    ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_modular, lgsvl.AgentType.EGO, ego_state)
    participants.append("Ego")
    participant_trajectories.append([ego_spawn, ego_destination, ego_speed])

    trajectory = generate_behavior_trajectory(network, road_type, behavior, ego_spawn, ego_destination, ego_speed)
    if 'Pedestrian' in behavior:
        generate_pedestrian_participants(sim, trajectory)
    else:
        generate_vehicle_participants(sim, trajectory, behavior)

    participants.append(behavior)
    participant_trajectories.append(trajectory)

    ego.on_collision(on_collision)
    # execute the scenario
    for i in range(500):
        if not isCollision:
            sim.run(0.1)

if __name__ == '__main__':
    # test_behavior('intersection', 'Turn Around')
    test_generation('freeway')
    # position = lgsvl.Vector(-72.2, 0, 130.9)
    # transform = lgsvl.Transform(position, rotation=lgsvl.Vector(0, 0, 0))
