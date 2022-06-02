import sys
sys.path.append("..")
from CRISCO.generate_scenario.TrafficFactors.BehaviorPattern import *
from CRISCO.generate_scenario.define_ego import define_ego_state
from CRISCO.generate_scenario.get_map import get_network
from CRISCO.generate_scenario.connect_simulator import *
from CRISCO.generate_scenario.environment_definition.environment_sampling import pairwise_environment_parameters
from CRISCO.generate_scenario.environment_definition.define_environment_svl import *
from CRISCO.generate_scenario.TrafficFactors.Environment import get_traffic_signal
from CRISCO.generate_scenario.generate_behavior_trajectory import *
from CRISCO.generate_scenario.determine_participants import *
import copy

network = get_network()
participants, participant_trajectories = [], []
ego_trajectory = []

def generate_apollo_case(road_type, num):
    global participants
    global participant_trajectories
    global road
    global isCollision
    global ego_trajectory
    isCollision = False
    road = road_type

    # define ego route and compute running time
    ego_destination, ego_spawn, ego_speed = define_ego_state(network, road_type)

    # define participants and their trajectories
    this_patterns, first_pattern, now_pattern = [], [], []
    if road_type == 'freeway' or road_type == 'urban':
        this_patterns = get_lane_patterns()
    elif road_type == 'intersection':
        this_patterns = get_intersection_patterns()
    this_pattern_size = len(this_patterns)

    # generate scenarios
    ettc = 200
    scenarios = []
    behaviors, trajectories = [], []
    # generate initial abstract scenarios
    now_pattern = this_patterns[random.randint(0, this_pattern_size - 1)]
    for behavior in now_pattern:
        behaviors.append(behavior)
        # generate trajectories of selected behaviors
        trajectory = generate_behavior_trajectory(network, road_type, behavior, ego_spawn, ego_destination,
                                                  ego_speed, num)
        trajectories.append(trajectory)
        
    # generate concrete scenarios
    for i in range(num):
        scenario = []
        for j in range(len(trajectories)):
            scenario.append((behaviors[j], trajectories[j][i]))
        scenarios.append(scenario)

    # evaluate and alter scenarios
    def expand_scenario(scenario):
        ettc = 200
        new_scenarios = []
        for behavior,traj in scenario:
            ettc = min(ettc, cal_TTC(network, ego_spawn, ego_destination, ego_speed, traj, behavior))
        new_pattern_cnt = 0
        while ettc >= get_threshold():
            new_pattern = random.choice(this_patterns)
            new_pattern_cnt += 1
            for behavior in new_pattern:
                trajectory = generate_behavior_trajectory(network, road_type, behavior, ego_spawn,
                                                            ego_destination, ego_speed, num)
                for traj in trajectory:
                    new_ettc = cal_TTC(network, ego_spawn, ego_destination, ego_speed, traj, behavior)
                    if new_ettc < get_threshold():
                        new_scenario = copy.deepcopy(scenario)
                        new_scenario.append((behavior, traj))
                        new_scenarios.append(new_scenario)
                        ettc = min(ettc, new_ettc)
        if new_scenarios:
            return new_scenarios
        else:
            return [scenario]

    new_scenarios = []
    for scenario in scenarios:
        new_scenarios.extend(expand_scenario(scenario)) 

    # execute scenario
    for scenario in new_scenarios:
        print("Current Scene = {}".format(sim.current_scene))

        # Loads the named map in the connected simulator, and define traffic signal state on selected road
        if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco_correct:
            sim.reset()
        else:
            sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco_correct)
        sim.set_time_of_day(24)
        
        ego_state, _ = spawn_direct(sim, ego_spawn[0], ego_spawn[1], ego_speed)
        # bridge ego into Apollo
        ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_full_analysis, lgsvl.AgentType.EGO, ego_state)
        bridgeApollo(sim, ego, ego_destination)
        
        # monitor safety violation of ego vehicle
        ego.on_collision(on_collision)
        
        define_ego(sim, road_type)

        for behavior, trajectory in scenario:
            if 'Pedestrian' in behavior:
                generate_pedestrian_participants(sim, trajectory)
            else:
                generate_vehicle_participants(sim, trajectory, behavior)
            participants.append(behavior)
            participant_trajectories.append(trajectory)

        isCollision = False
        ego_trajectory = []
        for i in range(500):
            tr = ego.state.transform
            ego_trajectory.append([tr.position.x, tr.position.z, ego.state.speed])
            sim.run(0.1)

            if isCollision:
               break
        
        if isCollision:
            record_scenario()
        
def on_collision(agent1, agent2, contact):
    global isCollision
    isCollision = True

def record_scenario():
    global ego_trajectory
    root_path = os.path.abspath(os.path.dirname(__file__)).split('CRISCO')[0]
    scenario_example = os.path.join(root_path, "CRISCO/reproduce_safety_violation_scenarios/scenario_example")
    f = open(scenario_example, encoding='utf-8')
    scenario_json = json.load(f)
    f.close()
    scenario_folder = os.path.join(root_path, "CRISCO/safety_violation_scenarios/" + road)
    file_nums = visitDir(scenario_folder)
    scenario_record = os.path.join(root_path,
                                   "CRISCO/safety_violation_scenarios/" + road + "/safety_violation" + str(file_nums))
    
    scenario_json["Ego"] = ego_trajectory

    for i in range(len(participants)):
        if participants[i] in scenario_json:
            this_key = participants[i] + str(i)
            scenario_json[this_key] = participant_trajectories[i]
        elif participants[i] not in scenario_json:
            scenario_json[participants[i]] = participant_trajectories[i]
    del scenario_json["agent"]
    file = open(scenario_record, 'w')
    json.dump(scenario_json, file, ensure_ascii=False)

def define_ego(sim, road_type):
    participants.append("road_type")
    participant_trajectories.append(road_type)

    define_road_traffic_signal_svl(sim, get_traffic_signal(), road_type)
    # define environment parameter
    parameters, parameter_list = pairwise_environment_parameters()
    weatherState = define_weather_svl(parameter_list, parameters)
    sim.weather = weatherState

    participants.append("environment")
    participant_trajectories.append(
        [weatherState.rain, weatherState.fog, weatherState.wetness, weatherState.cloudiness])

if __name__ == '__main__':

    # choose road type: freeway, urban, intersection
    road_type = 'urban'

    # set number of trajectories for each behavior
    num = 4
    
    # connect to simulator
    sim = connect_svl()

    for i in range(20):
        generate_apollo_case('intersection', num)
        # generate_apollo_case('freeway', num)
        # generate_apollo_case('urban', num)

