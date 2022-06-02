import sys
sys.path.append("..")
from CRISCO.generate_scenario.connect_simulator import *
from CRISCO.generate_scenario.environment_definition.define_environment_svl  import *
from CRISCO.generate_scenario.TrafficFactors.Environment  import *
from CRISCO.generate_scenario.TrafficFactors.BehaviorPattern  import *
from CRISCO.generate_scenario.get_waypoints import *
from CRISCO.generate_scenario.generate_behavior_trajectory import *

import json

def reproduce_scenarios(road_type, file_name):
    # connect to simulator
    sim = connect_svl()

    # Loads the named map in the connected simulator, and define traffic signal state on selected road
    if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco:
        sim.reset()
    else:
        sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)

    root_path = os.path.abspath(os.path.dirname(__file__)).split('CRISCO')[0]
    scenario_record = os.path.join(root_path, "CRISCO/safety_violation_scenarios/" + road_type + "/" + file_name)
    f = open(scenario_record, encoding='utf-8')
    scenario_json = json.load(f)
    f.close()
    define_road_traffic_signal_svl(sim, get_traffic_signal(), scenario_json["road_type"])

    # define environment parameter
    weather = scenario_json["environment"]
    sim.weather = lgsvl.WeatherState(rain=weather[0], fog=weather[1], wetness=weather[2], cloudiness=weather[3])

    # define participants and their trajectories
    ego_spawn = None
    for key, value in scenario_json.items():
        if key == "Ego":
            ego_spawn = lgsvl.Vector(value[0][0], 0, value[0][1])
            generate_vehicle_participants(sim, value, key, True)
        elif key in get_all_behaviors():
            if 'Pedestrian' in key:
                generate_pedestrian_participants(sim, value)
            else:
                generate_vehicle_participants(sim, value, key)

    print("start reproducing scenario")

    # execute the scenario
    tr = lgsvl.Transform(lgsvl.Vector(0, 50, 0) + ego_spawn, lgsvl.Vector(90, 0, 0))
    sim.set_sim_camera(tr)
    for i in range(100):
        sim.run(0.5)

    print("finish reproducing scenario")

if __name__ == '__main__':
    road_type = "freeway"
    file_name = "safety_violation0"
    reproduce_scenarios(road_type, file_name)



