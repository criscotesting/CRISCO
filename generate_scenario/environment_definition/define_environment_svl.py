import json

from CRISCO.generate_scenario.environment_definition.environment_sampling import importance_sampling
import lgsvl
import os

# get weather items mapping to weather attributes in svl
root_path = os.path.abspath(os.path.dirname(__file__)).split('CRISCO')[0]
path = os.path.join(root_path, "CRISCO/generate_scenario/environment_definition/environment_svl")
f = open(path, encoding='utf-8')
environment_svl_json = json.load(f)
f.close()

def define_road_traffic_signal_svl(sim, signal_position, road_type):
    if road_type == 'urban' or road_type == 'intersection':
        for signal_light in signal_position:
            signal = sim.get_controllable(lgsvl.Vector(signal_light), "signal")
            control_policy = "trigger=100;green=49;yellow=2;red=49;loop"
            signal.control(control_policy)
    elif road_type == 'freeway':
        for signal_light in signal_position:
            signal = sim.get_controllable(lgsvl.Vector(signal_light), "signal")
            control_policy = "trigger=100;green=100;yellow=0;red=0;loop"
            signal.control(control_policy)


def define_weather_svl(parameter_list, parameters):
    # weather_parameter in svl
    rain, fog, wetness, cloudiness = 0, 0, 0, 0
    for i in range(len(parameter_list)):
        if parameter_list[i] in environment_svl_json['view_factors'] and parameters[i] == 1:
            visibility = importance_sampling(0.4, 1)
            visibility = round(visibility[0], 2)
            cloudiness = 1 - visibility

        if parameter_list[i] in environment_svl_json['ground_factors'] and parameters[i] == 1:
            wetness = importance_sampling(0.6, 1)
            wetness = round(wetness[0], 2)
        if parameter_list[i] == 'heavy rain' and parameters[i] == 1:
            rain = importance_sampling(0.7, 1)
            rain = round(rain[0], 2)
        if parameter_list[i] == 'fog' and parameters[i] == 1:
            fog = importance_sampling(0.5, 1)
            fog = round(fog[0], 2)

    return lgsvl.WeatherState(rain=rain, fog=fog, wetness=wetness, cloudiness=cloudiness)
