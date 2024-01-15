import json
import os

root_path = os.path.abspath(os.path.dirname(__file__)).split('CRISCO')[0]

# get environment parameters and their value ranges
path = os.path.join(root_path, "CRISCO/UserDefined/Environment")
f = open(path, encoding='utf-8')
environment_json = json.load(f)
f.close()

# get traffic signal positions
path = os.path.join(root_path, "CRISCO/UserDefined/RoadDistrict")
f = open(path, encoding='utf-8')
traffic_signal_json = json.load(f)
f.close()

def get_environment():
    parameters = []
    value_ranges = []
    for key, value in environment_json.items():
        parameters.append(key)
        value_ranges.append(value)
    return parameters, value_ranges

def get_traffic_signal():
    traffic_signal_array = traffic_signal_json["signal_position"]

    return traffic_signal_array
