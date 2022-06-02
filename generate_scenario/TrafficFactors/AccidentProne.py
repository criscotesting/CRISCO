import json
import os

# get road districts and accident-prone districts
root_path = os.path.abspath(os.path.dirname(__file__)).split('CRISCO')[0]
path = os.path.join(root_path, "CRISCO/UserDefined/RoadDistrict")
f = open(path, encoding='utf-8')
road_json = json.load(f)
f.close()


def get_district(id):
    return road_json[id]

def get_critical_district(id):
    id = id + "_accident-prone"
    return road_json[id]

def get_ego_next_lane():
    return road_json["ego_next_lane"]