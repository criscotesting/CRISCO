import json
import random, os

# get behavior patterns
root_path = os.path.abspath(os.path.dirname(__file__)).split('CRISCO')[0]
path = os.path.join(root_path, "CRISCO/UserDefined/BehaviorPatterns")
f = open(path, encoding='utf-8')
behavior_json = json.load(f)
f.close()

patterns = behavior_json['patterns']

behaviors = behavior_json['all_behaviors']

lane_behaviors = behavior_json['multiple_lane_behaviors']

intersection_behaviors = behavior_json['intersection_behaviors']

common_behaviors = behavior_json['common_behaviors']

def get_intersection_patterns():
    intersection_patterns = []
    for pattern in patterns:
        saved = 1
        for behavior in pattern:
            if behavior in lane_behaviors:
                saved = 0
        if saved == 1:
            intersection_patterns.append(pattern)
    return intersection_patterns

def get_lane_patterns():
    lane_patterns = []
    for pattern in patterns:
        saved = 1
        for behavior in pattern:
            if behavior in intersection_behaviors:
                saved = 0
        if saved == 1:
            lane_patterns.append(pattern)
    return lane_patterns

def get_all_behaviors():
    return behaviors