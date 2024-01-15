import json
import sys
from scenic.core.vectors import Vector
from scenic.domains.driving.roads import Network
import os

# get map file
root_path = os.path.abspath(os.path.dirname(__file__)).split('CRISCO')[0]
selected_map = os.path.join(root_path, "CRISCO/UserDefined/map")
f = open(selected_map, encoding='utf-8')
map_json = json.load(f)
f.close()
map_name = map_json["map"]

def get_network():
    try:
        path = os.path.join(root_path, "CRISCO/UserDefined/"+map_name)
        network = Network.fromFile(path)
        return network
    except FileNotFoundError:
        print("The selected map was not in map folder of lgsvl dictionary")
        sys.exit(1)
