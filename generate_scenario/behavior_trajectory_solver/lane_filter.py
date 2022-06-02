from scenic.core.vectors import Vector

from CRISCO.generate_scenario.TrafficFactors.AccidentProne import get_ego_next_lane
from CRISCO.generate_scenario.get_map import get_network

def filter_lane(network, position):
    next_lane = get_ego_next_lane()
    lane = network.intersectionAt(Vector(next_lane[0], next_lane[1]))
    position_lane = network.laneAt(Vector(position[0], position[1]))
    return position_lane in lane.incomingLanes

def find_lane_by_id(network, uid):
    for lane in network.lanes:
        if lane.uid == uid:
            return lane

def linestring2array(linestring):
    linestring = str(linestring)
    line_array = linestring.split('LINESTRING ')[1]
    line_array = line_array.replace("(", "").replace(")", "")
    line_array = line_array.split(", ")
    lines = []
    for item in line_array:
        item_list = item.split(' ')
        coord_item = []
        for coord in item_list:
            coord = float(coord)
            coord = round(coord, 2)
            coord_item.append(coord)
        lines.append(coord_item)
    return lines

if __name__ == '__main__':
    network = get_network()
    init_lane = network.laneAt(Vector(36.4, -43.8))
    dest_lane = network.laneAt(Vector(-64.1, -118.9))
    # print(str(combine_center_line(init_lane, dest_lane)))