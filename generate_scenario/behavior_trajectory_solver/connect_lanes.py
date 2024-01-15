from scenic.core.regions import PolylineRegion
from scenic.core.vectors import Vector

from CRISCO.generate_scenario.TrafficFactors.AccidentProne import get_ego_next_lane
from CRISCO.generate_scenario.get_map import get_network


def combine_center_line(network, init_lane, dest_lane):
    forward = get_ego_next_lane()
    forward_lane = network.intersectionAt(Vector(forward[0], forward[1]))
    lanes = [init_lane.centerline, dest_lane.centerline]
    init_road = init_lane.road
    dest_road = dest_lane.road
    if init_road == dest_road:
        return dest_lane.centerline.lineString
    else:
        for lane in init_road.lanes:
            if lane in forward_lane.incomingLanes:
                for candidate_maneuver in lane.maneuvers:
                    if candidate_maneuver.endLane == dest_lane and candidate_maneuver.connectingLane is not None:
                        lanes.append(candidate_maneuver.connectingLane.centerline)
                        break
                if len(lanes) > 2:
                    break

    return PolylineRegion.unionAll(lanes).lineString

if __name__ == '__main__':
    network = get_network()
    init, dest = [-64.1, -118.9], [39.3, -44.1]
    init_point = Vector(init[0], init[1])
    dest_point = Vector(dest[0], dest[1])
    init_lane = network.laneAt(init_point)
    dest_lane = network.laneAt(dest_point)
    ego_line_string = combine_center_line(network, init_lane, dest_lane)
    print(str(ego_line_string))
