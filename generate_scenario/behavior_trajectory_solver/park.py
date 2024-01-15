from CRISCO.generate_scenario.TrafficFactors.AccidentProne import *
from CRISCO.generate_scenario.behavior_trajectory_solver.point_in_road import *

def park(network, road_type, ego_init, ego_dest, num):
    print("park")
    accident_prone = get_critical_district(road_type)

    # find road by road id
    # road_id = 0
    # road = filter(lambda r: r.id == road_id, network.roads).__next__()
    # road_length = ego_road.lanes[0].centerline.lineString.length

    # find road by point
    ego_point = Vector(ego_init[0], ego_init[1])
    ego_init_lane = network.laneAt(ego_point)
    # ego_road = network.findPointIn(ego_point, network.roads, False)
    ego_dest_lane = network.laneAt(Vector(ego_dest[0], ego_dest[1]))
    ego_init_project_length = ego_init_lane.centerline.lineString.project(Point(ego_init[0], ego_init[1]))
    ego_dest_project_length = ego_dest_lane.centerline.lineString.project(Point(ego_dest[0], ego_dest[1]))

    npc_dest = [random.uniform(accident_prone[0][0], accident_prone[0][1]),
                random.uniform(accident_prone[1][0], accident_prone[1][1])]
    npc_dest_lane = network.laneAt(Vector(npc_dest[0], npc_dest[1]))
    npc_dest_project_length = npc_dest_lane.centerline.lineString.project(Point(npc_dest[0], npc_dest[1]))
    while (ego_dest_project_length - npc_dest_project_length) < random.randint(0, 10):
        npc_dest = [random.uniform(accident_prone[0][0], accident_prone[0][1]),
                    random.uniform(accident_prone[1][0], accident_prone[1][1])]
        npc_dest_lane = network.laneAt(Vector(npc_dest[0], npc_dest[1]))
        npc_dest_project_length = npc_dest_lane.centerline.lineString.project(Point(npc_dest[0], npc_dest[1]))

    # npc_init = get_npc_init_point(network, ego_init, relative='front')
    # npc_init_project_length = ego_init_lane.centerline.lineString.project(Point(npc_init[0], npc_init[1]))
    # while (npc_init_project_length - ego_init_project_length) <= 10:
    #     npc_init = get_npc_init_point(network, ego_init, relative='front')
    #     npc_init_project_length = ego_init_lane.centerline.lineString.project(Point(npc_init[0], npc_init[1]))

    # solver
    parked_points = []
    for i in range(0, num):
        parked_point = [npc_dest[0] + random.random(), npc_dest[1] + random.random()]
        parked_points.append(parked_point)

    return parked_points