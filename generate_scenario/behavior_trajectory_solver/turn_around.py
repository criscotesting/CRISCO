from z3 import *
from random import randint

from CRISCO.generate_scenario.TrafficFactors.AccidentProne import *
from CRISCO.generate_scenario.behavior_trajectory_solver.connect_lanes import combine_center_line
from CRISCO.generate_scenario.behavior_trajectory_solver.point_in_road import *

def turn_around(network, road_type, ego_init, ego_dest, ego_speed, num):
    print("turn_around")
    accident_prone = get_critical_district(road_type)

    # get lanes of initial position and destination of ego
    ego_point = Vector(ego_init[0], ego_init[1])
    ego_init_lane = network.laneAt(ego_point)
    ego_dest_lane = network.laneAt(Vector(ego_dest[0], ego_dest[1]))
    ego_line_string = combine_center_line(network, ego_init_lane, ego_dest_lane)
    ego_init_project_length = ego_line_string.project(Point(ego_init[0], ego_init[1]))
    ego_dest_project_length = ego_line_string.project(Point(ego_dest[0], ego_dest[1]))

    rejection = True
    waypoint_sequence = []
    while rejection:
        # select destination of npc in accident-prone area
        npc_dest = [random.uniform(accident_prone[0][0], accident_prone[0][1]),
                    random.uniform(accident_prone[1][0], accident_prone[1][1])]
        npc_dest_road = network.findPointIn(Vector(npc_dest[0], npc_dest[1]), network.roads, False)
        npc_dest_lane = network.laneAt(Vector(npc_dest[0], npc_dest[1]))
        # select initial position of npc outside accident-prone area
        npc_init, npc_init_lane = turn_init_position(network, road_type, npc_dest_road, ego_init_project_length, ego_init_lane)
        lane_string = combine_center_line(network, npc_init_lane, npc_dest_lane)
        npc_init_project_length = lane_string.project(Point(npc_init[0], npc_init[1]))
        npc_dest_project_length = lane_string.project(Point(npc_dest[0], npc_dest[1]))

        # z3 solver
        T = Real("T")
        C = Real("C")

        ego_initial_position = Real("ego_initial_position")
        ego_target_position = Real("ego_target_position")
        # solve constraints of waypoints
        npc_initial_speed = Real("npc_initial_speed")
        npc_target_speed = Real("npc_target_speed")
        npc_initial_position = Real("npc_initial_position")
        npc_target_position = Real("npc_target_position")
        npc_accelerate = Real("npc_accelerate")
        npc_average_speed = Real("npc_average_speed")

        time_c = []
        position_c = []
        speed_c = []
        time = random.randint(0, 2)
        # driving time and trigger time
        time_c += [T * ego_speed == ego_target_position - ego_initial_position] + \
                [T - C >= time] + \
                [And(T >= 1, C >= 1)]
        # speeds
        speed_c += [And(0 <= npc_initial_speed, npc_initial_speed <= 25)] + \
                [And(0 <= npc_target_speed, npc_target_speed <= 25)] + \
                [npc_average_speed == (npc_target_speed + npc_initial_speed) / 2.0] + \
                [npc_accelerate == (npc_target_speed - npc_initial_speed) / C] + \
                [And(-10 <= npc_accelerate, npc_accelerate <= 5.5)]
        # positions
        position_c += [ego_initial_position == ego_init_project_length] + \
                    [ego_target_position == ego_dest_project_length] + \
                    [npc_target_position == npc_dest_project_length] + \
                    [npc_initial_position == npc_init_project_length] + \
                    [C * npc_average_speed == npc_target_position - npc_initial_position]

        s = Solver()
        s.add(speed_c + position_c + time_c)
        set_option(rational_to_decimal=True)
        set_option(precision=1)

        res = []
        cnt = 0
        m = None
        while cnt < 100:
            cnt += 1
            if s.check() == sat:
                m = s.model()
                res.append([toNum(m.evaluate(npc_initial_speed)), toNum(m.evaluate(npc_target_speed)),
                            toNum(m.evaluate(npc_initial_position))])
                fml = And(npc_initial_speed == res[-1][0], npc_target_speed == res[-1][1], npc_initial_position == res[-1][2])
                s.add(Not(fml))
        
        if len(res) < num:
            continue
        rejection = False
        
        ans_list = random.sample(res, num)
        for ans in ans_list:
            center_line = combine_center_line(network, npc_init_lane, npc_dest_lane)
            waypoints = []
            t = toNum(m.evaluate(T))
            c = toNum(m.evaluate(C))
            acc = (ans[1] - ans[0]) / c
            for i in range(int(c * 10)):
                speed = ans[0] + acc * (i + 1) * 0.1
                pos = ans[2] + (ans[0] + speed) / 2.0 * (i + 1) * 0.1
                p = center_line.interpolate(pos)
                waypoints.append([p.x, p.y, speed])
            waypoint_sequence.append([waypoints, t-c])

    return waypoint_sequence

def turn_init_position(network, road_type, npc_dest_road, ego_init_project_length, ego_init_lane):
    district = get_district(road_type)
    index = -2
    for i in range(len(district)):
        road = network.findPointIn(Vector(district[i][0], district[i][1]), network.roads, False)
        if road == npc_dest_road:
            index = i
    initial_district = 0

    if index == 0:
        initial_district = random.choice([1, 3])
    elif index == 1:
        initial_district = random.choice([0, 2])
    elif index == 2:
        initial_district = random.choice([1, 3])
    elif index == 3:
        initial_district = random.choice([0, 2])

    position = district[initial_district]
    npc_init = get_npc_init_point(network, position)
    npc_point = Vector(npc_init[0], npc_init[1])
    npc_init_lane = network.laneAt(npc_point)
    npc_init_project_length = npc_init_lane.centerline.lineString.project(Point(npc_point.x, npc_point.y))
    while npc_init_lane == ego_init_lane and abs(ego_init_project_length - npc_init_project_length) < 5:
        npc_init = get_npc_init_point(network, position)
        npc_point = Vector(npc_init[0], npc_init[1])
        npc_init_lane = network.laneAt(npc_point)
        npc_init_project_length = npc_init_lane.centerline.lineString.project(Point(npc_point.x, npc_point.y))

    return npc_init, npc_init_lane
