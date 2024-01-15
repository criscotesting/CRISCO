import random

from scenic.core.vectors import Vector
from shapely.geometry import Point

from CRISCO.generate_scenario.TrafficFactors.AccidentProne import get_ego_next_lane
from CRISCO.generate_scenario.get_map import get_network
from CRISCO.generate_scenario.behavior_trajectory_solver.lane_filter import filter_lane

# class RoadPoint:
#
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y
#
#     def __repr__(self):
#         return "({},{})".format(self.x, self.y)


class Line:

    def __init__(self, p1, p2):

        self.p1 = p1
        self.p2 = p2

        self.k = (p2.y - p1.y) / (p2.x - p1.x) if p1.x != p2.x else None
        self.b = -p1.x * (p2.y - p1.y) / (p2.x - p1.x) + p1.y if p1.x != p2.x else None

    def intersection(self, line2):

        # 两直线平行
        if self.is_vertical() and line2.is_vertical():
            return None

        if self.k == line2.k:
            return None

        # 相交

        # 有其中一条垂直
        if self.is_vertical():
            x = self.p1.x
            y = round(line2.k * x + line2.b, 2)
        elif line2.is_vertical():
            x = line2.p1.x
            y = round(self.k * x + self.b, 2)
        else:
            # 一般情况
            x = round((self.b - line2.b) / (line2.k - self.k), 2)
            y = round(self.k * x + self.b, 2)

        # 点在线段上：x在线段的范围内
        r1 = (self.p2.x, self.p1.x) if self.p1.x >= self.p2.x else (self.p1.x, self.p2.x)
        r2 = (line2.p2.x, line2.p1.x) if line2.p1.x >= line2.p2.x else (line2.p1.x, line2.p2.x)
        if not (r1[0] <= x <= r1[1] and r2[0] <= x <= r2[1]):
            return None

        return Point(x, y)

    def is_horizontal(self):

        return self.p1.y == self.p2.y

    def is_vertical(self):

        return self.p1.x == self.p2.x

    def __repr__(self):
        return "[{}->{}, k={}, b={}]".format(self.p1, self.p2, self.k, self.b)


class Polygen:

    def __init__(self, points):
        self.lines = []
        for i in range(-1, len(points) - 1):
            self.lines.append(Line(points[i], points[i + 1]))

    def contains(self, point):
        point_b = Point(point.x + 100000000, point.y)  # 这个值尽量大，因为暂时不支持射线

        line2 = Line(point, point_b)
        # line3 = Line(Point(2, -1), Point(2, 3))

        return len(
            list(filter(lambda p: p is not None, list(map(lambda l: l.intersection(line2), self.lines))))) % 2 == 1

def point_ploy(points):
    poly_items = []
    for item in points:
        poly_items.append(Point(item[0], item[1]))
    poly_items = tuple(poly_items)
    poly = Polygen(poly_items)

    return poly

    # return poly.contains(Point(point[0], point[1]))

def toNum(val):
    val = str(val)
    if val[-1] == '?':
        val = val[:-1]
    return float(val)

def sample_road_points(network, position):
    boundaries = get_road_boundary(network, position)
    poly = point_ploy(boundaries)
    x_min, x_max, y_min, y_max = 10000, -10000, 10000, -10000
    for boundary in boundaries:
        if boundary[0] > x_max:
            x_max = boundary[0]
        if boundary[0] < x_min:
            x_min = boundary[0]
        if boundary[1] > y_max:
            y_max = boundary[1]
        if boundary[1] < y_min:
            y_min = boundary[1]
    npc_init = [random.uniform(x_min, x_max),
                random.uniform(y_min, y_max)]
    while poly.contains(Point(npc_init[0], npc_init[1])) is False:
        npc_init = [random.uniform(x_min, x_max),
                random.uniform(y_min, y_max)]
    return npc_init

def sample_ego_init(network, road_type):
    ego_init = sample_road_points(road_type)
    while filter_lane(network, ego_init):
        sample_road_points(road_type)

    return ego_init

def get_road_boundary(network, position):
    road = network.findPointIn(Vector(position[0], position[1]), network.roads, False)
    line_array = str(road.boundary.lineString).split('MULTILINESTRING ')[1]
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

def get_point_in_segment(segment):
    aw = random.random()
    bw = 1 - aw
    a = segment[0]
    b = segment[1]
    x = aw * a[0] + bw * b[0]
    y = aw * a[1] + bw * b[1]
    return [x, y]

def get_npc_init_point(network, position, same_lane=None, relative=None, reverse=False):
    road = network.findPointIn(Vector(position[0], position[1]), network.roads, False)
    lane_position = network.laneAt(Vector(position[0], position[1]))
    lanes = road.lanes
    search_lanes = []
    next_lane = get_ego_next_lane()
    next_lane_intersection = network.intersectionAt(Vector(next_lane[0], next_lane[1]))
    incoming_lanes = next_lane_intersection.incomingLanes
    if reverse is None:
        search_lanes = lanes
    elif reverse is True:
        for lane in lanes:
            if lane not in incoming_lanes:
                search_lanes.append(lane)
    elif reverse is False:
        for lane in lanes:
            if lane in incoming_lanes:
                search_lanes.append(lane)

    search_lanes2 = []
    if same_lane is None:
        search_lanes2 = search_lanes
    elif same_lane:
        for lane in search_lanes:
            if lane == lane_position:
                search_lanes2.append(lane)
    elif same_lane is False:
        for lane in search_lanes:
            if lane != lane_position:
                search_lanes2.append(lane)

    # lane_index = random.randint(0, len(search_lanes2))
    # lane = search_lanes2[lane_index-1]
    lane = random.choice(search_lanes2)
    # segment_index = random.randint(2, len(lane.centerline.segments))
    # segment = lane.centerline.segments[segment_index-1]
    # point = get_point_in_segment(segment)
    length = lane.centerline.lineString.length
    position_project_length = lane_position.centerline.lineString.project(Point(position[0], position[1]))
    point_project_length = random.uniform(0, length)
    point = lane.centerline.lineString.interpolate(point_project_length)

    if relative == 'front':
        # point_project_length = lane.centerline.lineString.project(Point(point[0], point[1]))
        if length - position_project_length <= 5:
            point_project_length = length
            point = lane.centerline.lineString.interpolate(point_project_length)
        else:
            point_project_length = random.uniform(position_project_length, length)
            point = lane.centerline.lineString.interpolate(point_project_length)
            while (point_project_length - position_project_length) < min(5, position_project_length * 0.6):
                # segment_index = random.randint(0, len(lane.centerline.segments)-1)
                # segment = lane.centerline.segments[segment_index]
                # point = get_point_in_segment(segment)
                # position_project_length = lane_position.centerline.lineString.project(Point(position[0], position[1]))
                # point_project_length = lane.centerline.lineString.project(Point(point[0], point[1]))
                point_project_length = random.uniform(position_project_length, length)
                point = lane.centerline.lineString.interpolate(point_project_length)
    elif relative == 'behind':
        point_project_length = random.uniform(0, position_project_length)
        point = lane.centerline.lineString.interpolate(point_project_length)
        while (position_project_length - point_project_length) < min(10, position_project_length * 0.6):
            # segment_index = random.randint(0, len(lane.centerline.segments)-1)
            # segment = lane.centerline.segments[segment_index]
            # point = get_point_in_segment(segment)
            # position_project_length = lane_position.centerline.lineString.project(Point(position[0], position[1]))
            # point_project_length = lane.centerline.lineString.project(Point(point[0], point[1]))
            point_project_length = random.uniform(0, position_project_length)
            point = lane.centerline.lineString.interpolate(point_project_length)

    return Vector(point.x, point.y)

if __name__ == '__main__':
    network = get_network()
    print(network.laneAt(Vector(-64.1, -118.9)).centerline.segments)
    # print(get_npc_init_point(network, [-64.1, -118.9], relative='front'))