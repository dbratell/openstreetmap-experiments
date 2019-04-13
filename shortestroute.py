#import osmapi
import osmread
import sys
import time
import geopy
from geopy.distance import vincenty
from geopy.distance import great_circle
import random
import math

INFINITY = 99999999 # Well... It's longer than any distance at earth.
OSM_FILE = "ostergotland.osm"

nodes = {}
ways = {}

#roads = [] # Only roads that I care about
#rails = [] # Only railroads that I care about
#buildings = []

WALKING, CAR, BIKE = (True, False, False)
WALKING, CAR, BIKE = (False, True, False)

color_and_width_per_highway_type = {
    # Normal roads for cars
    "motorway": ("red", 4.0),
    "trunk": ("orange", 3.0),
    "primary": ("black", 2.0),
    "secondary": ("black", 1.5),
    "tertiary": ("grey", 1.2),
    "unclassified": ("grey", 1.0),
    "residential": ("grey", 1.0),
    "service": ("grey", 1.0),
    "living_street": ("yellow", 1.0),
    "raceway": ("grey", 2.0),

    # Link roads for cars
    "motorway_link": ("red", 3.5),
    "trunk_link": ("orange", 2.5),
    "primary_link": ("black", 1.75),
    "secondary_link": ("black", 1.35),
    "tertiary_link": ("grey", 1.1),

    # For terrain cars - maybe.
    "track": ("brown", 0.7),

    # Future
    "construction": (None, 1.0),
    "proposed": (None, 1.0),

    # Bikes (and maybe people)
    "cycleway": ("pink", 0.5),

    # Walking
    "footway": (None, 0.3),
    "path": (None, 0.3),
    "steps": (None, 0.3),
    "pedestrian": (None, 0.3),
    "platform": (None, 0.3),
    "traffic_island": (None, 0.2),

    # Indoors
    "corridor": (None, 0.2),
    "elevator": (None, 0.2),

    # For horses
    "bridleway": ("brown", 0.3),
}

# From jeromer: https://gist.github.com/jeromer/2005586
def calculate_initial_compass_bearing(pointA, pointB):
    """
    Calculates the bearing between two points.
    The formulae used is the following:
        theta = atan2(sin(delta_long).cos(lat2),
                  cos(lat1).sin(lat2) - sin(lat1).cos(lat2).cos(delta_long))

    :Parameters:
      - `pointA: The tuple representing the latitude/longitude for the
        first point. Latitude and longitude must be in decimal degrees
      - `pointB: The tuple representing the latitude/longitude for the
        second point. Latitude and longitude must be in decimal degrees
    :Returns:
      The bearing in degrees
    :Returns Type:
      float
    """
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])

    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    # Now we have the initial bearing but math.atan2 return values
    # from -180 degrees to + 180 degrees which is not what we want for
    # a compass bearing The solution is to normalize the initial
    # bearing as shown below
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing
# End jeromer's code

def compass_direction(node_1_id, node_2_id):
    node_1 = nodes[node_1_id]
    node_2 = nodes[node_2_id]
    direction = calculate_initial_compass_bearing((node_1.lat, node_1.lon),
                                                  (node_2.lat, node_2.lon))
    return direction
    
def approx_compass_direction(node_1_id, node_2_id):
    orig_direction = compass_direction(node_1_id, node_2_id)
    direction = orig_direction
    directions = ("N", "NNE", "NE", "ENE",
                  "E", "ESE", "SE", "SES",
                  "S", "SWS", "SW", "WSW",
                  "W", "WNW", "NW", "NWN",)
    chunk = 360.0 / len(directions) # degrees
    direction = (direction + chunk / 2) % 360.0 # Makes N be 0-chunk degrees
    direction -= chunk # Makes N be <= 0
    for code in directions:
        if direction <= 0:
            return code
        direction -= chunk

    assert False, "Not reachable %g" % orig_direction

node_neighbours = {} # node_id -> [(node_id, way_id]]

def build_graph():
    start_time = time.time()
    plot_count = 0
    data = osmread.parse_file(OSM_FILE)
    end_time = time.time()
    print("Parsing file: %gs" % (end_time - start_time))
    for entity in data:
        if isinstance(entity, osmread.Node):
            nodes[entity.id] = entity
        elif isinstance(entity, osmread.Way):
            if "highway" in entity.tags:
                highway_type = entity.tags["highway"]
                if highway_type in ("construction", "proposed", "corridor",
                                    "elevator", "bridleway"):
                    continue
                if not CAR and highway_type in ("motorway", "trunk"):
                    continue
                if not CAR and not BIKE and highway_type in ("motorway", "trunk"):
                    continue
                if not WALKING and not BIKE:
                    if highway_type in ("cycleway", "path",):
                        continue
                    if ("motor_vehicle" in entity.tags and
                        entity.tags["motor_vehicle"] == "no"):
                        continue
                if not WALKING and highway_type in ("pedestrian", "platform",
                                                    "footway", "steps"):
                    continue
                ways[entity.id] = entity
                way_id = entity.id
                if "area" in entity.tags and entity.tags["area"] == "yes":
                    # All nodes connected (n^2)
                    for node_id in entity.nodes:
                        for other_node_id in entity.nodes:
                            if node_id != other_node_id:
                                node_neighbours.setdefault(node_id, []).append((other_node_id, way_id))
                else:
                    last_node_id = None
                    for node_id in entity.nodes:
                        if last_node_id is not None:
                            # FIXME: Care about oneway if walking? Don't
                            # think so. Though escalators can be a
                            # problem.
                            def is_oneway():
                                return ("oneway" in entity.tags and
                                        entity.tags["oneway"] == "yes" or
                                        "junction" in entity.tags and
                                        entity.tags["junction"] == "roundabout")
                                
                            if WALKING or not is_oneway():
                                node_neighbours.setdefault(node_id, []).append((last_node_id, way_id))
                            node_neighbours.setdefault(last_node_id, []).append((node_id, way_id))
                        last_node_id = node_id

        elif isinstance(entity, osmread.Relation):
            # TODO: Include highways here too!
            def get_closed_outer_ways():
                outer_ways = []
                for member_way in entity.members:
                    if member_way.role == "outer":
                        # Skipping inner holes
                        outer_way = ways.get(member_way.member_id)
                        if outer_way and outer_way.nodes[0] == outer_way.nodes[-1]:
                            outer_ways.append(outer_way)
                return outer_ways
            if "highway" in entity.tags:
                outer_ways = get_closed_outer_ways()
                if len(outer_ways) == 1:
                    pass
                    
#                print("Missing road: " + str(entity))
            pass
        else:
            print("Unknown type of entity:")
            print(entity)

    end_end_time = time.time()
    print("Building graph: %gs" % (end_end_time - end_time))

    edge_count = 0
    max_local_edge_count = 0
    max_local_edge_count_node = -1
    for (node_id, neighbour_list) in node_neighbours.iteritems():
        local_edge_count = len(neighbour_list)
        edge_count += local_edge_count
        if local_edge_count > max_local_edge_count:
            max_local_edge_count = local_edge_count
            max_local_edge_count_node = node_id

    print("%d nodes, %d edges. Node with most edges have %d." % (
        len(node_neighbours), edge_count, max_local_edge_count))

build_graph()

if False:
    printed_ways = set()
    for (neighbour_id, way_id) in node_neighbours[max_local_edge_count_node]:
        if way_id not in printed_ways:
            print(ways[way_id])
            printed_ways.add(way_id)

def distance(node_1_id, node_2_id):
    node_1 = nodes[node_1_id]
    node_2 = nodes[node_2_id]
    if False:
        # vincenty is the most accurate but slower. There is another that
        # might be 0.5% wrong that we could use instead.
        dist = vincenty((node_1.lat, node_1.lon),
                        (node_2.lat, node_2.lon))
    else:
        # great_circle is up to 0.5% wrong but about twice as fast.
        dist = great_circle((node_1.lat, node_1.lon),
                            (node_2.lat, node_2.lon))
    return dist.meters

def closest_way_node(origin_node_id):
    if origin_node_id in node_neighbours:
        return origin_node_id

    closest_node_id = -1
    closest_node_distance = INFINITY
    for node_id in node_neighbours.iterkeys():
        d = distance(origin_node_id, node_id)
        if d < closest_node_distance:
            closest_node_id = node_id
            closest_node_distance = d
    return closest_node_id

def get_lowest_estimated_to_target(node_id_list, graph_state):
    candidate = None
    candidate_cost = -1
    for node_id in list(node_id_list):
        if node_id in graph_state:
            estimated_cost_to_target = graph_state[node_id][3]
        else:
            assert False, "Unreachable? %d" % node_id
            estimated_cost_to_target = INFINITY
        if candidate is None or estimated_cost_to_target < candidate_cost:
            candidate = node_id
            candidate_cost = estimated_cost_to_target
    return candidate

def print_path(start_node_id, end_node_id, graph_state):
    path = []
    path.append(end_node_id)
    node_id = end_node_id
    path_length = 0
    total_cost = graph_state[end_node_id][0]
    while node_id != start_node_id:
        _, prev_node_id, via_way, _ = graph_state[node_id]
        path.append(via_way)
        path.append(prev_node_id)
        path_length += distance(node_id, prev_node_id)
        node_id = prev_node_id

    assert len(path) % 2 == 1
    path = list(reversed(path))

    # Compress the path (skip middle nodes on the same "way")
    # Todo: skip this in the loop above already.
    froms = []
    tos = []
    the_ways = []
    the_distances = []
    the_costs = []

    i = 1
    while i < len(path):
        froms.append(path[i - 1])
        # Add the Way id
        the_ways.append(path[i])
        cum_distance = distance(froms[-1], path[i + 1])
        cum_cost = get_cost_for_way(path[i], froms[-1], path[i + 1])
        headings = [compass_direction(path[i - 1], path[i + 1])]
        # Skip as long as it's the same way in the same approx direction
        do_compress = True
        while (do_compress and i + 2 < len(path) and (
                path[i + 2] == path[i] or
                ("name" in ways[path[i]].tags and
                "name" in ways[path[i + 2]].tags and
                ways[path[i]].tags["name"] == ways[path[i + 2]].tags["name"]) or
                ("name" not in ways[path[i]].tags and
                 "name" not in ways[path[i + 2]].tags and
                 ways[path[i]].tags["highway"] == ways[path[i + 2]].tags["highway"]))):
            # Check that the direction is about the same as well.
            # Only for unnamed roads?
            # 45 degrees variation is ok
            if "name" in ways[path[i]].tags:
                # Directions for named roads are irrelevant. Just
                # follow the named road.
                pass
            else:
                headings.append(compass_direction(path[i + 1], path[i + 3]))
                if max(headings) - 45 > min(headings):
                    break
            cum_distance += distance(path[i + 1], path[i + 3])
            cum_cost += get_cost_for_way(path[i + 2], path[i + 1], path[i + 3])
            i += 2

        tos.append(path[i + 1])
        the_distances.append(cum_distance)
        the_costs.append(cum_cost)
        i += 2

    assert len(froms) == len(tos)
    assert len(the_ways) == len(the_distances)
    assert len(froms) == len(the_distances)
    assert len(the_costs) == len(the_distances)

    for from_id, to_id, way_id, the_distance, the_cost in zip(froms, tos, the_ways, the_distances, the_costs):
        way = ways[way_id]
        if "name" in way.tags:
            highway_type = way.tags["highway"]
            way_name = way.tags["name"]
            if highway_type == "platform":
                way_description = "platform %s" % way_name
            elif highway_type in ("secondary", "residential", "tertiary",
                                  "primary", "living_street", "motorway",
                                  "trunk"):
                way_description = way_name
            else:
                way_description = "%s (%s)" % (way_name, way.tags)
        else:
            way_description = "%s" % way.tags
        print("%d meters %s along %s (%g minutes)" % (
            the_distance,
            approx_compass_direction(from_id, to_id),
            way_description,
            (the_cost / 60.0)))

    print("%d meters long" % path_length)
    print("Cost %g minutes" % (total_cost / 60))

if WALKING:
    FASTEST_POSSIBLE_M_PER_S = 7 / 3.6
    WALKING_FOOTPATH_M_PER_S = 7 / 3.6
    WALKING_OTHER_ROAD_M_PER_S = 6 / 3.6 # Discourage other roads.
    WALKING_COMPLICATED_ROAD_M_PER_S = 3 / 3.6 # Stairs
elif CAR:
    FASTEST_POSSIBLE_M_PER_S = 130 / 3.6
    CAR_WALKING_M_PER_S = 7 / 3.6
    CAR_TRACK_M_PER_S = 10 / 3.6
    CAR_LIVING_STREET_M_PER_S = 20 / 3.6
    CAR_RESIDENTIAL_STREET_M_PER_S = 35 / 3.6
    CAR_SECONDARY_M_PER_S = 60 / 3.6
    CAR_TERTIARY_LINK_M_PER_S = 20 / 3.6
    CAR_SECONDARY_LINK_M_PER_S = 30 / 3.6
    CAR_PRIMARY_LINK_M_PER_S = 50 / 3.6
    CAR_MOTORWAY_LINK_M_PER_S = 70 / 3.6
#    WALKING_OTHER_ROAD_M_PER_S = 6 / 3.6 # Discourage other roads.
#    WALKING_COMPLICATED_ROAD_M_PER_S = 3 / 3.6 # Stairs

    
def heuristic_cost(node_1_id, node_2_id):
    dist = distance(node_1_id, node_2_id)
    cost = dist / FASTEST_POSSIBLE_M_PER_S
    return cost

def get_cost_for_way(way_id, node_1_id, node_2_id):
    dist = distance(node_1_id, node_2_id)
    way = ways[way_id]
    highway_type = way.tags["highway"]
    if WALKING:
        speed = WALKING_OTHER_ROAD_M_PER_S
        if highway_type in ("cycleway", "path", "footway"):
            speed = WALKING_FOOTPATH_M_PER_S
        elif highway_type in ("steps", "track"):
            speed = WALKING_COMPLICATED_ROAD_M_PER_S
    elif CAR:
        if "maxspeed" in way.tags:
            if way.tags["maxspeed"] == "walk":
                speed = CAR_WALKING_M_PER_S
            else:
                speed = int(way.tags["maxspeed"]) * 0.98 / 3.6
        elif "maxspeed:forward" in way.tags and "maxspeed:backward" in way.tags:
            if way.nodes.index(node_1_id) < way.nodes.index(node_2_id):
                # Going forward
                speed = int(way.tags["maxspeed:forward"]) * 0.98 / 3.6
            else:
                # Going "backward"
                speed = int(way.tags["maxspeed:backward"]) * 0.98 / 3.6
        elif highway_type == "track":
            speed = CAR_TRACK_M_PER_S
        elif highway_type in ("living_street", "service"):
            speed = CAR_LIVING_STREET_M_PER_S
        elif highway_type in ("residential", "tertiary", "unclassified"):
            speed = CAR_RESIDENTIAL_STREET_M_PER_S
        elif highway_type in ("secondary", ):
            speed = CAR_SECONDARY_M_PER_S
        elif highway_type == "raceway":
            # Normally just temporary search hits. Raceways rarely
            # lead anywhere.
            speed = FASTEST_POSSIBLE_M_PER_S
        elif highway_type == "tertiary_link":
            speed = CAR_TERTIARY_LINK_M_PER_S
        elif highway_type == "secondary_link":
            speed = CAR_SECONDARY_LINK_M_PER_S
        elif highway_type == "primary_link":
            speed = CAR_PRIMARY_LINK_M_PER_S
        elif highway_type == "motorway_link":
            speed = CAR_MOTORWAY_LINK_M_PER_S
        else:
            print("Don't know speed for %s" % way.tags)
            assert False
            
    cost = dist / speed
    return cost

# A* won't work because of turning restrictions. If the way you reach
# a node affects how you can move on, it just won't work. Maybe if the
# graph is rebuilt somehow?

VERBOSE_A_STAR = True
def a_star_search(start_node, end_node):
    start_node_id = closest_way_node(start_node)
    current_node_id = start_node_id
    target_node_id = closest_way_node(end_node)

#    print(nodes[end_node])
#    print(nodes[target_node_id])

    print("Total distance between nodes is %d meters" % distance(current_node_id, target_node_id))

    graph_state = {} # Map from id to (cost, prev_node, via_way, estimated_cost_to_target)

    # Going to implement A*
    visited = set()
    visited.add(current_node_id)
    print(current_node_id)
    graph_state[current_node_id] = (
        0,
        None,
        None,
        heuristic_cost(current_node_id, target_node_id))

    nodes_with_possible_exits = set()
    nodes_with_possible_exits.add(current_node_id)
    nodes_already_evaluated = set()
    if VERBOSE_A_STAR:
        last_shortest_left = INFINITY
    while nodes_with_possible_exits is not None:
        current_node_id = get_lowest_estimated_to_target(nodes_with_possible_exits, graph_state)

        if VERBOSE_A_STAR:
            left_from_current_node = heuristic_cost(current_node_id, target_node_id)
            if left_from_current_node * 1.2 < last_shortest_left:
                # 20% closer than before
                last_shortest_left = left_from_current_node
                print("Evaluating (%d meters left):" % left_from_current_node)
                print_path(start_node_id, current_node_id, graph_state)
                #    print(nodes[current_node_id].tags)

        if current_node_id == target_node_id:
            # DONE!
            print("Found a route!")
            print_path(start_node_id, current_node_id, graph_state)
            return

        nodes_with_possible_exits.remove(current_node_id)
        nodes_already_evaluated.add(current_node_id)
        neighbour_ids = node_neighbours.get(current_node_id, [])
        cost_from_start_to_current_node = graph_state[current_node_id][0]
        for (neighbour_id, way_id) in neighbour_ids:
            if neighbour_id in nodes_already_evaluated:
                continue
            if neighbour_id not in nodes_with_possible_exits:
                nodes_with_possible_exits.add(neighbour_id)
            cost_from_current_node_to_neighbour = get_cost_for_way(way_id, current_node_id, neighbour_id)
            cost_from_start_this_way = cost_from_start_to_current_node + cost_from_current_node_to_neighbour
            if neighbour_id in graph_state:
                lowest_cost_from_start_for_neighbour = graph_state[neighbour_id][0]
                if cost_from_start_this_way <= lowest_cost_from_start_for_neighbour:
                    # This is not better.
                    continue
            # So this is the best so far
            graph_state[neighbour_id] = (
                cost_from_start_this_way,
                current_node_id,
                way_id,
                cost_from_start_this_way + heuristic_cost(neighbour_id, target_node_id))
    #        time.sleep(0.05)

start_node = 4796398549L # Westmansgatan 98
end_node = 4640788458L # Repslagaregatan 21
#end_node = 449228555L # A corner of Coop Lagerhyddan
end_node = 1191673060L # A corner of Himnavagen 13
end_node = 1088114320L # A corner of Gamla Radstugugatan 22a, Norrkoping
print("Total raw distance is %d meters" % distance(start_node, end_node))

a_star_search(start_node, end_node)
