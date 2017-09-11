#import osmapi
import osmread
import sys
import time
import geopy
from geopy.distance import vincenty
import random
import math

OSM_FILE = "linkoping-big.osm"

nodes = {}
ways = {}

roads = [] # Only roads that I care about
rails = [] # Only railroads that I care about
buildings = []

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

def approx_compass_direction(node_1_id, node_2_id):
    node_1 = nodes[node_1_id]
    node_2 = nodes[node_2_id]
    direction = calculate_initial_compass_bearing((node_1.lat, node_1.lon),
                                                  (node_2.lat, node_2.lon))
    directions = ("N", "NNE", "NE", "ENE",
                  "E", "ESE", "SE", "SES",
                  "S", "SWS", "SW", "WSW",
                  "W", "WNW", "NW", "NWN",)
    chunk = 360.0 / len(directions) # degrees
    direction += chunk / 2 # To make the math more obvious
    for code in directions:
        if direction <= chunk:
            return code
        direction -= chunk

    assert False, "Not reachable"

node_neighbours = {} # node_id -> [(node_id, way_id]]

plot_count = 0
data = osmread.parse_file(OSM_FILE)
for entity in data:
    if isinstance(entity, osmread.Node):
        nodes[entity.id] = entity
    elif isinstance(entity, osmread.Way):
        if "highway" in entity.tags:
            highway_type = entity.tags["highway"]
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
                        node_neighbours.setdefault(node_id, []).append((last_node_id, way_id))
                        node_neighbours.setdefault(last_node_id, []).append((node_id, way_id))
                    last_node_id = node_id
            
    elif isinstance(entity, osmread.Relation):
        pass
    else:
        print("Unknown type of entity:")
        print(entity)

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

printed_ways = set()
for (neighbour_id, way_id) in node_neighbours[max_local_edge_count_node]:
    if way_id not in printed_ways:
        print(ways[way_id])
        printed_ways.add(way_id)

def distance(node_1_id, node_2_id):
    node_1 = nodes[node_1_id]
    node_2 = nodes[node_2_id]
    dist = vincenty((node_1.lat, node_1.lon),
                    (node_2.lat, node_2.lon))
    return dist.meters

start_node = 4796398550L
if start_node in node_neighbours:
    node_id = start_node
else:
    closest_node_id = -1
    closest_node_distance = 99999999
    for node_id in node_neighbours.iterkeys():
        d = distance(start_node, node_id)
        if d < closest_node_distance:
            closest_node_id = node_id
            closest_node_distance = d
    node_id = closest_node_id
        
visited = set()
visited.add(node_id)
while node_id is not None:
#    print(nodes[node_id])
    neighbour_ids = node_neighbours[node_id]
    option_count = len(neighbour_ids)
    # print("%d options" % option_count)
    current_node = nodes[node_id]
    for (neighbour_id, way_id) in neighbour_ids:
        neighbour = nodes[neighbour_id]
        # print("%d: %r" % (neighbour_id,
        #                   distance(node_id, neighbour_id)))

    options = set()
    for (neighbour_id, way_id) in neighbour_ids:
        if neighbour_id not in visited:
            options.add((neighbour_id, way_id))

    if options:
        (next_node_id, way_id) = random.choice(tuple(options))
        print("Going %s %d meters via" % (
            approx_compass_direction(node_id, next_node_id),
            distance(node_id, next_node_id)))
        print(ways[way_id].tags)

        node_id = next_node_id
        visited.add(node_id)
    else:
        node_id = None

    time.sleep(1)

