#import osmapi
import osmread
import sys
import time

OSM_FILE = "linkoping-big.osm"
# start_time = time.time()
# data = osmread.parse_file(OSM_FILE)
# 
# count = 0
# for _ in data:
#     count += 1
# end_time = time.time()
# print("%g seconds to read osm file" % (end_time - start_time))
# print("%d things in the osm file" % count)

nodes = {}
ways = {}

roads = [] # Only roads that I care about
rails = [] # Only railroads that I care about
aeroways = []
waterways = []
lakes = []
buildings = []
man_made_objects = []

landuses_areas = {}
leisure_areas = {}

boundaries = []

LAKE_WATER_TYPES = ("lake", "pond", "reservoir", "moat")
color_per_landuse_type = {
    "allotments": "green",
    "basin": "blue", # Maybe?
    "brownfield": "brown",
    "cemetery": "green",
    "commercial": "beige",
    "construction": "brown",
    "farmland": "green",
    "farmyard": "green",
    "forest": "green",
    "garages": "brown",
    "garages": "grey",
    "grass": "lime",
    "greenfield": "lime",
    "industrial": "grey",
    "meadow": "green",
    "military": "grey",
    "orchard": "green",
    "quarry": "grey",
    "railway": "grey",
    "recreation_ground": "lime",
    "religious": "lime",
    "reservoir": "blue",
    "residential": "beige",
    "retail": "beige",
    "village_green": "lime",
    }

color_per_leisure_type = {
    "park": "green",
    "dog_park": "green",
    "garden": "green",
    "pitch": "green",
    "playground": "green",
    "golf_course": "green",
    "dance": None,
    "miniature_golf": None,
    "slipway": None,
    "stadium": "brown",
    "fitness_station": None,
    "water_park": None,
    "nature_reserve": "green",
    "sports_centre": None,
    "swimming_pool": "blue",
    "marina": None,
    "ice_rink": None,
    "track": None,
    }

color_and_width_per_waterway_type = {
    "canal": ("blue", 3.0),
    "dam": (None, 1.0),
    "ditch": (None, 1.0),
    "drain": (None, 1.0),
    "weir": (None, 1.0),
    "riverbank": (None, 1.0),
    "river": ("blue", 4.0),
    "stream": ("blue", 2.0),
}

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

plot_count = 0
PLOT_LIMIT = 15000
data = osmread.parse_file(OSM_FILE)
for entity in data:
    if isinstance(entity, osmread.Node):
        nodes[entity.id] = entity
    elif isinstance(entity, osmread.Way):
        ways[entity.id] = entity
        if "landuse" in entity.tags:
            landuse_type = entity.tags["landuse"]
            landuse_list = landuses_areas.setdefault(landuse_type, [])
            landuse_list.append(entity)
            if landuse_type not in color_per_landuse_type:
                print("Unsupported landuse type: %s" % landuse_type)
        elif "natural" in entity.tags:
            natural_type = entity.tags["natural"]
            if natural_type == "water":
                if "water" in entity.tags:
                    if entity.tags["water"] in LAKE_WATER_TYPES:
                        lakes.append(entity)
        elif "highway" in entity.tags:
            highway_type = entity.tags["highway"]
            if highway_type in color_and_width_per_highway_type:
                if color_and_width_per_highway_type[highway_type][0]:
                    roads.append(entity)
            else:
                print("Unknown road type (ignoring):")
                print(highway_type)
                pass
        elif "waterway" in entity.tags:
            waterway_type = entity.tags["waterway"]
            if waterway_type in color_and_width_per_waterway_type:
                if color_and_width_per_waterway_type[waterway_type][0]:
                    waterways.append(entity)
            else:
                print("Unknown waterway type (ignoring):")
                print(waterway_type)
                pass
        elif "aeroway" in entity.tags:
            aeroway_type = entity.tags["aeroway"]
            if aeroway_type in ("runway", "taxiway", "apron", "helipad", ):
                aeroways.append(entity)
            elif aeroway_type in ("terminal", "hangar"):
                buildings.append(entity)
            else:
                print("Unknown aeroways type (ignoring):")
                print(aeroway_type)
                pass
        elif "building" in entity.tags:
            buildings.append(entity)
        elif "man_made" in entity.tags:
            man_made_objects.append(entity)
        elif "amenity" in entity.tags:
            # Ignoring these (bars, fountains, police, ...) for now
            pass
        elif "leisure" in entity.tags:
            leisure_type = entity.tags["leisure"]
            leisure_list = leisure_areas.setdefault(leisure_type, [])
            leisure_list.append(entity)
            if leisure_type not in color_per_leisure_type:
                print("Unsupported leisure type: %s" % leisure_type)
        elif "boundary" in entity.tags:
            print(entity)
            if "admin_level" in entity.tags:
                boundaries.append(entity)
            else:
                print("Unknown boundary:")
                print(entity)
        elif "place" in entity.tags:
            # Ignoring these for now.
            pass
        elif "building:part" in entity.tags:
            # Ignoring these for now.
            pass
        elif "indoor" in entity.tags:
            # Ignoring these for now.
            pass
        elif "public_transport" in entity.tags:
            # Ignoring these for now.
            pass
        elif "historic" in entity.tags:
            # Ignoring these for now.
            pass
        elif "attraction" in entity.tags:
            # Ignoring these for now.
            pass
        elif "tourism" in entity.tags:
            # Ignoring these for now.
            pass
        elif "piste:type" in entity.tags:
            # Ignoring these for now.
            pass
        elif "golf" in entity.tags:
            # Ignoring these for now.
            pass
        elif "disc_golf" in entity.tags:
            # Ignoring these for now.
            pass
        elif "power" in entity.tags:
            # Ignoring these for now.
            pass
        elif "allotments" in entity.tags:
            # Ignoring these for now.
            pass
        elif "railway" in entity.tags:
            railway_type = entity.tags["railway"]
            if railway_type in ("rail", "turntable"):
                rails.append(entity)
            elif railway_type in ("disused", "abandoned"):
                rails.append(entity)
            elif railway_type == "platform":
                # Ignoring this
                pass
            else:
                print("Unknown type of railway: %s" % railway_type)
        elif "barrier" in entity.tags:
            # Ignoring these (fences, walls, ...) for now 
            pass
        elif (not entity.tags or
              len(entity.tags) == 1 and "source" in entity.tags or
              len(entity.tags) == 1 and "area" in entity.tags or
              len(entity.tags) == 1 and "surface" in entity.tags or
              len(entity.tags) == 1 and "note" in entity.tags or
              len(entity.tags) == 1 and "building:use" in entity.tags or
              len(entity.tags) == 2 and "note" in entity.tags and "surface" in entity.tags or
              len(entity.tags) == 2 and "note" in entity.tags and "source" in entity.tags or
              len(entity.tags) == 1 and "description" in entity.tags or
              len(entity.tags) == 2 and "access" in entity.tags and "note" in entity.tags or
              len(entity.tags) == 2 and "access" in entity.tags and "service" in entity.tags or
              len(entity.tags) == 2 and "wikidata" in entity.tags and "FID" in entity.tags
        ):
            # Empty, so just a resource for others.
            pass
        elif (len(entity.tags) == 1 and "fence" in entity.tags or
              len(entity.tags) == 1 and "addr:interpolation" in entity.tags or
              "addr:housenumber" in entity.tags and "building" not in entity.tags):
            # Weird thing.
            pass
        else:
            pass
            print("Unknown entity:")
            print(entity.tags)
#            sys.exit(1)
    elif isinstance(entity, osmread.Relation):
        def get_closed_outer_ways():
            outer_ways = []
            for member_way in entity.members:
                if member_way.role == "outer":
                    # Skipping inner holes
                    outer_way = ways.get(member_way.member_id)
                    if outer_way and outer_way.nodes[0] == outer_way.nodes[-1]:
                        outer_ways.append(outer_way)
            return outer_ways
            
        if "type" in entity.tags and entity.tags["type"] == "multipolygon":
            if "natural" in entity.tags and entity.tags["natural"] == "water":
                if "water" in entity.tags and entity.tags["water"] in LAKE_WATER_TYPES:
                    lakes.extend(get_closed_outer_ways())
            elif "building" in entity.tags:
                buildings.extend(get_closed_outer_ways())
            elif "landuse" in entity.tags:
                landuse_type = entity.tags["landuse"]
                landuse_list = landuses_areas.setdefault(landuse_type, [])
                landuse_list.extend(get_closed_outer_ways())
                if landuse_type not in color_per_landuse_type:
                    print("(multi) Unsupported landuse type: %s" % landuse_type)
            elif "leisure" in entity.tags:
                leisure_type = entity.tags["leisure"]
                leisure_list = leisure_areas.setdefault(leisure_type, [])
                leisure_list.extend(get_closed_outer_ways())
                if leisure_type not in color_per_leisure_type:
                    print("Unsupported leisure type: %s" % leisure_type)
            elif "natural" in entity.tags:
                natural_type = entity.tags["natural"]
                if natural_type == "water" and "water" in entity.tags:
                    if entity.tags["water"] == "lake":
                        lakes.extend(get_closed_outer_ways())
            elif "highway" in entity.tags:
                highway_type = entity.tags["highway"]
                if highway_type in color_and_width_per_highway_type:
                    if color_and_width_per_highway_type[highway_type][0]:
                        roads.extend(get_closed_outer_ways())
                else:
                    print("(multi) Unknown road type (ignoring):")
                    print(highway_type)
                    pass
            elif "waterway" in entity.tags:
                waterway_type = entity.tags["waterway"]
                if waterway_type in color_and_width_per_waterway_type:
                    if color_and_width_per_waterway_type[waterway_type][0]:
                        waterways.extend(get_closed_outer_ways())
                else:
                    print("(multi) Unknown waterway type (ignoring):")
                    print(waterway_type)
                    pass
        pass
    else:
        print("Unknown type of entity:")
        print(entity)
        
print("%d nodes, %d roads, %d waterways, %d lakes, %d buildings, %d railways, %d aeroways, %d boundaries" % (
    len(nodes),
    len(roads),
    len(waterways),
    len(lakes),
    len(buildings),
    len(rails),
    len(aeroways),
    len(boundaries),
))
for area_lists in (landuses_areas, leisure_areas):
    for (area_type, area_list) in area_lists.iteritems():
        if area_list:
            print("%s: %d" % (area_type, len(area_list)))

# import matplotlib.pyplot as plt
# import matplotlib.patches as mpatches

# import cartopy.crs as ccrs

# desired_projections = [ccrs.PlateCarree(),
#                        ccrs.RotatedPole(pole_latitude=45, pole_longitude=180)]
# for plot_num, desired_proj in enumerate(desired_projections):

#     ax = plt.subplot(2, 1, plot_num + 1, projection=desired_proj)

#     ax.set_global()

#     # ax.add_patch(mpatches.Rectangle(xy=[-70, -45], width=90, height=90,
#     #                                 facecolor='blue',
#     #                                 alpha=0.2,
#     #                                 transform=ccrs.PlateCarree())
#     #              )

#     # ax.add_patch(mpatches.Rectangle(xy=[70, -45], width=90, height=90,
#     #                                 facecolor='red',
#     #                                 alpha=0.2,
#     #                                 transform=ccrs.Geodetic())
#     #              )
#     ax.add_patch(mpatches.Polygon([[80, -50], [34, 0], [0, -30], [80, -50]],
#                                   facecolor='yellow',
#                                   alpha=0.8,
#                                   transform=ccrs.Geodetic())
#                  )

#     ax.gridlines()
#     ax.coastlines()

# plt.show()


import cartopy.crs as ccrs
import matplotlib.pyplot as plt

#ax = plt.axes(projection=ccrs.Mollweide())
ax = plt.axes(projection=ccrs.PlateCarree()) # Good for the USA
ax = plt.axes(projection=ccrs.EuroPP()) # Good for the EU
ax.coastlines()
#ax.stock_img()
ax.set_global() # Show the whole globe.

def is_polygon_clockwise(coordinate_list):
    # http://en.wikipedia.org/wiki/Shoelace_formula
    # https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
    area = 0
    last_xy = coordinate_list[-1]
    for xy in coordinate_list:
        area += (xy[0] - last_xy[0]) * (xy[1] + last_xy[1])
        last_xy = xy

    return area > 0

if False:
    ax.add_patch(mpatches.Rectangle(xy=[-70, -45], width=90, height=90,
                                    facecolor='blue',
                                    alpha=0.2,
                                    transform=ccrs.PlateCarree())
                 )

    ax.add_patch(mpatches.Rectangle(xy=[70, -45], width=90, height=90,
                                    facecolor='red',
                                    alpha=0.2,
                                    transform=ccrs.Geodetic())
                 )
    ax.add_patch(mpatches.Polygon([[80, -50], [34, 0], [0, -30], [80, -50]],
                                  facecolor='yellow',
                                  alpha=0.8,
                                  transform=ccrs.Geodetic())
                 )

def polygon_coords_from_nodes(node_list):
    lons = []
    lats = []
    for node_id in node_list:
        node = nodes[node_id]
        lons.append(node.lon)
        lats.append(node.lat)
    polygon_coords = zip(lons, lats)
    if is_polygon_clockwise(polygon_coords):
        polygon_coords = list(reversed(polygon_coords))

    return polygon_coords

def plot_areas_from_entity_list(entity_list, color):
    from matplotlib.patches import Polygon
    from matplotlib.collections import PolyCollection
    area_list = []
    for entity in entity_list:
        polygon_coords = polygon_coords_from_nodes(entity.nodes)
        area_list.append(polygon_coords)
    area_polys = PolyCollection(area_list,
                                color=color,
                                alpha=0.2,
                                transform=ccrs.Geodetic())
    ax.add_collection(area_polys)

if True:
    plot_areas_from_entity_list(buildings, "brown")

if True:
    plot_areas_from_entity_list(lakes, "blue")

if True:
    for area_lists, color_table in ((landuses_areas, color_per_landuse_type),
                                    (leisure_areas, color_per_leisure_type)):
        for (area_type, area_list) in area_lists.iteritems():
            if area_list and area_type in color_table:
                plot_areas_from_entity_list(area_list, color_table[area_type])

if False:
    from matplotlib.patches import Polygon
#    from matplotlib.collections import PolyCollection
#    lake_list = []
    for lake in lakes:
        lake_polygon_coords = polygon_coords_from_nodes(lake.nodes)
        lake_polygon = Polygon(lake_polygon_coords,
                               color="blue",
                               alpha=0.2,
                               transform=ccrs.Geodetic())
#        print(lake_polygon)
        ax.add_patch(lake_polygon)
        # ax.add_patch(mpatches.Polygon([[80, -50], [34, 0], [0, -30], [80, -50]],
        #                               facecolor='yellow',
        #                               alpha=0.8,
        #                               transform=ccrs.Geodetic())
        # )
#        lake_list.append(lake_polygon)
#    lake_polys = PolyCollection(lake_list, color="blue",
#                                transform=ccrs.Geodetic())
#    ax.add_collection(lake_polys)

def plot_line(entity, color, width=1.0):
    lons = []
    lats = []
    for node_id in entity.nodes:
        node = nodes[node_id]
        lons.append(node.lon)
        lats.append(node.lat)
    plt.plot(lons, lats, color=color, marker=None, linewidth=width,
             transform=ccrs.Geodetic())
    
if True:
    for entity in roads:
        highway_type = entity.tags["highway"]
        color, width = color_and_width_per_highway_type[highway_type]
        # if "maxspeed" in entity.tags:
        #     maxspeed = int(entity.tags["maxspeed"])
        #     if maxspeed > 30:
        #         color = "blue"
        #     if maxspeed > 50:
        #         color = "green"
        #     if maxspeed > 70:
        #         color = "red"

        plot_line(entity, color, width)
        plot_count += 1
        if plot_count >= PLOT_LIMIT:
            print("Aborting...")
            break

if True:
    for entity in waterways:
        waterway_type = entity.tags["waterway"]
        color, width = color_and_width_per_waterway_type[waterway_type]
        plot_line(entity, color, width)
        plot_count += 1
        if plot_count >= PLOT_LIMIT:
            print("Aborting...")
            break

if True:
    for entity in rails:
        plot_line(entity, "grey")
        plot_count += 1
        if plot_count >= PLOT_LIMIT:
            print("Aborting...")
            break

if True:
    for entity in aeroways:
        plot_line(entity, "grey")
        plot_count += 1
        if plot_count >= PLOT_LIMIT:
            print("Aborting...")
            break

if True:
    for entity in boundaries:
        assert "admin_level" in entity.tags, entity
        admin_level = int(entity.tags["admin_level"])
        plot_line(entity, "white")
        plot_count += 1
        if plot_count >= PLOT_LIMIT:
            print("Aborting...")
            break

plt.show()

# import cartopy.crs as ccrs
# import matplotlib.pyplot as plt

# ax = plt.axes(projection=ccrs.PlateCarree())
# ax.coastlines()

# plt.show()
