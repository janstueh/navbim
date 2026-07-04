#!/usr/bin/env python3

from math import sqrt
from shapely.geometry import Point

def euclidean_distance_between_nodes(G, tn1, tn2):
    x1 = G.nodes[tn1]["position"]["x"]
    y1 = G.nodes[tn1]["position"]["y"]
    z1 = G.nodes[tn1]["position"]["z"]
    x2 = G.nodes[tn2]["position"]["x"]
    y2 = G.nodes[tn2]["position"]["y"]
    z2 = G.nodes[tn2]["position"]["z"]
    return sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)

def cost_between_nodes(G, tn1, tn2, penalize_z_movement=1.0):
    x1 = G.nodes[tn1]["position"]["x"]
    y1 = G.nodes[tn1]["position"]["y"]
    z1 = G.nodes[tn1]["position"]["z"]
    x2 = G.nodes[tn2]["position"]["x"]
    y2 = G.nodes[tn2]["position"]["y"]
    z2 = G.nodes[tn2]["position"]["z"]
    return sqrt((x1 - x2)**2 + (y1 - y2)**2 + (penalize_z_movement*(z1 - z2))**2)

def get_room_id_of_edge(graph, edge):
    return graph.get_edge_data(edge[0], edge[1]).get("room_id", "")

def get_room_name_of_edge(graph, edge):
    return graph.nodes[get_room_id_of_edge(graph, edge)].get("name", "")

def get_floor_of_edge(graph, edge):
    floor = graph.nodes[get_room_id_of_edge(graph, edge)].get("floor", "")
    if isinstance(floor, list):
        return floor[0] if floor else ""
    return floor

def find_floor_by_height(graph, coordinates):
    """ Find the floor based on the z-coordinate by checking existing floor nodes. """
    
    # Find nearest floor node in the graph based on z-coordinate
    z = coordinates[2]
    smallest_z_diff = float('inf')
    floor = None
    floor_nodes = [(node_id, data) for node_id, data in graph.nodes(data=True) 
                    if data.get("type") == "floor"]
    for node_id, data in floor_nodes:
        min_z = data.get("min_z", float('-inf'))
        max_z = data.get("max_z", float('inf'))
        if min_z <= z < max_z and z - min_z < smallest_z_diff:
            smallest_z_diff = z - min_z
            floor = data.get("name", f"Floor_{node_id}")
    if floor is not None:
        return floor

    print(f"Warning: No floor found for coordinates {coordinates}")
    return None

def find_room_by_coordinates(graph, coordinates, floor):
    """ Find the room node based on coordinates. """

    nodes = list(graph.nodes(data=True))  # Get list of nodes with data
    point = Point(coordinates[0], coordinates[1])

    # If no rooms are found, the boundaries are increased slightly
    buffer_radius = 0.0
    while buffer_radius <= 0.5:
        for node_id, data in nodes:
            if data.get("type") == "room" and floor in data.get("floor", []):
                if data.get("polygon").buffer(buffer_radius).contains(point):
                    return node_id
        buffer_radius += 0.1  # Increase buffer radius
                
    print(f"Warning: No room found for coordinates {coordinates} on floor {floor}")