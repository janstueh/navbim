#!/usr/bin/env python3

from shapely.geometry import mapping, shape

def convert_shapely_to_json(data):
    """
    Recursively convert all Shapely geometries in a data structure to GeoJSON format.
    """
    if isinstance(data, dict):
        for key, value in list(data.items()):
            if hasattr(value, '__geo_interface__'):
                # Convert Shapely geometry to GeoJSON
                data[key] = mapping(value)
            elif isinstance(value, (dict, list)):
                # Recursively process nested structures
                data[key] = convert_shapely_to_json(value)
    elif isinstance(data, list):
        for i, item in enumerate(data):
            if hasattr(item, '__geo_interface__'):
                data[i] = mapping(item)
            elif isinstance(item, (dict, list)):
                data[i] = convert_shapely_to_json(item)
    return data


def convert_json_to_shapely(data):
    """
    Recursively convert GeoJSON format back to Shapely geometries.
    """
    if isinstance(data, dict):
        # Check if this dict is a GeoJSON geometry
        if 'type' in data and 'coordinates' in data:
            try:
                return shape(data)
            except:
                pass
        # Otherwise process each value
        for key, value in list(data.items()):
            if isinstance(value, (dict, list)):
                data[key] = convert_json_to_shapely(value)
    elif isinstance(data, list):
        for i, item in enumerate(data):
            if isinstance(item, (dict, list)):
                data[i] = convert_json_to_shapely(item)
    return data