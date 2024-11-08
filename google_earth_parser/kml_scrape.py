import xml.etree.ElementTree as ET
from geopy.distance import geodesic
import json
import os

INPUT_FILE_NAME = "outdoor_sim.kml"

def parse_kml(file_path, output_file):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Namespaces for KML
    ns = {
        'kml': 'http://www.opengis.net/kml/2.2',
        'gx': 'http://www.google.com/kml/ext/2.2'
    }

    # Initialize variables
    origin_lat = origin_lon = None
    objects = {}

    for placemark in root.findall('.//kml:Placemark', ns):
        placemark_id = placemark.get('id')
        name = placemark.find('kml:name', ns).text
        coordinates_str = placemark.find('.//kml:coordinates', ns).text.strip()

        # Convert string of coordinates to a list of tuples (lon, lat, alt)
        coordinates = [
            tuple(map(float, coord.split(','))) for coord in coordinates_str.split()
        ]

        # Set the first coordinate as the origin if it hasn't been set yet
        if origin_lat is None and origin_lon is None:
            origin_lon = -87.56856191077318 # Currently hard coded to the current projects origin
            origin_lat = 33.20219960569318 

        # Convert each coordinate to meters relative to origin
        relative_coords = []
        for lon, lat, alt in coordinates:
            east_distance = geodesic((origin_lat, origin_lon), (origin_lat, lon)).meters
            north_distance = geodesic((origin_lat, origin_lon), (lat, origin_lon)).meters

            east_distance = east_distance if lon > origin_lon else -east_distance
            north_distance = north_distance if lat > origin_lat else -north_distance

            relative_coords.append((east_distance, north_distance, alt))

        # Add new object to the current objects dictionary
        objects[placemark_id] = {
            'type': name,
            'coordinates': relative_coords
        }

    # Save objects to the output file
    with open(output_file, 'w') as f:
        json.dump(objects, f, indent=4)

output_file = "kml_output/" + os.path.splitext(INPUT_FILE_NAME)[0] + ".json"
parse_kml(f"kml_input/{INPUT_FILE_NAME}", output_file)
with open(output_file, 'r') as f:
    objects = json.load(f)
    for obj_id, obj in objects.items():
        print("ID:", obj_id)
        print("Type:", obj['type'])
        print("Coordinates (relative to origin in meters):", obj['coordinates'])