import json
import os
import random

# Constants for input, output directories, and object heights
INPUT_FILE = "outdoor_sim.json"
BASE_HEIGHT = 0.1           
BUILDING_MIN = 5.0          
BUILDING_MAX = 15.0         
CURB_HEIGHT = 0.2           
TREE_H_MAX = 10
TREE_H_MIN = 6
TREE_W_MAX = 4
TREE_W_MIN = 1.5
SIDEWALK_HEIGHT = 0.1
END_HEIGHT = 8

# Define colors for each object type
COLOR_MAP = {
    "building": "0.8 0.5 0.5",  
    "base": "0 0 0",            
    "curb": "0.5 1 0.6",  
    "sidewalk": ".6 .6 .6",
    "end": "0 0 0"         
}

def create_wbt_solid(obj_id, obj_type, coordinates):
    if obj_type.lower() == "building":
        height = random.uniform(BUILDING_MIN, BUILDING_MAX)
        base_z = CURB_HEIGHT 
    elif obj_type.lower() == "base":
        height = BASE_HEIGHT
        base_z = -BASE_HEIGHT  
    elif obj_type.lower() == "curb":
        height = CURB_HEIGHT
        base_z = 0 
    elif obj_type.lower() == "sidewalk":
        height = SIDEWALK_HEIGHT
        base_z = CURB_HEIGHT
    elif obj_type.lower() == "end":
        height = END_HEIGHT
        base_z = 0
    else:
        height = 100
        base_z = 0

    # Generate the 3D points for the IndexedFaceSet
    points = []
    num_points = len(coordinates)

    # Bottom face vertices
    for lon, lat, _ in coordinates:
        points.append(f"{lon} {lat} {base_z}")  # Bottom face vertices at base_z

    # Top face vertices
    for lon, lat, _ in coordinates:
        points.append(f"{lon} {lat} {base_z + height}")  # Top face vertices at base_z + height

    # Create face index array for the sides, bottom, and top
    face_indices = []
    
    # Create side faces
    for i in range(num_points):
        # Connect the bottom face to the top face
        bottom_left = i
        bottom_right = (i + 1) % num_points
        top_left = bottom_left + num_points
        top_right = bottom_right + num_points
        
        face_indices.extend([bottom_left, bottom_right, top_right, top_left, -1])  # Using -1 to end the face

    # Bottom face indices (assumes a convex shape, clockwise order)
    bottom_face = [i for i in range(num_points)]
    bottom_face.append(-1)  # End of the face
    face_indices.extend(bottom_face)

    # Top face indices (clockwise order for correct orientation)
    top_face = [i + num_points for i in range(num_points)]
    top_face.append(-1)
    face_indices.extend(top_face)

    # Format IndexedFaceSet in .wbt syntax
    polygon_wbt = f"""
    Solid {{
        name "{obj_id}"  # Use object ID as the name
        children [
            Shape {{
                appearance PBRAppearance {{
                    baseColor {COLOR_MAP.get(obj_type.lower(), "0.5 0.5 0.5")}  # Unique color based on object type
                }}
                geometry IndexedFaceSet {{
                    coord Coordinate {{
                        point [
                            {' '.join(points)}
                        ]
                    }}
                    coordIndex [
                        {' '.join(map(str, face_indices))}
                    ]
                }}
            }}
        ]
    }}
    """
    return polygon_wbt

def create_tree_object(obj_id, coordinates):
    lon, lat, _ = coordinates[0]

    # Create a tree object using the Tree.proto
    tree_wbt = f"""
        Tree {{
            translation {lon} {lat} 0
            name "{obj_id}"  # Use object ID as the name
            scale {random.uniform(TREE_W_MIN, TREE_W_MAX)} {random.uniform(TREE_W_MIN, TREE_W_MAX)} {random.uniform(TREE_H_MIN, TREE_H_MAX)}
        }}
    """
    return tree_wbt

def create_street_light(obj_id, obj_type, coordinates):
    lon, lat, _ = coordinates[0]

    # Define rotation based on street light type
    rotation_map = {
        "sl_n": "0 0 1 1.89",
        "sl_e": "0 0 1 0.35",
        "sl_s": "0 0 1 -1.24",
        "sl_w": "0 0 1 -2.8"
    }
    rotation = rotation_map.get(obj_type.lower(), "0 0 1 1.57")

    # Create the street light object
    street_light_wbt = f"""
        ControlledStreetLight {{
            translation {lon} {lat} 0.1
            rotation {rotation}
            name "{obj_id}"  # Use object ID as the name
        }}
    """
    return street_light_wbt

def create_stop_sign_object(obj_id, obj_type, coordinates):
    lon, lat, _ = coordinates[0]

    # Define rotation based on the direction
    rotation_map = {
        "stop_sign_n": "1.89",
        "stop_sign_e": "0.35",
        "stop_sign_s": "-1.24",
        "stop_sign_w": "-2.8"
    }
    rotation = rotation_map.get(obj_type.lower(), "0")

    # Create a stop sign object using the StopSign proto
    stop_sign_wbt = f"""
        StopSign {{
            translation {lon} {lat} 0
            name "{obj_id}"
            rotation 0 0 1 {rotation}
        }}
    """
    return stop_sign_wbt

def generate_wbt_file(json_file, output_file):
    with open(json_file, 'r') as f:
        objects = json.load(f)

    # Start building the WBT content with headers and initial settings
    wbt_content = """#VRML_SIM R2023b utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/apartment_structure/protos/Wall.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/trees/protos/Tree.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/lights/protos/ControlledStreetLight.proto"

WorldInfo {
}
Viewpoint {
  orientation -0.3662014403983798 0.3643763742240471 0.8562279853856033 1.7303378312555011
  position 7.232180759536639 -82.0574949333934 78.77704826950453
}
TexturedBackground {
}
TexturedBackgroundLight {
}
"""

    for obj_id, obj_data in objects.items():
        obj_type = obj_data['type']
        
        # Skip if object type is "Road" # I used road as a guidline while tracing the curbs
        if obj_type.lower() == "road":
            continue
            
        coordinates = obj_data['coordinates']

        if obj_type.lower() == "tree":
            wbt_content += create_tree_object(obj_id, coordinates)
        elif obj_type.lower() in {"sl_n", "sl_e", "sl_s", "sl_w"}:
            wbt_content += create_street_light(obj_id, obj_type, coordinates)
        elif obj_type.lower() in ["stop_sign_n", "stop_sign_e", "stop_sign_s", "stop_sign_w"]:
            wbt_content += create_stop_sign_object(obj_id, obj_type, coordinates)
        else:
           wbt_content += create_wbt_solid(obj_id, obj_type, coordinates)

    with open(output_file, 'w') as f:
        f.write(wbt_content)

# Paths
input_json_path = os.path.join("kml_output", INPUT_FILE)
output_file_name = os.path.splitext(INPUT_FILE)[0] + ".wbt"
output_wbt_path = os.path.join("wbt_output", output_file_name)

# Generate the .wbt file
generate_wbt_file(input_json_path, output_wbt_path)

print("Generated .wbt file:", output_wbt_path)
