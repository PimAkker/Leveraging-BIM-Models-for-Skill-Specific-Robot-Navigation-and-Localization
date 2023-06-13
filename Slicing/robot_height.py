import xml.etree.ElementTree as ET
import pymesh

def get_height(element):
    if element.find('pose') is not None:
        pose = element.find('pose').text.split()
        return float(pose[2])
    if element.find('origin') is not None:
        pose = element.find('origin').get('xyz').split()
        return float(pose[2])
    return 0.0

def calculate_height_recursive(element, height_map, root, height_stl):
    height = get_height(element)
    parent_name = element.attrib.get("parent")

    if parent_name:
        if parent_name in height_map:
            parent_height = height_map[parent_name]
        else:
            parent_element = root.find(".//link[@name='{}']".format(parent_name))
            parent_height = calculate_height_recursive(parent_element, height_map, root, height_stl)

        height += parent_height

    visual_element = element.find('visual')
    geometry_element = None
    mesh_element = None
    if visual_element is not None:
        geometry_element = visual_element.find('geometry')
    if geometry_element is not None:
        mesh_element = geometry_element.find('mesh')
        
    height_map[element.attrib["name"]] = height
    
    if mesh_element is not None:
        stl_path = mesh_element.get('filename')
        stl_scale = mesh_element.get('scale').split()
        stl_path = stl_path[10:]
        stl_path = "ros_package/rosbot_description/src/" + stl_path
        mesh = pymesh.load_mesh(stl_path)
        min_coord, max_coord = mesh.bbox
        size = max_coord #- min_coord
        height += size[2]*float(stl_scale[2])


    height_stl[element.attrib["name"]] = height
    return height

def calculate_height(urdf_file):
    """
    Calculates the height of the robot.

    urdf_file: path to URDF
    """

    # Load the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # Find all joint and link elements
    joint_elements = root.findall(".//joint")
    link_elements = root.findall(".//link")

    # Create a dictionary to store the heights of each joint and link
    height_map = {}
    height_stl = {}

    # Calculate the height for each joint
    for joint_element in joint_elements:
        calculate_height_recursive(joint_element, height_map, root, height_stl)

    # Calculate the height for each link
    for link_element in link_elements:
        calculate_height_recursive(link_element, height_map, root, height_stl)

    # Find the base link and calculate the height recursively
    base_link_name = root.find(".//parent").attrib["link"]
    base_link = root.find(".//link[@name='{}']".format(base_link_name))
    robot_height = calculate_height_recursive(base_link, height_map, root, height_stl)

    # Find the largest height in the height map
    largest_height = max(height_stl.values())
    
    return largest_height

if  __name__ == "__main__":
    print(calculate_height("ros_package/rosbot_description/src/rosbot_description/urdf/rosbot.xacro"))
    # mesh = pymesh.load_mesh("ros_package/rosbot_description/src/rosbot_description/meshes/astra.stl")

    # # Get the size of the mesh
    # min_coord, max_coord = mesh.bbox
    # size = max_coord - min_coord

    # print("Size of the mesh (X, Y, Z):", size)
