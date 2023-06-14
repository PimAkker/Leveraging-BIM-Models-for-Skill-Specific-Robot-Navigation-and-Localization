from pymesh.boolean import boolean
from pymesh.misc import Quaternion
from pymesh.meshutils import generate_box_mesh, separate_mesh, remove_isolated_vertices, merge_meshes
from pymesh.meshio import form_mesh, save_mesh
import pymesh
import numpy as np
from numpy.linalg import norm
import requests
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

import os

def update_server(coordinates, height):
    """ Update the server with the coordinates of the robot.

    Args:
        coordinates (list): The coordinates of the robot.
    Returns:
        None
    """
    # x_offset =  -13.366
    x_offset = -1.6
    y_offset = 7.1558

    # coordinates = [xmin, xmax, ymin, ymax]
    coordinates = coordinates + np.array([x_offset, x_offset, y_offset, y_offset])
    corner1 = [coordinates[0], coordinates[2], -0.5]
    corner2 = [coordinates[1], coordinates[3], 3]


    box = generate_box_mesh(corner1, corner2)
    
    box_ver = box.vertices
    box_face = box.faces

    string_ver = "("
    for ver in box_ver:
        for number in ver:
            string_ver= string_ver + str(number) + ", "
    string_ver = string_ver[0:-2] + ")"
    
    string_face = "("
    for face in box_face:
        for number in face:
            string_face= string_face + str(number) + ", "
    string_face = string_face[0:-2] + ")"
    

    T_coordin = [0, 1, height]

    T_input = "(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, " + str(T_coordin[0]) + ", " + str(T_coordin[1]) + ", " + str(T_coordin[2]) + ")"

    url = 'http://localhost:9090/update'
    prefix = "PREFIX : <http://bedrock/> PREFIX props: <https://w3id.org/props#> INSERT DATA {"
    object_str = ":unknown_object" + str(np.random.randint(999999))
    faces = " props:Faces '"
    verts = " props:Verts '"
    T_str = " props:T '"
    edges = " props:Edges '"
    ref = " props:Reference 'Generic-wall-for-test' .}"
    full_query = prefix + object_str + faces + string_face + "' . " + object_str + verts + string_ver + "' . " + object_str + T_str + T_input + "' . " + object_str + edges + "(0, 0, 0)" + "' . " + object_str + ref
    myobj = {"query": full_query}
    x = requests.post(url, json = myobj)
    
    return

def slice_height(mesh, direction, height):
    """ Slice a given 3D mesh at a certain height.

    Args:
        mesh (:class:`Mesh`): The mesh to be sliced.
        direction (:class:`numpy.ndaray`): Direction orthogonal to the slices.
        height (int): height at which the mesh is sliced.

    Returns:
        A :class:`Mesh` object, representing a single slice.
    """
    N = 1

    if mesh.dim != 3:
        raise NotImplementedError("Only slicing 3D mesh is supported.")

    bbox_min, bbox_max = mesh.bbox
    slice_location = height - bbox_min

    radius = norm(bbox_max - slice_location)
    direction = np.array(direction)
    direction = direction / norm(direction)

    proj_len = np.dot(mesh.vertices, direction)
    min_val = np.amin(proj_len)
    max_val = np.amax(proj_len)
    mid_val = 0.5 * (min_val + max_val)
    intercepts = np.linspace(min_val - mid_val, max_val - mid_val, N+2)[1:-1]
    assert(len(intercepts) == N)
    if N%2 == 1:
        intercepts = np.append(intercepts, intercepts[-1]+radius)

    boxes = []
    for low, high in intercepts.reshape((-1, 2), order="C"):
        min_corner = -np.ones(3) * (radius+1)
        max_corner =  np.ones(3) * (radius+1)
        min_corner[2] = low
        max_corner[2] = high
        box = generate_box_mesh(min_corner, max_corner)
        boxes.append(box)

    num_boxes = len(boxes)
    boxes = merge_meshes(boxes)
    rot = Quaternion.fromData(
            np.array([0.0, 0.0, 1.0]), np.array(direction)).to_matrix()
    boxes = form_mesh(np.dot(rot, boxes.vertices.T).T + slice_location, boxes.faces)

    slabs = boolean(boxes, mesh, "intersection")

    cross_secs = []
    source = slabs.get_attribute("source").ravel()
    selected = source == 1
    cross_section_faces = slabs.faces[selected]
    cross_section = form_mesh(slabs.vertices, cross_section_faces)

    intersects = np.dot(slabs.vertices, direction).ravel() - \
            np.dot(slice_location, direction)
    eps = (max_val - min_val) / (2 * N)

    for i,val in enumerate(intercepts[:N]):
        selected_vertices = np.logical_and(
                intersects > val - eps,
                intersects < val + eps)
        selected_faces = np.all(selected_vertices[cross_section_faces], axis=1).ravel()
        faces = cross_section_faces[selected_faces]
        if i%2 == 0:
            faces = faces[:,[0, 2, 1]]
        m = form_mesh(slabs.vertices, faces)
        m = remove_isolated_vertices(m)[0]
        cross_secs.append(m)

    return cross_secs

def slice_volume(mesh, direction, height):
    """ Slice a given 3D mesh from the bottom to a certain height.

    Args:
        mesh (:class:`Mesh`): The mesh to be sliced.
        direction (:class:`numpy.ndaray`): Direction orthogonal to the slices.
        height (int): height at which the mesh is sliced.

    Returns:
        A :class:`Mesh` object, representing the sliced volume.
    """
    if mesh.dim != 3:
        raise NotImplementedError("Only slicing 3D mesh is supported.")

    bbox_min, bbox_max = mesh.bbox
    center = 0.5 * (bbox_min + bbox_max)
    radius = norm(bbox_max - center)
    direction = np.array(direction)
    direction = direction / norm(direction)

    proj_len = np.dot(mesh.vertices, direction)
    min_val = np.amin(proj_len)
    max_val = np.amax(proj_len)
    mid_val = 0.5 * (min_val + max_val)

    min_corner = -np.ones(3) * (radius+1)
    max_corner =  np.ones(3) * (radius+1)
    max_corner[2] = height
    box = generate_box_mesh(min_corner, max_corner)

    rot = Quaternion.fromData(
            np.array([0.0, 0.0, 1.0]), np.array(direction)).to_matrix()
    box = form_mesh(np.dot(rot, box.vertices.T).T + center, box.faces)
    
    box_ver = box.vertices
    box_face = box.faces

    box_vertices = []

    max_ver = 0

    for i in range(len(box_ver)):
        if box_ver[i][2] > max_ver:
            max_ver = box_ver[i][2]

    for i in range(len(box_ver)):
        vertice = box_ver[i].copy()
        if vertice[2] == max_ver:
            vertice[2] = height
        box_vertices.append(vertice)    

    box_bool = pymesh.form_mesh(box_vertices, box_face)

    slice = boolean(box_bool, mesh, "intersection")

    slice_back = []
    slice_back.append(slice)

    return slice_back

def string_to_array(string):
    """ Converts a string containing an array back to an array.

    Args:
        string (string): The string to be converted to array.

    Returns:
        The converted array.
    """
    array_value = ''
    array_counter = 0
    array = []

    for letter in string:
        if letter == '(':
            task = 0
        elif letter == ' ':
            task = 1
        elif letter == ',' or letter == ')':
            task = 2
        else:
            task = 3
        if task == 1:
            array_counter = array_counter + 1
            array_value = ''
        elif task == 3:
            array_value = array_value + letter
        elif task == 2:
            array.append(float(array_value))

    return array

def linemap_server(height, query, slice_mode):
    url = 'http://localhost:9090/select'
    myobj = {"query": query}

    x = requests.post(url, json = myobj)

    started = 0
    string_array = []
    full_string = ''
    for letter in x.text:
        if letter == '"' or letter == "'":
            if started != 1:
                started = 1
            else:
                started = 0
                string_array.append(full_string)
                full_string = ''
        else:
            if started == 1:
                    full_string = full_string + letter

    faces = []
    verts = []
    edges = []
    T_s = []

    for i in range(int(len(string_array)/4)):
        faces.append(string_to_array(string_array[4*i]))
        verts.append(string_to_array(string_array[4*i+1]))
        T_s.append(string_to_array(string_array[4*i+2]))
        edges.append(string_to_array(string_array[4*i+3]))

    grouped_faces = []
    grouped_verts = []

    for j in range(len(faces)):
        grouped_verts.append(np.array([[verts[j][i], verts[j][i + 1], verts[j][i + 2]] for i in range(0, len(verts[j]), 3)]))
        grouped_faces.append(np.array([[faces[j][i], faces[j][i + 1], faces[j][i + 2]] for i in range(0, len(faces[j]), 3)]))

    mesh = []

    for z in range(len(grouped_verts)):
        mesh.append(pymesh.form_mesh(grouped_verts[z], grouped_faces[z]))

    meshes = []

    for y in range(len(mesh)):
        if slice_mode == "volume":
            meshes.append(slice_volume(mesh[y], [0,0,1], height))
        elif slice_mode == "plane":
            meshes.append(slice_height(mesh[y], [0,0,1], height))
        else:
            raise NotImplementedError("No slice mode selected.")

    vs = []
    fs = []

    for mesh_object in meshes:
        vs.append(mesh_object[0].vertices)
        fs.append(mesh_object[0].faces)

    for n in range(len(vs)):
        for i, f in enumerate(fs[n]):
            p = []
            for idx in f:
                hom_coord = [vs[n][idx][0],vs[n][idx][1],0,1]
                T_coor = np.reshape(T_s[n], (4,3)).T
                hom_coord_map = np.matmul(T_coor, hom_coord)
                p.append((hom_coord_map[0], hom_coord_map[1]))


            
            pplot = Polygon(p)
            
            x,y = pplot.exterior.xy 

            xmap = []
            ymap = []
            
            plt.plot(x,y, color='black')  

    plt.margins(0,0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())


    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    plt.axis('off')
    path = os.path.split(__file__)[0]
    
    if slice_mode == "volume":
        plt.savefig(os.path.join(path, r"maps/generated_map_navigation.png"), bbox_inches='tight', pad_inches=0, transparent=True)
    if slice_mode == "plane":
        plt.savefig(os.path.join(path, r"maps/generated_localization_map.png"), bbox_inches='tight', pad_inches=0, transparent=True)
    plt.show()
    

    return