from pymesh.boolean import boolean
from pymesh.misc import Quaternion
from pymesh.meshutils import generate_box_mesh, separate_mesh, remove_isolated_vertices, merge_meshes
from pymesh.meshio import form_mesh, save_mesh
import pymesh
import numpy as np
from numpy.linalg import norm
# import ifcopenshell
# import ifcopenshell.util.placement
# import ifcopenshell.geom

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

def extract_mesh(element, height, slice_mode):
    settings = ifcopenshell.geom.settings()

    shape = []
    faces = []
    edges = []
    verts = []

    for i in range(len(element)):
        shape.append(ifcopenshell.geom.create_shape(settings, element[i]))

        faces.append(shape[i].geometry.faces)
        edges.append(shape[i].geometry.edges)
        verts.append(shape[i].geometry.verts)

    grouped_verts = []
    grouped_faces = []
    mesh = []

    for j in range(len(element)):
        grouped_verts.append(np.array([[verts[j][i], verts[j][i + 1], verts[j][i + 2]] for i in range(0, len(verts[j]), 3)]))
        grouped_faces.append(np.array([[faces[j][i], faces[j][i + 1], faces[j][i + 2]] for i in range(0, len(faces[j]), 3)]))

        mesh.append(pymesh.form_mesh(grouped_verts[j], grouped_faces[j]))
    meshes = []

    for j in range(len(element)):
        if slice_mode == "volume":
            meshes.append(slice_volume(mesh[j], [0,0,1], height))
        elif slice_mode == "plane":
            meshes.append(slice_height(mesh[j], [0,0,1], height))
        else:
            raise NotImplementedError("No slice mode selected.")

    return meshes

def extract_Tc(element):
    settings = ifcopenshell.geom.settings()

    shape = []
    Tc = []

    for i in range(len(element)):
        shape.append(ifcopenshell.geom.create_shape(settings, element[i]))
        T_col = shape[i].transformation.matrix.data
        Tc.append(np.reshape(T_col, (4,3)).T)

    return Tc