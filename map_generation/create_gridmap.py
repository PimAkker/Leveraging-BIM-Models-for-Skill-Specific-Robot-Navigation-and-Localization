import ifcopenshell
import ifcopenshell.util.placement
import ifcopenshell.geom
from additional_functions import *
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import requests
import math
from matplotlib import colors

def check_grid(coordinate, g_size, map, origin):
    """ checks which grid the coordinate is in and makes the grid unpassable

    Args:
        coordinate (array): coordinate that needs to be checked
        g_size (float): size of the grids
        map (matrix): the gridmap
        origin (array): the origin point of the map

    Returns:
        map (matrix): the checked map
    """
    x_coor, y_coor = coordinate
    x_real = x_coor - origin[0]
    y_real = origin[1] - y_coor
    x_grid = int(x_real//g_size)
    y_grid = int(y_real//g_size)
    if y_grid >= map.shape[1]:
        y_grid = map.shape[1] - 1
    if x_grid >= map.shape[0]:
        x_grid = map.shape[0] - 1
    map[x_grid][y_grid] = 1

    return map

def plot_grid(map, fig_size):
    """ plots the gridmap

    Args:
        map (matrix): the gridmap
        fig_size (int): size of the plotted map

    Returns:
        nothing
    """
    cmap = colors.ListedColormap(['White','Black'])
    if fig_size != 'none':
        plt.figure(figsize=(fig_size,fig_size))
    plt.pcolor(map[::-1],cmap=cmap,edgecolors='k',linewidths=0)
    ax = plt.gca()
    ax.set_aspect('equal')

    return

def create_map(g_size, x, y):
    """ creates empty gridmap

    Args:
        g_size (float): size of the grids
        x (array): array of all the x coordinates
        y (array): array of all the y coordinates

    Returns:
        empty map (matrix)
    """
    max_x = max(x)
    max_y = max(y)
    min_x = min(x)
    min_y = min(y)

    origin = [min_x, max_y]
    dis_x = max_x - min_x
    grids_x = math.ceil(dis_x/g_size)
    dis_y = max_y - min_y
    grids_y = math.ceil(dis_y/g_size)

    map = np.zeros((grids_x, grids_y))

    return map, origin

def generate_gridmap(coordinates, x, y, g_size, fig_size):
    """ generates gridmap based on coordinates

    Args:
        coordinates (array): array of arrays containing point coordinates
        x (array): array of all the x coordinates
        y (array): array of all the y coordinates
        g_size (float): size of the grids
        fig_size (int): size of plot

    Returns:
        generated map (matrix)
    """
    map, origin = create_map(g_size, x, y)
    for coordinate in coordinates:
        map = check_grid(coordinate, g_size, map, origin)
    plot_grid(map.T, fig_size)

    return map

def gridmap_IFC(elements, height, ifc_name, slice_mode, gridsize=0.1, point_rate=0.01, fig_size='none'):
    """ Generate a gridmap from an IFC file.

    Args:
        elements (array): array of strings containing the elements you want to query from the IFC file (example: ['IFCWALL', 'IFCCOLUMN'])
        height (float): the height at which the elements will be sliced
        ifc_name (string): string pointing to location and file name of IFC
        slice_mode (string): Either 'plane' or 'volume', depending on which slice_function you want to use
        gridsize (float): size of the grids in the gridmap
        point_rate (float): distance between generated points
        fig_size (int): size of plot of generated map

    Returns:
        map (matrix): a matrix containing 1's and 0's that represents the gridmap
    """
    model = ifcopenshell.open(ifc_name) #opens IFC file at given location

    storeys = model.by_type('IFCBUILDINGSTOREY') #selects storeys from the IFC to check elevation
    loaded_elements = []
    elevations = []
    for storey in storeys: #queries requested elements from each storey and assigns it the correct elevation
        storey_elements = ifcopenshell.util.element.get_decomposition(storey)
        for element in storey_elements:
            if(element.is_a().upper() in elements):
                new_element = []
                new_element.append(element)
                loaded_elements.append(new_element)
                elevations.append(storey.Elevation)

    Tc = []
    total_elements = []

    for element in loaded_elements: #extracts transformation from all elements
        new_Tc, num_elements = extract_Tc(element)
        Tc.append(new_Tc)
        total_elements.append(num_elements)

    loaded_mesh = []
    for i in range(len(loaded_elements)): #extract sliced geometry from all elements
        loaded_mesh.append(extract_mesh(loaded_elements[i], height-elevations[i], slice_mode))

    if fig_size != 'none':
        plt.figure(figsize=(fig_size,fig_size))

    x_complete = []
    y_complete = []

    for z in range(len(loaded_mesh)): #for every element, use transformation to get coordinates, generate polygons and plot the map
        slice = []
        vs = []
        fs = []

        for j in range(len(loaded_elements[z])):
            slice.append(loaded_mesh[z][j][0])

            vs.append(slice[j].vertices)
            fs.append(slice[j].faces)

            x = []
            y = []

            polygons = []

            for i, f in enumerate(fs[j]): #apply transformation to get coordinates
                p = []
                for idx in f:
                    hom_coord = [vs[j][idx][0],vs[j][idx][1],0,1]
                    hom_coord_map = np.matmul(Tc[z][j],hom_coord)

                    p.append((hom_coord_map[0], hom_coord_map[1]))
                polygons.append(p) #create polgyons

            for polygon in polygons: #for each polygon, retrieve corner coordinates and and generate points between them
                poly = Polygon(polygon)
                x,y = poly.exterior.xy
                for i in range(len(x)-1):
                    x1 = x[i]
                    y1 = y[i]
                    x2 = x[i+1]
                    y2 = y[i+1]
                    x_complete.append(x1)
                    x_complete.append(x2)
                    y_complete.append(y1)
                    y_complete.append(y2)
                    dx = x2 - x1
                    dy = y2 - y1
                    change_x = point_rate*dx
                    change_y = point_rate*dy
                    currentx = x1 + change_x 
                    currenty = y1 + change_y
                    if x2 > x1 and abs(change_x) > 0.000000001:
                        while currentx < x2:
                            x_complete.append(currentx)
                            y_complete.append(currenty)
                            currentx = currentx + change_x 
                            currenty = currenty + change_y
                    elif x2 < x1 and abs(change_x) > 0.000000001:
                        while currentx > x2:
                            x_complete.append(currentx)
                            y_complete.append(currenty)
                            currentx = currentx + change_x 
                            currenty = currenty + change_y
                    elif y2 > y1 and abs(change_y) > 0.000000001:
                        while currenty < y2:
                            x_complete.append(currentx)
                            y_complete.append(currenty)
                            currentx = currentx + change_x 
                            currenty = currenty + change_y
                    elif y2 < y1 and abs(change_y) > 0.000000001:
                        while currenty > y2:
                            x_complete.append(currentx)
                            y_complete.append(currenty)
                            currentx = currentx + change_x 
                            currenty = currenty + change_y
   
    coordinates = []
    for i in range(len(x_complete)):
        coordinates.append([x_complete[i], y_complete[i]])
    
    map = generate_gridmap(coordinates, x_complete, y_complete, gridsize, fig_size) #use the generated points to make a gridmap

    return map

def gridmap_server(height, elements, slice_mode, point_rate=0.01, grid_size=0.1, fig_size='none'):
    """ Generate a pointmap from a running GraphDB server

    Args:
        height (float): the height at which the elements will be sliced
        elements (array): array of strings containing the elements you want to query from the IFC file (example: ['IFCWALL', 'IFCCOLUMN'])
        slice_mode (string): Either 'plane' or 'volume', depending on which slice_function you want to use
        point_rate (float): distance between generated points
        grid_size (float): size of the grids in the gridmap
        fig_size (int): size of plot of generated map

    Returns:
        map (matrix): a matrix containing 1's and 0's that represents the gridmap
    """
    url = 'http://localhost:9090/select'

    faces = []
    verts = []
    edges = []
    T_s = []

    for element in elements: #for each requested element type, send query to server for obtaining the elements
        query1 = "PREFIX props: <https://w3id.org/props#> SELECT ?faces ?verts ?T ?edges WHERE {?inst props:Category '"
        query2 = "' . ?inst props:Verts ?verts . ?inst props:Faces ?faces . ?inst props:T ?T . ?inst props:Edges ?edges}"
        query = query1 + element + query2
        myobj = {"query": query}

        x = requests.post(url, json = myobj) #send query

        started = 0 #query is obtained as one big string, this seperates all the objects in seperate strings
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

        faces_sub = []
        verts_sub = []
        edges_sub = []
        T_s_sub = []

        for i in range(int(len(string_array)/4)): #sort new strings into correct array, if element had "sub" geometries, these are held apart because they need extra processing
            if string_array[i][0] == '[':
                faces_sub.append(string_array[4*i])
                verts_sub.append(string_array[4*i+1])
                T_s_sub.append(string_array[4*i+2])
                edges_sub.append(string_array[4*i+3])
            else:
                faces.append(string_to_array(string_array[4*i]))
                verts.append(string_to_array(string_array[4*i+1]))
                T_s.append(string_to_array(string_array[4*i+2]))
                edges.append(string_to_array(string_array[4*i+3]))

        for face_sub in faces_sub: #following four for loops process the "sub" geometries
            started = 0
            face_string = '('
            for letter in face_sub:
                if letter == '(':
                    if started != 1:
                        started = 1
                elif letter == ')':
                        started = 0
                        face_string += ')'
                        faces.append(string_to_array(face_string))
                        face_string = '('
                else:
                    if started == 1:
                        face_string = face_string + letter

        for vert_sub in verts_sub:
            started = 0
            vert_string = '('
            for letter in vert_sub:
                if letter == '(':
                    if started != 1:
                        started = 1
                elif letter == ')':
                        started = 0
                        vert_string += ')'
                        verts.append(string_to_array(vert_string))
                        vert_string = '('
                else:
                    if started == 1:
                        vert_string = vert_string + letter

        for T_sub in T_s_sub:
            started = 0
            T_string = '('
            for letter in T_sub:
                if letter == '(':
                    if started != 1:
                        started = 1
                elif letter == ')':
                        started = 0
                        T_string += ')'
                        T_s.append(string_to_array(T_string))
                        T_string = '('
                else:
                    if started == 1:
                        T_string = T_string + letter
            
        for edge_sub in edges_sub:
            started = 0
            edge_string = '('
            for letter in edge_sub:
                if letter == '(':
                    if started != 1:
                        started = 1
                elif letter == ')':
                        started = 0
                        edge_string += ')'
                        edges.append(string_to_array(edge_string))
                        edge_string = '('
                else:
                    if started == 1:
                        edge_string = edge_string + letter

    grouped_faces = []
    grouped_verts = []

    for j in range(len(faces)): #rest from here is similar to Gridmap_IFC above
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

    x = []
    y = []

    x_complete = []
    y_complete = []

    for mesh_object in meshes:
        vs.append(mesh_object[0].vertices)
        fs.append(mesh_object[0].faces)

    polygons = []

    for n in range(len(vs)):
        for i, f in enumerate(fs[n]):
            p = []
            for idx in f:
                hom_coord = [vs[n][idx][0],vs[n][idx][1],0,1]
                T_coor = np.reshape(T_s[n], (4,3)).T
                hom_coord_map = np.matmul(T_coor,hom_coord)

                p.append((hom_coord_map[0], hom_coord_map[1]))
            polygons.append(p)

    for polygon in polygons:
        poly = Polygon(polygon)
        x,y = poly.exterior.xy
        for i in range(len(x)-1):
            x1 = x[i]
            y1 = y[i]
            x2 = x[i+1]
            y2 = y[i+1]
            x_complete.append(x1)
            x_complete.append(x2)
            y_complete.append(y1)
            y_complete.append(y2)
            dx = x2 - x1
            dy = y2 - y1
            change_x = point_rate*dx
            change_y = point_rate*dy
            currentx = x1 + change_x 
            currenty = y1 + change_y
            if x2 > x1 and abs(change_x) > 0.000000001:
                while currentx < x2:
                    x_complete.append(currentx)
                    y_complete.append(currenty)
                    currentx = currentx + change_x 
                    currenty = currenty + change_y
            elif x2 < x1 and abs(change_x) > 0.000000001:
                while currentx > x2:
                    x_complete.append(currentx)
                    y_complete.append(currenty)
                    currentx = currentx + change_x 
                    currenty = currenty + change_y
            elif y2 > y1 and abs(change_y) > 0.000000001:
                while currenty < y2:
                    x_complete.append(currentx)
                    y_complete.append(currenty)
                    currentx = currentx + change_x 
                    currenty = currenty + change_y
            elif y2 < y1 and abs(change_y) > 0.000000001:
                while currenty > y2:
                    x_complete.append(currentx)
                    y_complete.append(currenty)
                    currentx = currentx + change_x 
                    currenty = currenty + change_y

    coordinates = []
    for i in range(len(x_complete)):
        coordinates.append([x_complete[i], y_complete[i]])
    
    map = generate_gridmap(coordinates, x_complete, y_complete, grid_size, fig_size)

    return map