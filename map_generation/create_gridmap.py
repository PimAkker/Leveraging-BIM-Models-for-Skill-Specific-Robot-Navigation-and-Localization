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
    x_coor, y_coor = coordinate
    x_real = x_coor - origin[0]
    y_real = origin[1] - y_coor
    x_grid = int(x_real//g_size)
    y_grid = int(y_real//g_size)
    map[x_grid][y_grid] = 1

    return map

def plot_grid(map):
    cmap = colors.ListedColormap(['White','Black'])
    plt.figure(figsize=(10,10))
    plt.pcolor(map[::-1],cmap=cmap,edgecolors='k',linewidths=0)

    ax = plt.gca()
    ax.set_aspect('equal')

    return

def create_map(g_size, x, y):
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

def generate_gridmap(coordinates, x, y, g_size):
    map, origin = create_map(g_size, x, y)
    for coordinate in coordinates:
        map = check_grid(coordinate, g_size, map, origin)
    plot_grid(map.T)

    return map

def gridmap_IFC(elements, height, ifc_name, slice_mode, gridsize=0.1, point_rate=0.1, fig_size='none'):
    model = ifcopenshell.open(ifc_name)

    storeys = model.by_type('IFCBUILDINGSTOREY')
    loaded_elements = []
    elevations = []
    for storey in storeys:
        storey_elements = ifcopenshell.util.element.get_decomposition(storey)
        for element in storey_elements:
            if(element.is_a().upper() in elements):
                new_element = []
                new_element.append(element)
                loaded_elements.append(new_element)
                elevations.append(storey.Elevation)

    Tc = []
    total_elements = []

    for element in loaded_elements:
        new_Tc, num_elements = extract_Tc(element)
        Tc.append(new_Tc)
        total_elements.append(num_elements)

    loaded_mesh = []
    for i in range(len(loaded_elements)):
        loaded_mesh.append(extract_mesh(loaded_elements[i], height-elevations[i], slice_mode))

    if fig_size != 'none':
        plt.figure(figsize=(fig_size,fig_size))

    x_complete = []
    y_complete = []

    for z in range(len(loaded_mesh)):
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

            for i, f in enumerate(fs[j]):
                p = []
                for idx in f:
                    hom_coord = [vs[j][idx][0],vs[j][idx][1],0,1]
                    hom_coord_map = np.matmul(Tc[z][j],hom_coord)

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
    
    map = generate_gridmap(coordinates, x_complete, y_complete, gridsize)

    return map

def gridmap_server(height, query, slice_mode):
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
                hom_coord_map = np.matmul(T_coor,hom_coord)
                p.append((hom_coord_map[0], hom_coord_map[1]))
                print(T_coor)
                print(hom_coord)
                print(hom_coord_map)
            pplot = Polygon(p)
            x,y = pplot.exterior.xy 

            plt.plot(x,y, color='black')  

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')

    return