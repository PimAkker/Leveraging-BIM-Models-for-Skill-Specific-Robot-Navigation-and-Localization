import ifcopenshell
import ifcopenshell.util.placement
import ifcopenshell.geom
from additional_functions import *
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import requests

def pointmap_IFC(elements, height, ifc_name, slice_mode, point_rate=0.01, fig_size='none'):
    """ Generate a pointfrom an IFC file.

    Args:
        elements (array): array of strings containing the elements you want to query from the IFC file (example: ['IFCWALL', 'IFCCOLUMN'])
        height (float): the height at which the elements will be sliced
        ifc_name (string): string pointing to location and file name of IFC
        slice_mode (string): Either 'plane' or 'volume', depending on which slice_function you want to use
        point_rate (float): distance between generated points
        fig_size (int): size of plot of generated map

    Returns:
        Nothing, but saves map as png and plots the map.
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

            for polygon in polygons: #for each polygon, retrieve corner coordinates and generates points between them
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
   
    plt.scatter(x_complete,y_complete,s=0.5,c='#000000') #plot all the points

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box') 

    plt.savefig('maplarge')

    return

def pointmap_server(height, elements, slice_mode, point_rate=0.01):
    """ Generate a pointmap from a running GraphDB server

    Args:
        height (float): the height at which the elements will be sliced
        elements (array): array of strings containing the elements you want to query from the IFC file (example: ['IFCWALL', 'IFCCOLUMN'])
        slice_mode (string): Either 'plane' or 'volume', depending on which slice_function you want to use
        point_rate (float): distance between generated points

    Returns:
        Nothing, but plots the map.
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

    for j in range(len(faces)): #rest from here is similar to Pointmap_IFC above
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
   
    plt.scatter(x_complete,y_complete,s=0.5,c='#000000') 

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')

    return