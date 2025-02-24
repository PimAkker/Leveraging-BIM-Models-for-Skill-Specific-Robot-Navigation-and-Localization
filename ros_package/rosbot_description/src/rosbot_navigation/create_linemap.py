# import ifcopenshell
# import ifcopenshell.util.placement
# import ifcopenshell.geom
#These functions can be outdated since integration into the ROS environment, please go to the map generation folder for the newest versions
from additional_functions import *
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import requests

import os 

def linemap_IFC(elements, height, ifc_name, slice_mode):
    model = ifcopenshell.open(ifc_name)

    loaded_elements = []
    for element in elements:
        loaded_elements.append(model.by_type(element))

    Tc = []
    for element in loaded_elements:
        Tc.append(extract_Tc(element))  

    loaded_mesh = []
    for element in loaded_elements:
        loaded_mesh.append(extract_mesh(element, height, slice_mode))

    for z in range(len(loaded_mesh)):
        slice = []
        vs = []
        fs = []

        for j in range(len(loaded_elements[z])):
            slice.append(loaded_mesh[z][j][0])

            vs.append(slice[j].vertices)
            fs.append(slice[j].faces)

            polygons = []

            for i, f in enumerate(fs[j]):
                p = []
                for idx in f:
                    hom_coord = [vs[j][idx][0],vs[j][idx][1],0,1]
                    hom_coord_map = np.multiply(Tc[z][j], hom_coord)
                    p.append((hom_coord_map[0], hom_coord_map[1]))
                polygons.append(p)

            for polygon in polygons:
                poly = Polygon(polygon)
                x,y = poly.exterior.xy
                plt.plot(x,y, color='black')

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box') 

    plt.savefig('maplarge')

    return

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
    # pdb.set_trace()
    path = os.path.split(__file__)[0]
    plt.savefig(os.path.join(path, r"maps/generated_map.png"), bbox_inches='tight', pad_inches=0, transparent=True)
    plt.show()
    

    return
