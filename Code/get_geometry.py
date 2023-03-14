import ifcopenshell
import ifcopenshell.util.placement
import numpy as np
import matplotlib.pyplot as plt
import ifcopenshell.geom
import pymesh
from shapely.geometry import Polygon
import geopandas as gpd
from newslice import *

def extract_mesh(element, height):
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
        meshes.append(slice_mesh2(mesh[j], [0,0,1], height))

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


def plot_elements(elements, height):
    #Plots elements from list of strings at a given height.
    model = ifcopenshell.open('./atlas_8_floor.ifc')

    loaded_elements = []
    for element in elements:
        loaded_elements.append(model.by_type(element))

    Tc = []
    for element in loaded_elements:
        Tc.append(extract_Tc(element))  

    loaded_mesh = []
    for element in loaded_elements:
        loaded_mesh.append(extract_mesh(element, height))

    for z in range(len(loaded_mesh)):
        slice = []
        vs = []
        fs = []

        for j in range(len(loaded_elements[z])):
            slice.append(loaded_mesh[z][j][0])

            vs.append(slice[j].vertices)
            fs.append(slice[j].faces)

            for i, f in enumerate(fs[j]):
                p = []
                for idx in f:
                    hom_coord = [vs[j][idx][0],vs[j][idx][1],0,1]
                    hom_coord_map = Tc[z][j]@hom_coord

                    p.append((hom_coord_map[0], hom_coord_map[1]))

                pplot = Polygon(p)
                x,y = pplot.exterior.xy 

                xmap = []
                ymap = []

                plt.plot(x,y )  

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box') 