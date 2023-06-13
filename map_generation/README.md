# Map Generation

## Dependencies
The scripts contained here are dependent on a couple of python libaries that need to be installed first. These include;
- PyMesh (tested on v0.3)
- IfcOpenShell (tested on v0.7.0.230218)
- Shapely (tested on v2.0.1)
Be aware that PyMesh cannot be installed through pip but needs to be installed manualy from https://pymesh.readthedocs.io/en/latest/. Furthermore, be aware that the functions to plot maps are also dependent on some functions that are contained in the additional_functions.py

## Introduction
The map generation scripts that are in this folder contain functions that can be independently used to generate different kind of maps either directly from an IFC file or from a graph database on a server. The database/server is explained in the server folder on this Gitlab. For a short overview, this folder contains the following python files;
- additional_functions.py: a python file with multiple functions that the map generation functions use. This file always needs to be downloaded together with one of the map generation scripts and be present within the same folder.
    - slice_height: a function that slices a given 3D mesh at a certain height. Inputs; mesh, direction, height. Output; a single slice (plane) of the mesh at a given height. This function is based on the slice_mesh function from the PyMesh library.
    - slice_volume: a function that slices a given 3D mesh from the bottom of the mesh up to a certain height. Inputs; mesh, direction, height. Output; the slices volume. This function is based on the slice_mesh function from the PyMesh library.
    - string_to_array:
    - extract_mesh:
    - extract_Tc:
- create_linemap: a python file that contains the linemap_IFC function, which generates a linemap from an IFC file and the linemap_server functions, which generates a linemap from a graph database server.
    - linemap_IFC:
    - linemap_server:
- create_pointmap: a python file that contains the pointmap_IFC function, which generates a pointmap from an IFC file and the pointmap_server functions, which generates a pointmap from a graph database server.
    - pointmap_IFC:
    - pointmap_server:
- create_gridmap: a python file that contains the gridmap_IFC function, which generates a gridmap from an IFC file and the gridmap_server functions, which generates a gridmap from a graph database server.
    - check_grid:
    - plot_grid:
    - create_map:
    - generate_gridmap:
    - gridmap_IFC:
    - gridmap_server:

## How to use


## How it works


## Shortcomings
