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
- create_linemap: a python file that contains the linemap_IFC function, which generates a linemap from an IFC file and the linemap_server functions, which generates a linemap from a graph database server.
