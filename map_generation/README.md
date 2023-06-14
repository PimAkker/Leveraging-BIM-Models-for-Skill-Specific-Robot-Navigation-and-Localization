# Map Generation

## Dependencies
The scripts contained here are dependent on a couple of python libraries that need to be installed first. These include;
- PyMesh (tested on v0.3)
- IfcOpenShell (tested on v0.7.0.230218)
- Shapely (tested on v2.0.1)

Be aware that PyMesh cannot be installed through pip but needs to be installed manually from https://pymesh.readthedocs.io/en/latest/. Furthermore, be aware that the functions to plot maps are also dependent on some functions that are contained in the additional_functions.py

## Introduction
The map generation scripts that are in this folder contain functions that can be independently used to generate different kind of maps either directly from an IFC file or from a graph database on a server. The database/server is explained in the server folder on this Gitlab. For a short overview, this folder contains the following python files;
- additional_functions.py: a python file with multiple functions that the map generation functions use. This file always needs to be downloaded together with one of the map generation scripts and be present within the same folder.
    - slice_height: a function that slices a given 3D mesh at a certain height. This function is based on the slice_mesh function from the PyMesh library.
    - slice_volume: a function that slices a given 3D mesh from the bottom of the mesh up to a certain height. This function is based on the slice_mesh function from the PyMesh library.
    - string_to_array: a function that converts a string containing an array back to an array.
    - extract_mesh: a function that extracts geometry from IFC elements, creates meshes from the geometry and slices them at the correct height.
    - extract_Tc: a function that extracts the transformations from IFC elements.
- create_linemap: a python file that contains the linemap_IFC function, which generates a linemap from an IFC file and the linemap_server functions, which generates a linemap from a graph database server.
    - linemap_IFC: A function that generates a linemap directly from an IFC.
    - linemap_server: A function that generates a linemap directly from a graph database stored on a server.
- create_pointmap: a python file that contains the pointmap_IFC function, which generates a pointmap from an IFC file and the pointmap_server functions, which generates a pointmap from a graph database server.
    - pointmap_IFC: A function that generates a pointmap directly from an IFC.
    - pointmap_server: A function that generates a pointmap directly from a graph database stored on a server.
- create_gridmap: a python file that contains the gridmap_IFC function, which generates a gridmap from an IFC file and the gridmap_server functions, which generates a gridmap from a graph database server.
    - check_grid: a function that can fill an empty gridmap using coordinates.
    - plot_grid: a function that plots a gridmap.
    - create_map: a function that creates an empty gridmap.
    - generate_gridmap: a function that generates a gridmap from a list of points.
    - gridmap_IFC: A function that generates a gridmap directly from an IFC.
    - gridmap_server: A function that generates a gridmap directly from a graph database stored on a server.

## How to use
When you want to use the functions in these scripts, download them and import them to your workspace. Be aware that using the server functions require the server with the graph database to be running. 

An example of how to use the ...map_IFC function:

linemap_IFC(['IFCCOLUMN', 'IFCWALLSTANDARDCASE', 'IFCSTAIR', 'IFCSTAIRFLIGHT', 'IFCCURTAINWALL', 'IFCPLATE', 'IFCDOOR', 'IFCWALL', 'IFCMEMBER'], 1, './simple_room_v5.ifc', 'volume')

An example of how to use the ...map_server function:

linemap_server(0.2, ['Stairs', 'Walls', 'Structural Columns'], "plane")

## How it works
The map generation process is described below. Be aware that the code sections shown below are not the same as in the scripts, but rather show how separate sections of the scripts work by showing small bits that are representative of what the scripts are doing.

-  Query elements: First the required elements are queries from either the IFC directly or from the graph database on the server. For the IFC, this is done using IfcOpenShell to retrieve the wanted elements and generate the shape of each of the individual elements. These shapes can then be used to retrieve the transformation, faces, edges and vertices of the elements. For the server, these are obtained using a SparQL query which retrieves the transformation, faces, edges and vertices of the elements directly.

For IFC:
```
#First all elements are obtained from the IFC per storey and checked if they are present in the element types queried by the user. This also allows for storing the relative elevation of the storey per element for determining the proper slicing height.

queried_elements = input by user
model = ifcopenshell.open(IFC file)
storeys = model.by_type('IFCBUILDINGSTOREY')
for storey in storeys:
    storey_elements = ifcopenshell.util.element.get_decomposition(storey)
    for element in storey_elements:
        if element is in queried_elements:
            add element to loaded_elements
            add storey.Elevation to elevations

#Next, the geometry data is obtained from each of the loaded elements. Some elements, like stairs, consist of multiple "sub" geometries. This means that these elements should first be subdivided.

for element in loaded_elements:
    if element has geometry:
        shape = ifcopenshell.geom.create_shape(element)
        add shape.geometry.faces to faces
        add shape.geometry.verts to verts
        add shape.transformation.matrix.data to transformations
    else:
        sub_elements = ifcopenshell.util.element.get_decomposition(element)
        for sub_element in sub_elements:
            shape = ifcopenshell.geom.create_shape(sub_element)
            add shape.geometry.faces to faces
            add shape.geometry.verts to verts
            add shape.transformation.matrix.data to transformations
```

For server:
```
#The geometry data is stored in terms of transformation, faces, edges and verts on the server already for each element. This means that for each element type that needs to be queried the geometry needs to be obtained from the server. Note that edges are not used for constructing the meshes later on. Even though faces and vertices are enough to reconstruct a mesh, the edges are stored on the server to have the complete geometry available.

queried_elements = input by user
for element in queried_elements:
    data = requests.post(url, json = query)
    add faces in data to faces
    add transformations in data to transformations
    add verts in data to verts
```

- Generate meshes: Having obtained the needed geometry of the individual elements, the faces and vertices are used to generate meshes. These meshes are an object type native to the PyMesh library.

```
#The faces and vertices are stored per element in one long array, even though they belong in groups of three. For this reason, they first need to be grouped before they can be used to generate meshes.

for j in range(amount of elements):
    grouped_verts = np.array([[verts[j][i], verts[j][i + 1], verts[j][i + 2]] for i in range(0, len(verts[j]), 3)])
    grouped_faces = np.array([[faces[j][i], faces[j][i + 1], faces[j][i + 2]] for i in range(0, len(faces[j]), 3)])
    element_mesh = pymesh.form_mesh(grouped_verts, grouped_faces)
    add element_mesh to meshes
```

- Slice meshes: Now that meshes of all queried elements are obtained, they are sliced using one of the two previously mentioned slice functions. If a localization map is needed, they are sliced using the slice_height function and if a navigation map is needed they are sliced using the slice_volume function. This will return the sliced mesh.

```
#The relative storey elevation is extracted from the inputed slice height here to make up for an element being on a different storey.

slice_mode = input by user ('plane' or 'volume')
height = input by user
for j in range(amount of elements):
    if slice_mode == "volume":
        slice = slice_volume(meshes[j], [0,0,1], height - elevation[j]))
        add slice to slices
    elif slice_mode == "plane":
        slice = slice_height(meshes[j], [0,0,1], height - elevation[j]))
        add slice to slices
```

- Generate polygons: The geometry stored in the sliced meshes are now converted into polygons using the previously obtained transformations. These polygons are a data type native to the Shapely python library. These are the polygons that together formed the meshes previously. Because the transformation is used in this process, these polygons also contain the necessary location information for plotting them into a map.

```
#First the new vertices and faces are obtained from the slices

for slice in slices:
    verts_new = slice.vertices
    add verts_new to sliced_verts
    faces_new = slice.faces
    add faces_new to sliced_faces

#Secondly, the previously obtained transformations are applied on these slices vertices and faces to obtain polygons with location information

for j in range(amount of elements):
    for i, face in enumerate(slices_faces[j]):
        for x in face:
            coordinates = [sliced_verts[j][x][0],sliced_verts[j][x][1],0,1]
            coordinates_map = np.matmul(transformations[j], coordinates)
            polygon_coordinate = (coordinates_map[0], coordinates_map[1])
            add polygon_coordinate to polygon_coordinates
        polygon = Polygon(polygon_coordinates)
        add polygon to polygons
```

- Obtain coordinates: Using the polygons that are obtained, x and y coordinates can be extracted from them. These coordinates describe the corner points of the polygons. the z coordinate is ignored, since the map needs to be flat. This automatically flattens the 3D volume mesh obtained from using the slice_volume function.

```
for polygon in polygons:
    x, y = poly.exterior.xy
    add x to all_x
    add y to all_y
```

- Plotting map: The x and y coordinates of the polygons are now used to plot the map. For the line map a simple plot is used that automatically draws lines between these points. For the point map, a script is used that generates coordinates for points between the polygon corners. The rate of points can be adjusted. These are then plotted as a scatter plot to show the map. For the grid map, the coordinates obtained by the point generating script are used to see if a grid in the grid map is passable. First, a general grid is generated. After this, each grid is checked with the coordinate points established, and if there exist a coordinate inside the given grid it becomes obstructed.

For linemap:
```
for x, y in all_x, all_y:
    plt.plot(x,y)
```

For pointmap:
```
#New points between two coordinates are calculated by using the vector from point 1 to point 2.

point_rate = input by user
for x, y in all_x, all_y:
    for i in range(len(x)-1):
        x1 = x[i]
        y1 = y[i]
        x2 = x[i+1]
        y2 = y[i+1]
        add x1, x2 to generated_x
        add y1, y2 to generated_y
        dx = (x2 - x1)*point_rate
        dy = (y2 - y1)*point_rate
        next_x = x1 + dx
        next_y = y1 + dy
        while next_x and next_y are within x2 and y2:
            add next_x to generated_x
            add next_y to generated_y
            next_x = x1 + dx
            next_y = y1 + dy
plt.scatter(generated_x, generated_y)
```

For gridmap:
```
#First, points are generated just like in the pointmap, but instead of the scatter plot the algorithm uses these points to generate the gridmap. To do this, first an empty gridmap is initialized.

grid_size = input by user
max_x = max(generated_x)
max_y = max(generated_y)
min_x = min(generated_x)
min_y = min(generated_y)
origin = [min_x, max_y]
dis_x = max_x - min_x
grids_x = math.ceil(dis_x/grid_size)
dis_y = max_y - min_y
grids_y = math.ceil(dis_y/grid_size)
map = np.zeros((grids_x, grids_y))

#Then, every coordinate in the generated points is placed in a grid. If at least one coordinate is present in a grid, it is deemed as not passable.

coordinates = [generated x, generated y]
for coordinate in coordinates:
    x_coor, y_coor = coordinate
    x_real = x_coor - origin[0]
    y_real = origin[1] - y_coor
    x_grid = int(x_real//g_size)
    y_grid = int(y_real//g_size)
    map[x_grid][y_grid] = 1
plt.pcolor(map[::-1])
```

## Shortcomings
Currently, generating maps from the server do not take the storey height into account. However, the storey elevation is stored in the database on the server. If this is needed to be implemented, one could query this elevation and check which elements belong to which storey. Then, the elevation can be subtracted from the slice height to correctly handle storeys.