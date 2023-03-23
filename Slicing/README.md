# Here is the code for slicing

### Make the directory 'Data' inside the Slicing directory.

## slice_mesh2

In the file newslice.py is a modified function from the pymesh. The modified function does the slicing of the mesh on the specified height.

## extract_mesh

In get_geometry.py file is a function for extracting a mesh. It gets the element that should be sliced and the height of the slice. The function generate a mesh of the element and slice it on the specified height.

## extract_Tc

This function generates the matrix that represents the location and rotation of the provided element.

## generate_pointcloud

Gets the list of elements (strings that could be queried by type using ifcopenshell library) and slices the mesh on specific height.

## save_pointcloud

Do the same as generate_pointcloud function, but it also saves the mesh into the csv file and saves the image.

## get_height

Extracts the height of the lidar sensor from the xacro (urdf) file. As a parameter there should be the path to the xacro file.
