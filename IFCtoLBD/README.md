# IFCtoLBD

The basis for this code is taken from https://github.com/ISBE-TUe/IFCtoLBD

To use this code, you have to use the following software https://graphdb.ontotext.com/

## Dependencies

This script is dependent on IfcOpenShell, which is a python library that should be installed before use.

## How to use

This version of the IFCtoLBD script also initializes the database with the geometry used to generate the localization and navigation maps. Most of the code is the same as the basis, except for a new part in the WriteElements section. Here, the geometry is extracted from the IFC using IfcOpenShell and put in the string of the ttl.

An example of how to use the function:
