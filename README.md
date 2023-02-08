# 2022-SEMB-Q3Q4: AIES Challenge: Integrated Building Information Models and Robotics



## Description

Detailed 3D Building Information Models are available in Revit. Standard practice in the building industry makes such BIM models available. While Revit is a proprietary (closed) commercial BIM tool, a neutral data model and file format is available as well, namely the Industry Foundation Classes (IFC). Such information needs to be made available to robots to ease their localisation and navigation. That is your challenge. This is tested in a barn environment, where agricultural robots need to be able to navigate in their environment and perform actions with cattle present inside the barn.

The challenge includes a number of steps, as detailed below.

# Project assignment
## Knowledge graph generator for BIM models
### what is a BIM model
3D BIM models are available. They have different shapes and levels of detail. They are commonly built in [Autodesk Revit](https://www.autodesk.eu/products/revit/architecture) (free version for students). The Revit models are typically exported to software-neutral IFC files. Sample RVT and IFC models are available for a Barn model here in the repository:

- RVT: [file](BIM%20models/Barn)
- IFC: [file](BIM%20models/Barn)

The IFC file can be generated anew from Revit also. IFC files can be viewed in an open viewer: https://view.ifcopenshell.org/.

### knowledge graph generation
These IFC files can be transformed into knowledge graphs (RDF graphs in TTL format) using an open IFCtoLBD Python converter that is available here:
https://github.com/ISBE-TUe/IFCtoLBD. This leads to a graph (TTL file) that can be loaded into a graph databases, such as [OntoText GraphDB](https://graphdb.ontotext.com/).

While you work on the Barn model first and foremost, similar sample data (RVT, IFC, TTL) is also available here for the Atlas building:
https://github.com/ISBE-TUe/atlas-building-graph

### challenge
While a lot of sample algorithms are available, it is your challenge to bring these together in your solution, and extend these software so that they respond to the data needs for robot navigation and localization (see next steps).

## High quality digital twin that includes good 2D and 3D geometry


## Creation of robot-specific navigation and localization maps


## Enable navigation and task execution (validation)


## Update the BIM model based on metrics obtained by robot



# Support
Are you stuck, then get help with p.pauwels@tue.nl or e.torta@tue.nl.

# Contributing
Contributions are restricted to students of the AI&ES Master program that take on this challenge.

# License
This code needs to stay private.
