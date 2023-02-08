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
### how to represent 2D and 3D geometry in a knowledge graph
The resulting RDF graph or knowledge graph contains lots of semantic data. It needs to be enriched with the geometry at specific heights (z-levels) so that the robot can use this data in its 2D navigation and localization maps / plans.

An indication of how geometry can be stored in a knowledge graph is available in the following examples:
- de Koning, R., Torta, E., Pauwels, P., Hendrikx, R. W. M., & van de Molengraft, M. J. G. (2021). Queries on Semantic Building Digital Twins for Robot Navigation. In 9th Linked Data in Architecture and Construction Workshop (pp. 32-42). (CEUR Workshop Proceedings; Vol. 3081). CEUR-WS.org. http://ceur-ws.org/Vol-3081/03paper.pdf

### challenge
Obtaining this geometry can be achieved in a number of ways and using a number of tools. This is part of your challenge. The resulting representation needs to become available in / via the knowledge graph.

## Creation of robot-specific navigation and localization maps
From the IFC and knowledge graphs, navigation and localization maps can be built. For reference, you can look into the below material:

- de Koning, R., Torta, E., Pauwels, P., Hendrikx, R. W. M., & van de Molengraft, M. J. G. (2021). Queries on Semantic Building Digital Twins for Robot Navigation. In 9th Linked Data in Architecture and Construction Workshop (pp. 32-42). (CEUR Workshop Proceedings; Vol. 3081). CEUR-WS.org. http://ceur-ws.org/Vol-3081/03paper.pdf
- Hendrikx, B., Pauwels, P., Torta, E., Bruyninckx, H. P. J., & van de Molengraft, M. J. G. (2021). Connecting Semantic Building Information Models and Robotics: An application to 2D LiDAR-based localization. In 2021 IEEE International Conference on Robotics and Automation, ICRA 2021 (pp. 11654-11660). [9561129] Institute of Electrical and Electronics Engineers. https://doi.org/10.1109/ICRA48506.2021.9561129
- 

## Enable navigation and task execution (validation)


## Update the BIM model based on metrics obtained by robot



# Support
Are you stuck, then get help with p.pauwels@tue.nl or e.torta@tue.nl.

# Contributing
Contributions are restricted to students of the AI&ES Master program that take on this challenge.

# License
This code needs to stay private.
