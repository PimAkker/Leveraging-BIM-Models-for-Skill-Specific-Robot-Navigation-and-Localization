# this file resets the generated_map.png from the database


from server_functions import *

linemap_server(0.3, "PREFIX props: <https://w3id.org/props#> SELECT ?faces ?verts ?T ?edges WHERE {?inst props:Reference 'Generic-wall-for-test' . ?inst props:Verts ?verts . ?inst props:Faces ?faces . ?inst props:T ?T . ?inst props:Edges ?edges}", "plane")
linemap_server(0.6, "PREFIX props: <https://w3id.org/props#> SELECT ?faces ?verts ?T ?edges WHERE {?inst props:Reference 'Generic-wall-for-test' . ?inst props:Verts ?verts . ?inst props:Faces ?faces . ?inst props:T ?T . ?inst props:Edges ?edges}", "volume")