# Topological map node definition
string id
string name
string type
string ifc_guid
geometry_msgs/Point position
float64 min_z                # Only floor nodes
float64 max_z                # Only floor nodes
string[] floor               # Only room and transition nodes
string subtype               # Only transition nodes
navbim_msgs/BoundingBox bbox # Only room and transition nodes
navbim_msgs/Polygon polygon  # Only room nodes - supports holes for columns, etc.
