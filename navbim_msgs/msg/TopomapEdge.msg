# Topological map edge definition
string source
string target
string id
string type
string subtype
# Following attributes only for transition edges
string room_id
float64 estimated_distance
float64 planned_distance
float64 estimated_cost
float64 planned_cost
nav_msgs/Path path